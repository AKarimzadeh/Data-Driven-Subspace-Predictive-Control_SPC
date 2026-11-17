%% Main Script: Data-Driven Subspace Predictive Control (SPC) for AVR System
% This implementation serves as a comparative baseline for the method 
% presented in the following paper:
%
% Karimzadeh, A., Mousavi, S. A., & Asemani, M. H. (2026).
% Robust data-driven control of Automatic Voltage Regulators using DeePC:
% Regularization techniques under noise and uncertainty.
% International Journal of Electrical Power & Energy Systems, 178, 111973.
%
% If you use this code or build upon it, please cite the paper above.

clear; clc; close all;

%% 1. System Setup

param_ranges.Ka = [10, 12];     param_ranges.Ta = [0.02, 0.1];
param_ranges.Ke = [1, 5];       param_ranges.Te = [0.4, 1.0];
param_ranges.Kg = [0.7, 1.0];   param_ranges.Tg = [1.0, 2.0];
param_ranges.Ks = [0.9, 1.1];   param_ranges.Tsensor = [0.001, 0.06];

p_nom.Ka = mean(param_ranges.Ka); p_nom.Ta = mean(param_ranges.Ta);
p_nom.Ke = mean(param_ranges.Ke); p_nom.Te = mean(param_ranges.Te);
p_nom.Kg = mean(param_ranges.Kg); p_nom.Tg = mean(param_ranges.Tg);
p_nom.Ks = mean(param_ranges.Ks); p_nom.Tsensor = mean(param_ranges.Tsensor);

s = tf('s');
Plant_tf = (p_nom.Ka/(1+p_nom.Ta*s)) * (p_nom.Ke/(1+p_nom.Te*s)) * (p_nom.Kg/(1+p_nom.Tg*s));
Sensor_tf = p_nom.Ks/(1+p_nom.Tsensor*s);
AVR_OpenLoop = Plant_tf * Sensor_tf;

Ts = 0.009;
AVR_System_disc = c2d(AVR_OpenLoop, Ts);
[A, B, C, D] = ssdata(AVR_System_disc);
[n, m, p] = deal(size(A,1), size(B,2), size(C,1));

fprintf('System discretized (Ts = %.3f s). States: %d | Inputs: %d | Outputs: %d\n', Ts, n, m, p);

%% 2. Data Collection & Predictor Identification

T_ini     = 10;      % Past horizon
T_future  = 40;      % Prediction horizon
T_data    = 120;     % Length of identification data

rng(12);
u_data = (rand(T_data, m) - 0.5) * 2;
y_data = dlsim(A, B, C, D, u_data);

[SPC_Predictor] = identify_SPC_predictor(u_data, y_data, T_ini, T_future, m, p);

%% 3. Controller Configuration
Q_val = 1.0;
R_val = 1e-4;

ControlParams.Q_bar   = kron(eye(T_future), Q_val * eye(p));
ControlParams.R_bar   = kron(eye(T_future), R_val * eye(m));
ControlParams.u_min   = -50;
ControlParams.u_max   = 50;
ControlParams.T_ini   = T_ini;
ControlParams.T_future = T_future;

P_uf = SPC_Predictor.P_u_future;
H_dense = 2 * (P_uf' * ControlParams.Q_bar * P_uf + ControlParams.R_bar);
ControlParams.H_osqp = sparse((H_dense + H_dense')/2);

fprintf('SPC configured with Q = %.1f, R = %.1e\n', Q_val, R_val);

%% 4. Simulation

T_simulation = 2222;
time_vector  = (0:T_simulation-1)' * Ts;

segment = floor(T_simulation / 6);
y_reference = [ones(segment,1); 0.75*ones(segment,1); 0.5*ones(segment,1); ...
               0.75*ones(segment,1); ones(segment,1); 1.25*ones(T_simulation-5*segment,1)];

SystemData.A = A; SystemData.B = B; SystemData.C = C; SystemData.D = D;
SystemData.Ts = Ts; SystemData.time = time_vector; SystemData.y_ref = y_reference;

[y_sim, u_sim] = simulate_SPC(SystemData, SPC_Predictor, ControlParams);

ess = abs(y_reference(end) - y_sim(end));

%% 5. Results 
fig = figure('Name', 'SPC for AVR', ...
             'Color', 'w', ...
             'Units', 'centimeters', ...
             'Position', [5 5 18 12]);  

% ---- Voltage Response ----
subplot(2,1,1);
plot(time_vector, y_reference, '--', 'Color', [0.85 0.2 0.2], ...
     'LineWidth', 2.5); hold on;
plot(time_vector, y_sim, '-', 'Color', [0.1 0.3 0.7], ...
     'LineWidth', 2.5);

ylabel('Terminal Voltage (p.u.)', 'FontSize', 18, 'Interpreter', 'latex');
title('AVR Response under Data-Driven Subspace Predictive Control', ...
      'FontSize', 18, 'Interpreter', 'latex');
legend({'Reference', 'SPC Output'}, 'Location', 'best', ...
       'FontSize', 12, 'Box', 'off', 'Interpreter', 'latex');
grid on; box on;
set(gca, 'FontSize', 12, 'LineWidth', 1.1);
xlim([time_vector(1) time_vector(end)]);

% ---- Control Input ----
subplot(2,1,2);
stairs(time_vector, u_sim, 'Color', [0.15 0.15 0.15], 'LineWidth', 2.5);

xlabel('Time (s)', 'FontSize', 11,'Interpreter', 'latex');
ylabel('Control Input $u(t)$', 'FontSize', 11, ...
       'Interpreter', 'latex');
grid on; box on;
set(gca, 'FontSize', 12, 'LineWidth', 1.1);
xlim([time_vector(1) time_vector(end)]);


set(fig, 'PaperPositionMode', 'auto');
print(fig, 'SPC_AVR_Result', '-dpng', '-r300');