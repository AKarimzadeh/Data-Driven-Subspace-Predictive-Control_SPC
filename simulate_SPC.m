function [y_sim, u_sim] = simulate_SPC(Sys, Pred, Ctrl)
% Closed-loop simulation of Subspace Predictive Control using OSQP.

    [A,B,C,D] = deal(Sys.A, Sys.B, Sys.C, Sys.D);
    n_x = size(A,1); n_u = size(B,2); n_y = size(C,1);
    T_sim = length(Sys.time);
    T_ini = Ctrl.T_ini; T_future = Ctrl.T_future;

    y_sim = zeros(T_sim, n_y);
    u_sim = zeros(T_sim, n_u);
    x = zeros(n_x, 1);

    u_past = zeros(n_u*T_ini, 1);
    y_past = zeros(n_y*T_ini, 1);

    n_vars = n_u * T_future;
    A_osqp = speye(n_vars);
    l_osqp = Ctrl.u_min * ones(n_vars,1);
    u_osqp = Ctrl.u_max * ones(n_vars,1);

    prob = osqp;
    settings = prob.default_settings();
    settings.verbose = false;
    settings.warm_start = true;
    settings.eps_abs = 1e-4;
    settings.eps_rel = 1e-4;
    prob.setup(Ctrl.H_osqp, zeros(n_vars,1), A_osqp, l_osqp, u_osqp, settings);

    for k = 1:T_sim
        y_curr = C*x;
        y_sim(k,:) = y_curr';
        y_past = [y_past(n_y+1:end); y_curr];

        % Reference over the horizon
        idx_end = min(k+T_future-1, T_sim);
        y_ref_h = Sys.y_ref(k:idx_end);
        if length(y_ref_h) < T_future
            y_ref_h = [y_ref_h; repmat(y_ref_h(end), T_future-length(y_ref_h), 1)];
        end

        % Free response
        Y_free = Pred.P_y_past*y_past + Pred.P_u_past*u_past;
        Error  = Y_free - y_ref_h;
        f_val  = 2 * Pred.P_u_future' * Ctrl.Q_bar * Error;

        % Solve QP
        prob.update('q', f_val);
        res = prob.solve();

        if strcmp(res.info.status,'solved') || strcmp(res.info.status,'solved inaccurate')
            u_opt = res.x(1:n_u);
        else
            u_opt = 0;
        end

        % Apply input
        x = A*x + B*u_opt;
        u_sim(k,:) = u_opt;
        u_past = [u_past(n_u+1:end); u_opt];
    end
end