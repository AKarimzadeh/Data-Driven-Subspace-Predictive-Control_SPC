function Predictor = identify_SPC_predictor(u_data, y_data, T_ini, T_future, m, p)

    L = T_ini + T_future;
    T_data = size(u_data, 1);
    n_cols = T_data - L + 1;

    U_Hankel = zeros(m*L, n_cols);
    Y_Hankel = zeros(p*L, n_cols);

    for i = 1:n_cols
        U_Hankel(:,i) = reshape(u_data(i:i+L-1,:)', [], 1);
        Y_Hankel(:,i) = reshape(y_data(i:i+L-1,:)', [], 1);
    end

    U_P = U_Hankel(1:m*T_ini, :);
    U_F = U_Hankel(m*T_ini+1:end, :);
    Y_P = Y_Hankel(1:p*T_ini, :);
    Y_F = Y_Hankel(p*T_ini+1:end, :);

    W = [Y_P; U_P; U_F];
    [Uu, Ss, Vv] = svd(W, 'econ');
    tol = 1e-10;
    r = sum(diag(Ss) > tol);

    W_pinv = Vv(:,1:r) * diag(1./diag(Ss(1:r,1:r))) * Uu(:,1:r)';
    P = Y_F * W_pinv;

    Predictor.P_y_past   = P(:, 1:p*T_ini);
    Predictor.P_u_past   = P(:, p*T_ini+1 : p*T_ini+m*T_ini);
    Predictor.P_u_future = P(:, p*T_ini+m*T_ini+1:end);

end