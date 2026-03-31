function [f_d, ekf_out] = motion_control_law_5state(del_pd, pd, p_m, params)
%MOTION_CONTROL_LAW_5STATE 5-state KF with dual measurement
%
%   Simplified 7-state EKF: drops disturbance (d, del_d), keeps gain (a, del_a).
%   Dual measurement: del_pm (per-sample) + a_m from Eq.13 (IIR variance).
%
%   State: [del_p1, del_p2, del_p3, a, del_a]^T  (per axis, 3 independent)
%   H = [1,0,0,0,0; 0,0,0,1,0]
%   Fe(f_d) = [0,1,0,0,0; 0,0,1,0,0; 0,0,1,-f_d,0; 0,0,0,1,1; 0,0,0,0,1]

    if params.ctrl.enable < 0.5
        f_d = zeros(3, 1);
        ekf_out = zeros(4, 1);
        return;
    end

    persistent initialized
    persistent lambda_c a_nom a_min Ts_ctrl
    persistent sigma2_deltaXT
    persistent Q_kf                   % 5x5 process noise
    persistent Rz_scaling             % 2x1 [r_pos; r_gain] scaling
    persistent pd_k1 pd_k2           % delay buffers
    persistent del_p1_hat del_p2_hat del_p3_hat  % 3x1 each
    persistent a_hat del_a_hat        % 3x1 each
    persistent Pf_1 Pf_2 Pf_3        % 5x5 each
    % IIR filter states (from ctrl7)
    persistent a_pd a_prd a_cov
    persistent k_B T_temp sigma2_noise
    persistent C_dpmr den_eq13
    persistent del_pmd_k1 del_pmrd_k1 del_pmr2_avg
    persistent a_m_k1
    persistent warmup_count

    %% Initialization
    if isempty(initialized)
        initialized = true;

        lambda_c = params.ctrl.lambda_c;
        Ts_ctrl  = params.ctrl.Ts;
        a_nom    = Ts_ctrl / params.ctrl.gamma;
        a_min    = 0.01 * a_nom;

        sigma2_deltaXT = params.ctrl.sigma2_deltaXT;

        % IIR filter parameters
        a_pd  = params.ctrl.a_pd;
        a_prd = params.ctrl.a_prd;
        a_cov = params.ctrl.a_cov;
        k_B    = params.ctrl.k_B;
        T_temp = params.ctrl.T;
        sigma2_noise = params.ctrl.sigma2_noise;  % 3x1

        % Eq.13: C_dpmr formula (same as ctrl7, controller-1 K=2 approximation)
        C_dpmr = (1-a_pd)^2 * (2*(1-a_pd)*(1-lambda_c) / (1-(1-a_pd)*lambda_c) ...
               + (2/(2-a_pd)) / ((1+lambda_c)*(1-(1-a_pd)*lambda_c)));
        den_eq13 = C_dpmr * 4 * k_B * T_temp;

        % Q (5x5): reuse first 5 of Qz_diag_scaling
        Q_kf = sigma2_deltaXT * diag(params.ctrl.Qz_diag_scaling(1:5));

        % R scaling: [r_pos; r_gain]
        Rz_scaling = params.ctrl.Rz_diag_scaling;  % 2x1

        % Pf init (5x5): reuse first 5 of Pf_init_diag
        Pf_init = diag(params.ctrl.Pf_init_diag(1:5));
        Pf_1 = Pf_init; Pf_2 = Pf_init; Pf_3 = Pf_init;

        % Delay buffers
        pd_k1 = pd; pd_k2 = pd;

        % States
        del_p1_hat = zeros(3,1);
        del_p2_hat = zeros(3,1);
        del_p3_hat = zeros(3,1);
        a_hat      = [a_nom; a_nom; a_nom];
        del_a_hat  = zeros(3,1);

        % IIR states
        del_pmd_k1   = zeros(3,1);
        del_pmrd_k1  = zeros(3,1);
        del_pmr2_avg = zeros(3,1);
        a_m_k1       = [a_nom; a_nom; a_nom];

        % Warm-up: 0.2s for IIR to settle
        warmup_count = round(0.2 / Ts_ctrl);

        f_d = zeros(3,1);
        ekf_out = [a_nom; a_nom; 0; 0];
        return;
    end

    %% Step [1]: Measurement
    del_pm = pd_k2 - p_m;

    %% Step [2]: IIR HP filter + Eq.13 (runs always, same as ctrl7)
    del_pmd  = (1 - a_pd) * del_pmd_k1 + a_pd * del_pm;
    del_pmr  = del_pm - del_pmd;
    del_pmrd = (1 - a_prd) * del_pmrd_k1 + a_prd * del_pmr;
    del_pmr2_avg = (1 - a_cov) * del_pmr2_avg + a_cov * del_pmr.^2;
    del_pmr_var  = max(del_pmr2_avg - del_pmrd.^2, 0);

    % Eq.13 gain recovery
    noise_corr = (2 / (1 + lambda_c)) * sigma2_noise;
    a_m = max((del_pmr_var - noise_corr) / den_eq13, 0);

    %% Warm-up gate
    if warmup_count > 0
        warmup_count = warmup_count - 1;
        pd_k2 = pd_k1; pd_k1 = pd;
        del_pmd_k1 = del_pmd; del_pmrd_k1 = del_pmrd;
        a_m_k1 = a_m;

        if warmup_count == 0
            del_p1_hat = del_pmd;
            del_p2_hat = del_pmd;
            del_p3_hat = del_pmd;
        end

        f_d = zeros(3,1);
        ekf_out = [a_hat(1); a_hat(3); 0; 0];
        return;
    end

    %% Step [3]: Control Law (uses a_hat, stable with dual measurement)
    f_d = (1 ./ a_hat) .* (del_pd + (1 - lambda_c) * del_p3_hat);

    %% Step [4]: Adaptive R (per axis)
    var_threshold = sigma2_deltaXT * 0.001;
    r_pos_base  = sigma2_deltaXT * Rz_scaling(1);
    r_gain_base = sigma2_deltaXT * Rz_scaling(2);

    R_1 = build_R(del_pmr_var(1), var_threshold, r_pos_base, r_gain_base);
    R_2 = build_R(del_pmr_var(2), var_threshold, r_pos_base, r_gain_base);
    R_3 = build_R(del_pmr_var(3), var_threshold, r_pos_base, r_gain_base);

    %% Step [5]: Per-axis EKF update (dual measurement)
    meas_1 = [del_pm(1); a_m_k1(1)];
    meas_2 = [del_pm(2); a_m_k1(2)];
    meas_3 = [del_pm(3); a_m_k1(3)];

    [inj_1, Pf_1] = ekf_update_5dual( ...
        [del_p1_hat(1); del_p2_hat(1); del_p3_hat(1); a_hat(1); del_a_hat(1)], ...
        Pf_1, meas_1, Q_kf, R_1, f_d(1));
    [inj_2, Pf_2] = ekf_update_5dual( ...
        [del_p1_hat(2); del_p2_hat(2); del_p3_hat(2); a_hat(2); del_a_hat(2)], ...
        Pf_2, meas_2, Q_kf, R_2, f_d(2));
    [inj_3, Pf_3] = ekf_update_5dual( ...
        [del_p1_hat(3); del_p2_hat(3); del_p3_hat(3); a_hat(3); del_a_hat(3)], ...
        Pf_3, meas_3, Q_kf, R_3, f_d(3));

    %% Step [6]: State update
    for ax = 1:3
        if ax == 1, inj = inj_1; elseif ax == 2, inj = inj_2; else, inj = inj_3; end
        del_p1_hat(ax) = del_p2_hat(ax)                + inj(1);
        del_p2_hat(ax) = del_p3_hat(ax)                + inj(2);
        del_p3_hat(ax) = lambda_c * del_p3_hat(ax)     + inj(3);
        a_hat(ax)      = a_hat(ax) + del_a_hat(ax)     + inj(4);
        del_a_hat(ax)  = del_a_hat(ax)                 + inj(5);
    end
    a_hat = max(a_hat, a_min);

    %% Step [7]: Update buffers
    pd_k2 = pd_k1; pd_k1 = pd;
    del_pmd_k1 = del_pmd; del_pmrd_k1 = del_pmrd;
    a_m_k1 = a_m;

    %% Output
    ekf_out = [a_hat(1); a_hat(3); 0; 0];
end


function R_i = build_R(var_i, var_thresh, r_pos, r_gain)
%BUILD_R Adaptive 2x2 measurement noise for one axis
    if var_i < var_thresh
        R_i = diag([r_pos, 1e6]);   % ignore gain channel
    else
        R_i = diag([r_pos, r_gain]);
    end
end


function [inj, Pf_new] = ekf_update_5dual(states, Pf, meas, Q, R, f_d_i)
%EKF_UPDATE_5DUAL 5-state KF update with dual measurement
%
%   H = [1,0,0,0,0;   <- del_pm
%        0,0,0,1,0]   <- a_m (Eq.13)

    H = [1, 0, 0, 0, 0;
         0, 0, 0, 1, 0];

    Fe = [0, 1, 0,      0, 0;
          0, 0, 1,      0, 0;
          0, 0, 1, -f_d_i, 0;
          0, 0, 0,      1, 1;
          0, 0, 0,      0, 1];

    % Innovation (2x1)
    err = meas - H * states;

    % Innovation covariance (2x2)
    S = H * Pf * H' + R;
    S = 0.5 * (S + S') + 1e-20 * eye(2);  % symmetry + numerical stability

    % Kalman gain (5x2)
    L = (Pf * H') / S;

    % Injection (5x1)
    inj = L * err;

    % Posterior covariance
    P = (eye(5) - L * H) * Pf;
    P = 0.5 * (P + P');

    % Forecast covariance
    Pf_new = Fe * P * Fe' + Q;
    Pf_new = 0.5 * (Pf_new + Pf_new');
end
