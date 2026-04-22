function [f_d, ekf_out] = motion_control_law_7state(del_pd, pd, p_m, params)
%MOTION_CONTROL_LAW_7STATE Per-axis 7-state EKF controller
%
%   [f_d, ekf_out] = motion_control_law_7state(del_pd, pd, p_m, params)
%
%   Implements 3 independent 7-state EKF controllers (one per axis) with
%   corrected execution order: control -> measurement -> IIR -> EKF.
%
%   State vector per axis: [del_p1, del_p2, del_p3, d, del_d, a, del_a]
%     del_p1..3: tracking error delay chain
%     d, del_d:  disturbance pair
%     a, del_a:  motion gain pair
%
%   Inputs:
%       del_pd - Trajectory increment p_d[k+1] - p_d[k] [3x1, um]
%       pd     - Current desired position p_d[k] [3x1, um]
%       p_m    - Measured position p_m[k] [3x1, um]
%       params - Parameter structure from calc_simulation_params
%
%   Outputs:
%       f_d     - Control force f_d[k] [3x1, pN]
%       ekf_out - EKF diagnostic [4x1]: [a_hat_x/a_nom; a_hat_z/a_nom; 0; 0]

    % Check if control is enabled
    if params.ctrl.enable < 0.5
        f_d = zeros(3, 1);
        ekf_out = [1; 1; 0; 0];
        return;
    end

    % --- Persistent declarations ---

    % 7-state estimates (3x1 vectors, one element per axis)
    persistent del_p1_hat del_p2_hat del_p3_hat
    persistent d_hat del_d_hat
    persistent a_hat del_a_hat

    % Covariance (3 independent 7x7 matrices)
    persistent Pf_1 Pf_2 Pf_3

    % Delay buffers
    persistent pd_k1 pd_k2

    % IIR states (3x1 vectors, same as 23-state dual-layer highpass)
    persistent del_pmd_k1 del_pmrd_k1
    persistent del_pmr2_avg

    % Previous a_m and warm-up counter
    persistent a_m_k1
    persistent warmup_count

    % Constants (extracted once)
    persistent initialized
    persistent gamma_N Ts lambda_c k_B T_temp
    persistent a_pd a_prd a_cov beta lamdaF
    persistent sigma2_noise sigma2_deltaXT
    persistent Q_1 Q_2 Q_3 Rz_scaling
    persistent C_dpmr_eff_const C_np_eff_const use_lookup
    persistent IIR_bias_factor_const

    %% Step [0]: Initialization
    if isempty(initialized)
        initialized = true;

        % 0A. Extract constants
        gamma_N = params.ctrl.gamma;
        Ts      = params.ctrl.Ts;
        lambda_c = params.ctrl.lambda_c;
        a_pd    = params.ctrl.a_pd;
        a_prd   = params.ctrl.a_prd;
        a_cov   = params.ctrl.a_cov;
        k_B     = params.ctrl.k_B;
        T_temp  = params.ctrl.T;
        beta    = params.ctrl.beta;
        lamdaF  = params.ctrl.lamdaF;
        sigma2_noise    = params.ctrl.sigma2_noise;       % 3x1 [um^2]
        sigma2_deltaXT  = params.ctrl.sigma2_deltaXT;     % scalar [um^2]

        % 0B. Nominal motion gain a_nom = Ts / gamma_N
        a_nom = Ts / gamma_N;

        % 0C. EKF states [k=0]
        del_p1_hat = zeros(3,1);
        del_p2_hat = zeros(3,1);
        del_p3_hat = zeros(3,1);
        d_hat      = zeros(3,1);
        del_d_hat  = zeros(3,1);
        % Wall-aware initial a_hat: compute h_bar from initial position and wall model
        % to avoid slow EKF convergence when starting near wall.
        p0_init       = params.common.p0;           % 3x1 initial position [um]
        w_hat_init    = params.wall.w_hat;          % 3x1 wall normal
        pz_init       = params.wall.pz;             % scalar wall offset [um]
        R_init        = params.common.R;            % particle radius [um]
        h_init        = p0_init(:)' * w_hat_init(:) - pz_init;
        h_bar_init    = max(h_init / R_init, 1.001);
        [cpara_init, cperp_init] = calc_correction_functions(h_bar_init);
        % For wall-normal w_hat = [0;0;1], this maps: x,y use c_para, z uses c_perp
        a_hat         = [a_nom / cpara_init; a_nom / cpara_init; a_nom / cperp_init];
        del_a_hat  = zeros(3,1);

        % 0D. Covariance initialization
        Pf_diag = params.ctrl.Pf_init_diag;  % 7x1
        Pf_1 = diag(Pf_diag);
        Pf_2 = diag(Pf_diag);
        Pf_3 = diag(Pf_diag);

        % 0E. Q and R matrices (per axis)
        % Qz_diag_scaling layout (9x1): [Q1;Q2;Q3;Q4;Q5;Q6; Q7_x;Q7_y;Q7_z]
        Qz_scaling = params.ctrl.Qz_diag_scaling;
        Rz_scaling = params.ctrl.Rz_diag_scaling;   % 6x1 [R_pos_x; R_pos_y; R_pos_z; R_gain_x; R_gain_y; R_gain_z]

        % Build per-axis Q: Q(1..6) shared, Q(7,7) per-axis from Qz_scaling(6+axis)
        Q_shared = sigma2_deltaXT * diag(Qz_scaling(1:6));   % 6x6 shared diagonal
        Q_1 = zeros(7,7); Q_1(1:6,1:6) = Q_shared; Q_1(7,7) = sigma2_deltaXT * Qz_scaling(7);
        Q_2 = zeros(7,7); Q_2(1:6,1:6) = Q_shared; Q_2(7,7) = sigma2_deltaXT * Qz_scaling(8);
        Q_3 = zeros(7,7); Q_3(1:6,1:6) = Q_shared; Q_3(7,7) = sigma2_deltaXT * Qz_scaling(9);

        % 0E.1 C_dpmr_eff / C_np_eff from augmented Lyapunov lookup
        % (computed offline by build_cdpmr_eff_lookup.m)
        if isfield(params.ctrl, 'C_dpmr_eff') && params.ctrl.C_dpmr_eff > 0
            C_dpmr_eff_const = params.ctrl.C_dpmr_eff;
            C_np_eff_const   = params.ctrl.C_np_eff;
            use_lookup       = true;
        else
            % Fallback: K=2 approximation (legacy formula)
            C_dpmr_eff_const = (1-a_pd)^2 * (2*(1-a_pd)*(1-lambda_c) / (1-(1-a_pd)*lambda_c) ...
                             + (2/(2-a_pd)) / ((1+lambda_c)*(1-(1-a_pd)*lambda_c)));
            C_np_eff_const   = 2 / (1 + lambda_c);  % legacy noise_corr formula
            use_lookup       = false;
        end

        % 0E.2 IIR_bias_factor (Task 1c): finite-sample + autocorrelation
        % correction for the EMA variance estimator. Default 1.0 = no correction.
        % See test_script/build_bias_factor_lookup.m and Task 1b report.
        if isfield(params.ctrl, 'IIR_bias_factor') && params.ctrl.IIR_bias_factor > 0
            IIR_bias_factor_const = params.ctrl.IIR_bias_factor;
        else
            IIR_bias_factor_const = 1.0;
        end

        % 0F. Delay buffers
        pd_k1 = pd;
        pd_k2 = pd;

        % 0G. IIR states
        del_pmd_k1  = zeros(3,1);
        del_pmrd_k1 = zeros(3,1);
        % Pre-fill del_pmr2_avg at predicted steady-state Var(del_pmr) per axis
        % (Option P5). This addresses both the gate transient (was Session 6
        % finding) AND the IIR EMA growth phase that drives a_m toward 0 for
        % ~1000 samples, biasing a_hat downward (Session 7 finding).
        %   Var(del_pmr)_ss = Cdpmr_eff * sigma2_dXT * (a_axis/a_nom)
        %                   + Cnp_eff  * sigma2_n(axis)
        % Per-axis using wall-aware a_hat from line 98.
        ar_init = a_hat / a_nom;   % 3x1
        del_pmr2_avg = C_dpmr_eff_const * sigma2_deltaXT * ar_init ...
                     + C_np_eff_const * sigma2_noise;
        % Floor at 10x var_threshold to ensure gate stays open even if lookup
        % is unavailable / reports tiny value.
        del_pmr2_avg = max(del_pmr2_avg, 0.01 * sigma2_deltaXT);

        % 0H. Previous values
        a_m_k1  = [a_nom; a_nom; a_nom];

        % 0I. IIR warm-up counter
        % P6 (Session 7): shortened from 0.2s (320 steps) to 2 steps minimum
        % required for delay buffer (pd_k2) validity. With P5 IIR pre-fill,
        % the original 320-step warmup is unnecessary AND harmful: during
        % no-control phase the IIR sees noise-only del_pmr, decaying its
        % pre-filled value toward sensor-noise floor. Result: a_m at warmup
        % end is biased low, which (with frozen Q) produces the residual
        % +5-9% bias seen in Sessions 5 and 7 short-window measurements.
        warmup_count = 2;

        % 0J. Return zeros on first call
        f_d = zeros(3, 1);
        ekf_out = [a_nom; a_nom; 0; 0];
        return;
    end

    %% Step [1]: Measurement Processing
    %  del_pm = p_d[k-2] - p_m[k]
    del_pm = pd_k2 - p_m;

    %% Step [2]: IIR Single-layer Highpass + Proper Variance
    %  LP of measurement
    del_pmd  = (1 - a_pd) * del_pmd_k1 + a_pd * del_pm;
    del_pmr  = del_pm - del_pmd;                             % HP residual
    %  Running mean and mean-square of HP residual
    del_pmrd = (1 - a_prd) * del_pmrd_k1 + a_prd * del_pmr; % E[del_pmr]
    del_pmr2_avg    = (1 - a_cov) * del_pmr2_avg + a_cov * del_pmr.^2;    % E[del_pmr^2]
    %  Variance = E[X^2] - E[X]^2
    del_pmr_var    = max(del_pmr2_avg - del_pmrd.^2, 0);                  % 3x1 [um^2]

    %% Eq.13 Gain Recovery (runs during warm-up AND normal operation)
    % Uses C_dpmr_eff and C_np_eff from augmented Lyapunov lookup
    % (motion_control_law_7state_cdpmr_eff). If lookup unavailable,
    % falls back to K=2 formula (legacy, 24% bias at lc=0.7 vs truth).
    %
    % See:
    %   test_script/compute_7state_cdpmr_eff.m  (derivation)
    %   test_script/build_cdpmr_eff_lookup.m    (offline construction)
    %   test_results/verify/cdpmr_eff_lookup.mat (the lookup table)
    den        = C_dpmr_eff_const * 4 * k_B * T_temp;        % [pN*um]
    noise_corr = C_np_eff_const * sigma2_noise;              % 3x1 [um^2]
    % Task 1c: unbias del_pmr_var for IIR finite-sample + autocorrelation bias
    % before subtracting noise term and dividing by den.
    del_pmr_var_unbiased = del_pmr_var / IIR_bias_factor_const;
    a_m        = max((del_pmr_var_unbiased - noise_corr) / den, 0);   % 3x1 [um/pN]

    %% IIR Warm-up gate: only run measurement + IIR + Eq.13, skip control + EKF
    if warmup_count > 0
        warmup_count = warmup_count - 1;

        % Update delay buffers, IIR states, and a_m history
        pd_k2 = pd_k1;
        pd_k1 = pd;
        del_pmd_k1  = del_pmd;
        del_pmrd_k1 = del_pmrd;
        a_m_k1 = a_m;

        % On last warmup step: seed EKF del_p states with IIR LP estimate
        if warmup_count == 0
            del_p1_hat = del_pmd;
            del_p2_hat = del_pmd;
            del_p3_hat = del_pmd;
        end

        f_d = zeros(3, 1);
        ekf_out = [a_hat(1); a_hat(3); 0; 0];
        return;
    end

    %% Step [3]: Control Force (uses persistent [k] estimates, BEFORE EKF update)
    %  f_d = (1./a_hat) .* (del_pd + (1-lc)*del_p3_hat - d_hat)

    f_d = (1 ./ a_hat) .* ...
          (del_pd + (1 - lambda_c) * del_p3_hat - d_hat);

    %% Step [4]: EKF Update

    % Pack per-axis state vectors
    states_x = [del_p1_hat(1); del_p2_hat(1); del_p3_hat(1); ...
                d_hat(1); del_d_hat(1); a_hat(1); del_a_hat(1)];
    states_y = [del_p1_hat(2); del_p2_hat(2); del_p3_hat(2); ...
                d_hat(2); del_d_hat(2); a_hat(2); del_a_hat(2)];
    states_z = [del_p1_hat(3); del_p2_hat(3); del_p3_hat(3); ...
                d_hat(3); del_d_hat(3); a_hat(3); del_a_hat(3)];

    % Measurements: [del_pm_i; a_m_k1_i] (a_m uses previous step delay)
    meas_x = [del_pm(1); a_m_k1(1)];
    meas_y = [del_pm(2); a_m_k1(2)];
    meas_z = [del_pm(3); a_m_k1(3)];

    % Adaptive R: when del_pmr_var is too small (no thermal excitation),
    % inflate gain channel noise to prevent EKF from trusting a_m = 0.
    %
    % Note: adaptive R(2,2) based on chi-squared statistics was tested but
    % destabilizes the system because Q and R were co-tuned as a pair.
    % Changing R alone breaks the balance. Proper adaptive R requires
    % re-sweeping Q simultaneously (future work).
    var_threshold = sigma2_deltaXT * 0.001;
    % Rz_scaling layout (6x1): [R_pos_x; R_pos_y; R_pos_z; R_gain_x; R_gain_y; R_gain_z]
    R_i = cell(3,1);
    for i = 1:3
        r_pos_base_i = sigma2_deltaXT * Rz_scaling(i);
        r_gain_base_i = sigma2_deltaXT * Rz_scaling(3+i);
        if del_pmr_var(i) < var_threshold
            r_gain_i = 1e6;     % effectively ignore gain measurement
        else
            r_gain_i = r_gain_base_i;
        end
        R_i{i} = diag([r_pos_base_i, r_gain_i]);
    end

    % --- Fe_err matrices for covariance propagation ---
    Fe_err_x = [0 1 0  0  0  0           0; ...
                0 0 1  0  0  0           0; ...
                0 0 1 -1  0 -f_d(1)  0; ...
                0 0 0  1  1  0           0; ...
                0 0 0  0  1  0           0; ...
                0 0 0  0  0  1           1; ...
                0 0 0  0  0  0           1];
    Fe_err_y = [0 1 0  0  0  0           0; ...
                0 0 1  0  0  0           0; ...
                0 0 1 -1  0 -f_d(2)  0; ...
                0 0 0  1  1  0           0; ...
                0 0 0  0  1  0           0; ...
                0 0 0  0  0  1           1; ...
                0 0 0  0  0  0           1];
    % z axis: beta coupling (chart extension)
    Fe_err_z = [0 1 0  0          0         0          0; ...
                0 0 1  0          0         0          0; ...
                0 0 1 -1          0        -f_d(3) 0; ...
                0 0 0  1+beta    -beta      0          0; ...
                0 0 0  1          0         0          0; ...
                0 0 0  0          0         1+beta    -beta; ...
                0 0 0  0          0         1          0];

    % --- EKF injection + covariance (returns inj, not states) ---
    % x-axis: lamdaF=1
    [inj_x, Pf_1] = ekf_update_7state( ...
        states_x, Pf_1, meas_x, Q_1, R_i{1}, Fe_err_x, 1);
    % y-axis: lamdaF=1
    [inj_y, Pf_2] = ekf_update_7state( ...
        states_y, Pf_2, meas_y, Q_2, R_i{2}, Fe_err_y, 1);
    % z-axis: beta coupling, lamdaF=lamdaF
    [inj_z, Pf_3] = ekf_update_7state( ...
        states_z, Pf_3, meas_z, Q_3, R_i{3}, Fe_err_z, lamdaF);

    % --- Explicit state update  ---
    del_p1_hat_new = zeros(3,1);
    del_p2_hat_new = zeros(3,1);
    del_p3_hat_new = zeros(3,1);
    d_hat_new      = zeros(3,1);
    del_d_hat_new  = zeros(3,1);
    a_hat_new      = zeros(3,1);
    del_a_hat_new  = zeros(3,1);

    for ax = 1:2   % x, y axes
        if ax == 1, inj = inj_x; else, inj = inj_y; end
        del_p1_hat_new(ax) = del_p2_hat(ax)                           + inj(1);
        del_p2_hat_new(ax) = del_p3_hat(ax)                           + inj(2);
        del_p3_hat_new(ax) = lambda_c * del_p3_hat(ax)                + inj(3);
        d_hat_new(ax)      = d_hat(ax) + del_d_hat(ax)                + inj(4);
        del_d_hat_new(ax)  = del_d_hat(ax)                            + inj(5);
        a_hat_new(ax)      = a_hat(ax) + del_a_hat(ax)                + inj(6);
        del_a_hat_new(ax)  = del_a_hat(ax)                            + inj(7);
    end

    % z-axis — beta coupling (chart extension)
    inj = inj_z;
    del_p1_hat_new(3) = del_p2_hat(3)                                 + inj(1);
    del_p2_hat_new(3) = del_p3_hat(3)                                 + inj(2);
    del_p3_hat_new(3) = lambda_c * del_p3_hat(3)                      + inj(3);
    d_hat_new(3)      = (1+beta)*d_hat(3) - beta*del_d_hat(3)         + inj(4);
    del_d_hat_new(3)  = d_hat(3)                                      + inj(5);
    a_hat_new(3)      = (1+beta)*a_hat(3) - beta*del_a_hat(3)         + inj(6);
    del_a_hat_new(3)  = a_hat(3)                                      + inj(7);

    %% Step [6]: Persistent Shifts

    % Apply updated states
    del_p1_hat = del_p1_hat_new;
    del_p2_hat = del_p2_hat_new;
    del_p3_hat = del_p3_hat_new;
    d_hat      = d_hat_new;
    del_d_hat  = del_d_hat_new;
    a_hat      = a_hat_new;
    del_a_hat  = del_a_hat_new;

    % Delay buffers
    pd_k2 = pd_k1;
    pd_k1 = pd;

    % IIR states
    del_pmd_k1  = del_pmd;
    del_pmrd_k1 = del_pmrd;  % stores running mean of HP residual

    % Previous values
    a_m_k1  = a_m;

    %% Step [7]: Output
    ekf_out = [a_hat(1); ...            % x-axis gain [um/pN]
               a_hat(3); ...            % z-axis gain [um/pN]
               0; 0];

end


%% ==================== Local Function ====================

function [inj, Pf_new] = ekf_update_7state( ...
    states, Pf, meas, Q, R, Fe_err, lamdaF_i)
%EKF_UPDATE_7STATE Single-axis 7-state EKF: injection + covariance
%
%   Split design per paper Eq.16-21:
%     - Returns injection vector (L * innovation), NOT updated states
%     - Caller performs explicit state update (Eq.16)
%     - Covariance uses Fe_err from error dynamics (Eq.18)
%
%   Inputs:
%       states:    7x1 = [del_p1; del_p2; del_p3; d; del_d; a; del_a]
%       Pf:        7x7 forecast covariance
%       meas:      2x1 = [del_pm_i; a_m_i]
%       Q:         7x7 process noise
%       R:         2x2 measurement noise
%       Fe_err:    7x7 error dynamics matrix (Eq.18), Fe_err(3,3) = 1
%       lamdaF_i:  scalar, forgetting factor
%
%   Outputs:
%       inj:     7x1 injection vector (= L * innovation)
%       Pf_new:  7x7 forecast covariance for next step

    H = [1 0 0 0 0 0 0; ...
         0 0 0 0 0 1 0];

    % Innovation (Eq.16 feedback)
    err = meas - H * states;

    % Innovation covariance (Eq.19)
    S = H * Pf * H' + R;
    S = 0.5 * (S + S') + 1e-20 * eye(2);

    % Kalman gain (Eq.19)
    L = (Pf * H') / S;

    % Injection vector — caller uses this for explicit state update
    inj = L * err;

    % Posterior covariance (Eq.20) with forgetting factor
    P = (1 / lamdaF_i) * (eye(7) - L * H) * Pf;
    P = 0.5 * (P + P');

    % Forecast covariance (Eq.21) — uses Fe_err (error dynamics)
    Pf_new = Fe_err * P * Fe_err' + Q;
    Pf_new = 0.5 * (Pf_new + Pf_new');

end
