function ctrl = calc_ctrl_params(config, constants)
%CALC_CTRL_PARAMS Calculate controller parameters
%
%   ctrl = calc_ctrl_params(config, constants)
%
%   Inputs:
%       config    - User config with fields: ctrl_enable, lambda_c, lambda_e,
%                   meas_noise_enable, meas_noise_std,
%                   a_pd, a_prd, a_cov, epsilon
%       constants - Physical constants with fields: gamma_N, Ts, k_B, T
%
%   Outputs:
%       ctrl - Struct with fields:
%           enable, lambda_c, gamma, Ts, lambda_e,
%           meas_noise_enable, meas_noise_std, meas_noise_seed,
%           a_pd, a_prd, a_cov, epsilon, g_cov, sigma2_deltaXT, k_B, T

    Ts = constants.Ts;

    ctrl.enable = double(config.ctrl_enable);
    ctrl.lambda_c = config.lambda_c;
    ctrl.gamma = constants.gamma_N;
    ctrl.Ts = Ts;

    % Measurement noise parameters
    ctrl.meas_noise_enable = double(config.meas_noise_enable);
    ctrl.meas_noise_std = config.meas_noise_std;
    ctrl.meas_noise_seed = randi(2^31-1);

    % EKF estimation parameters
    ctrl.a_pd = config.a_pd;
    ctrl.a_prd = config.a_prd;
    ctrl.a_cov = config.a_cov;
    ctrl.epsilon = config.epsilon;
    ctrl.k_B = constants.k_B;
    ctrl.T = constants.T;

    % Derived EKF parameters
    % sigma2_deltaXT: thermal position increment variance per axis [um^2]
    ctrl.sigma2_deltaXT = 4 * constants.k_B * constants.T * constants.Ts ...
                          / constants.gamma_N;

    % g_cov: normalization factor for residual covariance estimation
    ctrl.g_cov = sqrt(ctrl.sigma2_deltaXT) ...
                 * sqrt(2 + 1 / (1 - config.lambda_c^2));

    % Controller type and 7-state parameters
    ctrl.controller_type = config.controller_type;

    % Observer pole for controller_type=2 (0 = deadbeat)
    if isfield(config, 'lambda_e')
        ctrl.lambda_e = config.lambda_e;
    else
        ctrl.lambda_e = 0;  % default: deadbeat
    end

    ctrl.beta = config.beta;
    ctrl.lamdaF = config.lamdaF;
    ctrl.sigma2_noise = config.meas_noise_std.^2;  % 3x1 [um^2]
    ctrl.Pf_init_diag = config.Pf_init_diag;
    ctrl.Qz_diag_scaling = config.Qz_diag_scaling;
    ctrl.Rz_diag_scaling = config.Rz_diag_scaling;

    % KF measurement noise variance for controller_type=4 (0 = use default)
    if isfield(config, 'kf_R')
        ctrl.kf_R = config.kf_R;
    else
        ctrl.kf_R = 0;
    end

    % Pre-compute KF gain via DARE (Fe-based, closed-loop error dynamics)
    % L1=L2=L3=kf_L due to Fe structure
    if ctrl.kf_R > 0
        Fe = [0, 1, 0; 0, 0, 1; 0, 0, 1];
        H_kf = [1, 0, 0];
        Q_kf = ctrl.sigma2_deltaXT * diag([0, 0, 1]);
        [Pf_ss, ~, ~] = dare(Fe', H_kf', Q_kf, ctrl.kf_R);
        ctrl.kf_L = Pf_ss(1,1) / (Pf_ss(1,1) + ctrl.kf_R);
    else
        ctrl.kf_L = 1;  % deadbeat (R=0 equivalent)
    end

    % ---------------------------------------------------------------
    % 7-state EKF: per-axis self-consistent C_dpmr_eff, C_np_eff, IIR_bias_factor
    % via on-the-fly augmented Lyapunov (per writeup §5, §7).
    %
    % Each axis gets its own values computed from its actual (Q, R) since
    % R(1,1) and R(2,2) are per-axis (and Q(7,7) per-axis since 2026-04-22).
    % Replaces the legacy 2-D lookup which used a single Q/R for all axes
    % and was built with beta-derivation values, not production frozen_correct.
    % See test_script/build_helpers/compute_7state_cdpmr_eff.m for derivation.
    % ---------------------------------------------------------------
    here = fileparts(mfilename('fullpath'));              % .../model/controller
    project_root = fileparts(fileparts(here));            % .../
    addpath(fullfile(project_root, 'test_script', 'build_helpers'));

    ctrl.C_dpmr_eff      = zeros(3, 1);
    ctrl.C_np_eff        = zeros(3, 1);
    ctrl.IIR_bias_factor = ones(3, 1);

    opts_lyap = struct('f0', 0, 'verbose', false);
    L_max = 100;

    try
        for ax = 1:3
            % Q per-axis (Q1..Q6 shared, Q7 per-axis from slots 7,8,9 of 9x1 Qz)
            Q_kf_scale = [config.Qz_diag_scaling(1:6); config.Qz_diag_scaling(6+ax)];
            % R per-axis (R(1,1) from slots 1..3, R(2,2) from slots 4..6 of 6x1 Rz)
            R_kf_scale = [config.Rz_diag_scaling(ax); config.Rz_diag_scaling(3+ax)];

            [Cd, Cn, ~, A_aug, dout] = compute_7state_cdpmr_eff( ...
                config.lambda_c, 0, config.a_pd, Q_kf_scale, R_kf_scale, opts_lyap);

            ctrl.C_dpmr_eff(ax) = Cd;
            ctrl.C_np_eff(ax)   = Cn;

            % IIR bias factor (per axis) from same A_aug, Sigma_th
            Sigma_th = dout.Sigma_th;
            n_aug = 11;
            c_s = zeros(n_aug, 1); c_s(3) = 1; c_s(11) = -1;
            gamma_L = zeros(L_max+1, 1);
            A_L = eye(n_aug);
            for L = 0:L_max
                gamma_L(L+1) = c_s' * A_L * Sigma_th * c_s;
                A_L = A_L * A_aug;
            end
            rho = gamma_L / gamma_L(1);
            powers = ((1 - config.a_prd).^(1:L_max)).';
            S = sum(rho(2:L_max+1) .* powers);
            ctrl.IIR_bias_factor(ax) = 1 - (config.a_prd/(2-config.a_prd)) * (1 + 2*S);
        end
    catch ME
        warning('calc_ctrl_params:peraxis_failed', ...
                'Per-axis C_dpmr/beta compute failed: %s. Falling back to sentinel.', ...
                ME.message);
        ctrl.C_dpmr_eff      = -1 * ones(3, 1);
        ctrl.C_np_eff        = -1 * ones(3, 1);
        ctrl.IIR_bias_factor = ones(3, 1);
    end

    % ---------------------------------------------------------------
    % eq17-only: closed-form C_dpmr / C_n -> ctrl.C_*_eff_per_axis
    %
    % The _per_axis fields feed ONLY the eq17 chain:
    %   run_pure_simulation -> eq17_opts -> build_eq17_constants -> ctrl_const
    % The no-suffix Lyapunov fields above (ctrl.C_dpmr_eff / C_np_eff) feed
    % ONLY eq6 (motion_control_law_eq6.m reads them directly).
    % This split preserves each controller's verified configuration:
    %   eq6  -> per-axis augmented-Lyapunov values (qr-branch verified)
    %   eq17 -> closed-form values (eq17-branch verified, Phase 8/9)
    %
    % C_dpmr (paper Eq.21 H_T composed with HP1 LP1(a_pd), d=2):
    %   C_dpmr = (1-a)^2 * [ 2(1-a)(1-lc)/D + 2/((2-a)(1+lc)*D) ]
    %   with  D = 1 - (1-a)*lc,  a = a_pd,  lc = lambda_c
    %
    % C_n   (paper Eq.21 H_n composed with HP1 LP1(a_pd), d=2):
    %   C_n = (1-a)^2 * [ 2/(2-a)
    %                   + 2(1-a)^2*a*(1-lc) / ((2-a)*D)
    %                   + 2(1-lc)^2 / ((2-a)*(1+lc)*D) ]
    %   Reduces to 2/(1+lc) as a -> 0 (matches paper small-alpha limit).
    %   Derivation: reference/eq17_analysis/derivation/Cdpmr_Cn_derivation.tex
    %   Verification: test_script/learn_variance/derive_Cn_closed_v2.m
    % ---------------------------------------------------------------
    alpha           = config.a_pd;
    lambda          = config.lambda_c;
    one_minus_alpha = 1 - alpha;
    denom_common    = 1 - one_minus_alpha * lambda;

    term1 = 2 * one_minus_alpha * (1 - lambda) / denom_common;
    term2 = 2 / ((2 - alpha) * (1 + lambda) * denom_common);
    C_dpmr_closed = one_minus_alpha^2 * (term1 + term2);
    ctrl.C_dpmr_eff_per_axis = C_dpmr_closed * ones(3, 1);

    Cn_t1 = 2 / (2 - alpha);
    Cn_t2 = 2 * one_minus_alpha^2 * alpha * (1 - lambda) ...
            / ((2 - alpha) * denom_common);
    Cn_t3 = 2 * (1 - lambda)^2 ...
            / ((2 - alpha) * (1 + lambda) * denom_common);
    C_n_closed = one_minus_alpha^2 * (Cn_t1 + Cn_t2 + Cn_t3);
    ctrl.C_np_eff_per_axis = C_n_closed * ones(3, 1);

    % ---------------------------------------------------------------
    % X2a: Per-axis empirical IF_eff calibration (Phase 9 Stage I)
    %
    % The closed-form rho_dx assumes MA(2) eps with geometric tail
    % lambda_c^tau for tau >= 3. Empirical rho_dx in production conditions
    % oscillates negative around lag 7-10; closed-form IF_eff(s) over-
    % predicts variance by ~9% at a_cov=0.05. Using empirical IF_eff_emp(s)
    % from stored ACF makes the R(2,2) V1 ratio collapse to 1.00 +/- 0.05.
    %
    % Plumb: ctrl -> run_pure_simulation -> eq17_opts.IF_eff_calibrated
    %        -> build_eq17_constants -> ctrl_const.IF_eff_per_axis
    % If the calibration file is absent, leave empty so build_eq17_constants
    % falls back to the closed-form scalar IF_eff.
    % ---------------------------------------------------------------
    ctrl.IF_eff_calibrated_per_axis = [];

    if isfield(config, 'IF_eff_calibration_file') && ...
            ~isempty(config.IF_eff_calibration_file)
        calibration_file = config.IF_eff_calibration_file;
    else
        calibration_file = fullfile(project_root, 'reference', ...
                                    'eq17_analysis', 'verification', ...
                                    'phase9_stageI_acf_diagnosis.mat');
    end

    if isfile(calibration_file)
        try
            ctrl.IF_eff_calibrated_per_axis = calibrate_IF_eff( ...
                config.a_cov, config.lambda_c, calibration_file);
        catch ME
            warning('calc_ctrl_params:IF_eff_calibration_failed', ...
                    ['IF_eff calibration failed (%s); falling back ' ...
                     'to closed-form scalar.'], ME.message);
            ctrl.IF_eff_calibrated_per_axis = [];
        end
    end

end
