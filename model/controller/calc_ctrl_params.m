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
    % 7-state EKF: load C_dpmr_eff / C_np_eff lookup from augmented Lyapunov
    % See test_script/compute_7state_cdpmr_eff.m for derivation.
    % Fallback to K=2 approximation if lookup file is missing.
    % ---------------------------------------------------------------
    % Determine project root from this file's location
    here = fileparts(mfilename('fullpath'));              % .../model/controller
    project_root = fileparts(fileparts(here));            % .../
    lut_path = fullfile(project_root, 'test_results', 'verify', 'cdpmr_eff_lookup.mat');
    if exist(lut_path, 'file')
        LUT = load(lut_path);
        if isfield(LUT, 'Cdpmr_tab') && isfield(LUT, 'Cnp_tab') && isfield(LUT, 'lc_grid')
            lc_clamped = max(min(ctrl.lambda_c, LUT.lc_grid(end)), LUT.lc_grid(1));
            % Use first rho column (lookup is effectively 1-D over lc since
            % Q_kf/R_kf are fixed design parameters; rho dimension is redundant)
            ctrl.C_dpmr_eff = interp1(LUT.lc_grid, LUT.Cdpmr_tab(:, 1), lc_clamped, 'linear');
            ctrl.C_np_eff   = interp1(LUT.lc_grid, LUT.Cnp_tab(:, 1),   lc_clamped, 'linear');
            if abs(config.a_pd - LUT.a_pd) > 1e-6
                warning('calc_ctrl_params:apd_mismatch', ...
                    'config.a_pd=%.4f but lookup built at a_pd=%.4f', ...
                    config.a_pd, LUT.a_pd);
            end
        else
            ctrl.C_dpmr_eff = -1;   % sentinel -> use fallback in controller
            ctrl.C_np_eff   = -1;
        end
    else
        ctrl.C_dpmr_eff = -1;       % sentinel -> use fallback in controller
        ctrl.C_np_eff   = -1;
    end

    % ---------------------------------------------------------------
    % 7-state EKF: load IIR_bias_factor lookup (Task 1c)
    % Correction for finite-sample + autocorrelation bias of the EMA
    % variance estimator. Default 1.0 (no correction) if lookup missing.
    % See test_script/build_bias_factor_lookup.m for derivation.
    % ---------------------------------------------------------------
    bf_path = fullfile(project_root, 'test_results', 'verify', 'bias_factor_lookup.mat');
    if exist(bf_path, 'file')
        BF = load(bf_path);
        if isfield(BF, 'bias_factor_tab') && isfield(BF, 'lc_grid')
            lc_clamped_bf = max(min(ctrl.lambda_c, BF.lc_grid(end)), BF.lc_grid(1));
            ctrl.IIR_bias_factor = interp1(BF.lc_grid, BF.bias_factor_tab, ...
                                            lc_clamped_bf, 'linear');
            if abs(config.a_prd - BF.a_prd) > 1e-6
                warning('calc_ctrl_params:aprd_bf_mismatch', ...
                    'config.a_prd=%.4f but bias_factor_lookup built at a_prd=%.4f', ...
                    config.a_prd, BF.a_prd);
            end
        else
            ctrl.IIR_bias_factor = 1.0;
        end
    else
        ctrl.IIR_bias_factor = 1.0;
    end

end
