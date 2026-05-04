function ctrl = calc_ctrl_params(config, constants)
%CALC_CTRL_PARAMS Calculate controller parameters
%
%   ctrl = calc_ctrl_params(config, constants)
%
%   Inputs:
%       config    - User config with fields: ctrl_enable, lambda_c,
%                   meas_noise_enable, meas_noise_std,
%                   a_pd, a_prd, a_cov, epsilon,
%                   sigma2_w_fD (optional, default 0),
%                   sigma2_w_fA (optional, default 0)
%       constants - Physical constants with fields: gamma_N, Ts, k_B, T
%
%   Outputs:
%       ctrl - Struct with fields:
%           enable, lambda_c, gamma, Ts,
%           meas_noise_enable, meas_noise_std, meas_noise_seed,
%           a_pd, a_prd, a_cov, epsilon, g_cov, sigma2_deltaXT, k_B, T,
%           sigma2_w_fD, sigma2_w_fA

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
    ctrl.beta = config.beta;
    ctrl.lamdaF = config.lamdaF;
    ctrl.sigma2_noise = config.meas_noise_std.^2;  % 3x1 [um^2]
    ctrl.Pf_init_diag = config.Pf_init_diag;
    ctrl.Qz_diag_scaling = config.Qz_diag_scaling;
    ctrl.Rz_diag_scaling = config.Rz_diag_scaling;

    % eq17_7state v2 parameters (Wave 2D; Phase 5 §5.4)
    %   sigma2_w_fD: f_D random-walk innovation variance [pN^2/step]
    %                used in Q55,i = a_nom_axis^2 * sigma2_w_fD.
    if isfield(config, 'sigma2_w_fD')
        ctrl.sigma2_w_fD = config.sigma2_w_fD;
    else
        ctrl.sigma2_w_fD = 0;
    end

    % eq17_7state v2 parameters (Phase 5 §5.5)
    %   sigma2_w_fA: a_x random-walk innovation variance [(um/pN)^2/step]
    %                used in Q77_phase5_floor,i = a_nom_axis^2 * sigma2_w_fA
    %                (analogous to §5.4 σ²_w_fD applied to Q55).
    if isfield(config, 'sigma2_w_fA')
        ctrl.sigma2_w_fA = config.sigma2_w_fA;
    else
        ctrl.sigma2_w_fA = 0;
    end

    % ---------------------------------------------------------------
    % cdpmr_method: choose between Stage 11 Lyapunov (current production)
    % and the closed-form analytical formula for finite-alpha IIR LP
    % (Wave 4 verification test). C_np_eff_per_axis is ALWAYS taken from
    % the Lyapunov path (closed-form covers C_dpmr_eff only).
    %
    % Closed-form analytical formula (no per-axis dependence):
    %   alpha = a_pd, lambda = lambda_c
    %   one_minus_alpha = 1 - alpha
    %   denom_common    = 1 - one_minus_alpha * lambda
    %   term1 = 2 * one_minus_alpha * (1 - lambda) / denom_common
    %   term2 = 2 / ((2 - alpha) * (1 + lambda) * denom_common)
    %   C_dpmr_closed = one_minus_alpha^2 * (term1 + term2)
    % Sanity at alpha=0.05, lambda=0.7 -> C_dpmr_closed approx 3.161.
    % ---------------------------------------------------------------
    if isfield(config, 'cdpmr_method') && ~isempty(config.cdpmr_method)
        ctrl.cdpmr_method = config.cdpmr_method;
    else
        ctrl.cdpmr_method = 'closed_form';
    end

    % ---------------------------------------------------------------
    % Stage 11 Option I: Per-axis on-the-fly C_dpmr_eff / C_np_eff
    % Replaces paper Eq.22 closed-form (3.96, 1.18) with v2-effective
    % values via augmented Lyapunov. Same mechanism as qr branch.
    % Resolves Wave 4 v2 a_hat bias -31% → ~-1% target.
    %
    % Always run to populate C_np_eff_per_axis. C_dpmr_eff_per_axis
    % may be overwritten below by closed-form formula.
    % ---------------------------------------------------------------
    here = fileparts(mfilename('fullpath'));
    project_root = fileparts(fileparts(here));
    addpath(fullfile(project_root, 'test_script'));
    addpath(fullfile(project_root, 'model', 'wall_effect'));

    ctrl.C_dpmr_eff_per_axis = zeros(3, 1);
    ctrl.C_np_eff_per_axis   = zeros(3, 1);

    % Per-axis design-time mobility a_design = Ts / (gamma_N * c_axis).
    % x,y axes lie tangential to the wall (parallel correction c_para),
    % z axis is along the wall normal (perpendicular correction c_perp).
    % Using freespace a_design for all axes biases z's Stage-11 Lyapunov
    % calibration because c_perp > c_para near the wall (h=50 -> ~5% off).
    a_freespace = Ts / constants.gamma_N;
    h_bar_design = config.h_init / constants.R;
    [c_para_design, c_perp_design] = calc_correction_functions(h_bar_design);
    a_design_per_axis = [a_freespace / c_para_design; ...
                         a_freespace / c_para_design; ...
                         a_freespace / c_perp_design];

    C_dpmr_paper = 2 + 1 / (1 - config.lambda_c^2);
    C_n_paper    = 2 / (1 + config.lambda_c);

    opts_lyap = struct('f0', 0, 'verbose', false, 'Fe_form', 'eq19');

    try
        for ax = 1:3
            a_design_ax = a_design_per_axis(ax);
            sigma2_dXT_design_ax = 4 * constants.k_B * constants.T * a_design_ax;

            Q_kf_scale = [0; 0; 1; 0; 0; 0; 1e-15];

            sigma2_n_s_ax = config.meas_noise_std(ax)^2;
            xi_ax = (C_n_paper / C_dpmr_paper) * sigma2_n_s_ax ...
                    / (4 * constants.k_B * constants.T);
            IF_var_design = (1 + config.lambda_c^2) / (1 - config.lambda_c^2);
            R_22_design = config.a_cov * IF_var_design ...
                          * (a_design_ax + xi_ax)^2;
            R_kf_scale = [sigma2_n_s_ax / sigma2_dXT_design_ax; ...
                          R_22_design / sigma2_dXT_design_ax];

            [Cd, Cn, ~, ~, ~] = compute_7state_cdpmr_eff_v2( ...
                config.lambda_c, 2, config.a_pd, ...
                Q_kf_scale, R_kf_scale, opts_lyap);

            ctrl.C_dpmr_eff_per_axis(ax) = Cd;
            ctrl.C_np_eff_per_axis(ax)   = Cn;
        end
    catch ME
        warning('calc_ctrl_params:peraxis_lyap_failed', ...
                'Per-axis Lyapunov failed: %s. Falling back to paper closed form.', ...
                ME.message);
        ctrl.C_dpmr_eff_per_axis = C_dpmr_paper * ones(3, 1);
        ctrl.C_np_eff_per_axis   = C_n_paper   * ones(3, 1);
    end

    % ---------------------------------------------------------------
    % Closed-form override for C_dpmr_eff (Wave 4 verification).
    % Leaves C_np_eff_per_axis (Lyapunov) untouched.
    % ---------------------------------------------------------------
    if strcmp(ctrl.cdpmr_method, 'closed_form')
        alpha           = config.a_pd;
        lambda          = config.lambda_c;
        one_minus_alpha = 1 - alpha;
        denom_common    = 1 - one_minus_alpha * lambda;
        term1 = 2 * one_minus_alpha * (1 - lambda) / denom_common;
        term2 = 2 / ((2 - alpha) * (1 + lambda) * denom_common);
        C_dpmr_closed = one_minus_alpha^2 * (term1 + term2);

        ctrl.C_dpmr_eff_per_axis = C_dpmr_closed * ones(3, 1);
    end

    % ---------------------------------------------------------------
    % X2a: Per-axis empirical IF_eff calibration (Phase 9 Stage I)
    %
    % The Phase 1 §10 closed-form ρ_δx assumes MA(2) ε with geometric
    % tail λ_c^τ for τ ≥ 3. Empirical ρ_δx in production conditions
    % oscillates negative around lag 7-10; closed-form IF_eff(s) over-
    % predicts variance by ~9% at a_cov=0.05, contributing to a V1
    % ratio = 0.87 vs target 1.00. Using empirical IF_eff_emp(s) from
    % stored ACF makes V1 ratio collapse to 1.00 ± 0.05 across all
    % α and σ²_n conditions.
    %
    % Same architectural pattern as Stage 11 Lyapunov C_dpmr_eff
    % calibration above: compute once at init, store in ctrl, plumb
    % into ctrl_const via run_pure_simulation -> build_eq17_constants.
    %
    % If the calibration file is absent, leave the field empty so
    % build_eq17_constants falls back to the closed-form scalar IF_eff.
    % ---------------------------------------------------------------
    ctrl.IF_eff_calibrated_per_axis = [];

    if isfield(config, 'IF_eff_calibration_file') && ...
            ~isempty(config.IF_eff_calibration_file)
        calibration_file = config.IF_eff_calibration_file;
    else
        calibration_file = fullfile(project_root, 'reference', ...
                                    'eq17_analysis', ...
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
