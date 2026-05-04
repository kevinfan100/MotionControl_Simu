function ctrl_const = build_eq17_constants(opts)
%BUILD_EQ17_CONSTANTS Offline scalar-constants builder for Eq.17 7-state EKF
%
%   ctrl_const = build_eq17_constants(opts)
%
%   Computes time-invariant scalar constants for the eq17_7state controller.
%   Designed to be called once at simulation init; outputs are passed
%   downstream to the EKF / variance estimators / R-matrix builder.
%
%   Reference: design.md §9.1-9.2 (closed-loop variance shaping for Eq.17 KF).
%
%   --------- Inputs (struct opts) ---------
%       opts.lambda_c     - closed-loop pole, in (0,1).        [scalar, required]
%       opts.option       - 'A_MA2_full' (default) | 'B_AR1_approx'
%       opts.sigma2_n_s   - per-axis sensor noise variance     [3x1, μm^2, required]
%       opts.kBT          - thermal energy in user's existing  [scalar, required]
%                           consistent unit system (e.g., μm-based)
%       opts.t_warmup_kf  - KF warmup time [sec]               (default 0.2)
%       opts.h_bar_safe   - safe h_bar threshold               (default 1.5)
%       opts.d            - measurement delay (steps)          (default 2)
%       opts.a_cov        - IIR variance estimator weight      (default 0.05)
%       opts.a_pd         - IIR LP weight for δp_md mean       (default = a_cov)
%       opts.sigma2_w_fD  - f_D random-walk innovation var.    (default 0)
%                           [pN^2/step], used in Q55,i = a_nom_axis^2 * sigma2_w_fD
%       opts.sigma2_w_fA  - a_x random-walk innovation var.    (default 0)
%                           [(um/pN)^2/step], used in Q77_phase5_floor,i =
%                           a_nom_axis^2 * sigma2_w_fA (Phase 5 §5.5)
%
%   --------- Outputs (struct ctrl_const) ---------
%       lambda_c          - copied from opts
%       C_dpmr            - 2 + 1/(1 - lambda_c^2)
%       C_n               - 2/(1 + lambda_c)
%       option            - copied
%       IF_var            - autocorrelation inflation factor
%                           (formula depends on option, see §9.2)
%       xi_per_axis       - 3x1: (C_n/C_dpmr) * sigma2_n_s ./ (4*kBT)
%       delay_R2_factor   - sum_{j=1..d} (d-j+1)^2
%       t_warmup_kf       - copied
%       h_bar_safe        - copied
%       d                 - copied
%       a_cov             - copied
%       a_pd              - copied (defaults to a_cov for backward compat)
%       sigma2_w_fD       - copied (defaults to 0)
%       sigma2_w_fA       - copied (defaults to 0)
%       meta              - struct with derivation reference and option tag
%
%   --------- IF_var formula ---------
%   Option A (MA(2) full, recommended):
%       ε of Eq.19 form has MA(2) cross-step correlation. Closed-form
%       autocorrelations of ε and δx are computed analytically; tail of
%       δx autocorrelation (τ ≥ 3) is summed as a geometric series.
%
%   Option B (AR(1) approximation):
%       IF_var = (1 + lambda_c^2) / (1 - lambda_c^2)
%
%   --------- Validation references (lambda_c = 0.7) ---------
%       C_dpmr           ≈ 3.9608
%       C_n              ≈ 1.1765
%       IF_var (Opt A)   ≈ 4.224
%       IF_var (Opt B)   ≈ 2.922
%       delay_R2_factor  = 5    (when d = 2)
%
%   See also: motion_control_law_7state, calc_ctrl_params

    % ------------------------------------------------------------
    % Default-fill missing fields
    % ------------------------------------------------------------
    if nargin < 1 || ~isstruct(opts)
        error('build_eq17_constants:invalidInput', ...
              'opts must be a struct.');
    end

    if ~isfield(opts, 'option') || isempty(opts.option)
        opts.option = 'A_MA2_full';
    end
    if ~isfield(opts, 't_warmup_kf') || isempty(opts.t_warmup_kf)
        opts.t_warmup_kf = 0.2;
    end
    if ~isfield(opts, 'h_bar_safe') || isempty(opts.h_bar_safe)
        opts.h_bar_safe = 1.5;
    end
    if ~isfield(opts, 'd') || isempty(opts.d)
        opts.d = 2;
    end
    if ~isfield(opts, 'a_cov') || isempty(opts.a_cov)
        opts.a_cov = 0.05;
    end
    if ~isfield(opts, 'a_pd') || isempty(opts.a_pd)
        opts.a_pd = opts.a_cov;          % default same as a_cov for backward compat
    end
    if ~isfield(opts, 'sigma2_w_fD') || isempty(opts.sigma2_w_fD)
        opts.sigma2_w_fD = 0;            % Phase 5 §5.4 baseline 0
    end
    if ~isfield(opts, 'sigma2_w_fA') || isempty(opts.sigma2_w_fA)
        opts.sigma2_w_fA = 0;            % Phase 5 §5.5 baseline 0
    end
    % Optional test-only overrides (Wave 4 motion ramp). Defaults are empty
    % so the controller falls back to the existing amp*omega calculation.
    if ~isfield(opts, 'force_Q77_zero')
        opts.force_Q77_zero = false;
    end
    if ~isfield(opts, 'h_dot_max_override')
        opts.h_dot_max_override = [];
    end
    if ~isfield(opts, 'h_ddot_max_override')
        opts.h_ddot_max_override = [];
    end
    if ~isfield(opts, 'Pf_init_slot7')
        opts.Pf_init_slot7 = [];
    end

    % Stage 11 Option I: per-axis effective C_dpmr_eff / C_np_eff
    % If not provided, defaults to paper closed-form replicated per-axis.
    if ~isfield(opts, 'C_dpmr_eff_per_axis') || isempty(opts.C_dpmr_eff_per_axis)
        opts.C_dpmr_eff_per_axis = (2 + 1/(1 - opts.lambda_c^2)) * ones(3, 1);
    end
    if ~isfield(opts, 'C_np_eff_per_axis') || isempty(opts.C_np_eff_per_axis)
        opts.C_np_eff_per_axis = (2 / (1 + opts.lambda_c)) * ones(3, 1);
    end

    % X2a: per-axis empirical IF_eff calibration (Phase 9 Stage I).
    % If absent or empty, the closed-form scalar IF_eff (computed below)
    % is replicated to a 3x1 vector for downstream consumers.
    if ~isfield(opts, 'IF_eff_calibrated') || isempty(opts.IF_eff_calibrated)
        opts.IF_eff_calibrated = [];
    else
        opts.IF_eff_calibrated = opts.IF_eff_calibrated(:);
        if numel(opts.IF_eff_calibrated) ~= 3 || ...
                any(~isfinite(opts.IF_eff_calibrated)) || ...
                any(opts.IF_eff_calibrated <= 0)
            error('build_eq17_constants:invalidIFeffCal', ...
                  ['opts.IF_eff_calibrated must be a 3-element positive ' ...
                   'finite vector; got %s.'], mat2str(opts.IF_eff_calibrated));
        end
    end

    % ------------------------------------------------------------
    % Required-field presence checks
    % ------------------------------------------------------------
    required_fields = {'lambda_c', 'sigma2_n_s', 'kBT'};
    for k = 1:numel(required_fields)
        f = required_fields{k};
        if ~isfield(opts, f) || isempty(opts.(f))
            error('build_eq17_constants:missingInput', ...
                  'opts.%s is required.', f);
        end
    end

    % ------------------------------------------------------------
    % Input validation
    % ------------------------------------------------------------
    lambda_c = opts.lambda_c;
    if ~isscalar(lambda_c) || ~isnumeric(lambda_c) || ...
            ~isfinite(lambda_c) || lambda_c <= 0 || lambda_c >= 1
        error('build_eq17_constants:invalidLambda', ...
              'lambda_c must be a finite scalar in (0,1); got %g.', lambda_c);
    end

    sigma2_n_s = opts.sigma2_n_s(:);  % force column
    if numel(sigma2_n_s) ~= 3 || ~isnumeric(sigma2_n_s) || ...
            any(~isfinite(sigma2_n_s)) || any(sigma2_n_s <= 0)
        error('build_eq17_constants:invalidSigma2', ...
              'sigma2_n_s must be a 3-element positive finite vector.');
    end

    kBT = opts.kBT;
    if ~isscalar(kBT) || ~isnumeric(kBT) || ~isfinite(kBT) || kBT <= 0
        error('build_eq17_constants:invalidKBT', ...
              'kBT must be a positive finite scalar; got %g.', kBT);
    end

    valid_options = {'A_MA2_full', 'B_AR1_approx'};
    if ~ischar(opts.option) && ~(isstring(opts.option) && isscalar(opts.option))
        error('build_eq17_constants:invalidOption', ...
              'opts.option must be a string.');
    end
    option_str = char(opts.option);
    if ~any(strcmp(option_str, valid_options))
        error('build_eq17_constants:invalidOption', ...
              'opts.option must be one of {''A_MA2_full'', ''B_AR1_approx''}; got ''%s''.', ...
              option_str);
    end

    d = opts.d;
    if ~isscalar(d) || ~isnumeric(d) || d ~= round(d) || d < 0
        error('build_eq17_constants:invalidDelay', ...
              'opts.d must be a non-negative integer; got %g.', d);
    end

    % ------------------------------------------------------------
    % Closed-loop shape factors (design.md §9.1)
    % ------------------------------------------------------------
    % C_dpmr: closed-loop variance shape factor from p_d - r impulse response
    C_dpmr = 2 + 1 / (1 - lambda_c^2);

    % C_n: sensor-noise shape factor from (1 - lambda_c)/(1 - lambda_c z^-1)
    C_n = 2 / (1 + lambda_c);

    % ------------------------------------------------------------
    % IF_var (autocorrelation inflation factor, design.md §9.2)
    % ------------------------------------------------------------
    switch option_str
        case 'A_MA2_full'
            % ε MA(2) autocorrelation (thermal-only contribution dominates)
            denom_e = 1 + 2 * (1 - lambda_c)^2;
            rho_e_1 = (1 - lambda_c) * (2 - lambda_c) / denom_e;
            rho_e_2 = (1 - lambda_c) / denom_e;

            % Var(δx) / σ²_ε ratio
            Var_dx_over_sig_e = (1 + 2*lambda_c*rho_e_1 + 2*lambda_c^2*rho_e_2) ...
                                / (1 - lambda_c^2);

            % δx autocorrelation (closed form)
            % ρ_δx(1) = λ_c + (σ²_ε/Var(δx)) * (ρ_ε(1) + λ_c·ρ_ε(2))
            % ρ_δx(2) = λ_c·ρ_δx(1) + (σ²_ε/Var(δx)) * ρ_ε(2)
            % ρ_δx(τ≥3) = λ_c·ρ_δx(τ−1)
            inv_var_ratio = 1 / Var_dx_over_sig_e;   % = σ²_ε / Var(δx)
            rho_dx_1 = lambda_c + inv_var_ratio * (rho_e_1 + lambda_c*rho_e_2);
            rho_dx_2 = lambda_c * rho_dx_1 + inv_var_ratio * rho_e_2;

            % IF_var = 1 + 2·Σ_{τ≥1} ρ²_δx(τ)
            %        = 1 + 2·[ρ²(1) + ρ²(2)/(1-λ_c²)]
            %   (geometric tail summed for τ ≥ 3)
            IF_var = 1 + 2 * (rho_dx_1^2 + rho_dx_2^2 / (1 - lambda_c^2));

        case 'B_AR1_approx'
            % AR(1) approximation: IF = (1 + λ²)/(1 - λ²)
            IF_var = (1 + lambda_c^2) / (1 - lambda_c^2);
            % For Option B IF_eff(s) below: rho_dx_k = lambda_c^k → ρ²_k = λ_c^(2k)
            rho_dx_1 = lambda_c;
            rho_dx_2 = lambda_c^2;
    end

    % ------------------------------------------------------------
    % Phase 9 fix: s-weighted IF_eff(s) for finite-α R(2,2) intrinsic
    %   Var(σ̂²) = (2·a_cov / (2-a_cov)) · IF_eff(s) · (σ²_δxr)²
    %   IF_eff(s) = 1 + 2·Σ_{τ≥1} ρ²_δx(τ)·s^τ      (s = 1-a_cov)
    %
    % Replaces the small-α approximation `a_cov · IF_var` (design.md:880-897
    % flagged ~9% over-prediction at a_cov=0.05, ~17% at a_cov=0.10).
    % The factor of 2 arises from Var(δx_r²) = 2·σ_dxr⁴ (Gaussian Isserlis).
    % Phase 9 Wave 1 (commit f618f37) Path C empirically confirmed.
    % ------------------------------------------------------------
    s_ewma = 1 - opts.a_cov;
    switch option_str
        case 'A_MA2_full'
            % Closed form: ρ²(1)·s + ρ²(2)·s²·Σ_{j≥0}(λ²·s)^j
            %            = ρ²(1)·s + ρ²(2)·s² / (1 - λ²·s)
            IF_eff = 1 + 2 * (rho_dx_1^2 * s_ewma ...
                            + rho_dx_2^2 * s_ewma^2 / (1 - lambda_c^2 * s_ewma));
        case 'B_AR1_approx'
            % Geometric ρ²(τ) = λ²·^τ → IF_eff(s) = 1 + 2·(λ²·s)/(1-λ²·s)
            IF_eff = 1 + 2 * (lambda_c^2 * s_ewma) / (1 - lambda_c^2 * s_ewma);
    end
    R22_prefactor = 2 * opts.a_cov / (2 - opts.a_cov);

    % ------------------------------------------------------------
    % Per-axis ξ scaling (sensor-noise → effective process-noise, §9.2)
    % ------------------------------------------------------------
    xi_per_axis = (C_n / C_dpmr) * sigma2_n_s / (4 * kBT);  % 3x1

    % ------------------------------------------------------------
    % delay_R2_factor: sum_{j=1..d} (d - j + 1)^2 = 1^2 + 2^2 + ... + d^2
    % ------------------------------------------------------------
    if d == 0
        delay_R2_factor = 0;
    else
        j_idx = 1:d;
        delay_R2_factor = sum((d - j_idx + 1).^2);
    end

    % ------------------------------------------------------------
    % Pack outputs
    % ------------------------------------------------------------
    % X2a: per-axis IF_eff. If a calibration vector is provided by
    % calc_ctrl_params, use it; else replicate the closed-form scalar
    % to all 3 axes for downstream consumption.
    if ~isempty(opts.IF_eff_calibrated)
        IF_eff_per_axis = opts.IF_eff_calibrated;            % 3x1
    else
        IF_eff_per_axis = IF_eff * ones(3, 1);               % 3x1 fallback
    end

    ctrl_const.lambda_c          = lambda_c;
    ctrl_const.C_dpmr            = C_dpmr;
    ctrl_const.C_n               = C_n;
    ctrl_const.option            = option_str;
    ctrl_const.IF_var            = IF_var;
    ctrl_const.IF_eff            = IF_eff;        % Phase 9 fix: s-weighted (closed form)
    ctrl_const.IF_eff_per_axis   = IF_eff_per_axis; % X2a: 3x1 (calibrated if available)
    ctrl_const.IF_eff_calibrated = opts.IF_eff_calibrated; % 3x1 or [] when absent
    ctrl_const.R22_prefactor     = R22_prefactor; % Phase 9 fix: 2*a_cov/(2-a_cov)
    ctrl_const.xi_per_axis     = xi_per_axis;
    ctrl_const.delay_R2_factor = delay_R2_factor;
    ctrl_const.t_warmup_kf     = opts.t_warmup_kf;
    ctrl_const.h_bar_safe      = opts.h_bar_safe;
    ctrl_const.d               = d;
    ctrl_const.a_cov           = opts.a_cov;
    ctrl_const.a_pd            = opts.a_pd;
    ctrl_const.sigma2_w_fD     = opts.sigma2_w_fD;
    ctrl_const.sigma2_w_fA     = opts.sigma2_w_fA;

    % Wave 4 motion-ramp test overrides
    ctrl_const.force_Q77_zero      = opts.force_Q77_zero;
    ctrl_const.h_dot_max_override  = opts.h_dot_max_override;
    ctrl_const.h_ddot_max_override = opts.h_ddot_max_override;
    ctrl_const.Pf_init_slot7       = opts.Pf_init_slot7;

    % Stage 11 Option I: per-axis effective C_dpmr_eff / C_np_eff
    ctrl_const.C_dpmr_eff      = opts.C_dpmr_eff_per_axis(:);    % 3x1
    ctrl_const.C_np_eff        = opts.C_np_eff_per_axis(:);      % 3x1

    ctrl_const.meta = struct( ...
        'derivation', 'design.md §9.1-9.2', ...
        'option',     option_str, ...
        'description', 'Offline scalar constants for Eq.17 7-state EKF');
end
