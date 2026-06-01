function [s2_dx, s2_e_xD, s2_e_a, diag_out] = predict_closed_loop_var_eq17_v2(opts)
%PREDICT_CLOSED_LOOP_VAR_EQ17_V2  Closed-form variance prediction for v2 EKF (Eq.17, 5-state)
%
%   [s2_dx, s2_e_xD, s2_e_a, diag_out] = predict_closed_loop_var_eq17_v2(opts)
%
%   Implements modular block-triangular Lyapunov per Phase 7 §6 of
%   reference/eq17_analysis/phase7_lyapunov_bench.md.
%
%   Step 1: Estimation-error sub-Lyapunov (chi-sq chain approximation per
%           qr Level 6) for sigma2_e_a and sigma2_e_xD.
%   Step 2: Truth-driven Lyapunov for sigma2_dx (paper Eq.22 form with
%           estimation-error corrections).
%
%   Note on rho_a (autocorrelation amplification):
%       Phase 7 §4.3 used rho_a = 4.7 for thermal-dominated near-wall.
%       For sensor-dominated regimes (e.g. h=50 z-axis), rho_a may be
%       smaller. This function exposes rho_a as an input (default 4.7);
%       Phase 8 sensitivity sweeps should refine per regime.
%
% Inputs (struct opts):
%   .lambda_c       Closed-loop pole [scalar in (0,1)] (e.g. 0.7)
%   .a_x_axis       1x3 motion gain per axis [um/pN]
%   .h_bar          Wall-normal distance / R [-] (informational; not used in v2 closed form)
%   .sigma2_n_s     1x3 sensor noise variance per axis [um^2]
%   .k_B            Boltzmann constant [pN*um/K]
%   .T              Temperature [K]
%   .Q77_axis       1x3 Q(7,7) per axis (delta-a process noise) [scaled per Phase 5]
%   .R_axis         1x3 R(2,2) per axis (a_xm measurement noise) [Phase 6]
%   .a_cov          IIR coefficient for delta_p_mr^2 EMA [-] (e.g. 0.05)
%   .a_pd           IIR coefficient for delta_p_md LP [-] (e.g. 0.05)
%   .C_dpmr         Phase 2 thermal coefficient = 2 + 1/(1-lambda_c^2)
%   .C_n            Phase 2 sensor coefficient = 2/(1+lambda_c)
%   .IF_var         Phase 2 IF_var (Option A MA(2)) [-] (e.g. 4.224)
%   .sigma2_w_fD    f_D random-walk innovation variance [pN^2] (often 0 baseline)
%   .scenario       'positioning' or 'motion'
%   .traj_amplitude Motion only: A_h [um]
%   .traj_freq      Motion only: f [Hz]
%
% Optional fields:
%   .rho_a          scalar autocorrelation amplification [default 4.7]
%   .rho_a_axis     1x3 per-axis override (takes precedence if present)
%   .wall_term      1x3 additional CV^2(a_hat) wall-sensitivity contribution
%                   (e.g. (sens*std(p_m_z)/R)^2 per Phase 7 §4.3.1) [default zeros]
%   .Ts             sampling period [sec] (motion only) [default 1/1600]
%
% Outputs:
%   s2_dx       1x3 predicted sigma^2_dx per axis [um^2]
%   s2_e_xD     1x3 predicted sigma^2_e_xD per axis [um^2]
%   s2_e_a      1x3 predicted sigma^2_e_a per axis [(um/pN)^2]
%   diag_out    Diagnostics struct with intermediate quantities:
%                   sigma2_dXT, chi_sq, rho_a_used, L_eff, CV2_a,
%                   s2_dx_paper_only, correction_xD, correction_a,
%                   E_bracket2, scenario
%
% Reference: Phase 7 §6 spec (reference/eq17_analysis/phase7_lyapunov_bench.md)

    %% --- Argument validation ----------------------------------------------
    if ~isstruct(opts)
        error('predict_closed_loop_var_eq17_v2:badInput', ...
              'opts must be a struct');
    end

    required = {'lambda_c','a_x_axis','sigma2_n_s','k_B','T', ...
                'Q77_axis','R_axis','a_cov','a_pd','C_dpmr','C_n', ...
                'IF_var','sigma2_w_fD','scenario'};
    for ii = 1:numel(required)
        if ~isfield(opts, required{ii})
            error('predict_closed_loop_var_eq17_v2:missingField', ...
                  'opts missing required field: %s', required{ii});
        end
    end

    lambda_c = opts.lambda_c;
    if lambda_c < 0 || lambda_c >= 1
        error('predict_closed_loop_var_eq17_v2:badLambda', ...
              'lambda_c must be in [0,1); got %.4g', lambda_c);
    end

    if numel(opts.a_x_axis) ~= 3 || numel(opts.sigma2_n_s) ~= 3 || ...
       numel(opts.Q77_axis) ~= 3 || numel(opts.R_axis) ~= 3
        error('predict_closed_loop_var_eq17_v2:badAxisDim', ...
              'a_x_axis, sigma2_n_s, Q77_axis, R_axis must be 1x3');
    end

    a_x_axis    = opts.a_x_axis(:).';     % force 1x3
    sigma2_n_s  = opts.sigma2_n_s(:).';
    Q77_axis    = opts.Q77_axis(:).';
    R_axis      = opts.R_axis(:).';

    if any(a_x_axis <= 0)
        error('predict_closed_loop_var_eq17_v2:badAxAxis', ...
              'a_x_axis entries must be > 0');
    end

    %% --- Optional inputs --------------------------------------------------
    if isfield(opts, 'rho_a_axis') && ~isempty(opts.rho_a_axis)
        rho_a_axis = opts.rho_a_axis(:).';
        if numel(rho_a_axis) ~= 3
            error('predict_closed_loop_var_eq17_v2:badRhoAxis', ...
                  'rho_a_axis must be 1x3');
        end
    elseif isfield(opts, 'rho_a') && ~isempty(opts.rho_a)
        rho_a_axis = repmat(opts.rho_a, 1, 3);
    else
        rho_a_axis = 4.7 * ones(1, 3);    % Phase 7 §4.3 default
    end

    if isfield(opts, 'wall_term') && ~isempty(opts.wall_term)
        wall_term_axis = opts.wall_term(:).';
        if numel(wall_term_axis) ~= 3
            error('predict_closed_loop_var_eq17_v2:badWallTerm', ...
                  'wall_term must be 1x3');
        end
    else
        wall_term_axis = zeros(1, 3);
    end

    if isfield(opts, 'Ts') && ~isempty(opts.Ts)
        Ts = opts.Ts;
    else
        Ts = 1 / 1600;                    % project default
    end

    %% --- Step 1: Estimation-error variances (chi-sq chain approximation) -
    %  Per Phase 7 §4.3.1:
    %    chi_sq  = 2*a_cov / (2 - a_cov)
    %    L_eff   = sqrt(Q77 / R(2,2))
    %    CV^2(a) = chi_sq * rho_a * L_eff/(L_eff + a_cov) + wall_term
    %    sigma^2_e_a = CV^2(a) * a_x^2
    chi_sq = 2 * opts.a_cov / (2 - opts.a_cov);

    s2_e_a  = zeros(1, 3);
    s2_e_xD = zeros(1, 3);
    L_eff   = zeros(1, 3);
    CV2_a   = zeros(1, 3);

    eps_safe = eps;   % numerical safety floor

    for ii = 1:3
        % L_eff = sqrt(Q77 / R(2,2)). Guard divide-by-zero on R.
        if R_axis(ii) > 0
            L_eff(ii) = sqrt(max(Q77_axis(ii), 0) / R_axis(ii));
        else
            L_eff(ii) = 0;   % Q77/0 undefined; treat as no innovation gain
        end

        % CV^2(a_hat) = chi_sq * rho_a * L_eff/(L_eff + a_cov) + wall_term
        denom_chi = L_eff(ii) + opts.a_cov;
        if denom_chi > eps_safe
            CV2_a(ii) = chi_sq * rho_a_axis(ii) * L_eff(ii) / denom_chi ...
                       + wall_term_axis(ii);
        else
            CV2_a(ii) = wall_term_axis(ii);   % L_eff=0 collapses chi-sq term
        end

        s2_e_a(ii) = CV2_a(ii) * a_x_axis(ii)^2;

        % sigma^2_e_xD: small for sigma2_w_fD = 0, scaled with sqrt(Q55*R) for >0
        if opts.sigma2_w_fD > 0
            % SISO Riccati approximation per Phase 7 §4.3.2:
            %   Q55 = a_nom^2 * sigma2_w_fD ~= a_x^2 * sigma2_w_fD (cosmetic)
            %   sigma^2_e_xD ~ sqrt(Q55 * R(2,2))
            Q55_i = a_x_axis(ii)^2 * opts.sigma2_w_fD;
            s2_e_xD(ii) = sqrt(max(Q55_i, 0) * max(R_axis(ii), 0));
        else
            s2_e_xD(ii) = 0;   % effectively zero baseline
        end
    end

    %% --- Step 2: Truth-driven Lyapunov for sigma^2_dx --------------------
    %  Per Phase 7 §5.2:
    %    sigma^2_dx * (1 - lambda_c^2)
    %      = C_dpmr * 4kBT*a_x + C_n * sigma^2_n_s          (paper Eq.22)
    %      + sigma^2_e_xD                                   (Step 1)
    %      + (E[bracket^2] / a_x^2) * sigma^2_e_a           (Step 1)
    one_minus_lc2  = 1 - lambda_c^2;
    if one_minus_lc2 <= 0
        error('predict_closed_loop_var_eq17_v2:lambdaBoundary', ...
              'lambda_c=%.4g gives 1-lambda_c^2 <= 0', lambda_c);
    end

    sigma2_dXT       = 4 * opts.k_B * opts.T * a_x_axis;       % 1x3 [um^2]
    s2_dx_paper      = (opts.C_dpmr * sigma2_dXT + ...
                        opts.C_n   * sigma2_n_s) / one_minus_lc2;

    % E[bracket^2] depends on scenario
    switch lower(opts.scenario)
        case 'positioning'
            % E[bracket^2] ~ (1-lambda_c)^2 * sigma^2_dx_m
            %             ~ (1-lambda_c)^2 * (sigma^2_dx + sigma^2_n_s)
            % Use s2_dx_paper as best proxy for sigma^2_dx (paper part dominates)
            E_bracket2 = (1 - lambda_c)^2 * (s2_dx_paper + sigma2_n_s);

        case 'motion'
            % Rough estimate per Phase 7 §6.1 / §8.3:
            %   E[bracket^2] ~ (A_h * omega * Ts)^2 / 2  (feedforward)
            if ~isfield(opts, 'traj_amplitude') || ~isfield(opts, 'traj_freq')
                error('predict_closed_loop_var_eq17_v2:motionMissingTraj', ...
                      'scenario=motion requires opts.traj_amplitude and opts.traj_freq');
            end
            A_h   = opts.traj_amplitude;
            omega = 2 * pi * opts.traj_freq;
            E_bracket2 = repmat((A_h * omega * Ts)^2 / 2, 1, 3);

        otherwise
            error('predict_closed_loop_var_eq17_v2:badScenario', ...
                  'scenario must be ''positioning'' or ''motion''; got %s', ...
                  opts.scenario);
    end

    % Estimation-error corrections (per axis)
    a_x_sq             = a_x_axis.^2;
    correction_xD      = s2_e_xD ./ one_minus_lc2;
    correction_a       = (E_bracket2 ./ a_x_sq) .* s2_e_a / one_minus_lc2;

    s2_dx = s2_dx_paper + correction_xD + correction_a;

    %% --- Diagnostics output ----------------------------------------------
    if nargout >= 4
        diag_out = struct();
        diag_out.sigma2_dXT       = sigma2_dXT;
        diag_out.chi_sq           = chi_sq;
        diag_out.rho_a_used       = rho_a_axis;
        diag_out.wall_term        = wall_term_axis;
        diag_out.L_eff            = L_eff;
        diag_out.CV2_a            = CV2_a;
        diag_out.s2_dx_paper_only = s2_dx_paper;
        diag_out.correction_xD    = correction_xD;
        diag_out.correction_a     = correction_a;
        diag_out.E_bracket2       = E_bracket2;
        diag_out.one_minus_lc2    = one_minus_lc2;
        diag_out.scenario         = opts.scenario;
    end
end
