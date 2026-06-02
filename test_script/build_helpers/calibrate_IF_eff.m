function IF_eff_calibrated = calibrate_IF_eff(a_cov, lambda_c, calibration_acf_file)
%CALIBRATE_IF_EFF  Per-axis empirical IF_eff(s) from stored ρ_δx ACF.
%
%   IF_eff_calibrated = calibrate_IF_eff(a_cov, lambda_c, calibration_acf_file)
%
%   Computes the s-weighted autocorrelation inflation factor
%
%       IF_eff_emp(s) = 1 + 2 * sum_{tau=1..50} rho_emp(tau)^2 * s^tau,
%       where s = 1 - a_cov,
%
%   per axis (3x1) from a calibration .mat file containing the empirical
%   ACF v1_acf_mean[51, 3] of the residual δx_r.
%
%   Background:
%     The Phase 1 §10 closed-form ρ_δx assumes MA(2) ε with geometric tail
%     λ_c^τ for τ ≥ 3. Phase 9 Stage I diagnosis showed empirical ρ_δx
%     deviates (oscillates negative around lag 7-10) and the closed-form
%     IF_eff over-predicts variance by ~9% at α=0.05. Using empirical
%     IF_eff_emp instead drives V1 ratio to 1.00 ± 0.05.
%
%   Inputs:
%       a_cov                 - EWMA weight (scalar in (0,1))
%       lambda_c              - closed-loop pole (scalar; kept for signature
%                               compatibility, not used in empirical formula)
%       calibration_acf_file  - path to .mat file with field v1_acf_mean[51,3]
%
%   Output:
%       IF_eff_calibrated     - 3x1 vector of empirical IF_eff(s=1-a_cov)
%
%   See also: build_eq17_constants, calc_ctrl_params

    if nargin < 3 || isempty(calibration_acf_file)
        error('calibrate_IF_eff:missingFile', ...
              'calibration_acf_file is required.');
    end
    if ~isfile(calibration_acf_file)
        error('calibrate_IF_eff:fileNotFound', ...
              'Calibration ACF file not found: %s', calibration_acf_file);
    end
    if ~isscalar(a_cov) || ~isfinite(a_cov) || a_cov <= 0 || a_cov >= 1
        error('calibrate_IF_eff:invalidACov', ...
              'a_cov must be a scalar in (0,1); got %g.', a_cov);
    end

    data = load(calibration_acf_file);
    if ~isfield(data, 'v1_acf_mean')
        error('calibrate_IF_eff:missingField', ...
              'Calibration file %s missing field v1_acf_mean.', ...
              calibration_acf_file);
    end
    rho = data.v1_acf_mean;                    % expect [51, 3]
    sz  = size(rho);
    if numel(sz) ~= 2 || sz(1) < 51 || sz(2) ~= 3
        error('calibrate_IF_eff:badShape', ...
              'v1_acf_mean must be at least 51x3; got %s.', mat2str(sz));
    end

    s = 1 - a_cov;
    s_pow = s .^ (1:50);                        % 1x50 row
    s_pow = s_pow(:);                           % 50x1 column

    IF_eff_calibrated = zeros(3, 1);
    for ax = 1:3
        rho_ax = rho(2:51, ax);                 % 50x1: lags 1..50
        IF_eff_calibrated(ax) = 1 + 2 * sum((rho_ax .^ 2) .* s_pow);
    end

    % Suppress unused-input lint (kept in signature for symmetry with
    % closed-form callers which need lambda_c).
    %#ok<*INUSL>
end
