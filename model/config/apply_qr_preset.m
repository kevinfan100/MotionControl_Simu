function config = apply_qr_preset(config)
%APPLY_QR_PRESET Apply Q/R/Pf_init/a_cov values from config.qr_preset
%
%   config = apply_qr_preset(config)
%
%   Sets Qz_diag_scaling, Rz_diag_scaling, Pf_init_diag, a_cov on config
%   based on config.qr_preset string. Use 'frozen_correct' for positioning,
%   'empirical' for dynamic / motion scenarios. See Sessions 4-7 docs in
%   reference/for_test/.
%
%   Presets:
%     'frozen_correct' (default for positioning) — Sessions 4-7
%       Paper-level a_hat: bias < 1%, std 1-5%. Requires P5+P6 controller
%       (IIR pre-fill + warmup_count=2). Frozen Q makes recovery from spikes
%       slow → not suitable for trajectory motion where a changes.
%     'empirical' (default for motion / oscillation)
%       a_hat std ~20% but tracking unaffected. Robust across scenarios.
%       Historical default before Q/R derivation work.
%     'beta'
%       Derived backward-diff (qr_theoretical_values.md, 2026-04-17).
%       Drifts in simulation (a_hat std 22-39%) — not recommended.

    if ~isfield(config, 'qr_preset') || isempty(config.qr_preset)
        error('apply_qr_preset:missing', ...
              'config.qr_preset is required. Use frozen_correct/empirical/beta.');
    end

    switch config.qr_preset
        case 'frozen_correct'
            config.Qz_diag_scaling = [0; 0; 1; 0; 0; 1e-8; 1e-8];
            config.Rz_diag_scaling = [0.397; 1.0];
            config.Pf_init_diag    = [0; 0; 1e-4; 1e-4; 0; 1e-5; 0];
            config.a_cov           = 0.005;
        case 'empirical'
            config.Qz_diag_scaling = [0; 0; 1e4; 0.1; 0; 1e-4; 0];
            config.Rz_diag_scaling = [1e-2; 1.0];
            config.Pf_init_diag    = [0; 0; 1e-4; 1e-4; 0; 10*(0.0147)^2; 0];
            config.a_cov           = 0.05;
        case 'beta'
            config.Qz_diag_scaling = [0; 0; 1; 0; 0; 1.3444e-11; 1.3444e-11];
            config.Rz_diag_scaling = [0.3970; 1.7185];
            config.Pf_init_diag    = [0; 0; 1e-4; 1e-4; 0; 10*(0.0147)^2; 0];
            config.a_cov           = 0.05;
        otherwise
            error('apply_qr_preset:bad_preset', ...
                  'Unknown qr_preset "%s". Use frozen_correct / empirical / beta.', ...
                  config.qr_preset);
    end
end
