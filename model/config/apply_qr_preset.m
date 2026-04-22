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

    % Qz_diag_scaling layout (2026-04-22): 9x1 = [Q1; Q2; Q3; Q4; Q5; Q6; Q7_x; Q7_y; Q7_z]
    %   Q(1..6) shared across 3 axes; Q(7,7) per-axis for del_a random walk
    %   Legacy Q(7,7) = Q_last shared → replicated to all 3 in per-axis slots
    % Rz_diag_scaling layout: [R_pos_x; R_pos_y; R_pos_z; R_gain_x; R_gain_y; R_gain_z]
    switch config.qr_preset
        case 'frozen_correct'
            config.Qz_diag_scaling = [0; 0; 1; 0; 0; 1e-8; 1e-8; 1e-8; 1e-8];
            config.Rz_diag_scaling = [0.397; 0.397; 0.397; 1.0; 1.0; 1.0];
            config.Pf_init_diag    = [0; 0; 1e-4; 1e-4; 0; 1e-5; 0];
            config.a_cov           = 0.005;
        case 'frozen_correct_peraxisR11'
            config.Qz_diag_scaling = [0; 0; 1; 0; 0; 1e-8; 1e-8; 1e-8; 1e-8];
            config.Rz_diag_scaling = [1.53e-3; 1.29e-5; 4.35e-2; 1.0; 1.0; 1.0];
            config.Pf_init_diag    = [0; 0; 1e-4; 1e-4; 0; 1e-5; 0];
            config.a_cov           = 0.005;
        case 'frozen_correct_peraxisR11_R22'
            config.Qz_diag_scaling = [0; 0; 1; 0; 0; 1e-8; 1e-8; 1e-8; 1e-8];
            config.Rz_diag_scaling = [1.53e-3; 1.29e-5; 4.35e-2; 0.01881; 0.01881; 0.01784];
            config.Pf_init_diag    = [0; 0; 1e-4; 1e-4; 0; 1e-5; 0];
            config.a_cov           = 0.005;
        case 'empirical'
            config.Qz_diag_scaling = [0; 0; 1e4; 0.1; 0; 1e-4; 0; 0; 0];
            config.Rz_diag_scaling = [1e-2; 1e-2; 1e-2; 1.0; 1.0; 1.0];
            config.Pf_init_diag    = [0; 0; 1e-4; 1e-4; 0; 10*(0.0147)^2; 0];
            config.a_cov           = 0.05;
        case 'beta'
            config.Qz_diag_scaling = [0; 0; 1; 0; 0; 1.3444e-11; 1.3444e-11; 1.3444e-11; 1.3444e-11];
            config.Rz_diag_scaling = [0.3970; 0.3970; 0.3970; 1.7185; 1.7185; 1.7185];
            config.Pf_init_diag    = [0; 0; 1e-4; 1e-4; 0; 10*(0.0147)^2; 0];
            config.a_cov           = 0.05;
        otherwise
            error('apply_qr_preset:bad_preset', ...
                  ['Unknown qr_preset "%s". Use frozen_correct / ' ...
                   'frozen_correct_peraxisR11 / frozen_correct_peraxisR11_R22 / ' ...
                   'empirical / beta.'], ...
                  config.qr_preset);
    end
end
