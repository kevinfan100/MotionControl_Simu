function verify_eq17_unit_hbar_safe_plumbing()
%VERIFY_EQ17_UNIT_HBAR_SAFE_PLUMBING config.h_bar_safe -> ctrl_const plumbing
%   (gain_oracle_ab_design.md §12.1). Positioning hold at h_bar = 1.3
%   (inside the default gate band [1, 1.5)):
%   T1: default (no config field)  -> G3 fires (gate_active true somewhere).
%   T2: config.h_bar_safe = 1      -> G3 never fires on any axis.
%   T3: config.h_bar_safe = 1.5 (explicit default value) -> bit-identical
%       p_m_out/f_d_out vs T1 (plumbing adds no behavior change at default).

    fprintf('=== verify_eq17_unit_hbar_safe_plumbing ===\n');
    this_dir  = fileparts(mfilename('fullpath'));
    repo_root = fullfile(this_dir, '..', '..');
    addpath(fullfile(repo_root, 'model'), ...
            fullfile(repo_root, 'model', 'config'), ...
            fullfile(repo_root, 'model', 'controller'), ...
            fullfile(repo_root, 'model', 'wall_effect'), ...
            fullfile(repo_root, 'model', 'thermal_force'), ...
            fullfile(repo_root, 'model', 'trajectory'), ...
            fullfile(repo_root, 'model', 'dual_track'));

    pc  = physical_constants();
    cfg = user_config();
    cfg.eq17_variant      = '6state';
    cfg.trajectory_type   = 'positioning';
    cfg.h_init            = 1.3 * pc.R;       % h_bar = 1.3, inside gate band
    cfg.h_min             = 1.05 * pc.R;      % allow near-wall hold
    cfg.T_sim             = 0.05;             % 80 steps
    cfg.ctrl_enable       = true;
    cfg.thermal_enable    = true;
    cfg.meas_noise_enable = true;
    cfg.a_cov             = 0.05;             % eq17 6-state verified baseline

    ro = struct('seed', 1, 'verbose', false, 'collect_diag', true);

    so1 = run_pure_simulation(cfg, ro);                 % T1 default
    assert(any(so1.diag.gate_active(:)), ...
           'T1: G3 expected active at h_bar=1.3 under default h_bar_safe=1.5');
    fprintf('T1 PASS default gate fires at h_bar=1.3\n');

    cfg2 = cfg; cfg2.h_bar_safe = 1;
    so2 = run_pure_simulation(cfg2, ro);                % T2 gate-free
    assert(~any(so2.diag.gate_active(:)), ...
           'T2: G3 must never fire with h_bar_safe=1 (h_bar=1.3 > 1)');
    fprintf('T2 PASS h_bar_safe=1 -> gate-free\n');

    cfg3 = cfg; cfg3.h_bar_safe = 1.5;
    so3 = run_pure_simulation(cfg3, ro);                % T3 explicit default
    assert(isequal(so1.p_m_out, so3.p_m_out) && isequal(so1.f_d_out, so3.f_d_out), ...
           'T3: explicit h_bar_safe=1.5 must be bit-identical to default');
    fprintf('T3 PASS explicit default bit-identical\n');
    fprintf('ALL 3 PASS\n');
end
