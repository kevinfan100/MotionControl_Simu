function temp_diag_kfmeas_holdcause()
% TEMP diagnostic (chat 2026-07-22): what setting makes the first-hold oscillate?
% Contrast Option A (Q55 propto Dh_d^2 -> 0 in hold) vs a FLAT Q55 floor (nonzero
% in hold = a' driven while unobservable). init a=a_nom, a'=0. Delete after use.

    [sd, ~, ~] = fileparts(mfilename('fullpath')); pr = fileparts(sd);
    addpath(fullfile(pr, 'model'), fullfile(pr, 'model', 'config'), ...
            fullfile(pr, 'model', 'wall_effect'), fullfile(pr, 'model', 'thermal_force'), ...
            fullfile(pr, 'model', 'trajectory'), fullfile(pr, 'model', 'controller'), ...
            fullfile(pr, 'model', 'dual_track'), sd);

    pc = physical_constants(); R = pc.R;
    base = user_config();
    base.eq17_variant = '4state'; base.trajectory_type = 'osc';
    base.h_init = 50; base.h_bottom = 2.0 * R; base.amplitude = 2.5;
    base.frequency = 1; base.n_cycles = 4; base.t_hold = 0.5; base.t_descend_override = 1.0;
    base.h_min = 1.05 * R;
    base.T_sim = base.t_hold + base.t_descend_override + base.n_cycles / base.frequency + 0.5;
    base.ctrl_enable = true; base.thermal_enable = true; base.meas_noise_enable = true;
    base.lambda_c = 0.7; base.a_pd = 0.05; base.a_cov = 0.05;
    base.meas_noise_std = [0.00062; 0.00057; 0.00331];
    base.suppress_xD = true; base.h_bar_safe = 1;
    base.use_taylor_gain = true;
    base.aprime_source = 'kfmeas'; base.aprime_learn_t0 = 0;

    Pp = calc_simulation_params(base); Pp = Pp.Value; Ts = Pp.common.Ts;
    a_nom = Pp.common.Ts / Pp.ctrl.gamma;
    t1 = base.t_hold;

    fprintf('\n=== hold-oscillation cause: Option A (Q55 prop Dh_d^2) vs FLAT Q55 ===\n');
    fprintf('a_nom = %.4e um/pN, init a_d=a_nom, a''=0\n', a_nom);
    fprintf('%-18s | first-hold std(a_hat_z) [nm/pN] | hold |a''| max\n', 'Q55 setting');
    arms = { ...
      'OptionA(prop Dh^2)', struct('aprime_state_kappa', 0, 'aprime_state_Q55_floor', 1e-12); ...
      'flat 1e-10',         struct('aprime_state_kappa', 0, 'aprime_state_Q55_floor', 1e-10); ...
      'flat 1e-9',          struct('aprime_state_kappa', 0, 'aprime_state_Q55_floor', 1e-9); ...
      'flat 1e-8',          struct('aprime_state_kappa', 0, 'aprime_state_Q55_floor', 1e-8); ...
      'flat 1e-7',          struct('aprime_state_kappa', 0, 'aprime_state_Q55_floor', 1e-7); ...
    };
    for a = 1:size(arms, 1)
        cfg = base; f = fieldnames(arms{a, 2});
        for j = 1:numel(f); cfg.(f{j}) = arms{a, 2}.(f{j}); end
        so = temp_run_pure_sim_kfmeas(cfg, struct('seed', 1, 'collect_diag', true));
        N = size(so.diag.a_hat, 1); t = (0:N - 1)' * Ts; hf = t < t1;
        ah = so.diag.a_hat(:, 3); ap = so.diag.a_prime_used(:, 3);
        fprintf('%-18s | %28.3f | %.3e\n', arms{a, 1}, std(ah(hf)) * 1e3, max(abs(ap(hf))));
    end
end
