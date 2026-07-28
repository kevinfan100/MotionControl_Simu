function temp_diag_kfmeas_q55arms()
% TEMP diagnostic (chat 2026-07-22): compare Q55 choices on the kfmeas z-state
% (learn_t0=0, a' from start) + hunt the reported onset spike. Metrics in nm/pN
% and osc relative error %. Delete after use.

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

    Pp = calc_simulation_params(base); Pp = Pp.Value; Ts = Pp.common.Ts;
    t1 = base.t_hold; t2 = t1 + base.t_descend_override; t3 = t2 + base.n_cycles / base.frequency;

    % oracle reference arm ('known') for a'_true and a_true
    coK = base; coK.aprime_source = 'known';
    soK = run_pure_simulation(coK, struct('seed', 1, 'collect_diag', true));
    dK = soK.diag; N = size(dK.a_hat, 1); t = (0:N - 1)' * Ts;
    apO = dK.a_prime_used(:, 3);          % oracle a'(z)
    atz = soK.a_true_out(:, 3);           % true gain (z)

    arms = { ...
      'A1_kappa1_f0',   struct('aprime_source','kfmeas','aprime_learn_t0',0,'aprime_state_kappa',1,'aprime_state_Q55_floor',0); ...
      'A2_optA_f1e12',  struct('aprime_source','kfmeas','aprime_learn_t0',0,'aprime_state_kappa',0,'aprime_state_Q55_floor',1e-12); ...
      'A2b_optA_f0',    struct('aprime_source','kfmeas','aprime_learn_t0',0,'aprime_state_kappa',0,'aprime_state_Q55_floor',0); ...
      'H_initOracle',   struct('aprime_source','kfmeas','aprime_learn_t0',0,'aprime_state_kappa',1,'aprime_state_Q55_floor',0,'aprime_state_init_scale',1); ...
      'H_selfmod',      struct('aprime_source','kfmeas','aprime_learn_t0',0,'aprime_state_kappa',1,'aprime_state_Q55_floor',0,'aprime_state_selfmod',true); ...
    };

    hf = t < t1; ds = t >= t1 & t < t2; os = t >= t2 & t < t3;
    fprintf('\n=== kfmeas Q55 arms (seed 1, learn_t0=0). std/err in nm/pN, corr vs oracle a'' ===\n');
    fprintf('mean a_true_z osc = %.4e um/pN ; oracle a''(z) osc mean = %+.3e\n', mean(atz(os)), mean(apO(os)));
    fprintf('%-16s | hold std | desc std | osc std | osc relerr%% | max|a''| (t)      | osc corr(a'') | any|a''|>1\n', 'arm');
    for a = 1:size(arms, 1)
        cfg = base; f = fieldnames(arms{a, 2});
        for j = 1:numel(f); cfg.(f{j}) = arms{a, 2}.(f{j}); end
        so = temp_run_pure_sim_kfmeas(cfg, struct('seed', 1, 'collect_diag', true));
        d = so.diag; ahz = d.a_hat(:, 3); ap = d.a_prime_used(:, 3);
        [mx, kmx] = max(abs(ap));
        relerr = mean(abs(ahz(os) - atz(os)) ./ atz(os)) * 100;
        cc = corr(ap(os), apO(os));
        fprintf('%-16s | %8.3f | %8.3f | %7.3f | %10.2f | %.2e (%.3f) | %+.3f | %s\n', ...
                arms{a, 1}, std(ahz(hf)) * 1e3, std(ahz(ds)) * 1e3, std(ahz(os)) * 1e3, ...
                relerr, mx, t(kmx), cc, ternary(mx > 1, 'YES', 'no'));
    end
    fprintf('\n(oracle arm: hold std=%.3f desc std=%.3f osc std=%.3f nm/pN, osc relerr=%.2f%%)\n', ...
            std(dK.a_hat(hf, 3)) * 1e3, std(dK.a_hat(ds, 3)) * 1e3, std(dK.a_hat(os, 3)) * 1e3, ...
            mean(abs(dK.a_hat(os, 3) - atz(os)) ./ atz(os)) * 100);
end

function o = ternary(c, a, b); if c; o = a; else; o = b; end; end
