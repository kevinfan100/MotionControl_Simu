function temp_diag_kfmeas_seeds()
% TEMP diagnostic (chat 2026-07-22): seed robustness of kfmeas z-state Option A
% vs baseline Q55. Checks whether the reported sporadic a' blow-up (-24) recurs.
% Delete after use.

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

    arms = { ...
      'baseline_kappa1', struct('aprime_source','kfmeas','aprime_learn_t0',0,'aprime_state_kappa',1,'aprime_state_Q55_floor',0); ...
      'optA_f1e12',      struct('aprime_source','kfmeas','aprime_learn_t0',0,'aprime_state_kappa',0,'aprime_state_Q55_floor',1e-12); ...
    };
    seeds = 1:6;

    for a = 1:size(arms, 1)
        fprintf('\n=== arm %s ===\n', arms{a, 1});
        fprintf('seed | hold std | osc std | osc relerr%% | max|a''| (t)       | blowup\n');
        for s = seeds
            cfg = base; f = fieldnames(arms{a, 2});
            for j = 1:numel(f); cfg.(f{j}) = arms{a, 2}.(f{j}); end
            so = temp_run_pure_sim_kfmeas(cfg, struct('seed', s, 'collect_diag', true));
            d = so.diag; N = size(d.a_hat, 1); t = (0:N - 1)' * Ts;
            hf = t < t1; os = t >= t2 & t < t3;
            ahz = d.a_hat(:, 3); ap = d.a_prime_used(:, 3); atz = so.a_true_out(:, 3);
            [mx, kmx] = max(abs(ap));
            relerr = mean(abs(ahz(os) - atz(os)) ./ atz(os)) * 100;
            fprintf('%4d | %8.3f | %7.3f | %10.2f | %.2e (%.3f) | %s\n', ...
                    s, std(ahz(hf)) * 1e3, std(ahz(os)) * 1e3, relerr, mx, t(kmx), ...
                    ternary(mx > 0.5, 'YES', 'no'));
        end
    end
end

function o = ternary(c, a, b); if c; o = a; else; o = b; end; end
