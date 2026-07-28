function temp_diag_kfmeas_advantage()
% TEMP diagnostic (chat 2026-07-22): does the a'-state (kfmeas) and C2 actually
% buy accuracy vs a level-only baseline? All init_from_anom=1 (honest). Arms:
%   B0 = a' frozen at 0 (a_d level only)   A1 = kfmeas a'-state (Option A)
%   A2 = A1 + C2                            KN = known a' (oracle, "if we knew c")
% descent + osc a_z rel err, 3 seeds. Delete after use.

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
    base.use_taylor_gain = true; base.init_from_anom = true;

    Pp = calc_simulation_params(base); Pp = Pp.Value; Ts = Pp.common.Ts;
    t1 = base.t_hold; t2 = t1 + base.t_descend_override; t3 = t2 + base.n_cycles / base.frequency;
    relerr = @(ah, at, m) mean(abs(ah(m) - at(m)) ./ at(m)) * 100;

    arms = { ...
      'B0 a''=0 level', struct('aprime_source','kfmeas','aprime_learn_t0',1e9,'aprime_state_kappa',0,'aprime_state_Q55_floor',0,'use_c2',false); ...
      'A1 a''-state',   struct('aprime_source','kfmeas','aprime_learn_t0',0,'aprime_state_kappa',0,'aprime_state_Q55_floor',1e-12,'use_c2',false); ...
      'A2 +C2',         struct('aprime_source','kfmeas','aprime_learn_t0',0,'aprime_state_kappa',0,'aprime_state_Q55_floor',1e-12,'use_c2',true); ...
      'KN known a''',   struct('aprime_source','known','use_c2',false); ...
    };
    seeds = 1:3;

    fprintf('\n=== kfmeas / C2 advantage (init_from_anom=1, %d seeds) ===\n', numel(seeds));
    fprintf('%-16s | descent %% | osc %%\n', 'arm');
    for a = 1:size(arms, 1)
        ed = zeros(numel(seeds), 1); eo = zeros(numel(seeds), 1);
        for si = 1:numel(seeds)
            cfg = base; f = fieldnames(arms{a, 2});
            for j = 1:numel(f); cfg.(f{j}) = arms{a, 2}.(f{j}); end
            if strcmp(arms{a, 2}.aprime_source, 'known')
                so = run_pure_simulation(cfg, struct('seed', seeds(si), 'collect_diag', true));
            else
                so = temp_run_pure_sim_kfmeas(cfg, struct('seed', seeds(si), 'collect_diag', true));
            end
            N = size(so.diag.a_hat, 1); t = (0:N - 1)' * Ts;
            ds = t >= t1 & t < t2; os = t >= t2 & t < t3;
            ah = so.diag.a_hat(:, 3); at = so.a_true_out(:, 3);
            ed(si) = relerr(ah, at, ds); eo(si) = relerr(ah, at, os);
        end
        fprintf('%-16s | %8.2f | %6.2f\n', arms{a, 1}, mean(ed), mean(eo));
    end
    fprintf('(lower=better; KN=oracle upper bound / cheats with c)\n');
end
