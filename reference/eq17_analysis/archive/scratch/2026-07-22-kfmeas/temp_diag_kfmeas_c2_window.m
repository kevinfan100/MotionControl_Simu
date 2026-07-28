function temp_diag_kfmeas_c2_window()
% TEMP diagnostic (chat 2026-07-22): can multi-window weighted-LS C2 push the osc
% rel err below per-step C2? Modes: off / per-step / window {0.05,0.1,0.2,0.5}s.
% 3 seeds, osc + descent rel err of a_hat_z. Delete after use.

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
    base.aprime_state_kappa = 0; base.aprime_state_Q55_floor = 1e-12;

    Pp = calc_simulation_params(base); Pp = Pp.Value; Ts = Pp.common.Ts;
    t1 = base.t_hold; t2 = t1 + base.t_descend_override; t3 = t2 + base.n_cycles / base.frequency;
    relerr = @(ah, at, m) mean(abs(ah(m) - at(m)) ./ at(m)) * 100;

    modes = { ...
      'off',        struct('use_c2', false); ...
      'per-step',   struct('use_c2', true); ...
      'win 0.05s',  struct('use_c2', true, 'c2_window_s', 0.05); ...
      'win 0.10s',  struct('use_c2', true, 'c2_window_s', 0.10); ...
      'win 0.20s',  struct('use_c2', true, 'c2_window_s', 0.20); ...
      'win 0.50s',  struct('use_c2', true, 'c2_window_s', 0.50); ...
    };
    seeds = 1:3;

    fprintf('\n=== C2 multi-window sweep (Option A base, %d seeds). osc / descent rel err %% ===\n', numel(seeds));
    fprintf('%-10s | osc mean | desc mean\n', 'mode');
    for m = 1:size(modes, 1)
        eo = zeros(numel(seeds), 1); ed = zeros(numel(seeds), 1);
        for si = 1:numel(seeds)
            cfg = base; f = fieldnames(modes{m, 2});
            for j = 1:numel(f); cfg.(f{j}) = modes{m, 2}.(f{j}); end
            so = temp_run_pure_sim_kfmeas(cfg, struct('seed', seeds(si), 'collect_diag', true));
            N = size(so.diag.a_hat, 1); t = (0:N - 1)' * Ts;
            os = t >= t2 & t < t3; ds = t >= t1 & t < t2;
            ah = so.diag.a_hat(:, 3); at = so.a_true_out(:, 3);
            eo(si) = relerr(ah, at, os); ed(si) = relerr(ah, at, ds);
        end
        fprintf('%-10s | %8.2f | %8.2f\n', modes{m, 1}, mean(eo), mean(ed));
    end
    fprintf('(oracle osc ~2.3%%; Option A/off osc ~8.5%%, desc ~12.2%% over these 3 seeds)\n');
end
