function temp_diag_kfmeas_c2()
% TEMP diagnostic (chat 2026-07-22): does the C2 deterministic level channel
% (kf_meas.tex §9) lower the osc a_hat_z rel err vs Option A alone? C2 off vs on,
% 6 seeds, + r3_scale sensitivity. Delete after use.

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
    base.aprime_state_kappa = 0; base.aprime_state_Q55_floor = 1e-12;   % Option A

    Pp = calc_simulation_params(base); Pp = Pp.Value; Ts = Pp.common.Ts;
    t1 = base.t_hold; t2 = t1 + base.t_descend_override; t3 = t2 + base.n_cycles / base.frequency;

    relerr = @(ah, at, m) mean(abs(ah(m) - at(m)) ./ at(m)) * 100;

    fprintf('\n=== C2 off vs on (Option A base, 6 seeds). osc/descent rel err of a_hat_z [%%] ===\n');
    fprintf('seed |  C2 off osc | C2 on osc |  off desc | on desc | off hold-std | on hold-std\n');
    accum = zeros(2, 2);   % [off; on] x [osc; desc] means
    for s = 1:6
        c0 = base; c0.use_c2 = false;
        c1 = base; c1.use_c2 = true;   % default c2_Fmin=0.6, knu3=0.004, r3_scale=1
        s0 = temp_run_pure_sim_kfmeas(c0, struct('seed', s, 'collect_diag', true));
        s1 = temp_run_pure_sim_kfmeas(c1, struct('seed', s, 'collect_diag', true));
        N = size(s0.diag.a_hat, 1); t = (0:N - 1)' * Ts;
        os = t >= t2 & t < t3; ds = t >= t1 & t < t2; hf = t < t1;
        ah0 = s0.diag.a_hat(:, 3); ah1 = s1.diag.a_hat(:, 3);
        at = s0.a_true_out(:, 3);
        e0o = relerr(ah0, at, os); e1o = relerr(ah1, at, os);
        e0d = relerr(ah0, at, ds); e1d = relerr(ah1, at, ds);
        fprintf('%4d |   %8.2f  | %8.2f  | %8.2f  | %7.2f | %10.4f  | %9.4f\n', ...
                s, e0o, e1o, e0d, e1d, std(ah0(hf)) * 1e3, std(ah1(hf)) * 1e3);
        accum(1, :) = accum(1, :) + [e0o, e0d];
        accum(2, :) = accum(2, :) + [e1o, e1d];
    end
    accum = accum / 6;
    fprintf('MEAN | osc: off=%.2f%% on=%.2f%% (%+.1f%%) | desc: off=%.2f%% on=%.2f%%\n', ...
            accum(1, 1), accum(2, 1), (accum(2, 1) - accum(1, 1)) / accum(1, 1) * 100, ...
            accum(1, 2), accum(2, 2));

    fprintf('\n=== r3_scale sensitivity (seed 1, C2 on) ===\n');
    fprintf('r3_scale | osc rel err %% | desc rel err %%\n');
    for rs = [0.25 0.5 1 2 4]
        c = base; c.use_c2 = true; c.c2_r3_scale = rs;
        so = temp_run_pure_sim_kfmeas(c, struct('seed', 1, 'collect_diag', true));
        N = size(so.diag.a_hat, 1); t = (0:N - 1)' * Ts;
        os = t >= t2 & t < t3; ds = t >= t1 & t < t2;
        ah = so.diag.a_hat(:, 3); at = so.a_true_out(:, 3);
        fprintf('%8.2f | %11.2f | %11.2f\n', rs, relerr(ah, at, os), relerr(ah, at, ds));
    end
    fprintf('(oracle osc rel err ~2.3%%, Option A baseline ~6.5%%)\n');
end
