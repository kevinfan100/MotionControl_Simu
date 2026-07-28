function temp_diag_kfmeas_p44()
% TEMP diagnostic (chat 2026-07-22): in a sustained FAR hold, a'~0 => Q44=a'^2*Q33
% ~0 => does P44 collapse and a_d lock (can no longer track a_xm)? Positioning
% hold at h=50, log P44 / a_d / a_xm over time. Delete after use.

    [sd, ~, ~] = fileparts(mfilename('fullpath')); pr = fileparts(sd);
    addpath(fullfile(pr, 'model'), fullfile(pr, 'model', 'config'), ...
            fullfile(pr, 'model', 'wall_effect'), fullfile(pr, 'model', 'thermal_force'), ...
            fullfile(pr, 'model', 'trajectory'), fullfile(pr, 'model', 'controller'), ...
            fullfile(pr, 'model', 'dual_track'), sd);

    pc = physical_constants(); R = pc.R;
    cfg = user_config();
    cfg.eq17_variant = '4state'; cfg.trajectory_type = 'positioning';
    cfg.h_init = 50; cfg.T_sim = 3.0;
    cfg.ctrl_enable = true; cfg.thermal_enable = true; cfg.meas_noise_enable = true;
    cfg.lambda_c = 0.7; cfg.a_pd = 0.05; cfg.a_cov = 0.05;
    cfg.meas_noise_std = [0.00062; 0.00057; 0.00331];
    cfg.suppress_xD = true; cfg.h_bar_safe = 1;
    cfg.use_taylor_gain = true;
    cfg.aprime_source = 'kfmeas'; cfg.aprime_learn_t0 = 0;
    cfg.aprime_state_kappa = 0; cfg.aprime_state_Q55_floor = 1e-12;   % Option A
    cfg.use_c2 = true;

    Pp = calc_simulation_params(cfg); Pp = Pp.Value; Ts = Pp.common.Ts;
    so = temp_run_pure_sim_kfmeas(cfg, struct('seed', 1, 'collect_diag', true));
    d = so.diag; N = size(d.a_hat, 1); t = (0:N - 1)' * Ts;
    P44 = d.P_a(:, 3); ad = d.a_hat_d(:); axm = d.a_xm(:, 3); ah = d.a_hat(:, 3);

    fprintf('\n=== FAR positioning hold (h=50, 3s): does a_d lock? ===\n');
    idx = @(tt) find(t >= tt, 1, 'first');
    for tt = [0.1 0.5 1.0 2.0 3.0 - Ts]
        k = idx(tt);
        fprintf(' t=%.2f | P44=%.3e | a_d=%.4e | a_xm=%.4e | a_hat_z=%.4e\n', ...
                t(k), P44(k), ad(k), axm(k), ah(k));
    end
    fprintf('P44 ratio end/early = %.3f  (collapse if <<1)\n', P44(end) / P44(idx(0.1)));
    fprintf('a_d drift over 0.5-3s = %.3e um/pN (%.2f%% of a_d)\n', ...
            ad(end) - ad(idx(0.5)), (ad(end) - ad(idx(0.5))) / ad(end) * 100);
    fprintf('a_xm std over 0.5-3s  = %.3e (the measurement a_d would track if P44 alive)\n', ...
            std(axm(t >= 0.5)));
end
