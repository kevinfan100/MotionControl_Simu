function temp_diag_kfmeas_init()
% TEMP diagnostic (chat 2026-07-22): the from_anom figure used init_from_anom=false
% (DEFAULT) = a_x_init = a_nom/c(h_init) (WALL-AWARE, uses c). Re-check the first
% hold under the TRUE charter-legal init_from_anom=true (a_d = a_nom free-space):
% can a_d converge a_nom -> true in the far hold, or does it lock (P44->0 DARE bug)?
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
    base.aprime_source = 'kfmeas'; base.aprime_learn_t0 = 0;
    base.aprime_state_kappa = 0; base.aprime_state_Q55_floor = 1e-12;
    base.use_c2 = true;

    Pp = calc_simulation_params(base); Pp = Pp.Value; Ts = Pp.common.Ts;
    a_nom = Pp.common.Ts / Pp.ctrl.gamma; t1 = base.t_hold;

    fprintf('\n=== first-hold init check. a_nom(free-space) = %.3f nm/pN ===\n', a_nom * 1e3);
    for arm = [false true]
        cfg = base; cfg.init_from_anom = arm;
        so = temp_run_pure_sim_kfmeas(cfg, struct('seed', 1, 'collect_diag', true));
        d = so.diag; N = size(d.a_hat, 1); t = (0:N - 1)' * Ts;
        ad = d.a_hat_d(:) * 1e3; P44 = d.P_a(:, 3); tg = so.a_true_out(:, 3) * 1e3;
        idx = @(tt) find(t >= tt, 1, 'first');
        fprintf('\n--- init_from_anom = %d (%s) ---\n', arm, ...
                ternary(arm, 'a_nom free-space', 'a_nom/c(h_init) WALL-AWARE'));
        fprintf('true far gain = %.3f nm/pN\n', mean(tg(t < t1)));
        fprintf(' t=0.00 a_d=%.3f  P44=%.2e\n', ad(1), P44(1));
        for tt = [0.05 0.1 0.25 0.5]
            k = idx(tt);
            fprintf(' t=%.2f a_d=%.3f  P44=%.2e  (err vs true = %+.2f nm/pN)\n', ...
                    t(k), ad(k), P44(k), ad(k) - tg(k));
        end
        fprintf('  => first-hold: a_d moved %+.3f nm/pN from init; residual err at t=0.5 = %+.2f nm/pN (%.1f%%)\n', ...
                ad(idx(0.5)) - ad(1), ad(idx(0.5)) - tg(idx(0.5)), (ad(idx(0.5)) - tg(idx(0.5))) / tg(idx(0.5)) * 100);
    end
end

function o = ternary(c, a, b); if c; o = a; else; o = b; end; end
