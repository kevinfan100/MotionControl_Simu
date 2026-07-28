function temp_diag_kfmeas_onset()
% TEMP diagnostic (chat 2026-07-22): reproduce the kfmeas first-hold / descent-
% onset behaviour with a' LEARNING FROM START (learn_t0=0, NO time-freeze). Q55
% is already Dh_d^2-scaled (=0 in hold), so this tests whether the reported
% onset spike (a'->-24, transition std 2.91) is Q55-inflation or something else.
% Logs a'(slot5), P55, a_hat_z(recon), a_d, S_y2, K_a_y2, innov_y2, h_bar across
% hold(far)->descent->osc to locate WHERE/WHY a' blows up. Delete after use.

    [sd, ~, ~] = fileparts(mfilename('fullpath')); pr = fileparts(sd);
    addpath(fullfile(pr, 'model'), fullfile(pr, 'model', 'config'), ...
            fullfile(pr, 'model', 'wall_effect'), fullfile(pr, 'model', 'thermal_force'), ...
            fullfile(pr, 'model', 'trajectory'), fullfile(pr, 'model', 'controller'), ...
            fullfile(pr, 'model', 'dual_track'), sd);

    pc = physical_constants(); R = pc.R;
    cfg = user_config();
    cfg.eq17_variant = '4state'; cfg.trajectory_type = 'osc';
    cfg.h_init = 50; cfg.h_bottom = 2.0 * R; cfg.amplitude = 2.5;
    cfg.frequency = 1; cfg.n_cycles = 4; cfg.t_hold = 0.5; cfg.t_descend_override = 1.0;
    cfg.h_min = 1.05 * R;
    cfg.T_sim = cfg.t_hold + cfg.t_descend_override + cfg.n_cycles / cfg.frequency + 0.5;
    cfg.ctrl_enable = true; cfg.thermal_enable = true; cfg.meas_noise_enable = true;
    cfg.lambda_c = 0.7; cfg.a_pd = 0.05; cfg.a_cov = 0.05;
    cfg.meas_noise_std = [0.00062; 0.00057; 0.00331];
    cfg.suppress_xD = true; cfg.h_bar_safe = 1;
    cfg.use_taylor_gain = true;
    cfg.aprime_source = 'kfmeas';
    cfg.aprime_learn_t0 = 0;         % a' learns from start (NO time-freeze)

    Pp = calc_simulation_params(cfg); Pp = Pp.Value; Ts = Pp.common.Ts;
    t1 = cfg.t_hold; t2 = t1 + cfg.t_descend_override; t3 = t2 + cfg.n_cycles / cfg.frequency;

    so = temp_run_pure_sim_kfmeas(cfg, struct('seed', 1, 'collect_diag', true));
    d = so.diag; N = size(d.a_hat, 1); t = (0:N - 1)' * Ts;

    ap   = d.a_prime_used(:, 3);     % a' state (z)
    P55  = d.P_aprime(:, 3);
    ahz  = d.a_hat(:, 3);            % reconstructed a_hat_z = a_d - a'*dxhat3
    adz  = d.a_hat_d(:);             % a_d posterior (slot 4, z)
    Sy2  = d.S_y2(:, 3);
    Kay2 = d.K_kf_a_y2(:, 3);
    iy2  = d.innovation_y2(:, 3);
    hb   = d.h_bar(:);
    atz  = so.a_true_out(:, 3);      % true gain z [um/pN]

    win = @(a, b) t >= a & t < b;
    seg = {'hold_far', 0, t1; 'descent', t1, t2; 'osc', t2, t3; 'hold_near', t3, t(end) + Ts};
    fprintf('\n=== kfmeas onset diagnostic (learn_t0=0, a'' from start, seed 1) ===\n');
    fprintf('Ts=%.4e  N=%d  segs: t1=%.2f t2=%.2f t3=%.2f\n', Ts, N, t1, t2, t3);
    fprintf('%-10s | std(ahz)  mean(ahz) | a''[min , max]        | P55[min , max]     | max|ahz-atrue|\n', 'seg');
    for s = 1:size(seg, 1)
        m = win(seg{s, 2}, seg{s, 3});
        fprintf('%-10s | %8.4f %8.4f | [%+.2e %+.2e] | [%.1e %.1e] | %.4f\n', ...
                seg{s, 1}, std(ahz(m)), mean(ahz(m)), min(ap(m)), max(ap(m)), ...
                min(P55(m)), max(P55(m)), max(abs(ahz(m) - atz(m))));
    end
    fprintf('mean a_true_z over osc = %.4e ; oracle-ish |a''| scale ~ a/(h-hw) ~ small\n', mean(atz(win(t2, t3))));

    ib = find(abs(ap) > 1, 1, 'first');
    if ~isempty(ib)
        fprintf('\nFIRST |a''|>1 at k=%d t=%.4f (h_bar=%.2f). Trace k-10..k+10:\n', ib, t(ib), hb(ib));
        i0 = max(1, ib - 10); i1 = min(N, ib + 10);
        fprintf('   k     t       a''         P55        S_y2       K_a_y2     innov_y2    a_d        ahz        h_bar\n');
        for k = i0:i1
            mark = ''; if k == ib; mark = '  <== first |a''|>1'; end
            fprintf('%5d %.4f %+9.2e %.3e %.3e %+9.2e %+9.2e %+.3e %+.3e %6.2f%s\n', ...
                    k, t(k), ap(k), P55(k), Sy2(k), Kay2(k), iy2(k), adz(k), ahz(k), hb(k), mark);
        end
    else
        [mx, kmx] = max(abs(ap));
        fprintf('\nNo |a''|>1 in whole run. max|a''|=%.3e at t=%.3f (h_bar=%.2f).\n', mx, t(kmx), hb(kmx));
    end

    outmat = fullfile('/Users/kevin/.claude/jobs/202173de/tmp', 'diag_kfmeas_onset.mat');
    save(outmat, 't', 'ap', 'P55', 'ahz', 'adz', 'Sy2', 'Kay2', 'iy2', 'hb', 'atz', 'seg', 'Ts');
    fprintf('saved %s\n', outmat);
end
