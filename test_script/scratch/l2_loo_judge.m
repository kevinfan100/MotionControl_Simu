% L2 LOO battery -- judge. Pre-registration: test_script/scratch/l2_loo_prereg.txt
% STATUS: ACTIVE (scratch, L2 instrument) | PURPOSE: paired per-seed comparison
%   of each LOO arm against the production baseline on the registered
%   signature quantities, the a_hat effect, additivity and the clamp/gate
%   census; one figure page per member | EXPIRES: stacked-fix audit closes.
% Inputs: test_results/l2_loo/l2_<arm>_100.mat written by l2_loo_run_arms.m
% (never the driver's run_formC_b_*.mat). Seeds are the only resampling unit
% (l0_jackknife_se). Reports raw numbers and the registered verdicts only.
cd('/Users/kevin/code/MotionControl_Simu-motion-test');
root = pwd;
addpath(fullfile(root, 'test_script', 'scratch'));
IN_DIR  = fullfile(root, 'test_results', 'l2_loo');
FIG_DIR = IN_DIR;
AX = 3;                                   % z, wall-normal
ARMS = {'base', 'echo_off', 'ma2_off', 'fe4_off', 'both_off'};
W_TP = 10;                                % +-10-step window at turning points
ACF_LAGS = 1:5;

% ---------------------------------------------------------------- load
A = struct();
for i = 1:numel(ARMS)
    fn = fullfile(IN_DIR, sprintf('l2_%s_100.mat', ARMS{i}));
    if ~exist(fn, 'file'); fprintf('[skip] %s not found\n', fn); continue; end
    A.(ARMS{i}) = local_load(fn);
end
have = fieldnames(A);
assert(isfield(A, 'base'), 'l2_loo_judge:noBase', 'baseline arm missing');
B = A.base;
ns = B.ns; t = B.t;
seeds = B.seeds;
% windows (driver's own, stored)
w = B.windows;
m_desc = t >= w.descent(1) & t <= w.descent(2);
m_osc  = t >  w.osc(1)     & t <= w.osc(2);
m_hold = t >  w.hold(1);
fprintf('windows: descent [%.2f,%.2f] osc (%.2f,%.2f] hold (%.2f,end]; N=%d, ns=%d, Ts=%.6g\n', ...
        w.descent, w.osc, w.hold(1), numel(t), ns, t(2)-t(1));
% clamp constants from the run's own ctrl_const (else controller defaults)
cc = B.K;
a_floor = local_gfd(cc, 'a_bar_floor', 0.05);  a_ceil = local_gfd(cc, 'a_bar_ceil', 1 - 1e-4);
b_floor = local_gfd(cc, 'b_floor', 0.60);      b_ceil = local_gfd(cc, 'b_ceil', 1.05);
fprintf('clamps: a_bar [%g, %g]  b [%g, %g]  (%s)\n', a_floor, a_ceil, b_floor, b_ceil, ...
        local_src(cc, {'a_bar_floor', 'a_bar_ceil', 'b_floor', 'b_ceil'}));

% turning points of the commanded height inside the oscillation phase
hd = B.h_bar_d(:, 1, 1);
tp = local_turning_points(hd, t, [w.descent(2) - 0.05, w.osc(2) + 0.05]);
fprintf('turning points (h_bar_d extrema) at t = %s s  (n=%d; top=%d trough=%d)\n', ...
        mat2str(round(t(tp.idx).', 4)), numel(tp.idx), sum(tp.is_top), sum(~tp.is_top));
m_tp = false(size(t));
for i = 1:numel(tp.idx)
    m_tp(max(1, tp.idx(i) - W_TP):min(numel(t), tp.idx(i) + W_TP)) = true;
end
m_tp_top = false(size(t)); m_tp_tr = false(size(t));
for i = 1:numel(tp.idx)
    rg = max(1, tp.idx(i) - W_TP):min(numel(t), tp.idx(i) + W_TP);
    if tp.is_top(i); m_tp_top(rg) = true; else; m_tp_tr(rg) = true; end
end

% ---------------------------------------------------------------- per-seed stats
S = struct();
for i = 1:numel(have)
    r = A.(have{i});
    q = struct();
    at = squeeze(r.a_true_norm(:, AX, :));   ah = squeeze(r.a_bar_hat(:, AX, :));
    q.a_T      = mean(at(m_hold, :), 1).';
    q.bias_pct = (100 * mean(ah(m_hold, :) - at(m_hold, :), 1) ./ q.a_T.').';
    q.hold_mean_pct_drv = r.metrics.rows(:, 3);
    q.desc_pk_drv = r.metrics.rows(:, 1);
    q.osc_rms_drv = r.metrics.rows(:, 2);
    q.ahat_hold   = mean(ah(m_hold, :), 1).';
    q.bhat_end    = squeeze(r.b_hat(end, AX, :));
    q.bhat_hold   = squeeze(mean(r.b_hat(m_hold, AX, :), 1));
    % (a) echo signature
    i2 = squeeze(r.innov_y2(:, AX, :)); g = squeeze(r.gate(:, AX, :));
    R2 = squeeze(r.R2(:, AX, :)); ka2 = squeeze(r.K_a_y2(:, AX, :));
    q.innov2_hold = nan(ns, 1); q.var_ratio = nan(ns, 1); q.Ka2_hold = nan(ns, 1);
    q.n_open_hold = zeros(ns, 1);
    for s = 1:ns
        mk = m_hold & ~g(:, s);
        q.n_open_hold(s) = sum(mk);
        if sum(mk) > 10
            q.innov2_hold(s) = mean(i2(mk, s));
            q.var_ratio(s)   = var(i2(mk, s)) / mean(R2(mk, s));
            q.Ka2_hold(s)    = mean(abs(ka2(mk, s)));
        end
    end
    % (b) ma2 signature
    i1 = squeeze(r.innov_y1(:, AX, :));
    q.acf_hold = zeros(ns, numel(ACF_LAGS)); q.acf_osc = zeros(ns, numel(ACF_LAGS));
    for s = 1:ns
        q.acf_hold(s, :) = local_acf(i1(m_hold, s), ACF_LAGS);
        q.acf_osc(s, :)  = local_acf(i1(m_osc, s),  ACF_LAGS);
    end
    % (c) fe_row4 signature
    kb1 = squeeze(r.K_b_y1(:, AX, :)); kb2 = squeeze(r.K_b_y2(:, AX, :));
    q.Kb1_tp     = mean(abs(kb1(m_tp, :)), 1).';
    q.Kb1_tp_top = mean(abs(kb1(m_tp_top, :)), 1).';
    q.Kb1_tp_tr  = mean(abs(kb1(m_tp_tr, :)), 1).';
    q.Kb1_osc    = mean(abs(kb1(m_osc, :)), 1).';
    q.Kb1_tp_signed = mean(kb1(m_tp, :), 1).';
    q.Kb2_tp     = mean(abs(kb2(m_tp, :)), 1).';
    q.Kb2_osc    = mean(abs(kb2(m_osc, :)), 1).';
    % (e) census (post-clamp equality => lower bounds)
    for ax = 1:3
        ahx = squeeze(r.a_bar_hat(:, ax, :)); bhx = squeeze(r.b_hat(:, ax, :)); gx = squeeze(r.gate(:, ax, :));
        q.n_afloor(:, ax) = sum(ahx == a_floor, 1).';
        q.n_aceil(:, ax)  = sum(ahx == a_ceil, 1).';
        q.n_bfloor(:, ax) = sum(bhx == b_floor, 1).';
        q.n_bceil(:, ax)  = sum(bhx == b_ceil, 1).';
        q.n_gate_all(:, ax)  = sum(gx, 1).';
        q.n_gate_desc(:, ax) = sum(gx(m_desc, :), 1).';
        q.n_gate_osc(:, ax)  = sum(gx(m_osc, :), 1).';
        q.n_gate_hold(:, ax) = sum(gx(m_hold, :), 1).';
    end
    q.n_nan = squeeze(sum(~isfinite(r.a_bar_hat(:, AX, :)), 1));
    S.(have{i}) = q;
end
N_hold = sum(m_hold); N_desc = sum(m_desc); N_osc = sum(m_osc); N_all = numel(t);

% ---------------------------------------------------------------- report helpers
rep = @(name, x) fprintf('  %-34s mean %+.5g  sd %.4g  SEM %.4g  (n=%d)\n', name, mean(x, 'omitnan'), ...
                         std(x, 0, 'omitnan'), local_sem(x), sum(isfinite(x)));
fprintf('\n================ BASELINE (arm best, production) ================\n');
rep('a_T (trough truth, norm)', S.base.a_T);
rep('hold bias_pct (mean(dA)/a_T)', S.base.bias_pct);
rep('driver hold mean %', S.base.hold_mean_pct_drv);
rep('driver desc peak %', S.base.desc_pk_drv);
rep('driver osc RMS %', S.base.osc_rms_drv);
rep('innov2 hold mean (gate-open)', S.base.innov2_hold);
rep('Var(innov2)/mean R2 (hold)', S.base.var_ratio);
rep('|K_a_y2| hold', S.base.Ka2_hold);
rep('innov1 acf lag1 (hold)', S.base.acf_hold(:, 1));
rep('innov1 acf lag2 (hold)', S.base.acf_hold(:, 2));
rep('|K_b_y1| turning +-10', S.base.Kb1_tp);
rep('|K_b_y1| osc window', S.base.Kb1_osc);
rep('b_hat end', S.base.bhat_end);

% ---------------------------------------------------------------- paired tables
V = struct();
for i = 2:numel(ARMS)
    nm = ARMS{i};
    if ~isfield(S, nm); continue; end
    q = S.(nm); b = S.base;
    fprintf('\n================ ARM %s vs base (paired, OFF - ON, n=%d) ================\n', nm, ns);
    P = struct();
    P.bias_pct  = local_pair(q.bias_pct, b.bias_pct, 'hold bias_pct [pp of a_T]');
    P.hold_drv  = local_pair(q.hold_mean_pct_drv, b.hold_mean_pct_drv, 'driver hold mean %');
    P.desc_drv  = local_pair(q.desc_pk_drv, b.desc_pk_drv, 'driver desc peak %');
    P.osc_drv   = local_pair(q.osc_rms_drv, b.osc_rms_drv, 'driver osc RMS %');
    P.bhat_end  = local_pair(q.bhat_end, b.bhat_end, 'b_hat end');
    fprintf('  -- (a) echo signature --\n');
    P.innov2    = local_pair(q.innov2_hold, b.innov2_hold, 'innov2 hold mean');
    P.var_ratio = local_pair(q.var_ratio, b.var_ratio, 'Var(innov2)/R2 hold');
    P.Ka2       = local_pair(q.Ka2_hold, b.Ka2_hold, '|K_a_y2| hold');
    fprintf('     ON  Var ratio mean %.4f  OFF %.4f | |K_a_y2| ratio OFF/ON (median over seeds) %.3f\n', ...
            mean(b.var_ratio, 'omitnan'), mean(q.var_ratio, 'omitnan'), median(q.Ka2_hold ./ b.Ka2_hold, 'omitnan'));
    fprintf('  -- (b) ma2 signature (innov1 acf, hold window) --\n');
    for L = 1:numel(ACF_LAGS)
        fprintf('     lag %d: ON %+.4f (sd %.4f SEM %.4f)   OFF %+.4f (sd %.4f SEM %.4f)   |  osc window ON %+.4f OFF %+.4f\n', ...
                ACF_LAGS(L), mean(b.acf_hold(:, L)), std(b.acf_hold(:, L)), local_sem(b.acf_hold(:, L)), ...
                mean(q.acf_hold(:, L)), std(q.acf_hold(:, L)), local_sem(q.acf_hold(:, L)), ...
                mean(b.acf_osc(:, L)), mean(q.acf_osc(:, L)));
    end
    P.acf1 = local_pair(q.acf_hold(:, 1), b.acf_hold(:, 1), 'acf lag1 (hold)');
    P.acf2 = local_pair(q.acf_hold(:, 2), b.acf_hold(:, 2), 'acf lag2 (hold)');
    fprintf('  -- (c) fe_row4 signature (|K_b| around turning points, z) --\n');
    P.Kb1_tp  = local_pair(q.Kb1_tp, b.Kb1_tp, '|K_b_y1| turning +-10');
    P.Kb1_osc = local_pair(q.Kb1_osc, b.Kb1_osc, '|K_b_y1| osc window');
    P.Kb2_tp  = local_pair(q.Kb2_tp, b.Kb2_tp, '|K_b_y2| turning +-10');
    P.Kb2_osc = local_pair(q.Kb2_osc, b.Kb2_osc, '|K_b_y2| osc window');
    r_tp  = q.Kb1_tp ./ b.Kb1_tp;  r_osc = q.Kb1_osc ./ b.Kb1_osc;
    r_top = q.Kb1_tp_top ./ b.Kb1_tp_top; r_tr = q.Kb1_tp_tr ./ b.Kb1_tp_tr;
    fprintf('     ratio OFF/ON |K_b_y1|: turning median %.3f (IQR %.3f-%.3f) | top %.3f | trough %.3f | osc window median %.3f\n', ...
            median(r_tp), prctile(r_tp, 25), prctile(r_tp, 75), median(r_top), median(r_tr), median(r_osc));
    fprintf('     ratio OFF/ON |K_b_y2|: turning median %.3f | osc window median %.3f\n', ...
            median(q.Kb2_tp ./ b.Kb2_tp), median(q.Kb2_osc ./ b.Kb2_osc));
    fprintf('     signed mean K_b_y1 at turning: ON %+.4e  OFF %+.4e\n', mean(b.Kb1_tp_signed), mean(q.Kb1_tp_signed));
    P.r_tp = r_tp; P.r_osc = r_osc;
    V.(nm) = P;
end

% ---------------------------------------------------------------- verdicts (registered rules)
fprintf('\n================ REGISTERED VERDICTS ================\n');
if isfield(V, 'echo_off')
    P = V.echo_off;
    ok_vr = all([mean(S.base.var_ratio, 'omitnan'), mean(S.echo_off.var_ratio, 'omitnan')] >= 0.8) && ...
            all([mean(S.base.var_ratio, 'omitnan'), mean(S.echo_off.var_ratio, 'omitnan')] <= 1.25);
    if abs(P.innov2.t) >= 3 && ok_vr && abs(P.bias_pct.mu) < 2.0
        vd = 'REPRODUCED';
    elseif abs(P.innov2.t) < 2
        vd = 'EXPIRED';
    else
        vd = 'OTHER';
    end
    % Q_a2 clause audit: the band [0.8,1.25] was registered for Var(y2)/formula
    % ~1 (08-19); if the BASELINE itself is outside it, the clause cannot
    % discriminate arms and the verdict without it is printed as well.
    if abs(P.innov2.t) >= 3 && abs(P.bias_pct.mu) < 2.0; vd_noq2 = 'REPRODUCED';
    elseif abs(P.innov2.t) < 2; vd_noq2 = 'EXPIRED'; else; vd_noq2 = 'OTHER'; end
    fprintf('(a) Q_a2 clause: baseline Var(innov2)/R2 = %.3f, echo_off = %.3f, band [0.8,1.25] met on both = %d; verdict WITHOUT the Q_a2 clause = %s\n', ...
            mean(S.base.var_ratio, 'omitnan'), mean(S.echo_off.var_ratio, 'omitnan'), ok_vr, vd_noq2);
    fprintf('(a) y2_echo_corr : %s  [innov2 hold diff t=%+.2f; Var ratio ON %.3f OFF %.3f; a_hat effect %+.3f pp (t=%+.2f); |K_a_y2| OFF/ON %.3f]\n', ...
            vd, P.innov2.t, mean(S.base.var_ratio, 'omitnan'), mean(S.echo_off.var_ratio, 'omitnan'), ...
            P.bias_pct.mu, P.bias_pct.t, median(S.echo_off.Ka2_hold ./ S.base.Ka2_hold, 'omitnan'));
    fprintf('    stop-rule: signature %s, |t(a_hat)|=%.2f %s 1  => %s\n', vd, abs(P.bias_pct.t), ...
            local_lt(abs(P.bias_pct.t), 1), local_candidate(vd, P.bias_pct.t));
end
if isfield(V, 'ma2_off')
    P = V.ma2_off;
    on1 = abs(mean(S.base.acf_hold(:, 1)));  on2 = abs(mean(S.base.acf_hold(:, 2)));
    of1 = abs(mean(S.ma2_off.acf_hold(:, 1))); of2 = abs(mean(S.ma2_off.acf_hold(:, 2)));
    if (of1 >= 0.15 || of2 >= 0.15) && on1 <= 0.06 && on2 <= 0.06
        vd = 'REPRODUCED';
    elseif of1 <= 0.06 && of2 <= 0.06
        vd = 'EXPIRED';
    else
        vd = 'OTHER';
    end
    fprintf('(b) ma2_aug      : %s  [hold |acf| lag1/2: ON %.4f/%.4f  OFF %.4f/%.4f; a_hat effect %+.3f pp (t=%+.2f); desc pk diff %+.3f (t=%+.2f); osc RMS diff %+.3f (t=%+.2f)]\n', ...
            vd, on1, on2, of1, of2, P.bias_pct.mu, P.bias_pct.t, P.desc_drv.mu, P.desc_drv.t, P.osc_drv.mu, P.osc_drv.t);
    fprintf('    stop-rule: signature %s, |t(a_hat)|=%.2f %s 1  => %s\n', vd, abs(P.bias_pct.t), ...
            local_lt(abs(P.bias_pct.t), 1), local_candidate(vd, P.bias_pct.t));
end
if isfield(V, 'fe4_off')
    P = V.fe4_off;
    rt = median(P.r_tp); ro = median(P.r_osc);
    if rt <= 0.5 && (ro - rt) >= 0.2
        vd = 'REPRODUCED';
    elseif rt >= 0.8 && rt <= 1.25
        vd = 'EXPIRED';
    else
        vd = 'OTHER';
    end
    fprintf('(c) fe_row4_full : %s  [|K_b_y1| OFF/ON median: turning %.3f, osc window %.3f; a_hat effect %+.3f pp (t=%+.2f); b_hat end diff %+.4f (t=%+.2f)]\n', ...
            vd, rt, ro, P.bias_pct.mu, P.bias_pct.t, P.bhat_end.mu, P.bhat_end.t);
    fprintf('    stop-rule: signature %s, |t(a_hat)|=%.2f %s 1  => %s\n', vd, abs(P.bias_pct.t), ...
            local_lt(abs(P.bias_pct.t), 1), local_candidate(vd, P.bias_pct.t));
end
if all(isfield(S, {'echo_off', 'ma2_off', 'both_off'}))
    fprintf('\n================ (d) ADDITIVITY: both_off - base  vs  (echo_off - base) + (ma2_off - base) ================\n');
    flds = {'bias_pct', 'hold bias_pct [pp]'; 'innov2_hold', 'innov2 hold mean'; 'acf1', 'acf lag1 hold'; ...
            'desc_pk_drv', 'driver desc peak %'; 'osc_rms_drv', 'driver osc RMS %'};
    for i = 1:size(flds, 1)
        f = flds{i, 1};
        if strcmp(f, 'acf1')
            g = @(q) q.acf_hold(:, 1);
        else
            g = @(q) q.(f);
        end
        Db = g(S.both_off) - g(S.base); De = g(S.echo_off) - g(S.base); Dm = g(S.ma2_off) - g(S.base);
        Q = Db - (De + Dm);
        [mu, se] = l0_jackknife_se(Q); tq = mu / se;
        if abs(tq) < 2; vd = 'ADDITIVE'; elseif abs(tq) >= 3; vd = 'NON-ADDITIVE'; else; vd = 'INDETERMINATE'; end
        fprintf('  %-22s D_both %+.4g  D_echo %+.4g  D_ma2 %+.4g  sum %+.4g | interaction %+.4g sd %.4g SEM %.4g t=%+.2f  => %s\n', ...
                flds{i, 2}, mean(Db, 'omitnan'), mean(De, 'omitnan'), mean(Dm, 'omitnan'), ...
                mean(De + Dm, 'omitnan'), mu, std(Q, 0, 'omitnan'), se, tq, vd);
    end
    D_both = local_pair(S.both_off.bias_pct, S.base.bias_pct, 'both_off hold bias_pct [pp]');
end

% ---------------------------------------------------------------- (e) census
fprintf('\n================ (e) CLAMP / GATE CENSUS (POST-CLAMP equality => LOWER BOUNDS on clamp activity) ================\n');
fprintf('samples per seed: N_all %d, desc %d, osc %d, hold %d\n', N_all, N_desc, N_osc, N_hold);
fprintf('%-9s | %-4s | %10s %10s %10s %10s | %9s %9s %9s %9s | %s\n', 'arm', 'axis', ...
        'a==floor', 'a==ceil', 'b==floor', 'b==ceil', 'gate all', 'gate desc', 'gate osc', 'gate hold', 'NaN');
for i = 1:numel(have)
    q = S.(have{i});
    for ax = 1:3
        fprintf('%-9s | %-4s | %10s %10s %10s %10s | %9s %9s %9s %9s | %d\n', have{i}, char('w' + ax), ...
                local_cnt(q.n_afloor(:, ax)), local_cnt(q.n_aceil(:, ax)), local_cnt(q.n_bfloor(:, ax)), ...
                local_cnt(q.n_bceil(:, ax)), local_cnt(q.n_gate_all(:, ax)), local_cnt(q.n_gate_desc(:, ax)), ...
                local_cnt(q.n_gate_osc(:, ax)), local_cnt(q.n_gate_hold(:, ax)), sum(q.n_nan));
    end
end
fprintf('cell format: total samples over 100 seeds / seeds with >=1 hit / max per seed\n');

% ---------------------------------------------------------------- figures
close all;
i7 = find(seeds == 7, 1);
figs = {'echo_off', 'y2_echo_corr'; 'ma2_off', 'ma2_aug'; 'fe4_off', 'fe_row4_full'; 'both_off', 'echo+ma2'};
for f = 1:size(figs, 1)
    nm = figs{f, 1};
    if ~isfield(A, nm); continue; end
    fig = figure('Position', [50 50 1500 900], 'Color', 'w');
    arms2 = {'base', nm}; lab2 = {'baseline (production)', sprintf('%s OFF', figs{f, 2})};
    hx = gobjects(2, 2);
    for c = 1:2
        r = A.(arms2{c});
        at = squeeze(r.a_true_norm(:, AX, :)); ah = squeeze(r.a_bar_hat(:, AX, :));
        % row 1: a_bar_hat vs a_true, seed 7 + 100-seed band
        hx(1, c) = subplot(2, 2, c); hold on; box on;
        mu_h = mean(ah, 2); sd_h = std(ah, 0, 2);
        fill([t; flipud(t)], [mu_h - sd_h; flipud(mu_h + sd_h)], [0.6 0.75 1], 'EdgeColor', 'none', 'FaceAlpha', 0.5);
        plot(t, mean(at, 2), 'r-', 'LineWidth', 1.5);
        plot(t, mu_h, 'b-', 'LineWidth', 1.5);
        plot(t, at(:, i7), 'r:', 'LineWidth', 0.8);
        plot(t, ah(:, i7), 'b:', 'LineWidth', 0.8);
        xlabel('t [s]'); ylabel('a\_bar (z) [-]');
        legend({'estimate mean \pm sd (100 seeds)', 'true mean', 'estimate mean', 'true seed 7', 'estimate seed 7'}, ...
               'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', 8);
        text(0.02, 0.95, lab2{c}, 'Units', 'normalized', 'FontSize', 10, 'Interpreter', 'none');
        xlim([t(1) t(end)]);
        % row 2: signature
        hx(2, c) = subplot(2, 2, 2 + c); hold on; box on;
        switch nm
            case {'echo_off', 'both_off'}
                i2 = squeeze(r.innov_y2(:, AX, :)); g = squeeze(r.gate(:, AX, :));
                i2(g) = NaN;
                mu2 = mean(i2, 2, 'omitnan'); sd2 = std(i2, 0, 2, 'omitnan');
                ok = isfinite(mu2);
                fill([t(ok); flipud(t(ok))], [mu2(ok) - sd2(ok); flipud(mu2(ok) + sd2(ok))], [0.6 0.75 1], 'EdgeColor', 'none', 'FaceAlpha', 0.4);
                plot(t, movmean(i2(:, i7), 32, 'omitnan'), 'Color', [0.5 0.7 1], 'LineWidth', 0.6);
                plot(t, mu2, 'b-', 'LineWidth', 1.5);
                yline(0, 'k-');
                xline(w.hold(1), 'k--');
                xlabel('t [s]'); ylabel('innov_{y2} (z) [-]');
                legend({'mean \pm sd (100 seeds)', 'seed 7 (20 ms mov. mean)', 'mean'}, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', 8);
                xlim([t(1) t(end)]);
            case 'ma2_off'
                q = S.(arms2{c});
                mu_a = mean(q.acf_hold, 1); se_a = zeros(size(mu_a));
                for L = 1:numel(ACF_LAGS); se_a(L) = local_sem(q.acf_hold(:, L)); end
                bar(ACF_LAGS, mu_a, 0.6, 'FaceColor', [0.3 0.5 0.9]);
                errorbar(ACF_LAGS, mu_a, se_a, 'k.', 'LineWidth', 1);
                yline(0.06, 'k--'); yline(-0.06, 'k--');
                xlabel('lag [steps]'); ylabel('innov_{y1} acf, final hold (z)');
                legend({'mean over 100 seeds', 'SEM', '\pm0.06'}, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', 8);
                xlim([0.5 numel(ACF_LAGS) + 0.5]);
            case 'fe4_off'
                kb = abs(squeeze(r.K_b_y1(:, AX, :)));
                mu_k = mean(kb, 2); sd_k = std(kb, 0, 2);
                fill([t; flipud(t)], [max(mu_k - sd_k, 0); flipud(mu_k + sd_k)], [0.6 0.75 1], 'EdgeColor', 'none', 'FaceAlpha', 0.4);
                plot(t, mu_k, 'b-', 'LineWidth', 1.2);
                for i = 1:numel(tp.idx); xline(t(tp.idx(i)), 'k--'); end
                xlabel('t [s]'); ylabel('|K_{b,y1}| (z)');
                legend({'mean \pm sd (100 seeds)', 'mean', 'turning points'}, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', 8);
                xlim([w.descent(2) - 0.1, w.osc(2) + 0.1]);
        end
    end
    linkaxes(hx(1, :), 'y'); linkaxes(hx(2, :), 'y');
    out_png = fullfile(FIG_DIR, sprintf('l2_loo_%s.png', nm));
    exportgraphics(fig, out_png, 'Resolution', 150);
    fprintf('figure: %s\n', out_png);
end
fprintf('\nL2 LOO JUDGE DONE\n');

%% ================= local helpers =================
function r = local_load(fn)
    L = load(fn);
    r = l0_load_formC_run(fn, struct('seeds', L.seeds, 'verbose', true));
    % attach the channels the budget-layout loader does not map; drop the same init row
    nd = r.meta.init_rows_dropped;
    extra = {'K_b_y1', 'K_b_y1_out', 1; 'K_b_y2', 'K_b_y2_out', 1; 'h_bar_d', 'h_bar_d_out', 1; ...
             'h_bar_true', 'h_bar_true_out', 1; 'a_hat', 'a_hat_out', 1; ...
             'P_b', 'P_b_out', 'sq'; 'P_a', 'P_a_out', 'sq_anom'};
    for i = 1:size(extra, 1)
        if ~isfield(L, extra{i, 2}); continue; end
        X = double(L.(extra{i, 2}));
        if ischar(extra{i, 3})
            switch extra{i, 3}
                case 'sq';      X = X.^2;
                case 'sq_anom'; X = (X / L.a_nom).^2;
            end
        end
        r.(extra{i, 1}) = X(1 + nd:end, :, :);
    end
    r.ns = size(r.a_true_norm, 3); r.seeds = L.seeds; r.K = L.K; r.a_nom = L.a_nom;
    r.metrics = L.metrics; r.windows = L.windows; r.arm = L.arm; r.override = L.override;
    if isfield(L, 'fixture_maxdiff'); r.fixture_maxdiff = L.fixture_maxdiff; end
    assert(r.ns == numel(L.seeds), 'seed count mismatch');
end

function v = local_gfd(s, f, d)
    if isfield(s, f); v = s.(f); else; v = d; end
end

function s = local_src(cc, fl)
    pres = fl(isfield(cc, fl));
    if isempty(pres); s = 'all from controller defaults'; else; s = ['from ctrl_const: ' strjoin(pres, ' ')]; end
end

function tp = local_turning_points(hd, t, rng_t)
    d = diff(hd); sgn = sign(d); sgn(sgn == 0) = NaN;
    sgn = fillmissing(sgn, 'previous'); sgn = fillmissing(sgn, 'next');
    ch = find(sgn(1:end-1) ~= sgn(2:end)) + 1;    % index of extremum sample
    ch = ch(t(ch) >= rng_t(1) & t(ch) <= rng_t(2));
    keep = true(size(ch));
    for i = 2:numel(ch); if ch(i) - ch(i-1) < 20; keep(i) = false; end; end
    ch = ch(keep);
    tp.idx = ch;
    tp.is_top = false(size(ch));
    for i = 1:numel(ch)
        lo = max(1, ch(i) - 5); hi = min(numel(hd), ch(i) + 5);
        tp.is_top(i) = hd(ch(i)) >= max(hd(lo:hi)) - 1e-12;
    end
end

function a = local_acf(x, lags)
    x = x(:); x = x - mean(x); den = sum(x.^2);
    a = zeros(1, numel(lags));
    for i = 1:numel(lags)
        L = lags(i);
        a(i) = sum(x(1:end-L) .* x(1+L:end)) / den;
    end
end

function se = local_sem(x)
    x = x(:); x = x(isfinite(x));
    if numel(x) < 2; se = NaN; return; end
    [~, se] = l0_jackknife_se(x);
end

function P = local_pair(off, on, name)
    d = off - on;
    [mu, se] = l0_jackknife_se(d);
    P = struct('mu', mu, 'se', se, 'sd', std(d, 0, 'omitnan'), 't', mu / se, 'n', sum(isfinite(d)), ...
               'on_mu', mean(on, 'omitnan'), 'on_sd', std(on, 0, 'omitnan'), 'on_se', local_sem(on), ...
               'off_mu', mean(off, 'omitnan'), 'off_sd', std(off, 0, 'omitnan'), 'off_se', local_sem(off));
    fprintf('  %-28s ON %+.5g (sd %.4g SEM %.4g) | OFF %+.5g (sd %.4g SEM %.4g) | diff %+.5g sd %.4g SEM %.4g t=%+.2f (n=%d)\n', ...
            name, P.on_mu, P.on_sd, P.on_se, P.off_mu, P.off_sd, P.off_se, P.mu, P.sd, P.se, P.t, P.n);
end

function s = local_cnt(n)
    s = sprintf('%d/%d/%d', sum(n), sum(n > 0), max(n));
end

function s = local_lt(a, b)
    if a < b; s = '<'; else; s = '>='; end
end

function s = local_candidate(vd, tval)
    if strcmp(vd, 'EXPIRED') && abs(tval) < 1
        s = 'REMOVAL CANDIDATE (report only)';
    elseif strcmp(vd, 'REPRODUCED')
        s = 'retained (signature holds on this base)';
    else
        s = 'open (back to ledger)';
    end
end
