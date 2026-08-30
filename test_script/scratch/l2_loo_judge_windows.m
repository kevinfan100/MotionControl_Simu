% L2 LOO battery -- follow-up re-judge of member (a) y2_echo_corr on the
% DESCENT [0.5,1.5] s and OSCILLATION (1.6,3.5] s windows (post-processing
% only, no reruns). Reason: echo_fac scales only the back-off terms
% a_bar'*Grad_wbar_d in H2 / y2_pred, and Grad_wbar_d = 0 in the final hold,
% so the registered hold-window signature was structurally blind.
% STATUS: ACTIVE (scratch, L2 instrument) | PURPOSE: window re-judge of the
%   echo signature + consistency numbers for (b)/(c) on the same windows |
%   EXPIRES: stacked-fix audit closes.
% Reconstructions (controller motion_control_law_formC_b.m):
%   Grad_wbar_d[k] = (w_d[k] - w_d[k-d])/R = h_bar_d(k) - h_bar_d(k-2)   (:908, d=2)
%   a_bar'[k]      = b_hat[k-1] (1 - a_bar_hat[k-1])^2  on ap_src='post'  (:1021,:1057)
%   back-off term  = a_bar' * Grad_wbar_d  (the ONLY place echo_fac acts, :1268-1281)
cd('/Users/kevin/code/MotionControl_Simu-motion-test');
root = pwd;
addpath(fullfile(root, 'test_script', 'scratch'));
IN_DIR = fullfile(root, 'test_results', 'l2_loo');
AX = 3; D_DELAY = 2;
ARMS = {'base', 'echo_off', 'ma2_off', 'fe4_off', 'both_off'};
ACF_LAGS = 1:2;

A = struct();
for i = 1:numel(ARMS)
    A.(ARMS{i}) = local_load(fullfile(IN_DIR, sprintf('l2_%s_100.mat', ARMS{i})));
end
B = A.base; ns = B.ns; t = B.t; seeds = B.seeds; w = B.windows;
cc = B.K;
a_floor = local_gfd(cc, 'a_bar_floor', 0.05);  a_ceil = local_gfd(cc, 'a_bar_ceil', 1 - 1e-4);
b_floor = local_gfd(cc, 'b_floor', 0.60);      b_ceil = local_gfd(cc, 'b_ceil', 1.05);
WIN = {'descent', t >= w.descent(1) & t <= w.descent(2); ...
       'osc',     t >  w.osc(1)     & t <= w.osc(2); ...
       'hold',    t >  w.hold(1)};
fprintf('\n\n################ FOLLOW-UP: WINDOW RE-JUDGE OF (a) y2_echo_corr (post-processing, %s) ################\n', datestr(now, 'yyyy-mm-dd HH:MM')); %#ok<TNOW1,DATST>
fprintf('windows: descent [%.2f,%.2f] osc (%.2f,%.2f] hold (%.2f,end]\n', w.descent, w.osc, w.hold(1));

% ---- per-seed, per-window quantities for every arm ----
S = struct();
for i = 1:numel(ARMS)
    r = A.(ARMS{i});
    at = squeeze(r.a_true_norm(:, AX, :)); ah = squeeze(r.a_bar_hat(:, AX, :));
    i2 = squeeze(r.innov_y2(:, AX, :)); R2 = squeeze(r.R2(:, AX, :));
    ka2 = squeeze(r.K_a_y2(:, AX, :)); ka1 = squeeze(r.K_a_y1(:, AX, :));
    i1 = squeeze(r.innov_y1(:, AX, :));
    kb1 = squeeze(r.K_b_y1(:, AX, :));
    bh = squeeze(r.b_hat(:, AX, :)); hd = squeeze(r.h_bar_d(:, 1, :));
    % reconstructions
    Grad = zeros(size(hd)); Grad(1+D_DELAY:end, :) = hd(1+D_DELAY:end, :) - hd(1:end-D_DELAY, :);
    ah_prev = [ah(1, :); ah(1:end-1, :)];  bh_prev = [bh(1, :); bh(1:end-1, :)];
    ap = min(max(bh_prev, b_floor), b_ceil) .* (1 - min(max(ah_prev, a_floor), a_ceil)).^2;
    backoff = ap .* Grad;
    q = struct();
    q.a_T = mean(at(WIN{3, 2}, :), 1).';
    for wi = 1:size(WIN, 1)
        m = WIN{wi, 2}; nm = WIN{wi, 1};
        q.(nm).innov2_mean = mean(i2(m, :), 1).';
        q.(nm).var_ratio   = (var(i2(m, :), 0, 1) ./ mean(R2(m, :), 1)).';
        q.(nm).Ka2_abs     = mean(abs(ka2(m, :)), 1).';
        q.(nm).cum_y2      = sum(ka2(m, :) .* i2(m, :), 1).';          % Delta a_bar_hat from y2 [-]
        q.(nm).cum_y1      = sum(ka1(m, :) .* i1(m, :), 1).';          % same from y1 (context)
        q.(nm).backoff_rel_mean = mean(abs(backoff(m, :)) ./ max(ah(m, :), a_floor), 1).';
        q.(nm).backoff_rel_max  = max(abs(backoff(m, :)) ./ max(ah(m, :), a_floor), [], 1).';
        q.(nm).backoff_abs_mean = mean(abs(backoff(m, :)), 1).';
        q.(nm).grad_abs_mean    = mean(abs(Grad(m, :)), 1).';
        q.(nm).bias_pct    = (100 * mean(ah(m, :) - at(m, :), 1) ./ mean(at(m, :), 1)).';
        q.(nm).acf = zeros(ns, numel(ACF_LAGS));
        for s = 1:ns; q.(nm).acf(s, :) = local_acf(i1(m, s), ACF_LAGS); end
        q.(nm).Kb1_abs = mean(abs(kb1(m, :)), 1).';
    end
    S.(ARMS{i}) = q;
end

% ---- (a) echo: descent and oscillation windows ----
for wi = 1:2
    nm = WIN{wi, 1};
    fprintf('\n================ (a) y2_echo_corr, window %s: echo_off vs base (paired OFF - ON, n=%d) ================\n', upper(nm), ns);
    b = S.base.(nm); q = S.echo_off.(nm);
    fprintf('  back-off term |a_bar''*Grad| / a_bar_hat (baseline): mean-over-window %.4g (sd %.2g)  max-in-window %.4g (sd %.2g); |a_bar''*Grad| abs mean %.3e; |Grad| mean %.3e\n', ...
            mean(b.backoff_rel_mean), std(b.backoff_rel_mean), mean(b.backoff_rel_max), std(b.backoff_rel_max), ...
            mean(b.backoff_abs_mean), mean(b.grad_abs_mean));
    fprintf('  back-off term |a_bar''*Grad| / a_bar_hat (echo_off): mean-over-window %.4g  max-in-window %.4g\n', ...
            mean(q.backoff_rel_mean), mean(q.backoff_rel_max));
    P.innov2 = local_pair(q.innov2_mean, b.innov2_mean, 'innov2 mean');
    P.vr     = local_pair(q.var_ratio, b.var_ratio, 'Var(innov2)/mean R2');
    P.Ka2    = local_pair(q.Ka2_abs, b.Ka2_abs, '|K_a_y2| mean');
    fprintf('     |K_a_y2| ratio OFF/ON: median %.3f (IQR %.3f-%.3f)\n', median(q.Ka2_abs ./ b.Ka2_abs), ...
            prctile(q.Ka2_abs ./ b.Ka2_abs, 25), prctile(q.Ka2_abs ./ b.Ka2_abs, 75));
    P.cum    = local_pair(q.cum_y2, b.cum_y2, 'cum K2(4)*innov2 [a_bar]');
    P.cumpp  = local_pair(100 * q.cum_y2 ./ S.echo_off.a_T, 100 * b.cum_y2 ./ S.base.a_T, 'cum K2(4)*innov2 [pp of a_T]');
    P.cum1   = local_pair(q.cum_y1, b.cum_y1, 'cum K1(4)*innov1 [a_bar] (ctx)');
    P.bias   = local_pair(q.bias_pct, b.bias_pct, sprintf('%s-window bias %%', nm));
    V.(nm) = P;
end
fprintf('\n================ (a) RE-JUDGE on the registered thresholds, applied per window (post-hoc window change, declared) ================\n');
ahat_pp = local_pair(S.echo_off.hold.bias_pct, S.base.hold.bias_pct, 'hold bias_pct [pp] (a_hat effect, unchanged)');
for wi = 1:2
    nm = WIN{wi, 1}; P = V.(nm);
    okvr = mean(S.base.(nm).var_ratio) >= 0.8 && mean(S.base.(nm).var_ratio) <= 1.25 && ...
           mean(S.echo_off.(nm).var_ratio) >= 0.8 && mean(S.echo_off.(nm).var_ratio) <= 1.25;
    if abs(P.innov2.t) >= 3 && abs(ahat_pp.mu) < 2.0; vd = 'REPRODUCED'; ...
    elseif abs(P.innov2.t) < 2; vd = 'EXPIRED'; else; vd = 'OTHER'; end
    fprintf('  window %-8s innov2 offset diff t=%+.2f | Var ratio ON %.3f OFF %.3f (band met=%d, clause inert if 0) | |K_a_y2| OFF/ON %.3f | cum y2 contribution ON %+.4e OFF %+.4e diff t=%+.2f | a_hat %+.3f pp t=%+.2f  => %s (Q_a2 clause ignored)\n', ...
            nm, P.innov2.t, mean(S.base.(nm).var_ratio), mean(S.echo_off.(nm).var_ratio), okvr, ...
            median(S.echo_off.(nm).Ka2_abs ./ S.base.(nm).Ka2_abs), P.cum.on_mu, P.cum.off_mu, P.cum.t, ahat_pp.mu, ahat_pp.t, vd);
end

% ---- (b)/(c) consistency on the same windows ----
fprintf('\n================ (b)/(c) consistency on descent / osc windows (paired OFF - ON) ================\n');
for i = 2:numel(ARMS)
    an = ARMS{i};
    for wi = 1:2
        nm = WIN{wi, 1}; b = S.base.(nm); q = S.(an).(nm);
        d1 = local_stat(q.acf(:, 1) - b.acf(:, 1)); d2 = local_stat(q.acf(:, 2) - b.acf(:, 2));
        dk = local_stat(q.Kb1_abs - b.Kb1_abs); dc = local_stat(q.cum_y2 - b.cum_y2); db = local_stat(q.bias_pct - b.bias_pct);
        fprintf('  %-9s %-8s acf1 ON %+.4f OFF %+.4f (t %+.1f) | acf2 ON %+.4f OFF %+.4f (t %+.1f) | |K_b_y1| ON %.4f OFF %.4f ratio %.3f (t %+.1f) | cum y2 ON %+.3e OFF %+.3e (t %+.1f) | window bias ON %+.3f OFF %+.3f diff %+.3f (t %+.2f) | |K_a_y2| OFF/ON %.3f\n', ...
                an, nm, mean(b.acf(:, 1)), mean(q.acf(:, 1)), d1.t, mean(b.acf(:, 2)), mean(q.acf(:, 2)), d2.t, ...
                mean(b.Kb1_abs), mean(q.Kb1_abs), median(q.Kb1_abs ./ b.Kb1_abs), dk.t, ...
                mean(b.cum_y2), mean(q.cum_y2), dc.t, mean(b.bias_pct), mean(q.bias_pct), db.mu, db.t, ...
                median(q.Ka2_abs ./ b.Ka2_abs));
    end
end

% ---- echo figure: row 2 = innov2 over the OSC window ----
close all;
i7 = find(seeds == 7, 1);
fig = figure('Position', [50 50 1500 900], 'Color', 'w');
arms2 = {'base', 'echo_off'}; lab2 = {'baseline (production)', 'y2_echo_corr OFF'};
hx = gobjects(2, 2);
m_osc = WIN{2, 2};
for c = 1:2
    r = A.(arms2{c});
    at = squeeze(r.a_true_norm(:, AX, :)); ah = squeeze(r.a_bar_hat(:, AX, :));
    hx(1, c) = subplot(2, 2, c); hold on; box on;
    mu_h = mean(ah, 2); sd_h = std(ah, 0, 2);
    fill([t; flipud(t)], [mu_h - sd_h; flipud(mu_h + sd_h)], [0.6 0.75 1], 'EdgeColor', 'none', 'FaceAlpha', 0.5);
    plot(t, mean(at, 2), 'r-', 'LineWidth', 1.5); plot(t, mu_h, 'b-', 'LineWidth', 1.5);
    plot(t, at(:, i7), 'r:', 'LineWidth', 0.8); plot(t, ah(:, i7), 'b:', 'LineWidth', 0.8);
    xlabel('t [s]'); ylabel('a\_bar (z) [-]');
    legend({'estimate mean \pm sd (100 seeds)', 'true mean', 'estimate mean', 'true seed 7', 'estimate seed 7'}, ...
           'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', 8);
    text(0.02, 0.95, lab2{c}, 'Units', 'normalized', 'FontSize', 10, 'Interpreter', 'none');
    xlim([t(1) t(end)]);
    hx(2, c) = subplot(2, 2, 2 + c); hold on; box on;
    i2 = squeeze(r.innov_y2(:, AX, :));
    mu2 = mean(i2, 2); sd2 = std(i2, 0, 2);
    fill([t; flipud(t)], [mu2 - sd2; flipud(mu2 + sd2)], [0.6 0.75 1], 'EdgeColor', 'none', 'FaceAlpha', 0.4);
    plot(t, movmean(i2(:, i7), 32), 'Color', [0.5 0.7 1], 'LineWidth', 0.6);
    plot(t, mu2, 'b-', 'LineWidth', 1.2);
    plot(t, movmean(mu2, 160), 'k-', 'LineWidth', 1.5);
    yline(0, 'k:');
    xlabel('t [s]'); ylabel('innov_{y2} (z) [-], osc window');
    legend({'mean \pm sd (100 seeds)', 'seed 7 (20 ms mov. mean)', 'mean', 'mean, 100 ms mov. mean'}, ...
           'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', 8);
    xlim([w.osc(1) w.osc(2)]);
end
linkaxes(hx(1, :), 'y'); linkaxes(hx(2, :), 'y');
out_png = fullfile(IN_DIR, 'l2_loo_echo_off.png');
exportgraphics(fig, out_png, 'Resolution', 150);
fprintf('figure (regenerated, row 2 = innov2 over the osc window): %s\n', out_png);
fprintf('\nL2 LOO WINDOW RE-JUDGE DONE\n');

%% ================= local helpers =================
function r = local_load(fn)
    L = load(fn);
    r = l0_load_formC_run(fn, struct('seeds', L.seeds, 'verbose', false));
    nd = r.meta.init_rows_dropped;
    extra = {'K_b_y1', 'K_b_y1_out'; 'K_b_y2', 'K_b_y2_out'; 'h_bar_d', 'h_bar_d_out'};
    for i = 1:size(extra, 1)
        X = double(L.(extra{i, 2})); r.(extra{i, 1}) = X(1 + nd:end, :, :);
    end
    r.ns = size(r.a_true_norm, 3); r.seeds = L.seeds; r.K = L.K; r.windows = L.windows;
end
function v = local_gfd(s, f, d)
    if isfield(s, f); v = s.(f); else; v = d; end
end
function a = local_acf(x, lags)
    x = x(:); x = x - mean(x); den = sum(x.^2); a = zeros(1, numel(lags));
    for i = 1:numel(lags); L = lags(i); a(i) = sum(x(1:end-L) .* x(1+L:end)) / den; end
end
function st = local_stat(d)
    [mu, se] = l0_jackknife_se(d(:)); st = struct('mu', mu, 'se', se, 't', mu / se);
end
function P = local_pair(off, on, name)
    d = off - on; [mu, se] = l0_jackknife_se(d);
    [~, se_on] = l0_jackknife_se(on); [~, se_off] = l0_jackknife_se(off);
    P = struct('mu', mu, 'se', se, 'sd', std(d), 't', mu / se, 'on_mu', mean(on), 'off_mu', mean(off));
    fprintf('  %-34s ON %+.5g (sd %.3g SEM %.3g) | OFF %+.5g (sd %.3g SEM %.3g) | diff %+.5g sd %.3g SEM %.3g t=%+.2f\n', ...
            name, mean(on), std(on), se_on, mean(off), std(off), se_off, mu, std(d), se, mu / se);
end
