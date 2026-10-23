% run_formB_ws_drift_ens_ablation.m -- PURPOSE: ensemble verification of the
%   long-run wall-estimate drift ("defect 3", ~-0.15 %/s) decomposition, plus
%   the y2-channel ablation arm. So far the split was measured on seed 7 only:
%       TERM-A = y2 channel, mean(K)*mean(innov) rectification (late window)
%       TERM-B = y1 channel, cov(K, innov) rectification      (mid window)
%   Here both terms are ensemble-averaged over 15 fresh seeds (7001:7015) in two
%   arms, baseline (y2 on) and y2 OFF, same seeds => paired comparison.
%
%   PRE-REGISTERED PREDICTION, round 1 (seeds 7001:7006, written before those
%   runs): with y2 off TERM-A cannot exist, so the y2-off ensemble drift rate
%   must differ from the baseline rate by approximately the baseline ensemble
%   TERM-A rate -- same sign, magnitude within ~40 %.
%   Round-1 outcome: mid FAIL (sign), late PASS but unresolved (|t| < 1.3), and
%   the prediction was found to be under-specified -- switching the channel off
%   removes the WHOLE y2 rate, not just its mean-x-mean part.
%
%   REGISTERED PREDICTION, round 2 (extension seeds 7007:7015, team-lead
%   accepted 2026-08-04 before those runs): the paired drop equals the FULL
%   baseline y2 rate, TERM-A + (y2 cov) -- same sign, magnitude within ~40 %.
%   The round-1 TERM-A-only test is kept and still reported, as the legacy
%   comparison. Sizing: round 1 gave a paired-drop per-seed sd ~0.094 %/s
%   against a drop ~0.048 %/s, i.e. ~15 pairs for a 2-sigma resolution.
%
%   Channel split convention (exact and additive by construction):
%       the w_s state has no process noise and a unit predict row, so its whole
%       motion is dws_y1 + dws_y2 = K1(7)*innov1 + K2(7)*innov2. Per window,
%       with the mean taken over ALL window samples (dws = 0 exactly where a
%       channel is inactive, e.g. the near-wall y2 gate):
%           rate_tot = mean_window(dws) * fs
%           rate_mm  = duty * mean(K) * mean(innov) * fs   (mean-x-mean part)
%           rate_cov = rate_tot - rate_mm  == duty * cov(K, innov) * fs
%       duty = active fraction; K = dws./innov on |innov| > 1e-12 only (K is the
%       Kalman gain entry itself, so the ratio is well conditioned there).
%       Stored rates are in w_bar units per second; the console and the figures
%       print 100x that, i.e. % of R per second.
%
%   Incremental save/resume: done-flags in test_results/formB_drift_ens_ablation.mat.
%   Re-running the script resumes and finally prints the stats + writes figures.
%   EXPIRES: absorbed into the defect-3 (drift) case file.
%   Style: .claude/rules/figure-style.md.

here = fileparts(mfilename('fullpath'));
proj = fullfile(here, '..', '..');
pdirs = {'model', 'model/dual_track', 'model/diag', 'model/thermal_force', ...
         'model/config', 'model/wall_effect', 'model/controller', ...
         'model/trajectory', 'test_script/integration'};
for i = 1:numel(pdirs); addpath(fullfile(proj, pdirs{i})); end
res_dir  = fullfile(proj, 'test_results');
if ~exist(res_dir, 'dir'); mkdir(res_dir); end
res_file = fullfile(res_dir, 'formB_drift_ens_ablation.mat');

% ------------------------------------------------------------------
% Protocol constants
% ------------------------------------------------------------------
SEEDS   = 7001:7015;          % fresh band (disjoint from 4000/5000 bands);
                              % 7001:7006 round 1, 7007:7015 the extension
N_SEED  = numel(SEEDS);
N_ARM   = 2;                  % 1 = baseline (y2 on), 2 = y2 OFF
ARM_NAME = {'baseline (y2 on)', 'y2 OFF'};
AX      = 3;                  % z = wall-normal axis
INJ     = 0.05;               % [R] TRUE wall offset injected on the plant side
N_CYC   = 32;                 % long identification run
T_SIM   = 34.8;               % [s] = 0.5 hold + 1.0 descend + 32 cycles + 1.3
DS      = 20;                 % trajectory downsample for storage/plots
WINS    = [13.5, 23.5; 23.5, 33.5];   % [s] mid / late analysis windows
WIN_NAME = {'mid', 'late'};
N_WIN   = size(WINS, 1);
INNOV_EPS = 1e-12;            % |innov| floor for forming K = dws/innov
PCT     = 100;                % w_bar -> % of R
TOL_REL = 0.40;               % pre-registered magnitude tolerance
FS      = 18;                 % figure font size

ov = struct('lock_b', false, 'lock_p', true, 'lock_ws', false, 'Pf_ws_std', 0.111);

% ------------------------------------------------------------------
% Resume state
% ------------------------------------------------------------------
if exist(res_file, 'file')
    load(res_file, 'E');
    % Seed-list extension: keep every finished run, grow the arrays. The stored
    % seeds must be a prefix of the current list, otherwise the cache belongs to
    % a different experiment and silently reusing it would corrupt the ensemble.
    n_old = size(E.done, 2);
    assert(n_old <= N_SEED && isequal(E.seeds(:).', SEEDS(1:n_old)), ...
           'drift_ens:seedMismatch', ...
           'cached seeds are not a prefix of SEEDS -- refusing to reuse %s', res_file);
    if n_old < N_SEED
        n_add = N_SEED - n_old;
        f2 = {'done', 'dev_end', 'sqP_end'};
        for j = 1:numel(f2)
            E.(f2{j}) = [E.(f2{j}), zeros(N_ARM, n_add)];
        end
        f3 = {'rate_emp', 'y1_tot', 'y1_mm', 'y1_cov', 'y2_tot', 'y2_mm', ...
              'y2_cov', 'y2_duty'};
        for j = 1:numel(f3)
            E.(f3{j}) = cat(2, E.(f3{j}), zeros(N_ARM, n_add, N_WIN));
        end
        E.dev{N_ARM, N_SEED} = [];      % grows the cell, pads with []
        E.sqP{N_ARM, N_SEED} = [];
        E.seeds = SEEDS;
        save(res_file, 'E');
        fprintf('cache extended: %d seeds -> %d (kept %d finished runs)\n', ...
                n_old, N_SEED, nnz(E.done));
    end
else
    E = struct();
    E.done     = zeros(N_ARM, N_SEED);
    E.dev_end  = zeros(N_ARM, N_SEED);
    E.sqP_end  = zeros(N_ARM, N_SEED);
    E.rate_emp = zeros(N_ARM, N_SEED, N_WIN);   % dev slope over the window [%/s]
    E.y1_tot   = zeros(N_ARM, N_SEED, N_WIN);   % all *_tot/_mm/_cov in w_bar/s
    E.y1_mm    = zeros(N_ARM, N_SEED, N_WIN);
    E.y1_cov   = zeros(N_ARM, N_SEED, N_WIN);
    E.y2_tot   = zeros(N_ARM, N_SEED, N_WIN);
    E.y2_mm    = zeros(N_ARM, N_SEED, N_WIN);
    E.y2_cov   = zeros(N_ARM, N_SEED, N_WIN);
    E.y2_duty  = zeros(N_ARM, N_SEED, N_WIN);
    E.dev      = cell(N_ARM, N_SEED);
    E.sqP      = cell(N_ARM, N_SEED);
    E.tds      = [];
    E.seeds    = SEEDS;
    E.wins     = WINS;
end

% ------------------------------------------------------------------
% Run loop (12 runs, resumable)
% ------------------------------------------------------------------
for arm = 1:N_ARM
    for i = 1:N_SEED
        if E.done(arm, i); continue; end
        o = struct('seeds', SEEDS(i), 'ws_inject', INJ, 'verbose', false, ...
                   'ctrl_const_override', ov, ...
                   'config_override', struct('n_cycles', N_CYC, 'T_sim', T_SIM));
        if arm == 2; o.y2_on = false; end

        out = run_formB_ws(o);
        r   = out.runs{1};

        t   = r.tout(:);
        fs  = 1 / (t(2) - t(1));                      % [Hz]
        dev = PCT * (r.ws_hat_out(:, AX) - 1);        % [% of R]
        sqP = PCT * r.P_ws_out(:, AX);                % [% of R] (ALREADY std)

        E.tds        = t(1:DS:end);
        E.dev{arm, i} = dev(1:DS:end);
        E.sqP{arm, i} = sqP(1:DS:end);
        E.dev_end(arm, i) = dev(end);
        E.sqP_end(arm, i) = sqP(end);

        d1 = r.dws_y1_out(:, AX);  n1 = r.innov_y1_out(:, AX);
        d2 = r.dws_y2_out(:, AX);  n2 = r.innov_y2_out(:, AX);
        for wI = 1:N_WIN
            m = t >= WINS(wI, 1) & t < WINS(wI, 2);
            assert(nnz(m) > 10, 'drift_ens:emptyWindow', 'window %d empty', wI);
            tw = t(m);
            dw = dev(m);
            E.rate_emp(arm, i, wI) = (dw(end) - dw(1)) / (tw(end) - tw(1));
            s1 = local_channel_split(d1, n1, m, fs, INNOV_EPS);
            s2 = local_channel_split(d2, n2, m, fs, INNOV_EPS);
            E.y1_tot(arm, i, wI) = s1.tot;  E.y1_mm(arm, i, wI) = s1.mm;
            E.y1_cov(arm, i, wI) = s1.cov;
            E.y2_tot(arm, i, wI) = s2.tot;  E.y2_mm(arm, i, wI) = s2.mm;
            E.y2_cov(arm, i, wI) = s2.cov;  E.y2_duty(arm, i, wI) = s2.duty;
        end

        E.done(arm, i) = 1;
        save(res_file, 'E');
        fprintf(['[arm %d %s seed %d] dev_end %+.3f  sqP %.3f | ' ...
                 'mid tot %+.4f (A %+.4f B %+.4f) | late tot %+.4f (A %+.4f B %+.4f) %%/s\n'], ...
                arm, ARM_NAME{arm}, SEEDS(i), E.dev_end(arm, i), E.sqP_end(arm, i), ...
                PCT * (E.y1_tot(arm, i, 1) + E.y2_tot(arm, i, 1)), ...
                PCT * E.y2_mm(arm, i, 1), PCT * E.y1_cov(arm, i, 1), ...
                PCT * (E.y1_tot(arm, i, 2) + E.y2_tot(arm, i, 2)), ...
                PCT * E.y2_mm(arm, i, 2), PCT * E.y1_cov(arm, i, 2));
    end
end

if ~all(E.done(:))
    fprintf('INCOMPLETE: %d/%d runs done -- rerun this script to resume.\n', ...
            nnz(E.done), numel(E.done));
    return;
end

% ------------------------------------------------------------------
% Ensemble statistics (mean +- SEM over the 6 seeds)
% ------------------------------------------------------------------
msem = @(v) deal(mean(v), std(v) / sqrt(numel(v)));
rate_tot = PCT * (E.y1_tot + E.y2_tot);      % [%/s] channel-sum drift rate
termA    = PCT * E.y2_mm;                    % [%/s] y2 mean-x-mean part
termB    = PCT * E.y1_cov;                   % [%/s] y1 covariance part
rate_emp = E.rate_emp;                       % [%/s] measured dev slope

fprintf('\n================ ENSEMBLE (%d fresh seeds %d:%d, injected wall +%.0f %%) ================\n', ...
        N_SEED, SEEDS(1), SEEDS(end), PCT * INJ);
for arm = 1:N_ARM
    [m_end, s_end] = msem(E.dev_end(arm, :));
    [m_sqP, ~]     = msem(E.sqP_end(arm, :));
    fprintf('\n-- arm %d: %s --\n', arm, ARM_NAME{arm});
    fprintf('  end dev            : %+.3f +- %.3f %% (SEM)   [claimed sqrtP %.3f %%]\n', ...
            m_end, s_end, m_sqP);
    for wI = 1:N_WIN
        [mt, st] = msem(squeeze(rate_tot(arm, :, wI)));
        [me, se] = msem(squeeze(rate_emp(arm, :, wI)));
        [ma, sa] = msem(squeeze(termA(arm, :, wI)));
        [mb, sb] = msem(squeeze(termB(arm, :, wI)));
        [mo, ~]  = msem(squeeze(PCT * E.y1_mm(arm, :, wI)));
        [mc, ~]  = msem(squeeze(PCT * E.y2_cov(arm, :, wI)));
        [md, ~]  = msem(squeeze(E.y2_duty(arm, :, wI)));
        fprintf('  %-4s window [%.1f, %.1f) s\n', WIN_NAME{wI}, WINS(wI, 1), WINS(wI, 2));
        fprintf('    total drift rate : %+.4f +- %.4f %%/s  (measured slope %+.4f +- %.4f)\n', ...
                mt, st, me, se);
        fprintf('    TERM-A y2 meanxmean : %+.4f +- %.4f %%/s   (y2 duty %.3f)\n', ma, sa, md);
        fprintf('    TERM-B y1 cov       : %+.4f +- %.4f %%/s\n', mb, sb);
        fprintf('    residual parts      : y1 meanxmean %+.4f, y2 cov %+.4f %%/s\n', mo, mc);
    end
end

% ------------------------------------------------------------------
% Ablation verdict (paired: same seeds in both arms)
% ------------------------------------------------------------------
fprintf('\n================ ABLATION VERDICT ================\n');
fprintf('REGISTERED (round 2): rate(baseline) - rate(y2 off) ~= FULL y2 rate = TERM-A + y2 cov,\n');
fprintf('                      same sign, magnitude within %.0f %%\n', PCT * TOL_REL);
for wI = 1:N_WIN
    d_pair   = squeeze(rate_tot(1, :, wI) - rate_tot(2, :, wI));
    [md, sd] = msem(d_pair);
    [mf, sf] = msem(squeeze(PCT * (E.y2_mm(1, :, wI) + E.y2_cov(1, :, wI))));
    rel = abs(md - mf) / max(abs(mf), eps);
    ok_sign = (sign(md) == sign(mf)) && mf ~= 0;
    if ok_sign && rel <= TOL_REL
        verdict = 'PASS';
    elseif ok_sign
        verdict = 'FAIL (magnitude)';
    else
        verdict = 'FAIL (sign)';
    end
    fprintf('%-4s: observed drop %+.4f +- %.4f %%/s (paired t %.2f) | full y2 rate %+.4f +- %.4f %%/s | rel dev %.2f -> %s\n', ...
            WIN_NAME{wI}, md, sd, md / max(sd, eps), mf, sf, rel, verdict);
end

fprintf('\nLEGACY (round 1, under-specified): drop ~= TERM-A alone\n');
for wI = 1:N_WIN
    d_pair   = squeeze(rate_tot(1, :, wI) - rate_tot(2, :, wI));
    [md, sd] = msem(d_pair);
    [ma, sa] = msem(squeeze(termA(1, :, wI)));
    if abs(ma) > 0
        rel = abs(md - ma) / abs(ma);
    else
        rel = Inf;
    end
    ok_sign = (sign(md) == sign(ma)) && ma ~= 0;
    if ok_sign && rel <= TOL_REL
        verdict = 'PASS';
    elseif ok_sign
        verdict = 'FAIL (magnitude)';
    else
        verdict = 'FAIL (sign)';
    end
    fprintf('%-4s: observed drop %+.4f +- %.4f %%/s (paired t %.2f) | predicted (TERM-A) %+.4f +- %.4f %%/s | rel dev %.2f -> %s\n', ...
            WIN_NAME{wI}, md, sd, md / max(sd, eps), ma, sa, rel, verdict);
end

% Round-1 subset vs the whole ensemble: did the extension move the estimate, or
% only shrink its error bar? (Same quantities, seeds 1:6 only.)
N_R1 = 6;
if N_SEED > N_R1
    fprintf('\nround-1 subset (seeds 1:%d) vs full ensemble (N=%d):\n', N_R1, N_SEED);
    for wI = 1:N_WIN
        d_all = squeeze(rate_tot(1, :, wI) - rate_tot(2, :, wI));
        [ma, sa] = msem(d_all);
        [m1, s1] = msem(d_all(1:N_R1));
        fprintf('%-4s: paired drop  N=%d %+.4f +- %.4f  ->  N=%d %+.4f +- %.4f %%/s\n', ...
                WIN_NAME{wI}, N_R1, m1, s1, N_SEED, ma, sa);
    end
end

% Is anything here resolved? One-sample t-ratio of each ensemble mean.
if exist('tinv', 'file')
    t_crit = tinv(0.975, N_SEED - 1);
else
    t_crit = 1.96;   % normal fallback (Statistics Toolbox absent)
end
fprintf('\nresolution check (t = mean/SEM, |t| > %.2f = 95%% two-sided at N=%d):\n', ...
        t_crit, N_SEED);
for arm = 1:N_ARM
    for wI = 1:N_WIN
        [mt, st] = msem(squeeze(rate_tot(arm, :, wI)));
        [ma, sa] = msem(squeeze(termA(arm, :, wI)));
        [mb, sb] = msem(squeeze(termB(arm, :, wI)));
        fprintf('  arm %d %-4s: total t %+.2f | TERM-A t %+.2f | TERM-B t %+.2f\n', ...
                arm, WIN_NAME{wI}, mt / max(st, eps), ma / max(sa, eps), mb / max(sb, eps));
    end
end

% ------------------------------------------------------------------
% Figures
% ------------------------------------------------------------------
shade = [linspace(0.62, 0.00, N_SEED)', linspace(0.78, 0.20, N_SEED)', ...
         linspace(0.96, 0.55, N_SEED)'];
fig_paths = cell(1, 3);
for arm = 1:N_ARM
    figT = figure('Color', 'w', 'Position', [80 60 980 600]); drawnow;
    axT = axes(figT); hold(axT, 'on');
    M = zeros(numel(E.tds), N_SEED);
    for i = 1:N_SEED
        M(:, i) = E.dev{arm, i};
        hs = plot(axT, E.tds, M(:, i), '-', 'Color', shade(i, :), 'LineWidth', 1.2);
        if i == 1; h_seed = hs; end
    end
    h_mean = plot(axT, E.tds, mean(M, 2), 'k--', 'LineWidth', 2.4);
    h_true = plot(axT, [E.tds(1) E.tds(end)], PCT * INJ * [1 1], 'r-', 'LineWidth', 2.4);
    xlabel(axT, 'time  [s]', 'FontSize', FS, 'FontWeight', 'bold');
    ylabel(axT, '(w_s estimate - 1)  [% of R]', 'FontSize', FS, 'FontWeight', 'bold');
    set(axT, 'FontSize', FS, 'FontWeight', 'bold'); box(axT, 'on');
    xlim(axT, [E.tds(1), E.tds(end)]);
    legend(axT, [h_seed h_mean h_true], ...
           {sprintf('per-seed estimate (%d seeds)', N_SEED), 'ensemble mean', 'true wall'}, ...
           'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS - 2);
    if arm == 1
        fig_paths{1} = fullfile(res_dir, 'formB_drift_ens_baseline.png');
        exportgraphics(figT, fig_paths{1}, 'Resolution', 150);
    else
        fig_paths{2} = fullfile(res_dir, 'formB_drift_ens_y2off.png');
        exportgraphics(figT, fig_paths{2}, 'Resolution', 150);
    end
end

% grouped bars: [TERM-A, TERM-B, total] x [baseline, y2off], mid then late
cats = cell(1, 2 * 3);
bar_mu  = zeros(2 * 3, N_ARM);
bar_sem = zeros(2 * 3, N_ARM);
row = 0;
for wI = 1:N_WIN
    for term = 1:3
        row = row + 1;
        switch term
            case 1; src = termA;    lbl = 'TERM-A';
            case 2; src = termB;    lbl = 'TERM-B';
            case 3; src = rate_tot; lbl = 'total';
        end
        cats{row} = sprintf('%s (%s)', lbl, WIN_NAME{wI});
        for arm = 1:N_ARM
            v = squeeze(src(arm, :, wI));
            bar_mu(row, arm)  = mean(v);
            bar_sem(row, arm) = std(v) / sqrt(N_SEED);
        end
    end
end
figB = figure('Color', 'w', 'Position', [80 60 1040 620]); drawnow;
axB = axes(figB); hold(axB, 'on');
hb = bar(axB, bar_mu, 'grouped');
set(hb(1), 'FaceColor', [0.10 0.25 0.75]);
set(hb(2), 'FaceColor', [0.45 0.72 0.95]);
for arm = 1:N_ARM
    errorbar(axB, hb(arm).XEndPoints, bar_mu(:, arm)', bar_sem(:, arm)', ...
             'k', 'LineStyle', 'none', 'LineWidth', 1.4, 'CapSize', 8);
end
plot(axB, [0.4, size(bar_mu, 1) + 0.6], [0 0], 'k-', 'LineWidth', 0.8);
set(axB, 'XTick', 1:size(bar_mu, 1), 'XTickLabel', cats, 'FontSize', FS, ...
    'FontWeight', 'bold', 'XTickLabelRotation', 20);
xlim(axB, [0.4, size(bar_mu, 1) + 0.6]);
ylabel(axB, 'w_s drift rate  [%/s]', 'FontSize', FS, 'FontWeight', 'bold');
box(axB, 'on');
legend(axB, hb, ARM_NAME, 'Location', 'northoutside', 'Orientation', 'horizontal', ...
       'FontSize', FS - 2);
fig_paths{3} = fullfile(res_dir, 'formB_drift_term_rates.png');
exportgraphics(figB, fig_paths{3}, 'Resolution', 150);

fprintf('\nfigures:\n  %s\n  %s\n  %s\n', fig_paths{1}, fig_paths{2}, fig_paths{3});
fprintf('data: %s\n', res_file);


%% =================== Local Helpers ===================

function s = local_channel_split(dws, innov, mask, fs, innov_eps)
%LOCAL_CHANNEL_SPLIT  mean-x-mean vs covariance split of one w_s update channel.
%   dws = K(7)*innov per step [-]; mask = window selector; fs [Hz].
%   The window mean is taken over ALL masked samples (dws is exactly zero where
%   the channel is inactive), so tot = mm + cov holds identically and tot is the
%   drift rate this channel actually contributes.
    n_full = nnz(mask);
    v = mask & (abs(innov) > innov_eps);
    s.duty = nnz(v) / n_full;
    s.tot  = sum(dws(mask)) / n_full * fs;
    if nnz(v) < 2
        s.mm = 0; s.cov = s.tot; return;
    end
    K  = dws(v) ./ innov(v);
    s.mm  = s.duty * mean(K) * mean(innov(v)) * fs;
    s.cov = s.tot - s.mm;
end
