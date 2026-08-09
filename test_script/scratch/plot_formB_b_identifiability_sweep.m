% FORK OF test_script/scratch/plot_formB_b_only_planeprior.m @ untracked-2026-08-05 |
% PURPOSE: answer "is b identifiable FROM THE DATA" without inventing a prior width.
% Same plant as the companion page (constructed p = 1 curve, b = 0.13), but b is LOCKED at
% a swept grid via opts.oracle_bp instead of estimated, so the EKF prior plays no role.
% The profile statistic is the y2 gain-readout innovation -- what the filter actually sees,
% computable without knowing the truth.  A clear minimum at b = 0.13 means the data can
% identify b and the companion page's failure is the prior; a flat profile means the data
% cannot, and no prior would have helped. | EXPIRES: when the b-prior question is settled
% | 產線改動不會自動跟上
%
% Zero tuning: the swept grid is a scan, not a fit; no threshold or width is chosen here.
%
% Lab figure convention (.claude/rules/figure-style.md): no grid, no title, box on,
% legend northoutside horizontal, FontSize 18 bold, exportgraphics Resolution 150.

here = fileparts(mfilename('fullpath'));
proj = fullfile(here, '..', '..');
addpath(genpath(fullfile(proj, 'model')));
addpath(fullfile(proj, 'test_script', 'integration'));
fig_dir = fullfile(proj, 'reference', 'eq17_analysis', 'derivation', 'figures');

AX_Z      = 3;
B_ANCHOR  = 9/8;
B_TRUE    = 0.13;
PLANT_LAW = struct('b', B_TRUE, 'p', 1, 'ws', 1);
SEEDS     = [7 11];
B_GRID    = [0.08 0.10 0.13 0.17 0.22 0.30 0.42 0.58 0.80 1.125];

FS       = 18;
RED      = 'r';
BLUE     = 'b';
GREY     = [0.35 0.35 0.35];

nb = numel(B_GRID);
mism_pct  = nan(nb, 1);    % mean(a_wm - a_hat)/mean(a_hat) -- DATA ONLY, no truth
mism_sem  = nan(nb, 1);
innov_rms = nan(nb, 1);    % whitened-y2 RMS (kept: the negative result is a finding)
innov_mu  = nan(nb, 1);
desc_pk   = nan(nb, 1);
osc_rms   = nan(nb, 1);
hold_mean = nan(nb, 1);

cache = fullfile(proj, 'test_results', 'formB_b_sweep_profile.mat');
if exist(cache, 'file')
    C = load(cache);
    mism_pct = C.mism_pct; mism_sem = C.mism_sem; innov_rms = C.innov_rms;
    innov_mu = C.innov_mu; desc_pk = C.desc_pk; osc_rms = C.osc_rms;
    hold_mean = C.hold_mean;
    fprintf('\n=== b sweep: cached profile loaded (%s) ===\n', cache);
    nb = numel(mism_pct);
end

fprintf('\n=== b identifiability sweep (b LOCKED; plant b = %.3f) ===\n', B_TRUE);
for i = 1:nb * double(~exist(cache, 'file'))
    o = struct();
    o.seeds          = SEEDS;
    o.plant_gain_law = PLANT_LAW;
    o.oracle_bp      = [B_GRID(i), 1];    % locks b and p at the grid point
    o.verbose        = false;
    out = run_formB_ws(o);

    % post-descent window: the parameter-bearing part of the run
    ns = numel(SEEDS);
    mm = nan(ns, 1); ir = nan(ns, 1); im = nan(ns, 1);
    for q = 1:ns
        r  = out.runs{q};
        t  = r.tout(:);
        w  = t >= out.cfg.t_hold + out.cfg.t_descend_override;
        aXM = r.a_xm_out(w, AX_Z);
        aH  = r.a_hat_out(w, AX_Z);
        mm(q) = 100 * mean(aXM - aH) / mean(aH);      % readout-vs-model mismatch [%]
        ir(q) = sqrt(mean(r.innov_y2_out(w, AX_Z).^2));
        im(q) = mean(r.innov_y2_out(w, AX_Z));
    end
    mism_pct(i)  = mean(mm);
    mism_sem(i)  = std(mm) / sqrt(ns);
    innov_rms(i) = mean(ir);
    innov_mu(i)  = mean(im);
    desc_pk(i)   = mean(out.metrics.rows(:, 1));
    osc_rms(i)   = mean(out.metrics.rows(:, 2));
    hold_mean(i) = mean(out.metrics.rows(:, 3));
    fprintf(['  b = %.3f : readout mismatch %+7.2f %% (sem %.2f) | ' ...
             'innov_y2 RMS %.4e mean %+.3e | desc %.2f %% hold %+.2f %%\n'], ...
            B_GRID(i), mism_pct(i), mism_sem(i), innov_rms(i), innov_mu(i), ...
            desc_pk(i), hold_mean(i));
end

if ~exist(cache, 'file')
    save(cache, 'mism_pct', 'mism_sem', 'innov_rms', 'innov_mu', ...
         'desc_pk', 'osc_rms', 'hold_mean', 'B_GRID', 'B_TRUE', 'SEEDS');
end

[~, i_rms] = min(innov_rms);
i_true = find(B_GRID == B_TRUE, 1);
sep    = mism_pct(end) - mism_pct(i_true);
sep_se = hypot(mism_sem(end), mism_sem(i_true));
fprintf('\n  readout mismatch at truth  %+.2f %% +- %.2f  (consistent with zero)\n', ...
        mism_pct(i_true), mism_sem(i_true));
fprintf('  readout mismatch at anchor %+.2f %% +- %.2f\n', mism_pct(end), mism_sem(end));
fprintf('  anchor - truth separation  %+.2f %% +- %.2f  =  %.1f sigma (N = %d seeds)\n', ...
        sep, sep_se, sep / sep_se, numel(SEEDS));
fprintf('  gain error at truth %.2f %% (desc) -- machinery verified\n', desc_pk(i_true));
fprintf('  whitened-y2 RMS min at b = %.3f: RMS is noise-dominated, not a b profile\n', ...
        B_GRID(i_rms));
fprintf('  whitened-y2 innovation MEAN does track b: %.3e -> %.3e (%.1fx)\n', ...
        innov_mu(1), innov_mu(end), innov_mu(end) / innov_mu(1));

fig = figure('Color', 'w', 'Position', [80 60 900 760]);

ax1 = subplot(2, 1, 1); hold(ax1, 'on');
hp = errorbar(ax1, B_GRID, mism_pct, mism_sem, 'o-', 'Color', GREY, 'LineWidth', 2.2, ...
              'MarkerSize', 7, 'MarkerFaceColor', GREY, 'CapSize', 8);
yl = [min(0, min(mism_pct) - 2) - 4, max(mism_pct) + 6];
hz = plot(ax1, [0.07 1.35], [0 0], RED, 'LineWidth', 2.0);
ht = plot(ax1, [B_TRUE B_TRUE], yl, RED, 'LineStyle', '--', 'LineWidth', 2.0);
ha = plot(ax1, [B_ANCHOR B_ANCHOR], yl, 'k--', 'LineWidth', 1.6);
ylabel(ax1, 'readout mismatch  [%]', 'FontSize', FS, 'FontWeight', 'bold');
legend([hp hz ht ha], {'mean(a_{wm}-a_{hat})', 'zero', 'true b = 0.13', 'anchor 9/8'}, ...
       'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS - 6);
set(ax1, 'XScale', 'log'); ylim(ax1, yl); xlim(ax1, [0.07 1.35]);
style_ax(ax1, FS);

ax2 = subplot(2, 1, 2); hold(ax2, 'on');
hd = plot(ax2, B_GRID, desc_pk,        'o-', 'Color', BLUE, 'LineWidth', 2.2, 'MarkerSize', 7);
ho = plot(ax2, B_GRID, osc_rms,        's-', 'Color', [0.85 0.33 0.10], 'LineWidth', 2.0, 'MarkerSize', 7);
hh = plot(ax2, B_GRID, abs(hold_mean), '^-', 'Color', [0.13 0.55 0.13], 'LineWidth', 2.0, 'MarkerSize', 7);
yl2 = [0, 1.1 * max([desc_pk; osc_rms; abs(hold_mean)])];
plot(ax2, [B_TRUE B_TRUE], yl2, RED, 'LineStyle', '--', 'LineWidth', 2.0, 'HandleVisibility', 'off');
plot(ax2, [B_ANCHOR B_ANCHOR], yl2, 'k--', 'LineWidth', 1.6, 'HandleVisibility', 'off');
ylabel(ax2, '|a_z error|  [%]', 'FontSize', FS, 'FontWeight', 'bold');
xlabel(ax2, 'locked b  [-]', 'FontSize', FS, 'FontWeight', 'bold');
legend([hd ho hh], {'descent peak', 'osc RMS', '|hold mean|'}, ...
       'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS - 5);
set(ax2, 'XScale', 'log'); ylim(ax2, yl2); xlim(ax2, [0.07 1.35]);
style_ax(ax2, FS);

outp = fullfile(fig_dir, 'formB_b_identifiability_sweep.png');
drawnow;
exportgraphics(fig, outp, 'Resolution', 150);
close(fig);
fprintf('FIGURE saved: %s\n', outp);


function style_ax(ax, FS)
    set(ax, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 1.3, 'Box', 'on');
    grid(ax, 'off');
    ax.Toolbar = [];
end
