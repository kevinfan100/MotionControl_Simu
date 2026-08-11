% STATUS: ACTIVE | figures for the B8 adjudication -- reads the .mat written by
%   check_formB_b8_selfconfirm.m (run that first; it holds the pre-registration, the
%   power calculation and the verdict rules).  EXPIRES: with that script.
%
% Panel logic: rho(t) is a RATIO, so it is never shown without its two factors.  The
% left column is the honesty ratio and its numerator/denominator; the right column is the
% response test (what the filter claims to have learned vs what it demonstrably learned)
% and the common-mode drift (the truth-independent part of b_hat's motion).
%
% Lab figure convention (.claude/rules/figure-style.md): no grid, no title, box on,
% legend northoutside horizontal, reference/claim = red / measured = blue,
% FontSize 18 bold, exportgraphics Resolution 150, statistics to the console only.

clear cd    % MATLAB cd-shadowing hang, recorded in memory

here = fileparts(mfilename('fullpath'));
proj = fullfile(here, '..', '..');
db_file = fullfile(proj, 'test_results', 'temp_formB_b8_selfconfirm.mat');
assert(exist(db_file, 'file') == 2, 'plot_formB_b8:noData', ...
       'run check_formB_b8_selfconfirm.m first (missing %s).', db_file);
S = load(db_file, 'res');
res = S.res;
assert(isfield(res, 'verdict'), 'plot_formB_b8:incomplete', ...
       'the .mat has no verdict: the run was still partial.');

FS         = 18;
RED        = 'r';
BLUE       = 'b';
LIGHT_BLUE = [0.45 0.72 0.95];
BAND_HUE   = [0.78 0.85 0.95];
GREY       = [0.35 0.35 0.35];

t  = res.t;
tb = [1.5 4.8];                       % end of descent, canonical end (phase markers)
A  = res.main;

% =====================================================================================
% Figure 1: the main arm, four panels
% =====================================================================================
fig = figure('Color', 'w', 'Position', [60 30 980 1300]);

% --- 1. honesty ratio -----------------------------------------------------------------
ax1 = subplot(4, 1, 1); hold(ax1, 'on');
hbd = fill(ax1, [t; flipud(t)], [A.rho_hi; flipud(A.rho_lo)], BAND_HUE, ...
           'EdgeColor', 'none', 'FaceAlpha', 0.85);
hfz = plot(ax1, t, A.rho_frozen, 'Color', GREY, 'LineStyle', '--', 'LineWidth', 2.0);
hnl = plot(ax1, t, A.rho_hon, RED, 'LineWidth', 2.4);
hro = plot(ax1, t, A.rho, BLUE, 'LineWidth', 2.2);
ylabel(ax1, '\rho  [-]', 'FontSize', FS, 'FontWeight', 'bold');
legend([hnl hro hbd hfz], {'honest null', 'measured \rho', '3\sigma bootstrap', ...
       'hypothesis (b): MSE frozen'}, 'Location', 'northoutside', ...
       'Orientation', 'horizontal', 'FontSize', FS - 7);
ylim(ax1, [0 2.2]); local_style(ax1, FS, t, tb);

% --- 2. the two factors separately ----------------------------------------------------
ax2 = subplot(4, 1, 2); hold(ax2, 'on');
hsp = plot(ax2, t, A.sP,  RED,  'LineWidth', 2.4);
hrm = plot(ax2, t, A.rms, BLUE, 'LineWidth', 2.2);
ylabel(ax2, 'b error scale  [-]', 'FontSize', FS, 'FontWeight', 'bold');
legend([hsp hrm], {'declared sqrt(P_{bb})', 'realised RMS(b_{hat} - b_{true})'}, ...
       'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS - 6);
ylim(ax2, [0 0.11]); local_style(ax2, FS, t, tb);

% --- 3. response test -----------------------------------------------------------------
ax3 = subplot(4, 1, 3); hold(ax3, 'on');
hgb = fill(ax3, [t; flipud(t)], [A.G_hi; flipud(A.G_lo)], BAND_HUE, ...
           'EdgeColor', 'none', 'FaceAlpha', 0.85);
hgc = plot(ax3, t, A.G_claim, RED,  'LineWidth', 2.4);
hgm = plot(ax3, t, A.G_meas,  BLUE, 'LineWidth', 2.2);
ylabel(ax3, 'G  [-]', 'FontSize', FS, 'FontWeight', 'bold');
legend([hgc hgm hgb], {'claimed 1 - P/P_0', 'measured dE[b_{hat}]/db_{true}', ...
       '3\sigma bootstrap'}, 'Location', 'northoutside', ...
       'Orientation', 'horizontal', 'FontSize', FS - 7);
ylim(ax3, [0 1]); local_style(ax3, FS, t, tb);

% --- 4. common-mode drift -------------------------------------------------------------
ax4 = subplot(4, 1, 4); hold(ax4, 'on');
hcb = fill(ax4, [t; flipud(t)], [A.cm_hi; flipud(A.cm_lo)], BAND_HUE, ...
           'EdgeColor', 'none', 'FaceAlpha', 0.85);
hz  = plot(ax4, t, zeros(size(t)), RED, 'LineWidth', 2.4);
hcm = plot(ax4, t, A.cm, BLUE, 'LineWidth', 2.2);
ylabel(ax4, 'common mode  [-]', 'FontSize', FS, 'FontWeight', 'bold');
xlabel(ax4, 'time  [s]', 'FontSize', FS, 'FontWeight', 'bold');
legend([hz hcm hcb], {'no truth-independent drift', ...
       'mean(b_{hat}) - seed, truth-averaged', '3\sigma bootstrap'}, ...
       'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS - 7);
ylim(ax4, 0.08 * [-1 1]); local_style(ax4, FS, t, tb);

out1 = fullfile(proj, 'test_results', 'temp_formB_b8_main.png');
drawnow; exportgraphics(fig, out1, 'Resolution', 150); close(fig);
fprintf('FIGURE saved: %s\n', out1);

% =====================================================================================
% Figure 2: arm comparison -- honesty ratio against each arm's own null, and the
%           response deficit that separates a structural over-claim from curvature
% =====================================================================================
fig = figure('Color', 'w', 'Position', [60 60 980 760]);

ax1 = subplot(2, 1, 1); hold(ax1, 'on');
h = gobjects(0); lbl = {};
cols = {BLUE, GREY, LIGHT_BLUE};
arms = {'main', 'dfar', 'plane'};
for i = 1:numel(arms)
    Ai = res.(arms{i});
    plot(ax1, t, Ai.rho_hon, RED, 'LineWidth', 1.6, 'LineStyle', ':', 'HandleVisibility', 'off');
    h(end+1) = plot(ax1, t, Ai.rho, 'Color', cols{i}, 'LineWidth', 2.2); %#ok<SAGROW>
    lbl{end+1} = sprintf('%s (r = %.2f)', arms{i}, Ai.r); %#ok<SAGROW>
end
ylabel(ax1, '\rho  [-]', 'FontSize', FS, 'FontWeight', 'bold');
legend(h, lbl, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS - 6);
ylim(ax1, [0 2.4]); local_style(ax1, FS, t, tb);

ax2 = subplot(2, 1, 2); hold(ax2, 'on');
hz  = plot(ax2, t, zeros(size(t)), RED, 'LineWidth', 2.4);
h1  = plot(ax2, t, res.main.G_meas - res.main.G_claim, BLUE, 'LineWidth', 2.2);
h2  = plot(ax2, t, res.dfar.G_meas - res.dfar.G_claim, 'Color', GREY, 'LineWidth', 2.2);
h3  = plot(ax2, t, 6.25 * (res.main.G_meas - res.main.G_claim), 'Color', GREY, ...
           'LineStyle', '--', 'LineWidth', 1.8);
ylabel(ax2, 'G_{meas} - G_{claim}', 'FontSize', FS, 'FontWeight', 'bold');
xlabel(ax2, 'time  [s]', 'FontSize', FS, 'FontWeight', 'bold');
legend([hz h1 h2 h3], {'no deficit', 'd = 0.10', 'd = 0.25 (measured)', ...
       'd = 0.25 if the deficit were curvature (x6.25)'}, ...
       'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS - 8);
ylim(ax2, [-0.08 0.02]); local_style(ax2, FS, t, tb);

out2 = fullfile(proj, 'test_results', 'temp_formB_b8_arms.png');
drawnow; exportgraphics(fig, out2, 'Resolution', 150); close(fig);
fprintf('FIGURE saved: %s\n', out2);
fprintf('verdict on record: %s | deficit source: %s\n', res.verdict, res.deficit_source);


function local_style(ax, FS, t, tb)
    set(ax, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 1.3, 'Box', 'on');
    grid(ax, 'off');
    ax.Toolbar = [];
    xlim(ax, [t(1) t(end)]);
    yl = ylim(ax);
    for x = tb
        plot(ax, [x x], yl, '--', 'Color', [0.55 0.55 0.55], 'LineWidth', 1.0, ...
             'HandleVisibility', 'off');
    end
    ylim(ax, yl);
end
