% STATUS: ACTIVE | figure for B8 follow-ups 1 & 2 -- reads the .mat written by
%   check_formB_b8_followups.m (and the frozen parent .mat for the ON arm).
%   EXPIRES: with those scripts.
%
% Panel 1 is the whole task-2 adjudication in one picture: the response deficit statistic
% beta(t) for the echo correction ON and OFF, against what a uniform H2 row mis-scale
% predicts (a CONSTANT beta, and beta_off inside the band the independent S measurement
% allows).  Panel 2 shows where the deficit is actually accrued -- per unit of claimed
% learning, not per unit of time -- which is what selects the surviving candidate.
%
% Lab figure convention (.claude/rules/figure-style.md): no grid, no title, box on,
% legend northoutside horizontal, prediction/reference = red / measured = blue,
% FontSize 18 bold, exportgraphics Resolution 150, statistics to the console only.

clear cd    % MATLAB cd-shadowing hang, recorded in memory

here = fileparts(mfilename('fullpath'));
proj = fullfile(here, '..', '..');
F = load(fullfile(proj, 'test_results', 'temp_formB_b8_followups.mat'), 'res');
P = load(fullfile(proj, 'test_results', 'temp_formB_b8_selfconfirm.mat'), 'res');
assert(isfield(F.res, 'verdict2'), 'plot_b8fu:incomplete', 'follow-up run was partial.');

FS = 18; RED = 'r'; BLUE = 'b'; GREY = [0.35 0.35 0.35];
BAND_HUE = [0.78 0.85 0.95]; SBAND_HUE = [0.98 0.82 0.82];

t    = P.res.t;
b_on = P.res.main.G_meas ./ P.res.main.G_claim;
G_on = P.res.main.G_claim;
te   = F.res.echo.t;
b_off = F.res.echo.beta;
band = F.res.band;
tb   = [1.5 4.5];                     % descent end, oscillation end

fig = figure('Color', 'w', 'Position', [60 40 980 900]);

% --- 1. beta(t) on / off vs the H-row-mis-scale prediction ----------------------------
ax1 = subplot(2, 1, 1); hold(ax1, 'on');
hsb = fill(ax1, [te; flipud(te)], [band(1)*ones(size(te)); band(2)*ones(size(te))], ...
           SBAND_HUE, 'EdgeColor', 'none', 'FaceAlpha', 0.9);
hbd = fill(ax1, [te; flipud(te)], [F.res.echo.beta_hi; flipud(F.res.echo.beta_lo)], ...
           BAND_HUE, 'EdgeColor', 'none', 'FaceAlpha', 0.85);
hnl = plot(ax1, t, ones(size(t)), RED, 'LineWidth', 2.0);
hon = plot(ax1, t, b_on, BLUE, 'LineWidth', 2.2);
hof = plot(ax1, te, b_off, 'Color', GREY, 'LineWidth', 2.2);
ylabel(ax1, '\beta = G_{meas}/G_{claim}', 'FontSize', FS, 'FontWeight', 'bold');
legend([hnl hon hof hbd hsb], {'no deficit', 'echo ON', 'echo OFF', '3\sigma boot (OFF)', ...
       'where echo OFF must land if S explains it'}, 'Location', 'northoutside', ...
       'Orientation', 'horizontal', 'FontSize', FS - 9);
ylim(ax1, [0.5 1.05]); local_style(ax1, FS, t, tb);

% --- 2. cumulative deficit, both arms (derivative-free; phase boundaries marked) -----
ax2 = subplot(2, 1, 2); hold(ax2, 'on');
hz  = plot(ax2, t, zeros(size(t)), RED, 'LineWidth', 2.0);
h1  = plot(ax2, t, 100 * (1 - b_on), BLUE, 'LineWidth', 2.2);
h2  = plot(ax2, te, 100 * (1 - b_off), 'Color', GREY, 'LineWidth', 2.2);
ylabel(ax2, 'deficit 1-\beta  [%]', 'FontSize', FS, 'FontWeight', 'bold');
xlabel(ax2, 'time  [s]', 'FontSize', FS, 'FontWeight', 'bold');
legend([hz h1 h2], {'no deficit', 'echo ON', 'echo OFF'}, 'Location', 'northoutside', ...
       'Orientation', 'horizontal', 'FontSize', FS - 6);
ylim(ax2, [-1 8]); local_style(ax2, FS, t, tb);

outp = fullfile(proj, 'test_results', 'temp_formB_b8_echo.png');
drawnow; exportgraphics(fig, outp, 'Resolution', 150); close(fig);
fprintf('FIGURE saved: %s\n', outp);
fprintf('task1: %s\ntask2: %s\n', F.res.verdict1, F.res.verdict2);


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
