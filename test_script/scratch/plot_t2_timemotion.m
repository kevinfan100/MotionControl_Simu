% STATUS: SCRATCH -- 2026-10-22 (user: separate what time and what motion each give the angle). Fixed point at two gaps (rows), arms A
%   time only | B time + motion (dither) | C motion only (angle readout rows off) | D neither (negative control) as columns; runs
%   tm<gap>_<arm>_s<seed>. Each panel: angle errors theta-hat_y - theta_y (blue) and theta-hat_x - theta_x (green) of every seed (left axis)
%   and sqrt P_theta_y of every seed (black, right axis), shared limits. Console: per gap / arm / seed, sqrt P_theta_y and |errors| at 5, 10,
%   20 s, and the first time both errors stay within 5 and 2 deg.
%   -> test_results/temp_3d_wsonly/fig_t2_timemotion.png | run: plot_t2_timemotion([50 15], 1:3)
function plot_t2_timemotion(GAP, SD)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_3d_wsonly');
    ARM = {'A', 'B', 'C', 'D'};  LB = {'A: time only', 'B: time + motion', 'C: motion only', 'D: neither'};
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  cGr = [0 0.55 0.2];  FS = 16;  LFS = 12;  TH = char(952);  HT = char(770);  DG = char(176);  MU = char(181);
    f = figure('Position', [40 40 2400 1100], 'Color', 'w', 'Visible', 'off');
    tl = tiledlayout(numel(GAP), numel(ARM), 'TileSpacing', 'compact', 'Padding', 'compact');
    for ig = 1:numel(GAP)
        for ia = 1:numel(ARM)
            ax = nexttile(tl);  hold on;  box on;
            for s = SD
                S = load(fullfile(od, sprintf('temp_t2_tm%d_%s_s%d.mat', GAP(ig), ARM{ia}, s)));  r = S.R.runs{1};  ty = S.R.opts.theta_y;
                k = 1:r.kend;  t = r.t(k);  ey = r.thy(k) - ty;  ex = r.thx(k);
                yyaxis left
                h1 = plot(t, ey, '-', 'Color', cE, 'LineWidth', 1.1, 'DisplayName', [TH HT '_y - ' TH '_y']);
                h2 = plot(t, ex, '-', 'Color', cGr, 'LineWidth', 1.1, 'DisplayName', [TH HT '_x - ' TH '_x']);
                yyaxis right
                h3 = plot(t, r.sP(k, 6), '-', 'Color', [0 0 0], 'LineWidth', 1.6, 'DisplayName', ['\surdP_{' TH 'y}']);
                e = max(abs([ey ex]), [], 2);  tail = flipud(cummax(flipud(e)));
                g = @(tt) find(t >= tt, 1);
                fprintf('%2d um %s s%d: sqrtP 5/10/20 s %.2f %.2f %.2f | |e_thy| %.1f %.1f %.1f | |e_thx| %.1f %.1f %.1f | within 5 deg from %s s, 2 deg from %s s\n', ...
                        GAP(ig), ARM{ia}, s, r.sP(g(5), 6), r.sP(g(10), 6), r.sP(k(end), 6), abs(ey(g(5))), abs(ey(g(10))), abs(ey(end)), ...
                        abs(ex(g(5))), abs(ex(g(10))), abs(ex(end)), local_t(t, find(tail < 5, 1)), local_t(t, find(tail < 2, 1)));
            end
            yyaxis left;  plot([0 t(end)], [0 0], '-', 'Color', cT, 'LineWidth', 2, 'HandleVisibility', 'off');  ylim([-30 15]);
            set(ax.YAxis(1), 'Color', [0 0 0]);  if ia == 1; ylabel(sprintf('%d %sm:  angle error  [%s]', GAP(ig), MU, DG), 'FontSize', FS, 'FontWeight', 'bold'); end
            yyaxis right;  ylim([0 11]);  set(ax.YAxis(2), 'Color', [0 0 0]);  if ia == numel(ARM); ylabel(['\surdP  [' DG ']'], 'FontSize', FS, 'FontWeight', 'bold'); end
            xlim([0 t(end)]);  if ig == numel(GAP); xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold'); end
            if ig == 1
                title(LB{ia}, 'FontSize', FS, 'FontWeight', 'bold');          % column labels (one word line, no statistics)
                legend([h1 h2 h3], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
            end
            set(ax, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 2);
        end
    end
    out = fullfile(od, 'fig_t2_timemotion.png');
    exportgraphics(f, out, 'Resolution', 110);  close(f);  fprintf('figure: %s\n', out);
end

function s = local_t(t, i)
    if isempty(i); s = '-'; else; s = sprintf('%.1f', t(i)); end
end
