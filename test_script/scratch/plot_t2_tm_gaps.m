% STATUS: SCRATCH -- 2026-10-22 (user: convergence comparison drawn like the A / B angle-error panels). One arm of the fixed-point runs
%   tm<gap>_<arm>_s<seed> (default 'A' = wait only), columns = gaps, rows = theta-hat_y - theta_y | theta-hat_x - theta_x, every seed
%   (seed 1 bold), zero line red, shared axes. -> test_results/temp_3d_wsonly/fig_t2_tmgaps_<arm>.png | run: plot_t2_tm_gaps([50 15 10 5], 'A')
function plot_t2_tm_gaps(GAP, arm, SD)
    if nargin < 2; arm = 'A'; end
    if nargin < 3; SD = 1:3; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_3d_wsonly');
    cT = [0.85 0 0];  cB = {[0 0.25 0.85], [0 0.55 0.2]};  cL = {[0.6 0.7 0.95], [0.6 0.85 0.65]};  FS = 18;  LFS = 14;
    TH = char(952);  HT = char(770);  DG = char(176);  MU = char(181);  AB = char(257);  NM = {'y', 'x'};
    nc = numel(GAP);
    f = figure('Position', [40 40 620 * nc 1000], 'Color', 'w', 'Visible', 'off');
    tl = tiledlayout(2, nc, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ic = 1:nc
        for q = 1:2
            ax = nexttile(tl, (q - 1) * nc + ic);  hold on;  box on;
            for s = SD
                S = load(fullfile(od, sprintf('temp_t2_tm%d_%s_s%d.mat', GAP(ic), arm, s)));  r = S.R.runs{1};  k = 1:r.kend;  t = r.t(k);
                if q == 1; e = r.thy(k) - S.R.opts.theta_y; else; e = r.thx(k); end
                if s == SD(1)
                    hb = plot(t, e, '-', 'Color', cB{q}, 'LineWidth', 2, 'DisplayName', [TH HT '_' NM{q} ' - ' TH '_' NM{q} ',  seed 1']);
                    rat = mean(r.aN_true(k)) / mean(r.aT_true(k));
                else
                    plot(t, e, '-', 'Color', cL{q}, 'LineWidth', 1, 'HandleVisibility', 'off');
                end
            end
            plot([0 t(end)], [0 0], '-', 'Color', cT, 'LineWidth', 2.5, 'HandleVisibility', 'off');
            ylim([-30 15]);  xlim([0 t(end)]);
            if ic == 1; ylabel([TH '_' NM{q} ' error  [' DG ']'], 'FontSize', FS, 'FontWeight', 'bold'); end
            if q == 1; text(0.97, 0.92, sprintf('%d %sm  (%s_N/%s_T = %.2f)', GAP(ic), MU, AB, AB, rat), 'Units', 'normalized', 'HorizontalAlignment', 'right', 'FontSize', FS - 2, 'FontWeight', 'bold'); end
            if q == 2; xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold'); end
            legend(hb, 'Location', 'northoutside', 'FontSize', LFS, 'FontWeight', 'bold');
            set(ax, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 2);
        end
    end
    out = fullfile(od, sprintf('fig_t2_tmgaps_%s.png', arm));
    exportgraphics(f, out, 'Resolution', 110);  close(f);  fprintf('figure: %s\n', out);
end
