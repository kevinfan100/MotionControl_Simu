% STATUS: SCRATCH -- 2026-10-22 (user: single seed, every wall setting, with / without the angle measurement, any visible difference?).
%   Runs wm_<wall>_<W|N>_s2 (run_walls_meas.sh): W = with, N = without (theta_xm, theta_ym rows never used). Same flow on every wall:
%   start 15 um, hold 5 s + dither, regulation 5-12 s, scan / raster from 12 s.
%   Rows = walls; columns = theta-hat_y - theta_y | theta-hat_x - theta_x (truth = local wall normal under the probe) | h [um].
%   Blue = with, orange dashed = without; red = 0 (angles) / contact h = R (gap).
%   -> test_results/temp_3d_wsonly/fig_t2_walls_meas_s2.png | run: plot_t2_walls_meas({'plane','bend','bump','raster','bead'})
function plot_t2_walls_meas(WALL)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_3d_wsonly');  addpath(genpath(fullfile(root, 'model')));  pc = physical_constants();  Rum = pc.R;
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  cO = [0.9 0.5 0];  FS = 16;  LFS = 13;  TH = char(952);  HT = char(770);  DG = char(176);  MU = char(181);
    NM = struct('plane', 'plane 20\circ', 'bend', 'bend wall', 'bump', 'bump, one pass', 'raster', 'bump, raster', 'bead', 'glass bead');
    nr = numel(WALL);
    f = figure('Position', [40 40 1900 330 * nr], 'Color', 'w', 'Visible', 'off');
    tl = tiledlayout(nr, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    for iw = 1:nr
        R = cell(1, 2);  AR = {'W', 'N'};
        for ia = 1:2
            S = load(fullfile(od, sprintf('temp_t2_wm_%s_%s_s2.mat', WALL{iw}, AR{ia})));  R{ia} = S.R.runs{1};
        end
        tend = max(R{1}.t(R{1}.kend), R{2}.t(R{2}.kend));
        for q = 1:3
            ax = nexttile(tl, (iw - 1) * 3 + q);  hold on;  box on;  hh = gobjects(0);
            for ia = 1:2
                r = R{ia};  k = 1:r.kend;  t = r.t(k);
                switch q
                    case 1; v = r.thy(k) - r.thy_true(k);
                    case 2; v = r.thx(k) - r.thx_true(k);
                    case 3; v = r.h(k) * Rum;
                end
                if ia == 1; st = {'-', cE, 1.5}; nm = 'with angle measurement'; else; st = {'--', cO, 1.5}; nm = 'without angle measurement'; end
                hh(end + 1) = plot(t, v, st{1}, 'Color', st{2}, 'LineWidth', st{3}, 'DisplayName', nm);   %#ok<AGROW>
                if r.contact; plot(t(end), v(end), 'x', 'Color', st{2}, 'MarkerSize', 16, 'LineWidth', 3, 'HandleVisibility', 'off'); end
            end
            if q < 3; plot([0 tend], [0 0], '-', 'Color', cT, 'LineWidth', 2.5, 'HandleVisibility', 'off');  ylim([-20 20]);
            else; plot([0 tend], [Rum Rum], '-', 'Color', cT, 'LineWidth', 2.5, 'HandleVisibility', 'off');  ylim([0 16]); end
            xlim([0 tend]);
            yl = {[TH HT '_y - ' TH '_y  [' DG ']'], [TH HT '_x - ' TH '_x  [' DG ']'], ['h  [' MU 'm]']};
            ylabel(yl{q}, 'FontSize', FS - 2, 'FontWeight', 'bold');
            if q == 1; text(0.97, 0.88, NM.(WALL{iw}), 'Units', 'normalized', 'HorizontalAlignment', 'right', 'FontSize', FS, 'FontWeight', 'bold'); end
            if iw == nr; xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold'); end
            if iw == 1 && q == 2; legend(hh, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold'); end
            set(ax, 'FontSize', FS - 3, 'FontWeight', 'bold', 'LineWidth', 1.5);
        end
        for ia = 1:2
            r = R{ia};  k = 1:r.kend;  t = r.t(k);  m = t >= 12;  m5 = t >= 4 & t < 5;
            ey = r.thy(k) - r.thy_true(k);  ex = r.thx(k) - r.thx_true(k);
            fprintf('%-6s %s: contact %d (t %.2f) | end of hold (4-5 s) mean |e_y| %.1f |e_x| %.1f deg | scan max |e_y| %.1f |e_x| %.1f deg, min h %.3f um\n', ...
                    WALL{iw}, AR{ia}, r.contact, r.t_contact, mean(abs(ey(m5))), mean(abs(ex(m5))), max(abs(ey(m))), max(abs(ex(m))), min(r.h(k(m))) * Rum);
        end
    end
    out = fullfile(od, 'fig_t2_walls_meas_s2.png');  exportgraphics(f, out, 'Resolution', 150);  close(f);  fprintf('figure: %s\n', out);
end
