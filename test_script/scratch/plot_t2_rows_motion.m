% STATUS: SCRATCH -- 2026-10-22 (user: can the angle be estimated without the angle measurement?). Fixed points held 20 s on the
%   20 deg plane (run_timemotion.sh): A wait, angle rows on | B motion, angle rows on | C motion, angle rows off | D wait, angle rows off.
%   theta-hat_y - theta_y, every seed (seed 1 bold). Rows = with / without the angle measurement, columns = wait only / motion.
%   -> test_results/temp_3d_wsonly/fig_t2_rows_motion_<gap>.png | run: plot_t2_rows_motion(15)
function plot_t2_rows_motion(gap, SD)
    if nargin < 2; SD = 1:3; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_3d_wsonly');
    cT = [0.85 0 0];  cB = [0 0.25 0.85];  cL = [0.6 0.7 0.95];  FS = 18;  LFS = 14;
    TH = char(952);  HT = char(770);  DG = char(176);  MU = char(181);
    ARM = {'A', 'B'; 'D', 'C'};  CL = {'wait only', 'motion'};  RL = {'with angle measurement', 'without angle measurement'};
    f = figure('Position', [40 40 1240 1000], 'Color', 'w', 'Visible', 'off');
    tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ir = 1:2
        for ic = 1:2
            ax = nexttile(tl, (ir - 1) * 2 + ic);  hold on;  box on;
            for s = SD
                S = load(fullfile(od, sprintf('temp_t2_tm%d_%s_s%d.mat', gap, ARM{ir, ic}, s)));  r = S.R.runs{1};  k = 1:r.kend;  t = r.t(k);
                e = r.thy(k) - S.R.opts.theta_y;
                if s == SD(1)
                    hb = plot(t, e, '-', 'Color', cB, 'LineWidth', 2, 'DisplayName', [TH HT '_y - ' TH '_y,  seed ' num2str(s)]);
                else
                    plot(t, e, '-', 'Color', cL, 'LineWidth', 1, 'HandleVisibility', 'off');
                end
            end
            plot([0 t(end)], [0 0], '-', 'Color', cT, 'LineWidth', 2.5, 'HandleVisibility', 'off');
            ylim([-30 15]);  xlim([0 t(end)]);
            text(0.97, 0.08, sprintf('%d %sm,  %s,  %s', gap, MU, CL{ic}, RL{ir}), 'Units', 'normalized', 'HorizontalAlignment', 'right', ...
                 'FontSize', FS - 4, 'FontWeight', 'bold');
            if ic == 1; ylabel([TH '_y error  [' DG ']'], 'FontSize', FS, 'FontWeight', 'bold'); end
            if ir == 2; xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold'); end
            if ir == 1; legend(hb, 'Location', 'northoutside', 'FontSize', LFS, 'FontWeight', 'bold'); end
            set(ax, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 2);
        end
    end
    out = fullfile(od, sprintf('fig_t2_rows_motion_%d.png', gap));  exportgraphics(f, out, 'Resolution', 110);  close(f);  fprintf('figure: %s\n', out);
end
