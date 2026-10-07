% STATUS: SCRATCH -- 2026-10-07, motion_ctrl Section 2 figure in the layout of the paper's Fig. 5 (tracking error only;
%   left: true motion gain in the control law, right: nominal motion gain). z axis only, so one row. Paper's Scenario B
%   z-motion (start near the wall, 3 Hz sine, amplitude 1 um, closest h/R = 1.111), 5 s, plane wall, lambda_c 0.4, seed 1.
%   Style (user 10-07): measured tracking error raw only, no legend, no mean; the caption in the document names the arms.
function plot_mc_s2_fig5()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    id = fullfile(root, 'test_results', 'temp_plaw_walls');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    TAG = {'_s1_ctrltrue_mengB5', '_s1_ctrlnom_mengB5'};
    cE = [0 0.25 0.85];  FS = 22;  AXLW = 2.2;
    f = figure('Position', [40 40 1600 480], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ic = 1:2
        S = load(fullfile(id, sprintf('f1p2_plane_PI%s_rtri_f1_lc040.mat', TAG{ic})));  t = S.t(:);  x = S.dxm(:, 1);
        ax = nexttile(tl, ic);  hold on; box on;
        plot(ax, t, x, '-', 'Color', cE, 'LineWidth', 1.0);
        xlim(ax, [0 t(end)]);  ylim(ax, [-0.25 0.25]);
        set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(ax, 'off');
        xlabel(ax, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        if ic == 1; ylabel(ax, [char(948) 'z_m  [' char(181) 'm]'], 'FontSize', FS, 'FontWeight', 'bold'); end
        fprintf('%-22s | dz_m min %+.3f max %+.3f um, rms %.4f um\n', TAG{ic}, min(x), max(x), sqrt(mean(x.^2)));
    end
    out = fullfile(fd, 'fig_mc_s2_fig5.png');  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end
