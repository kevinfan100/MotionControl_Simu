% STATUS: SCRATCH -- 2026-10-25 (user: arm B (Meng structure, our controller) flat-wall check, then vs arm A). Meng Sec. 5.5 scenario, seed 2,
%   sqrt P_theta[0] = 20 deg. Columns = runs (TAG, LBL), rows = theta-hat_y, theta-hat_x with the truth (red).
%   -> test_results/temp_3d_wsonly/fig_t2_meng55_cmp.png | run: plot_t2_meng55_cmp({'meng55B_flat_s2', ...}, {'B flat', ...})
function plot_t2_meng55_cmp(TAG, LBL)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_3d_wsonly');
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  cG = [0 0.55 0.2];  FS = 16;  LFS = 12;  TH = char(952);  HT = char(770);  DG = char(176);
    nc = numel(TAG);
    f = figure('Position', [40 40 560 * nc 760], 'Color', 'w', 'Visible', 'off');
    tl = tiledlayout(2, nc, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ic = 1:nc
        S = load(fullfile(od, ['temp_t2_' TAG{ic} '.mat']));  r = S.R.runs{1};  k = 1:r.kend;  t = r.t(k);
        V = {r.thy(k), r.thx(k)};  Tr = {r.thy_true(k), r.thx_true(k)};  nm = {'y', 'x'};  co = {cE, cG};
        for q = 1:2
            ax = nexttile(tl, (q - 1) * nc + ic);  hold on;  box on;
            h1 = plot(t, Tr{q}, '-', 'Color', cT, 'LineWidth', 2.5, 'DisplayName', [TH '_' nm{q}]);
            h2 = plot(t, V{q}, '-', 'Color', co{q}, 'LineWidth', 1.5, 'DisplayName', [TH HT '_' nm{q}]);
            ylim([-30 60]);  if q == 2; ylim([-45 45]); end;  xlim([0 t(end)]);
            if q == 1; text(0.03, 0.92, LBL{ic}, 'Units', 'normalized', 'FontSize', FS - 2, 'FontWeight', 'bold'); end
            if ic == 1; ylabel([TH '_' nm{q} '  [' DG ']'], 'FontSize', FS, 'FontWeight', 'bold'); end
            if q == 2; xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold'); end
            legend([h1 h2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
            set(ax, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 2);
        end
    end
    out = fullfile(od, 'fig_t2_meng55_cmp.png');  exportgraphics(f, out, 'Resolution', 150);  close(f);  fprintf('figure: %s\n', out);
end
