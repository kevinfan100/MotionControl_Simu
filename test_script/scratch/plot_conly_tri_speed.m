% PURPOSE (2026-10-05, user: "how to improve the estimate in the oscillation"): ramp_tri, Td 1 s, triangle 1.10-2.00 R at 1 Hz vs 5 Hz,
%   10 seeds, plane init, columns plane | cell.  Rows: (1) e_abar seed mean (line) +- seed sd (band)  (2) c_hat seed mean with c_eff(h_d).
%   Arms: c known (a'_true at the estimator height; non-c floor), q55 = P_fix^2 I with sqrt(P_fix) 0.15 on new heights / 0.05 on revisits.
%   Reads aptrue_<wall>_rtri_f<f>.mat, f1p2_<wall>_PI_sig010_rn15_05_rtri_f<f>.mat.   STATUS: SCRATCH
function plot_conly_tri_speed()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    id = fullfile(root, 'test_results', 'temp_plaw_walls');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    WALLS = {'plane', 'cell'};  FS = 17;  LFS = 12;  AXLW = 2.0;
    cF = {[0.55 0.75 1.0], [0 0.2 0.9]};  cK = [0.4 0.4 0.4];  cT = [0.8 0 0];
    f = figure('Position', [40 40 1600 900], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for iw = 1:2
        a1 = nexttile(tl, iw);  hold on; box on;  a2 = nexttile(tl, 2 + iw);  hold on; box on;  h = gobjects(0);
        for ff = 1:2
            fq = [1 5];  fq = fq(ff);
            S = load(fullfile(id, sprintf('f1p2_%s_PI_sig010_rn15_05_rtri_f%d.mat', WALLS{iw}, fq)));  t = S.t(:);
            e = S.a_true - S.a_hat;  m = mean(e, 2);  s = std(e, 0, 2);
            fill(a1, [t; flipud(t)], [m - s; flipud(m + s)], cF{ff}, 'FaceAlpha', 0.25, 'EdgeColor', 'none', 'HandleVisibility', 'off');
            h(end + 1) = plot(a1, t, m, '-', 'Color', cF{ff}, 'LineWidth', 1.6, 'DisplayName', sprintf('P_{fix}, %d Hz', fq)); %#ok<AGROW>
            ce = S.c_eff_d(:);  ce(S.wd(:) < 1) = NaN;
            if ff == 2; plot(a2, t, ce, '-', 'Color', cT, 'LineWidth', 1.2, 'DisplayName', 'c_{eff}(h_d), 5 Hz'); end
            plot(a2, t, mean(S.c_hat, 2), '-', 'Color', cF{ff}, 'LineWidth', 2.0, 'DisplayName', sprintf('c_{hat}, %d Hz', fq));
            K = load(fullfile(id, sprintf('aptrue_%s_rtri_f%d.mat', WALLS{iw}, fq)));
            if ff == 2; h(end + 1) = plot(a1, K.t, mean(K.a_true - K.a_hat, 2), '--', 'Color', cK, 'LineWidth', 1.4, 'DisplayName', 'c known, 5 Hz'); end %#ok<AGROW>
        end
        plot(a1, [0.8 3], [0 0], ':', 'Color', [0.3 0.3 0.3], 'HandleVisibility', 'off');
        xlim(a1, [0.8 3]);  ylim(a1, [-0.08 0.08]);  xlim(a2, [0.8 3]);  ylim(a2, [-0.3 0.9]);
        legend(a1, h, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'AutoUpdate', 'off');
        lg = legend(a2, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'AutoUpdate', 'off');
        lg.Title.String = sprintf('%s wall, plane init', WALLS{iw});
        for ax = [a1 a2]; set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW); grid(ax, 'off'); end
        xlabel(a2, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    end
    ylabel(nexttile(tl, 1), 'e_{a}  (a_z / a_{nom}: true - est)', 'FontSize', FS, 'FontWeight', 'bold');
    ylabel(nexttile(tl, 3), 'c', 'FontSize', FS, 'FontWeight', 'bold');
    out = fullfile(fd, 'fig_conly_tri_speed.png');  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end
