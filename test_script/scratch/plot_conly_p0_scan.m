% PURPOSE (2026-10-04, user: "P_0 not important for our trajectory because we always come from far? run it"): c-only f1p2 on
%   ramp_tri (1 s constant-velocity descent 22.2 -> 1.10 R, triangle 1.10 <-> 2.00 R 1 Hz for 2 s), plane init, Q55 = P_fix^2 I
%   (q55_adapt 'ratio', sqrt(P_fix) = 0.2), start value sqrt(P_0) = sqrt(P^f_55[0]) in {0.1, 0.2, 0.6}; 10 seeds.
%   Columns = plane | cell | sphere.  Rows: a_true (red) and a_hat (one colour per P_0) | e_a = a_true - a_hat | sqrt(P^f_55).
%   Reads temp_plaw_walls/f1p2_<wall>_PI_sig<010|020|060>_rho20_rtri_f1.mat (run_plaw_walls, see the tmp runner of this date).
%   Notation: 1004_q55_ratio_from_kf.tex (P_0 = start value, P_fix = equilibrium).   STATUS: SCRATCH
function plot_conly_p0_scan()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    id = fullfile(root, 'test_results', 'temp_plaw_walls');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    WALLS = {'plane', 'cell', 'sphere'};
    ARMS = {'010', '\surd P_0 = 0.1', [0 0.2 0.9]; '020', '\surd P_0 = 0.2', [0 0.6 0.3]; '060', '\surd P_0 = 0.6', [0.9 0.5 0]};
    FS = 16;  LFS = 11;  AXLW = 1.8;
    f = figure('Position', [40 40 1800 1100], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(3, numel(WALLS), 'TileSpacing', 'compact', 'Padding', 'compact');
    ax = gobjects(3, numel(WALLS));
    for iw = 1:numel(WALLS)
        for r = 1:3; ax(r, iw) = nexttile(tl, (r - 1) * numel(WALLS) + iw); hold(ax(r, iw), 'on'); box(ax(r, iw), 'on'); end
        fprintf('\n%s\n', WALLS{iw});
        fprintf('  %-16s | e_a at troughs 1 / 2 / 3 s            | tops 1.5 / 2.5 s       | mean|e_a| tri | worst |e_a| last 0.1 s of descent | sqrtP55 at 0.9 / 1.0 s\n', 'arm');
        hl = gobjects(0);
        for m = 1:size(ARMS, 1)
            fn = fullfile(id, sprintf('f1p2_%s_PI_sig%s_rho20_rtri_f1.mat', WALLS{iw}, ARMS{m, 1}));
            if ~exist(fn, 'file'); fprintf('  missing %s\n', fn); continue; end
            S = load(fn);  t = S.t;  ea = S.a_true - S.a_hat;
            if m == 1; ht = plot(ax(1, iw), t, mean(S.a_true, 2), '-', 'Color', [0.8 0 0], 'LineWidth', 2.4, 'DisplayName', 'a_{true}'); hl(end + 1) = ht; end %#ok<AGROW>
            hh = plot(ax(1, iw), t, mean(S.a_hat, 2), '-', 'Color', ARMS{m, 3}, 'LineWidth', 1.6, 'DisplayName', ['a_{hat}, ' ARMS{m, 2}]);  hl(end + 1) = hh; %#ok<AGROW>
            plot(ax(2, iw), t, mean(ea, 2), '-', 'Color', ARMS{m, 3}, 'LineWidth', 1.6);
            plot(ax(3, iw), t, mean(S.sP_c, 2), '-', 'Color', ARMS{m, 3}, 'LineWidth', 1.6);
            at = @(tt) ea(find(t >= tt - 1e-9, 1), :);
            tr = [at(1); at(2); at(3)];  tp = [at(1.5); at(2.5)];
            ktri = t >= 1;  kd = t >= 0.9 & t < 1;
            w = max(abs(ea(kd, :)), [], 1);
            sp = @(tt) mean(S.sP_c(find(t >= tt - 1e-9, 1), :));
            fprintf('  %-16s | %+.3f+-%.3f %+.3f+-%.3f %+.3f+-%.3f | %+.3f+-%.3f %+.3f+-%.3f | %.4f        | %.3f+-%.3f                           | %.3f / %.3f\n', ...
                    strrep(ARMS{m, 2}, '\surd ', 'sqrt'), mean(tr(1, :)), std(tr(1, :)), mean(tr(2, :)), std(tr(2, :)), mean(tr(3, :)), std(tr(3, :)), ...
                    mean(tp(1, :)), std(tp(1, :)), mean(tp(2, :)), std(tp(2, :)), mean(mean(abs(ea(ktri, :)))), mean(w), std(w), sp(0.9), sp(1.0));
        end
        plot(ax(2, iw), [0 3], [0 0], '-', 'Color', [0.6 0.6 0.6], 'LineWidth', 0.8);
        plot(ax(3, iw), [0 3], [0.2 0.2], ':', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.2);
        for r = 1:3
            yl = ylim(ax(r, iw));  plot(ax(r, iw), [1 1], yl, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0);  ylim(ax(r, iw), yl);
            xlim(ax(r, iw), [0 3]);  set(ax(r, iw), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(ax(r, iw), 'off');
        end
        xlabel(ax(3, iw), 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        legend(ax(1, iw), hl, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'NumColumns', 2);
        title(ax(1, iw), '');  text(ax(1, iw), 0.03, 0.08, [WALLS{iw} ' wall, plane init'], 'Units', 'normalized', 'FontSize', FS - 2, 'FontWeight', 'bold');
    end
    ylabel(ax(1, 1), 'a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold');
    ylabel(ax(2, 1), 'e_a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold');
    ylabel(ax(3, 1), '\surd P^f_{55}', 'FontSize', FS, 'FontWeight', 'bold');
    linkaxes(ax(2, :), 'y');  linkaxes(ax(3, :), 'y');
    out = fullfile(fd, 'fig_conly_p0_scan_rtri.png');  exportgraphics(f, out, 'Resolution', 150);  fprintf('\nfigure: %s\n', out);
end
