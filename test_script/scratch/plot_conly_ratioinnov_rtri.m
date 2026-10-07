% PURPOSE (2026-10-04, user: "test your suggestion on the cell"): ramp_tri, c-only f1p2, plane init, sqrt(P_0) = sqrt(P^f_55[0]) = 0.1.
%   Arms: ratio sqrt(P_fix) = 0.15 (best ratio on this trajectory) | ratioinnov Q55 = P_low^2 I (mean nu1/sigma)^2, sqrt(P_low) = 0.10 / 0.15.
%   Columns plane | cell.  Rows: a_true (red) and a_hat means | e_a means | c_hat means with c_eff(h_d) (red) | sqrt(P^f_55) means.
%   set 'v1' (first test), 'v2' / 'v2lin' (10-04 methods 1 and 2, g = max(1,x) / x).
%   Reads temp_plaw_walls/f1p2_<wall>_PI_sig010_<arm>_rtri_f1.mat.   STATUS: SCRATCH
function plot_conly_ratioinnov_rtri(set_name)
    if nargin < 1; set_name = 'v1'; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    id = fullfile(root, 'test_results', 'temp_plaw_walls');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    WALLS = {'plane', 'cell'};
    switch set_name
        case 'v1'
            ARMS = {'rho15', 'ratio, \surdP_{fix} = 0.15', [0 0.2 0.9]; 'ri10', 'innov, \surdP_{low} = 0.10', [0.9 0.5 0]; 'ri15', 'innov, \surdP_{low} = 0.15', [0 0.6 0.3]};
        case 'v2'                                       % 10-04: method 1 (N = 25) and method 2 (I-weighted), g = max(1, x), sqrt(P_low) = 0.10
            ARMS = {'rho15', 'ratio, \surdP_{fix} = 0.15', [0 0.2 0.9]; 'rho10', 'ratio, \surdP_{fix} = 0.10', [0.5 0.5 0.5]; ...
                    'ri10n25max', 'innov N=25, max(1,x)', [0.9 0.5 0]; 'ri10wImax', 'innov I-weighted, max(1,x)', [0 0.6 0.3]};
        case 'v3'                                       % 10-04: ratiodrift Q55 = P_fix^2 I + dc^2 |M|/h^2, sqrt(P_fix) = 0.10
            ARMS = {'rho10', 'ratio, \surdP_{fix} = 0.10', [0.5 0.5 0.5]; 'rho15', 'ratio, \surdP_{fix} = 0.15', [0 0.2 0.9]; ...
                    'rd10dc02', 'drift \Deltac = 0.2', [0.9 0.5 0]; 'rd10dc05', 'drift \Deltac = 0.5', [0 0.6 0.3]; 'rd10dc08', 'drift \Deltac = 0.8', [0.6 0 0.6]};
        case 'v4'                                       % 10-04: covariance matching on c (observed vs model-predicted correction power)
            ARMS = {'rho10', 'ratio, \surdP_{fix} = 0.10', [0.5 0.5 0.5]; 'rho15', 'ratio, \surdP_{fix} = 0.15', [0 0.2 0.9]; ...
                    'cmratio', 'covmatch ratio, \surdP_{fix} = 0.10', [0.9 0.5 0]; 'cmpure', 'covmatch pure (no P_{fix})', [0 0.6 0.3]};
        case 'v5'                                       % 10-04: rationew, P_fix by new ground (P_new) / revisit (P_rev)
            ARMS = {'rho15', 'ratio, \surdP_{fix} = 0.15', [0 0.2 0.9]; 'rho20', 'ratio, \surdP_{fix} = 0.20', [0.5 0.5 0.5]; ...
                    'rn20_10', 'new 0.20 / revisit 0.10', [0.9 0.5 0]; 'rn20_05', 'new 0.20 / revisit 0.05', [0.6 0 0.6]; ...
                    'rn15_10', 'new 0.15 / revisit 0.10', [0 0.6 0.3]; 'rn15_05', 'new 0.15 / revisit 0.05', [0.4 0.8 0.9]};
        case 'v6'                                       % 10-04: start of the day vs the best rationew, alone
            ARMS = {'', 'start: Q_{55} = 0', [0.6 0.6 0.6]; 'rho20', 'start: ratio \surdP_{fix} = 0.20', [0 0.2 0.9]; ...
                    'rn15_05', 'now: new 0.15 / revisit 0.05', [0.9 0.5 0]};
        case 'v7'                                       % 10-05: best single P_fix vs best piecewise (new / revisit) P_fix
            ARMS = {'rho15', 'single: \surdP_{fix} = 0.15', [0 0.2 0.9]; 'rn15_05', 'piecewise: new 0.15 / revisit 0.05', [0.9 0.5 0]};
        case 'v8'                                       % 10-05: the evolution, layer 0 -> 3
            ARMS = {'', 'L0: Q_{55} = 0', [0.6 0.6 0.6]; 'r2r003', 'L1: Q_{55} = 0.03 R_2', [0.6 0 0.6]; ...
                    'rho15', 'L2: Q_{55} = P_{fix}^2 I, \surdP_{fix} = 0.15', [0 0.2 0.9]; 'rn15_05', 'L3: new 0.15 / revisit 0.05', [0.9 0.5 0]};
        case 'v8b'                                      % layer 1 at both rho
            ARMS = {'r2r003', 'Q_{55} = 0.03 R_2', [0.6 0 0.6]; 'r2r030', 'Q_{55} = 0.3 R_2', [0 0.6 0.3]; 'rn15_05', 'L3: new 0.15 / revisit 0.05', [0.9 0.5 0]};
        case 'v9'                                       % 10-05: Q55 = eps R2 scan vs P_fix^2 I
            ARMS = {'eps5e', '\epsilon = 10^{-5}', [0.6 0.6 0.6]; 'eps4e', '\epsilon = 10^{-4}', [0.4 0.8 0.9]; 'eps3e', '\epsilon = 10^{-3}', [0.6 0 0.6]; ...
                    'eps2e', '\epsilon = 10^{-2}', [0 0.6 0.3]; 'rho15', 'P_{fix}^2 I, \surdP_{fix} = 0.15', [0 0.2 0.9]; 'rn15_05', 'new 0.15 / revisit 0.05', [0.9 0.5 0]};
        case 'v10'                                      % 10-05: best eps per wall (plane 1e-3, cell 3e-3) vs the P_fix designs
            ARMS = {'eps3e', '\epsilon R_2, \epsilon = 10^{-3} (plane best)', [0.6 0 0.6]; 'epsv0p003', '\epsilon R_2, \epsilon = 3\times10^{-3} (cell best)', [0 0.6 0.3]; ...
                    'rho15', 'P_{fix}^2 I, \surdP_{fix} = 0.15', [0 0.2 0.9]; 'rn15_05', 'new 0.15 / revisit 0.05', [0.9 0.5 0]};
        case 'v2lin'                                    % same with g = x
            ARMS = {'rho15', 'ratio, \surdP_{fix} = 0.15', [0 0.2 0.9]; 'rho10', 'ratio, \surdP_{fix} = 0.10', [0.5 0.5 0.5]; ...
                    'ri10n25lin', 'innov N=25, x', [0.9 0.5 0]; 'ri10wIlin', 'innov I-weighted, x', [0 0.6 0.3]};
    end
    FS = 16;  LFS = 11;  AXLW = 1.8;  cT = [0.8 0 0];
    f = figure('Position', [40 40 1600 1400], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(4, numel(WALLS), 'TileSpacing', 'compact', 'Padding', 'compact');
    ax = gobjects(4, numel(WALLS));
    for iw = 1:numel(WALLS)
        for r = 1:4; ax(r, iw) = nexttile(tl, (r - 1) * numel(WALLS) + iw); hold(ax(r, iw), 'on'); box(ax(r, iw), 'on'); end
        fprintf('\n%s (plane init, sqrt P_0 = 0.1)\n', WALLS{iw});
        fprintf('  %-26s | troughs 1 / 2 / 3 s                          | tops 1.5 / 2.5 s            | mean|e_a| tri | sd_seed tri | worst 0.9-1 s | c_hat > 0.95: steps (seeds) | sqrtP55 tri mean / max\n', 'arm');
        hl = gobjects(0);
        for m = 1:size(ARMS, 1)
            if isempty(ARMS{m, 1}); fn = fullfile(id, sprintf('f1p2_%s_PI_sig010_rtri_f1.mat', WALLS{iw})); else; fn = fullfile(id, sprintf('f1p2_%s_PI_sig010_%s_rtri_f1.mat', WALLS{iw}, ARMS{m, 1})); end
            if ~exist(fn, 'file'); fprintf('  missing %s\n', fn); continue; end
            S = load(fn);  t = S.t;  ea = S.a_true - S.a_hat;
            if m == 1
                hl(end + 1) = plot(ax(1, iw), t, mean(S.a_true, 2), '-', 'Color', cT, 'LineWidth', 2.4, 'DisplayName', 'a_{true}'); %#ok<AGROW>
                ce = S.c_eff_d(:);  ce(S.wd(:) < 1) = NaN;  plot(ax(3, iw), t, ce, '-', 'Color', cT, 'LineWidth', 2.0);
            end
            hl(end + 1) = plot(ax(1, iw), t, mean(S.a_hat, 2), '-', 'Color', ARMS{m, 3}, 'LineWidth', 1.5, 'DisplayName', ARMS{m, 2}); %#ok<AGROW>
            plot(ax(2, iw), t, mean(ea, 2), '-', 'Color', ARMS{m, 3}, 'LineWidth', 1.5);
            plot(ax(3, iw), t, mean(S.c_hat, 2), '-', 'Color', ARMS{m, 3}, 'LineWidth', 1.5);
            plot(ax(4, iw), t, mean(S.sP_c, 2), '-', 'Color', ARMS{m, 3}, 'LineWidth', 1.5);
            at = @(tt) ea(find(t >= tt - 1e-9, 1), :);  tr = [at(1); at(2); at(3)];  tp = [at(1.5); at(2.5)];  kt = t >= 1;
            w = max(abs(ea(t >= 0.9 & t < 1, :)), [], 1);
            gh = S.c_hat > 0.95;
            fprintf('  %-26s | %+.3f+-%.3f %+.3f+-%.3f %+.3f+-%.3f | %+.3f+-%.3f %+.3f+-%.3f | %.4f        | %.4f      | %.3f+-%.3f  | %d (%d)                    | %.3f / %.3f\n', ...
                    strrep(strrep(ARMS{m, 2}, '\surd', 'sqrt'), '_{fix}', '_fix'), mean(tr(1, :)), std(tr(1, :)), mean(tr(2, :)), std(tr(2, :)), mean(tr(3, :)), std(tr(3, :)), ...
                    mean(tp(1, :)), std(tp(1, :)), mean(tp(2, :)), std(tp(2, :)), mean(mean(abs(ea(kt, :)))), mean(std(ea(kt, :), 0, 2)), mean(w), std(w), nnz(gh), nnz(any(gh, 1)), mean(mean(S.sP_c(kt, :))), max(max(S.sP_c(kt, :))));
        end
        plot(ax(2, iw), [0 3], [0 0], '-', 'Color', [0.6 0.6 0.6], 'LineWidth', 0.8);
        for r = 1:4
            yl = ylim(ax(r, iw));  plot(ax(r, iw), [1 1], yl, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0);  ylim(ax(r, iw), yl);
            xlim(ax(r, iw), [0 3]);  set(ax(r, iw), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(ax(r, iw), 'off');
        end
        xlabel(ax(4, iw), 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        lg = legend(ax(1, iw), hl, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'NumColumns', 2, 'AutoUpdate', 'off');
        lg.Title.String = sprintf('%s wall, plane init, \\surdP_0 = 0.1', WALLS{iw});
    end
    ylabel(ax(1, 1), 'a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold');
    ylabel(ax(2, 1), 'e_a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold');
    ylabel(ax(3, 1), 'c_{hat}  (red: c_{eff})', 'FontSize', FS, 'FontWeight', 'bold');
    ylabel(ax(4, 1), '\surd P^f_{55}', 'FontSize', FS, 'FontWeight', 'bold');
    linkaxes(ax(2, :), 'y');  linkaxes(ax(3, :), 'y');  linkaxes(ax(4, :), 'y');
    out = fullfile(fd, sprintf('fig_conly_ratioinnov_rtri_%s.png', set_name));  exportgraphics(f, out, 'Resolution', 150);  fprintf('\nfigure: %s\n', out);
end
