% STATUS: ACTIVE (scratch) | PURPOSE: Scenario-A figure set in the CURRENT
%   house per-seed style (copied from plot_formC_state_panels.m: 2 seed
%   COLUMNS side by side, shared row y-limits, C_TRUE/C_EST/C_BAND, FS 15,
%   no titles, stats to console). Pages:
%     meng_sA_delta : 3 rows = delta x/y/z [um], 2 seed columns
%     meng_sA_a<ax> : per axis, rows = a tracking / rel err [%] /
%                     e_a with +-sqrt(P66) honesty band
%   | EXPIRES: with run_meng_ch4_sA.
function plot_meng_ch4_sApanels(out, out_dir)

    if nargin < 2 || isempty(out_dir)
        here = fileparts(mfilename('fullpath'));
        root = fileparts(fileparts(here));
        out_dir = fullfile(root, 'test_results', 'meng_ch4_s0');
    end
    C_TRUE = [0.8 0 0];  C_EST = [0 0.2 0.9];  C_BAND = [0.75 0.80 0.95];
    FS = 15; LFS = 13; LW = 1.8; PAD = 0.04;
    axn = 'xyz';
    t = out.t(:);
    r1 = out.E98{1};  r2 = out.E98{2};  sq = out.seeds(1:2);

    % ===== page 1: tracking error, 3 axes x 2 seeds [um] =====
    f = figure('Position', [40 40 1500 900], 'Color', 'w', 'Visible', 'off');
    tl = tiledlayout(f, 3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ax = 1:3
        d1 = out.pd(:, ax) - r1.p_true(:, ax);
        d2 = out.pd(:, ax) - r2.p_true(:, ax);
        yl = local_lims([d1; d2], PAD);
        for c = 1:2
            a = nexttile(tl, 2*(ax-1) + c); hold(a, 'on');
            if c == 1; dd = d1; else; dd = d2; end
            plot(a, t, dd, '-', 'Color', C_EST, 'LineWidth', 0.5, ...
                 'DisplayName', sprintf('seed %d', sq(c)));
            ylim(a, yl); box(a, 'on'); set(a, 'FontSize', LFS, 'LineWidth', 1.2);
            if c == 1
                ylabel(a, sprintf('$\\delta %c\\ (\\mu m)$', axn(ax)), ...
                       'Interpreter', 'latex', 'FontSize', FS);
            end
            if ax == 1
                legend(a, 'Location', 'northoutside', 'Orientation', ...
                       'horizontal', 'FontSize', LFS);
            end
            if ax == 3; xlabel(a, 't (s)', 'FontSize', FS); end
        end
    end
    exportgraphics(f, fullfile(out_dir, 'meng_sA_delta.png'), 'Resolution', 150);

    % ===== pages 2-4: per axis, gain / rel err / honesty =====
    for ax = 1:3
        f = figure('Position', [60 60 1500 900], 'Color', 'w', 'Visible', 'off');
        tl = tiledlayout(f, 3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
        aT = {r1.a_true(:, ax), r2.a_true(:, ax)};
        aH = {r1.a_hat(:, ax),  r2.a_hat(:, ax)};
        er = {100*(aH{1}-aT{1})./aT{1}, 100*(aH{2}-aT{2})./aT{2}};
        ea = {aH{1}-aT{1}, aH{2}-aT{2}};
        sg = {sqrt(max(r1.P_a(:, ax), 0)), sqrt(max(r2.P_a(:, ax), 0))};
        yl1 = local_lims([aT{1}; aH{1}; aT{2}; aH{2}], PAD);
        yl2 = local_lims([er{1}; er{2}], PAD);
        yl3 = local_lims([ea{1}+sg{1}; ea{1}-sg{1}; ea{2}+sg{2}; ea{2}-sg{2}], PAD);
        for c = 1:2
            a1 = nexttile(tl, c); hold(a1, 'on');
            plot(a1, t, aT{c}, '-', 'Color', C_TRUE, 'LineWidth', LW+0.6, ...
                 'DisplayName', 'true');
            plot(a1, t, aH{c}, '-', 'Color', C_EST, 'LineWidth', 1.0, ...
                 'DisplayName', sprintf('estimate, seed %d', sq(c)));
            ylim(a1, yl1); box(a1, 'on'); set(a1, 'FontSize', LFS, 'LineWidth', 1.2);
            if c == 1
                ylabel(a1, sprintf('$a_%c\\ (\\mu m/pN)$', axn(ax)), ...
                       'Interpreter', 'latex', 'FontSize', FS);
            end
            legend(a1, 'Location', 'northoutside', 'Orientation', 'horizontal', ...
                   'FontSize', LFS);

            a2 = nexttile(tl, 2 + c); hold(a2, 'on');
            plot(a2, t, er{c}, '-', 'Color', C_EST, 'LineWidth', 0.7);
            yline(a2, 0, 'k--');
            ylim(a2, yl2); box(a2, 'on'); set(a2, 'FontSize', LFS, 'LineWidth', 1.2);
            if c == 1
                ylabel(a2, sprintf('$\\Delta a_%c/a\\ (\\%%)$', axn(ax)), ...
                       'Interpreter', 'latex', 'FontSize', FS);
            end

            a3 = nexttile(tl, 4 + c); hold(a3, 'on');
            fill(a3, [t; flipud(t)], [sg{c}; flipud(-sg{c})], C_BAND, ...
                 'FaceAlpha', 0.6, 'EdgeColor', 'none');
            plot(a3, t, ea{c}, '-', 'Color', C_EST, 'LineWidth', 0.7);
            yline(a3, 0, 'k--');
            ylim(a3, yl3); box(a3, 'on'); set(a3, 'FontSize', LFS, 'LineWidth', 1.2);
            if c == 1
                ylabel(a3, sprintf('$e_{a_%c}\\ \\pm\\sqrt{P_{66}}$', axn(ax)), ...
                       'Interpreter', 'latex', 'FontSize', FS);
            end
            if true; xlabel(a3, 't (s)', 'FontSize', FS); end
        end
        exportgraphics(f, fullfile(out_dir, sprintf('meng_sA_a%c.png', axn(ax))), ...
                       'Resolution', 150);
    end
    fprintf('[sApanels] wrote meng_sA_delta + meng_sA_ax/ay/az -> %s\n', out_dir);
end


function yl = local_lims(v, pad)
    lo = min(v); hi = max(v); sp = hi - lo;
    if sp <= 0; sp = max(abs(hi), 1e-12); end
    yl = [lo - pad*sp, hi + pad*sp];
end
