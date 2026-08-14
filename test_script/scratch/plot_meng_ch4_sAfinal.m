% STATUS: ACTIVE (scratch) | PURPOSE: final Scenario-A figure set in the house
%   style (copied from plot_var_ahat_6state.m: COL_TRUE/COL_HAT/BANDC, FS 18
%   bold, mean +/- across-seed sigma band, stats to console). Estimation arm
%   only, per-axis lambda_f. Figures: A gain vs true, B tracking error,
%   C relative gain error. | EXPIRES: with run_meng_ch4_sA.
function plot_meng_ch4_sAfinal(out, out_dir)

    if nargin < 2 || isempty(out_dir)
        here = fileparts(mfilename('fullpath'));
        root = fileparts(fileparts(here));
        out_dir = fullfile(root, 'test_results', 'meng_ch4_s0');
    end
    COL_TRUE = [0.8 0 0]; COL_HAT = [0 0.2 0.9]; BANDC = [0.45 0.55 0.95];
    FS = 18; LFS = 13; axl = 'xyz';

    t = out.t(:);  Ns = numel(out.E98);
    ah = [];  at = [];  dd = [];
    for s = 1:Ns
        r = out.E98{s};
        ah = cat(3, ah, r.a_hat);
        at = cat(3, at, r.a_true);
        dd = cat(3, dd, out.pd - r.p_true);
    end
    ah_m = mean(ah, 3);  ah_s = std(ah, 0, 3);
    at_m = mean(at, 3);
    erel = (ah - at) ./ at * 100;
    bias = mean(erel, 3);  sprd = std(erel, 0, 3);

    % console stats (far = t<3, near = t>8)
    wf = t < 3;  wn = t > 8;
    for ax = 1:3
        fprintf('[sAfinal ax=%c] far bias %+.1f%% | near bias %+.1f%% | std dxyz far/near = %.1f/%.1f nm (per-seed mean)\n', ...
            axl(ax), mean(bias(wf, ax)), mean(bias(wn, ax)), ...
            1e3*mean(squeeze(std(dd(wf, ax, :), 0, 1))), ...
            1e3*mean(squeeze(std(dd(wn, ax, :), 0, 1))));
    end

    % ===== FIG A: a_hat mean +/- seed band vs a_true =====
    f = figure('Position', [60 60 1100 900], 'Color', 'w', 'Visible', 'off');
    tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ax = 1:3
        nexttile; hold on; box on;
        yb = [ah_m(:, ax) - ah_s(:, ax); flipud(ah_m(:, ax) + ah_s(:, ax))];
        fill([t; flipud(t)], yb, BANDC, 'FaceAlpha', 0.30, 'EdgeColor', 'none', ...
             'DisplayName', 'a hat \pm \sigma_{seed}');
        plot(t, at_m(:, ax), '-', 'Color', COL_TRUE, 'LineWidth', 2.0, ...
             'DisplayName', 'a true');
        plot(t, ah_m(:, ax), '-', 'Color', COL_HAT, 'LineWidth', 1.4, ...
             'DisplayName', 'a hat mean');
        ylabel(sprintf('a_%c (\\mum/pN)', axl(ax)), 'FontSize', FS, 'FontWeight', 'bold');
        set(gca, 'FontSize', LFS, 'LineWidth', 1.2);
        if ax == 1
            legend('Location', 'northoutside', 'Orientation', 'horizontal', ...
                   'FontSize', LFS);
        end
        if ax == 3; xlabel('t (s)', 'FontSize', FS, 'FontWeight', 'bold'); end
    end
    exportgraphics(f, fullfile(out_dir, 'sAfinal_A_gain.png'), 'Resolution', 150);

    % ===== FIG B: tracking error, both seeds overlaid =====
    f = figure('Position', [90 90 1100 900], 'Color', 'w', 'Visible', 'off');
    tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ax = 1:3
        nexttile; hold on; box on;
        for s = Ns:-1:1
            cc = COL_HAT; lw = 0.5;
            if s > 1; cc = BANDC; end
            plot(t, 1e3*dd(:, ax, s), '-', 'Color', cc, 'LineWidth', lw, ...
                 'DisplayName', sprintf('seed %d', out.seeds(s)));
        end
        ylabel(sprintf('\\delta%c (nm)', axl(ax)), 'FontSize', FS, 'FontWeight', 'bold');
        set(gca, 'FontSize', LFS, 'LineWidth', 1.2);
        if ax == 1
            legend('Location', 'northoutside', 'Orientation', 'horizontal', ...
                   'FontSize', LFS);
        end
        if ax == 3; xlabel('t (s)', 'FontSize', FS, 'FontWeight', 'bold'); end
    end
    exportgraphics(f, fullfile(out_dir, 'sAfinal_B_error.png'), 'Resolution', 150);

    % ===== FIG C: relative gain error, bias + spread band =====
    f = figure('Position', [120 120 1100 900], 'Color', 'w', 'Visible', 'off');
    tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ax = 1:3
        nexttile; hold on; box on;
        yb = [bias(:, ax) - sprd(:, ax); flipud(bias(:, ax) + sprd(:, ax))];
        fill([t; flipud(t)], yb, BANDC, 'FaceAlpha', 0.30, 'EdgeColor', 'none', ...
             'DisplayName', 'bias \pm spread');
        plot(t, bias(:, ax), '-', 'Color', COL_HAT, 'LineWidth', 1.4, ...
             'DisplayName', 'bias (seed mean)');
        yline(0, 'k--', 'HandleVisibility', 'off');
        ylabel(sprintf('\\Deltaa_%c/a (%%)', axl(ax)), 'FontSize', FS, 'FontWeight', 'bold');
        ylim([-40 60]);
        set(gca, 'FontSize', LFS, 'LineWidth', 1.2);
        if ax == 1
            legend('Location', 'northoutside', 'Orientation', 'horizontal', ...
                   'FontSize', LFS);
        end
        if ax == 3; xlabel('t (s)', 'FontSize', FS, 'FontWeight', 'bold'); end
    end
    exportgraphics(f, fullfile(out_dir, 'sAfinal_C_relerr.png'), 'Resolution', 150);
    fprintf('figures: sAfinal_A_gain / sAfinal_B_error / sAfinal_C_relerr in %s\n', out_dir);
end
