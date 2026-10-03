% PURPOSE (2026-09-18, user: "with or without an a state -- a simple [1/1] comparison"): version A (no a state: c0, d1
%   constant states, gain computed from the law) against version B (a state along the law, d1 state, c0 fixed by the seed)
%   on the SAME runs (same plant, seed, prior, 10 paired seeds) of run_11_two_walls.m: cell T / cell P / plane T / plane N.
%   In the [1/1] writing an a slot makes c0 a dead state (it enters only through d1 - c0 in the slope, and H2 reads the
%   slot), so B IS "A plus an a state" with the one degree of freedom that cannot survive that change removed.
%     figure  a_state11_compare.png: row 1 e_a = a_true - a_hat, A (blue) and B (orange), seed mean +- sigma_seed;
%             row 2 d1_hat of both with the reference; one column per arm.
%     console paired statistics per arm: descent worst |mean|, osc mean, hold mean for A and B, and the paired A - B
%             hold difference (mean +- sd over seeds) -- the seed-independent signature.
%   Output reference/eq17_analysis/derivation/figures/ | EXPIRES: with the [1/1] two-wall line
function plot_11_a_state_compare(res_dir)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));
    if nargin < 1 || isempty(res_dir); res_dir = fullfile(root, 'test_results', 'temp_11_two_walls'); end
    od = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    ARMS = {'cell', 'T'; 'cell', 'P'; 'plane', 'T'; 'plane', 'N'};
    CA = [0 0.2 0.9]; CB = [0.85 0.45 0]; FS = 15; LFS = 11; AXLW = 2.0;
    f = figure('Position', [40 40 1900 800], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(2, 4, 'TileSpacing', 'compact', 'Padding', 'compact');
    fprintf('%-8s | %-28s | %-28s | %s\n', 'arm', 'A: desc worst / osc / hold', 'B: desc worst / osc / hold', 'paired hold A-B (mean +- sd) | d1 end A / B');
    for k = 1:4
        pl = ARMS{k, 1}; arm = ARMS{k, 2};
        RA = load(fullfile(res_dir, sprintf('%s_A_%s.mat', pl, arm)));  RB = load(fullfile(res_dir, sprintf('%s_B_%s.mat', pl, arm)));
        t = RA.t; seg = RA.seg;  eA = RA.a_true - RA.a_hat;  eB = RB.a_true - RB.a_hat;
        desc = t >= seg(1) & t < seg(2);  osc = t >= seg(2) & t < seg(3);  hold_m = t >= seg(3);
        mA = mean(eA, 2); sA = std(eA, 0, 2); mB = mean(eB, 2); sB = std(eB, 0, 2);
        dh = mean(eA(hold_m, :), 1) - mean(eB(hold_m, :), 1);                    % paired per seed
        fprintf('%-8s | %+.4f / %+.4f / %+.4f   | %+.4f / %+.4f / %+.4f   | %+.4f +- %.4f | %.3f / %.3f\n', [pl ' ' arm], ...
                max(abs(mA(desc))), mean(mA(osc)), mean(mA(hold_m)), max(abs(mB(desc))), mean(mB(osc)), mean(mB(hold_m)), ...
                mean(dh), std(dh), mean(RA.d1_hat(end, :)), mean(RB.d1_hat(end, :)));
        % ---- row 1: e_a overlay ----
        nexttile(k); hold on; box on;
        fill([t; flipud(t)], [mA - sA; flipud(mA + sA)], CA, 'FaceAlpha', 0.18, 'EdgeColor', 'none', 'HandleVisibility', 'off');
        fill([t; flipud(t)], [mB - sB; flipud(mB + sB)], CB, 'FaceAlpha', 0.18, 'EdgeColor', 'none', 'HandleVisibility', 'off');
        yline(0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off');
        hA = plot(t, mA, '-', 'Color', CA, 'LineWidth', 2.0, 'DisplayName', 'A  no a state: e_a = a_{true} - a_{hat}');
        hB = plot(t, mB, '-', 'Color', CB, 'LineWidth', 2.0, 'DisplayName', 'B  a state: e_a');
        h0 = plot(NaN, NaN, 'w.', 'DisplayName', sprintf('%s plant, arm %s: seed (c_0, d_1) = (%.3f, %.3f), prior %.2f', upper(pl), arm, RA.seed_cd(1), RA.seed_cd(2), RA.sig(1)));
        for s = seg; xline(s, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, 'HandleVisibility', 'off'); end
        xlim([0 t(end)]); ylim([-0.08 0.08]);
        if k == 1; ylabel('e_a  [a / a_{nom}]', 'FontSize', FS, 'FontWeight', 'bold'); end
        legend([h0 hA hB], 'Location', 'northoutside', 'Orientation', 'vertical', 'FontSize', LFS - 1, 'FontWeight', 'bold', 'Box', 'on');
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
        % ---- row 2: d1_hat overlay ----
        nexttile(4 + k); hold on; box on;
        dA = mean(RA.d1_hat, 2); dsA = std(RA.d1_hat, 0, 2); dB = mean(RB.d1_hat, 2); dsB = std(RB.d1_hat, 0, 2);
        fill([t; flipud(t)], [dA - dsA; flipud(dA + dsA)], CA, 'FaceAlpha', 0.18, 'EdgeColor', 'none', 'HandleVisibility', 'off');
        fill([t; flipud(t)], [dB - dsB; flipud(dB + dsB)], CB, 'FaceAlpha', 0.18, 'EdgeColor', 'none', 'HandleVisibility', 'off');
        if strcmp(pl, 'plane')
            cm0 = mean(RA.c0_hat, 2);  g = max(RA.wd - RA.W_S, 0.02);  ap = zeros(size(g));
            for i = 1:numel(g); [~, c] = calc_correction_functions(1 + g(i)); ap(i) = 1 / c; end
            d1_ref = (g + cm0) ./ ap - g;  ref_lab = 'd_{1,eff}(w_d; c_{0,hat} of A)  plane';
        else
            d1_ref = 1.1871 * ones(size(t));  ref_lab = 'd_1  cell band fit';   % cell_cperp_delta (2026-09-20); data-fit plant: 1.197
        end
        hr = plot(t, d1_ref, '-', 'Color', [0.8 0 0], 'LineWidth', 1.4, 'DisplayName', ref_lab);
        hA = plot(t, dA, '-', 'Color', CA, 'LineWidth', 2.0, 'DisplayName', 'd_{1,hat}  A');
        hB = plot(t, dB, '-', 'Color', CB, 'LineWidth', 2.0, 'DisplayName', 'd_{1,hat}  B');
        for s = seg; xline(s, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, 'HandleVisibility', 'off'); end
        xlim([0 t(end)]); ylim([-0.5 2.0]);
        if k == 1; ylabel('d_1', 'FontSize', FS, 'FontWeight', 'bold'); end
        xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        legend([hA hB hr], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS - 1, 'FontWeight', 'bold', 'Box', 'on');
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
    end
    fn = fullfile(od, 'a_state11_compare.png');  exportgraphics(f, fn, 'Resolution', 150); close(f);  fprintf('figure: %s\n', fn);
end
