% PURPOSE (2026-09-18, user: "set w_s = 0 in the simulation and let me see"): two quick tests on the T arms (A and B,
%   plane and cell, same 10 seeds as run_11_two_walls):
%     (i)  identity   w_s = 0 AND the two seed constants shifted by -1 (c0 - 1, d1 - 1): the law depends on g + c0 and
%                     g + d1 only, so this must reproduce the w_s = 1 runs to roundoff -> "the expression is a relabel"
%     (ii) mis-set    w_s = 0 with the seeds unchanged: a 1 R error of the reference height -> what a wall-position
%                     error of one radius does to A (c0 must absorb it against a 0.05 prior) and to B (a slot absorbs)
%   Console: max |e_a(identity) - e_a(baseline)| per arm; descent worst / hold for the mis-set arms; c0_hat end (A).
%   Figure ws_zero_probe.png: e_a of baseline / identity / mis-set for A and B, plane and cell.
%   EXPIRES: with the [1/1] two-wall line
function probe_ws_zero()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration')); addpath(here);
    rd = fullfile(root, 'test_results', 'temp_11_two_walls');
    % 2026-09-18 evening: the baseline of run_11_two_walls is now w_s = 0 (wall surface) with the shifted constants, so the
    % two probes are re-expressed relative to it: (i) identity = w_s back at the contact height 1 with +1 on both constants;
    % (ii) mis-set = w_s = 1 with the surface-coordinate constants unchanged (reference 1 R too HIGH now). Same logic,
    % same magnitudes (the first run, 09-18 afternoon, was the mirror image and gave 4e-15 / 0 for the identity).
    common = struct('seeds', 1:10, 'plants', {{'plane', 'cell'}}, 'versions', {{'A', 'B'}}, 'arms', {{'T'}});
    o = common; o.W_S = 1; o.seed_shift = +1; o.tag_suffix = '_ws0id';   run_11_two_walls(o);
    o = common; o.W_S = 1; o.seed_shift = 0;  o.tag_suffix = '_ws0mis';  run_11_two_walls(o);
    FS = 14; LFS = 10;  C = struct('base', [0 0.2 0.9], 'id', [0 0.6 0.2], 'mis', [0.85 0.45 0]);
    f = figure('Position', [40 40 1500 800], 'Color', 'w', 'Visible', 'off');
    tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');  k = 0;
    fprintf('\n%-10s | %-14s | %-26s | %s\n', 'arm', 'identity - base', 'mis-set desc worst / hold', 'mis-set c0 / d1 end (A: seed 0.016/1.161 must go to ~-0.98/0.16 on the plane)');
    for pl = {'plane', 'cell'}
        for ver = {'A', 'B'}
            k = k + 1;
            b  = load(fullfile(rd, sprintf('%s_%s_T.mat', pl{1}, ver{1})));
            id = load(fullfile(rd, sprintf('%s_%s_T_ws0id.mat', pl{1}, ver{1})));
            ms = load(fullfile(rd, sprintf('%s_%s_T_ws0mis.mat', pl{1}, ver{1})));
            t = b.t; seg = b.seg;  desc = t >= seg(1) & t < seg(2);  hold_m = t >= seg(3);
            eb = b.a_true - b.a_hat;  ei = id.a_true - id.a_hat;  em = ms.a_true - ms.a_hat;
            dmax = max(abs(ei(:) - eb(:)));  mm = mean(em, 2);
            fprintf('%-10s | %.2e       | %+.4f / %+.4f            | %.3f / %.3f\n', [pl{1} ' ' ver{1}], dmax, max(abs(mm(desc))), mean(mm(hold_m)), ...
                    mean(ms.c0_hat(end, :)), mean(ms.d1_hat(end, :)));
            nexttile(k); hold on; box on;
            yline(0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off');
            h1 = plot(t, mean(eb, 2), '-', 'Color', C.base, 'LineWidth', 2.2, 'DisplayName', 'w_s = 0 (surface), seed (c_0, d_1)  [baseline]');
            h2 = plot(t, mean(ei, 2), '--', 'Color', C.id, 'LineWidth', 1.6, 'DisplayName', 'w_s = 1, seed (c_0 + 1, d_1 + 1)  [identity]');
            h3 = plot(t, mean(em, 2), '-', 'Color', C.mis, 'LineWidth', 1.8, 'DisplayName', 'w_s = 1, seed unchanged  [reference 1 R too high]');
            h0 = plot(NaN, NaN, 'w.', 'DisplayName', sprintf('%s plant, arm T, version %s: e_a = a_{true} - a_{hat} (seed mean)', upper(pl{1}), ver{1}));
            for s = seg; xline(s, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, 'HandleVisibility', 'off'); end
            xlim([0 t(end)]); ylim([-0.3 0.3]);
            if k == 1 || k == 3; ylabel('e_a  [a / a_{nom}]', 'FontSize', FS, 'FontWeight', 'bold'); end
            if k >= 3; xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold'); end
            legend([h0 h1 h2 h3], 'Location', 'northoutside', 'Orientation', 'vertical', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
            set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2, 'Box', 'on'); grid off;
        end
    end
    fn = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures', 'ws_zero_probe.png');
    exportgraphics(f, fn, 'Resolution', 150); close(f); fprintf('figure: %s\n', fn);
end
