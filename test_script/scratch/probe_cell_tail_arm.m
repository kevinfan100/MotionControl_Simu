% PURPOSE (2026-09-18): is the +0.018 bump at 1.1 s (w_d ~ 7-8 R) of the [1/1] cell T arms the PLANT's far-field tail
%   (cell_cperp_fig11 beyond the 5.5 R data edge: 1 - 1.141/(g + 3.23)) disagreeing with the [1/1] band fit (1.056, 2.197)
%   by up to +0.020 at 7 R (probe_cell_plant_vs_law.m), or an estimator lag?
%   [hypothesis] plant tail   opponent: estimator lag   separate: swap the tail for the [1/1] law itself above 6 R
%   (smooth blend 5-6 R, data below 5 R) and rerun cell T, A and B, same 10 seeds. Bump gone => tail.
%   Output test_results/temp_11_two_walls/cell_{A,B}_T_lawtail.mat + console; figure cell_T_tail_compare.png (e_a, 4 panels).
%   EXPIRES: with the [1/1] two-wall line
function probe_cell_tail_arm()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration')); addpath(here);
    l11 = @(h) ((h - 1) + 1.056) ./ ((h - 1) + 2.197);
    plant = @(h) 1 / local_blend(h, l11);
    run_11_two_walls(struct('seeds', 1:10, 'plants', {{'cell'}}, 'versions', {{'A', 'B'}}, 'arms', {{'T'}}, ...
                            'plant_cperp_cell', plant, 'tag_suffix', '_lawtail'));
    % ---- figure: e_a of the data-tail and the law-tail plants, A | B ----
    rd = fullfile(root, 'test_results', 'temp_11_two_walls');
    FS = 15; LFS = 11; COL = [0 0.2 0.9]; BANDC = [0.45 0.55 0.95];
    f = figure('Position', [40 40 1500 700], 'Color', 'w', 'Visible', 'off');
    tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    rows = {'', '_lawtail'};  rown = {'data plant (tail 1 - 1.141/(g + 3.23) above 5.5 R)', 'law-tail plant ([1/1] band fit above 6 R)'};
    for i = 1:2
        for v = 1:2
            ver = char('A' + v - 1);
            r = load(fullfile(rd, sprintf('cell_%s_T%s.mat', ver, rows{i})));
            t = r.t; ea = r.a_true - r.a_hat; em = mean(ea, 2); es = std(ea, 0, 2);
            nexttile((i - 1) * 2 + v); hold on; box on;
            fill([t; flipud(t)], [em - es; flipud(em + es)], BANDC, 'FaceAlpha', 0.30, 'EdgeColor', 'none', 'HandleVisibility', 'off');
            yline(0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off');
            h = plot(t, em, '-', 'Color', COL, 'LineWidth', 2.0, 'DisplayName', sprintf('version %s, %s: e_a = a_{true} - a_{hat} (seed mean \\pm \\sigma_{seed})', ver, rown{i}));
            for s = [0.5 1.5 3.5]; xline(s, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, 'HandleVisibility', 'off'); end
            xlim([0 t(end)]); ylim([-0.05 0.05]);
            if v == 1; ylabel('e_a  [a / a_{nom}]', 'FontSize', FS, 'FontWeight', 'bold'); end
            if i == 2; xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold'); end
            legend(h, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
            set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2, 'Box', 'on'); grid off;
            win = t >= 0.9 & t <= 1.25; [mx, k] = max(em .* win);
            fprintf('cell %s T%-9s: e_a peak 0.9-1.25 s %+.4f at %.2f s | descent worst %+.4f | hold %+.4f sd %.4f\n', ver, rows{i}, mx, t(k), ...
                    em(find(abs(em .* (t >= 0.5 & t < 1.5)) == max(abs(em .* (t >= 0.5 & t < 1.5))), 1)), mean(em(t >= 3.5)), mean(es(t >= 3.5)));
        end
    end
    fn = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures', 'cell_T_tail_compare.png');
    exportgraphics(f, fn, 'Resolution', 150); close(f); fprintf('figure: %s\n', fn);
end

function a = local_blend(h, l11)
%LOCAL_BLEND  data fit below 5 R, [1/1] band-fit law above 6 R, smoothstep between.
    if h <= 5
        a = 1 / cell_cperp_fig11(h);
    elseif h >= 6
        a = l11(h);
    else
        s = (h - 5);  w = s^2 * (3 - 2 * s);
        a = (1 - w) / cell_cperp_fig11(h) + w * l11(h);
    end
end
