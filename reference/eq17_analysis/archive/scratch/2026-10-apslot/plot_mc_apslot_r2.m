% STATUS: SCRATCH -- 2026-10-18: figure for run_mc_apslot_r2, seed 1, plane wall. Columns raw y2 | whitened y2 (same estimator otherwise).
%   Rows: a-bar (true red, estimate blue); e_a; slope a-bar' (true red, slot 7 blue); running sum of the two update terms of the
%   a-bar row (y2: l42 e_am, blue; y1: l41 e_x1, grey). Output test_results/temp_1009_ws/fig_mc_apslot_r2.png.
function plot_mc_apslot_r2(seed_idx, arms, names, otag)
    if nargin < 1 || isempty(seed_idx); seed_idx = 1; end
    if nargin < 2 || isempty(arms); arms = {'raw', 'white'}; names = {'raw y_2 = ā_m', 'whitened y_2'}; end
    if nargin < 4; otag = ''; end   % 10-18: e.g. {'white_q0', 'white'}, '_q'
    od = fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), 'test_results', 'temp_1009_ws');
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  cG = [0.55 0.55 0.55];  FS = 18;  LFS = 15;  AB = char(257);  HAT = char(770);
    f = figure('Position', [40 40 1800 1500], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(4, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    ylims = {[0 1.05], [-0.3 0.3], [-0.1 0.9], [-3 3]};
    for ia = 1:2
        S = load(fullfile(od, sprintf('apr2_%s_plane_desc.mat', arms{ia})));  t = S.t;  q = seed_idx;
        rows = {
            {t, S.a_true(:, q), cT, 3.5, [AB '_z']; t, S.a_hat(:, q), cE, 1.4, [AB HAT '_z']}
            {t, S.a_true(:, q) - S.a_hat(:, q), cE, 1.4, ['e_{' AB '}']}
            {t, S.ap_true(:, q), cT, 3.5, [AB ''' (true slope)']; t, S.ap_hat(:, q), cE, 1.4, [AB HAT ''' (state)']}
            {t, cumsum(S.da_y1(:, q)), cG, 2.0, ['\Sigma ℓ_{41} e_{x1}  (y_1)']; t, cumsum(S.da_y2(:, q)), cE, 2.0, ['\Sigma ℓ_{42} e_{am}  (y_2)']}
        };
        ylab = {[AB '_z'], ['e_{' AB '}'], [AB ''''], ['push on ' AB HAT]};
        for ir = 1:4
            ax = nexttile(tl, (ir - 1) * 2 + ia);  hold on; box on;
            L = rows{ir};
            for j = 1:size(L, 1); plot(ax, L{j, 1}, L{j, 2}, '-', 'Color', L{j, 3}, 'LineWidth', L{j, 4}, 'DisplayName', L{j, 5}); end
            if ir == 2 || ir == 4; yline(ax, 0, ':', 'Color', [0.3 0.3 0.3], 'HandleVisibility', 'off'); end
            xlim(ax, [0 t(end)]);  ylim(ax, ylims{ir});
            if ir == 1; text(ax, 0.98, 0.9, sprintf('%s, plane wall, seed %d', names{ia}, q), 'Units', 'normalized', 'HorizontalAlignment', 'right', 'FontSize', LFS + 1, 'FontWeight', 'bold'); end
            if size(L, 1) > 1; legend(ax, 'Location', 'northwest', 'FontSize', LFS, 'FontWeight', 'bold'); end
            if ir == 4; xlabel(ax, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold'); end
            if ia == 1; ylabel(ax, ylab{ir}, 'FontSize', FS, 'FontWeight', 'bold'); end
            set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2);  grid(ax, 'off');
        end
    end
    out = fullfile(od, ['fig_mc_apslot_r2' otag '.png']);  exportgraphics(f, out, 'Resolution', 120);  fprintf('figure: %s\n', out);
end
