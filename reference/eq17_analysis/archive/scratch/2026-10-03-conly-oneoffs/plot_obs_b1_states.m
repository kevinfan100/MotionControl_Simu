% FORK OF test_script/scratch/plot_obs_b1_state.m (2026-09-14) | PURPOSE: per-state picture of analyze_obs_b1_states.m output,
%   Meng left, canon right, shared y per row. Rows: (1) w_d/R; (2) CRLB/sqrt(P[0]) per 500-step window for every free slot of the
%   b1 model (b1 added at b1_hat = 0 on the constant-b records); (3) the weakest prior-scaled direction lambda_min(D G D) for
%   constant b / b1 added / b1 run. Below the gray line = the window's data are weaker than the prior along that direction.
%   | EXPIRES: when b1_state is closed | 產線改動不會自動跟上
function plot_obs_b1_states(win)
    if nargin < 1 || isempty(win); win = 500; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    TR = {'meng', 'canon'};  FS = 15; LFS = 11; AXLW = 1.8;
    CS = struct('dw1', [0.55 0.55 0.55], 'dw2', [0.40 0.40 0.40], 'dw3', [0.25 0.25 0.25], 'a_w', [0 0.55 0.2], ...
                'b', [0 0.2 0.9], 'b1', [0.85 0.33 0.10], 'm1', [0.6 0.4 0.8], 'm2', [0.45 0.25 0.65]);
    f = figure('Units', 'inches', 'Position', [0 0 13 10.5], 'Color', 'w', 'Visible', 'off');
    tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');  ax = gobjects(3, 2);
    for c = 1:2
        S = load(fullfile(od, sprintf('analyze_obs_b1_states_%s_w%d.mat', TR{c}, win)));  T_END = ceil(S.T);
        ax(1, c) = nexttile(c); hold on;
        plot(S.t, S.hd, '-', 'Color', [0.2 0.2 0.2], 'LineWidth', 1.6);
        local_style(c == 1, 'w_d / R', false, T_END, FS, AXLW);  set(gca, 'YScale', 'log');
        ax(2, c) = nexttile(2 + c); hold on;  H = [];  Lg = {};
        R = S.V2;
        for j = 1:numel(R.free)
            nm = S.names{R.free(j)};  r = R.ratio(:, j);  r(~isfinite(r)) = NaN;
            lw = 1.0; if any(strcmp(nm, {'a_w', 'b', 'b1'})); lw = 2.0; end
            H(end+1) = plot(R.t_win, r, '-o', 'Color', CS.(nm), 'LineWidth', lw, 'MarkerSize', 3); %#ok<AGROW>
            Lg{end+1} = strrep(strrep(strrep(nm, 'dw', '\deltaw_'), 'a_w', '\^a_w'), 'b1', 'b_1'); %#ok<AGROW>
        end
        yline(1, '-', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, 'HandleVisibility', 'off');
        legend(H, Lg, 'Location', 'northoutside', 'Orientation', 'horizontal', 'NumColumns', 4, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        local_style(c == 1, 'CRLB / \surdP[0]   (b_1 model)', false, T_END, FS, AXLW);  set(gca, 'YScale', 'log');
        ax(3, c) = nexttile(4 + c); hold on;
        h1 = plot(S.V1.t_win, S.V1.lam_min, '-o', 'Color', [0 0.2 0.9], 'LineWidth', 1.8, 'MarkerSize', 3);
        h2 = plot(S.V2.t_win, S.V2.lam_min, '-o', 'Color', [0.85 0.33 0.10], 'LineWidth', 1.8, 'MarkerSize', 3);
        h3 = plot(S.V3.t_win, S.V3.lam_min, '--s', 'Color', [0.85 0.33 0.10], 'LineWidth', 1.3, 'MarkerSize', 3);
        yline(1, '-', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, 'HandleVisibility', 'off');
        legend([h1 h2 h3], {'constant b', 'b_1 added', 'b_1 run'}, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        local_style(c == 1, '\lambda_{min}(D G D)', true, T_END, FS, AXLW);  set(gca, 'YScale', 'log');
    end
    for r = 2:3; linkaxes(ax(r, :), 'y'); end
    png = fullfile(od, sprintf('obs_b1_states_w%d.png', win));
    exportgraphics(f, png, 'Resolution', 150);  close(f);  fprintf('saved %s\n', png);
end

function local_style(is_left, ylab, is_bottom, T_END, FS, AXLW)
    if is_left; ylabel(ylab, 'FontSize', FS, 'FontWeight', 'bold'); end
    if is_bottom; xlabel('time  [s]', 'FontSize', FS, 'FontWeight', 'bold'); end
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off; xlim([0 T_END]);
end
