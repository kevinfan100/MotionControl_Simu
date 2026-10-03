% PURPOSE (2026-09-23, user: "the hold-unobservability figure can be a plain contrast, in the project's figure style";
%   then "the same style but showing 6/1 at this speed"):
%   plane | cell side by side, law A (delta, p), plane init, T_d = 2 s, one seed; rows share y across the columns.
%     mode 'sigma' (default) -> obs_ramp_hold_contrast.png
%        row 1  commanded height w_d / R vs time            (hold - constant-velocity descent - hold)
%        row 2  sigma_5 of the whitened O accumulated from t = 0  (v_5 ~ p at this T_d, see obs_ramp_V_<wall>.png)
%        row 3  sigma_6 of the same                              (v_6 ~ delta)
%     mode 'ratio'           -> obs_ramp_ratio_contrast.png
%        row 1  commanded height
%        row 2  sigma_6 / sigma_1 (min / max of the same accumulated O)
%   Linear axes: the constants' sigma (and the ratio) are exactly flat in the holds and grow only while the command moves.
%   Reads test_results/temp_plaw_walls/obs_ramp_sigma_curves_{plane,cell}.mat written by plot_obs_ramp_sigma_curves.m.
%   Style: figure-style.md (no grid, no title, box on, legend northoutside horizontal, exportgraphics 150).
%   EXPIRES: with the T_d design line
function plot_obs_ramp_hold_contrast(mode)
    if nargin < 1 || isempty(mode); mode = 'sigma'; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    rd = fullfile(root, 'test_results', 'temp_plaw_walls');
    WALLS = {'plane', 'cell'};  WLAB = {'PLANE wall', 'CELL wall'};  nW = numel(WALLS);
    COL_LINE = [0 0.2 0.9];  COL_CMD = [0.2 0.2 0.2];  COL_SEG = [0.5 0.5 0.5];
    FS = 16; LFS = 12; AXLW = 2.0;
    switch mode
        case 'sigma'
            ROWS = {struct('get', @(s) s.SIGw(5, :), 'ylab', '\sigma_5', 'leg', '\sigma_5  (v_5 \approx p)   accumulated from t = 0'), ...
                    struct('get', @(s) s.SIGw(6, :), 'ylab', '\sigma_6', 'leg', '\sigma_6  (v_6 \approx \delta)   accumulated from t = 0')};
            fn = 'obs_ramp_hold_contrast.png';
        case 'ratio'
            ROWS = {struct('get', @(s) s.SIGw(6, :) ./ s.SIGw(1, :), 'ylab', '\sigma_6 / \sigma_1', ...
                           'leg', '\sigma_6 / \sigma_1   (min / max of the O accumulated from t = 0)')};
            fn = 'obs_ramp_ratio_contrast.png';
        otherwise
            error('mode must be ''sigma'' or ''ratio''');
    end
    nR = 1 + numel(ROWS);
    S = cell(1, nW);
    for w = 1:nW
        S{w} = load(fullfile(rd, sprintf('obs_ramp_sigma_curves_%s.mat', WALLS{w})), 'SIGw', 'HKw', 'TKw', 'Td_w', 'T_HOLD');
    end
    f = figure('Position', [60 60 560 * nW + 120, 300 * nR + 60], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(nR, nW, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax = gobjects(nR, nW);
    for w = 1:nW
        s = S{w};  t = s.TKw(:);
        % ---- row 1: commanded height
        ax(1, w) = nexttile(w); hold on; box on;
        h1 = plot(t, s.HKw(:), '-', 'Color', COL_CMD, 'LineWidth', 2.0, ...
                  'DisplayName', sprintf('w_d / R   %s,  A (\\delta, p),  plane init,  T_d = %g s', WLAB{w}, s.Td_w));
        legend(h1, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        if w == 1; ylabel('w_d  (R)', 'FontSize', FS, 'FontWeight', 'bold'); end
        % ---- the data rows
        for r = 1:numel(ROWS)
            ax(1 + r, w) = nexttile(r * nW + w); hold on; box on;
            h = plot(t, ROWS{r}.get(s), '-', 'Color', COL_LINE, 'LineWidth', 2.0, 'DisplayName', ROWS{r}.leg);
            if w == 1
                legend(h, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
                ylabel(ROWS{r}.ylab, 'FontSize', FS, 'FontWeight', 'bold');
            end
        end
        xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        for r = 1:nR
            set(ax(r, w), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid(ax(r, w), 'off');
            xlim(ax(r, w), [0 t(end)]);
        end
    end
    for r = 1:nR; linkaxes(ax(r, :), 'y'); end
    for w = 1:nW
        s = S{w};  seg = [s.T_HOLD, s.T_HOLD + s.Td_w];
        for r = 1:nR
            yl = ylim(ax(r, w));
            for x = seg; plot(ax(r, w), [x x], yl, '--', 'Color', COL_SEG, 'LineWidth', 1.0, 'HandleVisibility', 'off'); end
            ylim(ax(r, w), yl);
        end
    end
    fn = fullfile(od, fn);  exportgraphics(f, fn, 'Resolution', 150); close(f); fprintf('figure: %s\n', fn);
end
