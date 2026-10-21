% STATUS: SCRATCH -- 2026-10-21 (user: "補 w 為橫軸的 uv 圖"; is the approach smooth along w?). Columns = arms (each a cell of tags whose
%   runs are merged, e.g. two halves of the seeds), shared axes. Rows: u, v of the probe relative to its start in the TRUE local coordinate
%   [um] against the distance from the wall w = h R [um] (log axis, reversed so the approach runs left to right); every seed thin and
%   light, seed 1 bold. Console: per arm, median / max |u|, |v| at w = 10, 5, 3 um.
%   -> test_results/temp_3d_wsonly/fig_t2_uv_vs_w_<outtag>.png
%   | run: plot_t2_uv_vs_w({{'tu0p003_a','tu0p003_b'},{'ta4_a','ta4_b'},{'ta8_a','ta8_b'}}, {'\tau_a = 2 s','\tau_a = 4 s','\tau_a = 8 s'}, 'tau_a')
function plot_t2_uv_vs_w(ARMS, LB, outtag)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_3d_wsonly');  addpath(genpath(fullfile(root, 'model')));  pc = physical_constants();
    cE = [0 0.25 0.85];  cGr = [0 0.55 0.2];  cL = {[0.6 0.7 0.95], [0.6 0.85 0.65]};  cB = {cE, cGr};  FS = 18;  LFS = 14;  MU = char(181);
    NM = {'u', 'v'};  WQ = [10 5 3];  nc = numel(ARMS);
    f = figure('Position', [40 40 760 * nc 1000], 'Color', 'w', 'Visible', 'off');
    tl = tiledlayout(2, nc, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ic = 1:nc
        R = {};
        for it = 1:numel(ARMS{ic})
            S = load(fullfile(od, ['temp_t2_' ARMS{ic}{it} '.mat']));  R = [R, S.R.runs];  ty = S.R.opts.theta_y;
        end
        V = [cosd(ty) 0 sind(ty); 0 1 0; -sind(ty) 0 cosd(ty)];
        AT = NaN(numel(R), 2, numel(WQ));
        for q = 1:2
            ax = nexttile(tl, (q - 1) * nc + ic);  hold on;  box on;
            for j = 1:numel(R)
                r = R{j};  k = 1:r.kend;
                if r.contact || any(isnan(r.aN(k))); continue; end
                UV = (V(:, 1:2)' * (r.p(k, :) - r.p(1, :))')';  w = r.h(k) * pc.R;
                if r.seed == 1
                    hb = plot(w, UV(:, q), '-', 'Color', cB{q}, 'LineWidth', 2.2, 'DisplayName', [NM{q} ', seed 1']);
                else
                    plot(w, UV(:, q), '-', 'Color', cL{q}, 'LineWidth', 0.8, 'HandleVisibility', 'off');
                end
                for iq = 1:numel(WQ); i = find(w < WQ(iq), 1); if ~isempty(i); AT(j, q, iq) = UV(i, q); end; end
            end
            set(ax, 'XScale', 'log', 'XDir', 'reverse');  xlim([2 60]);  ylim([-10 10]);
            if ic == 1; ylabel([NM{q} '  [' MU 'm]'], 'FontSize', FS, 'FontWeight', 'bold'); end
            if q == 2; xlabel(['w  [' MU 'm]  (distance from the wall)'], 'FontSize', FS, 'FontWeight', 'bold'); end
            if q == 1; text(0.03, 0.9, LB{ic}, 'Units', 'normalized', 'FontSize', FS, 'FontWeight', 'bold'); end
            if exist('hb', 'var'); legend(hb, 'Location', 'northoutside', 'FontSize', LFS, 'FontWeight', 'bold'); clear hb; end
            set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2);
        end
        for iq = 1:numel(WQ)
            a = abs(squeeze(AT(:, :, iq)));
            fprintf('%-14s w=%2d um: |u| med %.2f max %.2f | |v| med %.2f max %.2f  (n=%d)\n', strjoin(ARMS{ic}, '+'), WQ(iq), ...
                    median(a(:, 1), 'omitnan'), max(a(:, 1)), median(a(:, 2), 'omitnan'), max(a(:, 2)), sum(~isnan(a(:, 1))));
        end
    end
    out = fullfile(od, sprintf('fig_t2_uv_vs_w_%s.png', outtag));
    exportgraphics(f, out, 'Resolution', 110);  close(f);  fprintf('figure: %s\n', out);
end
