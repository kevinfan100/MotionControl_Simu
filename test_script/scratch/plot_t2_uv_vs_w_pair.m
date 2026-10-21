% STATUS: SCRATCH -- 2026-10-21 (user: "fig_t2_uv_vs_w_tau_a 畫有對比到原先沒有控制 uv 其他照舊"; does the u, v control hurt the estimator?).
%   Columns = settings (e.g. tau_a); each column has two arms, u, v control ON (cell of tags) and OFF (cell of tags), same seeds.
%   Rows: u with control | u without | v with | v without, against the distance from the wall w = h R [um] (log, reversed: the approach
%   runs left to right), TRUE local coordinate relative to the start; every seed thin and light, seed 1 bold; shared axes.
%   Console per arm: fails; median / max |u|, |v| at w = 10, 5 um; estimator check over the run: median over seeds of rms(e_theta) [deg]
%   and of rms((a-hat_N - a_N)/a_N), so ON vs OFF shows whether the control changes the estimates.
%   -> test_results/temp_3d_wsonly/fig_t2_uv_vs_w_pair_<outtag>.png
function plot_t2_uv_vs_w_pair(ON, OFF, LB, outtag)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_3d_wsonly');  addpath(genpath(fullfile(root, 'model')));  pc = physical_constants();
    cB = {[0 0.25 0.85], [0 0.55 0.2]};  cL = {[0.6 0.7 0.95], [0.6 0.85 0.65]};  FS = 17;  LFS = 13;  MU = char(181);
    NM = {'u', 'v'};  ONOFF = {'with u, v control', 'without'};  WQ = [10 5];  nc = numel(ON);
    f = figure('Position', [40 40 720 * nc 1700], 'Color', 'w', 'Visible', 'off');
    tl = tiledlayout(4, nc, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ic = 1:nc
        GR = {ON{ic}, OFF{ic}};
        for ia = 1:2
            R = {};
            for it = 1:numel(GR{ia})
                S = load(fullfile(od, ['temp_t2_' GR{ia}{it} '.mat']));  R = [R, S.R.runs];  ty = S.R.opts.theta_y;
            end
            V = [cosd(ty) 0 sind(ty); 0 1 0; -sind(ty) 0 cosd(ty)];
            AT = NaN(numel(R), 2, numel(WQ));  nf = 0;  EQ = NaN(numel(R), 2);
            for q = 1:2
                ax = nexttile(tl, ((q - 1) * 2 + ia - 1) * nc + ic);  hold on;  box on;
                for j = 1:numel(R)
                    r = R{j};  k = 1:r.kend;
                    if r.contact || any(isnan(r.aN(k))); if q == 1; nf = nf + 1; end; continue; end
                    UV = (V(:, 1:2)' * (r.p(k, :) - r.p(1, :))')';  w = r.h(k) * pc.R;
                    if r.seed == 1
                        hb = plot(w, UV(:, q), '-', 'Color', cB{q}, 'LineWidth', 2.2, 'DisplayName', [NM{q} ', seed 1, ' ONOFF{ia}]);
                    else
                        plot(w, UV(:, q), '-', 'Color', cL{q}, 'LineWidth', 0.8, 'HandleVisibility', 'off');
                    end
                    for iq = 1:numel(WQ); i = find(w < WQ(iq), 1); if ~isempty(i); AT(j, q, iq) = UV(i, q); end; end
                    if q == 1
                        EQ(j, :) = [rms([r.thy(k) - ty; r.thx(k)]), rms((r.aN(k) - r.aN_true(k)) ./ r.aN_true(k))];
                    end
                end
                set(ax, 'XScale', 'log', 'XDir', 'reverse');  xlim([2 60]);  ylim([-12 12]);
                if ic == 1; ylabel([NM{q} '  [' MU 'm]'], 'FontSize', FS, 'FontWeight', 'bold'); end
                if q == 2 && ia == 2; xlabel(['w  [' MU 'm]  (distance from the wall)'], 'FontSize', FS, 'FontWeight', 'bold'); end
                if q == 1 && ia == 1; text(0.03, 0.9, LB{ic}, 'Units', 'normalized', 'FontSize', FS, 'FontWeight', 'bold'); end
                if exist('hb', 'var'); legend(hb, 'Location', 'northoutside', 'FontSize', LFS, 'FontWeight', 'bold'); clear hb; end
                set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2);
            end
            fprintf('%-28s fail %d | rms e_theta med %.2f deg | rms rel. e_aN med %.3f', strjoin(GR{ia}, '+'), nf, median(EQ(:, 1), 'omitnan'), median(EQ(:, 2), 'omitnan'));
            for iq = 1:numel(WQ)
                a = abs(squeeze(AT(:, :, iq)));
                fprintf(' | w=%d: |u| %.1f/%.1f |v| %.1f/%.1f', WQ(iq), median(a(:, 1), 'omitnan'), max(a(:, 1)), median(a(:, 2), 'omitnan'), max(a(:, 2)));
            end
            fprintf('\n');
        end
    end
    out = fullfile(od, sprintf('fig_t2_uv_vs_w_pair_%s.png', outtag));
    exportgraphics(f, out, 'Resolution', 100);  close(f);  fprintf('figure: %s\n', out);
end
