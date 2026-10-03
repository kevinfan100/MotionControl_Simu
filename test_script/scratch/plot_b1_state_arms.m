% FORK OF test_script/scratch/plot_three_walls_arms.m (2026-09-14) | PURPOSE: verdict page for ctrl_const.b1_state
%   (0914_estb_bprime_core.tex) -- arms bseed0 / btseed0 / bseed0_b1 / bseed0_b1np on one plant, Meng left, canon right,
%   shared y per row. Rows: (1) b_hat seed mean per arm with the plant's b along the true height (red = the b the btseed0 law
%   read); (2) (a_hat - a)/a_nom seed mean +- sigma_seed; (3) paired difference against bseed0, mean +- SEM; (4) b1_hat seed
%   mean +- sigma_seed with +- sqrt(P66) (b1 arms). Console: the four pre-registered criteria (design 2026-09-14) and health.
%   Usage: plot_b1_state_arms('plane') | plot_b1_state_arms('ramp')   Output b1_state_<wall>.png
%   | EXPIRES: when b1_state is closed | 產線改動不會自動跟上
function plot_b1_state_arms(wall)
    if nargin < 1 || isempty(wall); wall = 'plane'; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    ARMS = {'bseed0', 'btseed0', 'bseed0_b1', 'bseed0_b1np'};
    COL  = struct('bseed0', [0 0.2 0.9], 'btseed0', [0.8 0 0], 'bseed0_b1', [0.85 0.33 0.10], 'bseed0_b1np', [0 0.55 0.2]);
    LAB  = struct('bseed0', 'constant b (bseed0)', 'btseed0', 'b_{true}(w) fed (btseed0)', 'bseed0_b1', 'b + b_1 state, seed coupling', 'bseed0_b1np', 'b + b_1 state, no seed coupling');
    TR = {'meng', 'canon'};  tag = '_plane-ramp_bseed0-btseed0-bseed0_b1-bseed0_b1np';
    D = cell(1, 2);
    for c = 1:2; D{c} = load(fullfile(od, sprintf('three_walls_%s%s.mat', TR{c}, tag))); end
    FS = 15; LFS = 11; AXLW = 1.8; NR = 4;
    f = figure('Units', 'inches', 'Position', [0 0 15 4.0*NR], 'Color', 'w', 'Visible', 'off');
    tiledlayout(NR, 2, 'TileSpacing', 'compact', 'Padding', 'compact');  ax = gobjects(NR, 2);
    for c = 1:2
        S = D{c};  ph = S.phases;  base = S.([wall '_bseed0']);  t = base.t(:);  T_END = ceil(t(end));  nS = size(base.E, 2);
        % ---- row 1: b_hat ----
        ax(1, c) = nexttile(c); hold on; H = []; L = {};
        for ia = [1 3 4]
            d = S.([wall '_' ARMS{ia}]);  H(end+1) = plot(t, mean(d.B, 2), '-', 'Color', COL.(ARMS{ia}), 'LineWidth', 1.6); L{end+1} = LAB.(ARMS{ia}); %#ok<AGROW>
        end
        dt = S.([wall '_btseed0']);  H(end+1) = plot(t, mean(dt.B, 2), '-', 'Color', COL.btseed0, 'LineWidth', 2.0); L{end+1} = 'plant b_{true}(w) at the true height';
        local_style(ax(1, c), c == 1, '\^b   seed mean', false, T_END, ph, FS, AXLW);
        legend(H, L, 'Location', 'northoutside', 'Orientation', 'horizontal', 'NumColumns', 2, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        % ---- row 2: gain error ----
        ax(2, c) = nexttile(2 + c); hold on; H = []; L = {};
        for ia = 1:4
            d = S.([wall '_' ARMS{ia}]);  m = mean(d.E, 2);  s = std(d.E, 0, 2);  col = COL.(ARMS{ia});
            fill([t; flipud(t)], [m + s; flipud(m - s)], col, 'FaceAlpha', 0.12, 'EdgeColor', 'none', 'HandleVisibility', 'off');
            H(end+1) = plot(t, m, '-', 'Color', col, 'LineWidth', 1.6); L{end+1} = LAB.(ARMS{ia}); %#ok<AGROW>
        end
        yline(0, '-', 'Color', [0.5 0.5 0.5], 'HandleVisibility', 'off');
        local_style(ax(2, c), c == 1, '(\^a_z - a_z)/a_{nom}  mean \pm \sigma_{seed}', false, T_END, ph, FS, AXLW);
        legend(H, L, 'Location', 'northoutside', 'Orientation', 'horizontal', 'NumColumns', 2, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        % ---- row 3: paired difference ----
        ax(3, c) = nexttile(4 + c); hold on; H = []; L = {};
        for ia = 2:4
            d = S.([wall '_' ARMS{ia}]);  dE = d.E - base.E;  m = mean(dE, 2);  se = std(dE, 0, 2) / sqrt(nS);  col = COL.(ARMS{ia});
            fill([t; flipud(t)], [m + se; flipud(m - se)], col, 'FaceAlpha', 0.18, 'EdgeColor', 'none', 'HandleVisibility', 'off');
            H(end+1) = plot(t, m, '-', 'Color', col, 'LineWidth', 1.6); L{end+1} = [LAB.(ARMS{ia}) ' - bseed0']; %#ok<AGROW>
        end
        yline(0, '-', 'Color', [0.5 0.5 0.5], 'HandleVisibility', 'off');
        local_style(ax(3, c), c == 1, 'paired \DeltaE  mean \pm SEM', false, T_END, ph, FS, AXLW);
        legend(H, L, 'Location', 'northoutside', 'Orientation', 'horizontal', 'NumColumns', 2, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        % ---- row 4: b1 ----
        ax(4, c) = nexttile(6 + c); hold on; H = []; L = {};
        for ia = 3:4
            d = S.([wall '_' ARMS{ia}]);  m = mean(d.B1, 2);  s = std(d.B1, 0, 2);  p6 = mean(d.sP6, 2);  col = COL.(ARMS{ia});
            fill([t; flipud(t)], [m + s; flipud(m - s)], col, 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'HandleVisibility', 'off');
            H(end+1) = plot(t, m, '-', 'Color', col, 'LineWidth', 1.6); L{end+1} = ['\^b_1  ' LAB.(ARMS{ia})]; %#ok<AGROW>
            plot(t, m + p6, ':', 'Color', col, 'LineWidth', 1.0, 'HandleVisibility', 'off');  plot(t, m - p6, ':', 'Color', col, 'LineWidth', 1.0, 'HandleVisibility', 'off');
        end
        yline(0, '-', 'Color', [0.5 0.5 0.5], 'HandleVisibility', 'off');
        local_style(ax(4, c), c == 1, '\^b_1 [1/R]  mean \pm \sigma_{seed}  (dotted \pm\surdP_{66})', true, T_END, ph, FS, AXLW);
        legend(H, L, 'Location', 'northoutside', 'Orientation', 'horizontal', 'NumColumns', 2, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        % ---- console: health + pre-registered criteria ----
        md = t > ph(1) & t <= ph(2);  mnw = md & base.hd(:) < 2;  mmo = t > ph(1) & t <= ph(3);  mh = t > ph(3);  mos = t > ph(2) & t <= ph(3);
        fprintf('\n===== %s plant, %s (%d seeds) =====\n', wall, TR{c}, nS);
        for ia = 1:4
            d = S.([wall '_' ARMS{ia}]);
            fprintf('[%-11s] HEALTH NaN %d | min w %.3f | b_hat [%.3f %.3f], samples at b<=0.6001 %d / b>=1.4999 %d | sqrtP55 step1 %.3f step2 %.3f end %.4f | sqrtP66 step2 %.4f end %.4f | b1 end %+.4f\n', ...
                ARMS{ia}, nnz(~isfinite(d.E)), min(d.HB(:)), min(d.B(:)), max(d.B(:)), nnz(d.B <= 0.6001), nnz(d.B >= 1.4999), ...
                mean(d.sP5(1, :)), mean(d.sP5(2, :)), mean(d.sP5(end, :)), mean(d.sP6(2, :)), mean(d.sP6(end, :)), mean(d.B1(end, :)));
            wE = max(abs(d.E(mnw, :)), [], 1);  wB = max(abs(base.E(mnw, :)), [], 1);  dW = wE - wB;
            sd_mo = mean(std(d.E(mmo, :), 0, 2));  sd_mo0 = mean(std(base.E(mmo, :), 0, 2));
            fprintf('   (1) near-wall first descent worst |E| %.4f (base %.4f) paired %+.4f +- %.4f SEM -> %s\n', mean(wE), mean(wB), mean(dW), std(dW)/sqrt(nS), local_pf(mean(dW) <= std(dW)/sqrt(nS)));
            if strcmp(TR{c}, 'canon')
                mb = abs(t - 2.5) <= 0.05 | abs(t - 3.5) <= 0.05;
                fprintf('   (2) 2nd/3rd bottom mean |E| %.4f (base %.4f) -> %s\n', mean(abs(d.E(mb, :)), 'all'), mean(abs(base.E(mb, :)), 'all'), local_pf(mean(abs(d.E(mb, :)), 'all') <= mean(abs(base.E(mb, :)), 'all')));
            end
            if strcmp(wall, 'ramp')
                mnear = mmo & base.hd(:) < 1.5;
                fprintf('   (3) b_hat near the wall %.3f (target >= 1.05, plant ~1.16) | osc/dwell mean E %+.4f (base %+.4f) | hold mean E %+.5f (base %+.5f) -> %s\n', ...
                    mean(d.B(mnear, :), 'all'), mean(d.E(mos, :), 'all'), mean(base.E(mos, :), 'all'), mean(d.E(mh, :), 'all'), mean(base.E(mh, :), 'all'), ...
                    local_pf(mean(d.B(mnear, :), 'all') >= 1.05 && abs(mean(d.E(mos, :), 'all')) < abs(mean(base.E(mos, :), 'all')) && abs(mean(d.E(mh, :), 'all')) < abs(mean(base.E(mh, :), 'all'))));
            end
            fprintf('   (4) motion sigma_seed %.4f (base %.4f, ratio %.2f) -> %s | desc mean %+.4f | osc mean %+.4f | hold mean %+.5f SEM %.5f\n', ...
                sd_mo, sd_mo0, sd_mo / sd_mo0, local_pf(sd_mo <= 1.5 * sd_mo0), mean(d.E(md, :), 'all'), mean(d.E(mos, :), 'all'), mean(d.E(mh, :), 'all'), std(mean(d.E(mh, :), 1)) / sqrt(nS));
        end
    end
    for r = 1:NR; linkaxes(ax(r, :), 'y'); end
    png = fullfile(od, sprintf('b1_state_%s.png', wall));
    exportgraphics(f, png, 'Resolution', 150);  close(f);  fprintf('saved %s\n', png);
end

function local_style(a, is_left, ylab, is_bottom, T_END, ph, FS, AXLW)
    axes(a); %#ok<LAXES>
    xline(ph(1), ':', 'Color', [0.4 0.4 0.4], 'HandleVisibility', 'off'); xline(ph(2), ':', 'Color', [0.4 0.4 0.4], 'HandleVisibility', 'off'); xline(ph(3), '--', 'Color', [0.4 0.4 0.4], 'HandleVisibility', 'off');
    if is_left; ylabel(ylab, 'FontSize', FS, 'FontWeight', 'bold'); end
    if is_bottom; xlabel('time [s]', 'FontSize', FS, 'FontWeight', 'bold'); end
    set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off; xlim([0 T_END]);
end

function s = local_pf(ok)
    if ok; s = 'PASS'; else; s = 'FAIL'; end
end
