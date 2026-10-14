% STATUS: SCRATCH -- 2026-10-13, spread and wander of w_s-hat for the S8 estimator (numerator 1) from the wrong start w_s-hat[0] = -1
%   (user 10-13: "reduce the w_s spread, the a' error should not oscillate so much"). Arms (10 seeds each): S8 (prior 1.0), A (prior 0.3),
%   G (information gate, run_1009_wsonly opt.gate). Metrics on the descent: seed sd of w_s-hat at h_d = 6 / 3 / 2 / 1.10 R and at the end,
%   path length sum |dw_s-hat| (mean over seeds: how much it wanders), |a'-hat - a'| per window (10-seed mean).
%   Figure: rows w_s-hat (seed mean +- sd band) and |a'-hat - a'| (10-seed mean, log), columns plane | cell.
function probe_1009_ws_spread()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(genpath(fullfile(root, 'model')));  addpath(here);
    od = fullfile(root, 'test_results', 'temp_1009_ws');
    WL = {'plane', 'cell'};  PC = {[], @cell_cperp_delta};  AR = {'', '_sw030', '_gate'};  NM = {'S8 (prior 1.0)', 'A: prior 0.3', 'G: gate'};
    CL = {[0 0.25 0.85], [0.3 0.75 0.95], [0.85 0.1 0.45]};  FS = 18;  WB = ['w' char(772)];  HAT = char(770);  AB = char(257);
    f = figure('Position', [40 40 2000 1100], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for iw = 1:2
        a1 = nexttile(tl, iw);  hold on; box on;  a2 = nexttile(tl, 2 + iw);  hold on; box on;
        fprintf('\n[%s] start -1 | seed sd of w_s-hat at 6 / 3 / 2 / 1.1 R / end | path length | |a''err| >6R, 6-2R, 2-1.1R | end mean\n', WL{iw});
        for ia = 1:3
            S = load(fullfile(od, sprintf('wsonly_%s_m1%s.mat', WL{iw}, AR{ia})));  t = S.t;  wd = S.wd;  wd(1) = wd(2);  dsc = t <= 1;
            W = S.w_hat;  mu = mean(W, 2);  sd = std(W, 0, 2);
            E = abs(1 ./ (wd - W).^2 - local_ap(max(S.h_true, 1.001), PC{iw}));  m = mean(E, 2);
            sds = arrayfun(@(hv) sd(find(wd <= hv & t > 0.001, 1)), [6 3 2 1.10]);
            pl = mean(sum(abs(diff(W(dsc, :))), 1));
            fprintf('  %-16s | %.3f %.3f %.3f %.3f %.3f | %.2f | %.5f %.4f %.4f | %+.3f\n', NM{ia}, sds, sd(end), pl, ...
                    mean(m(dsc & wd > 6)), mean(m(dsc & wd <= 6 & wd > 2)), mean(m(dsc & wd <= 2)), mu(end));
            fill(a1, [t; flipud(t)], [mu + sd; flipud(mu - sd)], CL{ia}, 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'HandleVisibility', 'off');
            plot(a1, t, mu, '-', 'Color', CL{ia}, 'LineWidth', 2.4, 'DisplayName', NM{ia});
            mm = m;  mm(~dsc) = NaN;  plot(a2, t, mm, '-', 'Color', CL{ia}, 'LineWidth', 2.4, 'DisplayName', NM{ia});
        end
        yline(a1, 0, '-', 'Color', [0.85 0 0], 'LineWidth', 2, 'HandleVisibility', 'off');
        ylim(a1, [-2 2]);  xlim(a1, [0 1.2]);  text(a1, 0.03, 0.9, [WL{iw} ':  ' WB HAT '_s  (seed mean ' char(177) ' sd)'], 'Units', 'normalized', 'FontSize', FS, 'FontWeight', 'bold');
        legend(a1, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', 14, 'FontWeight', 'bold');
        set(a2, 'YScale', 'log');  ylim(a2, [1e-5 1]);  xlim(a2, [0 1]);
        text(a2, 0.03, 0.9, [WL{iw} ':  |' AB HAT ''' - ' AB '''|  (10-seed mean)'], 'Units', 'normalized', 'FontSize', FS, 'FontWeight', 'bold');
        xlabel(a2, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        for a = [a1 a2]; set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.8);  grid(a, 'off'); end
    end
    out = fullfile(od, 'fig_1009_ws_spread.png');  exportgraphics(f, out, 'Resolution', 130);  fprintf('figure: %s\n', out);
end

function ap = local_ap(h, pcp)
    d = 1e-4;  ap = (local_a(h + d, pcp) - local_a(h - d, pcp)) / (2 * d);
end

function a = local_a(h, pcp)
    h = max(h, 1.001);
    if isempty(pcp)
        a = zeros(size(h));  for i = 1:numel(h); [~, c] = calc_correction_functions(h(i)); a(i) = 1 / c; end
    else
        a = 1 ./ pcp(h);
    end
end
