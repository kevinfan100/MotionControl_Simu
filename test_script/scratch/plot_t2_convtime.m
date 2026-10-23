% STATUS: SCRATCH -- 2026-10-22 (user: a simple figure comparing the convergence times). Fixed points held 20 s on the 20 deg plane,
%   runs tm<gap>_A_s<seed> (wait only) and tm<gap>_B_s<seed> (with the dither 3 um, 8 Hz, 2 s ramp; 50 and 15 um only). Time after which
%   |theta-hat_y - theta_y| and |theta-hat_x - theta_x| both stay within 5 deg, per seed (dots) and median (line); runs that never get there
%   within 20 s are drawn as open markers at the top. Second axis: a_N / a_T at the gap.
%   -> test_results/temp_3d_wsonly/fig_t2_convtime.png | run: plot_t2_convtime([50 15 10 5], 1:3)
function plot_t2_convtime(GAP, SD)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_3d_wsonly');
    cE = [0 0.25 0.85];  cO = [0.9 0.5 0];  FS = 18;  LFS = 14;  MU = char(181);  AB = char(257);
    ARM = {'A', 'B'};  LB = {'wait only', 'motion'};  CO = {cE, cO};  MK = {'o', 's'};  TOP = 22;
    f = figure('Position', [40 40 1200 760], 'Color', 'w', 'Visible', 'off');  hold on;  box on;
    hh = gobjects(0);  ratio = NaN(size(GAP));
    for ia = 1:2
        med = NaN(size(GAP));
        for ig = 1:numel(GAP)
            T = NaN(size(SD));
            for is = 1:numel(SD)
                fn = fullfile(od, sprintf('temp_t2_tm%d_%s_s%d.mat', GAP(ig), ARM{ia}, SD(is)));
                if ~exist(fn, 'file'); continue; end
                S = load(fn);  r = S.R.runs{1};  k = 1:r.kend;  t = r.t(k);
                e = max(abs([r.thy(k) - S.R.opts.theta_y, r.thx(k)]), [], 2);  tail = flipud(cummax(flipud(e)));
                i5 = find(tail < 5, 1);  if isempty(i5); T(is) = Inf; else; T(is) = t(i5); end
                ratio(ig) = mean(r.aN_true(k)) / mean(r.aT_true(k));
            end
            if all(isnan(T)); continue; end
            fin = isfinite(T);  xo = GAP(ig) * (1 + 0.04 * (ia - 1.5));
            plot(xo + 0 * T(fin), T(fin), MK{ia}, 'Color', CO{ia}, 'MarkerFaceColor', CO{ia}, 'MarkerSize', 10, 'HandleVisibility', 'off');
            plot(xo + 0 * T(~fin), TOP + 0 * T(~fin), MK{ia}, 'Color', CO{ia}, 'MarkerSize', 12, 'LineWidth', 2, 'HandleVisibility', 'off');
            Tm = T;  Tm(~fin) = TOP;  med(ig) = median(Tm);
            fprintf('%-13s %2d um (a_N/a_T %.2f): %s s\n', LB{ia}, GAP(ig), ratio(ig), mat2str(T, 3));
        end
        ok = ~isnan(med);
        hh(end + 1) = plot(GAP(ok), med(ok), '-', 'Color', CO{ia}, 'LineWidth', 2.5, 'Marker', MK{ia}, 'MarkerFaceColor', CO{ia}, 'MarkerSize', 10, 'DisplayName', [LB{ia} ' (median)']);
    end
    yline(20, ':', 'end of the hold (20 s)', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.5, 'FontSize', LFS, 'FontWeight', 'bold', 'LabelHorizontalAlignment', 'left');
    text(GAP(1), TOP + 1.2, 'not within 5\circ', 'HorizontalAlignment', 'center', 'FontSize', LFS, 'FontWeight', 'bold');
    set(gca, 'XScale', 'log', 'XDir', 'reverse');  xticks(sort(GAP));  xlim([min(GAP) * 0.8, max(GAP) * 1.25]);  ylim([0 TOP + 3]);
    [gs, io] = sort(GAP);  xticklabels(arrayfun(@(i) sprintf('%d (%.2f)', gs(i), ratio(io(i))), 1:numel(gs), 'UniformOutput', false));
    xlabel(['gap held  [' MU 'm]   (' AB '_N/' AB '_T)'], 'FontSize', FS, 'FontWeight', 'bold');  ylabel('time to stay within 5\circ  [s]', 'FontSize', FS, 'FontWeight', 'bold');
    legend(hh, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
    set(gca, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 2);
    out = fullfile(od, 'fig_t2_convtime.png');
    exportgraphics(f, out, 'Resolution', 110);  close(f);  fprintf('figure: %s\n', out);
end
