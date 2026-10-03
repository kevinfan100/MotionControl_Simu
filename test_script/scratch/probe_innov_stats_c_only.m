% PURPOSE (2026-09-29, layer 0 of the lambda_f / Q_cc design: "is there a signal in the innovations, and from which height?"):
%   offline only, reads f1p2_{plane,cell}_PI_sig010_ramp2.mat (c-only, plane init, lambda = 1, T_d = 2 s, 10 seeds).
%   Normalized innovations nu_i = innov_i / sqrt(S_i). Three statistics along time, per seed then seed mean +- sd:
%     row 1  windowed mean of nu (N = 100 steps = 0.062 s)   -- a bias in the innovation = the model is wrong (c walked)
%     row 2  windowed NIS (mean of nu^2 over N)                -- spread consistent with S?
%     row 3  one-sided CUSUM g_k = max(0, g_{k-1} + nu_k - 0.5) -- accumulated evidence of a positive shift (Gustafsson 1997 style)
%   Columns plane | cell; y2 solid, y1 dashed. Console: values at h_d = 10, 5, 3, 2, 1.5, 1.1 R and in the hold, first height where
%   the y2 windowed mean leaves +-3 sigma_w (sigma_w = sqrt(NIS_design / N), NIS_design read from the far field).
%   House style. Output reference/eq17_analysis/derivation/figures/innov_stats_c_only.png   EXPIRES: with the c-only line
function probe_innov_stats_c_only()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    rd = fullfile(root, 'test_results', 'temp_plaw_walls');  od = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    PL = {'plane', 'cell'};  N = 100;  KCU = 0.5;
    COL2 = [0 0.2 0.9]; COL1 = [0.85 0.45 0]; BAND2 = [0.45 0.55 0.95]; FS = 15; LFS = 11; AXLW = 2.0; CG = [0.5 0.5 0.5];
    f = figure('Position', [60 60 1250 1000], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax = gobjects(3, 2);
    for ip = 1:2
        r = load(fullfile(rd, sprintf('f1p2_%s_PI_sig010_ramp2.mat', PL{ip})));
        t = r.t;  seg = r.seg;  desc = t >= seg(1) & t < seg(2);  hold_ = t >= seg(3);  ns = size(r.innov2, 2);
        nu2 = r.innov2 ./ sqrt(r.S2);  nu1 = r.innov1 ./ sqrt(r.S1);  nu2(~isfinite(nu2)) = 0;  nu1(~isfinite(nu1)) = 0;
        k = ones(N, 1) / N;
        wm2 = filter(k, 1, nu2);  wm1 = filter(k, 1, nu1);                      % windowed means (causal)
        wn2 = filter(k, 1, nu2.^2);  wn1 = filter(k, 1, nu1.^2);                % windowed NIS
        g2 = zeros(size(nu2));  g1 = zeros(size(nu1));
        for j = 2:numel(t); g2(j, :) = max(0, g2(j-1, :) + nu2(j, :) - KCU);  g1(j, :) = max(0, g1(j-1, :) + nu1(j, :) - KCU); end
        far = desc & r.wd > 8;  nis2_far = mean(nu2(far, :).^2, 'all');  nis1_far = mean(nu1(far, :).^2, 'all');
        sw2 = sqrt(nis2_far / N);  sw1 = sqrt(nis1_far / N);
        M2 = mean(wm2, 2);  S2 = std(wm2, 0, 2);  M1 = mean(wm1, 2);  Wn2 = mean(wn2, 2);  Wn1 = mean(wn1, 2);  G2 = mean(g2, 2);  G1 = mean(g1, 2);
        id = find(desc);  cross = find(abs(M2(id)) > 3 * sw2 & r.wd(id) < 15, 1, 'first');  hx = NaN;  if ~isempty(cross); hx = r.wd(id(cross)); end
        fprintf('\n%s: NIS design far field y2 %.2f  y1 %.2f | sigma_w(y2) = %.3f | y2 windowed mean leaves 3 sigma_w first at h_d = %.2f R\n', PL{ip}, nis2_far, nis1_far, sw2, hx);
        fprintf('%6s | %8s %8s | %8s %8s | %8s %8s\n', 'h_d', 'wmean2', 'wmean1', 'wNIS2', 'wNIS1', 'CUSUM2', 'CUSUM1');
        for hq = [10 5 3 2 1.5 1.1]
            [~, j] = min(abs(r.wd(id) - hq));  q = id(j);
            fprintf('%6.2f | %+8.3f %+8.3f | %8.2f %8.2f | %8.1f %8.1f\n', r.wd(q), M2(q), M1(q), Wn2(q), Wn1(q), G2(q), G1(q));
        end
        q = find(hold_, 1, 'last');
        fprintf('%6s | %+8.3f %+8.3f | %8.2f %8.2f | %8.1f %8.1f\n', 'holdE', M2(q), M1(q), Wn2(q), Wn1(q), G2(q), G1(q));
        % ---- figure
        for rr = 1:3
            ax(rr, ip) = nexttile(2 * (rr - 1) + ip); hold on; box on;
            switch rr
                case 1
                    fill([t; flipud(t)], [M2 - S2; flipud(M2 + S2)], BAND2, 'FaceAlpha', 0.25, 'EdgeColor', 'none');
                    h2 = plot(t, M2, '-', 'Color', COL2, 'LineWidth', 1.8, 'DisplayName', 'y_2:  windowed mean of \nu_2 = innov_2 / sqrt(S_2)  (N = 100, mean \pm \sigma_{seed})');
                    h1 = plot(t, M1, '--', 'Color', COL1, 'LineWidth', 1.6, 'DisplayName', 'y_1:  windowed mean of \nu_1');
                    yline(3 * sw2, ':', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.4, 'HandleVisibility', 'off');  yline(-3 * sw2, ':', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.4, 'HandleVisibility', 'off');
                    hl = plot(NaN, NaN, ':', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.4, 'DisplayName', '\pm 3 \sigma_w  (white-noise band for y_2)');
                    yline(0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off');
                    ylim([-0.6 1.2]);  ylabel('windowed mean', 'FontSize', FS, 'FontWeight', 'bold');  L = [h2 h1 hl];
                case 2
                    h2 = plot(t, Wn2, '-', 'Color', COL2, 'LineWidth', 1.8, 'DisplayName', 'y_2:  windowed NIS_2');
                    h1 = plot(t, Wn1, '--', 'Color', COL1, 'LineWidth', 1.6, 'DisplayName', 'y_1:  windowed NIS_1');
                    yline(nis2_far, ':', 'Color', COL2, 'LineWidth', 1.2, 'HandleVisibility', 'off');  yline(1, ':', 'Color', COL1, 'LineWidth', 1.2, 'HandleVisibility', 'off');
                    ylim([0 3]);  ylabel('windowed NIS', 'FontSize', FS, 'FontWeight', 'bold');  L = [h2 h1];
                case 3
                    h2 = plot(t, G2, '-', 'Color', COL2, 'LineWidth', 1.8, 'DisplayName', 'y_2:  CUSUM  g_k = max(0, g_{k-1} + \nu_2 - 0.5)');
                    h1 = plot(t, G1, '--', 'Color', COL1, 'LineWidth', 1.6, 'DisplayName', 'y_1:  CUSUM');
                    ylim([0 60]);  ylabel('CUSUM', 'FontSize', FS, 'FontWeight', 'bold');  xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');  L = [h2 h1];
            end
            for s = seg; xline(s, '--', 'Color', CG, 'LineWidth', 1.0, 'HandleVisibility', 'off'); end
            xlim([0 t(end)]);
            h0 = plot(NaN, NaN, 'w.', 'DisplayName', sprintf('%s wall, plane init', upper(PL{ip})));
            legend([h0 L], 'Location', 'northoutside', 'Orientation', 'vertical', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
            set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
        end
    end
    for rr = 1:3; linkaxes(ax(rr, :), 'y'); end
    fn = fullfile(od, 'innov_stats_c_only.png');  exportgraphics(f, fn, 'Resolution', 150); close(f); fprintf('figure: %s\n', fn);
end
