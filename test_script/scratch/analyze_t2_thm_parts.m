% STATUS: SCRATCH -- 2026-10-22 (user: look only at the angle measurement and its two components, start from wait only).
%   Angle row of theta_y, undivided form (run_t2_3d_reg): y_7 = [vA_m]_uw (single sample, readout in the theta-hat local frame).
%   Two components:  y_7 = s + n,   s = E[y_7] = [V-hat' A-bar V-hat]_uw (from the truth: gain difference x rotation error),
%                                   n = sample fluctuation of the product delta u * delta w (what is left).
%   Prints per gap: size of s, sd of n, per-sample ratio |s| / sd(n), the same in angle units (theta_m - theta-hat = y / H),
%   and the 1 s window mean of y_7 against s. Figure: columns = runs, row 1 = samples of y_7 with s (red), row 2 = 1 s mean of y_7 with s.
%   -> test_results/temp_3d_wsonly/fig_t2_thm_parts_<outtag>.png | run: analyze_t2_thm_parts({'tmg50_s1','tmg5_s1'}, {'50 um wait only','5 um wait only'}, 'wait')
function analyze_t2_thm_parts(TAG, LBL, outtag)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_3d_wsonly');
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  cL = [0.65 0.75 0.95];  FS = 17;  LFS = 13;  AB = char(257);
    nc = numel(TAG);
    f = figure('Position', [40 40 820 * nc 950], 'Color', 'w', 'Visible', 'off');
    tl = tiledlayout(2, nc, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ic = 1:nc
        S = load(fullfile(od, ['temp_t2_' TAG{ic} '.mat']));  r = S.R.runs{1};  k = (1:r.kend)';  t = r.t(k);  Ts = t(2) - t(1);
        y = r.vAm(k, 7);                                                   % [vA_m](1,3) = uw
        ty = S.R.opts.theta_y;  th = r.thr(k, :);                          % theta-hat used in the readout [deg] (x, y)
        s = NaN(size(k));
        for i = 1:numel(k)
            Vt = local_Ry(ty);  Vh = local_Ry(th(i, 2)) * local_Rx(th(i, 1));
            M = Vh' * Vt * diag([r.aT_true(k(i)) r.aT_true(k(i)) r.aN_true(k(i))]) * Vt' * Vh;  s(i) = M(1, 3);
        end
        n = y - s;  dA = r.aT_true(k) - r.aN_true(k);
        w = round(1 / Ts);  ym = movmean(y, w);
        ac = autocorr_time(n);
        m = t >= 1;
        fprintf('%-18s: a_N/a_T %.2f | |s| median %.2e (theta error %.1f deg) | sd(n) %.3f | per-sample |s|/sd(n) %.4f\n', LBL{ic}, ...
                median(r.aN_true(k) ./ r.aT_true(k)), median(abs(s(m))), median(abs(th(m, 2) - ty)), std(n(m)), median(abs(s(m))) / std(n(m)));
        fprintf('%-18s  in angle units: sd(n)/(a_T - a_N) = %.0f deg per sample | correlation length of n %.1f steps | 1 s mean: sd %.3f vs |s| %.3f\n', '', ...
                std(n(m)) / median(dA(m)) * 180 / pi, ac, std(ym(m) - movmean(s(m), w)), median(abs(s(m))));
        % ---- row 1: samples ----
        ax = nexttile(tl, ic);  hold on;  box on;
        plot(t(1:5:end), y(1:5:end), '.', 'Color', cL, 'MarkerSize', 3, 'DisplayName', ['[^v' char(256) '_m]_{uw}  (samples)']);
        plot(t, s, '-', 'Color', cT, 'LineWidth', 2.5, 'DisplayName', 'signal  s');
        yl = 4 * std(n(m));  ylim([-yl yl]);  xlim([0 t(end)]);
        text(0.03, 0.92, LBL{ic}, 'Units', 'normalized', 'FontSize', FS - 1, 'FontWeight', 'bold');
        if ic == 1; ylabel(['[' AB ']'], 'FontSize', FS, 'FontWeight', 'bold'); end
        legend('Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        set(ax, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 2);
        % ---- row 2: 1 s mean ----
        ax = nexttile(tl, nc + ic);  hold on;  box on;
        plot(t, ym, '-', 'Color', cE, 'LineWidth', 1.6, 'DisplayName', ['1 s mean of  [^v' char(256) '_m]_{uw}']);
        plot(t, s, '-', 'Color', cT, 'LineWidth', 2.5, 'DisplayName', 'signal  s');
        yl = max(4 * std(ym(m) - s(m)), 1.3 * max(abs(s(m))));  ylim([-yl yl]);  xlim([0 t(end)]);
        if ic == 1; ylabel(['[' AB ']'], 'FontSize', FS, 'FontWeight', 'bold'); end
        xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        legend('Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        set(ax, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 2);
    end
    out = fullfile(od, ['fig_t2_thm_parts_' outtag '.png']);  exportgraphics(f, out, 'Resolution', 100);  close(f);  fprintf('figure: %s\n', out);
end

function L = autocorr_time(n)
    % steps until the autocorrelation of n first drops below 1/e
    n = n(~isnan(n)) - mean(n(~isnan(n)));  c = xcorr(n, 200, 'coeff');  c = c(201:end);
    L = find(c < exp(-1), 1) - 1;
end

function R = local_Ry(a)
    R = [cosd(a) 0 sind(a); 0 1 0; -sind(a) 0 cosd(a)];
end

function R = local_Rx(a)
    R = [1 0 0; 0 cosd(a) -sind(a); 0 sind(a) cosd(a)];
end
