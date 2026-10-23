% STATUS: SCRATCH -- 2026-10-22 (user: the angle measurement is theta_ym = theta-hat_y + [vA_m]_uw / (a-hat_T - a-hat_N); look at its two
%   pieces separately, wait only first). Runs <TAG> (default tmg50_s1 | tmg5_s1, 20 deg plane, seed 1). Columns = runs.
%   Row 1 numerator [vA_m]_uw: 1 s mean of the samples (blue) vs the value it should average to, [V-hat' A-bar V-hat]_uw (red).
%   Row 2 denominator: a-hat_T - a-hat_N of the estimator (blue) vs the true a_T - a_N (red).
%   Row 3 result theta_ym: 1 s mean of the logged theta_ym (blue) vs theta_y = 20 deg (red), theta-hat_y (grey).
%   -> test_results/temp_3d_wsonly/fig_t2_thm_numden_<outtag>.png | run: plot_t2_thm_numden({'tmg50_s1','tmg5_s1'}, [50 5], 'wait')
function plot_t2_thm_numden(TAG, GAP, outtag)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_3d_wsonly');
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  cW = [0.5 0.5 0.5];  FS = 17;  LFS = 13;
    MU = char(181);  AB = char(257);  TH = char(952);  HT = char(770);  DG = char(176);  AM = [char(7515) char(256) '_m'];
    nc = numel(TAG);
    f = figure('Position', [40 40 820 * nc 1300], 'Color', 'w', 'Visible', 'off');
    tl = tiledlayout(3, nc, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ic = 1:nc
        S = load(fullfile(od, ['temp_t2_' TAG{ic} '.mat']));  r = S.R.runs{1};  k = (1:r.kend)';  t = r.t(k);  w = round(1 / (t(2) - t(1)));
        ty = S.R.opts.theta_y;  th = r.thr(k, :);  y = r.vAm(k, 7);
        s = NaN(size(k));
        for i = 1:numel(k)
            Vt = local_Ry(ty);  Vh = local_Ry(th(i, 2)) * local_Rx(th(i, 1));
            M = Vh' * Vt * diag([r.aT_true(k(i)) r.aT_true(k(i)) r.aN_true(k(i))]) * Vt' * Vh;  s(i) = M(1, 3);
        end
        dAh = r.aT(k) - r.aN(k);  dAt = r.aT_true(k) - r.aN_true(k);
        rows = {{movmean(y, w), s, ['1 s mean of  [' AM ']_{uw}'], ['should average to  [' AM ']_{uw}']}, ...
                {dAh, dAt, [AB char(770) '_T - ' AB char(770) '_N  (estimator)'], [AB '_T - ' AB '_N  (true)']}, ...
                {movmean(r.thm(k, 2), w), ty + 0 * t, ['1 s mean of  ' TH '_{ym}'], [TH '_y  (true)']}};
        YL = {[], [-0.05 0.35], [-200 200]};  ylab = {['[' AB ']'], ['[' AB ']'], ['[' DG ']']};
        for q = 1:3
            ax = nexttile(tl, (q - 1) * nc + ic);  hold on;  box on;
            plot(t, rows{q}{1}, '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', rows{q}{3});
            plot(t, rows{q}{2}, '-', 'Color', cT, 'LineWidth', 2.5, 'DisplayName', rows{q}{4});
            if q == 3; plot(t, r.thy(k), '-', 'Color', cW, 'LineWidth', 1.5, 'DisplayName', [TH HT '_y']); end
            if isempty(YL{q}); m = t >= 1; yl = 1.2 * max(abs(rows{q}{1}(m))); ylim([-yl yl]); else; ylim(YL{q}); end
            xlim([0 t(end)]);
            if q == 1; text(0.97, 0.9, sprintf('%d %sm', GAP(ic), MU), 'Units', 'normalized', 'HorizontalAlignment', 'right', 'FontSize', FS, 'FontWeight', 'bold'); end
            if ic == 1; ylabel(ylab{q}, 'FontSize', FS, 'FontWeight', 'bold'); end
            if q == 3; xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold'); end
            legend('Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
            set(ax, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 2);
        end
        m = t >= 1;
        fprintf('%2d um: numerator should-avg median %.4f, 1 s mean sd %.4f | denominator est %.3f -> %.3f (true %.3f) | theta_ym 1 s mean sd %.0f deg\n', ...
                GAP(ic), median(abs(s(m))), std(movmean(y(m), w) - movmean(s(m), w)), dAh(find(m, 1)), dAh(end), mean(dAt), std(movmean(r.thm(m, 2), w)));
    end
    out = fullfile(od, ['fig_t2_thm_numden_' outtag '.png']);  exportgraphics(f, out, 'Resolution', 100);  close(f);  fprintf('figure: %s\n', out);
end

function R = local_Ry(a)
    R = [cosd(a) 0 sind(a); 0 1 0; -sind(a) 0 cosd(a)];
end

function R = local_Rx(a)
    R = [1 0 0; 0 cosd(a) -sind(a); 0 sind(a) cosd(a)];
end
