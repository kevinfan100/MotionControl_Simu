% STATUS: SCRATCH -- 2026-10-22 (user): angle measurement theta_m with the ESTIMATED a-hat_T - a-hat_N (current) vs the MEASURED
%   a_Tm - a_Nm in the denominator, on the 10-21 flow (tag flow_s1..3: 20 deg plane, 15 um, hold 4 s + dither, regulation, scan from 11 s).
%   Data: run_t2_3d_reg with opts.angle_den = 'est' / 'meas', tags den_{est,meas}_s{1,2,3} (temp_den_compare.mat).
%   src = 'den2' (10-22 second run, user): no dither, no guard, measured arm with R_theta (1 + (R4 + R5) / dA-hat^2).
%   EXPIRES: when the denominator choice is settled
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
if ~exist('src', 'var'); src = 'den'; end                 % 'den' (10-22 first run, dither, guard, R unchanged) | 'den2' (no dither, no guard, R_meas)
S = load(fullfile(root, 'test_results', 'temp_3d_wsonly', ['temp_' src '_compare.mat']));  out = S.out;
arms = {'est', 'meas'};  lab = {'denominator: estimated', 'denominator: measured'};
cs = [0 0.2 0.8; 0.3 0.55 1; 0.55 0.75 1];
fig = figure('Color', 'w', 'Position', [80 80 1300 700]);
tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
ax = gobjects(2, 2);
for a = 1:2
    for row = 1:2
        ax(row, a) = nexttile(tl, (row - 1) * 2 + a);  hold on;  box on;
        for s = 1:3
            r = out.(arms{a}){s};  k = 1:r.kend;
            if row == 1; yv = r.thy(k); yt = r.thy_true(k); else; yv = r.thx(k); yt = r.thx_true(k); end
            plot(r.t(k), yv, 'Color', cs(s, :), 'LineWidth', 1.2, 'DisplayName', sprintf('seed %d', s));
        end
        plot(r.t(k), yt, 'r', 'LineWidth', 2.5, 'DisplayName', 'true');
        xline(4, ':k', 'HandleVisibility', 'off');  xline(11, ':k', 'HandleVisibility', 'off');
        set(gca, 'FontSize', 14);
        if row == 1; ylim([-5 30]); ylabel('\theta_y [deg]'); title(lab{a}, 'FontWeight', 'normal'); else; ylim([-10 10]); ylabel('\theta_x [deg]'); xlabel('t [s]'); end
        if row == 1 && a == 1; legend('Location', 'northoutside', 'Orientation', 'horizontal'); end
    end
end
linkaxes(ax(1, :), 'xy');  linkaxes(ax(2, :), 'xy');
fo = fullfile(root, 'test_results', 'temp_3d_wsonly', ['fig_t2_angle_' src '.png']);
exportgraphics(fig, fo, 'Resolution', 150);
% console statistics: |error| at 1, 2, 4 s and over the scan, both angles
for a = 1:2
    for s = 1:3
        r = out.(arms{a}){s};  e = hypot(r.thy - r.thy_true, r.thx - r.thx_true);
        at = @(tt) e(find(r.t >= tt, 1));  sc = r.t >= 11 & (1:numel(r.t))' <= r.kend;
        fprintf('%-5s seed %d: |e_theta| 1 s %5.2f  2 s %5.2f  4 s %5.2f  hold-end..11 s max %5.2f  scan mean %5.2f max %5.2f deg\n', ...
                arms{a}, s, at(1), at(2), at(4), max(e(r.t >= 4 & r.t < 11)), mean(e(sc)), max(e(sc)));
    end
end
