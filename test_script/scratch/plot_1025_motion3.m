% STATUS: SCRATCH -- 2026-10-25 (user): computed angle vs estimated angle under three motions, 20 deg wall, start 15 um, seeds 1-3, 5 s.
%   Motions: fixed point | u v motion (10-21 dither: circle in the estimated tangent plane, 3 um, 8 Hz, 2 s ramp, height held) |
%   w motion (Meng dissertation Sec. 5.5: ramp along -w-hat, 5 um/s for 11.5 um, then hold at 3.5 um).
%   Estimated angle: theta-hat[0] = 0 (user 10-25); computed angle: A-hat[0] = I. Slope q = 0 at the fixed point, P_fix with motion.
%   One figure per form; columns = the three motions; rows theta_y error, theta_x error (line-angle error for the computed form), gap h.
%   Console: mean |error| over 3-5 s per seed, contact. Output test_results/temp_3d_wsonly/fig_1025_motion3_{computed,estimated}.png
%   | EXPIRES: when the motion comparison is read
linerr = @(e) local_break(mod(e + 90, 180) - 90);
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  seeds = 1:3;  T = 5;  n = numel(seeds);  FS = 15;  hatc = char(770);
cB = [0 0.25 0.85];  cBl = [0.6 0.72 1];  cG = [0 0.5 0.2];  cGl = [0.6 0.85 0.65];
mot = {'fixed point', 'u v motion (circle 3 \mum, 8 Hz)', 'w motion (5 \mum/s, 11.5 \mum)'};
forms = {'computed', 'estimated'};
for f = 1:2
    runs = cell(3, n);
    for i = 1:n
        for m = 1:3
            o = struct('theta_y', 20, 'h0', 15 / Rr, 'seeds', seeds(i), 'T', T, 'aprime', true, 'tag', sprintf('%s_m%d', forms{f}, m));
            if f == 2; o.form = 'estimated';  o.th0_deg = [0 0]; end
            if m == 2; o.dither = [3 8 2];  o.ap_q = true; end
            if m == 3; o.ramp = [5 11.5];  o.ap_q = true; end
            R = run_t3_xyz_fixed(o);  runs{m, i} = R.runs{1};
        end
    end
    for m = 1:3
        me = NaN(n, 2);  ct = 0;
        for i = 1:n
            r = runs{m, i};  kk = find(r.t >= 3 & r.t <= r.t(r.kend));  ct = ct + r.contact;
            me(i, :) = [mean(abs(linerr(r.thy(kk) - 20))), mean(abs(linerr(r.thx(kk))))];
        end
        fprintf('%-9s | %-30s | contact %d | mean |err| 3-5 s: theta_y %s, theta_x %s deg\n', forms{f}, mot{m}, ct, ...
                mat2str(round(me(:, 1)', 2)), mat2str(round(me(:, 2)', 2)));
    end
    fig = figure('Color', 'w', 'Position', [60 80 1800 1000]);
    tiledlayout(3, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    for row = 1:3
        for m = 1:3
            nexttile;  hold on;  box on;  hs = gobjects(0);
            for i = n:-1:1
                r = runs{m, i};  k = 1:r.kend;  s1 = (seeds(i) == 1);
                switch row
                    case 1; y = linerr(r.thy(k) - 20); col = cBl; if s1; col = cB; end
                    case 2; y = linerr(r.thx(k));      col = cGl; if s1; col = cG; end
                    case 3; y = r.h(k) * Rr;           col = cBl; if s1; col = cB; end
                end
                h = plot(r.t(k), y, 'Color', col, 'LineWidth', 1 + 0.8 * s1);  if s1; hs = h; end
            end
            if row < 3; yline(0, 'r', 'LineWidth', 2.5); end
            uistack(hs, 'top');  xlim([0 T]);  set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);
            switch row
                case 1
                    ylim([-30 30]);  if m == 1; ylabel('\theta_y error  [\circ]'); end
                    legend(hs, sprintf('%s_y - %s_y,  seed 1   (%s angle)', ['θ' hatc], char(952), forms{f}), 'Location', 'northoutside');
                    text(0.97, 0.9, mot{m}, 'Units', 'normalized', 'HorizontalAlignment', 'right', 'FontSize', 13, 'FontWeight', 'bold');
                case 2
                    ylim([-30 30]);  if m == 1; ylabel('\theta_x error  [\circ]'); end
                    legend(hs, sprintf('%s_x - %s_x', ['θ' hatc], char(952)), 'Location', 'northoutside');
                case 3
                    ylim([0 16]);  if m == 1; ylabel(sprintf('h  [%sm]', char(181))); end
                    legend(hs, 'gap h', 'Location', 'northoutside');  xlabel('Time (sec)');
            end
        end
    end
    exportgraphics(fig, fullfile(fo, sprintf('fig_1025_motion3_%s.png', forms{f})), 'Resolution', 150);
    close(fig);
end

function e = local_break(e)
    e = e(:);  j = find(abs(diff(e)) > 90);  e(j + 1) = NaN;
end
