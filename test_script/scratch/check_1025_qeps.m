% STATUS: SCRATCH -- 2026-10-25 (user): computed angle at the 15 um fixed point, two ways to let A-hat follow: (A) A-bar' as states
%   (current, slope q = 0 at a fixed point) vs (B) no A-bar', gain random walk Q_a = eps R6 with eps = 1e-5 (Layer-1 first-order line).
%   20 deg wall, seeds 1-10, 10 s. Per element over 5-10 s: bias (SEM multiple), honesty = rms error / mean sqrt P, mean sqrt P;
%   angle: mean |theta_y error|, |theta_x error| over 5-10 s. Figure: seed 1 angle errors and a-hat_zz, A | B.
%   Output test_results/temp_3d_wsonly/fig_1025_qeps.png | EXPIRES: when the comparison is read
linerr = @(e) local_break(mod(e + 90, 180) - 90);
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  seeds = 1:10;  T = 10;  n = numel(seeds);  nm = {'xx', 'yy', 'zz', 'xy', 'xz', 'yz'};  ij = [1 1; 2 2; 3 3; 1 2; 1 3; 2 3];
arm = {'A: A-bar'' states, Q_a'' = 0', 'C: A-bar'' states, Q_a'' = 1e-5 R'};  keep = cell(2, 1);
for c = 1:2
    E = NaN(n, 6);  Er = NaN(n, 6);  S = NaN(n, 6);  ang = NaN(n, 2);
    for i = 1:n
        o = struct('theta_y', 20, 'h0', 15 / Rr, 'seeds', seeds(i), 'T', T, 'tag', 'qeps');
        o.aprime = true;  if c == 2; o.qap_eps = 1e-5; end
        R = run_t3_xyz_fixed(o);  r = R.runs{1};  if i == 1; keep{c} = r; end
        kk = find(r.t >= 5);  at = arrayfun(@(j) r.A_true(ij(j, 1), ij(j, 2)), 1:6);  e = r.a(kk, :) - at;
        E(i, :) = mean(e, 1);  Er(i, :) = sqrt(mean(e.^2, 1));  S(i, :) = mean(r.sPa(kk, :), 1);
        ang(i, :) = [mean(abs(linerr(r.thy(kk) - 20)), 'omitnan'), mean(abs(linerr(r.thx(kk))), 'omitnan')];
    end
    bias = mean(E, 1);  sem = std(E, 0, 1) / sqrt(n);  hon = sqrt(mean(Er.^2, 1)) ./ mean(S, 1);
    fprintf('\n%s | angle mean |err| 5-10 s: theta_y %.1f deg (seeds %s), theta_x %.1f deg\n', arm{c}, mean(ang(:, 1)), mat2str(round(ang(:, 1)', 1)), mean(ang(:, 2)));
    for j = 1:6
        fprintf('  a_%s: bias %+.4f (%+.1f SEM) | sd across seeds %.4f | rms err %.4f | mean sqrtP %.4f | honesty %.2f\n', nm{j}, bias(j), ...
                bias(j) / sem(j), std(E(:, j)), sqrt(mean(Er(:, j).^2)), mean(S(:, j)), hon(j));
    end
end
FS = 13;  cB = [0 0.25 0.85];  cBl = [0.6 0.72 1];  cG = [0 0.5 0.2];  hatc = char(770);
fig = figure('Color', 'w', 'Position', [80 80 1300 800]);
tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for row = 1:2
    for c = 1:2
        nexttile;  hold on;  box on;  r = keep{c};  t = r.t;
        if row == 1
            yline(0, 'r', 'LineWidth', 2.5);
            h1 = plot(t, linerr(r.thy - 20), 'Color', cB, 'LineWidth', 1.4);  h2 = plot(t, linerr(r.thx), 'Color', cG, 'LineWidth', 1.4);
            ylim([-30 30]);  legend([h1 h2], {sprintf('%s_y - %s_y   (%s)', ['θ' hatc], char(952), arm{c}), sprintf('%s_x - %s_x', ['θ' hatc], char(952))}, ...
                'Location', 'northoutside', 'Orientation', 'horizontal');  if c == 1; ylabel('angle error  [\circ]'); end
        else
            plot(t, 0 * t + 1 / (r.h(1))^2, 'r', 'LineWidth', 2.5);  h = plot(t, r.ap(:, 3), 'Color', cB, 'LineWidth', 1.2);
            ylim([-0.2 0.2]);  legend(h, sprintf('a%s''_{zz}  (red = far-field 1/h^2)', hatc), 'Location', 'northoutside');
            if c == 1; ylabel('a''_{zz}'); end;  xlabel('Time (sec)');
        end
        xlim([0 T]);  set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);
    end
end
exportgraphics(fig, fullfile(fo, 'fig_1025_qap_eps.png'), 'Resolution', 150);
close(fig);

function e = local_break(e)
    e = e(:);  j = find(abs(diff(e)) > 90);  e(j + 1) = NaN;
end
