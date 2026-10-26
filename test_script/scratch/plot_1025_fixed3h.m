% STATUS: SCRATCH -- 2026-10-25 (user): why does the angle converge slowly at a fixed point -- three heights 50 / 15 / 5 um, 20 deg wall,
%   seeds 1-3, 10 s, both forms (estimated: theta-hat[0] = 0). Rows = heights, columns = computed | estimated; each panel theta_y error
%   (blue) and theta_x error (green), seed 1 dark. Console per height:
%   gain difference d = a_par - a_perp (true); computed form: rms error of the off-diagonal element a-hat_xz over 5-10 s and the angle
%   error it implies, rms(e_xz) / d / 2 [deg] (the angle is the direction of (a_xx - a_zz, -2 a_xz), whose length is d);
%   estimated form: sqrt P of theta_y at 1, 5, 10 s. Output test_results/temp_3d_wsonly/fig_1025_fixed3h.png | EXPIRES: when explained
linerr = @(e) local_break(mod(e + 90, 180) - 90);
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  um = [50 15 5];  seeds = 1:3;  T = 10;  n = numel(seeds);  FS = 14;  hatc = char(770);
cB = [0 0.25 0.85];  cBl = [0.6 0.72 1];  cG = [0 0.5 0.2];  cGl = [0.6 0.85 0.65];
runs = cell(numel(um), 2, n);
for ih = 1:numel(um)
    for i = 1:n
        o = struct('theta_y', 20, 'h0', um(ih) / Rr, 'seeds', seeds(i), 'T', T, 'aprime', true, 'tag', 'f3h');
        R = run_t3_xyz_fixed(o);  runs{ih, 1, i} = R.runs{1};
        o.form = 'estimated';  o.th0_deg = [0 0];  R = run_t3_xyz_fixed(o);  runs{ih, 2, i} = R.runs{1};
    end
    r = runs{ih, 1, 1};  d = r.aT_true(1) - r.aN_true(1);  exz = NaN(n, 1);  ecmp = NaN(n, 1);  sp = NaN(n, 3);  eest = NaN(n, 1);
    for i = 1:n
        rc = runs{ih, 1, i};  kk = find(rc.t >= 5);  exz(i) = sqrt(mean((rc.a(kk, 5) - rc.A_true(1, 3)).^2));
        ecmp(i) = mean(abs(linerr(rc.thy(kk) - 20)));
        re = runs{ih, 2, i};  k3 = arrayfun(@(s) find(re.t >= s - 1e-9, 1), [1 5 10]);  sp(i, :) = re.sPth(k3, 2)';
        eest(i) = mean(abs(linerr(re.thy(kk) - 20)));
    end
    fprintf('%2d um | d = a_par - a_perp = %.3f | computed: rms(e_xz) %.4f -> rms(e_xz)/(2d) = %.1f deg, measured mean |e_thy| 5-10 s %s deg | estimated: sqrtP theta_y at 1/5/10 s %s deg, measured %s deg\n', ...
            um(ih), d, mean(exz), mean(exz) / (2 * d) * 180 / pi, mat2str(round(ecmp', 1)), mat2str(round(mean(sp, 1), 1)), mat2str(round(eest', 1)));
end
ttl = {'computed angle', sprintf('estimated angle, %s[0] = 0', ['θ' hatc])};
fig = figure('Color', 'w', 'Position', [60 60 1300 1200]);
tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for ih = 1:numel(um)
    for c = 1:2
        nexttile;  hold on;  box on;
        for i = n:-1:1
            r = runs{ih, c, i};  k = 1:r.kend;  s1 = (seeds(i) == 1);
            cy = cBl;  cx = cGl;  if s1; cy = cB;  cx = cG; end
            hy = plot(r.t(k), linerr(r.thy(k) - 20), 'Color', cy, 'LineWidth', 1 + 0.8 * s1);
            hx = plot(r.t(k), linerr(r.thx(k)), 'Color', cx, 'LineWidth', 1 + 0.8 * s1);
            if s1; h1 = hy; h2 = hx; end
        end
        yline(0, 'r', 'LineWidth', 2.5);  uistack([h1 h2], 'top');  xlim([0 T]);  ylim([-30 30]);
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);
        if c == 1; ylabel('angle error  [\circ]'); end
        r1 = runs{ih, c, 1};
        text(0.97, 0.9, sprintf('%d %sm  (%s_%s - %s_%s = %.2f)', um(ih), char(181), char(257), char(8741), char(257), char(8869), ...
             r1.aT_true(1) - r1.aN_true(1)), 'Units', 'normalized', 'HorizontalAlignment', 'right', 'FontSize', 13, 'FontWeight', 'bold');
        if ih == 1
            legend([h1 h2], {sprintf('%s_y - %s_y   (%s)', ['θ' hatc], char(952), ttl{c}), sprintf('%s_x - %s_x', ['θ' hatc], char(952))}, ...
                   'Location', 'northoutside', 'Orientation', 'horizontal');
        end
        if ih == numel(um); xlabel('Time (sec)'); end
    end
end
exportgraphics(fig, fullfile(fo, 'fig_1025_fixed3h.png'), 'Resolution', 150);
close(fig);

function e = local_break(e)
    e = e(:);  j = find(abs(diff(e)) > 90);  e(j + 1) = NaN;
end
