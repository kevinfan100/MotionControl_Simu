% STATUS: SCRATCH -- 2026-11-01 (user): cross-section of the 'fillet' wall for rho 1 and 4 um: wall, probe (radius 2.25 um) at the concave junction, level set of the model gap at the target 2.51 um (branch logic copied from local_wall_now in run_t3_xyz_fixed.m), and the gap the model reports along a horizontal path at z = 2.51 um.
%   Figure fig_1101_wall_profile.png. Console: gap jump at the ring radius.
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
rbd = 5;  Rr = 2.25;  g0 = 2.51;  rh = [1 4];  FS = 14;  cW = [0.35 0.35 0.35];  cE = [0 0.25 0.85];  cT = [0.85 0 0];
f = figure('Color', 'w', 'Position', [20 20 1500 950]);  tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for j = 1:2
    rho = rh(j);  Rt = 2 * sqrt(rbd * rho);  a_s = atan2(rbd - rho, -Rt);  aa = linspace(a_s, -pi / 2 + 2 * pi * (a_s > -pi / 2), 300);  aa = linspace(a_s, 2 * pi - pi / 2 + 0, 300);  aa = aa(mod(aa - a_s, 2 * pi) <= mod(-pi / 2 - a_s, 2 * pi));
    fx = Rt + rho * cos(aa);  fz = rho + rho * sin(aa);  ab = linspace(pi / 2, atan2(rho - rbd, Rt), 200);  bx = rbd * cos(ab);  bz = rbd + rbd * sin(ab);
    [R, Z] = meshgrid(linspace(0, 14, 701), linspace(0, 11, 551));  H = arrayfun(@(a, b) local_h(a, b, rbd, rho), R, Z);
    a1 = nexttile(tl, j);  hold on;  box on;  fill([bx fx 14 14 0], [bz fz 0 -1 -1], [0.88 0.88 0.88], 'EdgeColor', 'none', 'HandleVisibility', 'off');  plot([bx fx 14], [bz fz 0], 'Color', cW, 'LineWidth', 2, 'DisplayName', 'wall');
    contour(R, Z, H, [g0 g0], 'Color', cE, 'LineWidth', 1.8, 'DisplayName', 'model gap = 2.51 \mum');  th = linspace(0, 2 * pi, 100);  pcx = Rt + 0.0;  pcz = g0;  plot(pcx + Rr * cos(th), pcz + Rr * sin(th), 'Color', cT, 'LineWidth', 1.6, 'DisplayName', 'probe (radius 2.25 \mum)');  plot(pcx, pcz, '.', 'Color', cT, 'MarkerSize', 16, 'HandleVisibility', 'off');
    plot(Rt, rho, 'k+', 'MarkerSize', 10, 'LineWidth', 1.4, 'DisplayName', 'fillet tube centre');  axis equal;  xlim([0 14]);  ylim([-0.5 11]);  xlabel('radial position  [\mum]');  ylabel('z  [\mum]');  legend('Location', 'northoutside', 'NumColumns', 2);
    r1 = linspace(0, 14, 2801);  h1 = arrayfun(@(a) local_h(a, g0, rbd, rho), r1);  a2 = nexttile(tl, 2 + j);  hold on;  box on;  plot(r1, h1, 'Color', cE, 'LineWidth', 1.6, 'DisplayName', sprintf('model gap, \\rho = %d \\mum, z = 2.51 \\mum', rho));  yline(Rr, 'k-', 'LineWidth', 1.2, 'DisplayName', 'contact 2.25 \mum');  ylim([0 4]);  xlim([0 14]);  xlabel('radial position  [\mum]');  ylabel('gap  [\mum]');  legend('Location', 'northoutside', 'Orientation', 'horizontal');
    [mx, im] = max(abs(diff(h1)));  fprintf('rho %d: ring radius %.2f um, tube centre z %d um, largest gap step %.2f um at radial %.2f um (from %.2f to %.2f)\n', rho, Rt, rho, mx, r1(im), h1(im), h1(im + 1));
    for a = [a1 a2]; set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);  set(a.Toolbar, 'Visible', 'off'); end
end
fn = fullfile(fo, 'fig_1101_wall_profile.png');  exportgraphics(f, fn, 'Resolution', 130);  close(f);

function h = local_h(rq, z, rbd, rho)
    Rt = 2 * sqrt(rbd * rho);  a_q = atan2(z - rho, rq - Rt);  a_s = atan2(rbd - rho, -Rt);  inarc = mod(a_q - a_s, 2 * pi) <= mod(-pi / 2 - a_s, 2 * pi);  dt = hypot(Rt - rq, rho - z);
    if dt < rho && inarc; h = rho - dt;
    elseif rq >= Rt; h = z;
    else; h = hypot(rq, z - rbd) - rbd; end
end
