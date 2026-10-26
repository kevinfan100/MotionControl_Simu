% STATUS: SCRATCH -- 2026-10-24 (user: wall and trajectory): flow_wait5 (start 15 um, wait 5 s, regulation, scan from 12 s), seed 1.
%   Top: measurement coordinate x-z (wall drawn, theta_y = 20 deg), probe position p, computed | estimated angle. Bottom: local coordinate
%   along the wall u [um] vs gap h [um] (wall at h = 0, red = target gap). Dots: start, regulation start, scan start.
%   Output test_results/temp_3d_wsonly/fig_t3_flow_wait5_path.png | EXPIRES: when the flow is read
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
C = load(fullfile(fo, 'temp_t3_flow_wait5.mat'));  runs = {C.rc, C.rb};  Rr = 2.25;  FS = 15;
arm = {'computed angle', 'estimated angle'};  hstar = C.rb.h_star * Rr;
th = deg2rad(20);  w = [sin(th); 0; cos(th)];  u = [cos(th); 0; -sin(th)];
fig = figure('Color', 'w', 'Position', [80 80 1300 900]);
tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for c = 1:2
    r = runs{c};  k = 1:r.kend;  p = r.p(k, :)';                         % [um]
    pz = w' * p(:, 1) - r.h(1) * Rr;                                      % wall offset along w
    t0 = [1, find(r.t >= C.o.hold_until, 1), find(r.t >= C.o.t_scan, 1)];
    % ---- top: measurement coordinate ----
    nexttile(c);  hold on;  box on;  axis equal;
    s = linspace(-60, 160, 2);  wl = pz * w + u * s;                      % wall line in x-z
    fill([wl(1, :), wl(1, 2) - 40 * w(1), wl(1, 1) - 40 * w(1)], [wl(3, :), wl(3, 2) - 40 * w(3), wl(3, 1) - 40 * w(3)], ...
         [0.85 0.85 0.85], 'EdgeColor', 'none');
    plot(wl(1, :), wl(3, :), 'k', 'LineWidth', 2);
    plot(p(1, :), p(3, :), 'b', 'LineWidth', 1.5);
    plot(p(1, t0), p(3, t0), 'ko', 'MarkerFaceColor', 'y', 'MarkerSize', 8);
    xlim([-30 160]);  ylim([-60 30]);  xlabel(sprintf('x  [%sm]', char(181)));  if c == 1; ylabel(sprintf('z  [%sm]', char(181))); end
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);
    text(0.03, 0.93, sprintf('%s, seed 1  (measurement coordinate)', arm{c}), 'Units', 'normalized', 'FontSize', FS, 'FontWeight', 'bold');
    % ---- bottom: local coordinate, u along the wall vs gap h ----
    nexttile(c + 2);  hold on;  box on;
    us = u' * (p - p(:, 1));  hg = r.h(k)' * Rr;
    fill([-20 160 160 -20], [0 0 -1 -1], [0.85 0.85 0.85], 'EdgeColor', 'none');  yline(0, 'k', 'LineWidth', 2);
    yline(hstar, 'r', 'LineWidth', 2.5);
    plot(us, hg, 'b', 'LineWidth', 1.5);  plot(us(t0), hg(t0), 'ko', 'MarkerFaceColor', 'y', 'MarkerSize', 8);
    xlim([-20 160]);  ylim([-1 16]);  xlabel(sprintf('u along the wall  [%sm]', char(181)));
    if c == 1; ylabel(sprintf('gap h  [%sm]', char(181))); end
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);
    text(0.03, 0.93, 'local coordinate (red = target gap)', 'Units', 'normalized', 'FontSize', FS, 'FontWeight', 'bold');
end
exportgraphics(fig, fullfile(fo, 'fig_t3_flow_wait5_path.png'), 'Resolution', 150);
close(fig);
