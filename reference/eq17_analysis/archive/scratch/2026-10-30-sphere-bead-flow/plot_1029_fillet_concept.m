% STATUS: SCRATCH -- 2026-10-29 (user): alternative to the weighted blend -- a fillet of radius rho fills the junction sphere (radius 5 um) /
%   bottom wall, so sphere + fillet + bottom wall is ONE smooth wall; h = plain nearest distance to it (as the hemisphere case).
%   Fillet = torus tube of radius rho centred at (r, z) = (2 sqrt(5 rho), rho), tangent to the bottom wall and to the sphere.
%   The probe-centre path at h* = 2.37 um has a rounded corner only if rho > h* (corner radius rho - h*); rho < h* leaves a kink.
%   Panels: cross-sections for rho = 2, 4, 6 um (wall, fillet, path at h*). Output test_results/temp_3d_wsonly/fig_1029_fillet_concept.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
rb = 5;  hst = 2.37;  FS = 15;  cB = [0 0.25 0.85];  cGr = [0.5 0.5 0.5];  cF = [0.85 0.4 0];  rhos = [2 4 6];
rg = linspace(0, 16, 1601);  zg = linspace(0.005, 14, 1400);  [Rq, Zq] = meshgrid(rg, zg);
f = figure('Color', 'w', 'Position', [30 30 2200 720]);  tl = tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
for i = 1:3
    rho = rhos(i);  D = arrayfun(@(r, z) local_dist(r, z, rb, rho), Rq, Zq);
    ax = nexttile(tl);  hold on;  box on;
    Rt = 2 * sqrt(rb * rho);  cs = [0 rb];  ct = [Rt rho];  a1 = mod(atan2(cs(2) - ct(2), cs(1) - ct(1)), 2 * pi);   % tube direction toward the sphere centre
    ta = linspace(a1, 3 * pi / 2, 120);                    % fillet arc: from the sphere tangency to the bottom tangency (concave from the fluid)
    aS = atan2(ct(2) - cs(2), ct(1) - cs(1));  ts = linspace(-pi / 2, aS, 60);   % sphere from its bottom point to the tangency
    tg = linspace(0, pi, 200);  fill([rb * sin(tg) 0], [rb + rb * cos(tg) rb], [0.88 0.88 0.9], 'EdgeColor', 'none', 'HandleVisibility', 'off');
    fill([ct(1) + rho * cos(ta), cs(1) + rb * cos(ts)], [ct(2) + rho * sin(ta), cs(2) + rb * sin(ts)], [0.95 0.82 0.68], 'EdgeColor', 'none', ...
         'DisplayName', sprintf('fillet, rho = %g \\mum', rho));
    fill([0 16 16 0], [0 0 -1 -1], [0.85 0.85 0.85], 'EdgeColor', 'none', 'HandleVisibility', 'off');
    plot(rb * sin(tg), rb + rb * cos(tg), 'Color', cGr, 'LineWidth', 2.5, 'DisplayName', 'sphere');  plot([0 16], [0 0], 'k', 'LineWidth', 2.5, 'DisplayName', 'bottom wall');
    plot(ct(1) + rho * cos(ta), ct(2) + rho * sin(ta), 'Color', cF, 'LineWidth', 2.5, 'HandleVisibility', 'off');
    c = contourc(rg, zg, D, [hst hst]);  j = 1;  first = true;
    while j < size(c, 2)
        m = c(2, j);  xy = c(:, j + 1:j + m);  hv = 'off';  if first; hv = 'on';  first = false; end
        plot(xy(1, :), xy(2, :), 'Color', cB, 'LineWidth', 2.5, 'DisplayName', sprintf('probe centre at h* = %.2f \\mum', hst), 'HandleVisibility', hv);  j = j + m + 1;
    end
    axis equal;  xlim([0 16]);  ylim([-1 13]);  xlabel('r  [\mum]  (distance from the sphere axis)');  if i == 1; ylabel('z  [\mum]'); end
    if rho > hst; tt = sprintf('rho = %g \\mum: path corner radius %.1f \\mum', rho, rho - hst); else; tt = sprintf('rho = %g \\mum < h*: path still has a kink', rho); end
    title(tt);  legend('Location', 'northeast');  set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(ax.Toolbar, 'Visible', 'off');
end
fn = fullfile(fo, 'fig_1029_fillet_concept.png');  exportgraphics(f, fn, 'Resolution', 150);  close(f);  system(['open ' fn]);

function d = local_dist(r, z, rb, rho)                       % nearest distance to sphere + fillet + bottom wall in the (r, z) cross-section
    Rt = 2 * sqrt(rb * rho);  cs = [0 rb];  ct = [Rt rho];  q = [r z];
    a1 = atan2(cs(2) - ct(2), cs(1) - ct(1));  vq = q - ct;  ang = mod(atan2(vq(2), vq(1)), 2 * pi);
    if norm(vq) < rho && ang >= mod(a1, 2 * pi) && ang <= 3 * pi / 2; d = rho - norm(vq);  return; end   % fillet sector (fluid inside the tube circle)
    aS = atan2(ct(2) - cs(2), ct(1) - cs(1));  angS = atan2(q(2) - cs(2), q(1) - cs(1));
    ds = Inf;  if angS >= aS; ds = norm(q - cs) - rb; end   % sphere above the tangency
    dp = Inf;  if r >= Rt; dp = z; end                      % bottom wall outside the fillet foot
    d = min(ds, dp);  if ~isfinite(d); d = min([norm(q - cs) - rb, z, abs(rho - norm(vq))]); end
end
