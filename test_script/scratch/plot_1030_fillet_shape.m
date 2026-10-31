% STATUS: SCRATCH -- 2026-10-30 (user): what the wall 'fillet' looks like -- bead radius 5 um on the glass, concave junction filled by a torus
%   of tube radius rho (tangent to both). Left: r-z cross-section for rho = 10 um (used now) and 6 um (earlier), with the probe-centre path
%   at the target gap (a_d 0.1 -> 2.51 um) and the probe (radius 2.25 um) drawn at a few points. Right: 3D surface of bead + fillet + glass
%   for rho = 10 um. Output test_results/temp_3d_wsonly/fig_1030_fillet_shape.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
rbd = 5;  gap = 2.51;  Rp = 2.25;  FS = 15;  cGr = [0.5 0.5 0.5];  cF = [0.9 0.45 0];  cB = [0 0.25 0.85];
f = figure('Color', 'w', 'Position', [20 20 2100 850]);  tl = tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
rhos = [10 6];
for i = 1:2
    rho = rhos(i);  Rt = 2 * sqrt(rbd * rho);  rs = Rt * rbd / (rbd + rho);  zs = rho + rho * (rbd - rho) / (rbd + rho);
    ax = nexttile(tl, i);  hold on;  box on;
    rr = linspace(0, 18, 2000);  zw = arrayfun(@(x) local_z(x, rbd, rho, Rt), rr);
    fill([rr fliplr(rr)], [zw -ones(size(rr))], [0.9 0.9 0.9], 'EdgeColor', 'none', 'HandleVisibility', 'off');
    kb = rr <= rs;  kf = rr > rs & rr < Rt;  kg = rr >= Rt;
    plot(rr(kb), zw(kb), 'Color', cGr, 'LineWidth', 3, 'DisplayName', 'bead (r 5 \mum)');  plot(rr(kf), zw(kf), 'Color', cF, 'LineWidth', 3.5, 'DisplayName', sprintf('fillet (\\rho %d \\mum)', rho));
    plot(rr(kg), zw(kg), 'k', 'LineWidth', 3, 'DisplayName', 'glass');
    ta = linspace(0, 2 * pi, 200);  plot(Rt + rho * cos(ta), rho + rho * sin(ta), ':', 'Color', cF, 'LineWidth', 1.2, 'HandleVisibility', 'off');
    % probe-centre path at the target gap: offset of the wall by Rp + gap? (gap = centre distance to the wall in this code: h = gap)
    P = local_offset(rbd, rho, Rt, gap);  plot(P(:, 1), P(:, 2), '--', 'Color', cB, 'LineWidth', 2, 'DisplayName', sprintf('probe centre at gap %.2f \\mum', gap));
    for s = round(linspace(1, size(P, 1), 7)); plot(P(s, 1) + Rp * cos(ta), P(s, 2) + Rp * sin(ta), 'Color', [cB 0.5], 'LineWidth', 1, 'HandleVisibility', 'off'); end
    plot([rs Rt], [zs 0], 'o', 'Color', cF, 'MarkerFaceColor', cF, 'MarkerSize', 7, 'HandleVisibility', 'off');
    text(rs + 0.3, zs + 0.6, sprintf('(%.2f, %.2f)', rs, zs), 'FontSize', 12, 'Color', cF);  text(Rt - 1, -0.6, sprintf('r = %.2f', Rt), 'FontSize', 12, 'Color', cF);
    axis equal;  xlim([0 18]);  ylim([-1 14]);  xlabel('r  [\mum]  (distance from the bead axis)');  ylabel('z  [\mum]');
    title(sprintf('cross-section, \\rho = %d \\mum', rho));  if i == 1; legend('Location', 'northeast'); end
    set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);  set(ax.Toolbar, 'Visible', 'off');
end
rho = 10;  Rt = 2 * sqrt(rbd * rho);  ax = nexttile(tl, 3);  hold on;
[TH, RR] = meshgrid(linspace(0, 2 * pi, 120), linspace(0, 18, 300));  ZZ = arrayfun(@(x) local_z(x, rbd, rho, Rt), RR);
surf(RR .* cos(TH), RR .* sin(TH), ZZ, ZZ, 'EdgeColor', 'none');  colormap(ax, parula);  camlight;  lighting gouraud;
axis equal;  xlim([-18 18]);  ylim([-18 18]);  zlim([0 11]);  view(-35, 30);  xlabel('x  [\mum]');  ylabel('y  [\mum]');  zlabel('z  [\mum]');
title('bead + fillet (\rho 10 \mum) + glass');  set(ax, 'FontSize', FS, 'FontWeight', 'bold');  set(ax.Toolbar, 'Visible', 'off');
fn = fullfile(fo, 'fig_1030_fillet_shape.png');  exportgraphics(f, fn, 'Resolution', 130);  close(f);  system(['open ' fn]);

function z = local_z(r, rbd, rho, Rt)
    rs = Rt * rbd / (rbd + rho);
    if r <= rs; z = rbd + sqrt(max(rbd^2 - r^2, 0)); elseif r < Rt; z = rho - sqrt(max(rho^2 - (Rt - r)^2, 0)); else; z = 0; end
end
function P = local_offset(rbd, rho, Rt, g)                    % probe-centre curve: bead offset (radius rbd + g), fillet offset (radius rho - g), glass at z = g
    as = atan2(rbd - rho, -Rt);                                 % direction from the tube centre to the bead centre
    a1 = linspace(pi / 2, atan2(-(rbd - rho) * 0 + (rho - rbd), Rt), 1);   %#ok<NASGU>
    tb = linspace(pi / 2, atan2(rho - rbd, Rt), 200);          % bead part: polar angle from the bead centre, top -> tangency direction
    Pb = [(rbd + g) * cos(tb') , rbd + (rbd + g) * sin(tb')];
    tf = linspace(as, -pi / 2, 200);                            % fillet part: from the tangency direction to straight down (from the tube centre)
    Pf = [Rt + (rho - g) * cos(tf'), rho + (rho - g) * sin(tf')];
    Pg = [linspace(Rt, 18, 50)', g * ones(50, 1)];
    P = [Pb; Pf; Pg];
end
