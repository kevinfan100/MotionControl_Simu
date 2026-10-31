% STATUS: SCRATCH -- 2026-10-29 (user): where the whole-bead raster fails for the three single-knob options (temp_fillet_opts.mat from
%   run_1029_fillet_opts.m). Top row: top view, path of seed 1 + contact points of seeds 1-3, bead (r 5) and fillet edge; bottom row: the
%   contact points in the r-z cross-section with the wall profile and the probe-centre offset curve at the target gap.
%   Output test_results/temp_3d_wsonly/fig_1029_fillet_opts.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
S = load(fullfile(fo, 'temp_fillet_opts.mat'));  rs = S.rs;  arm = S.arm;  Rr = S.Rr;  hs = S.hs;
FS = 14;  cE = [0 0.25 0.85];  cGr = [0.5 0.5 0.5];  cs = [0.85 0 0; 0.9 0.5 0; 0.5 0 0.6];  rbd = 5;  tg = linspace(0, 2 * pi, 300);
f = figure('Color', 'w', 'Position', [20 20 1950 1100]);  tl = tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
for ia = 1:3
    rho = 6;  if ia == 2; rho = 10; end;  Rt = 2 * sqrt(rbd * rho);
    a1 = nexttile(tl, ia);  hold on;  box on;  plot(rbd * cos(tg), rbd * sin(tg), 'Color', cGr, 'LineWidth', 2.5);  plot(Rt * cos(tg), Rt * sin(tg), '--', 'Color', cGr, 'LineWidth', 1.5);
    r = rs{ia, 1};  kk = (1:r.kend)';  ks = kk(r.t(kk) >= r.tsc);  plot(r.p(ks, 1), r.p(ks, 2), 'Color', cE, 'LineWidth', 0.8);
    for id = 1:3; r = rs{ia, id};  pe = r.p(r.kend, :);  plot(pe(1), pe(2), 'x', 'Color', cs(id, :), 'MarkerSize', 16, 'LineWidth', 3); end
    axis equal;  xlim([-16 16]);  ylim([-16 16]);  xlabel('x  [\mum]');  ylabel('y  [\mum]');  title(sprintf('%s (path: seed 1)', arm{ia}));
    a2 = nexttile(tl, 3 + ia);  hold on;  box on;  rr = linspace(0, 16, 800);  zw = arrayfun(@(x) local_wall_z(x, rbd, rho, Rt), rr);
    fill([rr fliplr(rr)], [zw zeros(size(rr)) - 1], [0.88 0.88 0.88], 'EdgeColor', 'none', 'HandleVisibility', 'off');  plot(rr, zw, 'Color', cGr, 'LineWidth', 2.5, 'DisplayName', 'wall (r-z)');
    for id = 1:3; r = rs{ia, id};  pe = r.p(r.kend, :);  plot(hypot(pe(1), pe(2)), pe(3), 'x', 'Color', cs(id, :), 'MarkerSize', 16, 'LineWidth', 3, 'DisplayName', sprintf('contact, seed %d', id)); end
    axis equal;  xlim([0 16]);  ylim([-1 13]);  xlabel('r  [\mum]');  ylabel('z  [\mum]');  if ia == 1; legend('Location', 'northeast'); end
    for a = [a1 a2]; set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off'); end
end
fn = fullfile(fo, 'fig_1029_fillet_opts.png');  exportgraphics(f, fn, 'Resolution', 120);  close(f);  system(['open ' fn]);

function z = local_wall_z(r, rbd, rho, Rt)
    rs = Rt * rbd / (rbd + rho);
    if r <= rs; z = rbd + sqrt(max(rbd^2 - r^2, 0)); elseif r < Rt; z = rho - sqrt(max(rho^2 - (Rt - r)^2, 0)); else; z = 0; end
end
