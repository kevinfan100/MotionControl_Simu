% STATUS: SCRATCH -- 2026-11-02 (user): actual trajectory of the magnetic bead (probe centre, no recovery, no smoothing, no crop), start, descent and scan, over the plain glass bead and floor. temp_square_why.mat: fixed 36 um seed 2 | square +-13 seed 4 (finished) | square +-13 seed 2 (contact). MATLAB window 7. Console: minimum gap of every run. Sanity: start (x, -13, 15) um. Output fig_1102_probe_path.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');  Rr = 2.25;  rbd = 5;  MU = [char(956) 'm'];  load(fullfile(fo, 'temp_square_why.mat'), 'rs', 'A');
nm = {'+-13', '+-18', 'fixed 36', '+-13 v.5'};  G = NaN(numel(rs), 2);
for q = 1:numel(rs)
    r = rs{q};  ok = find(isfinite(r.aN(1:r.kend)) & isfinite(r.h(1:r.kend)));  fin = isfinite(r.te) && ~r.contact;  te = r.t(ok(end));  if fin; te = r.te; end;  k = ok(r.t(ok) >= r.tsc + 0.5 & r.t(ok) <= te);  G(q, :) = [min(r.h(k)) * Rr - Rr, fin];  [~, im] = min(r.h(k));  fprintf('%-9s seed %d: finished %d, min gap - contact %.1f nm at t %.1f s, r %.2f um, gap sd %.0f nm\n', nm{A(q).a}, A(q).seed, fin, 1e3 * G(q, 1), r.t(k(im)), hypot(r.p(k(im), 1), r.p(k(im), 2)), 1e3 * std(r.h(k)) * Rr);
end
sel = [12 4 2];  tt = {'fixed 36 \mum, seed 2: finished', 'square \pm13 \mum, seed 4: finished', 'square \pm13 \mum, seed 2: contact'};  [Xs, Ys, Zs] = sphere(80);
if any(findall(0, 'Type', 'figure') == 7); close(7); end
f = figure(7);  set(f, 'Color', 'w', 'Position', [30 60 2000 760], 'Visible', 'on', 'Name', 'actual trajectory of the magnetic bead');  tl = tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
for j = 1:3
    r = rs{sel(j)};  ok = find(isfinite(r.h(1:r.kend)) & all(isfinite(r.p(1:r.kend, :)), 2));  fin = isfinite(r.te) && ~r.contact;  te = r.t(ok(end));  if fin; te = r.te; end;  k = ok(r.t(ok) <= te);  P = r.p(k, :);  t = r.t(k);  t = t(:);  pre = t < r.tsc;
    a = nexttile(tl);  hold(a, 'on');  surf(a, rbd * Xs, rbd * Ys, rbd + rbd * Zs, 'FaceColor', [0.85 0.72 0.45], 'EdgeColor', 'none', 'AmbientStrength', 0.6, 'DiffuseStrength', 0.6, 'SpecularStrength', 0);  camlight(a);  lighting(a, 'gouraud');  patch(a, [-26 26 26 -26], [-16 -16 16 16], [-0.02 -0.02 -0.02 -0.02], [0.74 0.76 0.82], 'EdgeColor', 'none', 'FaceLighting', 'none');
    plot3(a, P(pre, 1), P(pre, 2), P(pre, 3), 'Color', [0.85 0 0], 'LineWidth', 2.0);  plot3(a, P(~pre, 1), P(~pre, 2), P(~pre, 3), 'Color', [0 0 1], 'LineWidth', 0.7);  plot3(a, P(1, 1), P(1, 2), P(1, 3), 'o', 'MarkerSize', 10, 'MarkerFaceColor', [0 0.6 0.2], 'MarkerEdgeColor', 'k');
    if ~fin; plot3(a, P(end, 1), P(end, 2), P(end, 3), 'p', 'MarkerSize', 20, 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k'); end
    title(a, tt{j});  axis(a, 'equal');  xlim(a, [-26 26]);  ylim(a, [-16 16]);  zlim(a, [-0.2 16]);  view(a, -37.5, 25);  grid(a, 'off');  box(a, 'off');  xlabel(a, ['X (' MU ')']);  ylabel(a, ['Y (' MU ')']);  zlabel(a, ['Z (' MU ')']);  set(a, 'FontSize', 13, 'LineWidth', 1.1);  rotate3d(a, 'on');
end
exportgraphics(f, fullfile(fo, 'fig_1102_probe_path.png'), 'Resolution', 120);
