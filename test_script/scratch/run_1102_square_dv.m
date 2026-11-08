% STATUS: SCRATCH -- 2026-11-02 (user): square range with a denser v: line spacing .5 um (53 lines) against 1 um, turn at x = +-13 um, rho 4, a_d .07, tau_a .06, seeds 1-3, cached in temp_square07_dv.mat. Figure 3 window: nominal route. Figure 4 window: recovered lines seen from above, estimated w-hat and true w-hat, to separate the crossings from the angle error of the run in temp_rho4_report.mat.
%   Sanity: y at the start -13, 53 lines. Outputs fig_1102_square_dv.png, fig_1102_lines_top.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');  Rr = 2.25;  Ts = 1 / 1612;  ad = 0.07;  rbd = 5;  rho = 4;  MU = [char(956) 'm'];  NS = 3;  dy = 0.5;  nl = 53;  hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
S = load(fullfile(fo, 'temp_rho4_report.mat'), 'r');  q = S.r;  ok = find(isfinite(q.aN(1:q.kend)));  ks = ok(q.t(ok) >= q.tsc + 0.5 & q.t(ok) <= q.rs_tend);  p = q.p(ks, :);  tx = q.thx(ks) * pi / 180;  ty = q.thy(ks) * pi / 180;  tx = tx(:);  ty = ty(:);
Wh = [sin(ty) .* cos(tx), -sin(tx), cos(ty) .* cos(tx)];  Vh = [sin(ty) .* sin(tx), cos(tx), cos(ty) .* sin(tx)];  a1 = q.th_tr(ks, 1) * pi / 180;  a2 = q.th_tr(ks, 2) * pi / 180;  Wt = [sin(a2) .* cos(a1), -sin(a1), cos(a2) .* cos(a1)];  dv = [0; sum(diff(p, 1, 1) .* Vh(2:end, :), 2)];  stepv = conv(double(movmean(dv, 21) > 0.012), ones(61, 1), 'same') > 0;
if any(findall(0, 'Type', 'figure') == 4); close(4); end
f4 = figure(4);  set(f4, 'Color', 'w', 'Position', [100 100 1500 760], 'Visible', 'on', 'Name', 'recovered lines from above');  tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');  tt = {'estimated w', 'true w'};  A = {Wh, Wt};
for j = 1:2
    W = movmean(p - Rr * hs * A{j}, 41, 1);  W(stepv | abs(W(:, 1)) > 13 | abs(W(:, 2)) > 13, :) = NaN;  a = nexttile(tl);  plot(a, W(:, 1), W(:, 2), 'Color', [0.9 0 0], 'LineWidth', 0.9);  axis(a, 'equal');  xlim(a, [-13 13]);  ylim(a, [-13 13]);  box(a, 'on');  title(a, tt{j});  xlabel(a, ['X (' MU ')']);  ylabel(a, ['Y (' MU ')']);  set(a, 'FontSize', 14, 'FontWeight', 'bold', 'LineWidth', 1.1);
end
exportgraphics(f4, fullfile(fo, 'fig_1102_lines_top.png'), 'Resolution', 120);  drawnow;
cf = fullfile(fo, 'temp_square07_dv.mat');
if isfile(cf)
    load(cf, 'rs');
else
    base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 0.06, 'a_d', ad, 'descent_v', 15 - hs * Rr, ...
                  'ap_q', true, 'wall', 'fillet', 'bead_r', rbd, 'fillet_rho', rho, 'bead_start', [13; -13], 'ap_floor_law', true, 'raster', [13 dy nl], 'scan_ramp', 0.5, 'scan_len', 1e6, 'sP_fix_theta_deg', 2, 'q_geo', [0.9 0.9], ...
                  'descent_T', 1, 'post_wait', [0 1], 'tag', 'sd', 'du_d', 40 * Ts / Rr, 'T', 12 + 1.4 * nl * 27 / 40 + 1, 'kappa_state', true, 'kappa_from_scan', true, 'kappa_pfix', 0.3);
    rs = cell(NS, 1);
    if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
    pctRunOnAll warning('off', 'all');
    parfor j = 1:NS
        o = base;  o.seeds = j;  R = run_t3_xyz_fixed(o);  r = R.runs{1};
        rs{j} = struct('t', r.t, 'kend', r.kend, 'contact', r.contact, 'tsc', r.tsc, 'te', r.rs_tend, 'p', r.p, 'h', r.h, 'aN', r.aN);
    end
    save(cf, 'rs', '-v7.3');
end
for j = 1:NS
    r = rs{j};  okr = find(isfinite(r.aN(1:r.kend)) & isfinite(r.h(1:r.kend)));  fin = isfinite(r.te) && ~r.contact;  te = r.t(okr(end));  if isfinite(r.te); te = r.te; end;  k = okr(r.t(okr) >= r.tsc & r.t(okr) <= te);
    fprintf('seed %d: finished %d, contact %d, scan %.2f-%.2f s, min gap %.3f um, y %.1f..%.1f um\n', j, fin, r.contact, r.tsc, te, min(r.h(k)) * Rr, min(r.p(k, 2)), max(r.p(k, 2)));
end
g = hs * Rr;  Rt = 2 * sqrt(rbd * rho);  rgj = rbd + g;  P = [13, -13, 15;  13, -13, g];  sgn = -1;  yv = -13 + (0:nl - 1) * dy;  xs = (13:-0.05:-13)';
for L = 1:nl
    x = sgn * xs;  y = yv(L) * ones(size(x));  P = [P; x, y, local_z(hypot(x, y), rbd, rho, g, Rt, rgj)];  if L < nl; P = [P; x(end), yv(L + 1), g]; end;  sgn = -sgn;
end
fprintf('nominal: %d lines, %.1f um, %.1f s at 40 um/s\n', nl, sum(vecnorm(diff(P(2:end, :), 1, 1), 2, 2)), sum(vecnorm(diff(P(2:end, :), 1, 1), 2, 2)) / 40);
if any(findall(0, 'Type', 'figure') == 3); close(3); end
f = figure(3);  set(f, 'Color', 'w', 'Position', [60 60 1250 860], 'Visible', 'on', 'Name', 'square route, v 0.5 um');  a = axes(f);  hold(a, 'on');
[Xs, Ys, Zs] = sphere(80);  surf(a, rbd * Xs, rbd * Ys, rbd + rbd * Zs, 'FaceColor', [0.85 0.72 0.45], 'EdgeColor', 'none', 'AmbientStrength', 0.6, 'DiffuseStrength', 0.6, 'SpecularStrength', 0);  camlight(a);  lighting(a, 'gouraud');  patch(a, [-16 16 16 -16], [-16 -16 16 16], [-0.02 -0.02 -0.02 -0.02], [0.74 0.76 0.82], 'EdgeColor', 'none', 'FaceLighting', 'none');
h1 = plot3(a, P(1:2, 1), P(1:2, 2), P(1:2, 3), 'Color', [0.85 0 0], 'LineWidth', 2.2);  h2 = plot3(a, P(2:end, 1), P(2:end, 2), P(2:end, 3), 'Color', [0 0 1], 'LineWidth', 0.6);  h3 = plot3(a, P(1, 1), P(1, 2), P(1, 3), 'o', 'MarkerSize', 11, 'MarkerFaceColor', [0 0.6 0.2], 'MarkerEdgeColor', 'k');  h5 = plot3(a, P(end, 1), P(end, 2), P(end, 3), '^', 'MarkerSize', 11, 'MarkerFaceColor', [0.95 0.6 0], 'MarkerEdgeColor', 'k');
axis(a, 'equal');  xlim(a, [-16 16]);  ylim(a, [-16 16]);  zlim(a, [-0.2 16]);  view(a, -37.5, 25);  grid(a, 'off');  box(a, 'off');  xlabel(a, ['X (' MU ')']);  ylabel(a, ['Y (' MU ')']);  zlabel(a, ['Z (' MU ')']);  set(a, 'FontSize', 16, 'LineWidth', 1.1);  legend([h3 h1 h2 h5], {'start', 'descent', 'scan', 'end'}, 'Location', 'northoutside', 'Orientation', 'horizontal');  rotate3d(a, 'on');
exportgraphics(f, fullfile(fo, 'fig_1102_square_dv.png'), 'Resolution', 130);

function z = local_z(r, rb, rho, g, Rt, rgj)
    z = g * ones(size(r));  a = r < rgj;  z(a) = rb + sqrt(rgj^2 - r(a).^2);  c = r >= rgj & r < Rt;  z(c) = rho - sqrt((rho - g)^2 - (Rt - r(c)).^2);
end
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
