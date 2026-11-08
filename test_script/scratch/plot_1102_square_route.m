% STATUS: SCRATCH -- 2026-11-02 (user): nominal complete route of the square range (start, descent, turn at x = +-13 um, y step 1 um, 27 lines, centre of the probe at gap a_d .07 over the rho 4 wall) in a MATLAB 3-D window; console gives the simulated square runs, tau_a .06, seeds 1-3 (all contact). Second window: wall recovered along the scan lines, p - R (1 + h*) w-hat, of temp_rho4_report.mat, no fitted sphere, no surface.
%   Sanity: route starts at (13, -13, 15) um, y steps 1 um. Run cached in temp_square07.mat. Outputs fig_1102_square_route.png, fig_1102_wall_lines.png. Windows stay open.
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  Ts = 1 / 1612;  nl = 27;  ad = 0.07;  rbd = 5;  rho = 4;  MU = [char(956) 'm'];  NS = 3;  hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
cf = fullfile(fo, 'temp_square07.mat');
if isfile(cf)
    load(cf, 'rs');
else
    base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 0.06, 'a_d', ad, 'descent_v', 15 - hs * Rr, ...
                  'ap_q', true, 'wall', 'fillet', 'bead_r', rbd, 'fillet_rho', rho, 'bead_start', [13; -13], 'ap_floor_law', true, 'raster', [13 1 nl], 'scan_ramp', 0.5, 'scan_len', 1e6, 'sP_fix_theta_deg', 2, 'q_geo', [0.9 0.9], ...
                  'descent_T', 1, 'post_wait', [0 1], 'tag', 'sq', 'du_d', 40 * Ts / Rr, 'T', 12 + 1.4 * nl * 27 / 40 + 1, 'kappa_state', true, 'kappa_from_scan', true, 'kappa_pfix', 0.3);
    rs = cell(NS, 1);
    if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
    pctRunOnAll warning('off', 'all');
    parfor q = 1:NS
        o = base;  o.seeds = q;  R = run_t3_xyz_fixed(o);  r = R.runs{1};
        rs{q} = struct('t', r.t, 'kend', r.kend, 'contact', r.contact, 'tsc', r.tsc, 'te', r.rs_tend, 'p', r.p, 'h', r.h, 'aN', r.aN);
    end
    save(cf, 'rs', '-v7.3');
end
best = 1;  bt = -inf;
for q = 1:NS
    r = rs{q};  ok = find(isfinite(r.aN(1:r.kend)) & isfinite(r.h(1:r.kend)));  fin = isfinite(r.te) && ~r.contact;  te = r.t(ok(end));  if isfinite(r.te); te = r.te; end;  k = ok(r.t(ok) >= r.tsc & r.t(ok) <= te);
    fprintf('seed %d: finished %d, contact %d, scan %.2f-%.2f s, min gap %.3f um, x %.1f..%.1f, y %.1f..%.1f um\n', q, fin, r.contact, r.tsc, te, min(r.h(k)) * Rr, min(r.p(k, 1)), max(r.p(k, 1)), min(r.p(k, 2)), max(r.p(k, 2)));
    if te + 1e3 * fin > bt; bt = te + 1e3 * fin;  best = q; end
end
g = hs * Rr;  Rt = 2 * sqrt(rbd * rho);  rgj = rbd + g;  P = [13, -13, 15;  13, -13, g];  np = size(P, 1);  sgn = -1;  yv = -13 + (0:nl - 1);  xs = (13:-0.05:-13)';
for L = 1:nl
    x = sgn * xs;  y = yv(L) * ones(size(x));  P = [P; x, y, local_z(hypot(x, y), rbd, rho, g, Rt, rgj)];  if L < nl; P = [P; x(end), yv(L + 1), g]; end;  sgn = -sgn;
end
fprintf('nominal route: start (%.1f, %.1f, %.1f) um, length %.1f um, %.1f s at 40 um/s; simulated route (seed %d) stops at %.2f s\n', P(1, :), sum(vecnorm(diff(P(np:end, :), 1, 1), 2, 2)), sum(vecnorm(diff(P(np:end, :), 1, 1), 2, 2)) / 40, best, bt - 1e3 * (bt > 1e3));
t = [0; 11; 12 * ones(size(P, 1) - 2, 1)];  r = struct('tsc', 12);
if any(findall(0, 'Type', 'figure') == 1); close(1); end
f = figure(1);  set(f, 'Color', 'w', 'Position', [60 60 1250 860], 'Visible', 'on', 'Name', 'square route');  a = axes(f);  hold(a, 'on');
[Xs, Ys, Zs] = sphere(80);  surf(a, rbd * Xs, rbd * Ys, rbd + rbd * Zs, 'FaceColor', [0.85 0.72 0.45], 'EdgeColor', 'none', 'AmbientStrength', 0.6, 'DiffuseStrength', 0.6, 'SpecularStrength', 0);  camlight(a);  lighting(a, 'gouraud');
patch(a, [-16 16 16 -16], [-16 -16 16 16], [-0.02 -0.02 -0.02 -0.02], [0.74 0.76 0.82], 'EdgeColor', 'none', 'FaceLighting', 'none');
pre = t < r.tsc;  h1 = plot3(a, P(pre, 1), P(pre, 2), P(pre, 3), 'Color', [0.85 0 0], 'LineWidth', 2.2);  h2 = plot3(a, P(~pre, 1), P(~pre, 2), P(~pre, 3), 'Color', [0 0 1], 'LineWidth', 0.8);
h3 = plot3(a, P(1, 1), P(1, 2), P(1, 3), 'o', 'MarkerSize', 11, 'MarkerFaceColor', [0 0.6 0.2], 'MarkerEdgeColor', 'k');  ie = find(~pre, 1);  h4 = plot3(a, P(ie, 1), P(ie, 2), P(ie, 3), 's', 'MarkerSize', 11, 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k');  h5 = plot3(a, P(end, 1), P(end, 2), P(end, 3), '^', 'MarkerSize', 11, 'MarkerFaceColor', [0.95 0.6 0], 'MarkerEdgeColor', 'k');
axis(a, 'equal');  xlim(a, [-16 16]);  ylim(a, [-16 16]);  zlim(a, [-0.2 16]);  view(a, -37.5, 25);  grid(a, 'off');  box(a, 'off');  xlabel(a, ['X (' MU ')']);  ylabel(a, ['Y (' MU ')']);  zlabel(a, ['Z (' MU ')']);  set(a, 'FontSize', 16, 'LineWidth', 1.1);
legend([h3 h1 h4 h2 h5], {'start', 'hold and descent', 'scan start', 'scan', 'end'}, 'Location', 'northoutside', 'Orientation', 'horizontal');  rotate3d(a, 'on');
exportgraphics(f, fullfile(fo, 'fig_1102_square_route.png'), 'Resolution', 130);
S = load(fullfile(fo, 'temp_rho4_report.mat'), 'r');  q = S.r;  ok = find(isfinite(q.aN(1:q.kend)));  ks = ok(q.t(ok) >= q.tsc + 0.5 & q.t(ok) <= q.rs_tend);  p = q.p(ks, :);  tx = q.thx(ks) * pi / 180;  ty = q.thy(ks) * pi / 180;  tx = tx(:);  ty = ty(:);
Wh = [sin(ty) .* cos(tx), -sin(tx), cos(ty) .* cos(tx)];  Vh = [sin(ty) .* sin(tx), cos(tx), cos(ty) .* sin(tx)];  dv = [0; sum(diff(p, 1, 1) .* Vh(2:end, :), 2)];  stepv = conv(double(movmean(dv, 21) > 0.012), ones(61, 1), 'same') > 0;
W = movmean(p - Rr * hs * Wh, 41, 1);  W(stepv | abs(W(:, 1)) > 13 | abs(W(:, 2)) > 13, :) = NaN;
if any(findall(0, 'Type', 'figure') == 2); close(2); end
f2 = figure(2);  set(f2, 'Color', 'w', 'Position', [120 120 1250 860], 'Visible', 'on', 'Name', 'wall recovered along the scan lines');  b = axes(f2);  hold(b, 'on');  plot3(b, W(:, 1), W(:, 2), W(:, 3), 'Color', [0.9 0 0], 'LineWidth', 1.0);
axis(b, 'equal');  xlim(b, [-13 13]);  ylim(b, [-13 13]);  zlim(b, [-0.5 11]);  view(b, -37.5, 22);  grid(b, 'off');  box(b, 'off');  xlabel(b, ['X (' MU ')']);  ylabel(b, ['Y (' MU ')']);  zlabel(b, ['Z (' MU ')']);  set(b, 'FontSize', 16, 'LineWidth', 1.1);  rotate3d(b, 'on');
exportgraphics(f2, fullfile(fo, 'fig_1102_wall_lines.png'), 'Resolution', 130);

function z = local_z(r, rb, rho, g, Rt, rgj)
    % upper side of the centre path of the probe at gap g: sphere, then the fillet arc, then the floor
    z = g * ones(size(r));  a = r < rgj;  z(a) = rb + sqrt(rgj^2 - r(a).^2);  c = r >= rgj & r < Rt;  z(c) = rho - sqrt((rho - g)^2 - (Rt - r(c)).^2);
end
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
