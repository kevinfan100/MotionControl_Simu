% STATUS: SCRATCH -- 2026-11-01 (user): rho 4 whole-bead scan at 40 um/s, seed 1, PI, tau_a .06, geometric angle Q 0.9. Three figures. fig_1101_rho4r_scan: measured motion | path with the plain bead and floor (fillet not drawn) | bead part only, blue probe path and red wall points p - h-hat R w-hat drawn as lines (Meng dissertation scan figure).
%   fig_1101_rho4r_full and fig_1101_rho4r_pre (0-12 s): u v w (true local coordinate, Fig. 45/49) | delta u v w | a_33, a_11, gap. Sanity: scan ends 34.7 s, no contact.
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.1;  Ts = 1 / 1612;  Xe = 13;  dy = 1;  nl = 27;  rbd = 5;  rho = 4;  FS = 14;  cT = [0.85 0 0];  cE = [0 0.25 0.85];  AB = char(257);  MU = [char(956) 'm'];  DL = char(948);
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
o = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 0.06, 'a_d', ad, ...
           'ap_q', true, 'wall', 'fillet', 'bead_r', rbd, 'fillet_rho', rho, 'bead_start', [Xe; -Xe], 'ap_floor_law', true, 'raster_keep_y', true, 'raster', [Xe dy nl], 'scan_ramp', 0.5, 'scan_len', 1e6, 'sP_fix_theta_deg', 2, 'q_geo', [0.9 0.9], ...
           'descent_T', 1, 'descent_v', 15 - hs * Rr, 'post_wait', [0 1], 'tag', 'rp', 'du_d', 40 * Ts / Rr, 'T', 12 + 1.4 * nl * (2 * Xe + dy) / 40 + 1, 'kappa_state', true, 'kappa_from_scan', true, 'kappa_pfix', 0.3, 'seeds', 1);
R = run_t3_xyz_fixed(o);  r = R.runs{1};  ok = find(isfinite(r.aN(1:r.kend)));  ke = ok(end);  ks = ok(r.t(ok) >= r.tsc & r.t(ok) <= r.rs_tend);
fprintf('contact %d, scan %.1f-%.1f s, run end %.1f s\n', r.contact, r.tsc, r.rs_tend, r.t(ke));
hg = linspace(1.0005, 40, 6000);  ag = arrayfun(@(h) 1 / local_cp(h), hg);  ah = min(max(r.aN(1:ke), ag(1)), ag(end));  hh = interp1(ag, hg, ah);
tx = r.thx(1:ke) * pi / 180;  ty = r.thy(1:ke) * pi / 180;  wh = [sin(ty) .* cos(tx), -sin(tx), cos(ty) .* cos(tx)];  wp = r.p(1:ke, :) - (hh * Rr) .* wh;
wsm = movmean(wp, 41, 1);  rq = hypot(r.p(1:ke, 1), r.p(1:ke, 2));
pp = r.p(ks, :);  ww = wsm(ks, :);  on = pp(:, 3) > 3.4 & rq(ks) < 10;  pb = pp;  pb(~on, :) = NaN;  wb = ww;  wb(~on, :) = NaN;
fprintf('red wall points on the bead: distance to the true sphere mean %+.3f um, sd %.3f um\n', mean(hypot(hypot(wb(on, 1), wb(on, 2)), wb(on, 3) - rbd) - rbd), std(hypot(hypot(wb(on, 1), wb(on, 2)), wb(on, 3) - rbd) - rbd));
[Xs, Ys, Zs] = sphere(80);  pk = ks(1:2:end);
f = figure('Color', 'w', 'Position', [20 20 2300 760]);  tl = tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
a1 = nexttile(tl);  hold on;  plot3(r.p(pk, 1), r.p(pk, 2), r.p(pk, 3), 'Color', [0 0 1], 'LineWidth', 0.8);
a2 = nexttile(tl);  hold on;  surf(rbd * Xs, rbd * Ys, rbd + rbd * Zs, 'FaceColor', [0.85 0.72 0.45], 'EdgeColor', 'none', 'AmbientStrength', 0.6, 'DiffuseStrength', 0.6, 'SpecularStrength', 0);  camlight;  lighting gouraud;  patch([-15 15 15 -15], [-15 -15 15 15], [-0.02 -0.02 -0.02 -0.02], [0.74 0.76 0.82], 'EdgeColor', 'none', 'FaceLighting', 'none');  plot3(r.p(pk, 1), r.p(pk, 2), r.p(pk, 3), 'Color', [0 0 1], 'LineWidth', 0.8);
a3 = nexttile(tl);  hold on;  plot3(pb(:, 1), pb(:, 2), pb(:, 3), 'Color', [0 0 1], 'LineWidth', 1.6);  plot3(wb(:, 1), wb(:, 2), wb(:, 3), 'Color', [0.9 0 0], 'LineWidth', 1.6);
for a = [a1 a2]; xlim(a, [-15 15]);  ylim(a, [-15 15]);  zlim(a, [-0.2 13]); end;  xlim(a3, [-10 10]);  ylim(a3, [-10 10]);  zlim(a3, [0 13]);
for a = [a1 a2 a3]; axis(a, 'equal');  view(a, -35, 25);  grid(a, 'off');  box(a, 'off');  xlabel(a, ['X (' MU ')']);  ylabel(a, ['Y (' MU ')']);  zlabel(a, ['Z (' MU ')']);  set(a, 'FontSize', 17, 'LineWidth', 1.1);  set(a.Toolbar, 'Visible', 'off'); end
exportgraphics(f, fullfile(fo, 'fig_1101_rho4r_scan.png'), 'Resolution', 130);  close(f);
nk = ke;  E = zeros(nk, 3);  dU = zeros(nk, 2);  D = r.pd(1:nk, :) - r.p(1:nk, :);
for i = 1:nk
    c = r.th_tr(i, 1) * pi / 180;  s = r.th_tr(i, 2) * pi / 180;  V = [cos(s) 0 sin(s); 0 1 0; -sin(s) 0 cos(s)] * [1 0 0; 0 cos(c) -sin(c); 0 sin(c) cos(c)];  E(i, :) = (V' * D(i, :)')';
    if i > 1; dU(i, :) = (V(:, 1:2)' * (r.p(i, :) - r.p(i - 1, :))')'; end
end
Lc = [cumsum(dU), r.h(1:nk) * Rr];  Ld = Lc + E;  t = r.t(1:nk);  NL = {'u', 'v', 'w'};
for w = 1:2
    if w == 1; g = (1:4:nk)'; nmf = 'full'; else; g = find(t < 12);  g = g(1:2:end); nmf = 'pre'; end
    f = figure('Color', 'w', 'Position', [20 20 2300 1000]);  tl = tiledlayout(3, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    for q = 1:3
        a = nexttile(tl);  hold on;  box on;  h1 = plot(t(g), Ld(g, q), 'Color', cT, 'LineWidth', 2.0);  h2 = plot(t(g), Lc(g, q), 'Color', cE, 'LineWidth', 0.9);  ylabel([NL{q} '  [' MU ']']);  xlim([t(g(1)) t(g(end))]);  if q == 1; legend([h1 h2], {'Reference', 'Measurement'}, 'Location', 'northoutside', 'Orientation', 'horizontal'); end;  if q == 3; xlabel('Time (sec)'); end;  set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);  set(a.Toolbar, 'Visible', 'off');
        a = nexttile(tl);  hold on;  box on;  plot(t(g), E(g, q), 'Color', cE, 'LineWidth', 0.6);  ylim([-0.3 0.3]);  ylabel([DL NL{q} '  [' MU ']']);  xlim([t(g(1)) t(g(end))]);  if q == 3; xlabel('Time (sec)'); end;  set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);  set(a.Toolbar, 'Visible', 'off');
        a = nexttile(tl);  hold on;  box on;
        switch q
            case 1; h1 = plot(t(g), r.aN_tr(g), 'Color', cT, 'LineWidth', 1.2);  h2 = plot(t(g), r.aN(g), 'Color', cE, 'LineWidth', 0.7);  h3 = yline(ad, 'k--', 'LineWidth', 1.4);  ylim([0 1]);  ylabel([AB '_{33}']);  legend([h1 h2 h3], {'true', 'estimate', [AB '_d = 0.1']}, 'Location', 'northoutside', 'Orientation', 'horizontal');
            case 2; plot(t(g), r.aT_tr(g), 'Color', cT, 'LineWidth', 1.2);  plot(t(g), r.aT(g), 'Color', cE, 'LineWidth', 0.7);  ylim([0.2 1]);  ylabel([AB '_{11}']);
            case 3; plot(t(g), r.h(g) * Rr, 'Color', cT, 'LineWidth', 1.2);  plot(t(g), hh(g) * Rr, 'Color', cE, 'LineWidth', 0.7);  yline(Rr, 'k-', 'LineWidth', 1.1);  ylim([0 16]);  ylabel(['gap  [' MU ']']);  xlabel('Time (sec)');
        end
        xlim([t(g(1)) t(g(end))]);  set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);  set(a.Toolbar, 'Visible', 'off');
    end
    exportgraphics(f, fullfile(fo, ['fig_1101_rho4r_' nmf '.png']), 'Resolution', 120);  close(f);
end

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
