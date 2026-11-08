% STATUS: SCRATCH -- 2026-11-01 (user): first test of the geometric angle Q (opts.q_geo [dkappa_n dtau] 1/R), PI + tau_a .25, 40 um/s, 3 seeds. A rho 10, 0.9 | B rho 4, 0.9 | C rho 4, 1.8. Range from the target wall (+0.30 to -1.51 /R), independent of the test wall.
%   Pre-registered: rho 4 finishes > 0/3, rho 10 no worse than theta 4.7 / 4.3 deg and a' outside .02 of the hand setting.
%   Console per arm: finished seeds, end times, min gap, |a33 - a_d|, est - true, a'/true outside, angle error rms. Figure fig_1101_bead_qgeo.png (seed with the longest run per arm: 3-D path, a_33, theta_x).
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.1;  Ts = 1 / 1612;  Xe = 13;  dy = 1;  nl = 27;  FS = 13;  cT = [0.85 0 0];  cE = [0 0.25 0.85];  NS = 3;
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, ...
              'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 0.5, 'a_d', ad, ...
              'ap_q', true, 'wall', 'fillet', 'bead_r', 5, 'fillet_rho', 10, 'bead_start', [Xe; -Xe], 'ap_floor_law', true, 'raster_keep_y', true, ...
              'raster', [Xe dy nl], 'scan_ramp', 0.5, 'scan_len', 1e6, 'sP_fix_theta_deg', 2, 'descent_T', 1, 'descent_v', 15 - hs * Rr, 'post_wait', [0 1], 'tag', 'qg', ...
              'du_d', 40 * Ts / Rr, 'T', 12 + 1.4 * nl * (2 * Xe + dy) / 40 + 1, 'kappa_state', true, 'kappa_from_scan', true, 'kappa_pfix', 0.3);
arm = {'rho 10, Q geo 0.9', 'rho 4, Q geo 0.9', 'rho 4, Q geo 1.8'};  avd = [0.1 0.1 0.1];  px = [NaN NaN NaN];  rv = [10 4 4];  ta = [0.25 0.25 0.25];  cw = [0 0 0];  sp = [40 40 40];  brd = [5 5 5];  nsa = [3 3 3];  qg = [0.9 0.9 1.8];  NA = numel(arm);  [IA, ID] = ndgrid(1:NA, 1:NS);  rs = cell(size(IA));  rn = ID <= nsa(IA);
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
pctRunOnAll warning('off', 'all');
parfor q = 1:numel(IA)
    if ~rn(q); continue; end
    o = base;  o.seeds = ID(q);  o.q_geo = qg(IA(q)) * [1 1];  o.bead_r = brd(IA(q));  o.du_d = sp(IA(q)) * Ts / Rr;  o.T = 12 + 1.4 * nl * (2 * Xe + dy) / sp(IA(q)) + 1;  o.fillet_rho = rv(IA(q));  o.tau_a = ta(IA(q));  if isfinite(px(IA(q))); o.sP_fix_thx_deg = px(IA(q)); end;  if cw(IA(q)); o.cheat_w = true; end;  o.a_d = avd(IA(q));  o.descent_v = 15 - fzero(@(h) 1 / local_cp(h) - o.a_d, [1.001 30]) * Rr;
    R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t, 'kend', r.kend, 'contact', r.contact, 'tsc', r.tsc, 'te', r.rs_tend, 'p', r.p, 'h', r.h, 'aN', r.aN, 'aN_tr', r.aN_tr, 'ap', r.apN, 'thx', r.thx, 'thy', r.thy, 'th_tr', r.th_tr);
end
save(fullfile(fo, 'temp_bead_qgeo.mat'), 'rs', 'arm', 'Rr', 'ad', 'hs', '-v7.3');
f = figure('Color', 'w', 'Position', [20 20 500 * NA 1000]);  tl = tiledlayout(3, NA, 'TileSpacing', 'compact', 'Padding', 'compact');  [Xs, Ys, Zs] = sphere(40);
for ia = 1:NA
    M = NaN(NS, 9);  best = 1;  bt = -inf;
    for id = 1:nsa(ia)
        r = rs{ia, id};  ok = find(isfinite(r.aN(1:r.kend)) & isfinite(r.h(1:r.kend)) & isfinite(r.thx(1:r.kend)));  ke = ok(end);  fin = isfinite(r.te) && ~r.contact && ke == r.kend;  tend = r.t(ke);  if isfinite(r.te); tend = r.te; end
        g = ok(r.t(ok) >= r.tsc + 0.5 & r.t(ok) <= tend);  sl = local_slope(r.h(g(1:20:end)));  rt = r.ap(g(1:20:end)) ./ sl;
        M(id, :) = [fin, tend, min(r.h(g)) * Rr, mean(abs(r.aN_tr(g) - avd(ia))) / avd(ia), mean(r.aN(g) - r.aN_tr(g)), mean(rt < 0.1 | rt > 1.9), sqrt(mean((r.thx(g) - r.th_tr(g, 1)).^2)), sqrt(mean((r.thy(g) - r.th_tr(g, 2)).^2)), mean(rt < 0)];
        if tend > bt; bt = tend;  best = id; end
    end
    fprintf('%-24s finished %d/%d | end %s | min gap %s | |a33-a_d|/a_d %.3f | est-true %+.3f | a'' outside %.2f, <0 %.2f | angle rms x %.1f, y %.1f deg\n', arm{ia}, sum(M(:, 1)), nsa(ia), mat2str(round(M(1:nsa(ia), 2)', 1)), mat2str(round(M(1:nsa(ia), 3)', 2)), median(M(:, 4), 'omitnan'), median(M(:, 5), 'omitnan'), median(M(:, 6), 'omitnan'), median(M(:, 9), 'omitnan'), median(M(:, 7), 'omitnan'), median(M(:, 8), 'omitnan'));
    r = rs{ia, best};  ok = find(isfinite(r.aN(1:r.kend)) & isfinite(r.h(1:r.kend)) & all(isfinite(r.p(1:r.kend, :)), 2));  k = ok(r.t(ok) >= r.tsc);  if isfinite(r.te); k = k(r.t(k) <= r.te); end;  t = r.t(k(1:6:end));  kd = k(1:6:end);
    a1 = nexttile(tl, ia);  hold on;  box on;  surf(brd(ia) * Xs, brd(ia) * Ys, brd(ia) * Zs + brd(ia), 'FaceColor', [0.75 0.75 0.75], 'EdgeColor', 'none', 'FaceAlpha', 0.3);  plot3(r.p(k, 1), r.p(k, 2), r.p(k, 3), 'Color', cE, 'LineWidth', 0.7);  axis equal;  xlim([-15 15]);  ylim([-15 15]);  zlim([0 20]);  view(-35, 30);  xlabel('x  [\mum]');  ylabel('y  [\mum]');  zlabel('z  [\mum]');  legend({'', sprintf('%s, seed %d', arm{ia}, best)}, 'Location', 'northoutside');
    a2 = nexttile(tl, NA + ia);  hold on;  box on;  plot(t, r.aN_tr(kd), 'Color', cT, 'LineWidth', 1.1);  plot(t, r.aN(kd), 'Color', cE, 'LineWidth', 0.7);  yline(avd(ia), 'k--');  ylim([0 0.3]);  ylabel('a_{33} / a_{nom}');  xlim([t(1) t(end)]);
    a3 = nexttile(tl, 2 * NA + ia);  hold on;  box on;  plot(t, r.th_tr(kd, 1), 'Color', cT, 'LineWidth', 1.1);  plot(t, r.thx(kd), 'Color', cE, 'LineWidth', 0.7);  ylim([-90 90]);  ylabel('\theta_x  [deg]');  xlabel('Time (sec)');  xlim([t(1) t(end)]);
    for a = [a1 a2 a3]; set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.1);  set(a.Toolbar, 'Visible', 'off'); end
end
fn = fullfile(fo, 'fig_1101_bead_qgeo.png');  exportgraphics(f, fn, 'Resolution', 110);  close(f);

function s = local_slope(hb)
    s = arrayfun(@(h) (1 / local_cp(max(h, 1.0003) + 1e-4) - 1 / local_cp(max(h, 1.0003) - 1e-4)) / 2e-4, hb);
end
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
