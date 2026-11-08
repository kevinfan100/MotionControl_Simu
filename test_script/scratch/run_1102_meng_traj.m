% STATUS: SCRATCH -- 2026-11-02 (user): Meng diss. 6.2 raster (u triangular wave of fixed path length, v ramp at the edges, y free, w from the regulation; opts.raster_local) on the rho 4 bead at 40 um/s, a_d .05 and .1, 3 seeds, PI, tau_a .06, geometric Q 0.9. L 36 um, start x 18 um, 27 lines, v 1 um per line, dv .05 um.
%   Pre-registered: more finished than the fixed-y raster (a_d .1 2/3, a_d .05 0/5). Sanity: u amplitude constant, v in steps, y moves within a line. Figure fig_1102_meng_traj.png (longest seed per arm): 3-D, u, v, y.
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  Ts = 1 / 1612;  nl = 27;  L0 = 36;  rbd = 5;  rho = 4;  FS = 13;  cE = [0 0.25 0.85];  cT = [0.85 0 0];  MU = [char(956) 'm'];  AB = char(257);  NS = 3;  avd = [0.05 0.1];  NA = 2;  [IA, ID] = ndgrid(1:NA, 1:NS);  rs = cell(size(IA));
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 0.06, ...
              'ap_q', true, 'wall', 'fillet', 'bead_r', rbd, 'fillet_rho', rho, 'bead_start', [18; -13], 'ap_floor_law', true, 'raster_local', [L0 1 nl 0.05], 'scan_ramp', 0.5, 'scan_len', 1e6, 'sP_fix_theta_deg', 2, 'q_geo', [0.9 0.9], ...
              'descent_T', 1, 'post_wait', [0 1], 'tag', 'mt', 'du_d', 40 * Ts / Rr, 'T', 12 + 1.2 * nl * L0 / 40 + 1, 'kappa_state', true, 'kappa_from_scan', true, 'kappa_pfix', 0.3);
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
pctRunOnAll warning('off', 'all');
parfor q = 1:numel(IA)
    o = base;  o.seeds = ID(q);  o.a_d = avd(IA(q));  o.descent_v = 15 - fzero(@(h) 1 / local_cp(h) - o.a_d, [1.001 30]) * Rr;  R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t, 'kend', r.kend, 'contact', r.contact, 'tsc', r.tsc, 'te', r.rs_tend, 'p', r.p, 'pd', r.pd, 'h', r.h, 'aN', r.aN, 'aN_tr', r.aN_tr, 'ap', r.apN, 'thx', r.thx, 'thy', r.thy, 'th_tr', r.th_tr);
end
save(fullfile(fo, 'temp_meng_traj.mat'), 'rs', 'avd', 'Rr', '-v7.3');
f = figure('Color', 'w', 'Position', [20 20 1500 1250]);  tl = tiledlayout(4, 2, 'TileSpacing', 'compact', 'Padding', 'compact');  [Xs, Ys, Zs] = sphere(60);
for ia = 1:NA
    M = NaN(NS, 7);  best = 1;  bt = -inf;  ad = avd(ia);
    for id = 1:NS
        r = rs{ia, id};  ok = find(isfinite(r.aN(1:r.kend)) & isfinite(r.h(1:r.kend)) & isfinite(r.thx(1:r.kend)));  ke = ok(end);  fin = isfinite(r.te) && ~r.contact && ke == r.kend;  tend = r.t(ke);  if isfinite(r.te); tend = r.te; end
        g = ok(r.t(ok) >= r.tsc + 0.5 & r.t(ok) <= tend);  M(id, :) = [fin, tend, min(r.h(g)) * Rr, mean(abs(r.aN_tr(g) - ad)) / ad, mean(r.aN(g) - r.aN_tr(g)), sqrt(mean((r.thx(g) - r.th_tr(g, 1)).^2)), sqrt(mean((r.thy(g) - r.th_tr(g, 2)).^2))];
        if tend > bt; bt = tend;  best = id; end
    end
    fprintf('a_d %.2f: finished %d/%d | end %s | min gap %s | |a33-a_d|/a_d %.3f | est-true %+.3f | angle rms x %.1f, y %.1f deg\n', ad, sum(M(:, 1)), NS, mat2str(round(M(:, 2)', 1)), mat2str(round(M(:, 3)', 2)), median(M(:, 4)), median(M(:, 5)), median(M(:, 6)), median(M(:, 7)));
    r = rs{ia, best};  ok = find(isfinite(r.aN(1:r.kend)) & isfinite(r.h(1:r.kend)) & isfinite(r.thx(1:r.kend)));  ke = ok(end);  tend = r.t(ke);  if isfinite(r.te); tend = r.te; end;  k = ok(r.t(ok) >= r.tsc & r.t(ok) <= tend);  kd = k(1:3:end);  t = r.t(kd);
    fprintf('   best seed %d: x range %.1f..%.1f, y range %.1f..%.1f um\n', best, min(r.p(k, 1)), max(r.p(k, 1)), min(r.p(k, 2)), max(r.p(k, 2)));
    nk = ke;  dU = zeros(nk, 2);  for i = 2:nk; c = r.th_tr(i, 1) * pi / 180;  s = r.th_tr(i, 2) * pi / 180;  V = [cos(s) 0 sin(s); 0 1 0; -sin(s) 0 cos(s)] * [1 0 0; 0 cos(c) -sin(c); 0 sin(c) cos(c)];  dU(i, :) = (V(:, 1:2)' * (r.p(i, :) - r.p(i - 1, :))')'; end;  UV = cumsum(dU, 1);
    a1 = nexttile(tl, ia);  hold on;  surf(rbd * Xs, rbd * Ys, rbd + rbd * Zs, 'FaceColor', [0.85 0.72 0.45], 'EdgeColor', 'none', 'AmbientStrength', 0.6, 'DiffuseStrength', 0.6, 'SpecularStrength', 0);  camlight;  lighting gouraud;  plot3(r.p(k, 1), r.p(k, 2), r.p(k, 3), 'Color', [0 0 1], 'LineWidth', 0.7);  axis equal;  view(-35, 25);  xlabel(['X (' MU ')']);  ylabel(['Y (' MU ')']);  zlabel(['Z (' MU ')']);  title(sprintf('%s_d = %.2f, seed %d', AB, ad, best));
    a2 = nexttile(tl, NA + ia);  hold on;  box on;  plot(t, UV(kd, 1), 'Color', cE, 'LineWidth', 1);  ylabel(['u  [' MU ']']);  xlim([t(1) t(end)]);
    a3 = nexttile(tl, 2 * NA + ia);  hold on;  box on;  plot(t, UV(kd, 2), 'Color', cE, 'LineWidth', 1);  ylabel(['v  [' MU ']']);  xlim([t(1) t(end)]);
    a4 = nexttile(tl, 3 * NA + ia);  hold on;  box on;  plot(t, r.p(kd, 2), 'Color', cE, 'LineWidth', 1);  ylabel(['y  [' MU ']']);  xlabel('Time (sec)');  xlim([t(1) t(end)]);
    for a = [a1 a2 a3 a4]; set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.1);  set(a.Toolbar, 'Visible', 'off'); end
end
fn = fullfile(fo, 'fig_1102_meng_traj.png');  exportgraphics(f, fn, 'Resolution', 120);  close(f);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
