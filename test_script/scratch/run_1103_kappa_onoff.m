% STATUS: SCRATCH -- 2026-11-03 (user): is the curvature slot useful on the bead. rho 4 bead, a_d .07, 40 um/s, current flow (scan right after the descent), fixed 36 um raster, seeds 1-3. A with kappa-hat (current) | B no kappa slot. Angle Q geometric .9 in both.
%   Pre-registered: B ring error same or lower -> kappa-hat not useful; clearly higher -> curvature information needed. Report by region (floor, ring, bead), finished, min gap. Figure in Preview. Output fig_1103_kappa_onoff.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');  Rr = 2.25;  Ts = 1 / 1612;  ad = 0.07;  NS = 3;  hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 0.06, 'a_d', ad, 'descent_v', 15 - hs * Rr, 'descent_T', 1, 'post_wait', [0 0], 'ap_q', true, 'wall', 'fillet', 'bead_r', 5, 'fillet_rho', 4, 'ap_floor_law', true, 'scan_ramp', 0.5, 'scan_len', 1e6, 'sP_fix_theta_deg', 2, 'q_geo', [0.9 0.9], 'tag', 'ko', 'du_d', 40 * Ts / Rr, 'kappa_state', true, 'kappa_from_scan', true, 'kappa_pfix', 0.3, 'raster_local', [36 1 27 0.05], 'bead_start', [18; -13], 'T', 11 + 1.2 * 27 * 36 / 40 + 1);
[IA, ID] = ndgrid(1:2, 1:NS);  N = numel(IA);  rs = cell(N, 1);  nm = {'A with kappa-hat', 'B no kappa slot'};
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end;  pctRunOnAll warning('off', 'all');
parfor q = 1:N
    o = base;  o.seeds = ID(q);  if IA(q) == 2; o = rmfield(o, {'kappa_state', 'kappa_from_scan', 'kappa_pfix'}); end;  R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t(:), 'kend', r.kend, 'contact', r.contact, 'tsc', r.tsc, 'te', r.rs_tend, 'p', r.p, 'h', r.h(:), 'aN', r.aN(:), 'aN_tr', r.aN_tr(:), 'thx', r.thx(:), 'thy', r.thy(:), 'th_tr', r.th_tr);
end
f = figure('Color', 'w', 'Position', [40 40 1700 560], 'Visible', 'off');  b = axes(f);  hold(b, 'on');  cc = [0 0 0; 0.85 0 0];  hl = gobjects(2, 1);
for a = 1:2
    Ang = [];  Al = [];  Rt = [];  RR = [];  fin = 0;  mg = [];  et = [];
    for s = find(IA == a)'
        r = rs{s};  ok = find(isfinite(r.aN(1:r.kend)) & isfinite(r.thx(1:r.kend)));  ke = ok(end);  te = r.t(ke);  if isfinite(r.te); te = min(te, r.te); end;  fn1 = isfinite(r.te) && ~r.contact && r.t(ke) >= r.te;  fin = fin + fn1;  et = [et; te];
        k = ok(r.t(ok) >= r.tsc + 0.5 & r.t(ok) <= te);  k = k(k > 2);  aw = local_w(r.thx(k), r.thy(k));  bw = local_w(r.th_tr(k, 1), r.th_tr(k, 2));  b0 = local_w(r.th_tr(k - 1, 1), r.th_tr(k - 1, 2));  db = movmean(bw - b0, 11, 1);  nn = vecnorm(db, 2, 2);  d = db ./ max(nn, 1e-9);
        Ang = [Ang; acosd(min(sum(aw .* bw, 2), 1))];  Al = [Al; asind(max(min(sum((aw - bw) .* d, 2), 1), -1))];  Rt = [Rt; asind(min(nn, 1))];  RR = [RR; hypot(r.p(k, 1), r.p(k, 2))];  mg = [mg; 1e3 * (min(r.h(k)) * Rr - Rr)];
        if ID(s) == 1; kk = k(1:4:end);  e1 = acosd(min(sum(local_w(r.thx(kk), r.thy(kk)) .* local_w(r.th_tr(kk, 1), r.th_tr(kk, 2)), 2), 1));  hl(a) = plot(b, r.t(kk), movmean(e1, 21), 'Color', cc(a, :), 'LineWidth', 1); end
    end
    m1 = RR > 10;  m2 = RR > 7.2 & RR < 9;  m3 = RR < 7.2;  mt = m2 & Rt >= 0.2 & Rt < 1;
    fprintf('%-17s finished %d/%d, ends %s | floor med %.2f 95%% %.2f | ring med %.2f 95%% %.2f, along %+.2f | bead med %.2f 95%% %.2f | min gap - contact %s nm\n', nm{a}, fin, NS, mat2str(round(et', 1)), median(Ang(m1)), prctile(Ang(m1), 95), median(Ang(m2)), prctile(Ang(m2), 95), mean(Al(mt)), median(Ang(m3)), prctile(Ang(m3), 95), mat2str(round(mg', 1)));
end
ylim(b, [0 20]);  xlim(b, [11 37]);  xlabel(b, 'Time (sec)');  ylabel(b, ['angle error of w' char(770) ', seed 1  [deg]']);  box(b, 'on');  set(b, 'FontSize', 14, 'FontWeight', 'bold', 'LineWidth', 1.2);  legend(b, hl, {'with \kappa', 'no \kappa slot'}, 'Location', 'northoutside', 'Orientation', 'horizontal');
fn = fullfile(fo, 'fig_1103_kappa_onoff.png');  exportgraphics(f, fn, 'Resolution', 130);  close(f);  system(['open -a Preview "' fn '"']);
function w = local_w(thx, thy)
    tx = thx(:) * pi / 180;  ty = thy(:) * pi / 180;  w = [sin(ty) .* cos(tx), -sin(tx), cos(ty) .* cos(tx)];
end
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
