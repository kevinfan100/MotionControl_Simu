% STATUS: SCRATCH -- 2026-10-29/30 (user) [arm set by arms_in; 10-30: (D) fillet rho 10 + kappa allowed to change, opts.kappa_pfix] whole-bead raster on the 'fillet' wall still fails at a_d 0.1 / kappa off / raster_keep_y (contact on
%   the downslope bead flank -> fillet at y ~ -7, angle estimate lagging; two seeds end with NaN). Three single-knob options, seeds 1-3:
%   (A) 5 um/s | (B) fillet radius 10 um | (C) angle P_fix 4 deg. Everything else as run_1029_fillet_raster.m (a_d 0.1, kappa off, keep y).
%   Console: contact / NaN / how many raster lines finished, gap range, mean |a33 - a_d|. Saves temp_fillet_opts.mat.
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.1;  Ts = 1 / 1612;  Xe = 13;  dy = 1;  nl = 27;
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, ...
              'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 0.5, 'a_d', ad, ...
              'ap_q', true, 'wall', 'fillet', 'bead_r', 5, 'fillet_rho', 6, 'bead_start', [Xe; -Xe], 'ap_floor_law', true, 'raster_keep_y', true, ...
              'raster', [Xe dy nl], 'scan_ramp', 0.5, 'scan_len', 1e6, 'sP_fix_theta_deg', 2, 'descent_T', 1, 'descent_v', 15 - hs * Rr, 'post_wait', [0 1], 'tag', 'fo');
arm = {'(A) 5 um/s', '(B) fillet rho 10 um', '(C) angle P_fix 4 deg', '(D) rho 10, kappa can change', '(E) as D at 40 um/s'};  SP = [5 10 10 10 40];
AS = 1:3;  if exist('arms_in', 'var'); AS = arms_in; end;  sfx = '';  if exist('sfx_in', 'var'); sfx = sfx_in; end;  NAr = numel(AS);
[IA, ID] = ndgrid(AS, 1:3);  rs = cell(size(IA));
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
parfor q = 1:numel(IA)
    o = base;  o.seeds = ID(q);  ia = IA(q);  o.du_d = SP(ia) * Ts / Rr;  o.T = 12 + 1.4 * nl * (2 * Xe + dy) / SP(ia) + 1;
    if ia == 2 || ia >= 4; o.fillet_rho = 10; end
    if ia >= 4; o.kappa_state = true;  o.kappa_from_scan = true;  o.kappa_pfix = 0.3; end
    if ia == 3; o.sP_fix_theta_deg = 4; end
    R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t, 'kend', r.kend, 'contact', r.contact, 'p', r.p, 'h', r.h, 'aN', r.aN, 'aN_tr', r.aN_tr, 'thx', r.thx, 'thy', r.thy, 'th_tr', r.th_tr, 'tsc', r.tsc, 'rs_tend', r.rs_tend);
end
for ja = 1:NAr
    ia = AS(ja);
    for id = 1:3
        r = rs{ja, id};  kk = (1:r.kend)';  ok = kk(isfinite(r.aN(kk)) & isfinite(r.h(kk)));  kn = find(~isfinite(r.aN(kk)), 1);  ks = ok(r.t(ok) >= r.tsc);
        ylast = r.p(ok(end), 2);  nline = max(0, round(ylast + Xe) + 1);
        fprintf('%-22s seed %d: contact %d, NaN at %s s, end t %.1f / %.1f s, last y %.1f um (~%d of %d lines) | gap %.2f-%.2f um | mean|a33 - a_d| %.3f\n', arm{ia}, id, r.contact, ...
                num2str(r.t(kn), 4), r.t(ok(end)), r.t(end), ylast, nline, nl, min(r.h(ks)) * Rr, max(r.h(ks)) * Rr, mean(abs(r.aN_tr(ks) - ad)));
    end
end
arm = arm(AS);  save(fullfile(fo, ['temp_fillet_opts' sfx '.mat']), 'rs', 'arm', 'Rr', 'ad', 'hs', '-v7.3');

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
