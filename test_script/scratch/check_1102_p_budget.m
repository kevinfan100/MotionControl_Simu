% STATUS: SCRATCH -- 2026-11-02 (user): sum of F_t^2 is 70 times larger with a 3 um circle but the angle error and sqrt P do not fall (run_1102_hold_dither.m). Budget of P(a_uw) over 2-6 s, seeds 1-3, flat wall true (8,-8): log change in the update (Pu / Pk) and in the prediction (Pk[k+1] / Pu). Plain regression from theory: info of a_uw per step = F_u^2 / sigma^2.
%   Sanity: no motion, update and prediction terms sum to the slope of sqrt P. No figure.
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
Rr = 2.25;  Ts = 1 / 1612;  ad = 0.07;  hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  AMP = [0 3];  NS = 3;
base = struct('theta_x', 8, 'theta_y', -8, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'doc_timing', true, 'theta_out', true, 'gap_gate', 2, 'gate_one', true, 'hold_until', 10, 'tau_a', 0.06, 'a_d', ad, 'descent_v', 15 - hs * Rr, 'descent_T', 1, 'post_wait', [0 1], 'ap_q', true, 'ap_floor_law', true, 'sP_fix_theta_deg', 2, 'q_geo', [0.9 0.9], 'tag', 'pb', 'du_d', 40 * Ts / Rr, 'kappa_state', true, 'kappa_from_scan', true, 'kappa_pfix', 0.3, 'scan_ramp', 0.5, 'scan_len', 1e6, 'T', 6);
[IA, ID] = ndgrid(1:2, 1:NS);  N = numel(IA);  rs = cell(N, 1);
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
pctRunOnAll warning('off', 'all');
parfor q = 1:N
    o = base;  o.seeds = ID(q);  if AMP(IA(q)) > 0; o.dither = [AMP(IA(q)) 8 2];  o.dither_win = [0 10]; end;  R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t(:), 'P', r.P1112, 'sP', r.sPth, 'g', r.dgap(:), 'thx', r.thx(:), 'vF', r.vF, 'vMh', r.vMh, 'tsc', r.tsc);
end
for a = 1:2
    for s = 1:NS
        r = rs{IA == a & ID == s};  m = find(r.t >= 2 & r.t < 6 & isfinite(r.P(:, 4)));  Pu = r.P(m, 4);  gk = abs(r.g(m));  cx = cosd(r.thx(m));  Pk1 = (r.sP(m, 2) .* cx .* (pi / 180) .* gk).^2;  % predicted P(12,12)[k+1]
        Pu1 = Pu(2:end);  Pk = Pk1(1:end - 1);  up = sum(log(Pu1 ./ Pk));  pr = sum(log(Pk(2:end) ./ Pu1(1:end - 1)));
        fprintf('dither %g seed %d: tsc %.1f | sqrt P_off start %.5f end %.5f | total log change update %+.3f prediction %+.3f | rms F_u %.4f F_v %.4f F_w %.4f, rms M_u %.4f M_v %.4f M_w %.4f [R/step]\n', AMP(a), s, r.tsc, sqrt(Pu(1)), sqrt(Pu(end)), up, pr, rms(r.vF(m, 1)), rms(r.vF(m, 2)), rms(r.vF(m, 3)), rms(r.vMh(m, 1)), rms(r.vMh(m, 2)), rms(r.vMh(m, 3)));
    end
end
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
