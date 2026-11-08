% STATUS: SCRATCH -- 2026-11-02 (user): budget of P(a_uw) in the hold and in the descent. check_1102_p_budget.m: with a 3 um circle the update takes log P down by about 25 and the prediction puts it back. [hypothesis] the curvature slot: Fe(12,18) = -g M_u with P(18,18) at the prior feeds the off-diagonal P every step of tangential motion. Rival: another row of Fe. Split: kappa_state off, flat wall true (8,-8), seeds 1-3, arms none and circle 3 um (0-10 s), t 0-11 s.
%   Pre-registered: with kappa off the prediction term falls to near 0, sqrt P falls with the force and the angle error at 10 s is below the no-motion arm. Budget windows 2-10 s and 10-11 s (vertical motion: the descent). No figure.
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
Rr = 2.25;  Ts = 1 / 1612;  ad = 0.07;  hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  NS = 3;  Tx = 8;  Ty = -8;
base = struct('theta_x', Tx, 'theta_y', Ty, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'doc_timing', true, 'theta_out', true, 'gap_gate', 2, 'gate_one', true, 'hold_until', 10, 'tau_a', 0.06, 'a_d', ad, 'descent_v', 15 - hs * Rr, 'descent_T', 1, 'post_wait', [0 1], 'ap_q', true, 'ap_floor_law', true, 'sP_fix_theta_deg', 2, 'q_geo', [0.9 0.9], 'tag', 'p2', 'du_d', 40 * Ts / Rr, 'kappa_state', true, 'kappa_from_scan', true, 'kappa_pfix', 0.3, 'scan_ramp', 0.5, 'scan_len', 1e6, 'T', 11);
ARM = [0 1; 3 1; 0 0; 3 0];  [IA, ID] = ndgrid(1:4, 1:NS);  N = numel(IA);  rs = cell(N, 1);
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
pctRunOnAll warning('off', 'all');
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');  cf = fullfile(fo, 'temp_p_budget2.mat');
if isfile(cf); load(cf, 'rs'); else
parfor q = 1:N
    o = base;  o.seeds = ID(q);  if ARM(IA(q), 1) > 0; o.dither = [ARM(IA(q), 1) 8 2];  o.dither_win = [0 10]; end;  if ~ARM(IA(q), 2); o = rmfield(o, {'kappa_state', 'kappa_from_scan', 'kappa_pfix'}); end;  R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t(:), 'P', r.P1112, 'sP', r.sPth, 'g', r.dgap(:), 'thx', r.thx(:), 'thy', r.thy(:), 'vF', r.vF, 'vMh', r.vMh);
end
save(cf, 'rs', '-v7.3');
end
W = [2 10; 10 11];  wn = {'hold 2-10 s', 'descent 10-11 s'};
for a = 1:4
    fprintf('circle %g um, kappa %d\n', ARM(a, 1), ARM(a, 2));  e10 = [];  e11 = [];
    for s = 1:NS
        r = rs{IA == a & ID == s};  i2 = find(isfinite(r.thx), 1, 'last');  i1 = min(find(r.t >= 9.99, 1), i2);  e10 = [e10; r.thx(i1) - Tx; r.thy(i1) - Ty];  e11 = [e11; r.thx(i2) - Tx; r.thy(i2) - Ty];
        for w = 1:2
            nn = min([numel(r.t), size(r.P, 1), size(r.sP, 1), numel(r.g), numel(r.thx), size(r.vF, 1)]);  m = find(r.t(1:nn) >= W(w, 1) & r.t(1:nn) < W(w, 2) & isfinite(r.P(1:nn, 4)) & isfinite(r.sP(1:nn, 2)));  if numel(m) < 5; continue; end;  Pu = r.P(m, 4);  Pk1 = (r.sP(m, 2) .* cosd(r.thx(m)) * pi / 180 .* abs(r.g(m))).^2;  Pu1 = Pu(2:end);  Pk = Pk1(1:end - 1);  up = sum(log(Pu1 ./ Pk));  pr = sum(log(Pk(2:end) ./ Pu1(1:end - 1)));
            fprintf('  seed %d %-16s sqrt P_off %.5f -> %.5f | log update %+8.3f, prediction %+8.3f | rms F_u %.4f F_w %.4f, M_u %.5f M_w %.5f\n', s, wn{w}, sqrt(Pu(1)), sqrt(Pu(end)), up, pr, rms(r.vF(m, 1)), rms(r.vF(m, 3)), rms(r.vMh(m, 1)), rms(r.vMh(m, 3)));
        end
    end
    fprintf('  rms angle error at 10 s %.2f deg, at end %.2f deg\n', rms(e10), rms(e11));
end
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
