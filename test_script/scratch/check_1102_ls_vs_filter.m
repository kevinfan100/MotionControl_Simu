% STATUS: SCRATCH -- 2026-11-02 (user): plateau of the angle under a circle. Offline least squares with the true data, no filter: in the true local frame, step of w against the force steps, d w = a_uw F_u + a_vw F_v + a_33 F_w + noise, 4-10 s, best delay of the force chosen by fit. True a_uw, a_vw = 0, so the fitted values / g are what the data say about the angle, seed by seed. Against the filter: theta-hat error over 8-10 s. Flat wall true (8,-8), circle 3 um, 8 Hz and 13 Hz, seeds 1-3.
%   If least squares is far closer to 0 than the filter, the filter loses the information or is biased; if it gives the same seed values, the data carry that error and sqrt P is too small. Sanity: fitted a_33 near .833. Cached in temp_ls_filter.mat.
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');  Rr = 2.25;  Ts = 1 / 1612;  ad = 0.07;  NS = 3;  Tx = 8;  Ty = -8;  FQ = [8 13];  hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  cf = fullfile(fo, 'temp_ls_filter.mat');
base = struct('theta_x', Tx, 'theta_y', Ty, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'doc_timing', true, 'theta_out', true, 'gap_gate', 2, 'gate_one', true, 'hold_until', 10, 'tau_a', 0.06, 'a_d', ad, 'descent_v', 15 - hs * Rr, 'descent_T', 1, 'post_wait', [0 1], 'ap_q', true, 'ap_floor_law', true, 'sP_fix_theta_deg', 2, 'q_geo', [0.9 0.9], 'tag', 'ls', 'du_d', 40 * Ts / Rr, 'kappa_state', true, 'kappa_from_scan', true, 'kappa_pfix', 0.3, 'scan_ramp', 0.5, 'scan_len', 1e6, 'T', 10, 'dither_win', [0 10]);
[IA, ID] = ndgrid(1:2, 1:NS);  N = numel(IA);
if isfile(cf)
    load(cf, 'rs');
else
    rs = cell(N, 1);
    if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
    pctRunOnAll warning('off', 'all');
    parfor q = 1:N
        o = base;  o.seeds = ID(q);  o.dither = [3 FQ(IA(q)) 2];  R = run_t3_xyz_fixed(o);  r = R.runs{1};
        rs{q} = struct('t', r.t(:), 'thx', r.thx(:), 'thy', r.thy(:), 'p', r.p, 'fa', r.fa, 'g', r.dgap(:), 'aT', r.aT(:), 'aN', r.aN(:), 'aT_tr', r.aT_tr(:), 'aN_tr', r.aN_tr(:), 'sP', r.sPth);
    end
    save(cf, 'rs', '-v7.3');
end
cx = cosd(Tx);  sx = sind(Tx);  cy = cosd(Ty);  sy = sind(Ty);  V = [cy 0 sy; 0 1 0; -sy 0 cy] * [1 0 0; 0 cx -sx; 0 sx cx];
fprintf('Hz seed | filter err x y (8-10 s) | LS err from a_vw, a_uw | LS sd | delay | fit a_33, a_11 | g true | g-hat err, a11 err, a33 err | sqrt P\n');
for q = 1:N
    r = rs{q};  n = find(isfinite(r.thx) & all(isfinite(r.p), 2), 1, 'last');  n = min(n, size(r.fa, 1)) - 6;  w = find(r.t(1:n) >= 4);  dp = (r.p(w + 1, :) - r.p(w, :)) / Rr * V;  best = inf;
    for d = 0:4
        F = r.fa(w - d, :) * V;  Xw = F(:, [1 2 3]);  b = Xw \ dp(:, 3);  e = dp(:, 3) - Xw * b;  if sum(e.^2) < best; best = sum(e.^2);  bd = d;  bw = b;  ew = e;  covb = inv(Xw' * Xw) * var(e); end
    end
    F = r.fa(w - bd, :) * V;  bu = F \ dp(:, 1);  gt = mean(r.aT_tr(w) - r.aN_tr(w));  m = r.t >= 8 & r.t < 10;
    ex = bw(2) / gt * 180 / pi;  ey = -bw(1) / gt * 180 / pi;  fprintf('%2d %d | %+.2f %+.2f | %+.2f %+.2f | %.2f %.2f | %d | %.3f %.3f | %.3f | %+.3f %+.3f %+.3f | %.2f\n', FQ(IA(q)), ID(q), mean(r.thx(m)) - Tx, mean(r.thy(m)) - Ty, ex, ey, sqrt(covb(2, 2)) / gt * 180 / pi, sqrt(covb(1, 1)) / gt * 180 / pi, bd, bw(3), bu(1), gt, mean(r.g(m) - (r.aT_tr(m) - r.aN_tr(m))), mean(r.aT(m) - r.aT_tr(m)), mean(r.aN(m) - r.aN_tr(m)), mean(r.sP(m, 1)));
end
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
