% STATUS: SCRATCH -- 2026-11-02 (user): is the angle residual in the innovation and ignored. Circle 3 um 8 Hz, flat wall true (8,-8), seeds 1-3, 5-10 s. Innovation of y1 in the frame of V-hat against the force in that frame, delays 0-6 steps: slope b = sum(nu_w F_u) / sum(F_u^2). True off-diagonal error e = true a_uw - estimate in that frame (off_tr, near constant). Pre-registered: b matches e with |t| large = the residual is in the innovation and the filter leaves it; b near 0 with e not 0 = the innovation does not carry it (something else explains the motion). Same for the u row (F_w), and for a_vw. No figure.
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
Rr = 2.25;  Ts = 1 / 1612;  ad = 0.07;  NS = 3;  hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
base = struct('theta_x', 8, 'theta_y', -8, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'doc_timing', true, 'theta_out', true, 'gap_gate', 2, 'gate_one', true, 'hold_until', 10, 'tau_a', 0.06, 'a_d', ad, 'descent_v', 15 - hs * Rr, 'descent_T', 1, 'post_wait', [0 1], 'ap_q', true, 'ap_floor_law', true, 'sP_fix_theta_deg', 2, 'q_geo', [0.9 0.9], 'tag', 'ir', 'du_d', 40 * Ts / Rr, 'kappa_state', true, 'kappa_from_scan', true, 'kappa_pfix', 0.3, 'scan_ramp', 0.5, 'scan_len', 1e6, 'T', 10, 'dither', [3 8 2], 'dither_win', [0 10]);
rs = cell(NS, 1);  if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end;  pctRunOnAll warning('off', 'all');
parfor q = 1:NS
    o = base;  o.seeds = q;  R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t(:), 'nu', r.nu1, 'vF', r.vF, 'off_tr', r.off_tr, 'thx', r.thx(:), 'thy', r.thy(:));
end
for q = 1:NS
    r = rs{q};  n = min([find(isfinite(r.thx), 1, 'last'), size(r.nu, 1), size(r.vF, 1)]) - 8;  w = find(r.t(1:n) >= 5);  e = mean(r.off_tr(w, :), 1);  fprintf('seed %d: true off-diagonal left in the frame of V-hat: a_uw %+.5f, a_vw %+.5f | theta error %+.2f %+.2f deg | rms nu_w %.5f, rms F_u %.4f\n', q, e(1), e(2), mean(r.thx(w)) - 8, mean(r.thy(w)) + 8, rms(r.nu(w, 3)), rms(r.vF(w, 1)));
    for d = 0:6
        for pr = 1:2
            if pr == 1; y = r.nu(w + d, 3);  F = [r.vF(w, 1), r.vF(w, 2)];  lab = 'w on F_u, F_v'; else; y = r.nu(w + d, 1);  F = [r.vF(w, 3), zeros(numel(w), 1)];  lab = 'u on F_w'; end
            if pr == 1; b = F \ y;  se = sqrt(var(y - F * b) * diag(inv(F' * F))); else; b = [F(:, 1) \ y; 0];  se = [sqrt(var(y - F(:, 1) * b(1)) / sum(F(:, 1).^2)); 0]; end
            if d <= 3; fprintf('   delay %d %-14s slope %+.5f (se %.5f), %+.5f (se %.5f)\n', d, lab, b(1), se(1), b(2), se(2)); end
        end
    end
end
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
