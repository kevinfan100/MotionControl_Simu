% STATUS: SCRATCH -- 2026-10-20, T2 of the three-axis estimator (user 10-20): the FULL estimator of 1015_estimator_3d_wsonly.tex
%   (a-bar' as a state variable, 15 states, everything estimated) + gain regulation along w-hat + scan along u-hat, on ONE tilted plane wall.
%   Plant: production primitives (calc_thermal_force, step_dynamics, d = 2 sensor delay, sensor noise [0.62 0.57 3.31] nm), wall
%   theta_y = opts.theta_y (20 deg), theta_x = 0, start 22.2 R along the true normal (P.common.p0).
%   Initial estimates from what is known / assumed at the start of an experiment (user 10-20), not from the simulated truth:
%     theta-hat = 0 (flat wall assumed), a-hat_T = a-hat_N = 1 (no wall effect assumed far away), a-hat'_N = (9/8)/h_d[0]^2,
%     a-hat'_T = (9/16)/h_d[0]^2 (far-field leading terms, any wall), dp-hat = 0, h_d = w-hat' p_d (visible surface at 0);
%     priors: theta 30 deg (user), gains (9/8)/h_d[0] (size of the far-field term), a' = its own seed (100 %), dp = sigma_n.
%   Measurements (single sample, a_cov = 1): dp_m (3), a_Tm = ([vA_m]_uu + [vA_m]_vv)/2, a_Nm = [vA_m]_ww, and the angle rows in the
%   undivided form y = [vA_m]_vw, H(theta_x) = (a-hat_T - a-hat_N); y = [vA_m]_uw, H(theta_y) = -(a-hat_T - a-hat_N) cos theta-hat_x
%   (equal to the doc's theta_m rows multiplied by the gain difference; avoids 0/0 when a-hat_T = a-hat_N at the start).
%   R: Sigma_n; IF (a_T + xi)^2 / 2 IF (a_N + xi)^2 / IF (a_T + xi)(a_N + xi), IF from the impulse responses, xi from v Sigma_n.
%   Q: Q33 (Section 7), cross terms with a_T', a_N'; q of a' = p_fix^2 I[k], sqrt p_fix = 0.4 (1.1/max(h_d, 1.1))^2 (1019_report).
%   Regulation (1019 Part III law, 3-D; user 10-20: tau_a 2 s = smallest without contact, no limit on Delta h_d):
%   Delta h_d = (1 - lambda_a)(a_d - a-hat_N)/max(a-hat'_N, 1/22.2^2) along w-hat, lambda_a = 1 - Ts/tau_a (opts.v_max = Inf; the
%   first run used the Part I setting tau_a 0.1 s + v_max, not the agreed one). Hold until t_scan = 7 s (3 tau + 1 s), then + Delta u_d
%   u-hat per step for 2 s.
%   opts.angle_gate (10-20, user option 2, after seed 4 diverged in the far field with theta-hat_y ~75 deg): the angles are updated
%   only while |a_T-hat - a_N-hat| > 3 sqrt(p_aT + p_aN - 2 p_aTaN); otherwise their Kalman gain rows are zero (Joseph form).
%   Contact (true h < 1) or a plant error (one step through the wall) stops the run (r.fail holds the state). Output temp_t2_<tag>.mat
%   opts.wall = 'bend' (10-20 user, stage 1 of the changing-angle test; wall drawn by plot_wall_bend_setup): flat z = pz0 -> smooth bend
%   (tilt alpha ramps 0 -> bend_alpha over bend_Lx um of the scan line, starting bend_xb um ahead of the start) -> tilted plane,
%   downslope azimuth bend_phi from +x (theta_x and theta_y both change). Plant = plane law on the local tangent plane at the probe
%   (P.wall w_hat / V / pz updated every step). The run starts on the flat part (true angles 0). On this wall h_d is integrated from the
%   commanded normal steps (h_d += w-hat' step) instead of w-hat' p_d (the tilted plane does not pass through the origin).
%   opts.q_theta = 'pfix' (10-20 user): q of theta_x, theta_y = P_fix^2 I[k], sqrt P_fix = opts.sP_fix_theta_deg (tuned), I[k] = 1/p - 1/p^f
%   of each angle slot (zero before the gate opens, since the angles get no update then).
%   opts.uv_hold (10-20 user; notation 10-21): desired position in the estimated local coordinate relative to its start,
%   [u_d; v_d; w_d] = V-hat' (p-bar_d - p-bar_d[0]); regulation moves w_d by Delta h_d, and u_d, v_d are pulled toward 0 by the
%   proportional gain opts.uv_gain (g, default 1 = back to 0 in one step): u_d -> (1 - g) u_d. No step limit unless opts.uv_max_um.
%   Source of u_d: Delta u_d = -Delta theta-hat * w_d (the frame rotates, the point stays).
%   10-21 fixed-point test (user): opts.h0 = start height [R] along the true normal, opts.fixed_point = true -> no regulation, p_d stays
%   at the start (position control only); opts.a_init_true = REFERENCE arm (gains / slopes seeded at the truth). Prior: sP_theta_deg.
%   10-21 user: angle prior sqrt P_theta[0] set to 20 deg (was 30; runs before 10-21 store their own opts.sP_theta_deg). Basis: 20 deg
%   wall, no gate, u, v hold, regulation 3 s, 10 seeds: prior 20 / 30 / 45 deg -> 0 / 1 / 2 fail; prior 15: 0 / 0 / 3 fail on 10 / 20 /
%   40 deg walls; q_theta = P_fix^2 I (2 deg) made every wall worse (3-6 / 10) -> q_theta = 0 here.
%   EXPIRES: when a controller file replaces this prototype | run: R = run_t2_3d_reg(struct('a_d', 0.3, 'seeds', 1, 'tag', 'G1'))
function R = run_t2_3d_reg(opts)
    if nargin < 1; opts = struct(); end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));  addpath(here);
    def = struct('seeds', 1, 'theta_y', 20, 'a_d', 0.3, 'tau_a', 2.0, 'v_max', Inf, 't_scan', 7.0, 'T_scan', 2.0, 'du_d', 0.011, ...
                 'lambda_c', 0.4, 'sP_theta_deg', 10, 'tag', 'G1', 'save', true, 'wall', 'plane', 'q_theta', 'none', 'sP_fix_theta_deg', 5, ...
                 'bend_alpha', 20, 'bend_phi', 45, 'bend_Lx', 20, 'bend_xb', 10);
    fn = fieldnames(def);
    for i = 1:numel(fn); if ~isfield(opts, fn{i}); opts.(fn{i}) = def.(fn{i}); end; end
    R = struct('opts', opts, 'runs', {{}});
    for s = opts.seeds
        r = local_run(opts, s);  R.runs{end+1} = r;
        fprintf('%s seed %d: contact %d (t %.3f), min h %.3f, end theta-hat_y %.2f (x %.2f), end a-hat_N %.3f vs %.3f, a-hat_T %.3f vs %.3f\n', ...
                opts.tag, s, r.contact, r.t_contact, min(r.h), r.thy(r.kend), r.thx(r.kend), r.aN(r.kend), r.aN_true(r.kend), r.aT(r.kend), r.aT_true(r.kend));
    end
    if opts.save
        fo = fullfile(root, 'test_results', 'temp_3d_wsonly');  if ~exist(fo, 'dir'); mkdir(fo); end
        save(fullfile(fo, ['temp_t2_' opts.tag '.mat']), 'R');
    end
end

function r = local_run(opts, seed)
    clear calc_thermal_force;
    rng(seed);
    cfg = canonical_scenario(0.05, 1.10, 'ramp_tri', 1);
    cfg.lambda_c = opts.lambda_c;  cfg.theta_y = opts.theta_y;  cfg.theta_x = 0;
    is_bead = strcmp(opts.wall, 'bead');                       % 10-22 user: Meng glass bead on the cover glass, start above the bead top
    is_bump = strcmp(opts.wall, 'bump');                       % 10-22 user: Gaussian bump (cell-like) on the glass, scan over it
    is_bend = strcmp(opts.wall, 'bend') || is_bead || is_bump;  if is_bend; cfg.theta_y = 0; end   % curved walls: start where the wall is flat
    params = calc_simulation_params(cfg);  P = params.Value;
    Ts = P.common.Ts;  Rr = P.common.R;  a_nom = Ts / P.common.gamma_N;  a_o = a_nom / Rr;
    kap = 4 * P.ctrl.k_B * P.ctrl.T * a_o / Rr;  Sn = diag((cfg.meas_noise_std(:) / Rr).^2);
    lc = cfg.lambda_c;  apd = cfg.a_pd;  d = 2;
    [C_T, C_n, cT, cn] = local_coeffs(lc, apd, d);
    w_true = P.wall.w_hat(:);  pz = P.wall.pz;  pz0 = pz;
    lam_a = 1 - Ts / opts.tau_a;  ap_min = 1 / 22.2^2;
    v_max = opts.v_max;                                        % 10-20 user: 1019 Part III law, tau_a 2 s, NO limit on Delta h_d (Inf)
    N = round((opts.t_scan + opts.T_scan) / Ts) + 1;  t = (0:N-1)' * Ts;
    % ---- estimator init (known / assumed only) ----
    p = P.common.p0;  pd_k = p;
    if isfield(opts, 'h0'); p = (pz + opts.h0 * Rr) * w_true;  pd_k = p; end   % 10-21: start at h0 [R] along the true normal
    pd0 = p;  uv_max = Inf;                                    % uv_hold: p-bar_d[0] (start of the desired position); no step limit (user 10-20)
    g_uv = 1;  if isfield(opts, 'uv_gain'); g_uv = opts.uv_gain; end   % uv_hold: proportional gain per step (1 = back to 0 in one step)
    if isfield(opts, 'tau_u'); g_uv = min(Ts / opts.tau_u, 1); end      % 1021_regulation_uv.tex: Delta u_d = -(T_s / tau_u) u_d
    th_c = [0; 0];                                             % 10-21 user: angle used by the command directions (theta_c), = theta-hat[0]
    u_ref = 0;                                                 % uv_hold + scan: target of u_d [R] (0 before t_scan)
    rs = struct('line', 1, 'phase', 'u', 'dir', 1, 's', 0);   % raster state (opts.raster = [Lu um, dv um, lines])
    g_th = 0;  if isfield(opts, 'tau_theta') && opts.tau_theta > 0; g_th = min(Ts / opts.tau_theta, 1); end
    if isfield(opts, 'uv_max_um'); uv_max = opts.uv_max_um / Rr; end
    hd_int = p(3) / Rr;                                        % bend wall: h_d integrated from the commanded normal steps
    x = zeros(15, 1);                                          % [dp1(3) dp2(3) dp3(3) aT aTp aN aNp thx thy]
    hd0 = p(3) / Rr;                                           % w-hat[0] = z: h_d[0] = w-hat' p_d (visible surface at 0)
    x(10) = 1;  x(12) = 1;  x(11) = (9/16) / hd0^2;  x(13) = (9/8) / hd0^2;
    if isfield(opts, 'a_init_true') && opts.a_init_true      % 10-21 REFERENCE arm only: gains and slopes seeded at the truth
        [x(10), x(12), x(11), x(13)] = local_truth((dot(p, w_true) - pz) / Rr);
    end
    sig2 = diag(Sn);
    Pk = diag([repmat(sig2, 3, 1); ((9/8) / hd0)^2; x(11)^2; ((9/8) / hd0)^2; x(13)^2; (opts.sP_theta_deg * pi / 180)^2 * [1; 1]]);
    if isfield(opts, 'fix_theta') && opts.fix_theta            % REFERENCE arm only: angles given (truth), not estimated
        x(14) = 0;  x(15) = opts.theta_y * pi / 180;  Pk(14, 14) = 1e-14;  Pk(15, 15) = 1e-14;
    end
    pbuf = repmat(p, 1, d + 1);  pdbuf = repmat(pd_k, 1, d + 1);  fbuf = zeros(3, d);  Abuf = repmat(eye(3), 1, 1, d);
    dbar = zeros(3, 1);
    L = struct('h', NaN(N,1), 'aT_true', NaN(N,1), 'aN_true', NaN(N,1), 'apT_true', NaN(N,1), 'apN_true', NaN(N,1), ...
               'aT', NaN(N,1), 'aN', NaN(N,1), 'apT', NaN(N,1), 'apN', NaN(N,1), 'thx', NaN(N,1), 'thy', NaN(N,1), ...
               'sP', NaN(N,6), 'p', NaN(N,3), 'pd', NaN(N,3), 'dhd', NaN(N,1), 'u_prog', NaN(N,1), 'aNm', NaN(N,1), 'gate', NaN(N,1), ...
               'thx_true', NaN(N,1), 'thy_true', NaN(N,1), 'hd_used', NaN(N,1), 'line', NaN(N,1), 'nis', NaN(N,2), ...
               'thm', NaN(N,2), 'Kth', NaN(N,2), 'dth_y1', zeros(N,2), 'dth_ro', zeros(N,2), 'dth_dp', zeros(N,2), 'dth_ag', zeros(N,2), ...
               'L8', NaN(N,7), 'L9', NaN(N,7), 'vAm', NaN(N,9), 'thr', NaN(N,2));   % 10-22 estimator feedback rows L_8, L_9 (doc Sec. 4), vA_m   % 10-22 angle readout / actual gain logs
    contact = false;  t_contact = NaN;  kend = N;  u0 = [];
    blk_on = isfield(opts, 'angle_block') && ~strcmp(opts.angle_block, 'none');  latch = false;  t_latch = NaN;
    for k = 1:N
        % ---- truth at k ----
        if is_bump
            [P.wall, thx_t, thy_t] = local_bump_wall(p, P.wall, pz0, opts);  w_true = P.wall.w_hat(:);  pz = P.wall.pz;
        elseif is_bead
            [P.wall, thx_t, thy_t] = local_bead_wall(p, P.wall, pz0, opts);  w_true = P.wall.w_hat(:);  pz = P.wall.pz;
        elseif is_bend
            [P.wall, thx_t, thy_t] = local_bend_wall(p, P.wall, pz0, opts);  w_true = P.wall.w_hat(:);  pz = P.wall.pz;
        else
            thx_t = 0;  thy_t = opts.theta_y;
        end
        h_true = (dot(p, w_true) - pz) / Rr;
        if h_true < 1; contact = true;  t_contact = t(k);  kend = k - 1;  break; end
        [aTt, aNt, apTt, apNt] = local_truth(h_true);
        % ---- readout (measurement coordinate -> local of theta-hat) ----
        pm = pbuf(:, 1);  dpm = (pdbuf(:, 1) - pm) / Rr;
        dbar = apd * dpm + (1 - apd) * dbar;  dpr = dpm - dbar;
        Vh = local_Ry(x(15)) * local_Rx(x(14));  uh = Vh(:, 1);  vh = Vh(:, 2);  wh = Vh(:, 3);
        vSn = Vh' * Sn * Vh;  vd = Vh' * dpr;  vA = (vd * vd' - C_n * vSn) / (C_T * kap);
        aT = max(x(10), 0.02);  aN = max(x(12), 0.02);  dA = x(10) - x(12);
        xi = C_n * diag(vSn) / (C_T * kap);
        rho = @(a, s2) (kap * a * cT + s2 * cn) / (kap * a * cT(1) + s2 * cn(1));
        ru = rho(aT, vSn(1, 1));  rv = rho(aT, vSn(2, 2));  rw = rho(aN, vSn(3, 3));
        IFsq = @(r) 1 + 2 * sum(r(2:end).^2);  IFx = @(r1, r2) 1 + 2 * sum(r1(2:end) .* r2(2:end));
        y  = [dpm; (vA(1,1) + vA(2,2)) / 2; vA(3,3); vA(2,3); vA(1,3)];
        Hm = zeros(7, 15);  Hm(1:3, 1:3) = eye(3);  Hm(4, 10) = 1;  Hm(5, 12) = 1;  Hm(6, 14) = dA;  Hm(7, 15) = -dA * cos(x(14));
        yh = [x(1:3); x(10); x(12); 0; 0];
        Rm =[sig2; 0.5 * (IFsq(ru) * (aT + xi(1))^2 + IFsq(rv) * (aT + xi(2))^2); 2 * IFsq(rw) * (aN + xi(3))^2; ...
              IFx(rv, rw) * (aT + xi(2)) * (aN + xi(3)); IFx(ru, rw) * (aT + xi(1)) * (aN + xi(3))];
        if isfield(opts, 'angle_den') && strcmp(opts.angle_den, 'meas')   % 10-22 user: theta_m with the MEASURED a_Tm - a_Nm in the
            dAm = y(4) - y(5);                                 %   denominator, y / H = [vA_m]_vw / dAm (no guard); R to second order:
            y(6:7) = y(6:7) * dA / dAm;                        %   R_theta,meas = R_theta (1 + (R4 + R5) / dA-hat^2)
            Rm(6:7) = Rm(6:7) * (1 + (Rm(4) + Rm(5)) / max(dA^2, 1e-12));
        end
        pf_ap = [Pk(11, 11); Pk(13, 13)];  pf_th = [Pk(14, 14); Pk(15, 15)];
        % 10-20 (user, option 2): update the angles only while the gain difference is significant, |a_T-hat - a_N-hat| > 3 sigma of
        % that difference (from P); otherwise the Kalman gain rows of theta_x, theta_y are zero for every measurement (y1 path too).
        sd_diff = sqrt(max(Pk(10, 10) + Pk(12, 12) - 2 * Pk(10, 12), 0));
        gate_th = ~isfield(opts, 'angle_gate') || ~opts.angle_gate || abs(x(10) - x(12)) > 3 * sd_diff;
        % 10-20 (user): discrimination arms, LATCHED gate (same 3-sigma condition, once open stays open):
        %   opts.angle_block = 'thm'  -> before opening, the theta_m rows (6, 7) are not used at all
        %                      'y1'   -> before opening, rows 1-5 may not update theta (K(14:15) = 0); theta_m rows used
        %                      'both' -> before opening, theta_m rows not used and rows 1-5 may not update theta
        open_cond = abs(x(10) - x(12)) > 3 * sd_diff;
        if isfield(opts, 'gate_dA'); open_cond = abs(x(10) - x(12)) > opts.gate_dA; end   % 10-20 user: constant threshold (tuned)
        if blk_on && ~latch && open_cond; latch = true;  t_latch = t(k); end
        % 10-22 log: estimator feedback matrix rows L_8 (theta_x), L_9 (theta_y) of 1015_estimator_3d_wsonly Sec. 4 (batch form, equal to the
        % sequential result), measurement columns [dp_m (3) a_Tm a_Nm theta_xm theta_ym]; angle columns rescaled to the theta_m form (x H)
        use = 1:7;  if blk_on && ~latch && any(strcmp(opts.angle_block, {'thm', 'both'})); use = 1:5; end   % 10-22: rows actually used
        Lb = NaN(15, 7);  Lb(:, use) = Pk * Hm(use, :)' / (Hm(use, :) * Pk * Hm(use, :)' + diag(Rm(use)));
        if ~gate_th; Lb(14:15, use) = 0; end
        if blk_on && ~latch && any(strcmp(opts.angle_block, {'y1', 'both'})); Lb(14:15, intersect(use, 1:5)) = 0; end
        L.L8(k, :) = Lb(14, :) .* [1 1 1 1 1 Hm(6, 14) Hm(7, 15)];  L.L9(k, :) = Lb(15, :) .* [1 1 1 1 1 Hm(6, 14) Hm(7, 15)];
        L.vAm(k, :) = vA(:)';  L.thr(k, :) = x(14:15)' * 180 / pi;
        for j = 1:7                                            % sequential scalar updates (R diagonal), Joseph form
            if blk_on && ~latch && j >= 6 && any(strcmp(opts.angle_block, {'thm', 'both'})); continue; end
            hj = Hm(j, :);  S = hj * Pk * hj' + Rm(j);  if S <= 0; continue; end
            if j >= 6; L.nis(k, j - 5) = (y(j) - yh(j))^2 / S; end   % 10-22: normalized innovation of the angle rows
            K = Pk * hj' / S;  if ~gate_th; K(14:15) = 0; end
            if blk_on && ~latch && j <= 5 && any(strcmp(opts.angle_block, {'y1', 'both'})); K(14:15) = 0; end
            dx = K * (y(j) - yh(j));
            if j <= 5; L.dth_y1(k, :) = L.dth_y1(k, :) + dx(14:15)' * 180 / pi; else; L.dth_ro(k, :) = L.dth_ro(k, :) + dx(14:15)' * 180 / pi; end
            if j <= 3; L.dth_dp(k, :) = L.dth_dp(k, :) + dx(14:15)' * 180 / pi; elseif j <= 5; L.dth_ag(k, :) = L.dth_ag(k, :) + dx(14:15)' * 180 / pi; end   % 10-22: split rows 1-3 (dp_m) / 4-5 (a_Tm, a_Nm)
            if j >= 6                                          % 10-22: single-sample angle reading and the ACTUAL gain on it
                L.thm(k, j - 5) = (x(13 + j - 5) + (y(j) - yh(j)) / hj(13 + j - 5)) * 180 / pi;   % theta_m = theta-hat + y / H
                L.Kth(k, j - 5) = K(13 + j - 5) * hj(13 + j - 5);  % fraction of (theta_m - theta-hat) applied this step
            end
            x = x + dx;
            IKH = eye(15) - K * hj;  Pk = IKH * Pk * IKH' + K * Rm(j) * K';
            yh = [x(1:3); x(10); x(12); 0; 0];             % later rows use the updated state (yh of the angle rows stays 0)
        end
        I_ap = max(1 ./ [Pk(11, 11); Pk(13, 13)] - 1 ./ pf_ap, 0);
        I_th = max(1 ./ [Pk(14, 14); Pk(15, 15)] - 1 ./ pf_th, 0);
        % ---- control law with the updated estimates ----
        Vh = local_Ry(x(15)) * local_Rx(x(14));  uh = Vh(:, 1);  vh = Vh(:, 2);  wh = Vh(:, 3);
        aT = max(x(10), 0.02);  aN = max(x(12), 0.02);
        Ab = aT * eye(3) + (aN - aT) * (wh * wh');
        % command directions: theta_c[k+1] = theta_c[k] + (T_s/tau_theta)(theta-hat[k] - theta_c[k]) (opts.tau_theta, 10-21);
        % without tau_theta the directions are those of theta-hat (u_c = u-hat ...). The force (Ab) always uses theta-hat.
        uc = uh;  vc = vh;  wc = wh;
        if g_th > 0
            th_c = th_c + g_th * (x(14:15) - th_c);
            Vc = local_Ry(th_c(2)) * local_Rx(th_c(1));  uc = Vc(:, 1);  vc = Vc(:, 2);  wc = Vc(:, 3);
        end
        L.thc(k, :) = th_c' * 180 / pi;
        % gain regulation along w_c (+ scan along u_c)
        s_ap = max(x(13), ap_min);
        dh = (1 - lam_a) * (opts.a_d - x(12)) / s_ap;  dh = sign(dh) * min(abs(dh), v_max);
        step = dh * wc;
        fp = (isfield(opts, 'fixed_point') && opts.fixed_point) || (isfield(opts, 'hold_until') && t(k) < opts.hold_until);
        if fp; dh = 0;  step = zeros(3, 1); end              % 10-21: p_d held at the start (whole run, or until opts.hold_until)
        if fp && isfield(opts, 'dither')                     % 10-21 user: circle in the estimated tangent plane while holding,
            A = opts.dither(1);  fd = opts.dither(2);        %   radius A [um], f [Hz]: offset (A(cos 2 pi f t - 1), A sin 2 pi f t) along (u_c, v_c)
            Tr = 0;  if numel(opts.dither) >= 3; Tr = opts.dither(3); end   % optional ramp: radius A min(t / T_ramp, 1) (10-21)
            Af = @(tt) A * min(tt / max(Tr, eps), 1) * (Tr > 0) + A * (Tr == 0);
            c1 = Af(t(k)) * [cos(2 * pi * fd * t(k)) - 1; sin(2 * pi * fd * t(k))];
            c2 = Af(t(k) + Ts) * [cos(2 * pi * fd * (t(k) + Ts)) - 1; sin(2 * pi * fd * (t(k) + Ts))];
            step = step + ((c2(1) - c1(1)) * uc + (c2(2) - c1(2)) * vc) / Rr;
        end
        uv_on = isfield(opts, 'uv_hold') && opts.uv_hold && (~isfield(opts, 'uv_hold_until') || t(k) < opts.uv_hold_until);   % 10-21: off from uv_hold_until
        if uv_on                                             % 10-20 user: hold u_d, v_d (estimated local coordinate, relative to p-bar_d[0])
            % (1) [u_d; v_d; w_d] = V-hat' (p-bar_d - p-bar_d[0]);  (2) w_d += Delta h_d,  u_d -> (1 - g_uv) u_d,  v_d -> (1 - g_uv) v_d
            % (3) p-bar_d[k+1] = p-bar_d[k] + R (Delta h_d w-hat - g_uv (u_d u-hat + v_d v-hat))
            % scan (t >= t_scan): u_d follows u_ref, u_ref += Delta u_d per step (feed-forward below), v_d still held at 0
            if t(k) >= opts.t_scan; u_ref = u_ref + opts.du_d; end
            dpd = (pd_k - pd0) / Rr;  corr = -g_uv * ((uc' * dpd - u_ref) * uc + (vc' * dpd) * vc);
            nc = norm(corr);  if nc > uv_max; corr = corr * uv_max / nc; end
            step = step + corr;
        end
        if t(k) >= opts.t_scan
            if isempty(u0); u0 = p; end
            if isfield(opts, 'raster')                         % 10-22 user (Meng Fig. 57): u triangle of length Lu, v ramp of dv at each end
                Lu = opts.raster(1) / Rr;  dvr = opts.raster(2) / Rr;  nl = opts.raster(3);
                if rs.line <= nl
                    if rs.phase == 'u'
                        step = step + rs.dir * opts.du_d * uc;  rs.s = rs.s + opts.du_d;
                        if rs.s >= Lu; rs.s = 0;  rs.dir = -rs.dir;  rs.phase = 'v';  rs.line = rs.line + 1; end
                    else
                        step = step + opts.du_d * vc;  rs.s = rs.s + opts.du_d;
                        if rs.s >= dvr; rs.s = 0;  rs.phase = 'u'; end
                    end
                end
                L.line(k) = rs.line;
            else
                step = step + opts.du_d * uc;
            end
        end
        pd_kp1 = pd_k + step * Rr;  hd_int = hd_int + wc' * step;
        phat = pm + a_nom * (Abuf(:, :, 1) * fbuf(:, 1) + Abuf(:, :, 2) * fbuf(:, 2));
        f_d = (a_nom * Ab) \ (pd_kp1 - phat - lc * (pd_k - phat));
        % ---- log ----
        L.h(k) = h_true;  L.aT_true(k) = aTt;  L.aN_true(k) = aNt;  L.apT_true(k) = apTt;  L.apN_true(k) = apNt;
        L.aT(k) = x(10);  L.aN(k) = x(12);  L.apT(k) = x(11);  L.apN(k) = x(13);  L.thx(k) = x(14);  L.thy(k) = x(15);
        L.sP(k, :) = sqrt(diag(Pk(10:15, 10:15)))';  L.p(k, :) = p';  L.pd(k, :) = pd_k';  L.dhd(k) = dh;  L.aNm(k) = vA(3, 3);  L.gate(k) = gate_th;  L.thx_true(k) = thx_t;  L.thy_true(k) = thy_t;
        if ~isempty(u0); L.u_prog(k) = dot(p - u0, local_Ry(opts.theta_y * pi / 180) * [1; 0; 0]) / Rr; end
        % ---- plant ----
        f_th = calc_thermal_force(p, P);
        try
            p = step_dynamics(p, f_d + f_th, P, Ts);
        catch ME                                               % one step through the wall: record the state that produced it
            contact = true;  t_contact = t(k);  kend = k;
            L.fail = struct('t', t(k), 'x', x, 'dh', dh, 'f_d', f_d, 'h', h_true, 'msg', ME.message, 'Pdiag', diag(Pk));
            break
        end
        pbuf = [pbuf(:, 2:end), p + cfg.meas_noise_std(:) .* randn(3, 1)];
        % ---- predict (Section 4/5 of 1015_estimator_3d_wsonly) ----
        Fb = a_o * (f_d + (1 - lc) * sum(fbuf, 2));
        Mh = step + (1 - lc) * x(7:9);
        apT = x(11);  apN = x(13);  wF = wh' * Fb;  wM = wh' * Mh;  cx = cos(x(14));
        Fe = zeros(15);  Fe(1:3, 4:6) = eye(3);  Fe(4:6, 7:9) = eye(3);  Fe(7:9, 7:9) = lc * eye(3);
        Fe(7:9, 10) = -(uh * uh' + vh * vh') * Fb;  Fe(7:9, 12) = -(wh * wh') * Fb;
        Fe(7:9, 14) = (x(12) - x(10)) * (vh * wh' + wh * vh') * Fb;
        Fe(7:9, 15) = -(x(12) - x(10)) * cx * (uh * wh' + wh * uh') * Fb;
        cthx = (x(10) - x(12)) * (vh' * Fb) - vh' * Mh;  cthy = cx * (uh' * Mh - (x(10) - x(12)) * (uh' * Fb));
        Fe(10, 7:9) = apT * (1 - lc) * wh';  Fe(10, 10) = 1;  Fe(10, 11) = wM;  Fe(10, 12) = apT * wF;
        Fe(10, 14) = apT * cthx;  Fe(10, 15) = apT * cthy;  Fe(11, 11) = 1;
        Fe(12, 7:9) = apN * (1 - lc) * wh';  Fe(12, 12) = 1 + apN * wF;  Fe(12, 13) = wM;
        Fe(12, 14) = apN * cthx;  Fe(12, 15) = apN * cthy;  Fe(13, 13) = 1;  Fe(14, 14) = 1;  Fe(15, 15) = 1;
        Q33 = kap * (Ab + (1 - lc)^2 * (Abuf(:, :, 1) + Abuf(:, :, 2))) + (1 - lc)^2 * Sn;
        g = zeros(15, 3);  g(7:9, :) = -eye(3);  g(10, :) = apT * wh';  g(12, :) = apN * wh';
        Q = g * Q33 * g';
        hd = max(dot(wh, pd_k) / Rr, 1.1);  if is_bend; hd = max(hd_int, 1.1); end
        pfix = (0.4 * (1.1 / hd)^2)^2;  L.hd_used(k) = hd;
        q_on = ~isfield(opts, 'q_theta_from') || t(k) >= opts.q_theta_from;   % 10-21 user: q_theta only from this time on (e.g. t_scan)
        if q_on && strcmp(opts.q_theta, 'pfix')               % 10-20 user: q_theta = P_fix^2 I[k] (sqrt P_fix tuned, deg)
            Pft = (opts.sP_fix_theta_deg * pi / 180)^2;  Q(14, 14) = Q(14, 14) + Pft^2 * I_th(1);  Q(15, 15) = Q(15, 15) + Pft^2 * I_th(2);
        elseif strcmp(opts.q_theta, 'const')                  % 10-21 user: plain random-walk q per step, sqrt q = opts.sq_theta_deg [deg/step]
            qc = (opts.sq_theta_deg * pi / 180)^2;  Q(14, 14) = Q(14, 14) + qc;  Q(15, 15) = Q(15, 15) + qc;
        end
        Q(11, 11) = Q(11, 11) + pfix^2 * I_ap(1);  Q(13, 13) = Q(13, 13) + pfix^2 * I_ap(2);
        x(1:3) = x(4:6);  x(4:6) = x(7:9);  x(7:9) = lc * x(7:9);
        x(10) = x(10) + apT * wM;  x(12) = x(12) + apN * wM;
        Pk = Fe * Pk * Fe' + Q;  Pk = 0.5 * (Pk + Pk');
        fbuf = [f_d, fbuf(:, 1)];  Abuf = cat(3, Ab, Abuf(:, :, 1));
        pdbuf = [pdbuf(:, 2:end), pd_kp1];  pd_k = pd_kp1;
    end
    r = struct('seed', seed, 't', t, 'contact', contact, 't_contact', t_contact, 'kend', kend, 'opts', opts, 't_latch', t_latch);
    f = fieldnames(L);  for i = 1:numel(f); r.(f{i}) = L.(f{i}); end
    r.thx = r.thx * 180 / pi;  r.thy = r.thy * 180 / pi;  r.sP(:, 5:6) = r.sP(:, 5:6) * 180 / pi;
    [r.h_star, ~] = local_hstar(opts.a_d);
end

function [aT, aN, apT, apN] = local_truth(h)
    [cpa, cpe] = calc_correction_functions(max(h, 1.001));  aT = 1 / cpa;  aN = 1 / cpe;
    dh = 1e-4;  [cpa2, cpe2] = calc_correction_functions(max(h + dh, 1.001));  [cpa1, cpe1] = calc_correction_functions(max(h - dh, 1.001));
    apT = (1 / cpa2 - 1 / cpa1) / (2 * dh);  apN = (1 / cpe2 - 1 / cpe1) / (2 * dh);
end

function [hs, ok] = local_hstar(a_d)                             % plane height where a_N = a_d
    f = @(h) 1 / local_cperp(h) - a_d;  hs = fzero(f, [1.001 30]);  ok = true;
end

function c = local_cperp(h)
    [~, c] = calc_correction_functions(h);
end

function [W, thx, thy] = local_bump_wall(p, W, pz0, o)      % 10-22: z_w = pz0 + H exp(-r^2 / 2 sigma^2), local tangent plane under the probe
    H = 3;  sg = 7;  xc = 25;  yc = -3.5;                      % [um]; scan line y = 0 passes sigma / 2 off the centre (plot_wall_bump_setup)
    if isfield(o, 'bump'); H = o.bump(1); sg = o.bump(2); xc = o.bump(3); yc = o.bump(4); end
    dx = p(1) - xc;  dy = p(2) - yc;  zb = H * exp(-(dx^2 + dy^2) / (2 * sg^2));
    n = [dx / sg^2 * zb; dy / sg^2 * zb; 1];  n = n / norm(n);  % (-dz/dx, -dz/dy, 1) normalised
    thx = -asin(max(min(n(2), 1), -1));  thy = atan2(n(1), n(3));
    V = local_Ry(thy) * local_Rx(thx);
    W.V = V;  W.u_hat = V(:, 1);  W.v_hat = V(:, 2);  W.w_hat = V(:, 3);  W.pz = n' * [p(1); p(2); pz0 + zb];
    thx = thx * 180 / pi;  thy = thy * 180 / pi;
end

function [W, thx, thy] = local_bead_wall(p, W, pz0, o)      % 10-22: bead (radius bead_r um, top at z = pz0 under the start) on the glass
    rb = 5;  if isfield(o, 'bead_r'); rb = o.bead_r; end      % Meng diss. Sec. 6.2.1: 10 um glass bead; glass plane at z = pz0 - 2 rb
    c = [0; 0; pz0 - rb];  zg = pz0 - 2 * rb;
    d = p(:) - c;  db = norm(d) - rb;  dg = p(3) - zg;         % distances to the bead surface and to the glass
    if db <= dg                                                % nearest surface + plane law (local tangent plane)
        n = d / norm(d);  pzw = n' * c + rb;
    else
        n = [0; 0; 1];  pzw = zg;
    end
    thx = -asin(max(min(n(2), 1), -1));  thy = atan2(n(1), n(3));
    V = local_Ry(thy) * local_Rx(thx);
    W.V = V;  W.u_hat = V(:, 1);  W.v_hat = V(:, 2);  W.w_hat = V(:, 3);  W.pz = pzw;
    thx = thx * 180 / pi;  thy = thy * 180 / pi;
end

function [W, thx, thy] = local_bend_wall(p, W, pz0, o)      % local tangent plane of the bend wall at the probe (x, y); angles deg
    n = [cosd(o.bend_phi); sind(o.bend_phi)];  ae = o.bend_alpha * pi / 180;  Lxi = o.bend_Lx * n(1);
    xi = n' * p(1:2) - o.bend_xb * n(1);                       % 0 at the bend start on the scan line (y = 0)
    if xi <= 0
        a = 0;  zw = 0;
    elseif xi <= Lxi
        a = ae * xi / Lxi;  zw = (Lxi / ae) * log(cos(a));     % -int tan(ae s / Lxi) ds
    else
        a = ae;  zw = (Lxi / ae) * log(cos(ae)) - tan(ae) * (xi - Lxi);
    end
    w = [sin(a) * n(1); sin(a) * n(2); cos(a)];
    thx = -asin(sin(a) * n(2));  thy = atan2(sin(a) * n(1), cos(a));
    V = local_Ry(thy) * local_Rx(thx);
    W.V = V;  W.u_hat = V(:, 1);  W.v_hat = V(:, 2);  W.w_hat = V(:, 3);
    W.pz = W.w_hat' * [p(1); p(2); pz0 + zw];
    thx = thx * 180 / pi;  thy = thy * 180 / pi;
end

function M = local_Rx(a)
    M = [1 0 0; 0 cos(a) -sin(a); 0 sin(a) cos(a)];
end

function M = local_Ry(a)
    M = [cos(a) 0 sin(a); 0 1 0; -sin(a) 0 cos(a)];
end

function [C_T, C_n, cT, cn] = local_coeffs(lc, apd, d)     % as run_t1_angle_readout (impulse responses of dp_r)
    Lg = 400;  n = Lg + 60;
    gT = local_chain(lc, apd, d, n, 'tau');  gn = local_chain(lc, apd, d, n, 'n');
    cT = zeros(Lg + 1, 1);  cn = cT;
    for tau = 0:Lg
        cT(tau + 1) = sum(gT(1:end-tau) .* gT(1+tau:end));  cn(tau + 1) = sum(gn(1:end-tau) .* gn(1+tau:end));
    end
    C_T = cT(1);  C_n = cn(1);
end

function g = local_chain(lc, apd, d, n, src)
    tau = zeros(n, 1);  nz = zeros(n, 1);
    if strcmp(src, 'tau'); tau(1) = 1; else; nz(1) = 1; end
    dp = zeros(n + 1, 1);
    for k = 1:n
        past = 0;  for i = 1:d; if k - i >= 1; past = past + tau(k - i); end; end
        dp(k + 1) = lc * dp(k) - (tau(k) + (1 - lc) * past - (1 - lc) * nz(k));
    end
    bar = 0;  g = zeros(n, 1);
    for k = 1:n
        prev = 0;  if k - d >= 1; prev = dp(k - d); end
        dpm = prev - nz(k);  bar = apd * dpm + (1 - apd) * bar;  g(k) = dpm - bar;
    end
end
