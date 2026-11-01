% FORK OF test_script/scratch/run_t2_3d_reg.m @ 70c8a42 (plant, sensor, readout chain, local_coeffs) | PURPOSE: first test of
%   1023_estimator_3d_xyz.tex -- estimator in the measurement coordinate, states [dp1 dp2 dp3 (x, y, z), six entries of A-bar],
%   wall angles computed every step from A-hat (A5, closed form from trace / sum of squares; 10-23 replaced the incremental form); fixed point (p_d held), tilted plane theta_y.
%   Same random-number order as run_t2_3d_reg (paired seeds with its fixed_point arm). | EXPIRES: when the xyz estimator moves into a
%   controller file | 產線改動不會自動跟上
%   Settings for the test only (not in the document): A-hat[0] = I (flat wall, far field), theta-hat[0] = 0, P[0]: dp = sigma_n^2,
%   every entry of a = ((9/8)/h0)^2 (size of the far-field term, as the gains of run_t2_3d_reg); Q of a = 0 (gain constant at a fixed
%   point); R of the six readout entries = IF (S_ik S_jl + S_il S_jk) / (C_T kappa_T)^2, S = C_T kappa_T A-hat + C_n Sigma_n
%   (Isserlis, single sample), IF of the thermal chain; control uses A-hat with eigenvalues floored at 0.02 (counted in r.n_floor).
%   Logged: theta-hat (A5 closed form), theta_eig (eigenvector of the smallest eigenvalue of A-hat, the reference of the "no drift" check),
%   a_T-hat / a_N-hat (A5), entries of A-hat and of the true A-bar, min eigenvalue of A-hat.
%   opts.dir_kf (10-24): option B, command directions from theta_c (weights from P), estimator unchanged.
%   opts.flow (10-24): 10-21 flow (hold_until + dither, regulation tau_a / a_d along w-hat, scan du_d along u-hat from t_scan).
%   opts.aprime (10-24): 21 states with the six entries of A-bar' (1024_estimator_xyz_aprime.tex), default off.
%   opts.t_scan, opts.du_d (10-24): scan along u-hat after t_scan (no regulation).
%   run: R = run_t3_xyz_fixed(struct('theta_y', 20, 'h0', 1.5, 'seeds', 1, 'T', 5))
function R = run_t3_xyz_fixed(opts)
    if nargin < 1; opts = struct(); end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));  addpath(here);
    def = struct('seeds', 1, 'theta_y', 20, 'h0', 1.5, 'T', 5, 'lambda_c', 0.4, 'tag', 'xyz', 'save', false);
    fn = fieldnames(def);
    for i = 1:numel(fn); if ~isfield(opts, fn{i}); opts.(fn{i}) = def.(fn{i}); end; end
    R = struct('opts', opts, 'runs', {{}});
    for s = opts.seeds
        r = local_run(opts, s);  R.runs{end+1} = r;
        fprintf('%s seed %d: th_y %2d h0 %.2f | end theta-hat (%.2f, %.2f) eig (%.2f, %.2f) | a_T %.3f vs %.3f, a_N %.3f vs %.3f | min eig %.3f, floor %d\n', ...
                opts.tag, s, opts.theta_y, opts.h0, r.thx(end), r.thy(end), r.thx_eig(end), r.thy_eig(end), r.aT(end), r.aT_true(end), ...
                r.aN(end), r.aN_true(end), min(r.mineig), r.n_floor);
    end
    if opts.save
        fo = fullfile(root, 'test_results', 'temp_3d_wsonly');  if ~exist(fo, 'dir'); mkdir(fo); end
        save(fullfile(fo, ['temp_t3_' opts.tag '.mat']), 'R');
    end
end

function r = local_run(opts, seed)
    if isfield(opts, 'form') && strcmp(opts.form, 'estimated'); r = local_run_est(opts, seed); return; end
    if isfield(opts, 'form') && strcmp(opts.form, 'local'); r = local_run_loc(opts, seed); return; end
    clear calc_thermal_force;
    gen = 'threefry';  if isfield(opts, 'rng_gen'); gen = opts.rng_gen; end;  rng(seed, gen);   % 10-29: generator named (threefry = the parfor-worker default used by the morning figures), serial = parfor
    cfg = canonical_scenario(0.05, 1.10, 'ramp_tri', 1);
    cfg.lambda_c = opts.lambda_c;  cfg.theta_y = opts.theta_y;  cfg.theta_x = 0;  if isfield(opts, 'theta_x'); cfg.theta_x = opts.theta_x; end   % 10-26: theta_x [deg]
    if isfield(opts, 'noise_um'); cfg.meas_noise_std = opts.noise_um(:); end   % 10-25 (user): sensor noise override [um] per axis
    params = calc_simulation_params(cfg);  P = params.Value;
    Ts = P.common.Ts;  Rr = P.common.R;  a_nom = Ts / P.common.gamma_N;  a_o = a_nom / Rr;
    kap = 4 * P.ctrl.k_B * P.ctrl.T * a_o / Rr;  Sn = diag((cfg.meas_noise_std(:) / Rr).^2);
    lc = cfg.lambda_c;  apd = cfg.a_pd;  d = 2;
    [C_T, C_n, cT, cn] = local_coeffs(lc, apd, d);
    rT = cT / cT(1);  IF = 1 + 2 * sum(rT(2:end).^2);
    w_true = P.wall.w_hat(:);  pz = P.wall.pz;  V_true = P.wall.V;
    N = round(opts.T / Ts) + 1;  t = (0:N-1)' * Ts;
    p = (pz + opts.h0 * Rr) * w_true;  pd_k = p;               % start (and stay) at h0 [R] along the true normal
    h_true0 = (dot(p, w_true) - pz) / Rr;
    [aTt, aNt] = local_truth(h_true0);  A_true = V_true * diag([aTt aTt aNt]) * V_true';
    hd0 = p(3) / Rr;                                           % w-hat[0] = z: h_d[0] seen by the estimator
    x = [zeros(9, 1); 1; 1; 1; 0; 0; 0];                       % [dp1 dp2 dp3 | a_xx a_yy a_zz a_xy a_xz a_yz]
    sig2 = diag(Sn);
    Pk = diag([repmat(sig2, 3, 1); ((9/8) / hd0)^2 * ones(6, 1)]);
    ap_on = isfield(opts, 'aprime') && opts.aprime;            % 10-24 (user): 1024_estimator_xyz_aprime.tex, six entries of A-bar' as states
    if ap_on                                                   %   seeds = far-field leading terms of 1015 (9/16, 9/16, 9/8) / h_d[0]^2,
        ap0 = [9/16; 9/16; 9/8; 0; 0; 0] / hd0^2;              %   prior = the size of the a_N' seed for every entry
        x = [x; ap0];  Pk = blkdiag(Pk, ((9/8) / hd0^2)^2 * eye(6));
    end
    th_on = isfield(opts, 'th_state') && opts.th_state;         % 10-25 (user): 1025_computed_estimated_angle.tex Section 6, theta as a state
    if th_on                                                   %   (needs aprime); theta-hat[0] = 0, prior 10 deg (as run_t2_3d_reg), q = 0;
        x = [x; 0; 0];  Pk = blkdiag(Pk, (10 * pi / 180)^2 * eye(2));   %   theta_m = theta(A-hat) + d theta / d a (a_m - a-hat),
    end                                                        %   R with the cross terms (T R T')
    nx = numel(x);  I_ap = zeros(6, 1);
    th = [0; 0];  Vh = eye(3);                                 % theta-hat[0] = 0, V-hat[0] = I
    H = [eye(3), zeros(3, nx - 3); zeros(6, 9), eye(6), zeros(6, nx - 15)];
    ij = [1 1; 2 2; 3 3; 1 2; 1 3; 2 3];
    pbuf = repmat(p, 1, d + 1);  pdbuf = repmat(pd_k, 1, d + 1);  fbuf = zeros(3, d);  Abuf = repmat(eye(3), 1, 1, d);
    dbar = zeros(3, 1);  n_floor = 0;  kend = N;  contact = false;
    thc = [0; 0];  sc2 = [1e6; 1e6];  s_ramp = 0;                           % 10-24 option B: command direction theta_c and its variance (huge = no prior)
    L = struct('h', NaN(N,1), 'thx', NaN(N,1), 'thy', NaN(N,1), 'thx_eig', NaN(N,1), 'thy_eig', NaN(N,1), 'aT', NaN(N,1), ...
               'aN', NaN(N,1), 'a', NaN(N,6), 'mineig', NaN(N,1), 'sPa', NaN(N,6), 'dp', NaN(N,3), 'p', NaN(N,3), 'ap', NaN(N,6), 'rmax', NaN(N,1), 'aN_tr', NaN(N,1), 'aNw', NaN(N,1), 'aT_tr', NaN(N,1), 'aTw', NaN(N,1), 'apNw', NaN(N,1), 'thc', NaN(N,2), 'sth', NaN(N,2), 'thA', NaN(N,2), 'sPth', NaN(N,2), 'use_thm', false(N,1), 'am', NaN(N,6));
    for k = 1:N
        h_true = (dot(p, w_true) - pz) / Rr;
        if h_true < 1; kend = k - 1; contact = true; break; end
        % ---- A3: random component and measured motion gain matrix (x, y, z; no V-hat) ----
        pm = pbuf(:, 1);  dpm = (pdbuf(:, 1) - pm) / Rr;
        dbar = apd * dpm + (1 - apd) * dbar;  dpr = dpm - dbar;
        Am = (dpr * dpr' - C_n * Sn) / (C_T * kap);
        am = [Am(1,1); Am(2,2); Am(3,3); Am(1,2); Am(1,3); Am(2,3)];
        L.am(k, :) = am';                                       % 10-25: single-sample readout (log only)
        % ---- A4: update (one vector update, R of the six entries from Isserlis) ----
        Ah = local_sym(x(10:15));  Ac = local_floor(Ah, 0.02);
        S = C_T * kap * Ac + C_n * Sn;  R6 = zeros(6);
        for a = 1:6
            for b = 1:6
                i1 = ij(a, 1); j1 = ij(a, 2); k1 = ij(b, 1); l1 = ij(b, 2);
                R6(a, b) = IF * (S(i1, k1) * S(j1, l1) + S(i1, l1) * S(j1, k1)) / (C_T * kap)^2;
                if isfield(opts, 'r6_if_axis') && opts.r6_if_axis  % 10-25 diagnostic (hypothesis 1): time-correlation factor per pair,
                    rr = @(p1, p2) local_rho(p1, p2, Ac, Sn, kap, C_T, C_n, cT, cn);   %   thermal + noise mixed per axis (as 1021 rho)
                    F1 = 1 + 2 * sum(rr(i1, k1) .* rr(j1, l1));  F2 = 1 + 2 * sum(rr(i1, l1) .* rr(j1, k1));
                    R6(a, b) = (F1 * S(i1, k1) * S(j1, l1) + F2 * S(i1, l1) * S(j1, k1)) / (C_T * kap)^2;
                end
            end
        end
        if isfield(opts, 'r6_diag') && opts.r6_diag; R6 = diag(diag(R6)); end   % 10-25 diagnostic: elements updated independently
        Rm = blkdiag(diag(sig2), R6);
        y = [dpm; am];  yh = [x(1:3); x(10:15)];  Hk = H;  nu = y - yh;
        if th_on
            [thA, Jth, okA] = local_theta_of_A(Ah);
            if okA                                             % first order valid only when the angle of A-hat is already known to
                okA = all(sqrt(diag(Jth * Pk(10:15, 10:15) * Jth')) < 10 * pi / 180);   %   within the theta prior (10 deg); else skip theta_m
                L.use_thm(k) = okA;
            end
            if okA                                            % Section 6: measured angle and R with the cross terms
                thA = thA - pi * round((thA - x(22:23)) / pi);      % theta(A-hat) on the same branch as theta-hat (line angle, period 180 deg);
                thm = thA + Jth * (am - x(10:15));                  %   the single-sample term is NOT folded (hundreds of deg, Gaussian in R)
                Tm = [eye(9); zeros(2, 3), Jth];  Rm = Tm * Rm * Tm';
                e_th = thm - x(22:23);
                Hk = [H; zeros(2, nx - 2), eye(2)];  nu = [nu; e_th];
            end
        end
        Sy = Hk * Pk * Hk' + Rm;  K = Pk * Hk' / Sy;
        if isfield(opts, 'dbg') && k <= opts.dbg && th_on && okA
            fprintf('k %3d |J| %9.3g thA (%7.2f %7.2f) thm (%8.2f %8.2f) th-hat (%8.2f %8.2f) a_xz %8.4f a_xx-a_zz %8.4f cond(S) %.2g\n', k, norm(Jth), ...
                    thA * 180 / pi, thm * 180 / pi, x(22:23) * 180 / pi, x(14), x(10) - x(12), cond(Sy));
        end
        if ap_on; pf_ap = diag(Pk(16:21, 16:21)); end
        x = x + K * nu;
        IKH = eye(nx) - K * Hk;  Pk = IKH * Pk * IKH' + K * Rm * K';  Pk = 0.5 * (Pk + Pk');
        if ap_on; I_ap = max(1 ./ diag(Pk(16:21, 16:21)) - 1 ./ pf_ap, 0); end   % information of each a' slot this step
        % ---- A5: wall angles from A-hat (default 'plane', 10-24; 'trace' = the 10-23 closed form; the incremental form locked in 23 / 36) ----
        % trace t and sum of squares s of A-hat give a_T-hat > a_N-hat; w-hat ~ third column of A-hat - a_T-hat I, written with w_z > 0
        Ah = local_sym(x(10:15));  tA = trace(Ah);  sA = sum(Ah(:).^2);
        aTh = (2 * tA + sqrt(max(6 * sA - 2 * tA^2, 0))) / 6;  aNh = tA - 2 * aTh;
        if ~isfield(opts, 'readout') || strcmp(opts.readout, 'plane')   % 10-24 (user): plane by plane, x-z gives theta_y,
            thy_p = atan2(-2 * Ah(1,3), Ah(1,1) - Ah(3,3)) / 2;          %   y-z gives phi_yz -> theta_x = -atan(tan phi_yz cos theta_y)
            phi_yz = atan2(-2 * Ah(2,3), Ah(2,2) - Ah(3,3)) / 2;
            th = [-atan(tan(phi_yz) * cos(thy_p)); thy_p];
            th_log = th;   % 10-24: an unwrapped log was tried and dropped (wraps accumulate from the Â ~ I start); figures take the
            %   line-angle error mod 180 deg instead
        else                                                           % 'trace': trace / sum of squares, third column of A-hat - a_T-hat I
            cw = [Ah(1,3); Ah(2,3); Ah(3,3) - aTh];  wh = cw / norm(cw);  if wh(3) < 0; wh = -wh; end
            th = [-asin(wh(2)); atan2(wh(1), wh(3))];  th_log = th;
        end
        L.thA(k, :) = th' * 180 / pi;                              % 10-25: computed angle theta(A-hat) (also logged in the theta-state arm)
        if th_on; th = x(22:23);  th_log = th; end                 % Section 6: w-hat from the state theta-hat
        Vh = local_Ry(th(2)) * local_Rx(th(1));
        [Ev, D] = eig(Ah);  [ev, imin] = min(diag(D));  we = Ev(:, imin);  if we(3) < 0; we = -we; end
        Vc = Vh;  sth2 = [NaN; NaN];
        if isfield(opts, 'dir_kf') && opts.dir_kf                % 10-24 (user) option B: theta_c <- theta_c + k (theta-hat - theta_c),
            hx = Ah(1,1) - Ah(3,3);  vx = -2 * Ah(1,3);  hy = Ah(2,2) - Ah(3,3);  vy = -2 * Ah(2,3);   % k = s_c^2 / (s_c^2 + s_theta^2),
            cy = [Ah(1,3), 0, -Ah(1,3), 0, -hx, 0] / (hx^2 + vx^2);    % s_theta^2 = c P_a c' with c the rows of
            cp = [0, Ah(2,3), -Ah(2,3), 0, 0, -hy] / (hy^2 + vy^2);    % 1024_estimator_xyz_aprime.tex Section 4
            Tp = tan(atan2(vy, hy) / 2);  cth = cos(th(2));  sth = sin(th(2));  dn = 1 + Tp^2 * cth^2;
            cx = -(cth * (1 + Tp^2) / dn) * cp + (Tp * sth / dn) * cy;
            Pa = Pk(10:15, 10:15);  sth2 = [cx * Pa * cx'; cy * Pa * cy'];
            for q = 1:2
                if isfinite(sth2(q)) && sth2(q) > 0
                    dlt = th(q) - thc(q);  dlt = dlt - pi * round(dlt / pi);          % line-angle difference
                    kq = sc2(q) / (sc2(q) + sth2(q));  thc(q) = thc(q) + kq * dlt;  sc2(q) = (1 - kq) * sc2(q);
                    thc(q) = thc(q) - pi * round(thc(q) / pi);                         % keep the principal value
                end
            end
            Vc = local_Ry(thc(2)) * local_Rx(thc(1));
        end
        % ---- A2: control law (fixed point) ----
        Ac = local_floor(Ah, 0.02);  if ev < 0.02; n_floor = n_floor + 1; end
        pd_kp1 = pd_k;
        if isfield(opts, 'flow') && opts.flow                     % 10-24 (user): the 10-21 flow as run_t2_3d_reg (flow_s1): hold until
            wh_ = Vc(:, 3);  aN_r = wh_' * Ah * wh_;               %   hold_until with a ramped dither circle in the estimated tangent plane,
            apN_r = max(wh_' * local_sym(x(16:21)) * wh_, 1 / 22.2^2);   % then regulation along w-hat to a_N = a_d (tau_a), scan along u-hat
            dhr = (Ts / opts.tau_a) * (opts.a_d - aN_r) / apN_r;   %   from t_scan (regulation continues); (1 - lambda_a) = Ts / tau_a
            step = dhr * wh_;
            if t(k) < opts.hold_until && ~isfield(opts, 'dither'); step = zeros(3, 1); end   % 10-24: plain wait (no dither)
            if t(k) < opts.hold_until && isfield(opts, 'dither')
                step = zeros(3, 1);  Ad = opts.dither(1);  fdd = opts.dither(2);  Tr = opts.dither(3);
                Af = @(tt) Ad * min(tt / Tr, 1);
                c1 = Af(t(k)) * [cos(2 * pi * fdd * t(k)) - 1; sin(2 * pi * fdd * t(k))];
                c2 = Af(t(k) + Ts) * [cos(2 * pi * fdd * (t(k) + Ts)) - 1; sin(2 * pi * fdd * (t(k) + Ts))];
                step = ((c2(1) - c1(1)) * Vc(:, 1) + (c2(2) - c1(2)) * Vc(:, 2)) / Rr;
            end
            if t(k) >= opts.t_scan; step = step + opts.du_d * Vc(:, 1); end
            pd_kp1 = pd_k + step * Rr;
        elseif isfield(opts, 't_scan') && t(k) >= opts.t_scan     % 10-24 (user): after the hold, p_d moves along u-hat (computed angle),
            pd_kp1 = pd_k + opts.du_d * Rr * Vh(:, 1);             %   du_d [R] per step, no regulation: h drifts by scan length * sin(angle error)
        elseif isfield(opts, 'ramp')                              % 10-25 (user): w motion, ramp along -w-hat (as the estimated arm)
            stp = min(opts.ramp(1) * Ts, opts.ramp(2) - s_ramp);  pd_kp1 = pd_k - stp * Vh(:, 3);  s_ramp = s_ramp + stp;
        elseif isfield(opts, 'dither')                            % 10-25 (user): u v motion, ramped circle in the estimated tangent plane
            pd_kp1 = pd_k + local_circle_step(opts.dither, t(k), Ts, Vh);   %   (10-21 dither, now without the flow)
        end
        phat = pm + a_nom * (Abuf(:, :, 1) * fbuf(:, 1) + Abuf(:, :, 2) * fbuf(:, 2));
        f_d = (a_nom * Ac) \ (pd_kp1 - phat - lc * (pd_k - phat));
        % ---- log ----
        L.h(k) = h_true;  L.thx(k) = th_log(1);  L.thy(k) = th_log(2);  L.thc(k, :) = thc' * 180 / pi;  L.sth(k, :) = sqrt(sth2') * 180 / pi;
        if isfield(opts, 'flow') && opts.flow; [L.aT_tr(k), L.aN_tr(k)] = local_truth(h_true); L.aNw(k) = Vh(:,3)' * Ah * Vh(:,3); L.aTw(k) = Vh(:,1)' * Ah * Vh(:,1); if ap_on; L.apNw(k) = Vh(:,3)' * local_sym(x(16:21)) * Vh(:,3); end; end
        L.thx_eig(k) = -asin(we(2));  L.thy_eig(k) = atan2(we(1), we(3));
        L.aT(k) = aTh;  L.aN(k) = aNh;  L.a(k, :) = x(10:15)';  L.mineig(k) = ev;  L.sPa(k, :) = sqrt(diag(Pk(10:15, 10:15)))';
        if th_on; L.sPth(k, :) = sqrt(diag(Pk(22:23, 22:23)))' * 180 / pi; end
        L.dp(k, :) = ((pd_k - p) / Rr)';  L.p(k, :) = p';  if ap_on; L.ap(k, :) = x(16:21)'; end
        % ---- plant ----
        f_th = calc_thermal_force(p, P);
        try
            p = step_dynamics(p, f_d + f_th, P, Ts);
        catch
            kend = k;  contact = true;  break                    % one step through the wall: stop, keep the record up to here
        end
        pbuf = [pbuf(:, 2:end), p + cfg.meas_noise_std(:) .* randn(3, 1)];
        % ---- predict (B5) ----
        Fb = a_o * (f_d + (1 - lc) * sum(fbuf, 2));
        Fx = Fb(1);  Fy = Fb(2);  Fz = Fb(3);
        GF = [Fx 0 0 Fy Fz 0; 0 Fy 0 Fx 0 Fz; 0 0 Fz 0 Fx Fy];
        Fe = zeros(nx);  Fe(1:3, 4:6) = eye(3);  Fe(4:6, 7:9) = eye(3);  Fe(7:9, 7:9) = lc * eye(3);
        Fe(7:9, 10:15) = -GF;  Fe(10:15, 10:15) = eye(6);
        Q33 = kap * (Ac + (1 - lc)^2 * (Abuf(:, :, 1) + Abuf(:, :, 2))) + (1 - lc)^2 * Sn;
        Q = zeros(nx);  Q(7:9, 7:9) = Q33;
        if isfield(opts, 'q_eps') && ~ap_on                    % 10-25 (user): no A-bar' states; gain random walk Q_a = eps R
            Q(10:15, 10:15) = opts.q_eps * R6;                 %   (Layer-1 first-order line, eps = 1e-5), R6 = this step's readout R
        end
        if ap_on                                               % 1024_estimator_xyz_aprime.tex Sections 3-7
            wh = Vh(:, 3);  uh = Vh(:, 1);  vh = Vh(:, 2);  aph = x(16:21);
            Mh = (pd_kp1 - pd_k) / Rr + (1 - lc) * x(7:9);     % M-hat [R]
            hx = Ah(1,1) - Ah(3,3);  vx = -2 * Ah(1,3);  hy = Ah(2,2) - Ah(3,3);  vy = -2 * Ah(2,3);
            cy = [Ah(1,3), 0, -Ah(1,3), 0, -hx, 0] / (hx^2 + vx^2);          % e_theta_y = cy e_a (Section 4)
            cp = [0, Ah(2,3), -Ah(2,3), 0, 0, -hy] / (hy^2 + vy^2);          % e_phi_yz  = cp e_a
            Tp = tan(atan2(vy, hy) / 2);  cth = cos(th(2));  sth = sin(th(2));  dn = 1 + Tp^2 * cth^2;
            cx = -(cth * (1 + Tp^2) / dn) * cp + (Tp * sth / dn) * cy;        % e_theta_x = cx e_a
            rth = -(vh' * Mh) * cx + cos(th(1)) * (uh' * Mh) * cy;
            if ~(isfield(opts, 'ap_rtheta') && opts.ap_rtheta); rth = zeros(1, 6); end   % 10-24 (user): r_theta dropped by default
            %   (1.6e8 at the start where A-hat ~ I; ~0.0025 e_a otherwise, negligible next to I_6); opts.ap_rtheta = true keeps it
            L.rmax(k) = max(abs(rth));
            Fe(10:15, 7:9) = (1 - lc) * aph * wh';
            Fe(10:15, 10:15) = eye(6) + aph * (wh' * GF) + aph * rth;
            Fe(10:15, 16:21) = (wh' * Mh) * eye(6);  Fe(16:21, 16:21) = eye(6);
            g = zeros(nx, 3);  g(7:9, :) = -eye(3);  g(10:15, :) = aph * wh';
            Q = g * Q33 * g';
            hd = max(dot(wh, pd_k) / Rr, 1.1);  pfix = (0.4 * (1.1 / hd)^2)^2;   % P_fix (1015, 1019_report)
            Q(16:21, 16:21) = diag(pfix^2 * I_ap);
            q_on = isfield(opts, 'ap_q') && opts.ap_q;                              % 10-24 (user): q of A-bar' = 0 while holding (fixed point),
            if isfield(opts, 'flow') && opts.flow; q_on = t(k) >= opts.hold_until; end   %   P_fix^2 I[k] once the probe moves (regulation, scan);
            if ~q_on; Q(16:21, 16:21) = 0; end                                         %   the slope changes ~30x from 15 um to 1.1 R
            if isfield(opts, 'qap_eps'); Q(16:21, 16:21) = opts.qap_eps * R6; end      % 10-25 (user): keep A-bar', Q_a' = eps R6
            if th_on                                                                   % Section 7: a-hat' M-hat^T (d w / d theta at theta-hat)
                sx = sin(th(1));  cxa = cos(th(1));  sy = sin(th(2));  cya = cos(th(2));
                dW = [-sy * sx, cya * cxa; -cxa, 0; -cya * sx, -sy * cxa];
                Fe(10:15, 22:23) = aph * (Mh' * dW);  Fe(22:23, 22:23) = eye(2);
            end
            x(10:15) = x(10:15) + aph * (wh' * Mh);
        end
        x(1:3) = x(4:6);  x(4:6) = x(7:9);  x(7:9) = lc * x(7:9);
        Pk = Fe * Pk * Fe' + Q;  Pk = 0.5 * (Pk + Pk');
        if isfield(opts, 'decouple_pa') && opts.decouple_pa   % 10-25 diagnostic (hypothesis 2): cut the dp - A-hat cross covariance
            Pk(1:9, 10:end) = 0;  Pk(10:end, 1:9) = 0;
        end
        fbuf = [f_d, fbuf(:, 1)];  Abuf = cat(3, Ac, Abuf(:, :, 1));
        pdbuf = [pdbuf(:, 2:end), pd_kp1];  pd_k = pd_kp1;
    end
    r = struct('seed', seed, 't', t, 'opts', opts, 'kend', kend, 'contact', contact, 'n_floor', n_floor, 'aT_true', aTt, 'aN_true', aNt, 'A_true', A_true, 'IF', IF);
    f = fieldnames(L);  for i = 1:numel(f); r.(f{i}) = L.(f{i}); end
    r.aT_true = repmat(aTt, N, 1);  r.aN_true = repmat(aNt, N, 1);
    r.thx = r.thx * 180 / pi;  r.thy = r.thy * 180 / pi;  r.thx_eig = r.thx_eig * 180 / pi;  r.thy_eig = r.thy_eig * 180 / pi;
    r.thx_true = 0;  if isfield(opts, 'theta_x'); r.thx_true = opts.theta_x; end;  r.thy_true = opts.theta_y;
end

function r = local_run_est(opts, seed)
% 10-25 (user): estimated angle, 1025_computed_estimated_angle.tex Section 6 (option B). States [dp1 dp2 dp3 (x y z) | a_par a_perp |
%   a_par' a_perp' | theta_x theta_y] (15); A-hat = V(theta-hat) diag(a_par, a_par, a_perp) V'(theta-hat); process outputs = the six
%   elements a-bar_m (same readout, R and plant as the computed-angle arm, same random-number order -> paired seeds); H gain rows =
%   d a / d(a_par, a_perp, theta) at the estimate; F_e of Section 6. Initial values and priors as run_t2_3d_reg (1021): gains 1,
%   slopes (9/16, 9/8)/h_d[0]^2, priors (9/8)/h_d[0] and the slope seeds, theta prior 10 deg; theta-hat[0] = opts.th0_deg (default 0).
%   Fixed point only (p_d held). q of the slopes as the computed arm (0 unless opts.ap_q). opts.dump_k: store x, H, finite-difference
%   H, F_e and the step inputs at that step (wiring check, check_1025_est_wiring.m).
    clear calc_thermal_force;
    gen = 'threefry';  if isfield(opts, 'rng_gen'); gen = opts.rng_gen; end;  rng(seed, gen);   % 10-29: generator named (threefry = the parfor-worker default used by the morning figures), serial = parfor
    cfg = canonical_scenario(0.05, 1.10, 'ramp_tri', 1);
    cfg.lambda_c = opts.lambda_c;  cfg.theta_y = opts.theta_y;  cfg.theta_x = 0;  if isfield(opts, 'theta_x'); cfg.theta_x = opts.theta_x; end   % 10-26: theta_x [deg]
    if isfield(opts, 'noise_um'); cfg.meas_noise_std = opts.noise_um(:); end   % 10-25 (user): sensor noise override [um] per axis
    params = calc_simulation_params(cfg);  P = params.Value;
    Ts = P.common.Ts;  Rr = P.common.R;  a_nom = Ts / P.common.gamma_N;  a_o = a_nom / Rr;
    kap = 4 * P.ctrl.k_B * P.ctrl.T * a_o / Rr;  Sn = diag((cfg.meas_noise_std(:) / Rr).^2);
    lc = cfg.lambda_c;  apd = cfg.a_pd;  d = 2;
    [C_T, C_n, cT, ~] = local_coeffs(lc, apd, d);
    rT = cT / cT(1);  IF = 1 + 2 * sum(rT(2:end).^2);
    w_true = P.wall.w_hat(:);  pz = P.wall.pz;  V_true = P.wall.V;
    N = round(opts.T / Ts) + 1;  t = (0:N-1)' * Ts;
    p = (pz + opts.h0 * Rr) * w_true;  pd_k = p;
    h_true0 = (dot(p, w_true) - pz) / Rr;
    [aTt, aNt] = local_truth(h_true0);  A_true = V_true * diag([aTt aTt aNt]) * V_true';
    hd0 = p(3) / Rr;
    x = zeros(15, 1);  x(10) = 1;  x(11) = 1;  x(12) = (9/16) / hd0^2;  x(13) = (9/8) / hd0^2;
    if isfield(opts, 'gain0') && strcmp(opts.gain0, 'seed'); x(10) = 1 - (9/16) / hd0;  x(11) = 1 - (9/8) / hd0; end   % 10-26: reflection seeds
    if isfield(opts, 'th0_deg'); x(14:15) = opts.th0_deg(:) * pi / 180; end
    sPth = 10 * pi / 180;  if isfield(opts, 'sP_theta_deg'); sPth = opts.sP_theta_deg * pi / 180; end
    sig2 = diag(Sn);
    Pk = diag([repmat(sig2, 3, 1); ((9/8) / hd0)^2 * [1; 1]; x(12)^2; x(13)^2; sPth^2 * [1; 1]]);
    ig = [10 11 14 15];  ij = [1 1; 2 2; 3 3; 1 2; 1 3; 2 3];  I_ap = zeros(2, 1);
    pbuf = repmat(p, 1, d + 1);  pdbuf = repmat(pd_k, 1, d + 1);  fbuf = zeros(3, d);  Abuf = repmat(eye(3), 1, 1, d);
    dbar = zeros(3, 1);  n_floor = 0;  kend = N;  contact = false;  dump = struct();  s_ramp = 0;
    L = struct('h', NaN(N,1), 'thx', NaN(N,1), 'thy', NaN(N,1), 'aT', NaN(N,1), 'aN', NaN(N,1), 'apT', NaN(N,1), 'apN', NaN(N,1), ...
               'a', NaN(N,6), 'mineig', NaN(N,1), 'sPth', NaN(N,2), 'sPa', NaN(N,2), 'dp', NaN(N,3), 'thA', NaN(N,2), 'aN_tr', NaN(N,1), 'aT_tr', NaN(N,1));
    for k = 1:N
        h_true = (dot(p, w_true) - pz) / Rr;
        if h_true < 1; kend = k - 1; contact = true; break; end
        % ---- process outputs (as the computed arm) ----
        pm = pbuf(:, 1);  dpm = (pdbuf(:, 1) - pm) / Rr;
        dbar = apd * dpm + (1 - apd) * dbar;  dpr = dpm - dbar;
        Am = (dpr * dpr' - C_n * Sn) / (C_T * kap);  am = local_six(Am);
        % ---- update: a-hat from (a_par, a_perp, theta), H gain rows = derivatives ----
        [Ah, Ja] = local_est_A(x);
        Ac = local_floor(Ah, 0.02);  S = C_T * kap * Ac + C_n * Sn;  R6 = zeros(6);
        for a = 1:6
            for b = 1:6
                i1 = ij(a, 1); j1 = ij(a, 2); k1 = ij(b, 1); l1 = ij(b, 2);
                R6(a, b) = IF * (S(i1, k1) * S(j1, l1) + S(i1, l1) * S(j1, k1)) / (C_T * kap)^2;
            end
        end
        Rm = blkdiag(diag(sig2), R6);
        H = zeros(9, 15);  H(1:3, 1:3) = eye(3);  H(4:9, ig) = Ja;
        nu = [dpm - x(1:3); am - local_six(Ah)];
        if isfield(opts, 'dump_k') && k == opts.dump_k             % finite-difference H (gain rows) at this step
            Jfd = zeros(6, 4);  hs = 1e-7;
            for c = 1:4
                xp = x;  xp(ig(c)) = xp(ig(c)) + hs;  xm = x;  xm(ig(c)) = xm(ig(c)) - hs;
                Jfd(:, c) = (local_six(local_est_A(xp)) - local_six(local_est_A(xm))) / (2 * hs);
            end
            dump.Ja = Ja;  dump.Jfd = Jfd;
        end
        Sy = H * Pk * H' + Rm;  K = Pk * H' / Sy;
        pf_ap = diag(Pk(12:13, 12:13));
        x = x + K * nu;
        IKH = eye(15) - K * H;  Pk = IKH * Pk * IKH' + K * Rm * K';  Pk = 0.5 * (Pk + Pk');
        I_ap = max(1 ./ diag(Pk(12:13, 12:13)) - 1 ./ pf_ap, 0);
        % ---- angle: the estimate ----
        [Ah, Ja, wh, dW] = local_est_A(x);  Vh = local_Ry(x(15)) * local_Rx(x(14));
        ev = min(eig(0.5 * (Ah + Ah')));
        % ---- control law (fixed point) ----
        Ac = local_floor(Ah, 0.02);  if ev < 0.02; n_floor = n_floor + 1; end
        pd_kp1 = pd_k;
        if isfield(opts, 'ramp')                                % 10-25 (user, Meng dissertation Sec. 5.5): ramp toward the wall along
            stp = min(opts.ramp(1) * Ts, opts.ramp(2) - s_ramp);   %   -w-hat at opts.ramp(1) um/s for opts.ramp(2) um, then hold
            pd_kp1 = pd_k - stp * wh;  s_ramp = s_ramp + stp;
        elseif isfield(opts, 'dither')                          % 10-25 (user): u v motion, ramped circle in the estimated tangent plane
            pd_kp1 = pd_k + local_circle_step(opts.dither, t(k), Ts, Vh);
        elseif isfield(opts, 'reg')                              % 10-25 (user): regulation from t = 0, 1021 law along w-hat:
            dh = (Ts / opts.reg(2)) * (opts.reg(1) - x(11)) / max(x(13), 1 / 22.2^2);   %   Delta h = (Ts/tau_a)(a_d - a_perp-hat)/a_perp'-hat
            pd_kp1 = pd_k + dh * Rr * wh;                          %   opts.reg = [a_d, tau_a]
        end
        phat = pm + a_nom * (Abuf(:, :, 1) * fbuf(:, 1) + Abuf(:, :, 2) * fbuf(:, 2));
        f_d = (a_nom * Ac) \ (pd_kp1 - phat - lc * (pd_k - phat));
        % ---- log ----
        [thA_k, ~, ~] = local_theta_of_A(Ah);
        L.h(k) = h_true;  L.thx(k) = x(14);  L.thy(k) = x(15);  L.aT(k) = x(10);  L.aN(k) = x(11);  L.apT(k) = x(12);  L.apN(k) = x(13);
        L.a(k, :) = local_six(Ah)';  L.mineig(k) = ev;  L.sPth(k, :) = sqrt(diag(Pk(14:15, 14:15)))' * 180 / pi;
        L.sPa(k, :) = sqrt(diag(Pk(10:11, 10:11)))';  L.dp(k, :) = ((pd_k - p) / Rr)';  L.thA(k, :) = thA_k' * 180 / pi;  [L.aT_tr(k), L.aN_tr(k)] = local_truth(h_true);
        % ---- plant ----
        f_th = calc_thermal_force(p, P);
        try
            p = step_dynamics(p, f_d + f_th, P, Ts);
        catch
            kend = k;  contact = true;  break
        end
        pbuf = [pbuf(:, 2:end), p + cfg.meas_noise_std(:) .* randn(3, 1)];
        % ---- predict (Section 6) ----
        Fb = a_o * (f_d + (1 - lc) * sum(fbuf, 2));
        GF = [Fb(1) 0 0 Fb(2) Fb(3) 0; 0 Fb(2) 0 Fb(1) 0 Fb(3); 0 0 Fb(3) 0 Fb(1) Fb(2)];
        Mh = (pd_kp1 - pd_k) / Rr + (1 - lc) * x(7:9);  sM = wh' * Mh;
        Fe = zeros(15);  Fe(1:3, 4:6) = eye(3);  Fe(4:6, 7:9) = eye(3);  Fe(7:9, 7:9) = lc * eye(3);  Fe(7:9, ig) = -GF * Ja;
        for q = 1:2                                            % rows of a_par (10), a_perp (11); slopes in 12, 13
            ap_q = x(11 + q);  rw = 9 + q;
            Fe(rw, 7:9) = (1 - lc) * ap_q * wh';
            Fe(rw, ig) = ap_q * (wh' * GF) * Ja;  Fe(rw, rw) = Fe(rw, rw) + 1;
            Fe(rw, 11 + q) = sM;
            Fe(rw, 14:15) = Fe(rw, 14:15) + ap_q * (Mh' * dW);
        end
        Fe(12, 12) = 1;  Fe(13, 13) = 1;  Fe(14:15, 14:15) = eye(2);
        Q33 = kap * (Ac + (1 - lc)^2 * (Abuf(:, :, 1) + Abuf(:, :, 2))) + (1 - lc)^2 * Sn;
        g = zeros(15, 3);  g(7:9, :) = -eye(3);  g(10, :) = x(12) * wh';  g(11, :) = x(13) * wh';
        Q = g * Q33 * g';
        if isfield(opts, 'ap_q') && opts.ap_q
            hd = max(dot(wh, pd_k) / Rr, 1.1);  pfix = (0.4 * (1.1 / hd)^2)^2;  Q(12:13, 12:13) = diag(pfix^2 * I_ap);
        end
        if isfield(opts, 'dump_k') && k == opts.dump_k
            dump.x = x;  dump.Fe = Fe;  dump.GF = GF;  dump.dpd = (pd_kp1 - pd_k) / Rr;  dump.lc = lc;
        end
        x(10) = x(10) + x(12) * sM;  x(11) = x(11) + x(13) * sM;
        x(1:3) = x(4:6);  x(4:6) = x(7:9);  x(7:9) = lc * x(7:9);
        Pk = Fe * Pk * Fe' + Q;  Pk = 0.5 * (Pk + Pk');
        if isfield(opts, 'lf_theta')                           % 10-26 (Meng dissertation (5.26)): forgetting factor on the angle slots,
            sc = ones(15, 1);  sc(14:15) = 1 / sqrt(opts.lf_theta);   %   P_theta <- P_theta / lambda_f, cross terms / sqrt(lambda_f)
            Pk = diag(sc) * Pk * diag(sc);
        end
        fbuf = [f_d, fbuf(:, 1)];  Abuf = cat(3, Ac, Abuf(:, :, 1));
        pdbuf = [pdbuf(:, 2:end), pd_kp1];  pd_k = pd_kp1;
    end
    r = struct('seed', seed, 't', t, 'opts', opts, 'kend', kend, 'contact', contact, 'n_floor', n_floor, 'A_true', A_true, 'IF', IF, 'dump', dump);
    f = fieldnames(L);  for i = 1:numel(f); r.(f{i}) = L.(f{i}); end
    r.aT_true = repmat(aTt, N, 1);  r.aN_true = repmat(aNt, N, 1);
    r.thx = r.thx * 180 / pi;  r.thy = r.thy * 180 / pi;  r.thx_eig = r.thx;  r.thy_eig = r.thy;
    r.thx_true = 0;  if isfield(opts, 'theta_x'); r.thx_true = opts.theta_x; end;  r.thy_true = opts.theta_y;
end

function r = local_run_loc(opts, seed)
% 10-26 (user): 1026_local_angle_estimator.tex -- estimator in the local coordinate of V-hat = V(theta-hat). States
%   [dp1 dp2 dp3 (u v w) | a_par a_perp a_uw a_vw | a_par' a_perp' | theta_x theta_y] (17). Process outputs rotated by V-hat[k]:
%   (uu + vv)/2, ww, uw, vw of V-hat' A_m V-hat; H = 0 / 1. Written as the document (combined form, every term at step k):
%   x[k+1] = f(x[k]) + L[k] e[k] with L = F_e K (K = P^f H' S^-1); f includes theta-hat[k] + delta-theta-hat[k],
%   delta-theta-hat[k] = a_vw-hat[k] / (a_par-hat[k] - a_perp-hat[k]) (estimates before the feedback); F_e angle rows of Section 5
%   (e_theta[k+1] = e_off[k] / (a_par - a_perp)). Order: feedback -> control law (estimates after the feedback, local of V-hat[k])
%   -> plant -> x[k+1] in the local coordinate of V-hat[k] -> all estimates (and the dp blocks of P) into the local coordinate of
%   V-hat[k+1]. R: R6 of the six elements mapped linearly to the four local elements (not in the document). Same plant, readout and
%   random-number order as local_run_est (paired seeds). opts.gain0 = 'seed': gains start at 1 - (9/16)/h, 1 - (9/8)/h.
%   opts.gap_gate (test only): delta-theta-hat = 0 and angle rows of F_e = I unless |a_par-hat - a_perp-hat| > n sqrt(P_gap).
    clear calc_thermal_force;
    gen = 'threefry';  if isfield(opts, 'rng_gen'); gen = opts.rng_gen; end;  rng(seed, gen);  rs_ch = RandStream('threefry', 'Seed', seed + 7919);   % 10-29: separate stream for the ym_cheat noise (thermal draws unchanged)   % 10-29: generator named (threefry = the parfor-worker default used by the morning figures), serial = parfor
    cfg = canonical_scenario(0.05, 1.10, 'ramp_tri', 1);
    cfg.lambda_c = opts.lambda_c;  cfg.theta_y = opts.theta_y;  cfg.theta_x = 0;  if isfield(opts, 'theta_x'); cfg.theta_x = opts.theta_x; end
    if isfield(opts, 'noise_um'); cfg.meas_noise_std = opts.noise_um(:); end
    params = calc_simulation_params(cfg);  P = params.Value;
    Ts = P.common.Ts;  Rr = P.common.R;  a_nom = Ts / P.common.gamma_N;  a_o = a_nom / Rr;
    kap = 4 * P.ctrl.k_B * P.ctrl.T * a_o / Rr;  Sn = diag((cfg.meas_noise_std(:) / Rr).^2);
    lc = cfg.lambda_c;  apd = cfg.a_pd;  d = 2;
    [C_T, C_n, cT, ~] = local_coeffs(lc, apd, d);
    rT = cT / cT(1);  IF = 1 + 2 * sum(rT(2:end).^2);
    w_true = P.wall.w_hat(:);  pz = P.wall.pz;  V_true = P.wall.V;
    N = round(opts.T / Ts) + 1;  t = (0:N-1)' * Ts;
    p = (pz + opts.h0 * Rr) * w_true;  pd_k = p;
    wall = 'plane';  if isfield(opts, 'wall'); wall = opts.wall; end   % 10-28 (user): curved walls -- 'bend' (as run_t2_3d_reg) or 'hemi' (hemisphere
    curved = ~strcmp(wall, 'plane');  pz0 = pz;  rb = 15;  if isfield(opts, 'hemi_r'); rb = opts.hemi_r; end   %   of radius hemi_r um on the plane, start above its top);
    if strcmp(wall, 'hemi'); p = [0; 0; pz0 + rb + opts.h0 * Rr];  pd_k = p; end
    if strcmp(wall, 'hemi') && isfield(opts, 'hemi_start_deg')   % 10-28 (user): start off the top, polar angle hemi_start_deg along scan_dir
        sd0 = [1; 0];  if isfield(opts, 'scan_dir'); sd0 = opts.scan_dir(:) / norm(opts.scan_dir); end
        ph0 = opts.hemi_start_deg * pi / 180;  p = [0; 0; pz0] + (rb + opts.h0 * Rr) * [sin(ph0) * sd0; cos(ph0)];  pd_k = p;
    end   %   plant = plane law on the local tangent plane of the nearest surface point
    b2 = strcmp(wall, 'bead2');                                % 10-29 (user): Meng glass bead -- bead (radius bead_r um) on the glass z = pz0,
    if b2 || strcmp(wall, 'bead3') || strcmp(wall, 'fillet')                             %   two-wall superposition of the plane-law drag (local_gamma2); start above the glass
        sxy = [9; -9];  if isfield(opts, 'bead_start'); sxy = opts.bead_start(:); end;  p = [sxy; pz0 + opts.h0 * Rr];  pd_k = p;
        if isfield(opts, 'bead_top_deg')                       %   or above the bead, bead_top_deg off its top on the +x side, h0 above its surface
            rbd = 5;  if isfield(opts, 'bead_r'); rbd = opts.bead_r; end;  a0 = opts.bead_top_deg * pi / 180;
            p = [0; 0; pz0 + rbd] + (rbd + opts.h0 * Rr) * [sin(a0); 0; cos(a0)];  pd_k = p;
        end
    end
    if curved; [P.wall, w_true, pz, V_true] = local_wall_now(wall, p, P.wall, pz0, rb, opts); end
    h_true0 = (dot(p, w_true) - pz) / Rr;
    [aTt, aNt] = local_truth(h_true0);  A_true = V_true * diag([aTt aTt aNt]) * V_true';
    hd0 = opts.h0;   % 10-28 fix: commanded distance along the wall normal (was p(3)/Rr = z height, wrong on a tilted plane: 5 um at theta_x -70 gave 1.6 um)
    hd_int = hd0;  thT = [cfg.theta_x; cfg.theta_y];  s_scan = 0;   % 10-29 fix: plane wall logged true angles (were 0 for a tilted plane)
    eI = 0;  e_sgn0 = NaN;  pi_go = false;                     % PI regulation: running sum of a_d - a-hat_33 (opts.reg_pi); 'cross' gate state
    rs_on = isfield(opts, 'raster') || isfield(opts, 'raster_disc');  rs_line = 1;  rs_dir = -1;  rs_mode = 0;  rs_y0 = 0;  rs_tend = NaN;   % 10-28 (user): raster [X_end um, dy um, n_lines]
    dsc_on = isfield(opts, 'descent_v');  dsc_done = false;  tsc = inf;  if isfield(opts, 't_scan'); tsc = opts.t_scan; end   % 10-28 (user): constant-speed descent
    if dsc_on; tsc = inf; end                                 %   after the hold, no regulation, until a-hat_33 <= a_d; then scan + regulation start
    ap_hold = NaN;                                            % 10-29 (user): opts.reg_ap_hold -- regulation slope frozen at a'-hat_33 of the scan start
    treg = inf;  pw = [0 0];  if isfield(opts, 'post_wait'); pw = opts.post_wait; end   % 10-29 (user): [regulation, scan] start this long after the descent stop
    x = zeros(18, 1);  x(10) = 1;  x(11) = 1;  x(14) = (9/16) / hd0^2;  x(15) = (9/8) / hd0^2;
    if isfield(opts, 'gain0') && strcmp(opts.gain0, 'seed'); x(10) = 1 - (9/16) / hd0;  x(11) = 1 - (9/8) / hd0; end
    if isfield(opts, 'gain0') && isnumeric(opts.gain0); x(10:11) = opts.gain0(:); end   % reference arm only (e.g. the true gains)
    if isfield(opts, 'th0_deg'); x(16:17) = opts.th0_deg(:) * pi / 180; end
    sPth = 10 * pi / 180;  if isfield(opts, 'sP_theta_deg'); sPth = opts.sP_theta_deg * pi / 180; end
    sig2 = diag(Sn);  Vh = local_Ry(x(17)) * local_Rx(x(16));
    sPg = (9/8) / hd0;  if isfield(opts, 'sP_gain'); sPg = opts.sP_gain; end   % reference arm only: small gain prior
    Pk = diag([repmat(sig2, 3, 1); sPg^2 * [1; 1]; 0; 0; x(14)^2; x(15)^2; sPth^2 * [1; 1]]);
    Pk(1:9, 1:9) = kron(eye(3), Vh' * diag(sig2) * Vh);
    dh0 = x(10) - x(11);                                       % prior of a_uw, a_vw consistent with the angle prior
    if isfield(opts, 'off_gap_seed') && opts.off_gap_seed; dh0 = (9/8 - 9/16) / hd0; end   % 10-27 (user test): gains start at 1, off-diagonal prior from the reflection-seed gap
    Jc = zeros(2);  Jc(1, 2) = -dh0 * cos(x(16));  Jc(2, 1) = dh0;   % [a_uw; a_vw] = Jc [e_thx; e_thy]
    Pk(12:13, 12:13) = Jc * Pk(16:17, 16:17) * Jc';  Pk(12:13, 16:17) = Jc * Pk(16:17, 16:17);  Pk(16:17, 12:13) = Pk(12:13, 16:17)';
    th_out = isfield(opts, 'theta_out') && opts.theta_out;   % 10-27 (user): angle out of the KF -- theta-hat kept outside, P has no angle block,
    if th_out; Pk(16:17, :) = 0;  Pk(:, 16:17) = 0; end
    if isfield(opts, 'freeze_ap11') && opts.freeze_ap11; Pk(14, :) = 0;  Pk(:, 14) = 0; end   % 10-28 test: a'_11 held at its seed (no P, no Q)       %   angle step from the updated off-diagonal elements, angle error in F_e = e_off / gap
    mk = isfield(opts, 'mekf') && opts.mekf;                  % 10-28 (user test): error-state angle (MEKF) -- slots 12, 13 = small angles (d theta_y, d theta_x),
    if mk                                                     %   H of the uw, vw readouts = (-gap cos theta-hat_x, gap) (multiplication, no gate), reset to 0 each step
        Pk(12:13, :) = 0;  Pk(:, 12:13) = 0;  Pk(12, 12) = sPth^2;  Pk(13, 13) = sPth^2;  Pk(16:17, :) = 0;  Pk(:, 16:17) = 0;  th_out = true;
    end
    kp_on = isfield(opts, 'kappa_state') && opts.kappa_state;   % 10-28 (user test): slot 18 = wall curvature kappa [1/R] (constant, Q = 0):
    Pk(18, 18) = 0;  if kp_on; sPk = 0.2;  if isfield(opts, 'sP_kappa'); sPk = opts.sP_kappa; end;  Pk(18, 18) = sPk^2; end   %   the normal turns by kappa * (tangential step)
    dt_on = isfield(opts, 'doc_timing') && opts.doc_timing;  % 10-28 (user): document timing -- delta-theta-hat[k+1] = a_off-hat[k+1] / gap[k]
                                                            %   from x[k+1] (predict + feedback, coordinate of V-hat[k]); with theta_out = arm A,
                                                            %   without = arm B1 (angle also has its own gain L10 = rows of F_e K, F_e angle rows = I)
    ij = [1 1; 2 2; 3 3; 1 2; 1 3; 2 3];  I_ap = zeros(2, 1);  nn = [0 0 1];
    pbuf = repmat(p, 1, d + 1);  pdbuf = repmat(pd_k, 1, d + 1);  fbuf = zeros(3, d);  Abuf = repmat(eye(3), 1, 1, d);
    dbar = zeros(3, 1);  n_floor = 0;  kend = N;  contact = false;  s_ramp = 0;
    L = struct('h', NaN(N,1), 'thx', NaN(N,1), 'thy', NaN(N,1), 'aT', NaN(N,1), 'aN', NaN(N,1), 'apT', NaN(N,1), 'apN', NaN(N,1), ...
               'auw', NaN(N,1), 'avw', NaN(N,1), 'dth', NaN(N,2), 'dgap', NaN(N,1), 'a', NaN(N,6), 'mineig', NaN(N,1), 'sPth', NaN(N,2), ...
               'sPa', NaN(N,2), 'Pcond', NaN(N,1), 'dp', NaN(N,3), 'aN_tr', NaN(N,1), 'aT_tr', NaN(N,1), 'gate', NaN(N,1), 'off_post', NaN(N,2), 'off_tr', NaN(N,2), 'dLe', NaN(N,2), 'sgap', NaN(N,1), 'ym', NaN(N,4), 'yhat', NaN(N,4), 'off_k1', NaN(N,2), 'p', NaN(N,3), 'pd', NaN(N,3), 'th_tr', NaN(N,2), 'hd_int', NaN(N,1));  dump = struct();
    for k = 1:N
        if curved; [P.wall, w_true, pz, V_true, thT] = local_wall_now(wall, p, P.wall, pz0, rb, opts); end
        h_true = (dot(p, w_true) - pz) / Rr;
        if h_true < 1; kend = k - 1; contact = true; break; end
        [aTk, aNk] = local_truth(h_true);  A_true = V_true * diag([aTk aTk aNk]) * V_true';   % 10-29 fix: also on a plane (was frozen at h_true0; log / reference arms only)
        if b2; Gn = local_gamma2(p, opts, pz0, Rr);  A_true = inv(Gn);  ea = sort(eig(0.5 * (A_true + A_true')));  aNk = ea(1);  aTk = mean(ea(2:3)); end
        % ---- process outputs, into the local coordinate of V-hat[k] ----
        pm = pbuf(:, 1);  dpm = (pdbuf(:, 1) - pm) / Rr;
        dbar = apd * dpm + (1 - apd) * dbar;  dpr = dpm - dbar;
        Am = (dpr * dpr' - C_n * Sn) / (C_T * kap);
        vdpm = Vh' * dpm;  vAm = Vh' * Am * Vh;
        y4 = [(vAm(1,1) + vAm(2,2)) / 2; vAm(3,3); vAm(1,3); vAm(2,3)];
        L.vdpr(k, :) = (Vh' * dpr)';  L.vdpm(k, :) = vdpm';   % 10-29 log: readout input (high-passed tracking error) in the local coordinate of V-hat[k]
        % ---- R: six elements (as the other arms) -> four local elements ----
        vAh = local_loc_A(x);  Ah = Vh * vAh * Vh';  Ahx = Ah;   % 10-29: gain matrix before the feedback (log of the known control-gain term m)
        Ac = local_floor(Ah, 0.02);  S = C_T * kap * Ac + C_n * Sn;  R6 = zeros(6);
        for a = 1:6
            for b = 1:6
                i1 = ij(a, 1); j1 = ij(a, 2); k1 = ij(b, 1); l1 = ij(b, 2);
                R6(a, b) = IF * (S(i1, k1) * S(j1, l1) + S(i1, l1) * S(j1, k1)) / (C_T * kap)^2;
            end
        end
        T4 = zeros(4, 6);                                       % y4 = T4 * (six elements of A_m), linear in A_m
        for c = 1:6
            E6 = zeros(6, 1);  E6(c) = 1;  vE = Vh' * local_sym(E6) * Vh;
            T4(:, c) = [(vE(1,1) + vE(2,2)) / 2; vE(3,3); vE(1,3); vE(2,3)];
        end
        Rm = blkdiag(Vh' * diag(sig2) * Vh, T4 * R6 * T4');
        if isfield(opts, 'ym_cheat') && t(k) >= opts.ym_cheat_from   % 10-29 reference arms only: gain readout replaced by the TRUE elements
            vAt0 = Vh' * A_true * Vh;  y4 = [(vAt0(1,1) + vAt0(2,2)) / 2; vAt0(3,3); vAt0(1,3); vAt0(2,3)];   %   in the local coordinate of V-hat[k]
            if strcmp(opts.ym_cheat, 'true_noise'); [Ub, Db] = eig(0.5 * (Rm(4:7, 4:7) + Rm(4:7, 4:7)'));  y4 = y4 + Ub * diag(sqrt(max(diag(Db), 0))) * randn(rs_ch, 4, 1); end   %   + white noise with the model R (own stream)
        end
        H = zeros(7, 18);  H(1:3, 1:3) = eye(3);  H(4:7, 10:13) = eye(4);
        if mk; gk = x(10) - x(11);  H(6, 12) = -gk * cos(x(16));  H(7, 13) = gk; end
        nu = [vdpm - x(1:3); y4 - H(4:7, :) * x];
        Sy = H * Pk * H' + Rm;  K = Pk * H' / Sy;                % x = estimates at step k (before the feedback), Pk = P^f[k]
        if isfield(opts, 'k_ap_cut') && t(k) >= opts.k_ap_cut_from   % 10-29 test: the slopes (slots 14:15) not updated by one channel
            if strcmp(opts.k_ap_cut, 'y1'); K(14:15, 1:3) = 0; else; K(14:15, 4:7) = 0; end   %   ('y1' = position readings, 'ym' = gain readout); Joseph P update stays valid
        end
        xu = x + K * nu;                                        % estimates after the feedback: control law and log only
        L.Pap(k) = Pk(15, 15);  L.da33(k, :) = K(11, :) .* nu';   % 11-01 log only: prior P(a'_33); 10-31 log only: update of a-hat_33 by each of the 7 innovations (same channels as L.dap)
        L.dap(k, :) = K(15, :) .* nu';  L.k15w(k) = K(15, 3);  L.nuw(k) = nu(3);  L.c15w(k) = Pk(15, 3) / sqrt(Pk(15, 15) * Pk(3, 3));   % 10-29 log: gain, innovation, correlation of a'_33 with the normal position   % 10-29 log: update of a'-hat_33 by each channel (y1 u v w | y4 four readouts), local of V-hat[k]
        IKH = eye(18) - K * H;  Pu = IKH * Pk * IKH' + K * Rm * K';  Pu = 0.5 * (Pu + Pu');
        I_ap = max(1 ./ diag(Pu(14:15, 14:15)) - 1 ./ diag(Pk(14:15, 14:15)), 0);  I_kap = max(1 / Pu(18, 18) - 1 / Pk(18, 18), 0);   % 10-30: information on kappa this step (kappa_pfix)
        I_off = max(1 ./ diag(Pu(12:13, 12:13)) - 1 ./ diag(Pk(12:13, 12:13)), 0);   % information on a_13, a_23 this step (angle P_fix)
        % ---- small angle from the estimates at step k (document: delta-theta-hat[k] = a_vw-hat[k] / (a_par-hat[k] - a_perp-hat[k])) ----
        dgap = x(10) - x(11);  den = dgap;                       % den: denominator of the angle step (delta-theta-hat and F_e angle rows)
        if isfield(opts, 'den_mode')                           % 10-26 diagnostic (sign vs size of the denominator), not in the document:
            switch opts.den_mode                               %   'abs' -> |gap| (sign fixed); 'floor' -> sign(gap) max(|gap|, den_floor)
                case 'abs';   den = abs(dgap);                 %   (size fixed); 'both' -> max(gap, den_floor)
                case 'floor'; den = sign(dgap) * max(abs(dgap), opts.den_floor);  if den == 0; den = opts.den_floor; end
                case 'both';  den = max(dgap, opts.den_floor);
            end
        end
        dth = [x(13) / den; -x(12) / (den * cos(x(16)))];
        if th_out; dth = [xu(13) / den; -xu(12) / (den * cos(x(16)))]; end   % angle out: the off-diagonal elements after the feedback
        sgap = sqrt(max(Pk(10,10) + Pk(11,11) - 2 * Pk(10,11), 0));      % opts.gap_gate = n (test only, not in the document):
        go = ~isfield(opts, 'gap_gate') || abs(dgap) > opts.gap_gate * sgap;   %   angle step only when |a_par-hat - a_perp-hat| > n sqrt(P_gap)
        if isfield(opts, 'gap_gate') && isfield(opts, 'gate_one') && opts.gate_one; go = dgap > opts.gap_gate * sgap; end   % 10-26: one-sided (a_par > a_perp)
        if isfield(opts, 'gap_min'); go = abs(dgap) > opts.gap_min; end   % 10-26 (user test): fixed threshold on |a_par-hat - a_perp-hat|
        if isfield(opts, 'no_dth') && opts.no_dth; go = false; end
        if isfield(opts, 'freeze_angle') && opts.freeze_angle; go = false;  K(16:17, :) = 0; end   % 10-27 (explanation): angle and frame frozen, off-diagonal estimates kept   % 10-26 discriminator: delta-theta-hat = 0, angle rows of F_e = I
        if ~go; dth = [0; 0]; end
        L.ym(k, :) = y4';  L.yhat(k, :) = x(10:13)';                    % 10-27: the four local process outputs and their estimates at step k
        vAt = Vh' * A_true * Vh;  L.off_post(k, :) = [xu(12) xu(13)];  L.off_tr(k, :) = [vAt(1,3) vAt(2,3)];  L.gate(k) = go;  L.sgap(k) = sgap;
        % ---- motion control law in the local coordinate of V-hat[k] (estimates after the feedback) ----
        xa = xu;  if mk; xa(12:13) = 0; end
        if isfield(opts, 'ctrl_no_off_from') && t(k) >= opts.ctrl_no_off_from; xa(12:13) = 0; end   % 11-01 diagnostic only: control law without a_uw-hat, a_vw-hat (plant no longer matches F_e)
        Ah = Vh * local_loc_A(xa) * Vh';  wh = Vh(:, 3);
        ev = min(eig(0.5 * (Ah + Ah')));
        Ac = local_floor(Ah, 0.02);  if ev < 0.02; n_floor = n_floor + 1; end
        pd_kp1 = pd_k;
        if isfield(opts, 'ramp')
            stp = min(opts.ramp(1) * Ts, opts.ramp(2) - s_ramp);  pd_kp1 = pd_k - stp * wh;  s_ramp = s_ramp + stp;
        elseif isfield(opts, 'dither') && (~isfield(opts, 'dither_win') || (t(k) >= opts.dither_win(1) && t(k) + Ts < opts.dither_win(2)))   % 10-29: optional window
            pd_kp1 = pd_k + local_circle_step(opts.dither, t(k), Ts, Vh);
        elseif isfield(opts, 'wdither') && (~isfield(opts, 'wdither_win') || (t(k) >= opts.wdither_win(1) && t(k) + Ts < opts.wdither_win(2)))   % 10-28: optional time window [t0 t1]
            if isfield(opts, 'wdither_win'); tt0 = opts.wdither_win(1); else; tt0 = 0; end   %   (the ramp and phase start at t0)
            Af = @(tt) opts.wdither(1) * min((tt - tt0) / opts.wdither(3), 1);   %   10-27 (user): oscillation along the estimated normal w-hat, [amplitude um, f Hz, ramp s]
            pd_kp1 = pd_k + (Af(t(k) + Ts) * sin(2 * pi * opts.wdither(2) * (t(k) + Ts - tt0)) - Af(t(k)) * sin(2 * pi * opts.wdither(2) * (t(k) - tt0))) * wh;
        end
        if dsc_on && ~dsc_done && t(k) >= opts.hold_until        % descent along w-hat at opts.descent_v [um/s], open loop
            pd_kp1 = pd_kp1 - opts.descent_v * Ts * wh;  hd_int = hd_int - opts.descent_v * Ts / Rr;
            if isfield(opts, 'descent_T')                         % 10-29 (user): fixed descent time instead of the a-hat_33 <= a_d stop
                if t(k) + Ts >= opts.hold_until + opts.descent_T - 1e-9; dsc_done = true;  tsc = t(k) + Ts + pw(2);  treg = t(k) + Ts + pw(1); end
            elseif xu(11) <= opts.a_d; dsc_done = true;  tsc = t(k) + Ts + pw(2);  treg = t(k) + Ts + pw(1); end
        end
        if isfield(opts, 'hold_until') && t(k) >= opts.hold_until && (~dsc_on || t(k) >= treg) && ~(isfield(opts, 'reg_off') && opts.reg_off) && ~(isfield(opts, 'reg_off_from') && t(k) >= opts.reg_off_from)   % 10-28 (user): regulation along w-hat after the hold (1019 Part III law); 10-29 opts.reg_off: none
            tau_k = opts.tau_a;  if isfield(opts, 'tau_a_scan') && t(k) >= tsc + opts.tau_a_scan(2); tau_k = opts.tau_a_scan(1); end   % 10-29 (user): tau_a switches to tau_a_scan(1) at tsc + tau_a_scan(2)
            dhr = (Ts / tau_k) * (opts.a_d - xu(11)) / max(xu(15), 1 / 22.2^2);   %   Delta h_d = (Ts / tau_a)(a_d - a_33-hat) / a'_33-hat
            if isfield(opts, 'ap_floor_law') && opts.ap_floor_law   % 10-28 (user): floor from the exact plane law a' = (1 - a)^2 / b_true(a), b_true in [1, 9/8]
                dhr = (Ts / tau_k) * (opts.a_d - xu(11)) / max(xu(15), (8 / 9) * (1 - min(xu(11), 1))^2);
            end
            if isfield(opts, 'reg_ap_hold') && opts.reg_ap_hold && t(k) >= tsc
                if isnan(ap_hold); ap_hold = max(xu(15), (8 / 9) * (1 - min(xu(11), 1))^2); end
                dhr = (Ts / tau_k) * (opts.a_d - xu(11)) / ap_hold;
            end
            if isfield(opts, 'reg_slope_law') && opts.reg_slope_law   % 10-28 (user, option 1): regulation slope from the plane law a' = (1 - a)^2 / b at a-hat_33,
                dhr = (Ts / tau_k) * (opts.a_d - xu(11)) / ((17 / 18) * (1 - min(xu(11), 0.999))^2);   %   1/b at the middle of [8/9, 1]; a'-hat not used
            end
            if isfield(opts, 'cheat_ap') && opts.cheat_ap         % 10-28 reference arm only: regulation slope = true d a_33 / d h-bar at the true gap
                hc = max(h_true, 1.0003);  [~, c1] = calc_correction_functions(hc + 1e-4);  [~, c0] = calc_correction_functions(hc - 1e-4);
                dhr = (Ts / tau_k) * (opts.a_d - xu(11)) / ((1 / c1 - 1 / c0) / 2e-4);
            end
            whc = wh;  if isfield(opts, 'cheat_w') && opts.cheat_w; whc = w_true; end   % reference arm only: true normal for regulation and scan
            if isfield(opts, 'reg_pi') && opts.reg_pi && t(k) >= tsc   % 10-29 (user): PI regulation -- integral of e = a_d - a-hat_33 from the scan start,
                ek = opts.a_d - xu(11);  tau_i = 4 * tau_k;  if isfield(opts, 'tau_i'); tau_i = opts.tau_i; end   %   tau_i = 4 tau_a (critical damping of s^2 + s/tau_a + 1/(tau_a tau_i))
                gate = true;  if isfield(opts, 'reg_pi_cond') && isequal(opts.reg_pi_cond, true); gate = abs(ek) <= 2 * sqrt(Pu(11, 11)); end   % 10-30: conditional integration --
                if isfield(opts, 'reg_pi_cond') && strcmp(opts.reg_pi_cond, 'cross')           %   'cross': start integrating once e first changes sign
                    if isnan(e_sgn0); e_sgn0 = sign(ek); end;  if ~pi_go && sign(ek) ~= e_sgn0; pi_go = true; end;  gate = pi_go;   %   (a-hat has reached a_d once), then always
                end
                L.gate_pi(k) = gate;  if gate; eI = eI + ek; end;  L.eI(k) = eI;
                sl = max(xu(15), (8 / 9) * (1 - min(xu(11), 1))^2);                %   integrate only when |e| is within 2 sd of a-hat_33 (anti-windup);
                dhr = dhr + Ts^2 / (tau_k * tau_i) * eI / sl;                      %   10-30: I term uses the same tau as the P term (tau_k)
            end
            if isfield(opts, 'raster_keep_y') && opts.raster_keep_y && rs_on && t(k) >= tsc && rs_mode == 0   % 10-29 (user): on a raster line keep y --
                wxz = [whc(1); 0; whc(3)];  cg = norm(wxz);  whc = wxz / max(cg, 1e-9);   %   regulate along w-hat projected on the line's x-z plane, step scaled
                dhr = dhr / max(cg, 1 / 3);                                         %   by 1 / (w-hat . direction) so the gap changes by dhr (capped at 3x)
            end
            pd_kp1 = pd_kp1 + dhr * Rr * whc;  hd_int = hd_int + dhr;
        end
        if t(k) >= tsc && (~isfield(opts, 'scan_len') || s_scan < opts.scan_len)   % scan along u-hat, du_d [R] per step
            sdir = [1; 0];  if isfield(opts, 'scan_dir'); sdir = opts.scan_dir(:) / norm(opts.scan_dir); end   % 10-28: direction in the estimated (u, v) plane
            dsc = sdir(1) * Vh(:, 1) + sdir(2) * Vh(:, 2);
            if isfield(opts, 'scan_meridian') && opts.scan_meridian   % 10-28 (user): heading e = scan_dir in the world x-y plane; step along (e - z) projected
                e3 = [sdir; 0];  dsc = (eye(3) - Vh(:, 3) * Vh(:, 3)') * (e3 - [0; 0; 1]);  dsc = dsc / norm(dsc);   %   on the estimated tangent plane (meridian on a sphere)
            end
            if isfield(opts, 'raster_disc')                        % 10-29 (user): raster inside a disc of radius Rd (top view): transfer along -y to the first
                Rd = opts.raster_disc(1);  dyr = opts.raster_disc(2);  nl = round(2 * Rd / dyr);   %   line y_1 = -Rd + dy/2, lines along +-x turning at
                yk = -Rd + (rs_line - 0.5) * dyr;  Xe = sqrt(max(Rd^2 - yk^2, 0));                 %   x = +-sqrt(Rd^2 - y_k^2), steps dy along +y
                if rs_mode == 0 && rs_line == 1 && ~rs_y0; rs_mode = 2; end
                if rs_mode == 2                                    % transfer
                    dsc = (eye(3) - Vh(:, 3) * Vh(:, 3)') * [0; -1; 0];  dsc = dsc / norm(dsc);
                    if pd_kp1(2) <= yk; rs_mode = 0;  rs_y0 = 1; end
                elseif rs_mode == 0
                    dsc = cross(rs_dir * [0; 1; 0], Vh(:, 3));  dsc = dsc / norm(dsc);
                    if rs_dir * pd_kp1(1) >= Xe
                        if rs_line >= nl; dsc = zeros(3, 1);  s_scan = inf;  rs_tend = t(k); else; rs_mode = 1; end
                    end
                else
                    dsc = (eye(3) - Vh(:, 3) * Vh(:, 3)') * [0; 1; 0];  dsc = dsc / norm(dsc);
                    if pd_kp1(2) >= -Rd + (rs_line + 0.5) * dyr; rs_mode = 0;  rs_line = rs_line + 1;  rs_dir = -rs_dir; end
                end
            elseif rs_on                                           % raster: lines along +-x (direction n x w-hat, n = +-y, y kept), steps along +y
                X_end = opts.raster(1);  dyr = opts.raster(2);  nl = opts.raster(3);
                if rs_mode == 0                                    % line
                    dsc = cross(rs_dir * [0; 1; 0], Vh(:, 3));  dsc = dsc / norm(dsc);
                    if rs_dir * pd_kp1(1) >= X_end
                        if rs_line >= nl; dsc = zeros(3, 1);  s_scan = inf;  rs_tend = t(k); else; rs_mode = 1;  rs_y0 = pd_kp1(2); end
                    end
                else                                               % step along +y projected on the estimated tangent plane
                    dsc = (eye(3) - Vh(:, 3) * Vh(:, 3)') * [0; 1; 0];  dsc = dsc / norm(dsc);
                    if pd_kp1(2) - rs_y0 >= dyr; rs_mode = 0;  rs_line = rs_line + 1;  rs_dir = -rs_dir; end
                end
            elseif isfield(opts, 'scan_plane_n')                   % 10-28 (user): great circle -- step along n x w-hat (n = normal of the scan plane), works all the way round
                wsc = Vh(:, 3);  if isfield(opts, 'cheat_w') && opts.cheat_w; wsc = w_true; end
                dsc = cross(opts.scan_plane_n(:), wsc);  dsc = dsc / norm(dsc);
            end
            fr = 1;  if isfield(opts, 'scan_ramp'); fr = min((t(k) + Ts - tsc) / opts.scan_ramp, 1); end   % 10-29 (user): scan speed ramps up over scan_ramp s
            if all(isfinite(dsc)); pd_kp1 = pd_kp1 + fr * opts.du_d * Rr * dsc; end;  s_scan = s_scan + fr * opts.du_d * Rr;   %   opts.scan_len [um]: stop after this path length
        end
        phat = pm + a_nom * (Abuf(:, :, 1) * fbuf(:, 1) + Abuf(:, :, 2) * fbuf(:, 2));
        f_d = (a_nom * Ac) \ (pd_kp1 - phat - lc * (pd_k - phat));
        % ---- plant ----
        h_log = h_true;  dp_log = ((pd_k - p) / Rr)';  p_log = p;  pd_log = pd_k;  thT_log = thT;
        if b2                                                   % two-wall plant: Gamma = gamma_N Gn(p), thermal covariance (4 k_B T / Ts) Gamma
            if ~all(isfinite(Gn(:))) || ~all(isfinite(f_d)); kend = k;  contact = true;  break; end   % the run has diverged
            f_th = sqrt(4 * P.thermal.k_B * P.thermal.T * P.common.gamma_N / Ts) * chol(0.5 * (Gn + Gn'), 'lower') * randn(3, 1);
            try
                p = local_step2(p, f_d + f_th, opts, pz0, Rr, P.common.gamma_N, Ts);
            catch
                kend = k;  contact = true;  break
            end
        else
            f_th = calc_thermal_force(p, P);
            try
                p = step_dynamics(p, f_d + f_th, P, Ts);
            catch
                kend = k;  contact = true;  break
            end
        end
        pbuf = [pbuf(:, 2:end), p + cfg.meas_noise_std(:) .* randn(3, 1)];
        % ---- estimator (document Section 4): x[k+1] = f(x[k]) + L[k] e[k], L = F_e K, in the local coordinate of V-hat[k] ----
        vF = Vh' * (a_o * (f_d + (1 - lc) * sum(fbuf, 2)));
        Ga = [vF(1) 0 vF(3) 0; vF(2) 0 0 vF(3); 0 vF(3) vF(1) vF(2)];
        L.vF(k, :) = vF';   % 10-29 log: force step in the local coordinate (a_o f, R units)
        L.T15(k, :) = -[vF(3) * Pu(15, 11), vF(1) * Pu(15, 12), vF(2) * Pu(15, 13)];   % 10-31 log only: sources of P(a'_33, delta w_3) at k+1: F_w P(a', a_33), F_u P(a', a_uw), F_v P(a', a_vw)
        L.c15(k, :) = Pu(15, [3 9 11 12 13]) ./ sqrt(Pu(15, 15) * diag(Pu([3 9 11 12 13], [3 9 11 12 13]))');   % 10-31 log only: corr of a'_33 with delta w_1, delta w_3, a_33, a_uw, a_vw (after the update)
        vMh = Vh' * (pd_kp1 - pd_k) / Rr + (1 - lc) * x(7:9);
        L.vMh(k, :) = vMh';  L.mk(k, :) = (Vh' * a_o * ((Ac - Ahx) * f_d + (1 - lc) * ((Abuf(:, :, 1) - Ahx) * fbuf(:, 1) + (Abuf(:, :, 2) - Ahx) * fbuf(:, 2))))';   % 10-29 log only: known term m missing from fx(7:9)
        rs = (x(14) - x(15)) / dgap;
        if ~go; rs = 0; end                                    % 10-27: slope ratio of the off-diagonal rows also divides by the gap -> used only when the gate is open
        if ~(isfield(opts, 'off_slope') && opts.off_slope); rs = 0; end   % 10-28 (user): off-diagonal rows without the slope term (document); opts.off_slope = true restores it
        Fe = zeros(18);  Fe(18, 18) = 1;  Fe(1:3, 4:6) = eye(3);  Fe(4:6, 7:9) = eye(3);  Fe(7:9, 7:9) = lc * eye(3);  Fe(7:9, 10:13) = -Ga;
        for q = 1:2
            aq = x(13 + q);  rw = 9 + q;
            Fe(rw, 7:9) = (1 - lc) * aq * nn;  Fe(rw, 10:13) = aq * Ga(3, :);  Fe(rw, rw) = Fe(rw, rw) + 1;
            Fe(rw, 13 + q) = vMh(3);  Fe(rw, 16:17) = aq * [-vMh(2), cos(x(16)) * vMh(1)];
            if th_out                                           % angle error written with the off-diagonal errors: e_thx = e_vw / gap, e_thy = -e_uw / (gap cos)
                if ~(isfield(opts, 'no_mu_term_from') && t(k) >= opts.no_mu_term_from); Fe(rw, 13) = Fe(rw, 13) - aq * vMh(2) / den;  Fe(rw, 12) = Fe(rw, 12) - aq * vMh(1) / den; end;  Fe(rw, 16:17) = 0;   % 11-01 diagnostic only: opts.no_mu_term_from drops a' M_(u,v) / gap, keeps a' G_a
                if isfield(opts, 'no_ang_gain') && opts.no_ang_gain; Fe(rw, 12:13) = 0; end   % 10-28 test: drop the angle -> gain coupling (tangential motion x slope / gap)
            end
        end
        Fe(12, 12) = 1 + rs * vMh(3);  Fe(13, 13) = 1 + rs * vMh(3);  Fe(14, 14) = 1;  Fe(15, 15) = 1;
        ap_pred = isfield(opts, 'ap_predict') && opts.ap_predict;   % 10-28 (user test): slope predicted with the height, from the plane law a' = (1 - a)^2 / b:
        if ap_pred                                              %   a'' = -2 a'^2 / (1 - a)  ->  a'_33[k+1] = a'_33 - 2 a'_33^2 / (1 - a_33) M_w
            om = max(1 - x(11), 0.02);
            Fe(15, 15) = 1 - 4 * x(15) * vMh(3) / om;  Fe(15, 11) = -2 * x(15)^2 * vMh(3) / om^2;  Fe(15, 9) = Fe(15, 9) - 2 * x(15)^2 / om * (1 - lc);
        end
        if mk                                                   % MEKF columns: e_13 = -gap cos e_thy, e_23 = gap e_thx (slot 12 = theta_y, 13 = theta_x)
            gk = x(10) - x(11);  cx = cos(x(16));  Fe(12, 12) = 1;  Fe(13, 13) = 1;
            Fe(7:9, 12) = Ga(:, 3) * gk * cx;  Fe(7:9, 13) = -Ga(:, 4) * gk;
            for q = 1:2
                aq = x(13 + q);  rw = 9 + q;
                Fe(rw, 12) = -aq * Ga(3, 3) * gk * cx + aq * cx * vMh(1);  Fe(rw, 13) = aq * Ga(3, 4) * gk - aq * vMh(2);
            end
        end
        if th_out                                               % angle out: no angle rows
        elseif dt_on                                            % arm B1: angle constant, own gain from P
            Fe(16:17, 16:17) = eye(2);
        elseif go                                               % angle rows (document Section 5): e_theta[k+1] = e_off[k] / (a_par - a_perp)
            Fe(16, 13) = 1 / den;  Fe(17, 12) = -1 / (den * cos(x(16)));
        else
            Fe(16:17, 16:17) = eye(2);
        end
        if kp_on                                                % off-diagonal error from the curvature error: the true wall turns by kappa M_u about v-hat
            Fe(12, 18) = -dgap * vMh(1);  Fe(13, 18) = -dgap * vMh(2);   %   (theta_y up for +u) and by -kappa M_v about u-hat; e_13 = -gap e_rot_v, e_23 = gap e_rot_u
        end
        if isfield(opts, 'fe_cut') && t(k) >= opts.fe_cut_from      % 10-29 diagnostic only: drop F_e couplings from t >= fe_cut_from
            if any(strcmp(opts.fe_cut, 'angain')); Fe(10:11, 12:13) = 0; end   %   off-diagonal (angle) error -> gain rows (tangential force / motion x slope)
            if any(strcmp(opts.fe_cut, 'offpos')); Fe(7:9, 12:13) = 0; end     %   off-diagonal error -> position error (force x a_uw, a_vw)
            if any(strcmp(opts.fe_cut, 'kap')); Fe(12:13, 18) = 0; end         %   curvature error -> off-diagonal
            if any(strcmp(opts.fe_cut, 'gainpos')); Fe(7:9, 10:11) = 0; end    %   gain error -> position error (force x a_11, a_33)
        end
        Q33 = Vh' * (kap * (Ac + (1 - lc)^2 * (Abuf(:, :, 1) + Abuf(:, :, 2))) + (1 - lc)^2 * Sn) * Vh;
        cb = Pu(15, :) .* Fe(11, :);  ks11 = K(11, :)';   % 11-01 log only: budget of P(a'_33, a_33): prediction by F_e column (position, a_11, a_33 self, a_uw a_vw, slope) and update (y1 channels, gain readouts)
        L.B15(k, :) = [sum(cb(7:9)), cb(10), cb(11) - Pu(15, 11), sum(cb(12:13)), cb(15), -K(15, 1:3) * Sy(1:3, :) * ks11, -K(15, 4:7) * Sy(4:7, :) * ks11, Pk(15, 11), sum(cb([1:6 14 16:18]))];
        g = zeros(18, 3);  g(7:9, :) = -eye(3);  g(10, :) = x(14) * nn;  g(11, :) = x(15) * nn;
        Q = g * Q33 * g';
        if kp_on && isfield(opts, 'kappa_pfix') && t(k) >= tsc   % 10-30 (user): curvature allowed to change (random walk), same Q/R-ratio design as the
            Q(18, 18) = opts.kappa_pfix^4 * I_kap;                %   slopes: Q = P_fix^2 I, sqrt P_fix = kappa_pfix [1/R] = curvature span of the probe-centre path
        end
        if isfield(opts, 'ap_q') && opts.ap_q
            hd = max(dot(wh, pd_k) / Rr, 1.1);  if curved; hd = max(hd_int, 1.1); end
            pfix = (0.4 * (1.1 / hd)^2)^2;  if isfield(opts, 'ap_pfix_scale'); pfix = pfix * opts.ap_pfix_scale^2; end
            if isfield(opts, 'ap_pfix_scale_scan') && t(k) >= tsc; pfix = pfix * opts.ap_pfix_scale_scan^2; end   % 10-28 test: slope sqrt P_fix x scale from t_scan   % 10-28 test: sqrt P_fix of the slopes x scale
            if isfield(opts, 'ap_pfix_law') && opts.ap_pfix_law && (~isfield(opts, 'ap_pfix_law_from_scan') || t(k) >= tsc)   % 10-29: law only from the scan start (report P_fix before)
                                                                % 10-28 (user): slope P_fix from the plane law a' = (1 - a)^2 / b_true, b_true in [1, 9/8]:
                pfix = ((1 / 9) * (1 - min(xu(10:11), 1)).^2).^2;      %   relative width 1 - 8/9 = 1/9 of the law slope, per slot (a_11 -> slot 14, a_33 -> slot 15)
            end
            Q(14:15, 14:15) = diag(pfix.^2 .* I_ap);
            if isfield(opts, 'freeze_ap11') && opts.freeze_ap11; Q(14, 14) = 0; end   % 10-28 test: tangential slope a'_11 not driven
            if isfield(opts, 'ap_q_off_from') && t(k) >= opts.ap_q_off_from; Q(14:15, 14:15) = 0; end   % 10-28 (user test): slope Q off (e.g. during the scan)
        end
        qtf = inf;  if isfield(opts, 'q_theta_from'); qtf = opts.q_theta_from; end;  if dsc_on && ~(isfield(opts, 'q_theta_off') && opts.q_theta_off); qtf = tsc;  if isfield(opts, 'q_theta_delay'); qtf = tsc + opts.q_theta_delay; end; end   % 10-29 opts.q_theta_off: no angle P_fix; descent mode: angle P_fix from the scan start
        if t(k) >= qtf   % 10-28 (user): angle P_fix on the off-diagonal slots, P_off = (gap)^2 P_theta
            Pft = (opts.sP_fix_theta_deg * pi / 180)^2;  g2 = dgap^2;
            Q(12, 12) = Q(12, 12) + (g2 * cos(x(16))^2 * Pft)^2 * I_off(1);  Q(13, 13) = Q(13, 13) + (g2 * Pft)^2 * I_off(2);
            if mk; Q(12, 12) = Pft^2 * I_off(1);  Q(13, 13) = Pft^2 * I_off(2); end   % angle units directly
        end
        Rw9 = Pu * Fe(9, :)';   % 11-01 log only: sources of P(a_33, delta w_3)[k+1] by group of row 11 of F_e, Q term, K(11,w), P(a_33, delta w_1), P(a_33, delta w_3)
        L.B119(k, :) = [Fe(11, 7:9) * Rw9(7:9), Fe(11, 10) * Rw9(10), Fe(11, 11) * Rw9(11), Fe(11, 12:13) * Rw9(12:13), Fe(11, 15) * Rw9(15), Q(11, 9), K(11, 3), Pk(11, 3), Pk(11, 9), Fe(11, [1:6 14 16:18]) * Rw9([1:6 14 16:18])];
        HPk = H * Pk;  grp = {7:9, 11, 12:13, 15, [1:6 10 14 16:18]};  p6c = zeros(1, 5);  p3c = zeros(1, 5);   % 11-01 log only: chain budget of P(a_33, delta w_i): update of P(11,9), P(11,6) by channel, prediction of P(11,6), P(11,3) by group of row 11
        for ig = 1:5; p6c(ig) = Fe(11, grp{ig}) * Pu(grp{ig}, 9);  p3c(ig) = Fe(11, grp{ig}) * Pu(grp{ig}, 6); end
        L.C119(k, :) = [-K(11, :) .* HPk(:, 9)', -K(11, :) .* HPk(:, 6)', p6c, p3c, Pk(11, 6)];
        fg = {11, 11, 7:9, 12:13, 15};  e12 = zeros(1, 5);  for ig = 1:5; cg = Fe(11, fg{ig});  if ig == 1; cg = 1; elseif ig == 2; cg = Fe(11, 11) - 1; end;  e12(ig) = cg * Pu(fg{ig}, 12); end   % 11-01 log only: budget of P(a_33, a_uw): update by channel, prediction by group (1, a' F_w, position, off-diagonal, slope), P before
        L.E1112(k, :) = [-K(11, :) .* HPk(:, 12)', e12, Pk(11, 12)];
        fi = zeros(5, 18);  fi(1, 11) = 1;  fi(2, 11) = Fe(11, 11) - 1;  fi(3, 7:9) = Fe(11, 7:9);  fi(4, 12:13) = Fe(11, 12:13);  fi(5, 15) = Fe(11, 15);   % 11-01 log only: P(11,9)[k+1] = sum over (row group of 11: 1, a' F_w, position, off-diagonal, slope) x (column group of 9: lc, -F_w, -F_u, -F_v)
        fj = zeros(4, 18);  fj(1, 9) = Fe(9, 9);  fj(2, 11) = Fe(9, 11);  fj(3, 12) = Fe(9, 12);  fj(4, 13) = Fe(9, 13);  D119 = fi * Pu * fj';  L.D119(k, :) = D119(:)';  L.P1112(k, :) = [Pu(11, 11), Pu(11, 12), Pu(9, 9), Pu(12, 12)];
        fx = x;
        fx(1:3) = x(4:6);  fx(4:6) = x(7:9);  fx(7:9) = lc * x(7:9);
        fx(10) = x(10) + x(14) * vMh(3);  fx(11) = x(11) + x(15) * vMh(3);
        if isfield(opts, 'cheat_slope_pred') && opts.cheat_slope_pred   % 10-29 reference arm only: the gain prediction uses the TRUE slopes at the true gap
            hc = max(h_true, 1.0003);  [p1, q1] = calc_correction_functions(hc + 1e-4);  [p0, q0] = calc_correction_functions(hc - 1e-4);
            fx(10) = x(10) + (1 / p1 - 1 / p0) / 2e-4 * vMh(3);  fx(11) = x(11) + (1 / q1 - 1 / q0) / 2e-4 * vMh(3);
        end
        if ap_pred; fx(15) = x(15) - 2 * x(15)^2 / max(1 - x(11), 0.02) * vMh(3); end
        fx(12) = x(12) * (1 + rs * vMh(3));  fx(13) = x(13) * (1 + rs * vMh(3));
        fx(16:17) = x(16:17) + dth;
        if dt_on; fx(16:17) = x(16:17);  thx_k = x(16); end
        Lg = Fe * K;  kpk = x(18);
        L.fb9(k, :) = [Fe(9, 9) * K(9, :) * nu, Fe(9, 11) * K(11, :) * nu, Fe(9, 12) * K(12, :) * nu, Fe(9, 13) * K(13, :) * nu, Lg(9, :) * nu, x(9)];   % 11-01 log only: feedback into delta-w-hat_3: lc K9, -F_w K11, -F_u K12, -F_v K13, total; state
        if isfield(opts, 'dump_k') && k == opts.dump_k             % one-step inputs for check_1026_local_wiring.m
            dump = struct('Fe', Fe, 'xk', x, 'fx', fx, 'vF', vF, 'dpd', Vh' * (pd_kp1 - pd_k) / Rr, 'lc', lc, 'go', go);
        end
        x = fx + Lg * nu;
        if dt_on                                                % angle step from the off-diagonal elements at k+1 (coordinate of V-hat[k]), gap at k
            L.off_k1(k, :) = [x(12) x(13)];                     % a_uw-hat[k+1], a_vw-hat[k+1] (coordinate of V-hat[k], before the angle step)
            cyk = cos(thx_k);  if isfield(opts, 'no_cos') && opts.no_cos; cyk = 1; end   % 10-28 (user test): drop cos theta-hat_x in the theta_y step
            if isfield(opts, 'dthy_scale'); cyk = cyk / opts.dthy_scale; end   % 10-28 test: scale the theta_y step (loop-gain check)
            dth = [0; 0];  if go; dth = [x(13) / den; -x(12) / (den * cyk)]; end
            if mk; dth = [x(13); x(12)];  x(12:13) = 0; end      % MEKF: the small angles themselves, then reset
            x(16:17) = x(16:17) + dth;
            Vone = local_Ry(x(17)) * local_Rx(x(16));         % frame after the measured step only (used to carry the gain matrix)
            if kp_on; x(17) = x(17) + kpk * vMh(1) / cos(thx_k);  x(16) = x(16) - kpk * vMh(2); end   % + predicted turn kappa-hat M (u, v)
        end
        Pk = Fe * Pu * Fe' + Q;  Pk = 0.5 * (Pk + Pk');
        if isfield(opts, 'cut_p1112_from') && t(k) >= opts.cut_p1112_from; Pk(11, 12:13) = 0;  Pk(12:13, 11) = 0; end   % 11-01 diagnostic only: P(a_33, a_uw), P(a_33, a_vw) forced to 0
        if isfield(opts, 'lf_off') && (~isfield(opts, 'lf_from') || t(k) >= opts.lf_from)   % 10-28 (user, Meng diss. lambda_fh): forgetting factor
            sc = ones(18, 1);  sc(12:13) = 1 / sqrt(opts.lf_off);  Pk = diag(sc) * Pk * diag(sc);   %   on the off-diagonal (angle) slots: P_off / lambda,
        end                                                                                          %   cross terms / sqrt(lambda)
        % ---- all estimates in the local coordinate of V-hat[k+1] ----
        Vold = Vh;  Vh = local_Ry(x(17)) * local_Rx(x(16));  C = Vh' * Vold;
        Ca = C;  if kp_on && dt_on; Ca = Vone' * Vold; end      % the gain matrix turns with the predicted wall turn (only the measured step changes it)
        vAh = Ca * local_loc_A(x) * Ca';
        L.rot(k, :) = [(vAh(1,1) + vAh(2,2)) / 2 - x(10), vAh(3,3) - x(11)];   % 11-01 log only: change of a_11, a_33 a full matrix rotation would make
        x(12) = vAh(1,3);  x(13) = vAh(2,3);                    % 11-01 fix: a_11, a_33 unchanged through the frame rotation (1026 tex:185, they are eigenvalues). local_loc_A is only the first-order matrix,
        if isfield(opts, 'rot_diag_legacy') && opts.rot_diag_legacy; x(10) = (vAh(1,1) + vAh(2,2)) / 2;  x(11) = vAh(3,3); end   %   rotating it took a_uw^2 / gap off a_33 every step with no paired add; rot_diag_legacy restores the old rule for comparison
        if mk; x(12:13) = 0; end
        for i = 0:2; x(3*i+1:3*i+3) = C * x(3*i+1:3*i+3); end
        Cb = blkdiag(C, C, C, eye(9));  Pk = Cb * Pk * Cb';
        if kp_on && isfield(opts, 'kappa_from_scan') && opts.kappa_from_scan && t(k) + Ts < tsc   % 10-29 (user): kappa-hat held at 0 (plane), P at the prior,
            x(18) = 0;  Pk(18, :) = 0;  Pk(:, 18) = 0;  Pk(18, 18) = sPk^2;                     %   until the scan starts (no curvature information before)
        end
        % ---- log (theta-hat[k+1]) ----
        ev2 = eig(0.5 * (Pk + Pk'));
        L.h(k) = h_log;  L.thx(k) = x(16);  L.thy(k) = x(17);  L.aT(k) = xu(10);  L.aN(k) = xu(11);  L.apT(k) = xu(14);  L.apN(k) = xu(15);
        L.kap(k) = x(18);  L.auw(k) = x(12);  L.avw(k) = x(13);  L.dth(k, :) = dth' * 180 / pi;  L.dLe(k, :) = (Lg(16:17, :) * nu)' * 180 / pi;  L.dgap(k) = dgap;
        L.a(k, :) = local_six(Ah)';  L.mineig(k) = ev;  L.sPth(k, :) = sqrt(diag(Pk(16:17, 16:17)))' * 180 / pi;
        if th_out; L.sPth(k, :) = [sqrt(Pk(13, 13)), sqrt(Pk(12, 12)) / cos(x(16))] / abs(x(10) - x(11)) * 180 / pi; end
        if mk; L.sPth(k, :) = [sqrt(Pk(13, 13)), sqrt(Pk(12, 12))] * 180 / pi; end
        L.sPa(k, :) = sqrt(diag(Pu(10:11, 10:11)))';  L.Pcond(k) = min(ev2) / max(ev2);
        L.dp(k, :) = dp_log;  [L.aT_tr(k), L.aN_tr(k)] = local_truth(h_log);  if b2; L.aT_tr(k) = aTk;  L.aN_tr(k) = aNk; end
        L.p(k, :) = p_log';  L.pd(k, :) = pd_log';  L.th_tr(k, :) = thT_log';  L.hd_int(k) = hd_int;
        fbuf = [f_d, fbuf(:, 1)];  Abuf = cat(3, Ac, Abuf(:, :, 1));
        pdbuf = [pdbuf(:, 2:end), pd_kp1];  pd_k = pd_kp1;
    end
    r = struct('seed', seed, 't', t, 'opts', opts, 'kend', kend, 'contact', contact, 'n_floor', n_floor, 'A_true', A_true, 'IF', IF, 'dump', dump, 'tsc', tsc, 'rs_tend', rs_tend);  r.CTkap = C_T * kap;  r.Cn_Sn = C_n * diag(Sn)';   % 10-29: readout scale
    f = fieldnames(L);  for i = 1:numel(f); r.(f{i}) = L.(f{i}); end
    r.aT_true = repmat(aTt, N, 1);  r.aN_true = repmat(aNt, N, 1);
    r.thx = r.thx * 180 / pi;  r.thy = r.thy * 180 / pi;  r.thx_eig = r.thx;  r.thy_eig = r.thy;
    r.thx_true = 0;  if isfield(opts, 'theta_x'); r.thx_true = opts.theta_x; end;  r.thy_true = opts.theta_y;
end

function [W, w, pz, V, th] = local_wall_now(wall, p, W, pz0, rb, o)   % 10-28: local tangent plane of the nearest surface point; th = [thx thy] deg
    switch wall
        case 'hemi'                                            % hemisphere of radius rb um, centre on the plane z = pz0
            c = [0; 0; pz0];  dv = p(:) - c;  db = norm(dv) - rb;  dg = p(3) - pz0;
            void = isfield(o, 'hemi_void') && o.hemi_void;   % 10-28 (user): no plane around the hemisphere (void) -> nearest surface is always the sphere
            if void || (db <= dg && dv(3) >= 0); n = dv / norm(dv);  pzw = n' * c + rb; else; n = [0; 0; 1];  pzw = pz0; end
        case 'bead3'                                           % 10-29 (user): bead + glass blended into ONE effective plane-law wall (rounds the
            rbd = 5;  if isfield(o, 'bead_r'); rbd = o.bead_r; end;  lb = 1;  if isfield(o, 'blend_l'); lb = o.blend_l; end   %   concave junction):
            cbd = [0; 0; pz0 + rbd];  d = p(:) - cbd;  hs = [norm(d) - rbd; p(3) - pz0];  ns = [d / norm(d), [0; 0; 1]];   %   w_i = exp(-h_i/l)/sum,
            m0 = min(hs);  e = exp(-(hs - m0) / lb);  wgt = e / sum(e);  heff = m0 - lb * log(sum(e));   %   h_eff = -l log sum exp(-h_i/l) (soft-min),
            n = ns * wgt;  n = n / norm(n);  pzw = n' * p(:) - heff;   %   n_eff = sum w_i n_i (unit); the plant then is the plane law at (h_eff, n_eff)
        case 'fillet'                                          % 10-29 (user): bead (radius bead_r) on the glass z = pz0, concave junction filled by a torus
            rbd = 5;  if isfield(o, 'bead_r'); rbd = o.bead_r; end;  rho = 6;  if isfield(o, 'fillet_rho'); rho = o.fillet_rho; end   %   fillet of tube radius rho, tangent to both:
            Rt = 2 * sqrt(rbd * rho);  q = p(:) - [0; 0; pz0];  rq = norm(q(1:2));  er = [1; 0];  if rq > 1e-12; er = q(1:2) / rq; end   %   ring radius Rt = 2 sqrt(rbd rho)
            ct = [Rt * er; rho];  cs = [0; 0; rbd];  dt = ct - q;                 %   (centre distance rbd + rho); one smooth wall, h = nearest distance
            a_q = atan2(q(3) - rho, rq - Rt);  a_s = atan2(rbd - rho, -Rt);      %   arc of the fillet seen from the tube centre: from straight down (-pi/2)
            inarc = mod(a_q - a_s, 2 * pi) <= mod(-pi / 2 - a_s, 2 * pi);        %   through the corner side to the bead tangency (direction a_s to the bead centre)
            if norm(dt) < rho && inarc; n = dt / norm(dt);  hq = rho - norm(dt);   % concave fillet: normal toward the tube centre
            elseif rq >= Rt; n = [0; 0; 1];  hq = q(3);                         % glass
            else; n = (q - cs) / norm(q - cs);  hq = norm(q - cs) - rbd; end      % bead
            pzw = n' * p(:) - hq;
        case 'bead2'                                           % 10-29: principal normal of the two-wall drag (largest eigenvalue), h = nearest distance
            [Gn, hmin] = local_gamma2(p, o, pz0, 2.25);  [Ve, De] = eig(0.5 * (Gn + Gn'));  [~, im] = max(diag(De));  n = Ve(:, im);
            rbd = 5;  if isfield(o, 'bead_r'); rbd = o.bead_r; end;  cbd = [0; 0; pz0 + rbd];  nout = (p(:) - cbd) / norm(p(:) - cbd);
            if norm(p(:) - cbd) - rbd < p(3) - pz0; ref = nout; else; ref = [0; 0; 1]; end;  if dot(n, ref) < 0; n = -n; end
            pzw = n' * p(:) - hmin * 2.25;
        case 'bend'                                            % as run_t2_3d_reg local_bend_wall
            m = [cosd(o.bend_phi); sind(o.bend_phi)];  ae = o.bend_alpha * pi / 180;  Lxi = o.bend_Lx * m(1);
            xi = m' * p(1:2) - o.bend_xb * m(1);
            if xi <= 0; a = 0;  zw = 0;
            elseif xi <= Lxi; a = ae * xi / Lxi;  zw = (Lxi / ae) * log(cos(a));
            else; a = ae;  zw = (Lxi / ae) * log(cos(ae)) - tan(ae) * (xi - Lxi); end
            n = [sin(a) * m(1); sin(a) * m(2); cos(a)];  pzw = n' * [p(1); p(2); pz0 + zw];
    end
    thx = -asin(max(min(n(2), 1), -1));  thy = atan2(n(1), n(3));
    V = local_Ry(thy) * local_Rx(thx);
    W.V = V;  W.u_hat = V(:, 1);  W.v_hat = V(:, 2);  W.w_hat = V(:, 3);  W.pz = pzw;
    w = V(:, 3);  pz = pzw;  th = [thx; thy] * 180 / pi;
end

function A = local_loc_A(x)                                    % 1026: gain matrix in the local coordinate from (a_par, a_perp, a_uw, a_vw)
    A = [x(10) 0 x(12); 0 x(10) x(13); x(12) x(13) x(11)];
end

function [Ah, Ja, wh, dW] = local_est_A(x)                     % Section 6: A-hat and d a / d(a_par, a_perp, theta_x, theta_y)
    tx = x(14);  ty = x(15);  ap = x(10);  an = x(11);
    wh = [sin(ty) * cos(tx); -sin(tx); cos(ty) * cos(tx)];
    dW = [-sin(ty) * sin(tx), cos(ty) * cos(tx); -cos(tx), 0; -cos(ty) * sin(tx), -sin(ty) * cos(tx)];
    W = wh * wh';  Ah = ap * eye(3) + (an - ap) * W;
    Ja = [local_six(eye(3) - W), local_six(W), ...
          (an - ap) * local_six(dW(:, 1) * wh' + wh * dW(:, 1)'), (an - ap) * local_six(dW(:, 2) * wh' + wh * dW(:, 2)')];
end

function stp = local_circle_step(dz, tk, Ts, Vh)            % 10-21 dither [radius um, f Hz, ramp s]: step [um] of the offset
    Ad = dz(1);  fdd = dz(2);  Tr = dz(3);  Af = @(tt) Ad * min(tt / Tr, 1);     %   A(t)(cos 2 pi f t - 1, sin 2 pi f t) along (u-hat, v-hat)
    c1 = Af(tk) * [cos(2 * pi * fdd * tk) - 1; sin(2 * pi * fdd * tk)];
    c2 = Af(tk + Ts) * [cos(2 * pi * fdd * (tk + Ts)) - 1; sin(2 * pi * fdd * (tk + Ts))];
    stp = (c2(1) - c1(1)) * Vh(:, 1) + (c2(2) - c1(2)) * Vh(:, 2);
end

function r = local_rho(i, k, Ac, Sn, kap, C_T, C_n, cT, cn)   % normalized autocorrelation (lags 1..) of dp_r,i x dp_r,k:
    if i == k                                                  %   same axis: thermal and sensor-noise chains mixed by their variances
        r = (kap * Ac(i, i) * cT + Sn(i, i) * cn) / (kap * Ac(i, i) * C_T + Sn(i, i) * C_n);
    else                                                       %   different axes: sensor noise independent -> thermal chain only
        r = cT / C_T;
    end
    r = r(2:end);
end

function v = local_six(M)                                      % six elements in the order of a-bar (Section 1)
    v = [M(1,1); M(2,2); M(3,3); M(1,2); M(1,3); M(2,3)];
end

function [th, J, ok] = local_theta_of_A(A)                 % 1025 Section 4: plane formulas and d theta / d a (2 x 6)
    hx = A(1,1) - A(3,3);  vx = -2 * A(1,3);  hy = A(2,2) - A(3,3);  vy = -2 * A(2,3);
    ok = (hx^2 + vx^2) > 1e-12 && (hy^2 + vy^2) > 1e-12;
    thy = atan2(vx, hx) / 2;  phi = atan2(vy, hy) / 2;  th = [-atan(tan(phi) * cos(thy)); thy];
    J = zeros(2, 6);
    if ~ok; return; end
    cy = [A(1,3), 0, -A(1,3), 0, -hx, 0] / (hx^2 + vx^2);
    cp = [0, A(2,3), -A(2,3), 0, 0, -hy] / (hy^2 + vy^2);
    Tp = tan(phi);  dn = 1 + Tp^2 * cos(thy)^2;
    J = [-(cos(thy) * (1 + Tp^2) / dn) * cp + (Tp * sin(thy) / dn) * cy; cy];
end

function A = local_sym(a)
    A = [a(1) a(4) a(5); a(4) a(2) a(6); a(5) a(6) a(3)];
end

function A = local_floor(A, lo)                                % eigenvalues floored at lo (control only)
    [E, D] = eig(0.5 * (A + A'));  A = E * diag(max(diag(D), lo)) * E';
end

function [Gn, hmin] = local_gamma2(p, o, pz0, Rr)            % 10-29: two-wall drag, Gamma / gamma_N = I + sum_i V_i (C_i - I) V_i' (plane law per wall)
    rbd = 5;  if isfield(o, 'bead_r'); rbd = o.bead_r; end
    cbd = [0; 0; pz0 + rbd];  d = p(:) - cbd;  hs = [norm(d) - rbd; p(3) - pz0];  ns = [d / norm(d), [0; 0; 1]];
    Gn = eye(3);  hmin = min(hs) / Rr;
    for i = 1:2
        hb = max(hs(i) / Rr, 1.0005);  [cpa, cpe] = calc_correction_functions(hb);  n = ns(:, i);
        Gn = Gn + (cpa - 1) * (eye(3) - n * n') + (cpe - 1) * (n * n');
    end
end

function p = local_step2(p, F, o, pz0, Rr, gN, Ts)          % 10-29: one RK4 step over Ts with the two-wall mobility (the drag changes < 1 %
    Ni = 1;  hh = Ts / Ni;                                     %   over one step: ~0.03 um of travel at a 2.4 um gap)
    for j = 1:Ni
        k1 = local_pdot2(p, F, o, pz0, Rr, gN);  k2 = local_pdot2(p + hh / 2 * k1, F, o, pz0, Rr, gN);
        k3 = local_pdot2(p + hh / 2 * k2, F, o, pz0, Rr, gN);  k4 = local_pdot2(p + hh * k3, F, o, pz0, Rr, gN);
        p = p + hh / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
    end
end

function v = local_pdot2(p, F, o, pz0, Rr, gN)
    Gn = local_gamma2(p, o, pz0, Rr);  v = (gN * Gn) \ F;
end

function [aT, aN] = local_truth(h)
    [cpa, cpe] = calc_correction_functions(max(h, 1.001));  aT = 1 / cpa;  aN = 1 / cpe;
end

function M = local_Rx(a)
    M = [1 0 0; 0 cos(a) -sin(a); 0 sin(a) cos(a)];
end

function M = local_Ry(a)
    M = [cos(a) 0 sin(a); 0 1 0; -sin(a) 0 cos(a)];
end

function [C_T, C_n, cT, cn] = local_coeffs(lc, apd, d)     % as run_t2_3d_reg (impulse responses of dp_r)
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
