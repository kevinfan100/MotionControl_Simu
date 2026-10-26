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
    clear calc_thermal_force;
    rng(seed);
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
    rng(seed);
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
