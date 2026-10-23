% STATUS: SCRATCH -- 2026-10-15 (user: "先幫我測試模擬看看結果"). PROTOTYPE of the three-axis estimator of
%   reference/eq17_analysis/derivation/1015_estimator_3d_wsonly.tex with the wall angles removed (flat wall, theta = 0 known,
%   V = I): 12 states [dx1 dx2 dx3 (x,y,z each), a_T, a_N, w_s]; a_T' = (9/16)/h^2, a_N' = 1/h^2, one shared h = z_d - dz3 - w_s;
%   y = [dp_m (3), a_Tm, a_Nm], single-sample measured input gains (a_cov = 1), R2 with IF from the exact impulse responses.
%   Self-contained loop on the production plant primitives (trajectory_generator, calc_thermal_force, step_dynamics, d = 2
%   sensor delay as run_formC_b); does NOT use formC_b (other session's file) -- numbers are not bit-comparable to motion_ctrl S8.
%   Arms (same seed = same thermal / sensor draws):
%     'joint'  : one filter, x/y measurements update every state (shared w_s two-way)
%     'oneway' : x/y measurements (y1x, y1y, a_Tm) may not update the z group (dz1-3, a_N, w_s) -- Schmidt update, Joseph form;
%                the z group then equals a z-only Section-8-type filter, a_T rides on the z group's w_s-hat
%   Walls: 'plane' (pz 0) and 'shift' (plane slid down 1.05 R: c_par AND c_perp shifted, i.e. the C62 cell law applied to both
%   directions; cell_cperp_delta shifts only c_perp, which would make x/y and z disagree by construction).
%   10-16 (user): opts.num_T = tangential slope numerator, default 9/16 (numerator 1 was tried as T0, '_n1', and reverted:
%   a_T +0.25 at the trough, a_N on the cell 3x worse through the shared w_s). opts.tag names the output .mat ('_n916', '_n1').
%   EXPIRES: when a real 3-D controller exists | run: R = run_3d_wsonly_flat(struct('seeds', 1:5, 'tag', '_n916'))
function R = run_3d_wsonly_flat(opts)
    if nargin < 1; opts = struct(); end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));
    def = struct('seeds', 1:5, 'arms', {{'joint', 'oneway'}}, 'walls', {{'plane', 'shift'}}, 'lat', [0 1], ...
                 'lambda_c', 0.4, 'f_tri', 1, 'sqrtP_ws', 0.3, 'sqrtP_a', 0.01, 'lat_amp_R', 1.0, 'lat_freq', 1, 'save', true, 'num_T', 9/16, 'tag', '');
    fn = fieldnames(def);
    for i = 1:numel(fn); if ~isfield(opts, fn{i}); opts.(fn{i}) = def.(fn{i}); end; end
    R = struct('opts', opts, 'runs', {{}});
    for w = 1:numel(opts.walls)
        for l = opts.lat
            for a = 1:numel(opts.arms)
                for s = opts.seeds
                    r = local_run(opts, opts.arms{a}, opts.walls{w}, l, s);
                    R.runs{end+1} = r;
                    fprintf('%-6s %-6s lat %d seed %d: min h %.3f, floor hits %d, NaN %d, end w_s-hat %+.3f\n', ...
                            r.wall, r.arm, r.lat, r.seed, min(r.h_true), r.n_floor, r.n_nan, r.ws_hat(end));
                end
            end
        end
    end
    if opts.save
        fo = fullfile(root, 'test_results', 'temp_3d_wsonly');  if ~exist(fo, 'dir'); mkdir(fo); end
        save(fullfile(fo, sprintf('temp_3d_wsonly_runs%s.mat', opts.tag)), 'R');
    end
end

function r = local_run(opts, arm, wall, lat, seed)
    clear trajectory_generator calc_thermal_force;
    rng(seed);
    cfg = canonical_scenario(0.05, 1.10, 'ramp_tri', opts.f_tri);
    cfg.lambda_c = opts.lambda_c;
    if lat
        pc = physical_constants();
        cfg.amp_u = opts.lat_amp_R * pc.R;  cfg.freq_u = opts.lat_freq;
        cfg.amp_v = opts.lat_amp_R * pc.R;  cfg.freq_v = opts.lat_freq;  cfg.tan_shape = 1;
    end
    params = calc_simulation_params(cfg);  P = params.Value;
    Ts = P.common.Ts;  Rr = P.common.R;  gN = P.common.gamma_N;
    a_nom = Ts / gN;  a_o = a_nom / Rr;  kBT = P.ctrl.k_B * P.ctrl.T;
    kap = 4 * kBT * a_o / Rr;                                 % one-step thermal variance / R^2 per unit a-bar
    sig2 = (cfg.meas_noise_std(:) / Rr).^2;                   % sensor variance / R^2, per axis
    lc = cfg.lambda_c;  apd = cfg.a_pd;  d = 2;
    [C_T, C_n, cT, cn] = local_coeffs(lc, apd, d);
    xi = C_n * sig2 / (C_T * kap);

    P_plant = P;  ws_true = 0;
    if strcmp(wall, 'shift'); ws_true = -1.05;  P_plant.wall.pz = P.wall.pz + ws_true * Rr; end

    N = round(cfg.T_sim / Ts) + 1;  t = (0:N-1)' * Ts;
    p = P.common.p0;  pbuf = repmat(p, 1, d + 1);  pd_k = p;  pdbuf = repmat(p, 1, d + 1);
    fbuf = zeros(3, d);  Abuf = zeros(3, d);                  % f_d[k-1], f_d[k-2] [pN]; a-hat diag at k-1, k-2 [a-bar]

    % estimator init (plane law at the start height, w_s-hat = 0 = visible surface)
    h0 = (pd_k(3) - P.wall.pz) / Rr;
    [cpa0, cpe0] = calc_correction_functions(h0);
    x = zeros(12, 1);  x(10) = 1 / cpa0;  x(11) = 1 / cpe0;  x(12) = 0;
    Pk = diag([repmat(sig2, 3, 1); opts.sqrtP_a^2; opts.sqrtP_a^2; opts.sqrtP_ws^2]);
    Abuf(:) = repmat([x(10); x(10); x(11)], 1, d);
    dpm_bar = zeros(3, 1);
    zgrp = [3 6 9 11 12];  xy_meas = [1 2 4];                  % z group states; y rows 1 (x), 2 (y), 4 (a_Tm)

    L = struct('aT', zeros(N,1), 'aN', zeros(N,1), 'ws', zeros(N,1), 'aT_true', zeros(N,1), 'aN_true', zeros(N,1), ...
               'h', zeros(N,1), 'hd', zeros(N,1), 'sP', zeros(N,3), 'aTm', zeros(N,1), 'aNm', zeros(N,1), 'p', zeros(N,3), 'pd', zeros(N,3), 'dpm', zeros(N,3));
    n_floor = 0;
    for k = 1:N
        [pd_kp1, ~] = trajectory_generator(t(k), P);
        % ---- measurement (sensor delay d) ----
        pm = pbuf(:, 1);
        dpm = (pdbuf(:, 1) - pm) / Rr;                         % delta p_m[k] = p_d[k-d] - p_m[k], / R
        dpm_bar = apd * dpm + (1 - apd) * dpm_bar;
        dpr = dpm - dpm_bar;
        aTm = ((dpr(1)^2 - C_n * sig2(1)) + (dpr(2)^2 - C_n * sig2(2))) / (2 * C_T * kap);
        aNm = (dpr(3)^2 - C_n * sig2(3)) / (C_T * kap);
        y = [dpm; aTm; aNm];
        H = zeros(5, 12);  H(1:3, 1:3) = eye(3);  H(4, 10) = 1;  H(5, 11) = 1;
        IFx = local_IF(x(10), kap, sig2(1), cT, cn);  IFy = local_IF(x(10), kap, sig2(2), cT, cn);  IFz = local_IF(x(11), kap, sig2(3), cT, cn);
        rT = 0.5 * (IFx * (x(10) + xi(1))^2 + IFy * (x(10) + xi(2))^2);
        rN = 2 * IFz * (x(11) + xi(3))^2;
        Rm = [sig2; rT; rN];
        % ---- update: z measurements first (full), then x/y (arm-dependent) ----
        for j = [3 5 1 2 4]
            h_row = H(j, :);  S = h_row * Pk * h_row' + Rm(j);
            K = Pk * h_row' / S;
            if strcmp(arm, 'oneway') && any(j == xy_meas); K(zgrp) = 0; end
            x = x + K * (y(j) - h_row * x);
            IKH = eye(12) - K * h_row;
            Pk = IKH * Pk * IKH' + K * Rm(j) * K';
        end
        aT = max(x(10), 0.02);  aN = max(x(11), 0.02);  if aT ~= x(10) || aN ~= x(11); n_floor = n_floor + 1; end
        % ---- control law (Section 1, three axes): f = A^-1 {p_d[k+1] - p^ - lc (p_d[k] - p^)} ----
        Ahat = a_nom * [aT; aT; aN];
        phat = pm + sum(a_nom * Abuf .* fbuf, 2);
        f_d = (pd_kp1 - phat - lc * (pd_k - phat)) ./ Ahat;
        % ---- plant ----
        p_k = p;                                                % true position at k: the gain the estimate at k refers to
        f_th = calc_thermal_force(p, P_plant);
        p = step_dynamics(p, f_d + f_th, P_plant, Ts);
        n_meas = cfg.meas_noise_std(:) .* randn(3, 1);
        pbuf = [pbuf(:, 2:end), p + n_meas];
        % ---- log (true gains at the position the estimate refers to: k) ----
        h_true = max((p_k(3) - P_plant.wall.pz) / Rr, 1.001);   % 1.001 = log floor (physical), not h_min
        L.p(k, :) = p_k';
        [cpa, cpe] = calc_correction_functions(h_true);
        L.aT(k) = x(10);  L.aN(k) = x(11);  L.ws(k) = x(12);  L.aT_true(k) = 1 / cpa;  L.aN_true(k) = 1 / cpe;
        L.h(k) = h_true;  L.hd(k) = (pd_k(3) - P.wall.pz) / Rr;  L.sP(k, :) = sqrt([Pk(10,10) Pk(11,11) Pk(12,12)]);
        L.aTm(k) = aTm;  L.aNm(k) = aNm;  L.pd(k, :) = pd_k';  L.dpm(k, :) = dpm' * Rr;   % [um]
        % ---- predict ----
        Fb = a_o * (f_d + (1 - lc) * sum(fbuf, 2));            % F_dp (normalized)
        Mz = (pd_kp1(3) - pd_k(3)) / Rr + (1 - lc) * x(9);
        hh = max((pd_k(3) - P.wall.pz) / Rr - x(9) - x(12), 1.0);
        aTp = opts.num_T / hh^2;  aNp = 1 / hh^2;
        q33 = kap * ([aT; aT; aN] + (1 - lc)^2 * sum(Abuf, 2)) + (1 - lc)^2 * sig2;
        Fe = zeros(12);  Fe(1:3, 4:6) = eye(3);  Fe(4:6, 7:9) = eye(3);  Fe(7:9, 7:9) = lc * eye(3);
        Fe(7, 10) = -Fb(1);  Fe(8, 10) = -Fb(2);  Fe(9, 11) = -Fb(3);
        Fe(10, 9) = aTp * (1 - lc) + 2 * aTp / hh * Mz;  Fe(10, 10) = 1;  Fe(10, 11) = aTp * Fb(3);  Fe(10, 12) = 2 * aTp / hh * Mz;
        Fe(11, 9) = aNp * (1 - lc) + 2 * aNp / hh * Mz;  Fe(11, 11) = 1 + aNp * Fb(3);           Fe(11, 12) = 2 * aNp / hh * Mz;
        Fe(12, 12) = 1;
        Q = zeros(12);  Q(7, 7) = q33(1);  Q(8, 8) = q33(2);
        g = [-1; aTp; aNp];  idx = [9 10 11];  Q(idx, idx) = q33(3) * (g * g');      % q = [-eps_z; a_T' eps_z; a_N' eps_z]
        x([1 2 3]) = x([4 5 6]);  x([4 5 6]) = x([7 8 9]);  x([7 8 9]) = lc * x([7 8 9]);
        x(10) = x(10) + aTp * Mz;  x(11) = x(11) + aNp * Mz;
        Pk = Fe * Pk * Fe' + Q;  Pk = 0.5 * (Pk + Pk');
        fbuf = [f_d, fbuf(:, 1:end-1)];  Abuf = [[aT; aT; aN], Abuf(:, 1:end-1)];
        pdbuf = [pdbuf(:, 2:end), pd_kp1];  pd_k = pd_kp1;
    end
    r = struct('arm', arm, 'wall', wall, 'lat', lat, 'seed', seed, 't', t, 'ws_true', ws_true, 'C_T', C_T, 'C_n', C_n, 'xi', xi, 'num_T', opts.num_T);
    f = fieldnames(L);  for i = 1:numel(f); r.(f{i}) = L.(f{i}); end
    r.h_true = L.h;  r.ws_hat = L.ws;  r.n_floor = n_floor;  r.n_nan = sum(~isfinite(L.aT + L.aN + L.ws));
end

function [C_T, C_n, cT, cn] = local_coeffs(lc, apd, d)
% Impulse responses of the random component dp_r to one thermal displacement (tau) and to one sensor sample (n),
% closed loop with A-hat = A: dp[k+1] = lc dp[k] - eps[k], eps = tau[k] + (1-lc) sum_{i=1..d} tau[k-i] - (1-lc) n[k],
% dp_m[k] = dp[k-d] - n[k], dp_r = dp_m - EWMA_apd(dp_m). Returns the variance factors and autocovariances (lags 0..Lg).
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
    dpm = zeros(n, 1);  bar = 0;  g = zeros(n, 1);
    for k = 1:n
        prev = 0;  if k - d >= 1; prev = dp(k - d); end
        dpm(k) = prev - nz(k);
        bar = apd * dpm(k) + (1 - apd) * bar;
        g(k) = dpm(k) - bar;
    end
end

function IF = local_IF(a, kap, s2, cT, cn)
    c = kap * max(a, 0.02) * cT + s2 * cn;
    rho = c(2:end) / c(1);
    IF = 1 + 2 * sum(rho.^2);
end
