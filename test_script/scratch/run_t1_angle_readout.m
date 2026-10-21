% STATUS: SCRATCH -- 2026-10-16, test T1 of the three-axis estimator (user: "開始", results to go into the readout derivation
%   1015_wall_angle_readout.tex once confirmed). Question: is the measured angle from the random component unbiased, and is its
%   per-sample variance the one written in R, when the estimated wall angle is WRONG and the control law uses it?
%   Plant: production primitives (calc_thermal_force, step_dynamics, d = 2 sensor delay, sensor noise [0.62 0.57 3.31] nm),
%   plane wall tilted theta_y = 20 deg (theta_x = 0). Command: along the TRUE normal 22.2 -> 1.2 R in 1 s, then hold.
%   Control law (Section 1, three axes): A-hat = a_nom V-hat diag(a_T, a_T, a_N) V-hat^T with the TRUE gains at the true height
%   (isolates the angle), theta-hat_y FIXED (no estimator): 20 (right), 15, 25, 0 (the estimator's start).
%   Readout every step (1015_estimator_3d_wsonly Section 6, a_cov = 1): v dp_r = V-hat^T dp_r, vA_m = (v dp_r v dp_r^T - C_n v Sigma_n)/(C_T kappa_T),
%   theta_ym = theta-hat_y - [vA_m]_uw / ((a_T - a_N) cos theta-hat_x), theta_xm = theta-hat_x + [vA_m]_vw / (a_T - a_N), a_Tm, a_Nm.
%   Window: hold 1.2-6.0 s. Reported: seed means (5 seeds), the no-echo prediction theta-hat + sin(2 e)/2 (exact rotation, first
%   order = theta_y), per-sample variance of theta_ym vs the Isserlis value, empirical IF (batch means) vs the impulse-response IF.
%   EXPIRES: when the 14-state estimator (T2) exists | run: R = run_t1_angle_readout()
function R = run_t1_angle_readout(opts)
    if nargin < 1; opts = struct(); end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));
    def = struct('seeds', 1:5, 'th_hat', [20 15 25 0], 'theta_y', 20, 'h_hold', 1.2, 'T_desc', 1.0, 'T_hold', 5.0, ...
                 'lambda_c', 0.4, 't_win', 1.2, 'save', true);
    fn = fieldnames(def);
    for i = 1:numel(fn); if ~isfield(opts, fn{i}); opts.(fn{i}) = def.(fn{i}); end; end
    R = struct('opts', opts, 'runs', {{}});
    for th = opts.th_hat
        for s = opts.seeds
            r = local_run(opts, th, s);  R.runs{end+1} = r;
        end
    end
    runs = [R.runs{:}];
    fprintf('\nT1: wall theta_y = %g deg, hold at h = %.2f R, gains known, theta-hat fixed; %d-seed mean +- sd of the seed means\n', opts.theta_y, opts.h_hold, numel(opts.seeds));
    fprintf('theta-hat | mean theta_ym [deg]      | no-echo exact | ratio (ym - hat)/(pred - hat) | mean theta_xm | a_Tm / a_T     | a_Nm / a_N\n');
    for th = opts.th_hat
        S = runs([runs.th_hat] == th);
        m = arrayfun(@(r) r.m_thy, S);  mx = arrayfun(@(r) r.m_thx, S);
        aT = arrayfun(@(r) r.m_aTm / r.aT, S);  aN = arrayfun(@(r) r.m_aNm / r.aN, S);
        e = (opts.theta_y - th) * pi / 180;  pred = th + (sin(2 * e) / 2) * 180 / pi;
        rat = (mean(m) - th) / (pred - th);  if th == opts.theta_y; rat = NaN; end
        fprintf('%9g | %6.2f +- %4.2f (sem %.2f) | %13.2f | %6.3f                       | %+5.2f +- %4.2f | %.3f +- %.3f | %.3f +- %.3f\n', ...
                th, mean(m), std(m), std(m) / sqrt(numel(m)), pred, rat, mean(mx), std(mx), mean(aT), std(aT), mean(aN), std(aN));
    end
    fprintf('\nper-sample variance of theta_ym [rad^2] and IF (window mean over seeds)\n');
    fprintf('theta-hat | var measured | var Isserlis | ratio | IF measured | IF impulse response\n');
    for th = opts.th_hat
        S = runs([runs.th_hat] == th);
        fprintf('%9g | %.4f       | %.4f      | %.3f | %6.1f      | %6.1f\n', th, mean([S.v_thy]), mean([S.v_pred]), ...
                mean([S.v_thy]) / mean([S.v_pred]), mean([S.IF_meas]), mean([S.IF_pred]));
    end
    if opts.save
        fo = fullfile(root, 'test_results', 'temp_3d_wsonly');  if ~exist(fo, 'dir'); mkdir(fo); end
        save(fullfile(fo, 'temp_t1_angle_readout.mat'), 'R');
    end
end

function r = local_run(opts, th_hat_deg, seed)
    clear calc_thermal_force;
    rng(seed);
    cfg = canonical_scenario(0.05, 1.10, 'ramp_tri', 1);
    cfg.lambda_c = opts.lambda_c;  cfg.theta_y = opts.theta_y;  cfg.theta_x = 0;
    params = calc_simulation_params(cfg);  P = params.Value;
    Ts = P.common.Ts;  Rr = P.common.R;  gN = P.common.gamma_N;
    a_nom = Ts / gN;  a_o = a_nom / Rr;  kap = 4 * P.ctrl.k_B * P.ctrl.T * a_o / Rr;
    Sn = diag((cfg.meas_noise_std(:) / Rr).^2);
    lc = cfg.lambda_c;  apd = cfg.a_pd;  d = 2;
    [C_T, C_n, cT, cn] = local_coeffs(lc, apd, d);
    w = P.wall.w_hat(:);  pz = P.wall.pz;
    thx_h = 0;  thy_h = th_hat_deg * pi / 180;
    Vh = local_Ry(thy_h) * local_Rx(thx_h);  uh = Vh(:, 1);  vh = Vh(:, 2);  wh = Vh(:, 3);
    vSn = Vh' * Sn * Vh;

    h0 = (dot(P.common.p0, w) - pz) / Rr;
    T = opts.T_desc + opts.T_hold;  N = round(T / Ts) + 1;  t = (0:N-1)' * Ts;
    hd = max(h0 - (h0 - opts.h_hold) * t / opts.T_desc, opts.h_hold);
    pd = (pz + hd' * Rr) .* w;                                  % 3 x N, along the TRUE normal
    p = pd(:, 1);  pbuf = repmat(p, 1, d + 1);  fbuf = zeros(3, d);  Abuf = repmat(a_nom * eye(3), 1, 1, d);
    dbar = zeros(3, 1);
    L = struct('thy', NaN(N, 1), 'thx', NaN(N, 1), 'aTm', NaN(N, 1), 'aNm', NaN(N, 1), 'aTu', NaN(N, 1), 'h', zeros(N, 1));
    for k = 1:N
        pm = pbuf(:, 1);
        kd = max(k - d, 1);
        dpm = (pd(:, kd) - pm) / Rr;
        dbar = apd * dpm + (1 - apd) * dbar;  dpr = dpm - dbar;
        h_true = max((dot(p, w) - pz) / Rr, 1.001);
        [cpa, cpe] = calc_correction_functions(h_true);  aT = 1 / cpa;  aN = 1 / cpe;
        vd = Vh' * dpr;
        vA = (vd * vd' - C_n * vSn) / (C_T * kap);
        L.aTm(k) = (vA(1, 1) + vA(2, 2)) / 2;  L.aNm(k) = vA(3, 3);  L.aTu(k) = vA(1, 1);   % aTu = the u direction only (x-z plane readout, 1015_wall_angle_readout Section 6)
        L.thy(k) = thy_h - vA(1, 3) / ((aT - aN) * cos(thx_h));
        L.thx(k) = thx_h + vA(2, 3) / (aT - aN);
        L.h(k) = h_true;
        Ah = a_nom * Vh * diag([aT aT aN]) * Vh';
        phat = pm + Abuf(:, :, 1) * fbuf(:, 1) + Abuf(:, :, 2) * fbuf(:, 2);
        pk1 = pd(:, min(k + 1, N));
        f_d = Ah \ (pk1 - phat - lc * (pd(:, k) - phat));
        f_th = calc_thermal_force(p, P);
        p = step_dynamics(p, f_d + f_th, P, Ts);
        pbuf = [pbuf(:, 2:end), p + cfg.meas_noise_std(:) .* randn(3, 1)];
        fbuf = [f_d, fbuf(:, 1)];  Abuf = cat(3, Ah, Abuf(:, :, 1));
    end
    win = t >= opts.t_win;
    r = struct('th_hat', th_hat_deg, 'seed', seed, 't', t, 'h', L.h);
    r.thy = L.thy * 180 / pi;  r.thx = L.thx * 180 / pi;  r.aTm = L.aTm;  r.aNm = L.aNm;
    r.m_thy = mean(r.thy(win));  r.m_thx = mean(r.thx(win));  r.m_aTm = mean(L.aTm(win));  r.m_aNm = mean(L.aNm(win));  r.m_aTu = mean(L.aTu(win));
    hw = mean(L.h(win));  [cpa, cpe] = calc_correction_functions(hw);  r.aT = 1 / cpa;  r.aN = 1 / cpe;  r.h_win = hw;
    % per-sample variance of theta_ym: Var([vA]_uw) = (S_uu S_ww + S_uw^2) / (C_T kappa)^2, S = local covariance of dp_r
    S = Vh' * (C_T * kap * (P.wall.V * diag([r.aT r.aT r.aN]) * P.wall.V') + C_n * Sn) * Vh / (C_T * kap);
    r.v_pred = (S(1, 1) * S(3, 3) + S(1, 3)^2) / ((r.aT - r.aN) * cos(thx_h))^2;
    x = L.thy(win);  r.v_thy = var(x);
    B = 400;  nb = floor(numel(x) / B);  bm = mean(reshape(x(1:nb * B), B, nb), 1);
    r.IF_meas = B * var(bm) / var(x);
    rho_u = (kap * r.aT * cT + Sn(1, 1) * cn) / (kap * r.aT * cT(1) + Sn(1, 1) * cn(1));
    rho_w = (kap * r.aN * cT + Sn(3, 3) * cn) / (kap * r.aN * cT(1) + Sn(3, 3) * cn(1));
    r.IF_pred = 1 + 2 * sum(rho_u(2:end) .* rho_w(2:end));
end

function M = local_Rx(a)
    M = [1 0 0; 0 cos(a) -sin(a); 0 sin(a) cos(a)];
end

function M = local_Ry(a)
    M = [cos(a) 0 sin(a); 0 1 0; -sin(a) 0 cos(a)];
end

function [C_T, C_n, cT, cn] = local_coeffs(lc, apd, d)     % same chain as run_3d_wsonly_flat (impulse responses of dp_r)
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
