% STATUS: SCRATCH -- 2026-10-18, test T1b of the three-axis estimator (user: "先做 T1b"): estimate ONLY the wall angles, motion
%   gains known. Same plant and command as T1 (run_t1_angle_readout): plane wall theta_y = 20 deg (theta_x = 0), along the TRUE normal
%   22.2 -> 1.2 R in 1 s, then held 5 s; production primitives, sensor noise [0.62 0.57 3.31] nm, lambda_c 0.4.
%   Estimator: theta_x, theta_y constant (q = 0), F = 1, measurement theta_m with H = 1 (1015_estimator_3d_wsonly Sections 6-7),
%   r_theta = IF (a_T + xi_u)(a_N + xi_w) / (a_T - a_N)^2 (/cos^2 theta-hat_x for theta_y), xi from v Sigma_n of the current theta-hat,
%   IF from the impulse responses (1 + 2 sum rho_u rho_w); gains in r and in the readout = TRUE gains at the true height.
%   theta-hat[0] = 0, sqrt P_theta[0] = opts.sP0_deg (provisional, no basis yet). Order per step: readout with theta-hat[k] -> KF update
%   -> control law with the updated theta-hat (A-hat = a_nom V-hat diag(a_T, a_T, a_N) V-hat^T, true gains).
%   Logged per step: theta-hat_x/y, sqrt P, a_Nm, a_Tm (u direction), h. Output test_results/temp_3d_wsonly/temp_t1b_angle_only.mat.
%   EXPIRES: when T2 (14-state) exists | run: matlab -batch "addpath('test_script/scratch'); run_t1b_angle_only();"
function R = run_t1b_angle_only(opts)
    if nargin < 1; opts = struct(); end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));  addpath(here);
    def = struct('seeds', 1:15, 'theta_y', 20, 'h_hold', 1.2, 'T_desc', 1.0, 'T_hold', 5.0, 'lambda_c', 0.4, 'sP0_deg', 30);
    fn = fieldnames(def);
    for i = 1:numel(fn); if ~isfield(opts, fn{i}); opts.(fn{i}) = def.(fn{i}); end; end
    R = struct('opts', opts, 'runs', {{}});
    for s = opts.seeds
        R.runs{end+1} = local_run(opts, s);
        r = R.runs{end};
        fprintf('seed %2d: theta-hat_y end %.2f deg (sqrtP %.2f), theta-hat_x end %+.2f, |e|<2 deg from t = %.3f s\n', ...
                s, r.thy(end), r.sPy(end), r.thx(end), r.t_conv);
    end
    fo = fullfile(root, 'test_results', 'temp_3d_wsonly');  if ~exist(fo, 'dir'); mkdir(fo); end
    save(fullfile(fo, 'temp_t1b_angle_only.mat'), 'R');
    runs = [R.runs{:}];  t = runs(1).t;
    fprintf('\n%d seeds: theta-hat_y end %.2f +- %.2f deg (mean sqrtP %.2f), theta-hat_x end %+.2f +- %.2f\n', numel(runs), ...
            mean(arrayfun(@(r) r.thy(end), runs)), std(arrayfun(@(r) r.thy(end), runs)), mean(arrayfun(@(r) r.sPy(end), runs)), ...
            mean(arrayfun(@(r) r.thx(end), runs)), std(arrayfun(@(r) r.thx(end), runs)));
    fprintf('time to |e_theta| < 2 deg for good: %.3f +- %.3f s (min %.3f, max %.3f)\n', mean([runs.t_conv]), std([runs.t_conv]), min([runs.t_conv]), max([runs.t_conv]));
    for tt = [0.9 1.0 1.05 1.1 1.2 1.5 2 3 6]
        k = find(t >= tt - 1e-9, 1);
        e = arrayfun(@(r) opts.theta_y - r.thy(k), runs);  sp = arrayfun(@(r) r.sPy(k), runs);
        fprintf('t %.2f s: e_theta %+6.2f +- %5.2f deg, sqrtP %5.2f deg, honesty sd/sqrtP %.2f\n', tt, mean(e), std(e), mean(sp), std(e) / mean(sp));
    end
end

function r = local_run(opts, seed)
    clear calc_thermal_force;
    rng(seed);
    cfg = canonical_scenario(0.05, 1.10, 'ramp_tri', 1);
    cfg.lambda_c = opts.lambda_c;  cfg.theta_y = opts.theta_y;  cfg.theta_x = 0;
    params = calc_simulation_params(cfg);  P = params.Value;
    Ts = P.common.Ts;  Rr = P.common.R;  a_nom = Ts / P.common.gamma_N;  a_o = a_nom / Rr;
    kap = 4 * P.ctrl.k_B * P.ctrl.T * a_o / Rr;  Sn = diag((cfg.meas_noise_std(:) / Rr).^2);
    lc = cfg.lambda_c;  apd = cfg.a_pd;  d = 2;
    [C_T, C_n, cT, cn] = local_coeffs(lc, apd, d);
    w = P.wall.w_hat(:);  pz = P.wall.pz;
    h0 = (dot(P.common.p0, w) - pz) / Rr;
    N = round((opts.T_desc + opts.T_hold) / Ts) + 1;  t = (0:N-1)' * Ts;
    hd = max(h0 - (h0 - opts.h_hold) * t / opts.T_desc, opts.h_hold);
    pd = (pz + hd' * Rr) .* w;
    p = pd(:, 1);  pbuf = repmat(p, 1, d + 1);  fbuf = zeros(3, d);  Abuf = repmat(a_nom * eye(3), 1, 1, d);  dbar = zeros(3, 1);
    th = [0; 0];  Pth = (opts.sP0_deg * pi / 180)^2 * [1; 1];      % [theta_x; theta_y]
    L = struct('thx', zeros(N, 1), 'thy', zeros(N, 1), 'sPx', zeros(N, 1), 'sPy', zeros(N, 1), 'aNm', zeros(N, 1), 'aTu', zeros(N, 1), ...
               'aT', zeros(N, 1), 'aN', zeros(N, 1), 'h', zeros(N, 1), 'z', zeros(N, 1), 'zd', zeros(N, 1));
    for k = 1:N
        pm = pbuf(:, 1);  dpm = (pd(:, max(k - d, 1)) - pm) / Rr;
        dbar = apd * dpm + (1 - apd) * dbar;  dpr = dpm - dbar;
        h_true = max((dot(p, w) - pz) / Rr, 1.001);
        [cpa, cpe] = calc_correction_functions(h_true);  aT = 1 / cpa;  aN = 1 / cpe;
        % ---- readout in the local coordinate of theta-hat[k] ----
        Vh = local_Ry(th(2)) * local_Rx(th(1));  vSn = Vh' * Sn * Vh;
        vd = Vh' * dpr;  vA = (vd * vd' - C_n * vSn) / (C_T * kap);
        thm = [th(1) + vA(2, 3) / (aT - aN);  th(2) - vA(1, 3) / ((aT - aN) * cos(th(1)))];
        % ---- r_theta (true gains, noise in the current local coordinate, IF from the impulse responses) ----
        xi = C_n * diag(vSn) / (C_T * kap);
        rho = @(a, s2) (kap * a * cT + s2 * cn) / (kap * a * cT(1) + s2 * cn(1));
        ru = rho(aT, vSn(1, 1));  rv = rho(aT, vSn(2, 2));  rw = rho(aN, vSn(3, 3));
        IFu = 1 + 2 * sum(ru(2:end) .* rw(2:end));  IFv = 1 + 2 * sum(rv(2:end) .* rw(2:end));
        r_th = [IFv * (aT + xi(2)) * (aN + xi(3)) / (aT - aN)^2;  IFu * (aT + xi(1)) * (aN + xi(3)) / ((aT - aN)^2 * cos(th(1))^2)];
        % ---- KF update (q = 0, F = 1, H = 1) ----
        K = Pth ./ (Pth + r_th);  th = th + K .* (thm - th);  Pth = (1 - K) .* Pth;
        L.thx(k) = th(1);  L.thy(k) = th(2);  L.sPx(k) = sqrt(Pth(1));  L.sPy(k) = sqrt(Pth(2));
        L.aNm(k) = vA(3, 3) / aN;  L.aTu(k) = vA(1, 1) / aT;  L.aT(k) = aT;  L.aN(k) = aN;  L.h(k) = h_true;
        L.z(k) = p(3);  L.zd(k) = pd(3, k);
        % ---- control law with the updated angle, true gains ----
        Vc = local_Ry(th(2)) * local_Rx(th(1));
        Ah = a_nom * Vc * diag([aT aT aN]) * Vc';
        phat = pm + Abuf(:, :, 1) * fbuf(:, 1) + Abuf(:, :, 2) * fbuf(:, 2);
        f_d = Ah \ (pd(:, min(k + 1, N)) - phat - lc * (pd(:, k) - phat));
        f_th = calc_thermal_force(p, P);
        p = step_dynamics(p, f_d + f_th, P, Ts);
        pbuf = [pbuf(:, 2:end), p + cfg.meas_noise_std(:) .* randn(3, 1)];
        fbuf = [f_d, fbuf(:, 1)];  Abuf = cat(3, Ah, Abuf(:, :, 1));
    end
    r = struct('seed', seed, 't', t);
    f = fieldnames(L);  for i = 1:numel(f); r.(f{i}) = L.(f{i}); end
    r.thx = r.thx * 180 / pi;  r.thy = r.thy * 180 / pi;  r.sPx = r.sPx * 180 / pi;  r.sPy = r.sPy * 180 / pi;
    bad = find(abs(opts.theta_y - r.thy) >= 2, 1, 'last');
    if isempty(bad); r.t_conv = 0; elseif bad == N; r.t_conv = NaN; else; r.t_conv = t(bad + 1); end
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
