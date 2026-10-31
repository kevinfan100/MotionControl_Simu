% FORK OF test_script/scratch/run_t2_3d_reg.m @ 6be7fe3 | PURPOSE: arm B of the 10-25 Meng Sec. 5.5 comparison -- the estimator
%   STRUCTURE of Meng diss. Ch. 5 (5.9)-(5.14), (5.20)-(5.26) without the disturbance, run with OUR controller, readout chain, R and P_fix
%   rules (plant, sensor and random-number order identical to run_t2_3d_reg -> paired seeds) | EXPIRES: when the A / B comparison is closed |
%   產線改動不會自動跟上
% States (17): dp1 dp2 dp3 (measurement coordinate, as ours) | lambda_u lambda_v lambda_w | d-lambda (3, Meng's rate states) |
%   theta_Lambda_1, theta_Lambda_2 = off-diagonal (v, w) and (u, w) of V-hat' A-bar V-hat = gain difference x small angle (5.9)-(5.10).
%   Dropped: theta_Lambda_3 (about w, lambda_u = lambda_v so not defined) and the angle rate d-theta_Lambda (plane: angle constant).
% Each step: readout in the V-hat frame, single sample (a_cov = 1) -> sequential Kalman update (R as run_t2_3d_reg, per axis) ->
%   rotate V-hat (5.13) by dtheta_1 = theta-hat_L1 / (l_v - l_w), dtheta_2 = theta-hat_L2 / (l_w - l_u) (5.14) and reset theta-hat_Lambda
%   to 0 (increment reading, user 10-25) -> our control law with A-hat = V-hat diag(lambda-hat) V-hat' -> predict.
% Settings (ours; Meng gives none): sqrt P0 of theta_Lambda = (9/16)/h0 x sqrt P_theta[0] (far-field gain difference x angle prior);
%   lambda: prior (9/8)/h0, Q = 0; d-lambda: prior and sqrt P_fix = (9/8)/h_d^2 x |commanded step| (expected gain change per step at the
%   commanded height h_d = w-hat' p-bar_d, our P_fix idea; 10-25 fix: a constant 1e-3 let the three gains wander and cross), Q = P_fix^2 I[k]; lambda_f = 1; rotation skipped while |l_i - l_j| < 2 % of max l (Meng notes' eps, value ours).
% Scenario options as run_t2_3d_reg: theta_y [deg], h0 [R], approach [um, um/s], t_end [s], sP_theta_deg, seeds, tag.
%   run: run_t4_meng_est(struct('theta_y', 45, 'h0', 15/2.25, 'approach', [11.5 5], 't_end', 2.3, 'sP_theta_deg', 20, 'seeds', 2, 'tag', 'meng55B_s2'))
function R = run_t4_meng_est(opts)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));  addpath(here);
    def = struct('seeds', 1, 'theta_y', 45, 'lambda_c', 0.4, 'sP_theta_deg', 20, 'tag', 'meng55B', 'save', true, 't_end', 2.3);
    fn = fieldnames(def);
    for i = 1:numel(fn); if ~isfield(opts, fn{i}); opts.(fn{i}) = def.(fn{i}); end; end
    R = struct('opts', opts, 'runs', {{}});
    for s = opts.seeds
        r = local_run(opts, s);  R.runs{end+1} = r;
        fprintf('%s seed %d: contact %d, end theta-hat_y %.2f (x %.2f), a-hat_N %.3f vs %.3f, a-hat_T %.3f vs %.3f\n', opts.tag, s, r.contact, ...
                r.thy(r.kend), r.thx(r.kend), r.aN(r.kend), r.aN_true(r.kend), r.aT(r.kend), r.aT_true(r.kend));
    end
    if opts.save
        save(fullfile(root, 'test_results', 'temp_3d_wsonly', ['temp_t2_' opts.tag '.mat']), 'R');
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
    w_true = P.wall.w_hat(:);  pz = P.wall.pz;
    N = round(opts.t_end / Ts) + 1;  t = (0:N-1)' * Ts;
    p = (pz + opts.h0 * Rr) * w_true;  pd_k = p;
    % ---- estimator init ----
    hd0 = p(3) / Rr;  sig2 = diag(Sn);
    x = zeros(17, 1);  x(10:12) = 1;  Vm = eye(3);
    vstep = 0;  if isfield(opts, 'approach'); vstep = opts.approach(2) * Ts / Rr; end   % commanded step [R] during the approach
    sdl0 = (9/8) / hd0^2 * vstep;
    Pk = diag([repmat(sig2, 3, 1); ((9/8) / hd0)^2 * [1; 1; 1]; max(sdl0, 1e-12)^2 * [1; 1; 1]; ((9/16) / hd0 * opts.sP_theta_deg * pi / 180)^2 * [1; 1]]);
    pbuf = repmat(p, 1, d + 1);  pdbuf = repmat(pd_k, 1, d + 1);  fbuf = zeros(3, d);  Abuf = repmat(eye(3), 1, 1, d);  dbar = zeros(3, 1);
    L = struct('h', NaN(N,1), 'aT_true', NaN(N,1), 'aN_true', NaN(N,1), 'aT', NaN(N,1), 'aN', NaN(N,1), 'lam', NaN(N,3), ...
               'thx', NaN(N,1), 'thy', NaN(N,1), 'thx_true', NaN(N,1), 'thy_true', NaN(N,1), 'sP', NaN(N,6), 'p', NaN(N,3), 'pd', NaN(N,3), ...
               'n_skip', zeros(N,1));
    contact = false;  t_contact = NaN;  kend = N;
    for k = 1:N
        h_true = (dot(p, w_true) - pz) / Rr;
        if h_true < 1; contact = true;  t_contact = t(k);  kend = k - 1;  break; end
        [cpa, cpe] = calc_correction_functions(max(h_true, 1.001));  aTt = 1 / cpa;  aNt = 1 / cpe;
        % ---- readout in the V-hat frame (single sample) ----
        pm = pbuf(:, 1);  dpm = (pdbuf(:, 1) - pm) / Rr;
        dbar = apd * dpm + (1 - apd) * dbar;  dpr = dpm - dbar;
        uh = Vm(:, 1);  vh = Vm(:, 2);  wh = Vm(:, 3);
        vSn = Vm' * Sn * Vm;  vd = Vm' * dpr;  vA = (vd * vd' - C_n * vSn) / (C_T * kap);
        lf = max(x(10:12), 0.02);  xi = C_n * diag(vSn) / (C_T * kap);
        rho = @(a, s2) (kap * a * cT + s2 * cn) / (kap * a * cT(1) + s2 * cn(1));
        ru = rho(lf(1), vSn(1, 1));  rv = rho(lf(2), vSn(2, 2));  rw = rho(lf(3), vSn(3, 3));
        IFsq = @(r) 1 + 2 * sum(r(2:end).^2);  IFx = @(r1, r2) 1 + 2 * sum(r1(2:end) .* r2(2:end));
        y  = [dpm; vA(1,1); vA(2,2); vA(3,3); vA(2,3); vA(1,3)];
        Hm = zeros(8, 17);  Hm(1:3, 1:3) = eye(3);  Hm(4, 10) = 1;  Hm(5, 11) = 1;  Hm(6, 12) = 1;  Hm(7, 16) = 1;  Hm(8, 17) = 1;
        Rm = [sig2; 2 * IFsq(ru) * (lf(1) + xi(1))^2; 2 * IFsq(rv) * (lf(2) + xi(2))^2; 2 * IFsq(rw) * (lf(3) + xi(3))^2; ...
              IFx(rv, rw) * (lf(2) + xi(2)) * (lf(3) + xi(3)); IFx(ru, rw) * (lf(1) + xi(1)) * (lf(3) + xi(3))];
        pf_dl = diag(Pk(13:15, 13:15));
        for j = 1:8                                            % sequential scalar updates, Joseph form
            hj = Hm(j, :);  S = hj * Pk * hj' + Rm(j);
            K = Pk * hj' / S;  x = x + K * (y(j) - hj * x);
            IKH = eye(17) - K * hj;  Pk = IKH * Pk * IKH' + K * Rm(j) * K';
        end
        I_dl = max(1 ./ diag(Pk(13:15, 13:15)) - 1 ./ pf_dl, 0);
        % ---- rotate V-hat (5.13)-(5.14), reset theta-hat_Lambda ----
        l = x(10:12);  dth = [0; 0];  d1 = l(2) - l(3);  d2 = l(3) - l(1);
        thr = [0.02; 0.02] * max(l);                           % default: Meng notes' eps (value ours)
        if isfield(opts, 'rot_gate') && opts.rot_gate          % 10-25 option: rotate only while |l_i - l_j| > 3 sigma of it (from P; our 10-20 angle gate)
            thr = 3 * sqrt(max([Pk(11,11) + Pk(12,12) - 2 * Pk(11,12); Pk(12,12) + Pk(10,10) - 2 * Pk(10,12)], 0));
        end
        if abs(d1) > thr(1); dth(1) = x(16) / d1;  x(16) = 0; else; L.n_skip(k) = 1; end
        if abs(d2) > thr(2); dth(2) = x(17) / d2;  x(17) = 0; else; L.n_skip(k) = 1; end
        Vm = Vm * [1 0 dth(2); 0 1 -dth(1); -dth(2) dth(1) 1];  [Uo, ~, Wo] = svd(Vm);  Vm = Uo * Wo';
        % ---- our control law ----
        uh = Vm(:, 1);  vh = Vm(:, 2);  wh = Vm(:, 3);  lf = max(x(10:12), 0.02);
        Ab = Vm * diag(lf) * Vm';
        step = zeros(3, 1);
        if isfield(opts, 'approach') && t(k) < opts.approach(1) / opts.approach(2)
            step = -opts.approach(2) * Ts / Rr * w_true;     % Meng Sec. 5.5: toward the wall along the true normal
        end
        pd_kp1 = pd_k + step * Rr;
        phat = pm + a_nom * (Abuf(:, :, 1) * fbuf(:, 1) + Abuf(:, :, 2) * fbuf(:, 2));
        f_d = (a_nom * Ab) \ (pd_kp1 - phat - lc * (pd_k - phat));
        % ---- log ----
        L.h(k) = h_true;  L.aT_true(k) = aTt;  L.aN_true(k) = aNt;  L.lam(k, :) = x(10:12)';
        L.aT(k) = (x(10) + x(11)) / 2;  L.aN(k) = x(12);
        L.thx(k) = -asind(max(min(wh(2), 1), -1));  L.thy(k) = atan2d(wh(1), wh(3));  L.thx_true(k) = 0;  L.thy_true(k) = opts.theta_y;
        sth = sqrt([Pk(16, 16); Pk(17, 17)]) ./ max(abs([d1; d2]), 1e-6) * 180 / pi;
        L.sP(k, :) = [sqrt(diag(Pk(10:12, 10:12)))', NaN, sth'];   % columns 5, 6 = theta_x, theta_y [deg] as run_t2_3d_reg  L.p(k, :) = p';  L.pd(k, :) = pd_k';
        % ---- plant (same calls and order as run_t2_3d_reg) ----
        f_th = calc_thermal_force(p, P);
        try
            p = step_dynamics(p, f_d + f_th, P, Ts);
        catch
            contact = true;  t_contact = t(k);  kend = k;  break
        end
        pbuf = [pbuf(:, 2:end), p + cfg.meas_noise_std(:) .* randn(3, 1)];
        % ---- predict ----
        Fb = a_o * (f_d + (1 - lc) * sum(fbuf, 2));
        Fe = eye(17);  Fe(1:9, 1:9) = 0;  Fe(1:3, 4:6) = eye(3);  Fe(4:6, 7:9) = eye(3);  Fe(7:9, 7:9) = lc * eye(3);
        Fe(7:9, 10) = -(uh * uh') * Fb;  Fe(7:9, 11) = -(vh * vh') * Fb;  Fe(7:9, 12) = -(wh * wh') * Fb;   % gains enter the position error
        Fe(7:9, 16) = -(vh * wh' + wh * vh') * Fb;  Fe(7:9, 17) = -(uh * wh' + wh * uh') * Fb;          % angle enters it (Meng -G_theta)
        Fe(10:12, 13:15) = eye(3);                                                                       % lambda += d-lambda
        Q33 = kap * (Ab + (1 - lc)^2 * (Abuf(:, :, 1) + Abuf(:, :, 2))) + (1 - lc)^2 * Sn;
        Q = zeros(17);  Q(7:9, 7:9) = Q33;
        hd = max(dot(wh, pd_k) / Rr, 1.1);  sdl = (9/8) / hd^2 * norm(step);   % sqrt P_fix of d-lambda at the commanded height
        Q(13:15, 13:15) = diag(sdl^4 * I_dl);
        x(1:3) = x(4:6);  x(4:6) = x(7:9);  x(7:9) = lc * x(7:9);  x(10:12) = x(10:12) + x(13:15);
        Pk = Fe * Pk * Fe' + Q;  Pk = 0.5 * (Pk + Pk');
        fbuf = [f_d, fbuf(:, 1)];  Abuf = cat(3, Ab, Abuf(:, :, 1));
        pdbuf = [pdbuf(:, 2:end), pd_kp1];  pd_k = pd_kp1;
    end
    r = struct('seed', seed, 't', t, 'contact', contact, 't_contact', t_contact, 'kend', kend, 'opts', opts);
    f = fieldnames(L);  for i = 1:numel(f); r.(f{i}) = L.(f{i}); end
end

function [C_T, C_n, cT, cn] = local_coeffs(lc, apd, d)     % as run_t2_3d_reg
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
