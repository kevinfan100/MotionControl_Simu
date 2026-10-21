% STATUS: SCRATCH -- 2026-10-16, verification of 1015_wall_angle_readout Section 5 (exact expectations in the estimated local
%   coordinate: E[du~^2] = C_T k_T (cos^2 e a_T + sin^2 e a_N) + C_n s^2, E[dw~^2] = C_T k_T (sin^2 e a_T + cos^2 e a_N) + C_n s^2,
%   E[du~ dw~] = -C_T k_T sin e cos e (a_T - a_N)), user 10-16 "這個的驗證圖先產出". Two arms, same plant and settings as T1
%   (run_t1_angle_readout: wall theta_y 20 deg, hold at h 1.2 R along the true normal, gains true in the control law, 15 seeds):
%     A 'readout only' : control law uses the TRUE angle; the readout is rotated by theta-hat in {0 10 15 20 25 30 40} deg
%                        (all read angles from the same run) -> the random component is the one of Section 3, Section 5 should hold
%     B 'closed loop'  : control law AND readout use theta-hat (T1); the new theta-hat 10 / 30 / 40 are run here, 0 / 15 / 20 / 25
%                        come from temp_t1_angle_readout.mat
%   Readout per step (a_cov = 1): a_Tm = u direction only (that document is the x-z plane), a_Nm, e_theta_m with the TRUE gains in
%   the denominator. Output test_results/temp_3d_wsonly/temp_t1_formula_check.mat. | run: run_t1_formula_check (or matlab -batch)
function R = run_t1_formula_check(opts)
    if nargin < 1; opts = struct(); end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));  addpath(here);
    def = struct('seeds', 1:15, 'theta_y', 20, 'th_read', [0 10 15 20 25 30 40], 'th_closed', [10 30 40], ...
                 'h_hold', 1.2, 'T_desc', 1.0, 'T_hold', 5.0, 'lambda_c', 0.4, 't_win', 1.2);
    fn = fieldnames(def);
    for i = 1:numel(fn); if ~isfield(opts, fn{i}); opts.(fn{i}) = def.(fn{i}); end; end
    R = struct('opts', opts, 'A', {{}}, 'B', {{}});
    for s = opts.seeds
        R.A{end+1} = local_run(opts, opts.theta_y, opts.th_read, s);
        fprintf('A seed %d done\n', s);
    end
    for th = opts.th_closed
        for s = opts.seeds
            R.B{end+1} = local_run(opts, th, th, s);
        end
        fprintf('B theta-hat %g done\n', th);
    end
    fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
    save(fullfile(fo, 'temp_t1_formula_check.mat'), 'R');
    fprintf('saved %s\n', fullfile(fo, 'temp_t1_formula_check.mat'));
end

function r = local_run(opts, th_ctrl_deg, th_read_deg, seed)
    clear calc_thermal_force;
    rng(seed);
    cfg = canonical_scenario(0.05, 1.10, 'ramp_tri', 1);
    cfg.lambda_c = opts.lambda_c;  cfg.theta_y = opts.theta_y;  cfg.theta_x = 0;
    params = calc_simulation_params(cfg);  P = params.Value;
    Ts = P.common.Ts;  Rr = P.common.R;  a_nom = Ts / P.common.gamma_N;  a_o = a_nom / Rr;
    kap = 4 * P.ctrl.k_B * P.ctrl.T * a_o / Rr;  Sn = diag((cfg.meas_noise_std(:) / Rr).^2);
    lc = cfg.lambda_c;  apd = cfg.a_pd;  d = 2;
    [C_T, C_n] = local_coeffs(lc, apd, d);
    w = P.wall.w_hat(:);  pz = P.wall.pz;
    Vc = local_Ry(th_ctrl_deg * pi / 180);                       % control law frame
    nr = numel(th_read_deg);  Vr = zeros(3, 3, nr);  vSn = zeros(3, 3, nr);
    for j = 1:nr; Vr(:, :, j) = local_Ry(th_read_deg(j) * pi / 180);  vSn(:, :, j) = Vr(:, :, j)' * Sn * Vr(:, :, j); end
    h0 = (dot(P.common.p0, w) - pz) / Rr;
    N = round((opts.T_desc + opts.T_hold) / Ts) + 1;  t = (0:N-1)' * Ts;
    hd = max(h0 - (h0 - opts.h_hold) * t / opts.T_desc, opts.h_hold);
    pd = (pz + hd' * Rr) .* w;
    p = pd(:, 1);  pbuf = repmat(p, 1, d + 1);  fbuf = zeros(3, d);  Abuf = repmat(a_nom * eye(3), 1, 1, d);  dbar = zeros(3, 1);
    win = t >= opts.t_win;  nw = sum(win);
    sTu = zeros(nr, 1);  sNw = zeros(nr, 1);  sE = zeros(nr, 1);  sh = 0;
    for k = 1:N
        pm = pbuf(:, 1);  dpm = (pd(:, max(k - d, 1)) - pm) / Rr;
        dbar = apd * dpm + (1 - apd) * dbar;  dpr = dpm - dbar;
        h_true = max((dot(p, w) - pz) / Rr, 1.001);
        [cpa, cpe] = calc_correction_functions(h_true);  aT = 1 / cpa;  aN = 1 / cpe;
        if win(k)
            for j = 1:nr
                vd = Vr(:, :, j)' * dpr;  vA = (vd * vd' - C_n * vSn(:, :, j)) / (C_T * kap);
                sTu(j) = sTu(j) + vA(1, 1);  sNw(j) = sNw(j) + vA(3, 3);  sE(j) = sE(j) - vA(1, 3) / (aT - aN);
            end
            sh = sh + h_true;
        end
        Ah = a_nom * Vc * diag([aT aT aN]) * Vc';
        phat = pm + Abuf(:, :, 1) * fbuf(:, 1) + Abuf(:, :, 2) * fbuf(:, 2);
        f_d = Ah \ (pd(:, min(k + 1, N)) - phat - lc * (pd(:, k) - phat));
        f_th = calc_thermal_force(p, P);
        p = step_dynamics(p, f_d + f_th, P, Ts);
        pbuf = [pbuf(:, 2:end), p + cfg.meas_noise_std(:) .* randn(3, 1)];
        fbuf = [f_d, fbuf(:, 1)];  Abuf = cat(3, Ah, Abuf(:, :, 1));
    end
    hw = sh / nw;  [cpa, cpe] = calc_correction_functions(hw);
    r = struct('seed', seed, 'th_ctrl', th_ctrl_deg, 'th_read', th_read_deg, 'aT', 1 / cpa, 'aN', 1 / cpe, ...
               'm_aTu', sTu / nw, 'm_aNm', sNw / nw, 'm_eth', sE / nw * 180 / pi);
end

function M = local_Ry(a)
    M = [cos(a) 0 sin(a); 0 1 0; -sin(a) 0 cos(a)];
end

function [C_T, C_n] = local_coeffs(lc, apd, d)            % variance factors of dp_r (impulse responses, as run_t1_angle_readout)
    n = 460;  gT = local_chain(lc, apd, d, n, 'tau');  gn = local_chain(lc, apd, d, n, 'n');
    C_T = sum(gT.^2);  C_n = sum(gn.^2);
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
