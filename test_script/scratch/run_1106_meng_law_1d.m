function R = run_1106_meng_law_1d(opts)
% PURPOSE: 1-D (z, plane wall) a-bar' state estimator with two motion control laws, same seeds:
%   'acc'  = current law (past-input sum, motion_ctrl/sec1_model.tex) with its estimator (F_e(3,3) = lc, F_dx, Q33 with the
%            past-input and sensor-noise terms, F_e(4,3) = (1-lc) a'),
%   'meng' = TIE 2025 (6), reading the updated delta-x-hat_3 (reference/eq17_analysis/derivation/1106_aprime_meng_law.tex,
%            run update -> control -> predict; P with F_e, row 3 = [0 0 1 -f 0], row 4 = [0 0 0 1+a'f M]).
%   y2 = a-bar_m from one sample, C_T, C_n of the past-input law for both arms (motion_ctrl/sec3_am.tex), r2 = IF 2 (a-hat + xi)^2.
% EXPIRES: when the user decides whether the Meng law goes into run_t3_xyz_fixed.m.
% opts: seeds (1:5), laws ({'acc','meng'}), noise_nm (3.31), true_gain_ctrl (false: control with the true gain, wiring check),
%       h0 (22.2), h1 (1.10), T_move (1), T_hold (0.2), lc (0.4), apd (0.05), plot (unused), tag ('').
    def = struct('seeds', 1:5, 'laws', {{'acc', 'meng'}}, 'noise_nm', 3.31, 'true_gain_ctrl', false, 'h0', 22.2, 'h1', 1.10, ...
                 'T_move', 1, 'T_hold', 0.2, 'lc', 0.4, 'apd', 0.05, 'plot', false, 'tag', '');   % figures: plot_1106_meng_vs_acc.m
    if nargin < 1; opts = struct(); end
    fn = fieldnames(def);
    for i = 1:numel(fn); if ~isfield(opts, fn{i}); opts.(fn{i}) = def.(fn{i}); end; end
    addpath(genpath(fullfile(fileparts(mfilename('fullpath')), '..', '..', 'model')));
    R = struct();
    for il = 1:numel(opts.laws)
        for is = 1:numel(opts.seeds)
            R.(opts.laws{il})(is) = local_run(opts, opts.laws{il}, opts.seeds(is));
        end
    end
    local_report(R, opts);
end

function r = local_run(opts, law, seed)
    c = physical_constants();  Ts = c.Ts;  Rr = c.R;  a_o = Ts / c.gamma_N / Rr;
    kap = 4 * c.k_B * c.T * a_o / Rr;                       % thermal step variance per unit a-bar [R^2]
    sn2 = (opts.noise_nm * 1e-3 / Rr)^2;                    % sensor noise variance [R^2]
    lc = opts.lc;  apd = opts.apd;  d = 2;
    b = 1 - apd;                                            % C_T, C_n of motion_ctrl/sec3_am.tex (past-input law)
    C_T = b^2 * (2 * b * (1 - lc) / (1 - b * lc) + 2 / ((2 - apd) * (1 + lc) * (1 - b * lc)));
    C_n = 2 * b^2 / (2 - apd) * (1 + b^2 * apd * (1 - lc) / (1 - b * lc) + (1 - lc)^2 / ((1 + lc) * (1 - b * lc)));
    xi = C_n / C_T * sn2 / kap;
    IF = local_IF(lc, apd, d);
    nK = round((opts.T_move + opts.T_hold) / Ts);  t = (0:nK - 1)' * Ts;
    xd = max(opts.h0 - (opts.h0 - opts.h1) * min(t / opts.T_move, 1), opts.h1);   % commanded height [R]
    xd(end + 1) = xd(end);
    rng(seed, 'threefry');  rs2 = RandStream('threefry', 'Seed', 1000 + seed);   % own stream for the discriminator noise (plant draws unchanged)
    xt = xd(1);                                             % true height [R]
    xbuf = xt * ones(1, d + 1);                             % true heights k-d..k
    fbuf = zeros(1, d);  abuf = ones(1, d);                 % past forces and the gains the law used (acc law)
    hd0 = opts.h0;                                          % plane-law seed (as run_t3_xyz_fixed local arm, gain0 'seed')
    x = [0; 0; 0; 1 - (9/8) / hd0; (9/8) / hd0^2];
    Pk = diag([sn2; sn2; sn2; ((9/8) / hd0)^2; x(5)^2]);
    dbar = 0;  H = [1 0 0 0 0; 0 0 0 1 0];  eI = 0;  e_sgn0 = NaN;  pi_go = false;   % PI regulation state
    L = struct('t', t, 'xd', xd(1:nK), 'x', zeros(nK, 1), 'a', zeros(nK, 1), 'ah', zeros(nK, 1), 'ap', zeros(nK, 1), ...
               'aph', zeros(nK, 1), 'dxm', zeros(nK, 1), 'dxr', zeros(nK, 1), 'dx', zeros(nK, 1), 'K3', zeros(nK, 1), 'am', zeros(nK, 1), 'u41', zeros(nK, 1), 'u42', zeros(nK, 1), 'k41', zeros(nK, 1), 'k42', zeros(nK, 1), 'pr4', zeros(nK, 1), 'p55', zeros(nK, 1), 'q55', zeros(nK, 1), 'I5', zeros(nK, 1));
    for k = 1:nK
        [a_t, ap_t] = local_truth(xt);
        % ---- process outputs ----
        n = sqrt(sn2) * randn;
        xm = xbuf(1) + n;                                   % measured height, d steps old
        dxm = (k > d) * xd(max(k - d, 1)) + (k <= d) * xd(1) - xm;   % delta-x-bar_m[k] = x_d[k-d] - x_m[k]
        dbar = apd * dxm + (1 - apd) * dbar;  dxr = dxm - dbar;
        am = (dxr^2 - C_n * sn2) / (C_T * kap);
        % ---- update ----
        ahR = max(x(4), 0.02);
        Rm = diag([sn2, IF * 2 * (ahR + xi)^2]);
        if isfield(opts, 'ym_true') && opts.ym_true; am = a_t + sqrt(Rm(2, 2)) * randn(rs2); end   % 11-06 discriminator: y2 = true gain + white noise of the model r2 (no C_T, C_n bias)
        nu = [dxm - x(1); am - x(4)];
        S = H * Pk * H' + Rm;  K = Pk * H' / S;
        if isfield(opts, 'y1_gain_off') && opts.y1_gain_off; K(4:5, 1) = 0; end   % 11-06 discriminator: y1 does not update a-hat, a'-hat (Joseph form keeps P valid)
        xu = x + K * nu;
        IKH = eye(5) - K * H;  Pu = IKH * Pk * IKH' + K * Rm * K';  Pu = 0.5 * (Pu + Pu');
        I5 = max(1 / Pu(5, 5) - 1 / Pk(5, 5), 0);
        % ---- control law ----
        ac = max(xu(4), 0.02);  if opts.true_gain_ctrl; ac = a_t; end
        if isfield(opts, 'reg') && ~isempty(opts.reg)          % 11-06: PI gain regulation from t = 0 (as run_t3_xyz_fixed local arm, 1103 tex Sec. 5)
            g = opts.reg;  ek = g.a_d - xu(4);                  %   e_a = a_d - a-hat (after the update)
            sl = max(xu(5), (8 / 9) * (1 - min(xu(4), 1))^2);   %   a'-hat floored by the plane law (ap_floor_law)
            dreg = (Ts / g.tau_p) * ek / sl;
            if isfinite(g.tau_i)
                if isnan(e_sgn0); e_sgn0 = sign(ek); end;  if ~pi_go && sign(ek) ~= e_sgn0; pi_go = true; end   %   'cross': integrate once e first changes sign
                if pi_go; eI = eI + ek; end
                dreg = dreg + Ts^2 / (g.tau_p * g.tau_i) * eI / sl;
            end
            xd(k + 1) = xd(k) + dreg;  xd(k + 2:end) = xd(k + 1);
        end
        Dxd = xd(k + 1) - xd(k);
        if strcmp(law, 'meng')
            f = (Dxd + (1 - lc) * xu(3)) / ac;
            Fdx = f;
        else
            xh = xm + sum(abuf .* fbuf);                    % x-hat[k] = x_m[k] + sum a-hat[k-i] f[k-i]
            f = (xd(k + 1) - xh - lc * (xd(k) - xh)) / ac;
            Fdx = f + (1 - lc) * sum(fbuf);
        end
        Mh = Dxd + (1 - lc) * xu(3);
        % ---- log ----
        L.xd(k) = xd(k);  L.x(k) = xt;  L.a(k) = a_t;  L.ah(k) = xu(4);  L.ap(k) = ap_t;  L.aph(k) = xu(5);
        L.p55(k) = Pu(5, 5);  L.I5(k) = I5;   % 11-06 log: P of a'-hat after the update, information on a' this step
        L.u41(k) = K(4, 1) * nu(1);  L.u42(k) = K(4, 2) * nu(2);  L.k41(k) = K(4, 1);  L.k42(k) = K(4, 2);  L.pr4(k) = xu(5) * Mh;   % 11-06 log: update of a-hat by y1, by y2, and the prediction a'-hat M-hat
        L.dxm(k) = dxm;  L.dxr(k) = dxr;  L.dx(k) = xd(k) - xt;  L.K3(k) = K(3, 1);  L.am(k) = am;
        % ---- plant ----
        xt = xt + a_t * f + sqrt(kap * a_t) * randn;
        if xt < 1; L.contact = true; break; end
        xbuf = [xbuf(2:end), xt];
        a_past = abuf;                                      % gains the law used at k-1, k-2 (Q33 of the acc law)
        fbuf = [f, fbuf(1:end - 1)];  abuf = [ac, abuf(1:end - 1)];
        % ---- error dynamics, process noise ----
        aph = xu(5);  ahq = max(xu(4), 0.02);
        Fe = zeros(5);  Fe(1, 2) = 1;  Fe(2, 3) = 1;  Fe(5, 5) = 1;  Fe(4, 5) = Mh;
        if strcmp(law, 'meng')
            Fe(3, 3) = 1;  Fe(3, 4) = -f;  Fe(4, 4) = 1 + aph * f;
            q33 = kap * ahq;
        else
            Fe(3, 3) = lc;  Fe(3, 4) = -Fdx;  Fe(4, 4) = 1 + aph * Fdx;  Fe(4, 3) = (1 - lc) * aph;
            q33 = kap * (ahq + (1 - lc)^2 * sum(a_past)) + (1 - lc)^2 * sn2;   % sec4: kappa_T (a-hat[k] + (1-lc)^2 sum a-hat[k-i]) + (1-lc)^2 sigma_n^2
        end
        g = [0; 0; -1; aph; 0];
        Q = g * q33 * g';
        sPf = 0.4 * (1.1 / max(xd(k), 1.1))^2;  Q(5, 5) = (sPf^2)^2 * I5;
        if isfield(opts, 'q55_hold0') && opts.q55_hold0 && Dxd == 0; Q(5, 5) = 0; end   % 11-06 (user): no commanded motion -> no process noise on a'
        L.q55(k) = Q(5, 5);
        % ---- predict (from the updated estimate) ----
        x = [xu(2); xu(3); lc * xu(3); xu(4) + xu(5) * Mh; xu(5)];
        Pk = Fe * Pu * Fe' + Q;  Pk = 0.5 * (Pk + Pk');
    end
    if ~isfield(L, 'contact'); L.contact = false; end
    r = L;  r.seed = seed;  r.law = law;  r.kap = kap;  r.sn2 = sn2;  r.Rr = Rr;
end

function [a, ap] = local_truth(h)
    hh = max(h, 1.0005);  e = 1e-5;
    [~, cp] = calc_correction_functions(hh);  [~, c1] = calc_correction_functions(hh + e);  [~, c0] = calc_correction_functions(hh - e);
    a = 1 / cp;  ap = (1 / c1 - 1 / c0) / (2 * e);
end

function IF = local_IF(lc, apd, d)                          % as run_t3_xyz_fixed:50 (thermal chain of the past-input law)
    n = 460;  tau = zeros(n, 1);  tau(1) = 1;  dp = zeros(n + 1, 1);
    for k = 1:n
        past = 0;  for i = 1:d; if k - i >= 1; past = past + tau(k - i); end; end
        dp(k + 1) = lc * dp(k) - (tau(k) + (1 - lc) * past);
    end
    bar = 0;  g = zeros(n, 1);
    for k = 1:n
        prev = 0;  if k - d >= 1; prev = dp(k - d); end
        bar = apd * prev + (1 - apd) * bar;  g(k) = prev - bar;
    end
    cT = zeros(401, 1);  for tau_ = 0:400; cT(tau_ + 1) = sum(g(1:end - tau_) .* g(1 + tau_:end)); end
    rT = cT / cT(1);  IF = 1 + 2 * sum(rT(2:end).^2);
end

function local_report(R, opts)
    laws = fieldnames(R);
    fprintf('\n=== run_1106_meng_law_1d  noise %.2f nm  true_gain_ctrl %d  seeds %s ===\n', opts.noise_nm, opts.true_gain_ctrl, mat2str(opts.seeds));
    win = {'motion 0-1 s', [0, opts.T_move]; 'near wall 0.8-1 s', [0.8, opts.T_move]; 'hold', [opts.T_move, opts.T_move + opts.T_hold]};
    M = struct();
    for il = 1:numel(laws)
        rr = R.(laws{il});
        fprintf('-- %s  (contact %d/%d)\n', laws{il}, sum([rr.contact]), numel(rr));
        for iw = 1:size(win, 1)
            for is = 1:numel(rr)
                m = rr(is).t >= win{iw, 2}(1) & rr(is).t < win{iw, 2}(2) & rr(is).a > 0;
                ea = rr(is).a(m) - rr(is).ah(m);
                M.(laws{il}).mae(is, iw) = mean(abs(ea));  M.(laws{il}).bias(is, iw) = mean(ea);  M.(laws{il}).worst(is, iw) = max(abs(ea));
                M.(laws{il}).apr(is, iw) = median(rr(is).aph(m) ./ rr(is).ap(m));
                M.(laws{il}).sdx(is, iw) = std(rr(is).dx(m)) * rr(is).Rr * 1e3;
                M.(laws{il}).K3(is, iw) = mean(rr(is).K3(m));
            end
            v = M.(laws{il});
            fprintf('   %-18s mean|e_a| %.4f (sd %.4f)  mean e_a %+.4f  worst %.3f  median a''-hat/a'' %.2f  std(dx) %.2f nm  K3 %.3f\n', win{iw, 1}, ...
                mean(v.mae(:, iw)), std(v.mae(:, iw)), mean(v.bias(:, iw)), max(v.worst(:, iw)), mean(v.apr(:, iw)), mean(v.sdx(:, iw)), mean(v.K3(:, iw)));
        end
    end
    if all(isfield(M, {'acc', 'meng'}))
        fprintf('-- paired meng - acc (per seed, mean +- sd, SEM)\n');
        for iw = 1:size(win, 1)
            dm = M.meng.mae(:, iw) - M.acc.mae(:, iw);  ds = M.meng.sdx(:, iw) - M.acc.sdx(:, iw);  n = numel(dm);
            fprintf('   %-18s d mean|e_a| %+.4f +- %.4f (SEM %.4f)   d std(dx) %+.2f +- %.2f nm\n', win{iw, 1}, mean(dm), std(dm), std(dm) / sqrt(n), mean(ds), std(ds));
        end
    end
end
