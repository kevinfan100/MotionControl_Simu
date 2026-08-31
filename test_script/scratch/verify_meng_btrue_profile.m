function out = verify_meng_btrue_profile(seeds)
%VERIFY_MENG_BTRUE_PROFILE  Cross-scenario check of the closed-loop mean-bias
%   framework on the Meng 10 s ramp, b_true arm (2026-08-31).
%
%   Question: does the mean forcing extracted on a COMPLETELY DIFFERENT
%   trajectory obey the same closed form that matched canonical at 1-5%?
%     row 3:  G3_mean  ~  (a'/a_hat) <dw_c * step_c>      (cross-seed centred)
%     row 4:  G4       ~  a'_hat * (dw_true - M_hat)      (slope/corr per seed)
%   and does the forcing concentrate in the near-wall stretch (8-10 s), where
%   the motion-test gains page shows the seed fans?
%
%   Arm: Meng ramp (h 15 -> 2.5 um over 10 s, then hold), b_true at TRUE
%   height (as in motion-test's arm), law_exact_step ON (clean base; the
%   Euler ratchet is < 1 pp on this ramp).  Per seed: run with obs_dump,
%   rebuild K (y1 then y2), A = (I-K2H2)(I-K1H1)F, truth-aligned e, and
%   G_tot[k] = e_upd[k] - A_k e_upd[k-1]  (update-level, tautology-exact).
%   Only slim per-seed series are kept (no big matrices saved).
% STATUS: ACTIVE | closed-loop mean-bias derivation line, cross-scenario leg

    if nargin < 1 || isempty(seeds); seeds = [7 11 23 42 101 777]; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'loop_mean_bias');
    if ~exist(od, 'dir'); mkdir(od); end
    ax = 3;  lam = 0.7;  pcR = physical_constants().R;
    ov = struct('trajectory_type','osc','h_init',15.0,'h_bottom',2.5,'amplitude',0,'frequency',1,'n_cycles',1, ...
                't_hold',0.5,'t_descend_override',10.0,'T_sim',12.5,'h_min',1.1*pcR);
    ns0 = numel(seeds);
    G3 = [];  G4 = [];  E4A = [];  DWJ = [];  STP = [];  APH = [];  R4P = [];
    for q = 1:ns0
        O = run_formC_b(struct('arm', 'best', 'ap_src', 'post', 'seeds', seeds(q), 'verbose', false, ...
                               'b_true', true, 'b_true_at', 'true', 'config_override', ov, ...
                               'ctrl_const_override', struct('law_exact_step', true, 'obs_dump', true)));
        r = O.runs{1};  L = obs_dump('get');  Lz = L([L.ax] == ax);
        n = numel(Lz);  nsl = numel(Lz(1).x_pred);  a_nom = r.a_nom;
        kk = zeros(1, n);  XU = zeros(nsl, n);  A = zeros(nsl, nsl);  EUp = [];  Gt = zeros(nsl, n);
        ht = r.h_bar_true_out(:);  hd = r.h_bar_d_out(:);
        dw_true = hd - ht;
        wT = r.a_true_out(:, ax) .* r.F_th_out(:, ax) / pcR;
        e_gate = 0;
        for i = 1:n
            s = Lz(i);  k = s.k + 1;  kk(i) = k;                 % LOG_OFFSET +1
            H1 = zeros(1, nsl);  H1(1) = 1;  P = s.P_pred;
            k1 = (P * H1.') / (H1 * P * H1.' + s.R(1));
            P1 = (eye(nsl) - k1 * H1) * P;
            H2 = s.H{2};  I2 = eye(nsl);
            if ~isempty(H2) && ~s.gate
                k2 = (P1 * H2.') / (H2 * P1 * H2.' + s.R(2));
                I2 = eye(nsl) - k2 * H2;
            else
                k2 = zeros(nsl, 1);
            end
            % identity gate (vs the run's own K log)
            e_gate = max(e_gate, abs(k1(4) - r.K_a_y1_out(min(k, size(r.K_a_y1_out,1)), ax)));
            A = I2 * (eye(nsl) - k1 * H1) * s.F;
            xt = [dw_true(max(k-2,1)); dw_true(max(k-1,1)); dw_true(min(k, numel(dw_true))); ...
                  r.a_true_out(min(k, size(r.a_true_out,1)), ax)/a_nom; s.x_upd(5:7); wT(min(k, numel(wT))); wT(max(k-1,1))];
            eu = xt - s.x_upd;
            if ~isempty(EUp);  Gt(:, i) = eu - A * EUp;  end
            EUp = eu;
            XU(:, i) = s.x_upd;
        end
        assert(e_gate < 1e-9, 'gain identity failed for seed %d (%.2e)', seeds(q), e_gate);
        if isempty(G3)
            N = n;  t = r.tout(kk(1:min(n, numel(r.tout))));  t = r.tout(min(kk, numel(r.tout)));  t = t(:);
            G3 = zeros(N, ns0);  G4 = G3;  E4A = G3;  DWJ = G3;  STP = G3;  APH = G3;  R4P = G3;
        end
        G3(:, q) = Gt(3, :).';  G4(:, q) = Gt(4, :).';
        E4A(:, q) = (r.a_true_out(kk, ax)/a_nom) - XU(4, :).';
        DWJ(:, q) = -dw_true(kk);
        dwd = [0; diff(hd)];
        STP(:, q) = dwd(kk) + (1 - lam) * XU(3, :).';
        ah = r.a_bar_hat_out(kk, ax);  bh = r.b_hat_out(kk, ax);
        APH(:, q) = (bh .* (1 - r.a_true_out(kk, ax)/a_nom).^2) ./ max(ah, 0.02);
        dtrs = [0; diff(ht)];  dtrs = dtrs(kk);
        Mhat = dwd(kk) + [0; (1-lam) * XU(3, 1:end-1).'] + [0; (1-lam) * (XU(8, 1:end-1) + XU(9, 1:end-1)).'];
        R4P(:, q) = (bh .* (1 - ah).^2) .* (dtrs - Mhat);
        fprintf('seed %d done (%d steps, gate %.1e)\n', seeds(q), n, e_gate);
    end
    save(fullfile(od, 'meng_btrue_profile.mat'), 'G3', 'G4', 'E4A', 'DWJ', 'STP', 'APH', 'R4P', 't', 'seeds');

    % ---- profile vs closed form -------------------------------------------
    SEG = {'far half',  t > 0.5 & t < 5.5; 'near half', t > 5.5 & t < 10.5; ...
           'near-wall 8-10 s', t > 8 & t < 10; 'hold', t > 11.5};
    DWc = DWJ - mean(DWJ, 2);  STc = STP - mean(STP, 2);
    comp = mean(APH, 2) .* mean(DWc .* STc, 2);                 % SIGN as calibrated on canonical (comp sign -1 there = +this)
    fprintf('\nMeng b_true arm, %d seeds: mean forcing per step (x1e-5)\n', ns0);
    fprintf('%-18s %14s %14s | %10s %8s\n', 'segment', 'G3 (dw row)', 'G4 (a row)', 'closed form', 'ratio');
    for g = 1:4
        m = SEG{g,2};
        m3 = mean(G3(m,:), 1);  m4 = mean(G4(m,:), 1);
        cf = mean(comp(m));
        fprintf('%-18s %+7.2f +- %-5.2f %+7.2f +- %-5.2f | %+10.2f %8.2f\n', SEG{g,1}, ...
                1e5*mean(m3), 1e5*std(m3)/sqrt(ns0), 1e5*mean(m4), 1e5*std(m4)/sqrt(ns0), 1e5*cf, cf/mean(m3));
    end
    % row-4 unified check
    SL = zeros(1, ns0);  CC = zeros(1, ns0);  m = t > 0.5;
    for q = 1:ns0
        SL(q) = R4P(m, q) \ G4(m, q);  CC(q) = corr(R4P(m, q), G4(m, q));
    end
    fprintf('[row4 unified] G4 ~ a''*(dw_true - M_hat): slope %.3f +- %.3f  corr %.3f +- %.3f  (canonical: 0.716 / 0.796)\n', ...
            mean(SL), std(SL)/sqrt(ns0), mean(CC), std(CC)/sqrt(ns0));
    % where is the forcing concentrated?
    p = movmean(abs(mean(G4, 2)), 161);
    [~, ipk] = max(p);
    fprintf('[concentration] |mean G4| peaks at t = %.2f s (motion-test fans: 8-10 s)\n', t(ipk));
    out = struct('t', t, 'G3', G3, 'G4', G4, 'comp', comp, 'SL', SL, 'CC', CC);
    fprintf('MENG PROFILE DONE\n');
end
