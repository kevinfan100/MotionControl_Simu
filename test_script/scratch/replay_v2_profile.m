function out = replay_v2_profile(seeds)
%REPLAY_V2_PROFILE  Update-level exact forcing extraction, 8 seeds (2026-08-31).
%
%   Per seed, with the PASS-stamped capture:
%       G_tot[k] = e_upd[k] - A_k e_upd[k-1]           (A = (I-K2H2)(I-K1H1)F)
%   is the COMPLETE forcing (process + measurement noise + every model error):
%   replaying it is a tautology (gate = machine zero), so the value is in the
%   ENSEMBLE: the cross-seed mean profile of G_tot is the mean forcing that
%   creates the bias, with an honest SEM from the seed spread.
%   Outputs:
%     - segment table of mean G_tot rows 3 and 4 (+- SEM)
%     - replay of the ensemble-mean forcing through the seed-7 loop ->
%       predicted mean e4 trajectory vs the actual cross-seed mean e4
%     - regression of the ensemble-mean row-3 profile on the cross-seed
%       CENTRED mobility component  (a'/a_hat) <dw_c * step_c>  (deterministic
%       contamination removed by the centring)
% STATUS: ACTIVE | closed-loop mean-bias derivation line

    if nargin < 1 || isempty(seeds); seeds = 1:8; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'loop_mean_bias');
    ax = 3;  lam = 0.7;  ns0 = numel(seeds);
    G3 = [];  G4 = [];  E4A = [];  DWJ = [];  STP = [];  APH = [];
    for q = 1:ns0
        sd = seeds(q);
        C = load(fullfile(od, sprintf('loop_capture_seed%d.mat', sd)));
        assert(C.ok, 'capture seed %d not PASS-stamped', sd);
        R = load(fullfile(od, sprintf('run_log_seed%d.mat', sd)));  r = R.run_log;
        nsl = size(C.F, 1);  n = size(C.F, 3);  kk = C.KK;  a_nom = r.a_nom;
        dw_true = r.h_bar_d_out(:) - r.h_bar_true_out(:);
        wT = r.a_true_out(:, ax) .* r.F_th_out(:, ax) / physical_constants().R;
        XT = zeros(nsl, n);
        for i = 1:n
            k = kk(i);
            XT(:, i) = [dw_true(max(k-2,1)); dw_true(max(k-1,1)); dw_true(k); r.a_true_out(k, ax)/a_nom; ...
                        C.XU(5:7, i); wT(k); wT(max(k-1,1))];
        end
        EU = XT - C.XU;
        i0 = find(kk >= 3, 1);
        Gt = zeros(nsl, n);
        for i = i0+1:n
            Gt(:, i) = EU(:, i) - C.A(:, :, i) * EU(:, i-1);
        end
        % tautology gate on the first seed
        if q == 1
            e = EU(:, i0);  gmax = 0;
            for i = i0+1:n
                e = C.A(:,:,i) * e + Gt(:, i);
                gmax = max(gmax, max(abs(e - EU(:, i))));
            end
            fprintf('[v2 gate, seed %d] replay tautology max err %.2e  %s\n', sd, gmax, string(gmax < 1e-12));
        end
        if isempty(G3); N = n;  G3 = zeros(N, ns0);  G4 = G3;  E4A = G3;  DWJ = G3;  STP = G3;  APH = G3;  tC = C.t(:);  i0g = i0; end
        G3(:, q) = Gt(3, :).';  G4(:, q) = Gt(4, :).';  E4A(:, q) = EU(4, :).';
        DWJ(:, q) = -dw_true(kk);                                   % w - w_d (jitter + lag), per seed
        dwd = [0; diff(r.h_bar_d_out(:))];
        STP(:, q) = dwd(kk) + (1 - lam) * C.XU(3, :).';
        APH(:, q) = (r.b_hat_out(kk, ax) .* (1 - r.a_true_out(kk, ax)/a_nom).^2) ./ max(r.a_bar_hat_out(kk, ax), 0.02);
    end
    t = tC;
    SEG = {'hold start', t > 0.05 & t < 0.5; 'descend', t > 0.55 & t < 1.45; ...
           'oscillate', t >= 1.5 & t < 3.5; 'hold end', t > 3.7};
    fprintf('\nensemble mean forcing per step (x1e-5), %d seeds:\n%-12s %14s %14s\n', ns0, 'segment', 'row3 (dw)', 'row4 (a)');
    for g = 1:4
        m = SEG{g, 2};
        m3 = mean(G3(m, :), 1);  m4 = mean(G4(m, :), 1);
        fprintf('%-12s %+8.2f +- %-5.2f %+8.2f +- %-5.2f\n', SEG{g,1}, 1e5*mean(m3), 1e5*std(m3)/sqrt(ns0), 1e5*mean(m4), 1e5*std(m4)/sqrt(ns0));
    end

    % ---- replay the ensemble-mean forcing through the seed-7 loop ----------
    C7 = load(fullfile(od, 'loop_capture_seed7.mat'));
    gbar3 = mean(G3, 2);  gbar4 = mean(G4, 2);
    nsl = size(C7.F, 1);  n = size(C7.F, 3);
    e = zeros(nsl, 1);  E4 = zeros(n, 1);
    for i = i0g+1:n
        g = zeros(nsl, 1);  g(3) = gbar3(i);  g(4) = gbar4(i);
        e = C7.A(:,:,i) * e + g;
        E4(i) = e(4);
    end
    mh = t > 3.7;
    fprintf('\n[mean-forcing replay] end-hold e4: predicted %+.4f  vs actual seed-mean %+.4f (+- %.4f)  a_o\n', ...
            mean(E4(mh)), mean(mean(E4A(mh, :), 1)), std(mean(E4A(mh, :), 1))/sqrt(ns0));

    % ---- centred mobility component vs the mean row-3 profile --------------
    DWc = DWJ - mean(DWJ, 2);  STc = STP - mean(STP, 2);
    comp = mean(APH, 2) .* mean(DWc .* STc, 2) * (-1);              % row-3 sign: extra +w displacement lowers dw
    ms = movmean(mean(G3, 2), 161);  cs = movmean(comp, 161);
    mo = t >= 1.5 & t < 3.5;
    fprintf('[mobility component, centred] osc: mean %+0.2e vs G3 mean %+0.2e | slope %+.2f corr %+.2f\n', ...
            mean(comp(mo)), mean(gbar3(mo)), cs(mo) \ ms(mo), corr(cs(mo), ms(mo)));
    fprintf('%-40s hold end: comp %+0.2e vs G3 %+0.2e\n', '', mean(comp(mh)), mean(gbar3(mh)));
    out = struct('t', t, 'G3', G3, 'G4', G4, 'E4A', E4A, 'E4rep', E4, 'comp', comp);
    save(fullfile(od, 'replay_v2_profile.mat'), 'out');
    fprintf('V2 PROFILE DONE\n');
end
