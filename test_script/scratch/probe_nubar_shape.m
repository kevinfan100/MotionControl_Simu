function out = probe_nubar_shape()
%PROBE_NUBAR_SHAPE  TEMPORARY probe (2026-08-31) -- shape-matching, offline only.
%   The noise-added mean innovation nubar_noise(t) = mean_seeds nu_noisy - nu_det
%   (b_true + exact base, oscillation segment) is the fingerprint of the
%   remaining ~15 pp.  Compare its time/phase shape against the shapes the
%   candidate mechanisms predict (all built from the DETERMINISTIC arm's log,
%   physics closed forms, no filter P):
%     S1  a'(w_d) * v_d         gain-error/lag family: sign follows command velocity
%     S2  |a''(w_d)| * a(w_d)   Jensen/position family: Var ~ kappa_T*a, no velocity sign
%     S3  a'(w_d) * a(w_d)      even-in-velocity alternative
%   Reads test_results/btrue_log_ledger/check_btrue_log_ledger.mat.
%   Output: correlations + 8-bin phase patterns.  NO verdict is implied by a
%   single correlation; a candidate is only alive if sign pattern AND corr fit.
% STATUS: TEMPORARY | EXPIRES: remedy-2 derivation lands

    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    S = load(fullfile(root, 'test_results', 'btrue_log_ledger', 'check_btrue_log_ledger.mat'));
    N = S.out.noisyExact;  D = S.out.detExact;
    t = N.tt;  Ts = t(2) - t(1);
    nu_noise = mean(N.n1(2:end, :), 2) - D.n1(2:end);        % noise-added mean innovation
    hd = N.hd(2:end);  vd = [0; diff(hd)] / Ts;              % command height [R], velocity [R/s]
    ab = D.at(2:end);                                        % a_bar along the (tightly tracked) det run
    bh = D.bh(2:end);
    ap  = bh .* (1 - ab).^2;                                 % a'
    app = 2 * bh.^2 .* (1 - ab).^3;                          % |a''|
    SH = {ap .* vd, 'S1  a''*v_d  (lag family)'; ...
          app .* ab, 'S2  |a''''|*a  (Jensen family)'; ...
          ap .* ab,  'S3  a''*a   (even alt)'};
    m = t >= 1.5 & t < 3.5;
    x = movmean(nu_noise, 81);                               % 50 ms smoothing
    fprintf('\n[TEMPORARY shape probe] oscillation 1.5-3.5 s, nu_noise smoothed 50 ms\n');
    for i = 1:3
        s = movmean(SH{i,1}, 81);
        fprintf('  corr(nubar_noise, %-28s) = %+.3f\n', SH{i,2}, corr(x(m), s(m)));
    end
    ph = mod(t(m) - 1.5, 1);  edges = 0:0.125:1;  idx = find(m);
    fprintf('  %-10s %10s | %10s %10s %10s\n', 'phase', 'nubar', 'S1', 'S2', 'S3');
    nrm = @(v) v / max(abs(v) + eps);
    P = zeros(8, 4);
    for b = 1:8
        sel = idx(ph >= edges(b) & ph < edges(b+1));
        P(b, :) = [mean(x(sel)), mean(SH{1,1}(sel)), mean(SH{2,1}(sel)), mean(SH{3,1}(sel))];
    end
    P(:, 1) = nrm(P(:, 1)); for c = 2:4; P(:, c) = nrm(P(:, c)); end
    for b = 1:8
        fprintf('  %.3f-%.3f %+10.2f | %+10.2f %+10.2f %+10.2f\n', edges(b), edges(b+1), P(b,1), P(b,2), P(b,3), P(b,4));
    end
    fprintf('  (columns normalised to their own max; compare SIGN PATTERNS, not magnitudes)\n');
    out = struct('P', P, 'nu_noise', nu_noise, 't', t);
    fprintf('SHAPE PROBE DONE\n');
end
