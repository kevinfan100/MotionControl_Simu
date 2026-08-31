function out = probe_jensen_lag_candidate()
%PROBE_JENSEN_LAG_CANDIDATE  TEMPORARY probe (2026-08-31) -- offline, no verdict.
%   Candidate for the remaining ~15 pp (lag family, matches the velocity-locked
%   sign pattern of probe_nubar_shape): the PLANT moves with a(w_true) while
%   every model in the loop uses a at a point value; a(w) is concave, so under
%   thermal jitter the mean realised mobility is LOWER:
%       E[a(w+d)] = a(w) + 1/2 a'' Var(d),   a'' = -2 b^2 (1-a)^3 < 0
%   The mean displacement shortfall per step is the Jensen term times the
%   commanded step:  s[k] = 1/2 (a''/a) Var_w[k] * dwd[k]   [R]
%   and the innovation mean it implies (level, loop pole lc) is
%       nu_pred[k] ~ s[k] / (1 - lc).
%   This is the same family as the 08-26 finding (a' evaluated at the jittering
%   height, -0.021 vs -0.002).  Var_w is taken from the DATA (cross-seed var of
%   the tracking-error estimate, corr 0.95 with truth), not from filter P.
%   Checks: magnitude of nu_pred vs measured nu_noise; shape corr; implied y1
%   leg  sum K1bar nu_pred  vs the measured noise-added y1 mean (+0.006 a_o).
% STATUS: TEMPORARY | EXPIRES: remedy-2 derivation lands

    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    S = load(fullfile(root, 'test_results', 'btrue_log_ledger', 'check_btrue_log_ledger.mat'));
    N = S.out.noisyExact;  D = S.out.detExact;  lam = 0.7;
    t = N.tt;
    nu_noise = mean(N.n1(2:end, :), 2) - D.n1(2:end);
    dwd = [0; diff(N.hd(2:end))];
    ab  = D.at(2:end);  bh = D.bh(2:end);
    app = -2 * bh.^2 .* (1 - ab).^3;                       % a'' (signed, < 0)
    varw = var(N.dx3(1:end-1, :), 0, 2);                   % cross-seed Var of tracking error [R^2]
    s_k  = 0.5 * (app ./ ab) .* varw .* dwd;               % per-step mean shortfall [R]
    nu_pred = s_k / (1 - lam);                             % implied innovation level
    m = t >= 1.5 & t < 3.5;
    x = movmean(nu_noise, 81);  y = movmean(nu_pred, 81);
    fprintf('\n[TEMPORARY Jensen-lag probe] oscillation 1.5-3.5 s\n');
    fprintf('  magnitude: rms nu_noise %.3e   rms nu_pred %.3e   ratio pred/meas %.2f\n', ...
            rms(x(m)), rms(y(m)), rms(y(m))/rms(x(m)));
    fprintf('  shape:     corr(nu_noise, nu_pred) = %+.3f\n', corr(x(m), y(m)));
    K1b = mean(N.K1(2:end, :), 2);
    y1_pred = sum(K1b(m) .* nu_pred(m));
    fprintf('  implied y1 leg  sum K1bar*nu_pred = %+.4f a_o   (measured noise-added y1 mean part: +0.006)\n', y1_pred);
    ph = mod(t(m) - 1.5, 1);  edges = 0:0.125:1;  idx = find(m);
    nrm = @(v) v / max(abs(v) + eps);  P = zeros(8, 2);
    for b = 1:8
        sel = idx(ph >= edges(b) & ph < edges(b+1));
        P(b, :) = [mean(x(sel)), mean(nu_pred(sel))];
    end
    P(:,1) = nrm(P(:,1));  P(:,2) = nrm(P(:,2));
    fprintf('  %-12s %10s %10s\n', 'phase', 'nubar', 'nu_pred');
    for b = 1:8; fprintf('  %.3f-%.3f  %+10.2f %+10.2f\n', edges(b), edges(b+1), P(b,1), P(b,2)); end
    out = struct('nu_noise', nu_noise, 'nu_pred', nu_pred, 'P', P);
    fprintf('JENSEN LAG PROBE DONE\n');
end
