% check_formB_chain_response40.m -- Option A: powered chain-response
%   measurement (Stage 3 / V1- stall, 2026-08-02).
%   40 chains x 6 stages, delta = -0.05 (true wall 0.95), chain inits SPREAD
%   over 0.85..1.10 (both sides of truth) to (i) give the transition
%   regression real slope leverage and (ii) unconfound approach-direction
%   from geometry. Per-stage and pooled regression dws = kappa*(-offset) + c:
%   kappa_measured vs kappa_claimed (from the P contraction) separates the
%   "response deficit" component; the intercept c pins any fixed push.
%   EXPIRES: when the V1- stall is resolved and re-examed.

proj = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(genpath(fullfile(proj, 'model')));
addpath(fullfile(proj, 'test_script', 'integration'));

NC = 40; NS = 6; DELTA = -0.05; TRU = 1 + DELTA;
inits = linspace(0.85, 1.10, NC);
base = struct('lock_b', false, 'lock_p', true, 'lock_ws', false);

ws = zeros(NC, NS); P = zeros(NC, NS);
for c = 1:NC
    ws0 = inits(c); Pws0 = 0.111;
    for k = 1:NS
        o = struct(); o.seeds = 90000*1 + 100*c + k; o.ws_inject = DELTA;
        o.verbose = false;
        o.ctrl_const_override = base;
        o.ctrl_const_override.Pf_ws_std = Pws0;
        o.ctrl_const_override.ws_init   = ws0;
        [~, r] = evalc('run_formB_ws(o)');
        ws0  = r.runs{1}.ws_hat_out(end, 3);
        Pws0 = r.runs{1}.P_ws_out(end, 3);
        ws(c, k) = ws0; P(c, k) = Pws0;
    end
    if mod(c, 5) == 0; fprintf('chain %d/%d done\n', c, NC); end
end
save(fullfile(proj, 'test_results', 'check_formB_chain_response40.mat'), ...
     'ws', 'P', 'inits', 'DELTA', '-v7.3');

off  = [inits(:) - TRU, ws(:, 1:end-1) - TRU];      % offset before each run
dws  = [ws(:, 1) - inits(:), diff(ws, 1, 2)];       % movement in each run
Ppre = [0.111 * ones(NC, 1), P(:, 1:end-1)];        % prior entering each run

fprintf('\n=== per-stage regression dws = kappa*(-off) + c ===\n');
fprintf('%5s %8s %10s %10s %11s %10s %9s\n', 'stage', 'priorP', ...
        'kappa', 'SE(k)', 'intercept', 'SE(c)', 'k_claimed');
for k = 1:NS
    x = -off(:, k); y = dws(:, k);
    X = [ones(NC, 1), x]; b = X \ y;
    res = y - X * b; s2 = sum(res.^2) / (NC - 2);
    cv = s2 * inv(X' * X); %#ok<MINV>
    kcl = 1 - mean(P(:, k))^2 / mean(Ppre(:, k))^2;   % claimed info fraction
    fprintf('%5d %8.4f %10.3f %10.3f %+11.4f %10.4f %9.3f\n', ...
            k, mean(Ppre(:, k)), b(2), sqrt(cv(2,2)), b(1), sqrt(cv(1,1)), kcl);
end

xs = -off(:, 2:NS); ys = dws(:, 2:NS); xs = xs(:); ys = ys(:);
X = [ones(numel(xs), 1), xs]; b = X \ ys;
res = ys - X * b; s2 = sum(res.^2) / (numel(xs) - 2);
cv = s2 * inv(X' * X); %#ok<MINV>
fprintf('POOLED stages 2..%d: kappa %.3f +- %.3f | intercept %+.4f +- %.4f (t %.2f) | N=%d\n', ...
        NS, b(2), sqrt(cv(2,2)), b(1), sqrt(cv(1,1)), b(1)/sqrt(cv(1,1)), numel(xs));

% approach-direction split (same geometry): offsets above vs below truth
up = xs > 0;  % est BELOW truth -> pull up
for tag = 1:2
    if tag == 1; m = up; nm = 'est BELOW truth (pull up)';
    else;        m = ~up; nm = 'est ABOVE truth (pull down)'; end
    X = [ones(sum(m), 1), xs(m)]; b2 = X \ ys(m);
    res = ys(m) - X * b2; s2 = sum(res.^2) / (sum(m) - 2);
    cv = s2 * inv(X' * X); %#ok<MINV>
    fprintf('  %-28s kappa %.3f +- %.3f  intercept %+.4f +- %.4f  N=%d\n', ...
            nm, b2(2), sqrt(cv(2,2)), b2(1), sqrt(cv(1,1)), sum(m));
end
% tail diagnostics
zres = res / std(res);
fprintf('residual kurtosis %.2f (gauss 3) | max|z| %.1f\n', kurtosis(ys - [ones(numel(xs),1), xs]*b), max(abs((ys - [ones(numel(xs),1) xs]*b)/sqrt(s2))));
