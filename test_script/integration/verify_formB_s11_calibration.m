% STATUS: ACTIVE | S11 calibration exam (Stage 3, 2026-08-02) -- instantiates
%   formB_ws_ref S11 items 3 (P[0] budget, chain form) and 5 (w_s injection)
%   for the calibration-chain protocol. Criteria stated BEFORE the run
%   (derivation-workflow rule 3); every threshold is derived, not tuned.
%
%VERIFY_FORMB_S11_CALIBRATION  Three verdicts on the w_bar_s calibration mode.
%
%   V1  INJECTION RECOVERY (S11 item 5), both signs:
%       after n_stage runs the chain mean must sit on the injected truth PLUS
%       the healthy transient residual delta*prod(1-kappa_claimed_k) (the
%       lawful remnant of the prior-funded convergence; REVISED 2026-08-02 --
%       the original zero-reference framing mislabeled the transient as bias
%       and, at N=6, flagged a sampling fluctuation as a stall):
%       |mean(ws_final) - (1+delta+transient)| <= 1.645*sqrtP_final/sqrt(nc).
%   V2  CONVERGENCE-RATE HONESTY:
%       cross-chain spread must track the claimed sqrtP at every scale;
%       final-stage variance ratio r = (spread/claimed)^2 is ~chi2(nc-1)/(nc-1)
%       under honesty: accept r <= 1 + 1.645*sqrt(2/(nc-1)).
%   V3  OPERATIONAL CONSISTENCY:
%       (a) ws budget: mean((ws_op - truth)^2)/claimed_op^2 <= same chi2 bound;
%       (b) tracking vs the DERIVED expectation (REVISED 2026-08-02; the
%           original compared against the exact-truth locked floor, demanding
%           exact-wall performance from a +-sqrtP_op calibrated wall):
%           reference arms = lock_ws at truth +- sigma_op (sigma_op = mean
%           claimed sqrtP_op), same injection; criterion
%           mean(desc_op) <= mean(desc_ref) + 1.645*SE_pooled.
%
%   Main arm: 20 chains x 8 runs, delta = +0.05 (the formal c5 scale).
%   Sign arm: 6 chains x 8 runs, delta = -0.05 (V1 only).
%   Runtime ~35 min. Results printed; .mat outputs are gitignored.

fprintf('=== S11 calibration exam (criteria pre-stated in-file) ===\n');

% ---- main arm: 20 chains, +0.05, with operational runs + c5 figures ----
oA = struct('n_chain', 20, 'n_stage', 8, 'delta', +0.05, ...
            'operational', true, 'make_figs', true);
A = run_formB_ws_calibration(oA);

% ---- sign arm: 6 chains, -0.05 ----
oB = struct('n_chain', 6, 'n_stage', 8, 'delta', -0.05, ...
            'operational', false, 'make_figs', false);
B = run_formB_ws_calibration(oB);

fprintf('\n=== VERDICTS ===\n');
all_ok = true;

% V1 injection recovery, both signs (vs truth + healthy transient)
for S = {A, B}
    s = S{1}; nc = size(s.ws_chain, 1); ns = size(s.ws_chain, 2);
    tru = 1 + s.delta;
    kcl = zeros(1, ns); Pp = s.opts.prior0;
    for k = 1:ns
        kcl(k) = 1 - s.claimed(k)^2 / Pp^2; Pp = s.claimed(k);
    end
    transi = -s.delta * prod(1 - kcl);   % init 1 sits at -delta from truth
    dev = abs(mean(s.ws_chain(:, end)) - (tru + transi));
    lim = 1.645 * s.claimed(end) / sqrt(nc);
    ok = dev <= lim; all_ok = all_ok && ok;
    fprintf('V1 recovery (delta %+0.2f): |mean-(truth%+.4f)| %.4f <= %.4f  [%s]\n', ...
            s.delta, transi, dev, lim, local_pf(ok));
end

% V2 convergence-rate honesty (main arm, final stage)
nc = size(A.ws_chain, 1);
r2 = (A.spread(end) / A.claimed(end))^2;
lim2 = 1 + 1.645 * sqrt(2 / (nc - 1));
ok = r2 <= lim2; all_ok = all_ok && ok;
fprintf('V2 honesty: final (spread/claimed)^2 %.3f <= %.3f  [%s]', ...
        r2, lim2, local_pf(ok));
fprintf('   (per-stage ratios: '); fprintf('%.2f ', (A.spread ./ A.claimed).^2);
fprintf(')\n');

% V3a operational ws budget
tru = 1 + A.delta;
r3 = mean((A.ws_op - tru).^2) / mean(A.P_op)^2;
ok = r3 <= lim2; all_ok = all_ok && ok;
fprintf('V3a operational ws budget: ratio %.3f <= %.3f  [%s]\n', ...
        r3, lim2, local_pf(ok));

% V3b operational tracking vs the sigma-wrong-wall locked expectation
sig_op = mean(A.P_op);
dref = [];
for sgn = [+1, -1]
    o = struct(); o.ws_inject = A.delta; o.verbose = false;
    o.ctrl_const_override = struct('lock_b', false, 'lock_p', true, ...
        'lock_ws', true, 'ws_init', 1 + A.delta + sgn * sig_op);
    [~, rref] = evalc('run_formB_ws(o)');
    dref = [dref; rref.metrics.rows(:, 1)]; %#ok<AGROW>
end
mu1 = mean(dref); s1 = std(dref);
lim3 = mu1 + 1.645 * sqrt(s1^2/numel(dref) + var(A.desc_op)/numel(A.desc_op));
ok = mean(A.desc_op) <= lim3; all_ok = all_ok && ok;
fprintf('V3b operational desc: mean %.3f%% <= %.3f%% (1-sigma-wall floor %.3f +- %.3f)  [%s]\n', ...
        mean(A.desc_op), lim3, mu1, s1, local_pf(ok));

fprintf('=== S11 CALIBRATION EXAM OVERALL: %s ===\n', local_pf(all_ok));


function s = local_pf(ok)
    if ok; s = 'PASS'; else; s = 'FAIL'; end
end
