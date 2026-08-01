% FORK OF test_script/integration/verify_5state_expgain_readout.m @ 7c46cef (ledger
% computation reused verbatim) | PURPOSE: formal re-adjudication of falsification #5
% (gain-swing / modulation mechanism) on the timing-fixed + post-Q formB machinery |
% EXPIRES: when #5 is re-ruled in the ledger | 產線改動不會自動跟上
%
% check_formB_gainswing_far.m -- NEAR vs FAR oscillation at IDENTICAL amplitude and
%   frequency, anchor-lock config (b, p, w_s all locked = cleanest machinery, no
%   estimator feedback into the quantities measured here).
%
%   The readout inverts one equation (S8 / Cdpmr_Cn_derivation):
%       Var(dw_r) = C_dpmr*4kBT*a_h + C_n*sigma2_n            (*)
%       a_wm      = (Var_hat(dw_r) - C_n*sigma2_n) / (C_dpmr*4kBT)
%   so a_wm is biased by exactly the amount by which (*) fails. Both quantities
%   below are read straight from the logs; no estimator sits in the path.
%
%   NEAR = the canonical scenario (trough h_bar 2.0, large gain swing over a cycle).
%   FAR  = h_bottom raised to h_init, so the descent phase is flat and the SAME
%          2.5 um / 1 Hz oscillation happens about ~22 R, where the gain swing is
%          negligible. Timings are untouched, so the metric windows coincide.
%
%   VERDICT LOGIC. The modulation mechanism attributes the excess to the gain
%   swing itself, whose power is mean(u^2), u = a_true/<a_true> - 1. If that is
%   the cause, FAR (with mean(u^2) smaller by orders of magnitude) must collapse
%   to ratio ~1.005-1.01 and a_wm/a_true ~1.00. Falsification #5 standing
%   predicts FAR keeps the NEAR-sized excess (~1.03-1.04).

clear; clc;
here = fileparts(mfilename('fullpath'));
proj = fullfile(here, '..', '..');
addpath(genpath(proj));

AX_Z    = 3;
NEAR_MAT = fullfile(proj, 'test_results', 'run_formB_ws_anchorlock.mat');
ANCHOR_BP = [9/8 1];        % anchor-lock: (b, p) pinned at the derived anchors

% ---------------- NEAR: reuse the existing post-Q anchor-lock run ----------------
assert(exist(NEAR_MAT, 'file') == 2, 'check_formB_gainswing_far:missingNear', ...
       'expected the post-Q anchor-lock run at %s', NEAR_MAT);
S = load(NEAR_MAT);
out_near = S.out_anch;

% ---------------- FAR: same amplitude / frequency, centred far from the wall ----
% h_bottom = h_init makes phase 2 (descent) flat: trajectory_generator gives
% h = h_bottom + (h_init-h_bottom)*(...) = h_init throughout, then oscillates
% between h_bottom and h_bottom + 2*amplitude. Everything else is untouched.
opts = struct();
opts.oracle_bp       = ANCHOR_BP;
opts.config_override = struct('h_bottom', 50);   % = h_init [um]
fprintf('running FAR arm (osc about ~22 R, no descent)...\n');
out_far = run_formB_ws(opts);

% ---------------------------- ledger, both arms ---------------------------------
fprintf('\n=================== FALSIFICATION #5 RE-ADJUDICATION ===================\n');
res = struct('name', {}, 'ratio', {}, 'a_ratio', {}, 'u2', {}, 'hbar', {});
for which = 1:2
    if which == 1
        o = out_near;  nm = 'NEAR (trough h_bar 2.0)';
    else
        o = out_far;   nm = 'FAR  (osc about h_bar 22)';
    end
    w = o.metrics.windows.osc;
    K = o.runs{1}.ctrl_const;
    Pv = o.runs{1}.meta.params_value;
    kBT = Pv.ctrl.k_B * Pv.ctrl.T;
    s2n = Pv.ctrl.sigma2_noise(AX_Z);              % [um^2]

    nS = numel(o.seeds);
    t  = o.runs{1}.tout(:);
    m  = t > w(1) & t <= w(2);
    DR = zeros(sum(m), nS); AT = DR; AM = DR; HB = DR;
    for q = 1:nS
        r = o.runs{q};
        DR(:, q) = r.dx_r_out(m, AX_Z);            % [um]
        AT(:, q) = r.a_true_out(m, AX_Z);          % [um/pN]
        AM(:, q) = r.a_xm_out(m, AX_Z);            % [um/pN] raw readout
        HB(:, q) = r.h_bar_true_out(m);
    end

    fprintf('\n--- %s | window [%.2f %.2f] s | h_bar %.2f..%.2f ---\n', ...
            nm, w(1), w(2), min(HB(:)), max(HB(:)));
    fprintf('%6s | %10s %10s %7s | %8s | %10s\n', 'seed', ...
            'Var meas', 'Var pred', 'ratio', 'a_wm/aT', 'mean(u^2)');
    rr = zeros(nS, 1); ar = rr; uu = rr;
    for q = 1:nS
        v_tot  = mean(DR(:, q).^2);
        a_bar  = mean(AT(:, q));
        v_pred = K.C_dpmr * 4 * kBT * a_bar + K.C_n * s2n;
        u      = AT(:, q) / mean(AT(:, q)) - 1;     % gain modulation, dimensionless
        rr(q)  = v_tot / v_pred;
        ar(q)  = mean(AM(:, q) ./ max(AT(:, q), eps));
        uu(q)  = mean(u.^2);
        fprintf('%6d | %10.4g %10.4g %7.3f | %8.4f | %10.3e\n', ...
                o.seeds(q), 1e6 * v_tot, 1e6 * v_pred, rr(q), ar(q), uu(q));
    end
    fprintf('%6s | %10s %10s %7.3f | %8.4f | %10.3e\n', 'mean', '', '', ...
            mean(rr), mean(ar), mean(uu));
    res(which).name = nm; res(which).ratio = mean(rr);
    res(which).a_ratio = mean(ar); res(which).u2 = mean(uu);
    res(which).hbar = [min(HB(:)) max(HB(:))];
end

% ------------------------------- verdict ----------------------------------------
fprintf('\n---------------------------- VERDICT ----------------------------\n');
fprintf('modulation power mean(u^2): NEAR %.3e -> FAR %.3e  (factor %.1fx smaller)\n', ...
        res(1).u2, res(2).u2, res(1).u2 / max(res(2).u2, realmin));
fprintf('Var ratio      : NEAR %.4f -> FAR %.4f   (excess %.2f%% -> %.2f%%)\n', ...
        res(1).ratio, res(2).ratio, 100*(res(1).ratio-1), 100*(res(2).ratio-1));
fprintf('a_wm/a_true    : NEAR %.4f -> FAR %.4f   (bias %.2f%% -> %.2f%%)\n', ...
        res(1).a_ratio, res(2).a_ratio, 100*(res(1).a_ratio-1), 100*(res(2).a_ratio-1));
frac_ratio = (res(2).ratio - 1) / max(res(1).ratio - 1, realmin);
frac_bias  = (res(2).a_ratio - 1) / max(res(1).a_ratio - 1, realmin);
fprintf('FAR keeps %.0f%% of the NEAR Var excess and %.0f%% of the NEAR readout bias.\n', ...
        100*frac_ratio, 100*frac_bias);
if frac_ratio < 0.35
    fprintf('=> consistent with the MODULATION mechanism (excess collapses with the swing)\n');
elseif frac_ratio > 0.65
    fprintf('=> consistent with FALSIFICATION #5 STANDING (excess survives without the swing)\n');
else
    fprintf('=> INDETERMINATE: the excess neither collapsed nor survived intact\n');
end
fprintf('-----------------------------------------------------------------\n');
