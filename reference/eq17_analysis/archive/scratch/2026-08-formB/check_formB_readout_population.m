% FORK OF test_script/scratch/check_formB_gainswing_far.m @ untracked-2026-08-01
% (same readout ledger) | PURPOSE: gate tests for defect 2 -- does the +4~5%
% readout excess survive as a POPULATION bias (N = 48), and is it a motion-time
% effect at all (stationary-window vs oscillation-window bias, per seed)? |
% EXPIRES: when defect 2 is re-ruled | 產線改動不會自動跟上
%
% check_formB_readout_population.m
%
%   Background. On 6 seeds the osc-window readout bias a_wm/a_true reads
%   +4.40% with SEM 2.64% (1.67 sigma), and the per-seed value reproduced at
%   r = 0.9994 between two operating points three octaves apart in gain swing.
%   That is the signature of a per-realization offset, not an operating-point
%   effect. Two consequences are testable and cheap:
%
%   (a) POPULATION TEST. With N = 48 the SEM shrinks ~2.8x. If the excess is a
%       real population bias the mean stays near +4.4% and becomes significant;
%       if it was a 6-seed accident the mean moves toward 0.
%   (b) MOTION-TIME TEST. Same runs, per seed: bias measured in the STATIONARY
%       final-hold window vs the MOVING oscillation window. Slope ~1 and
%       correlation ~1 mean the bias is carried by the noise realization and is
%       present at rest, i.e. the "motion-time" framing dissolves.
%
%   Config: anchor-lock canonical (b, p, w_s all locked) -- no estimator
%   feedback into either measured quantity. Both quantities are read from the
%   logs exactly as in the falsification-#5 ledger.

clear; clc;
here = fileparts(mfilename('fullpath'));
proj = fullfile(here, '..', '..');
addpath(genpath(proj));

AX_Z      = 3;
ANCHOR_BP = [9/8 1];
N_SEEDS   = 48;
SEEDS_BIG = 1:N_SEEDS;            % deterministic list
BATCH_MAT = fullfile(proj, 'test_results', 'run_formB_ws_anchorlock_n48.mat');
PREVIEW_MAT = fullfile(proj, 'test_results', 'run_formB_ws_anchorlock.mat');

% ---------------------------------------------------------------- preview (6)
fprintf('=========== PREVIEW: existing 6-seed anchor-lock mat ===========\n');
S = load(PREVIEW_MAT);
local_window_report(S.out_anch, AX_Z, 'preview (6 seeds)');

% ---------------------------------------------------------------- batch (48)
fprintf('\n=========== BATCH: anchor-lock, N = %d seeds ===========\n', N_SEEDS);
opts = struct();
opts.oracle_bp = ANCHOR_BP;
opts.seeds     = SEEDS_BIG;
t0 = tic;
out48 = run_formB_ws(opts);
fprintf('batch elapsed %.1f s\n', toc(t0));
save(BATCH_MAT, 'out48', '-v7.3');
fprintf('saved %s\n', BATCH_MAT);

[osc48, hold48, vr48] = local_biases(out48, AX_Z);

% ------------------------------------------------- (a) population statistics
fprintf('\n================= (a) POPULATION TEST, N = %d =================\n', N_SEEDS);
local_stat_block('a_wm/a_true, OSC window',  osc48,  1.0,   1.0440);
local_stat_block('a_wm/a_true, HOLD window', hold48, 1.0,   1.0440);
local_stat_block('Var(dw_r) ratio, OSC',     vr48,   1.0,   1.0342);

% --------------------------------------------- (b) motion-time test (N = 48)
fprintf('\n============ (b) MOTION-TIME TEST: hold vs osc, N = %d ============\n', N_SEEDS);
local_pair_report(osc48, hold48);

fprintf('\n---------------------------------------------------------------\n');
fprintf('Reading: slope ~1 with r ~1 means the SAME per-seed offset is present\n');
fprintf('while stationary and while oscillating, i.e. the excess is carried by\n');
fprintf('the noise realization and is not created by the motion.\n');


% =============================== helpers ===============================

function [b_osc, b_hold, vratio] = local_biases(o, ax)
%LOCAL_BIASES  Per-seed readout bias in the osc and final-hold windows, and the
%   osc-window Var(dw_r)/formula ratio. Ledger definitions, unchanged.
    w   = o.metrics.windows;
    t   = o.runs{1}.tout(:);
    m_o = t > w.osc(1)  & t <= w.osc(2);
    m_h = t > w.hold(1);
    nS  = numel(o.seeds);
    b_osc = zeros(nS, 1); b_hold = b_osc; vratio = b_osc;
    for q = 1:nS
        r   = o.runs{q};
        K   = r.ctrl_const;
        Pv  = r.meta.params_value;
        kBT = Pv.ctrl.k_B * Pv.ctrl.T;
        s2n = Pv.ctrl.sigma2_noise(ax);                  % [um^2]
        aT  = r.a_true_out(:, ax);
        aM  = r.a_xm_out(:, ax);
        b_osc(q)  = mean(aM(m_o) ./ max(aT(m_o), eps));
        b_hold(q) = mean(aM(m_h) ./ max(aT(m_h), eps));
        v_tot     = mean(r.dx_r_out(m_o, ax).^2);
        v_pred    = K.C_dpmr * 4 * kBT * mean(aT(m_o)) + K.C_n * s2n;
        vratio(q) = v_tot / v_pred;
    end
end


function local_window_report(o, ax, tag)
    [b_osc, b_hold, vr] = local_biases(o, ax);
    fprintf('%s: osc bias %.4f (std %.4f) | hold bias %.4f (std %.4f) | Var ratio %.4f\n', ...
            tag, mean(b_osc), std(b_osc), mean(b_hold), std(b_hold), mean(vr));
    local_pair_report(b_osc, b_hold);
end


function local_pair_report(b_osc, b_hold)
%LOCAL_PAIR_REPORT  Correlation and OLS slope of hold-window bias on osc-window
%   bias, plus the paired difference.
    n = numel(b_osc);
    r = corr(b_osc(:), b_hold(:));
    X = [ones(n, 1), b_osc(:) - 1];
    c = X \ (b_hold(:) - 1);
    d = b_hold - b_osc;
    fprintf('  hold vs osc: r = %.4f, slope = %.4f, intercept = %+.4f\n', r, c(2), c(1));
    fprintf('  paired (hold - osc): mean %+.4f, std %.4f, SEM %.4f, t = %.2f\n', ...
            mean(d), std(d), std(d)/sqrt(n), mean(d)/(std(d)/sqrt(n)));
end


function local_stat_block(name, x, h0a, h0b)
%LOCAL_STAT_BLOCK  Mean/std/SEM and two-sided t tests against two hypotheses.
    n   = numel(x);
    mu  = mean(x);
    sd  = std(x);
    sem = sd / sqrt(n);
    ta  = (mu - h0a) / sem;
    tb  = (mu - h0b) / sem;
    fprintf('%s\n', name);
    fprintf('  mean %.4f (%+.2f%%), std %.4f, SEM %.4f, N = %d\n', mu, 100*(mu-1), sd, sem, n);
    fprintf('  vs %.4f : t = %+6.2f  -> %s\n', h0a, ta, local_sig(ta));
    fprintf('  vs %.4f : t = %+6.2f  -> %s\n', h0b, tb, local_sig(tb));
    fprintf('  95%% CI on the mean: [%.4f, %.4f]  (%+.2f%% .. %+.2f%%)\n', ...
            mu - 1.96*sem, mu + 1.96*sem, 100*(mu - 1.96*sem - 1), 100*(mu + 1.96*sem - 1));
end


function s = local_sig(t)
    if abs(t) >= 1.96
        s = 'DISTINGUISHABLE (95%)';
    else
        s = 'not distinguishable (95%)';
    end
end
