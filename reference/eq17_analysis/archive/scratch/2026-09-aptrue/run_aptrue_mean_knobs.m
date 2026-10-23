% FORK OF test_script/scratch/run_aptrue_long_hold.m (2026-09-03) | PURPOSE: which first-order mean
%   flow carries the +0.25e-6/step upward hold drift of the pred_mean2 recipe? Two mean-only knobs
%   (motion_control_law_formC_b, default off, P untouched), switched on at the first hold step:
%     base    knobs off (must be bit-identical to aptrue_long_hold_<traj>_mean2.mat)
%     pslope  pred_slope_scale = 1.10  -- the row-4 known-step increment a' M scaled by 10%
%     y1leg   y1_gain_leg_scale = 1.05 -- the y1 -> gain state correction K1(4) innov1 scaled by 5%
%   Hold extended by 4 s, same 10 seeds, both trajectories. Read the PAIRED last-3-s slope difference
%   (arm - base), SEM ~ 0.01e-6/step.
%   PRE-REGISTERED: the flow that carries the drift moves the slope by |dm| >= 0.1e-6/step; the other
%   by < 0.05. Both moving = the drift is the residual of two large cancelling flows (then the fix is
%   in their balance, not in either alone). Neither = the source is elsewhere (stop).
%   RESULT (2026-09-03): the knobs are NOT mean-only in effect -- hold sd(E) 0.0026 -> 0.0062 (pslope) /
%   0.0039 (y1leg) on canon, 0.0034 -> 0.0070 / 0.0060 on meng: the two flows ARE the fluctuation-mode
%   machinery (rank-1 cancellation l41 ~ -a' l31), so a 5-10% imbalance lets the gain jitter grow and
%   the paired slope SEM is 0.35-0.74 instead of 0.01. Identification FAILED its own pre-registration.
%   What survives (paired end-of-hold level, 2-3 sigma, same sign on both trajectories): a' M x 1.10
%   moves the level by -0.0043 +- 0.0022 (canon) / -0.0054 +- 0.0030 (meng); y1 leg x 1.05 by
%   +0.0033 +- 0.0012 / +0.0043 +- 0.0017: the two flows are each ~5-8e-6/step and opposite, the
%   +0.25e-6/step drift is a ~3-5% residual of their cancellation. Negative control: base bit-identical
%   to aptrue_long_hold_<traj>_mean2 (max |dE| = 0), knobs inert before the hold. Stop-line.
%   Output: aptrue_mean_knobs_<traj>.mat + aptrue_mean_knobs_band.png | EXPIRES: with the parent
function out = run_aptrue_mean_knobs(traj, seeds)
    if nargin < 1 || isempty(traj); traj = 'canon'; end
    if nargin < 2 || isempty(seeds); seeds = 1:10; end
    recipe = 'mean2';
    traj = lower(traj);
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    pc = physical_constants();  EXTRA = 4.0;
    switch traj
        case 'meng'
            OV = struct('trajectory_type','osc','h_init',15,'h_bottom',2.5,'amplitude',0, ...
                        'frequency',1,'n_cycles',1,'t_hold',0.5,'t_descend_override',10,'T_sim',12.5 + EXTRA,'h_min',2.475);
            cfg0 = OV;
        case 'canon'
            cfg0 = canonical_scenario(0.05, 1.1, 'deep');  OV = struct('T_sim', cfg0.T_sim + EXTRA);  cfg0.T_sim = cfg0.T_sim + EXTRA;
    end
    w0bar = cfg0.h_init / pc.R;  [~, cp] = calc_correction_functions(w0bar);  at = 1/cp;
    ws0 = 1 + w0bar - 1/((8/9)*(1 - at));
    t1 = cfg0.t_hold;  t2 = t1 + cfg0.t_descend_override;  t3 = t2 + cfg0.n_cycles/cfg0.frequency;
    k_hold = round(t3 / pc.Ts) + 1;
    fprintf('[%s knobs] ws0 %.5f | hold from %.2f to %.2f s (k_step %d) | seeds %s\n', traj, ws0, t3, cfg0.T_sim, k_hold, mat2str(seeds));
    ARM = {'base','pslope','y1leg'};  KN = {struct(), struct('mean_knob_from',k_hold,'pred_slope_scale',1.10), struct('mean_knob_from',k_hold,'y1_gain_leg_scale',1.05)};
    out = struct('traj', traj, 'seeds', seeds, 'phases', [t1 t2 t3 cfg0.T_sim]);  nS = numel(seeds);
    for a = 1:3
        cc = struct('lock_b',true,'ws0_perp',ws0,'law_exact_step',true,'pred_mean2',true);
        fn = fieldnames(KN{a}); for i = 1:numel(fn); cc.(fn{i}) = KN{a}.(fn{i}); end
        o = struct('arm','best','ap_known',true,'ap_known_at','est','app_known',true, 'ctrl_const_override',cc, ...
                   'config_override',OV,'scenario','deep','verbose',false,'seeds',seeds,'log_P_full',false);
        clear run_formC_b motion_control_law_formC_b;
        evalc('R = run_formC_b(o);');
        t = R.runs{1}.tout(:);  N = numel(t);  E = zeros(N, nS); AH = E; HB = E;
        for q = 1:nS
            rr = R.runs{q}; ad = rr.a_hat_out(1,3)/rr.a_bar_hat_out(1,3);
            AH(:,q) = rr.a_bar_hat_out(:,3); E(:,q) = AH(:,q) - rr.a_true_out(:,3)/ad; HB(:,q) = rr.h_bar_true_out(:,1);
        end
        clear R;
        m3 = t > cfg0.T_sim - 3;  k = (1:sum(m3)).';  sl = zeros(1,nS); for q = 1:nS; p = polyfit(k, E(m3,q), 1); sl(q) = p(1); end
        out.(ARM{a}) = struct('t', t, 'E', E, 'sdE', std(AH,0,2), 'slope', sl);
        fprintf('[%s %-6s] health: min w %.4f | min a_hat %.5f | NaN %d | hold sd(E) %.5f | last-3-s slope %+.3f e-6/step (SEM %.3f)\n', ...
            traj, ARM{a}, min(HB(:)), min(AH(:)), sum(~isfinite(E(:))), mean(std(E(t>t3,:),0,2)), 1e6*mean(sl), 1e6*std(sl)/sqrt(nS));
    end
    S = load(fullfile(od, sprintf('aptrue_long_hold_%s_mean2.mat', traj)));
    fprintf('[%s] negative control: base vs aptrue_long_hold_%s_mean2: max |dE| %.2e\n', traj, traj, max(abs(out.base.E - S.E), [], 'all'));
    for a = 2:3
        d = out.(ARM{a}).slope - out.base.slope;  pre = max(abs(out.(ARM{a}).E(t < t3, :) - out.base.E(t < t3, :)), [], 'all');
        m1 = t > t3 & t <= t3 + 1;  dE1 = mean(out.(ARM{a}).E(m1,:),1) - mean(out.base.E(m1,:),1);
        m4 = t > cfg0.T_sim - 1;    dE4 = mean(out.(ARM{a}).E(m4,:),1) - mean(out.base.E(m4,:),1);
        fprintf('[%s %-6s] PAIRED: slope diff %+.3f e-6/step (SEM %.3f) | before hold max |dE| %.1e | first-hold-s diff %+.5f (SEM %.5f) | last-s diff %+.5f (SEM %.5f)\n', ...
            traj, ARM{a}, 1e6*mean(d), 1e6*std(d)/sqrt(nS), pre, mean(dE1), std(dE1)/sqrt(nS), mean(dE4), std(dE4)/sqrt(nS));
    end
    save(fullfile(od, sprintf('aptrue_mean_knobs_%s.mat', traj)), '-struct', 'out', '-v7.3');
    fprintf('[%s knobs] saved aptrue_mean_knobs_%s.mat\n', traj, traj);
end
