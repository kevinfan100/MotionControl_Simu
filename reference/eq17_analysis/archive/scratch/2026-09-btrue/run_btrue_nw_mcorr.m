% FORK OF test_script/scratch/run_aptrue_nw_mcorr.m (2026-09-04) | PURPOSE: b_true rung of the ladder --
%   the law reads b_true(w_bar) at the particle's true height (opts.b_true, b_true_at = 'true', slot 5
%   locked by the driver) and the slope at the ESTIMATED gain, a_bar' = b_true (1 - a_hat)^2; exact step;
%   pred_mean2 with the constant-b law curvature (c-free, no app_known). Same 10 seeds, both trajectories,
%   hold extended by 4 s:
%     base     b_true@true + exact + pred_mean2   (standard-length part must be bit-identical to the
%              09-02 arms10_seedtruth_comp_<traj>.mat 'btrue' arm = negative control)
%     nwmcorr  the same + nw_mcorr (predict input gn (y1 - x1_hat), F_e - gn e1', Q - R1 gn gn')
%   PRE-REGISTERED (est - true, written before any run):
%     (1) base short-hold mean (first 1.3 s canon / 1 s meng) reproduces 09-02: canon +0.0020, meng +0.0008
%         (within ~0.0005 / 0.0003 SEM);
%     (2) nwmcorr PAIRED last-3-s slope change equals the a'_true arm's: canon -0.25 +- 0.02, meng -0.15 +- 0.03
%         e-6/step (paired SEM was 0.005 there);
%     (3) hold sd(E) unchanged within 10% between the two arms;
%     (4) the nwmcorr slope is NOT expected to be zero: the a_hat-reading slope adds the line
%         (d a_bar'/d a_hat) e4 M_true to the error dynamics, whose mean per step (est - true) is
%         +2 b (1 - a_hat) [(1 - lc) P34 + F_dw P44] and is not in pred_mean2. Its value is computed from the
%         P log by probe_btrue_e4_line.m BEFORE this run's slopes are read; the remaining slope must match it
%         within 2 SEM. Opponent: it does not => the e4 line is not the (only) missing mean term.
%   Output: btrue_nw_mcorr_<traj>.mat (t, E, AH, AT, HB, hd, slope per arm) | EXPIRES: with the b_true rung
%   | 產線改動不會自動跟上
function out = run_btrue_nw_mcorr(traj, seeds)
    if nargin < 1 || isempty(traj); traj = 'canon'; end
    if nargin < 2 || isempty(seeds); seeds = 1:10; end
    traj = lower(traj);
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    pc = physical_constants();  EXTRA = 4.0;
    switch traj
        case 'meng'
            OV = struct('trajectory_type','osc','h_init',15,'h_bottom',2.5,'amplitude',0, ...
                        'frequency',1,'n_cycles',1,'t_hold',0.5,'t_descend_override',10,'T_sim',12.5 + EXTRA,'h_min',2.475);
            cfg0 = OV;  T_STD = 12.5;  SHORT = 1.0;
        case 'canon'
            cfg0 = canonical_scenario(0.05, 1.1, 'deep');  T_STD = cfg0.T_sim;  SHORT = 1.3;
            OV = struct('T_sim', cfg0.T_sim + EXTRA);  cfg0.T_sim = cfg0.T_sim + EXTRA;
    end
    w0bar = cfg0.h_init / pc.R;  [~, cp] = calc_correction_functions(w0bar);  at = 1/cp;
    ws0 = 1 + w0bar - 1/((8/9)*(1 - at));
    t1 = cfg0.t_hold;  t2 = t1 + cfg0.t_descend_override;  t3 = t2 + cfg0.n_cycles/cfg0.frequency;
    fprintf('[%s btrue] ws0 %.5f | hold from %.2f to %.2f s | seeds %s\n', traj, ws0, t3, cfg0.T_sim, mat2str(seeds));
    ARM = {'base','nwmcorr'};  KN = {struct(), struct('nw_mcorr', true)};
    out = struct('traj', traj, 'seeds', seeds, 'phases', [t1 t2 t3 cfg0.T_sim], 't_hold', t3, 'ws0', ws0, ...
                 'recipe', 'b_true@true + lock_b + exact step + pred_mean2 (law curvature) [+ nw_mcorr]');
    nS = numel(seeds);
    for a = 1:2
        cc = struct('ws0_perp',ws0,'law_exact_step',true,'pred_mean2',true,'nw_mcorr',false,'pred_mean2_e4',false,'fe44_Aa_scale',1);   % lock_b is set by the driver for b_true; flags explicit since the 09-04 production defaults turned them ON
        fn = fieldnames(KN{a}); for i = 1:numel(fn); cc.(fn{i}) = KN{a}.(fn{i}); end
        o = struct('arm','best','b_true',true,'b_true_at','true','ctrl_const_override',cc, ...
                   'config_override',OV,'scenario','deep','verbose',false,'seeds',seeds,'log_P_full',false);
        clear run_formC_b motion_control_law_formC_b;
        evalc('R = run_formC_b(o);');
        t = R.runs{1}.tout(:);  N = numel(t);  E = zeros(N, nS); AH = E; AT = E; HB = E;  BU = E;
        for q = 1:nS
            rr = R.runs{q}; ad = rr.a_hat_out(1,3)/rr.a_bar_hat_out(1,3);
            AH(:,q) = rr.a_bar_hat_out(:,3); AT(:,q) = rr.a_true_out(:,3)/ad; E(:,q) = AH(:,q) - AT(:,q);
            HB(:,q) = rr.h_bar_true_out(:,1);  BU(:,q) = rr.b_hat_out(:,3);
        end
        hd = R.runs{1}.p_d_out(:,3)/R.runs{1}.R;  clear R;
        m3 = t > cfg0.T_sim - 3;  k = (1:sum(m3)).';  sl = zeros(1,nS); for q = 1:nS; p = polyfit(k, E(m3,q), 1); sl(q) = p(1); end
        out.(ARM{a}) = struct('t', t, 'E', E, 'AH', AH, 'AT', AT, 'HB', HB, 'hd', hd, 'b_used', BU, 'sdE', std(AH,0,2), 'slope', sl);
        fprintf('[%s %-7s] health: min w %.4f | min a_hat %.5f | NaN %d | b_used [%.4f, %.4f] | hold sd(E) %.5f | last-3-s slope %+.3f e-6/step (SEM %.3f)\n', ...
            traj, ARM{a}, min(HB(:)), min(AH(:)), sum(~isfinite(E(:))), min(BU(:)), max(BU(:)), mean(std(E(t>t3,:),0,2)), 1e6*mean(sl), 1e6*std(sl)/sqrt(nS));
    end
    % negative control: the standard-length part of base vs the 09-02 arms10 'btrue' arm
    f0 = fullfile(od, sprintf('arms10_seedtruth_comp_%s.mat', traj));
    if exist(f0, 'file')
        S = load(f0);  n0 = numel(S.btrue.t);
        fprintf('[%s] negative control: base(1:%d) vs arms10_seedtruth_comp_%s btrue: max |dE| %.2e (seeds equal: %d)\n', ...
            traj, n0, traj, max(abs(out.base.E(1:n0,:) - S.btrue.E), [], 'all'), isequal(S.seeds(:).', seeds(:).'));
    else
        fprintf('[%s] negative control file missing: %s\n', traj, f0);
    end
    d = out.nwmcorr.slope - out.base.slope;  pre = max(abs(out.nwmcorr.E(t < t3, :) - out.base.E(t < t3, :)), [], 'all');
    m1 = t > t3 & t <= t3 + 1;  dE1 = mean(out.nwmcorr.E(m1,:),1) - mean(out.base.E(m1,:),1);
    m4 = t > cfg0.T_sim - 1;    dE4 = mean(out.nwmcorr.E(m4,:),1) - mean(out.base.E(m4,:),1);
    fprintf('[%s nwmcorr] PAIRED: slope diff %+.3f e-6/step (SEM %.3f) | before hold max |dE| %.1e | first-hold-s diff %+.5f (SEM %.5f) | last-s diff %+.5f (SEM %.5f)\n', ...
        traj, 1e6*mean(d), 1e6*std(d)/sqrt(nS), pre, mean(dE1), std(dE1)/sqrt(nS), mean(dE4), std(dE4)/sqrt(nS));
    for b = 1:2
        EE = out.(ARM{b}).E;  ms = t > t3 & t <= t3 + SHORT;  mm = t > t3;  pm = mean(EE(ms,:),1);
        mstd = t > t3 & t <= T_STD;
        fprintf('[%s %-7s] short-hold (first %.1f s) est-true %+.5f (SEM %.5f) | std-length hold mean %+.5f | whole-hold mean %+.5f | hold sd(E) %.5f | motion-segment mean %+.5f\n', ...
            traj, ARM{b}, SHORT, mean(pm), std(pm)/sqrt(nS), mean(EE(mstd,:),'all'), mean(EE(mm,:),'all'), mean(std(EE(mm,:),0,2)), mean(EE(t > t2 - 4 & t <= t2, :), 'all'));
    end
    save(fullfile(od, sprintf('btrue_nw_mcorr_%s.mat', traj)), '-struct', 'out', '-v7.3');
    fprintf('[%s btrue] saved btrue_nw_mcorr_%s.mat\n', traj, traj);
end
