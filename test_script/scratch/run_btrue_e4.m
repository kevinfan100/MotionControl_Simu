% FORK OF test_script/scratch/run_btrue_nw_mcorr.m (2026-09-04) | PURPOSE: third rung of the b_true ladder --
%   the gain-reading start-point term (ctrl_const.pred_mean2_e4, 0903_aptrue_4state_from_true.tex S10) on top
%   of b_true@true + exact + pred_mean2 + nw_mcorr. Same 10 seeds, both trajectories, hold extended by 4 s:
%     nwmcorr  re-run of run_btrue_nw_mcorr's nwmcorr arm (must be bit-identical to btrue_nw_mcorr_<traj>.mat:
%              the new flag is default off)
%     e4       the same + pred_mean2_e4
%   PRE-REGISTERED (est - true; the prediction is the running sum of -RESID_mcorr of probe_btrue_e4_line.m,
%   i.e. of -2 b (1 - a_hat) Cov(e4 + a' e3, u) evaluated on the base arm's P):
%     (1) PAIRED e4 - nwmcorr at the start of the hold: canon +0.0009, meng +0.0018 (within 30%; the term is a
%         deterministic input, its closed-loop response differs from the open sum by the y2 leg only, tau ~ 12 s);
%     (2) PAIRED last-3-s hold slope change 0 +- 0.02 e-6/step (RESID ~ 0 in the hold);
%     (3) hold sd(E) unchanged within 5%;
%     (4) the near-wall descent offset (mean of E over the last 4 s of the descent) moves toward zero by the
%         predicted amount: canon -0.0010 -> ~ -0.0002, meng -0.0052 -> ~ -0.0035.
%   Opponent for (1): the paired shift is < 50% of the prediction => the flag is not wired to the predict, or the
%   term is consumed by the y1 leg within the step (check the pre-hold profile).
%   Output: btrue_e4_<traj>.mat | EXPIRES: with the b_true rung | 產線改動不會自動跟上
function out = run_btrue_e4(traj, seeds)
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
    fprintf('[%s btrue e4] ws0 %.5f | hold from %.2f to %.2f s | seeds %s\n', traj, ws0, t3, cfg0.T_sim, mat2str(seeds));
    ARM = {'nwmcorr','e4'};  KN = {struct('nw_mcorr', true), struct('nw_mcorr', true, 'pred_mean2_e4', true)};
    out = struct('traj', traj, 'seeds', seeds, 'phases', [t1 t2 t3 cfg0.T_sim], 't_hold', t3, 'ws0', ws0, ...
                 'recipe', 'b_true@true + lock_b + exact step + pred_mean2 + nw_mcorr [+ pred_mean2_e4]');
    nS = numel(seeds);
    for a = 1:2
        cc = struct('ws0_perp',ws0,'law_exact_step',true,'pred_mean2',true);
        fn = fieldnames(KN{a}); for i = 1:numel(fn); cc.(fn{i}) = KN{a}.(fn{i}); end
        o = struct('arm','best','b_true',true,'b_true_at','true','ctrl_const_override',cc, ...
                   'config_override',OV,'scenario','deep','verbose',false,'seeds',seeds,'log_P_full',false);
        clear run_formC_b motion_control_law_formC_b;
        evalc('R = run_formC_b(o);');
        t = R.runs{1}.tout(:);  N = numel(t);  E = zeros(N, nS); AH = E; AT = E; HB = E;  PM = E;
        for q = 1:nS
            rr = R.runs{q}; ad = rr.a_hat_out(1,3)/rr.a_bar_hat_out(1,3);
            AH(:,q) = rr.a_bar_hat_out(:,3); AT(:,q) = rr.a_true_out(:,3)/ad; E(:,q) = AH(:,q) - AT(:,q);
            HB(:,q) = rr.h_bar_true_out(:,1);  PM(:,q) = rr.pred_mean2_out(:,3);
        end
        hd = R.runs{1}.p_d_out(:,3)/R.runs{1}.R;  clear R;
        m3 = t > cfg0.T_sim - 3;  k = (1:sum(m3)).';  sl = zeros(1,nS); for q = 1:nS; p = polyfit(k, E(m3,q), 1); sl(q) = p(1); end
        out.(ARM{a}) = struct('t', t, 'E', E, 'AH', AH, 'AT', AT, 'HB', HB, 'hd', hd, 'pm2', PM, 'sdE', std(AH,0,2), 'slope', sl);
        fprintf('[%s %-7s] health: min w %.4f | min a_hat %.5f | NaN %d | hold sd(E) %.5f | last-3-s slope %+.3f e-6/step (SEM %.3f) | sum pred_mean2 @t3 %+.5f\n', ...
            traj, ARM{a}, min(HB(:)), min(AH(:)), sum(~isfinite(E(:))), mean(std(E(t>t3,:),0,2)), 1e6*mean(sl), 1e6*std(sl)/sqrt(nS), mean(sum(PM(t <= t3, :), 1)));
    end
    S = load(fullfile(od, sprintf('btrue_nw_mcorr_%s.mat', traj)));
    fprintf('[%s] negative control: nwmcorr re-run vs btrue_nw_mcorr_%s nwmcorr: max |dE| %.2e\n', traj, traj, max(abs(out.nwmcorr.E - S.nwmcorr.E), [], 'all'));
    Pb = load(fullfile(od, sprintf('probe_btrue_e4_line_%s.mat', traj)));
    pred = -mean(cumsum(Pb.TERM(:,:,6), 1), 2);           % -RESID mcorr, running sum, est - true
    dE = out.e4.E - out.nwmcorr.E;  md = mean(dE, 2);
    i2 = find(t > t2, 1);  i3 = find(t > t3, 1);
    fprintf('[%s e4] PAIRED shift e4 - nwmcorr: end of descent %+.5f (SEM %.5f) vs predicted %+.5f | hold start %+.5f (SEM %.5f) vs predicted %+.5f | end of run %+.5f vs %+.5f\n', ...
        traj, md(i2), std(dE(i2,:))/sqrt(nS), pred(i2), md(i3), std(dE(i3,:))/sqrt(nS), pred(i3), md(end), pred(end));
    d = out.e4.slope - out.nwmcorr.slope;
    fprintf('[%s e4] PAIRED last-3-s slope diff %+.3f e-6/step (SEM %.3f) | hold sd(E) %.5f -> %.5f | sum pred_mean2 diff @t3 %+.5f\n', ...
        traj, 1e6*mean(d), 1e6*std(d)/sqrt(nS), mean(std(out.nwmcorr.E(t>t3,:),0,2)), mean(std(out.e4.E(t>t3,:),0,2)), mean(sum(out.e4.pm2(t <= t3,:) - out.nwmcorr.pm2(t <= t3,:), 1)));
    for b = 1:2
        EE = out.(ARM{b}).E;  ms = t > t3 & t <= t3 + SHORT;  mm = t > t3;  pm = mean(EE(ms,:),1);
        fprintf('[%s %-7s] short-hold (first %.1f s) est-true %+.5f (SEM %.5f) | whole-hold mean %+.5f | motion-segment mean (last 4 s of descent) %+.5f (SEM %.5f)\n', ...
            traj, ARM{b}, SHORT, mean(pm), std(pm)/sqrt(nS), mean(EE(mm,:),'all'), mean(EE(t > t2 - 4 & t <= t2, :), 'all'), std(mean(EE(t > t2 - 4 & t <= t2, :), 1))/sqrt(nS));
    end
    save(fullfile(od, sprintf('btrue_e4_%s.mat', traj)), '-struct', 'out', '-v7.3');
    fprintf('[%s btrue e4] saved btrue_e4_%s.mat\n', traj, traj);
end
