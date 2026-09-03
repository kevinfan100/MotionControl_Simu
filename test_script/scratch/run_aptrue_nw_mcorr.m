% FORK OF test_script/scratch/run_aptrue_mean_knobs.m (2026-09-03) | PURPOSE: does the correlated
%   process/measurement noise correction (ctrl_const.nw_mcorr, derivation 0903_formC_aptrue_update_half
%   S6) remove the +0.25e-6/step upward hold drift of the pred_mean2 recipe? Same 10 seeds, both
%   trajectories, hold extended by 4 s:
%     base     exact + slope@est + pred_mean2                 (bit-identical to aptrue_long_hold_<traj>_mean2)
%     nwmcorr  the same + nw_mcorr (predict input gn (y1 - x1_hat), F_e - gn e1', Q - R1 gn gn')
%   PRE-REGISTERED (est - true): last-3-s hold slope canon +0.239 -> 0 +- 0.05, meng +0.126 -> 0 +- 0.08;
%   hold sd unchanged within 10% (0.0026 / 0.0034); short-hold mean (first 1.3 s / 1 s) canon +0.0006 -> 0,
%   meng +0.002 -> 0 +- 0.0007. Opponent: slope unchanged => M is not the drift's source.
%   Output: aptrue_nw_mcorr_<traj>.mat + aptrue_nw_mcorr_band.png | EXPIRES: with the parent
function out = run_aptrue_nw_mcorr(traj, seeds)
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
    fprintf('[%s nwmcorr] ws0 %.5f | hold from %.2f to %.2f s | seeds %s\n', traj, ws0, t3, cfg0.T_sim, mat2str(seeds));
    ARM = {'base','nwmcorr'};  KN = {struct(), struct('nw_mcorr', true)};
    out = struct('traj', traj, 'seeds', seeds, 'phases', [t1 t2 t3 cfg0.T_sim]);  nS = numel(seeds);
    for a = 1:2
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
    for a = 2:2
        d = out.(ARM{a}).slope - out.base.slope;  pre = max(abs(out.(ARM{a}).E(t < t3, :) - out.base.E(t < t3, :)), [], 'all');
        m1 = t > t3 & t <= t3 + 1;  dE1 = mean(out.(ARM{a}).E(m1,:),1) - mean(out.base.E(m1,:),1);
        m4 = t > cfg0.T_sim - 1;    dE4 = mean(out.(ARM{a}).E(m4,:),1) - mean(out.base.E(m4,:),1);
        fprintf('[%s %-7s] PAIRED: slope diff %+.3f e-6/step (SEM %.3f) | before hold max |dE| %.1e | first-hold-s diff %+.5f (SEM %.5f) | last-s diff %+.5f (SEM %.5f)\n', ...
            traj, ARM{a}, 1e6*mean(d), 1e6*std(d)/sqrt(nS), pre, mean(dE1), std(dE1)/sqrt(nS), mean(dE4), std(dE4)/sqrt(nS));
        for b = 1:2
            EE = out.(ARM{b}).E;  ms = t > t3 & t <= t3 + 1.3;  mm = t > t3;  pm = mean(EE(ms,:),1);
            fprintf('[%s %-7s] short-hold (first 1.3 s) est-true %+.5f (SEM %.5f) | whole-hold mean %+.5f | hold sd(E) %.5f | motion-segment mean %+.5f\n', ...
                traj, ARM{b}, mean(pm), std(pm)/sqrt(nS), mean(EE(mm,:),'all'), mean(std(EE(mm,:),0,2)), mean(EE(t > t2 - 4 & t <= t2, :), 'all'));
        end
    end
    save(fullfile(od, sprintf('aptrue_nw_mcorr_%s.mat', traj)), '-struct', 'out', '-v7.3');
    fprintf('[%s nwmcorr] saved aptrue_nw_mcorr_%s.mat\n', traj, traj);
end
