% FORK OF test_script/scratch/run_aptrue_cmd_vs_est.m (2026-09-03) | PURPOSE: the two closed
%   forms of the y1-noise feedthrough term on the FINAL a'_true @est recipe (exact step +
%   slope/curvature at the filter's height + pred_mean2), same 10 seeds, both trajectories:
%     left  = pred_mean2_kr1       a'' (1-lc)^2 K31 R1   (dw3_hat share only, 09-02)
%     right = pred_mean2_kr1_full  a'' (1-lc)   K31 R1   (all three shares, 09-03)
%   PRE-REGISTERED (from analyze_aptrue_est_final.m, 8 seeds): the predict-stage remainder
%   after kr1 equals a'' (1-lc) lc K31 R1 in every near-wall segment (meng near -0.229 vs
%   -0.200, hold -0.472 vs -0.473; canon hold -0.513 vs -0.486, 1e-6/step), so the right
%   arm's remainder is 0 +- 0.05e-6/step and its hold mean (est - true) is 0 +- 2 SEM:
%   meng from +0.0011 (SEM 0.0007), canon from +0.00015 (SEM 0.0003). Spread unchanged.
%   Output: test_results/apd_acov_meng/aptrue_kr1_full_<traj>.mat, plotted by
%   plot_aptrue_kr1_full.m | EXPIRES: with the parent | 產線改動不會自動跟上
function out = run_aptrue_kr1_full(traj, seeds)
    if nargin < 1 || isempty(traj); traj = 'canon'; end
    if nargin < 2 || isempty(seeds); seeds = 1:10; end
    traj = lower(traj);
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    pc = physical_constants();
    switch traj
        case 'meng'
            OV = struct('trajectory_type','osc','h_init',15,'h_bottom',2.5,'amplitude',0, ...
                        'frequency',1,'n_cycles',1,'t_hold',0.5,'t_descend_override',10,'T_sim',12.5,'h_min',2.475);
            cfg0 = OV;
        case 'canon'
            OV = struct();  cfg0 = canonical_scenario(0.05, 1.1, 'deep');
    end
    w0bar = cfg0.h_init / pc.R;  [~, cp] = calc_correction_functions(w0bar);  at = 1/cp;
    ws0 = 1 + w0bar - 1/((8/9)*(1 - at));
    t1 = cfg0.t_hold;  t2 = t1 + cfg0.t_descend_override;  t3 = t2 + cfg0.n_cycles/cfg0.frequency;
    fprintf('[%s] ws0 %.5f | phases hold<%.2f descend<%.2f osc<%.2f hold<=%.2f | seeds %s\n', traj, ws0, t1, t2, t3, cfg0.T_sim, mat2str(seeds));
    base = struct('arm','best','ap_known',true,'ap_known_at','est','app_known',true, ...
                  'ctrl_const_override',struct('lock_b',true,'ws0_perp',ws0,'law_exact_step',true,'pred_mean2',true,'pred_mean2_kr1',true), ...
                  'config_override',OV,'scenario','deep','verbose',false,'seeds',seeds,'log_P_full',true);
    O = {base, base};  O{2}.ctrl_const_override.pred_mean2_kr1_full = true;  TAG = {'left','right'};  LAB = {'kr1','kr1_full'};
    ZF = {'a_bar_hat_out','P_a_out','P_b_out','K_a_y1_out','K_a_y2_out','K_dx_y1_out','K_dx_y2_out', ...
          'innov_y1_out','innov_y2_out','a_prime_out','a_prime_true_out','delta_x_hat_3_out','pred_mean2_out'};
    out = struct('traj', traj, 'seeds', seeds, 'ws0', ws0, 'phases', [t1 t2 t3 cfg0.T_sim]);
    nS = numel(seeds);
    for a = 1:2
        clear run_formC_b motion_control_law_formC_b;
        evalc('R = run_formC_b(O{a});');
        t = R.runs{1}.tout(:);  N = numel(t);
        E = zeros(N, nS); AH = E; P44 = E; C = struct(); for f = ZF; C.(f{1}) = zeros(N, nS); end
        C.a_true_norm = zeros(N, nS); C.h_bar_true = C.a_true_norm; C.trk_true = C.a_true_norm; C.P33_sqrt = C.a_true_norm; C.ad = zeros(1, nS);
        for q = 1:nS
            r = R.runs{q}; ad = r.a_hat_out(1,3)/r.a_bar_hat_out(1,3);
            AH(:,q) = r.a_bar_hat_out(:,3); E(:,q) = AH(:,q) - r.a_true_out(:,3)/ad; P44(:,q) = r.P_a_out(:,3)/ad;
            for f = ZF; C.(f{1})(:,q) = r.(f{1})(:,3); end
            C.a_true_norm(:,q) = r.a_true_out(:,3)/ad; C.h_bar_true(:,q) = r.h_bar_true_out(:,1); C.ad(q) = ad;
            C.trk_true(:,q) = (r.p_d_out(:,3) - r.p_true_out(:,3)) / r.R;
            C.P33_sqrt(:,q) = sqrt(squeeze(r.P_full_out(:,3,3,3)));  R.runs{q}.P_full_out = [];
        end
        hd = R.runs{1}.p_d_out(:,3)/R.runs{1}.R;
        m_h = t > t3;  m_o = t > t2 & t <= t3;  pm = mean(E(m_h,:),1);
        fprintf('[%s %-9s] health: min w %.4f | min a_hat %.5f | NaN %d | E(0) %+.1e | mean2 sum/seed %+.5f\n', ...
            traj, LAB{a}, min(C.h_bar_true(:)), min(AH(:)), sum(~isfinite(E(:))), mean(E(1,:)), mean(sum(C.pred_mean2_out,1)));
        fprintf('[%s %-9s] osc/desc %+.5f | hold %+.5f (SEM %.5f) | sd(E) hold %.5f vs sqrt(P44) %.5f\n', ...
            traj, LAB{a}, mean(mean(E(m_o,:),1)), mean(pm), std(pm)/sqrt(nS), mean(std(E(m_h,:),0,2)), mean(mean(P44(m_h,:),2)));
        out.(TAG{a}) = struct('t',t,'E',E,'sdE',std(AH,0,2),'sP',mean(P44,2),'hd',hd,'L41',C.K_a_y1_out,'L42',C.K_a_y2_out,'C',C,'label',LAB{a});
        clear R;
    end
    d = mean(out.left.E(t > t3,:),1) - mean(out.right.E(t > t3,:),1);
    fprintf('[%s] paired hold kr1 - kr1_full %+.5f (SEM %.5f)\n', traj, mean(d), std(d)/sqrt(nS));
    save(fullfile(od, sprintf('aptrue_kr1_full_%s.mat', traj)), '-struct', 'out', '-v7.3');
    fprintf('[%s] saved aptrue_kr1_full_%s.mat\n', traj, traj);
end
