% FORK OF test_script/scratch/run_aptrue_kr1_full.m (2026-09-03) | PURPOSE: does the near-wall
%   hold down-drift of the final recipe (exact + @est + pred_mean2 + kr1_full) come through the
%   y1 -> gain leg? Same 10 seeds, both trajectories:
%     left  = final recipe
%     right = final recipe + ctrl_const.y1_gain_off_from = first hold step (K1(4:7) = 0 from there)
%   PRE-REGISTERED: [hypothesis] a mean y1 innovation offset iota ~ -1.3e-5 R drives the gain row
%   through l41 = -0.51 (predict leg a' l31 iota and gain leg l41 iota cancel to 8%); switching the
%   gain leg off leaves the predict leg alone, so the paired hold difference (right - left, est - true)
%   = -l41 iota N_hold = -(0.51)(1.3e-5)(2080) ~ -0.013 on canon (meng, 3200 steps: ~ -0.02).
%   [opponent] iota ~ 0, drift from Cov(a', u) or the y2 offset: |paired difference| < 0.001.
%   Output: aptrue_y1leg_hold_<traj>.mat, plotted by plot_aptrue_y1leg_hold.m | EXPIRES: with the parent
function out = run_aptrue_y1leg_hold(traj, seeds)
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
    base.ctrl_const_override.pred_mean2_kr1_full = true;
    k_hold = round(t3 / pc.Ts) + 1;                       % first controller step of the hold (log row k has k_step = k)
    O = {base, base};  O{2}.ctrl_const_override.y1_gain_off_from = k_hold;  TAG = {'left','right'};  LAB = {'final','y1leg_off'};
    fprintf('[%s] y1 gain leg off from k_step %d (t = %.3f s)\n', traj, k_hold, (k_hold-1)*pc.Ts);
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
    fprintf('[%s] paired hold final - y1leg_off %+.5f (SEM %.5f)\n', traj, mean(d), std(d)/sqrt(nS));
    m_end = t > t(end) - 0.3;  d2 = mean(out.right.E(m_end,:),1) - mean(out.left.E(m_end,:),1);
    fprintf('[%s] paired LAST 0.3 s: y1leg_off - final %+.5f (SEM %.5f)  => iota = d / (-l41 N) with l41 %.3f, N %d: %+.2e R\n', ...
        traj, mean(d2), std(d2)/sqrt(nS), mean(out.left.L41(t > t3, :), 'all'), sum(t > t3), mean(d2) / (-mean(out.left.L41(t > t3, :), 'all') * sum(t > t3)));
    save(fullfile(od, sprintf('aptrue_y1leg_hold_%s.mat', traj)), '-struct', 'out', '-v7.3');
    fprintf('[%s] saved aptrue_y1leg_hold_%s.mat\n', traj, traj);
end
