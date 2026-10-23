% FORK OF test_script/scratch/run_arms_seedtruth_pair.m @ 075cee5 | PURPOSE: the same
%   seed-at-truth oracle pair on both trajectories, but with the a'_true section's
%   COMPENSATED method in both arms (2026-09-02, user request, 10 seeds):
%     aptrue    = ladder arm 4: ap_known @est + app_known + pred_mean2 + lock_b + exact step
%     btrue     = b_true @true + exact step + pred_mean2 (law curvature, c-free: app_known
%                 is only fed with ap_known, so this is the production-like form)
%     btrue_ref = b_true @true + exact step, NO pred_mean2 (reference: what pred_mean2 does)
%   Same compact schema as the parent so plot_arms_seedtruth_pair_comp.m (fork of the
%   parent's plot) renders the 5x2 panel. Output: arms10_seedtruth_comp_<traj>.mat
%   EXPIRES: when pred_mean2 for the b_true arm gets its own derivation (rung 3) | 產線改動不會自動跟上
function out = run_arms_seedtruth_pair_comp(traj, SEEDS)
    if nargin < 1 || isempty(traj);  traj = 'meng'; end
    if nargin < 2 || isempty(SEEDS); SEEDS = 1:10; end
    traj = lower(traj);
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    pc = physical_constants();
    switch traj
        case 'meng'
            OV = struct('trajectory_type','osc','h_init',15,'h_bottom',2.5,'amplitude',0, ...
                        'frequency',1,'n_cycles',1,'t_hold',0.5,'t_descend_override',10, ...
                        'T_sim',12.5,'h_min',2.475);
            cfg0 = OV;
        case 'canon'
            OV = struct(); cfg0 = canonical_scenario(0.05, 1.1, 'deep');
    end
    fname = sprintf('arms10_seedtruth_comp_%s.mat', traj);
    w0bar = cfg0.h_init / pc.R; [~, cp] = calc_correction_functions(w0bar); at = 1/cp;
    ws0 = 1 + w0bar - 1/((8/9)*(1 - at));
    t1 = cfg0.t_hold;  t2 = t1 + cfg0.t_descend_override;  t3 = t2 + cfg0.n_cycles/cfg0.frequency;
    fprintf('[%s] w_bar0 %.4f -> a_true %.5f -> ws0_perp %.5f | phases: hold<%.2f descend<%.2f osc<%.2f hold<=%.2f\n', ...
            traj, w0bar, at, ws0, t1, t2, t3, cfg0.T_sim);
    base = struct('arm','best','config_override',OV,'scenario','deep','verbose',false,'seeds',SEEDS,'log_P_full',true);
    A = base; A.ap_known = true; A.ap_known_at = 'est'; A.app_known = true;
    A.ctrl_const_override = struct('lock_b',true,'ws0_perp',ws0,'law_exact_step',true,'pred_mean2',true);
    B = base; B.b_true = true; B.b_true_at = 'true';
    B.ctrl_const_override = struct('ws0_perp',ws0,'law_exact_step',true,'pred_mean2',true);
    Bref = B; Bref.ctrl_const_override = struct('ws0_perp',ws0,'law_exact_step',true);
    ARMO = {A, B, Bref}; TAG = {'aptrue','btrue','btrue_ref'};
    out = struct('ws0', ws0, 'traj', traj, 'phases', [t1 t2 t3 cfg0.T_sim], 'seeds', SEEDS, ...
                 'method', 'exact step + pred_mean2 (+ ap_known@est + app_known on aptrue)');
    for a = 1:3
        clear run_formC_b motion_control_law_formC_b;
        LOG = evalc('R = run_formC_b(ARMO{a});');
        t = R.runs{1}.tout(:); nS = numel(SEEDS);
        ZF = {'a_bar_hat_out','P_a_out','P_b_out','K_a_y1_out','K_a_y2_out', ...
              'K_dx_y1_out','K_dx_y2_out','innov_y1_out','innov_y2_out', ...
              'a_prime_out','a_prime_true_out','delta_x_hat_3_out','pred_mean2_out'};
        E = zeros(numel(t), nS); AH = E; P44 = E;
        C = struct(); for f = ZF; C.(f{1}) = zeros(numel(t), nS); end
        C.a_true_norm = zeros(numel(t), nS); C.h_bar_true = C.a_true_norm; C.trk_true = C.a_true_norm; C.P33_sqrt = C.a_true_norm; C.ad = zeros(1, nS);
        for q = 1:nS
            r = R.runs{q}; ad = r.a_hat_out(1,3)/r.a_bar_hat_out(1,3);
            AH(:,q) = r.a_bar_hat_out(:,3); E(:,q) = AH(:,q) - r.a_true_out(:,3)/ad;
            P44(:,q) = r.P_a_out(:,3)/ad;
            for f = ZF; C.(f{1})(:,q) = r.(f{1})(:,3); end
            C.a_true_norm(:,q) = r.a_true_out(:,3)/ad; C.h_bar_true(:,q) = r.h_bar_true_out(:,1); C.ad(q) = ad;
            C.trk_true(:,q) = (r.p_d_out(:,3) - r.p_true_out(:,3)) / r.R;
            C.P33_sqrt(:,q) = sqrt(squeeze(r.P_full_out(:,3,3,3)));
            R.runs{q}.P_full_out = [];
        end
        hd = R.runs{1}.p_d_out(:,3)/R.runs{1}.R;
        fprintf('[%s %s] health: min w_bar_true %.4f | min a_bar_hat %.5f | NaN in E %d | E(t=0) %+.2e | mean2 sum/seed %+.5f\n', ...
                traj, TAG{a}, min(C.h_bar_true(:)), min(AH(:)), sum(~isfinite(E(:))), mean(E(1,:)), mean(sum(C.pred_mean2_out,1)));
        W = {'descend', t> t1 & t<=t2; 'osc/flat', t> t2 & t<=t3; 'hold', t> t3};
        if strcmp(traj,'meng'); W(end+1,:) = {'legacy nearwall', t>=7.5 & t<=10.5}; end
        for w = 1:size(W,1)
            m = W{w,2}; pm = mean(E(m,:),1);
            fprintf('[%s %s] %-16s E %+.5f (SEM %.5f)  sd %.5f  sqrtP44 %.5f  honesty %.2f\n', traj, TAG{a}, W{w,1}, ...
                mean(pm), std(pm)/sqrt(nS), mean(std(E(m,:),0,2)), mean(P44(m,:),'all'), mean(std(E(m,:),0,2))/mean(P44(m,:),'all'));
        end
        out.(TAG{a}) = struct('t',t,'E',E,'sdE',std(AH,0,2),'sP',mean(P44,2),'hd',hd,'L41',C.K_a_y1_out,'L42',C.K_a_y2_out,'C',C);
        clear R;
    end
    S = out; save(fullfile(od,fname), '-struct', 'S', '-v7.3');
    fprintf('saved %s\n', fname);
end
