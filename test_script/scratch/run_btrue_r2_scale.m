% FORK OF test_script/scratch/run_jac_exact_arms.m (2026-09-06) | PURPOSE: discriminator for the alternative reading of kappa = 0.5,
%   written after jac_exact_step (the exact-step Jacobian, verified) turned out INERT (paired diff <= 0.0002 on every arm):
%   [hypothesis] kappa = 0.5 was compensating for R2 being ~3x too large (NIS2 = e_y2^2/S2 = 0.27-0.34 in every segment, both
%     trajectories; the a_cov / IF backlog): y2's information is under-counted, P44 decays too slowly along the descent, the gains
%     stay large and the spread is realised through them.
%   [opponents] (a) the spread is the honest amplified prior of the free part z (ODE sensitivity a'_wall/a'_entry ~ 25x) limited by
%     y2 -- then R2/3 lowers P but the realised spread follows only partly (honesty rises above 1.2);
%     (b) y2 is a downstream echo of the same error -- then R2/3 buys bias (hold level moves by > 1 SEM).
%   Same b_true arm (both trajectories) and the locked-8/9 arm (canon, the arm kappa = 0.5 collapsed), 10 seeds, four blocks,
%   kappa = 1; R2_int scaled by ctrl_const.K_var x s, s in {1, 1/3} (K_var = 2 a_cov/(2 - a_cov), a_cov = 0.05; the delay term
%   d Q44 is not scaled).
%   PRE-REGISTERED: s = 1/3 vs 1 --
%     P1 btrue worst-instant sigma_seed falls toward the kappa = 0.5 level (canon 0.0054 / Meng 0.0158), honesty (fast, hold)
%        stays in 0.8-1.2, hold level within 1 SEM;
%     P2 lockb (canon) hold level within 1 SEM of s = 1 (+0.0006) and no floor hits -- the OPPOSITE of kappa = 0.5;
%     P3 (measured separately with probe_btrue_nis2 if P1/P2 hold) NIS2 -> 0.8-1.0.
%   Output btrue_r2_scale_<traj>.mat | EXPIRES: with the R2 / IF backlog | 產線改動不會自動跟上
function out = run_btrue_r2_scale(traj, seeds)
    if nargin < 1 || isempty(traj); traj = 'canon'; end
    if nargin < 2 || isempty(seeds); seeds = 1:10; end
    traj = lower(traj);
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'apd_acov_meng');  pc = physical_constants();
    switch traj
        case 'meng'; OV = struct('trajectory_type','osc','h_init',15,'h_bottom',2.5,'amplitude',0,'frequency',1,'n_cycles',1,'t_hold',0.5,'t_descend_override',10,'T_sim',12.5,'h_min',2.475); cfg0 = OV;
                     WIN = [7 10];  TW = 9.24;  ARMS = {'btrue', struct('arm','best','b_true',true,'b_true_at','true')};
        case 'canon'; OV = struct(); cfg0 = canonical_scenario(0.05, 1.1, 'deep');  WIN = [1.0 1.5];  TW = 1.41;
                     ARMS = {'btrue', struct('arm','best','b_true',true,'b_true_at','true'); 'lockb', struct('arm','bmid')};
    end
    A_COV = 0.05;  K_VAR = 2 * A_COV / (2 - A_COV);  SC = [1, 1/3];  SN = {'s1', 's13'};
    w0bar = cfg0.h_init / pc.R; [~, cp] = calc_correction_functions(w0bar); at = 1/cp; ws0 = 1 + w0bar - 1/((8/9)*(1 - at));
    t3 = cfg0.t_hold + cfg0.t_descend_override + cfg0.n_cycles/cfg0.frequency;  nS = numel(seeds);
    ON4 = struct('law_exact_step',true,'pred_mean2',true,'nw_mcorr',true,'pred_mean2_e4',true,'fe44_Aa_scale',1,'ws0_perp',ws0);
    out = struct('traj', traj, 'seeds', seeds, 't_hold', t3, 'win', WIN, 'scales', SC);
    for ia = 1:size(ARMS, 1)
        for js = 1:2
            cc = ON4;  cc.K_var = K_VAR * SC(js);
            o = struct('ctrl_const_override',cc,'config_override',OV,'scenario','deep','verbose',false,'seeds',seeds,'log_P_full',false);
            fo = fieldnames(ARMS{ia,2}); for i = 1:numel(fo); o.(fo{i}) = ARMS{ia,2}.(fo{i}); end
            clear run_formC_b motion_control_law_formC_b;
            evalc('R = run_formC_b(o);');
            kv = R.runs{1}.ctrl_const.K_var;
            t = R.runs{1}.tout(:);  N = numel(t);  E = zeros(N,nS); AH = E; SP = E; L41 = E; L31 = E; AP = E; HB = E;
            for q = 1:nS
                r = R.runs{q}; ad = r.a_hat_out(1,3)/r.a_bar_hat_out(1,3);
                AH(:,q) = r.a_bar_hat_out(:,3); E(:,q) = AH(:,q) - r.a_true_out(:,3)/ad; HB(:,q) = r.h_bar_true_out(:,1);
                SP(:,q) = r.P_a_out(:,3)/ad;  L41(:,q) = r.K_a_y1_out(:,3);  L31(:,q) = r.K_dx_y1_out(:,3);  AP(:,q) = r.a_prime_out(:,3)/ad;
            end
            clear R;
            Ev = L41 + AP .* L31;  m = t >= WIN(1) & t <= WIN(2);  mh = t > t3;  iw = find(t >= TW, 1);
            key = [ARMS{ia,1} '_' SN{js}];
            out.(key) = struct('t', t, 'E', E, 'AH', AH, 'HB', HB, 'sP', SP, 'E_l', Ev, 'K_var', kv);
            fprintf('[%s %-5s R2x%.2f (K_var %.4f)] worst t=%.2f: sd %.4f mean %+.4f | fast: sd %.4f sqrtP %.4f honesty %.2f E_l %+.3f | hold: mean %+.5f SEM %.5f sd %.5f sqrtP %.5f honesty %.2f | min a_hat %.4f (floor hits %d seeds)\n', ...
                traj, ARMS{ia,1}, SC(js), kv, t(iw), std(E(iw,:)), mean(E(iw,:)), mean(std(E(m,:),0,2)), mean(SP(m,:),'all'), mean(std(E(m,:),0,2))/mean(SP(m,:),'all'), mean(Ev(m,:),'all'), ...
                mean(E(mh,:),'all'), std(mean(E(mh,:),1))/sqrt(nS), mean(std(E(mh,:),0,2)), mean(SP(mh,:),'all'), mean(std(E(mh,:),0,2))/mean(SP(mh,:),'all'), min(AH(:)), nnz(min(AH,[],1) < 0.0312));
        end
        dE = out.([ARMS{ia,1} '_s13']).E - out.([ARMS{ia,1} '_s1']).E;
        fprintf('[%s %-5s] PAIRED R2/3 - R2: worst instant %+.4f (SEM %.4f) | hold level %+.5f (SEM %.5f) | hold sd ratio %.2f\n', traj, ARMS{ia,1}, ...
            mean(dE(iw,:)), std(dE(iw,:))/sqrt(nS), mean(dE(mh,:),'all'), std(mean(dE(mh,:),1))/sqrt(nS), mean(std(out.([ARMS{ia,1} '_s13']).E(mh,:),0,2))/mean(std(out.([ARMS{ia,1} '_s1']).E(mh,:),0,2)));
    end
    save(fullfile(od, sprintf('btrue_r2_scale_%s.mat', traj)), '-struct', 'out', '-v7.3');
    fprintf('[%s] saved btrue_r2_scale_%s.mat\n', traj, traj);
end
