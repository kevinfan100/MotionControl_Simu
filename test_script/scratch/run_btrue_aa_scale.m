% FORK OF test_script/scratch/run_btrue_aa_off.m (2026-09-04) | PURPOSE: sweep kappa = ctrl_const.fe44_Aa_scale (the weight of
%   the law's self-sensitivity A_a*M in F_e(4,4) for the P propagation) on the b_true arm, four blocks, standard length,
%   seed-at-truth. kappa = 1 = current EKF (bit-identical to ladder_endpoints btest), kappa = 0 = fe44_Aa_off.
%   CALIBRATION seeds 1:10; VALIDATION of the chosen kappa* on seeds 11:20 (stacked-fix-audit C.8).
%   PRE-REGISTERED: kappa* (geometric-mean honesty over {fast, hold} x {canon, Meng} closest to 1) in [0.3, 0.7];
%   at kappa* the fast-window sd < 1/2 of kappa = 1 and the hold honesty in [0.8, 1.3]; hold level within 1 SEM of kappa = 1.
%   MECHANISM MEASURED BEFORE THIS RUN (canon 1.0-1.5 s, 30 seeds): the near-wall spread is generated in the fast segment
%   (corr with the error at 1.0 s = 0.09), the predict injects the law-mapped position noise (sd 0.025) and the y1 leg
%   cancels only 1/3 of it (sd 0.009, corr -0.40) because l41 flipped sign (+0.036; law-consistent -a' l31 = -0.157);
%   in the a'_true arm the pair cancels to sd 0.0025 (corr -0.99). l41 flips because P44 grows x16 along the descent
%   through A_a*M and P41 becomes -F_dw P44 dominated.
%   PRE-REGISTERED (est - true): if the loop is self-fulfilling, with the flag
%     (1) the cross-seed sd at the worst instant (canon 1.41 s / Meng 9.24 s) drops from 0.023 / 0.034 toward the a'_true
%         arm's 0.0025 / 0.0022 (any factor > 3 counts);
%     (2) E = l41 + a' l31 over the fast window returns to ~0 (|E| < 0.02);
%     (3) honesty sigma_seed/sqrt(P44) in the fast window moves toward 1 from 0.66 / 0.81 -- if it goes ABOVE ~1.5 the
%         filter is over-confident (the 08-13 reading) and the flag is a lie, not a fix;
%     (4) hold level, hold sd and the hold slope unchanged within 1 SEM (the flag acts only where M is large).
%   Opponent: sd unchanged while sqrt(P44) shrinks => the spread is real information loss, the flag only hides it.
%   Output: btrue_aa_off_<traj>.mat | EXPIRES: with the b_true rung | 產線改動不會自動跟上
function out = run_btrue_aa_scale(traj, seeds, kappas)
    if nargin < 1 || isempty(traj); traj = 'canon'; end
    if nargin < 2 || isempty(seeds); seeds = 1:10; end
    if nargin < 3 || isempty(kappas); kappas = [0 0.25 0.5 0.75 1]; end
    traj = lower(traj);
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'apd_acov_meng');  pc = physical_constants();
    switch traj
        case 'meng'; OV = struct('trajectory_type','osc','h_init',15,'h_bottom',2.5,'amplitude',0,'frequency',1,'n_cycles',1,'t_hold',0.5,'t_descend_override',10,'T_sim',12.5,'h_min',2.475); cfg0 = OV;
                     WIN = [7 10];  TW = 9.24;
        case 'canon'; OV = struct(); cfg0 = canonical_scenario(0.05, 1.1, 'deep');  WIN = [1.0 1.5];  TW = 1.41;
    end
    w0bar = cfg0.h_init / pc.R; [~, cp] = calc_correction_functions(w0bar); at = 1/cp; ws0 = 1 + w0bar - 1/((8/9)*(1 - at));
    t3 = cfg0.t_hold + cfg0.t_descend_override + cfg0.n_cycles/cfg0.frequency;
    ON4 = struct('law_exact_step',true,'pred_mean2',true,'nw_mcorr',true,'pred_mean2_e4',true,'ws0_perp',ws0);
    nK = numel(kappas);  ARM = arrayfun(@(k) sprintf('k%03d', round(100*k)), kappas, 'UniformOutput', false);  nS = numel(seeds);
    out = struct('traj', traj, 'seeds', seeds, 't_hold', t3, 'win', WIN, 'kappas', kappas);  TAB = zeros(nK, 12);
    for a = 1:nK
        cc = ON4;  cc.fe44_Aa_scale = kappas(a);
        o = struct('arm','best','b_true',true,'b_true_at','true','ctrl_const_override',cc,'config_override',OV,'scenario','deep','verbose',false,'seeds',seeds,'log_P_full',false);
        clear run_formC_b motion_control_law_formC_b;
        evalc('R = run_formC_b(o);');
        t = R.runs{1}.tout(:);  N = numel(t);  E = zeros(N,nS); AH = E; AT = E; HB = E; SP = E; L41 = E; L31 = E; AP = E;
        for q = 1:nS
            r = R.runs{q}; ad = r.a_hat_out(1,3)/r.a_bar_hat_out(1,3);
            AH(:,q) = r.a_bar_hat_out(:,3); AT(:,q) = r.a_true_out(:,3)/ad; E(:,q) = AH(:,q) - AT(:,q); HB(:,q) = r.h_bar_true_out(:,1);
            SP(:,q) = r.P_a_out(:,3)/ad;  L41(:,q) = r.K_a_y1_out(:,3);  L31(:,q) = r.K_dx_y1_out(:,3);  AP(:,q) = r.a_prime_out(:,3)/ad;
        end
        hd = R.runs{1}.p_d_out(:,3)/R.runs{1}.R;  clear R;
        Ev = L41 + AP .* L31;
        m = t >= WIN(1) & t <= WIN(2);  mh = t > t3;  iw = find(t >= TW, 1);  m3 = t > t(end) - 1.0;
        sl = zeros(1,nS); k = (1:sum(m3)).'; for q = 1:nS; p = polyfit(k, E(m3,q), 1); sl(q) = p(1); end
        out.(ARM{a}) = struct('t', t, 'E', E, 'AH', AH, 'AT', AT, 'HB', HB, 'hd', hd, 'sP', SP, 'E_l', Ev, 'slope', sl);
        TAB(a,:) = [kappas(a), std(E(iw,:)), mean(E(iw,:)), mean(std(E(m,:),0,2)), mean(std(E(m,:),0,2))/mean(SP(m,:),'all'), mean(Ev(m,:),'all'), ...
                    mean(E(mh,:),'all'), std(mean(E(mh,:),1))/sqrt(nS), mean(std(E(mh,:),0,2)), mean(std(E(mh,:),0,2))/mean(SP(mh,:),'all'), 1e6*mean(sl), min(AH(:))];
        fprintf('[%s %-5s] worst instant t=%.2f: sd %.4f (mean %+.4f, SEM %.4f) | fast window: sd %.4f, sqrtP %.4f, honesty %.2f, E=l41+a''l31 %+.3f (l41 %+.3f) | hold: mean %+.5f (SEM %.5f) sd %.5f sqrtP %.5f honesty %.2f | last-s slope %+.3f e-6/step (SEM %.3f) | min a_hat %.4f\n', ...
            traj, sprintf('k=%.2f', kappas(a)), t(iw), std(E(iw,:)), mean(E(iw,:)), std(E(iw,:))/sqrt(nS), mean(std(E(m,:),0,2)), mean(SP(m,:),'all'), mean(std(E(m,:),0,2))/mean(SP(m,:),'all'), ...
            mean(Ev(m,:),'all'), mean(L41(m,:),'all'), mean(E(mh,:),'all'), std(mean(E(mh,:),1))/sqrt(nS), mean(std(E(mh,:),0,2)), mean(SP(mh,:),'all'), mean(std(E(mh,:),0,2))/mean(SP(mh,:),'all'), ...
            1e6*mean(sl), 1e6*std(sl)/sqrt(nS), min(AH(:)));
    end
    f0 = fullfile(od, sprintf('ladder_endpoints_%s.mat', traj));
    ik1 = find(abs(kappas - 1) < 1e-9, 1);
    if exist(f0, 'file') && ~isempty(ik1); S = load(f0); fprintf('[%s] negative control kappa=1 vs ladder_endpoints btest: max |dE| %.2e\n', traj, max(abs(out.(ARM{ik1}).E - S.btest.E), [], 'all')); end
    out.table = TAB;  out.table_cols = {'kappa','sd_worst','mean_worst','sd_fast','honesty_fast','E_fast','hold_mean','hold_SEM','hold_sd','honesty_hold','slope_e6','min_ahat'};
    fprintf('[%s] kappa | sd_worst mean_worst | sd_fast hon_fast E_fast | hold_mean(SEM) hold_sd hon_hold | slope | min_ahat\n', traj);
    for a = 1:nK; fprintf('[%s] %.2f | %.4f %+.4f | %.4f %.2f %+.3f | %+.5f(%.5f) %.5f %.2f | %+.2f | %.4f\n', traj, TAB(a,1), TAB(a,2), TAB(a,3), TAB(a,4), TAB(a,5), TAB(a,6), TAB(a,7), TAB(a,8), TAB(a,9), TAB(a,10), TAB(a,11), TAB(a,12)); end
    tag = ''; if ~isequal(seeds, 1:10); tag = sprintf('_s%d-%d', seeds(1), seeds(end)); end
    save(fullfile(od, sprintf('btrue_aa_scale_%s%s.mat', traj, tag)), '-struct', 'out', '-v7.3');
    fprintf('[%s] saved btrue_aa_scale_%s%s.mat\n', traj, traj, tag);
end
