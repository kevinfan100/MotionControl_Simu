% FORK OF test_script/scratch/run_btrue_r2_scale.m (2026-09-06) | PURPOSE: third discriminator of the day, after the exact-step
%   Jacobian (inert) and R2/3 (no effect on the fast-window P44, hold worse) were both refuted:
%   [hypothesis] the fast-descent P44 of the b_true arm is the PRIOR P44[0] amplified by the law's own sensitivity
%     (D = a'(a_pred)/a'(a) per step, sum ~ a'_wall/a'_entry ~ 25x in a_bar units = the (1-a)^2 growth of a fixed error in
%     1/(1-a)); the arm is SEEDED AT TRUTH, so the prior it carries (P44[0] = a'[0]^2 Pf_w0_std^2 + Pf_a_floor^2 = 0.0031^2 on
%     Meng, 0.00026^2 on canon) is a phantom in this simulation: the filter amplifies an initial error it does not have, its P
%     runs 1.3-1.5x above the realised spread (honesty 0.66-0.81) and the realised spread follows P through the gains.
%     kappa = 0.5 halves the amplification of the phantom; the 09-06 swap (forcing Meng's P44[0] to canon's) moved the whole
%     descent curve. A real initial error would make kappa = 1 honest and kappa = 0.5 over-confident.
%   [opponents] (a) the amplification is realised regardless of the prior (then P0small changes nothing);
%     (b) the spread is gain-injected measurement noise unrelated to P44[0] (then P0small changes nothing either, and the offset
%     arm at kappa = 1 stays honesty < 0.8).
%   Arms (b_true, four blocks, 10 seeds, standard length):
%     p0small   kappa 1, prior P44[0] matched to the seed-at-truth start: Pf_w0_std = 0, Pf_a_floor = 3e-4   (canon + Meng)
%     off_k1    kappa 1,   seed gain offset +delta, delta = 0.0031 = sqrt P44[0] of the standard Meng prior   (Meng)
%     off_k05   kappa 0.5, same offset                                                                          (Meng)
%   (offset applied through ws0_perp: ws0 = 1 + w0 - 1/((8/9)(1 - (a_t + delta))), the seed's gain is a_t + delta)
%   PRE-REGISTERED: P1 p0small worst-instant sd <= kappa = 0.5 level (canon 0.0054 / Meng 0.0158), fast honesty 0.8-1.2;
%     P2 off_k1 fast honesty in 0.8-1.2 and off_k05 fast honesty > 1.4 (P too small for a real initial error);
%     P3 the offset's seed-mean error at the worst instant is amplified relative to +0.0031 (|mean| > 0.006) unless y2 has
%        removed it -- reported, not asserted.
%   Output btrue_prior_vs_offset_<traj>.mat | EXPIRES: with the b_true rung | 產線改動不會自動跟上
function out = run_btrue_prior_vs_offset(traj, seeds, only)
    if nargin < 1 || isempty(traj); traj = 'canon'; end
    if nargin < 2 || isempty(seeds); seeds = 1:10; end
    traj = lower(traj);
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'apd_acov_meng');  pc = physical_constants();
    DELTA = 0.0031;
    switch traj
        case 'meng'; OV = struct('trajectory_type','osc','h_init',15,'h_bottom',2.5,'amplitude',0,'frequency',1,'n_cycles',1,'t_hold',0.5,'t_descend_override',10,'T_sim',12.5,'h_min',2.475); cfg0 = OV;
                     WIN = [7 10];  TW = 9.24;  ARMS = {'p0small', 1, 0, struct('Pf_w0_std',0,'Pf_a_floor',3e-4); 'off_k1', 1, DELTA, struct(); 'off_k05', 0.5, DELTA, struct()};
        case 'canon'; OV = struct(); cfg0 = canonical_scenario(0.05, 1.1, 'deep');  WIN = [1.0 1.5];  TW = 1.41;
                     ARMS = {'p0small', 1, 0, struct('Pf_w0_std',0,'Pf_a_floor',3e-4); 'p0tiny', 1, 0, struct('Pf_w0_std',0,'Pf_a_floor',1e-5)};
                     % p0small is a NON-TEST on canon (its standard prior is already 0.00026); p0tiny (1e-5) is the canon test,
                     % added after the Meng p0small result (worst-instant sd 0.0323 -> 0.0051 at kappa = 1).
    end
    if nargin >= 3 && ~isempty(only); ARMS = ARMS(strcmp(ARMS(:,1), only), :); end
    w0bar = cfg0.h_init / pc.R; [~, cp] = calc_correction_functions(w0bar); at = 1/cp;
    t3 = cfg0.t_hold + cfg0.t_descend_override + cfg0.n_cycles/cfg0.frequency;  nS = numel(seeds);
    out = struct('traj', traj, 'seeds', seeds, 't_hold', t3, 'win', WIN, 'delta', DELTA);
    for ia = 1:size(ARMS, 1)
        ws0 = 1 + w0bar - 1/((8/9)*(1 - (at + ARMS{ia,3})));
        cc = struct('law_exact_step',true,'pred_mean2',true,'nw_mcorr',true,'pred_mean2_e4',true,'fe44_Aa_scale',ARMS{ia,2},'ws0_perp',ws0);
        fx = fieldnames(ARMS{ia,4}); for i = 1:numel(fx); cc.(fx{i}) = ARMS{ia,4}.(fx{i}); end
        o = struct('arm','best','b_true',true,'b_true_at','true','ctrl_const_override',cc,'config_override',OV,'scenario','deep','verbose',false,'seeds',seeds,'log_P_full',false);
        clear run_formC_b motion_control_law_formC_b;
        evalc('R = run_formC_b(o);');
        t = R.runs{1}.tout(:);  N = numel(t);  E = zeros(N,nS); AH = E; SP = E; L41 = E; L31 = E; AP = E; HB = E;
        for q = 1:nS
            r = R.runs{q}; ad = r.a_hat_out(1,3)/r.a_bar_hat_out(1,3);
            AH(:,q) = r.a_bar_hat_out(:,3); E(:,q) = AH(:,q) - r.a_true_out(:,3)/ad; HB(:,q) = r.h_bar_true_out(:,1);
            SP(:,q) = r.P_a_out(:,3)/ad;  L41(:,q) = r.K_a_y1_out(:,3);  L31(:,q) = r.K_dx_y1_out(:,3);  AP(:,q) = r.a_prime_out(:,3)/ad;
        end
        clear R;
        Ev = L41 + AP .* L31;  m = t >= WIN(1) & t <= WIN(2);  mh = t > t3;  iw = find(t >= TW, 1);
        out.(ARMS{ia,1}) = struct('t', t, 'E', E, 'AH', AH, 'HB', HB, 'sP', SP, 'E_l', Ev, 'kappa', ARMS{ia,2}, 'delta', ARMS{ia,3});
        fprintf('[%s %-8s kappa %.1f delta %+.4f] E(1) %+.5f sqrtP44(1) %.5f | worst t=%.2f: sd %.4f mean %+.4f | fast: sd %.4f sqrtP %.4f honesty %.2f E_l %+.3f | hold: mean %+.5f SEM %.5f sd %.5f sqrtP %.5f honesty %.2f | min a_hat %.4f\n', ...
            traj, ARMS{ia,1}, ARMS{ia,2}, ARMS{ia,3}, mean(E(1,:)), mean(SP(1,:)), t(iw), std(E(iw,:)), mean(E(iw,:)), mean(std(E(m,:),0,2)), mean(SP(m,:),'all'), mean(std(E(m,:),0,2))/mean(SP(m,:),'all'), mean(Ev(m,:),'all'), ...
            mean(E(mh,:),'all'), std(mean(E(mh,:),1))/sqrt(nS), mean(std(E(mh,:),0,2)), mean(SP(mh,:),'all'), mean(std(E(mh,:),0,2))/mean(SP(mh,:),'all'), min(AH(:)));
    end
    tag = ''; if nargin >= 3 && ~isempty(only); tag = ['_' only]; end
    save(fullfile(od, sprintf('btrue_prior_vs_offset_%s%s.mat', traj, tag)), '-struct', 'out', '-v7.3');
    fprintf('[%s] saved btrue_prior_vs_offset_%s.mat\n', traj, traj);
end
