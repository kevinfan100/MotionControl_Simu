% FORK OF test_script/scratch/run_btrue_aa_off.m (2026-09-06) | PURPOSE: acceptance of ctrl_const.jac_exact_step (row-4 Jacobian /
%   Q / g_n of the EXACT law step, 0903 tex S12 closed form; replaces the calibrated kappa). Three arms x two trajectories,
%   same 10 seeds, standard length, four blocks ON, kappa = 1, seed-at-truth; flag OFF then ON:
%     btrue   b_true fed at the previous true height, slope at a_hat   (driver arm 'best', b_true=true, b_true_at='true')
%     lockb   b locked at 8/9                                          (driver arm 'bmid')
%     prod    b_hat estimated from the 8/9 seed = production            (driver arm 'best')
%   PRE-REGISTERED (written before the run; all four must hold or the derivation is not done -- no knob):
%     P1 btrue: worst-instant sigma_seed <= the kappa = 0.5 level (canon 0.0054 / Meng 0.0158) and fast-window
%        |l41 + a' l31| < 0.02 (kappa = 1: +0.19 / +0.04); honesty sigma_seed/sqrt(P44) in 0.8-1.2 (fast and hold);
%        hold level within 1 SEM of the OFF arm.
%     P2 lockb: NO collapse -- min a_hat above the floor 0.031 in every seed, hold level within 1 SEM of the OFF arm
%        (canon OFF ~ +0.0006; kappa = 0.5 gave -0.0197 +- 0.0007, every seed at the floor).
%     P3 prod: hold level within 1 SEM of the OFF arm; b_hat hold mean within 0.01 of the OFF arm; sqrt(P55) at the end
%        not collapsed (>= 0.015; kappa = 0.5 collapsed it to 0.004).
%     P4 OFF arms bit-identical to ladder_endpoints_<traj>.mat btest / bhat (negative control; flag default false).
%   Opponent: P1 passes but P2 or P3 fails => the term competes with the y2 model-error correction (the kappa story) and
%   the closed form is still incomplete; P1 fails => the row-4 factor is not the mechanism.
%   Output jac_exact_arms_<traj>.mat | EXPIRES: when jac_exact_step is adopted or refuted | 產線改動不會自動跟上
function out = run_jac_exact_arms(traj, seeds)
    if nargin < 1 || isempty(traj); traj = 'canon'; end
    if nargin < 2 || isempty(seeds); seeds = 1:10; end
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
    t3 = cfg0.t_hold + cfg0.t_descend_override + cfg0.n_cycles/cfg0.frequency;  nS = numel(seeds);
    ON4 = struct('law_exact_step',true,'pred_mean2',true,'nw_mcorr',true,'pred_mean2_e4',true,'fe44_Aa_scale',1,'ws0_perp',ws0);
    ARMS = {'btrue', struct('arm','best','b_true',true,'b_true_at','true'); 'lockb', struct('arm','bmid'); 'prod', struct('arm','best')};
    FL = {'off', 'on'};
    out = struct('traj', traj, 'seeds', seeds, 't_hold', t3, 'win', WIN);
    for ia = 1:size(ARMS, 1)
        for jf = 1:2
            cc = ON4;  cc.jac_exact_step = (jf == 2);
            o = struct('ctrl_const_override',cc,'config_override',OV,'scenario','deep','verbose',false,'seeds',seeds,'log_P_full',false);
            fo = fieldnames(ARMS{ia,2}); for i = 1:numel(fo); o.(fo{i}) = ARMS{ia,2}.(fo{i}); end
            clear run_formC_b motion_control_law_formC_b;
            evalc('R = run_formC_b(o);');
            t = R.runs{1}.tout(:);  N = numel(t);  E = zeros(N,nS); AH = E; SP = E; L41 = E; L31 = E; AP = E; B = E; SP5 = E; HB = E;
            for q = 1:nS
                r = R.runs{q}; ad = r.a_hat_out(1,3)/r.a_bar_hat_out(1,3);
                AH(:,q) = r.a_bar_hat_out(:,3); E(:,q) = AH(:,q) - r.a_true_out(:,3)/ad; HB(:,q) = r.h_bar_true_out(:,1);
                SP(:,q) = r.P_a_out(:,3)/ad;  L41(:,q) = r.K_a_y1_out(:,3);  L31(:,q) = r.K_dx_y1_out(:,3);  AP(:,q) = r.a_prime_out(:,3)/ad;
                B(:,q) = r.b_hat_out(:,3);  SP5(:,q) = r.P_b_out(:,3);
            end
            clear R;
            Ev = L41 + AP .* L31;  m = t >= WIN(1) & t <= WIN(2);  mh = t > t3;  iw = find(t >= TW, 1);  m3 = t > t(end) - 1.0;
            sl = zeros(1,nS); kk = (1:sum(m3)).'; for q = 1:nS; p = polyfit(kk, E(m3,q), 1); sl(q) = p(1); end
            key = [ARMS{ia,1} '_' FL{jf}];
            out.(key) = struct('t', t, 'E', E, 'AH', AH, 'HB', HB, 'sP', SP, 'E_l', Ev, 'B', B, 'sP5', SP5, 'slope', sl);
            fprintf('[%s %-5s %-3s] worst t=%.2f: sd %.4f mean %+.4f | fast: sd %.4f sqrtP %.4f honesty %.2f E_l %+.3f | hold: mean %+.5f SEM %.5f sd %.5f sqrtP %.5f honesty %.2f | slope %+.3f e-6/step | min a_hat %.4f (floor hits %d seeds) | b_hat hold %.4f sqrtP55 end %.4f\n', ...
                traj, ARMS{ia,1}, FL{jf}, t(iw), std(E(iw,:)), mean(E(iw,:)), mean(std(E(m,:),0,2)), mean(SP(m,:),'all'), mean(std(E(m,:),0,2))/mean(SP(m,:),'all'), mean(Ev(m,:),'all'), ...
                mean(E(mh,:),'all'), std(mean(E(mh,:),1))/sqrt(nS), mean(std(E(mh,:),0,2)), mean(SP(mh,:),'all'), mean(std(E(mh,:),0,2))/mean(SP(mh,:),'all'), ...
                1e6*mean(sl), min(AH(:)), nnz(min(AH,[],1) < 0.0312), mean(B(mh,:),'all'), mean(SP5(end,:)));
        end
        dE = out.([ARMS{ia,1} '_on']).E - out.([ARMS{ia,1} '_off']).E;
        fprintf('[%s %-5s] PAIRED on - off: worst instant %+.4f (SEM %.4f) | hold level %+.5f (SEM %.5f) | hold sd ratio %.2f\n', traj, ARMS{ia,1}, ...
            mean(dE(iw,:)), std(dE(iw,:))/sqrt(nS), mean(dE(mh,:),'all'), std(mean(dE(mh,:),1))/sqrt(nS), mean(std(out.([ARMS{ia,1} '_on']).E(mh,:),0,2))/mean(std(out.([ARMS{ia,1} '_off']).E(mh,:),0,2)));
    end
    f0 = fullfile(od, sprintf('ladder_endpoints_%s.mat', traj));
    if exist(f0, 'file'); S = load(f0); fprintf('[%s] NEGATIVE CONTROL off vs ladder_endpoints: btrue max|dE| %.2e | prod max|dE| %.2e\n', traj, max(abs(out.btrue_off.E - S.btest.E),[],'all'), max(abs(out.prod_off.E - S.bhat.E),[],'all')); end
    save(fullfile(od, sprintf('jac_exact_arms_%s.mat', traj)), '-struct', 'out', '-v7.3');
    fprintf('[%s] saved jac_exact_arms_%s.mat\n', traj, traj);
end
