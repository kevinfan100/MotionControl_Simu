% FORK OF test_script/scratch/run_btrue_e4.m (2026-09-04) | PURPOSE: the production rung of the ladder. Same 10 seeds,
%   both trajectories, hold extended by 4 s. Arms (opts.arm of run_formC_b; b_mid = 8/9 since 2026-08-18):
%     lockb   arm 'bmid'  : b LOCKED at 8/9, four blocks on (exact + pred_mean2 + nw_mcorr + pred_mean2_e4)
%                           -> A (b_true curve, btrue_e4_<traj>.mat e4 arm) minus lockb = the constant-b MODEL error alone
%     prod    arm 'best'  : b_hat ESTIMATED from 8/9 (production), four blocks on (+ the e_b line of pred_mean2_e4)
%                           -> lockb minus prod = what estimating b buys / costs
%     hist    arm 'best'  : production as it was (Euler, no mean term, no correlated-noise predict) -- history only
%     lockw   arm 'bmid' + b_init = b_true(w_hold) locked (ORACLE discriminator, 09-04): lockb - curve showed a paired
%             hold-slope difference of -0.176 / -0.184 e-6/step (canon / Meng). PRE-REGISTERED: if it is the slope mismatch
%             at the hold height (b_hat 8/9 vs b_true 0.928 there), lockw - curve hold slope -> 0 +- 0.03 while its
%             end-of-descent paired offset flips sign (b_hat > b_true over the mid range). Opponent: unchanged => path history.
%   PRE-REGISTERED (0903_aptrue_4state_from_true.tex S11, written before any run):
%     (1) lockb: est - true at the hold start NEGATIVE on both trajectories; open-loop first-order model error
%         int_{6.67}^{1.10} (1-a)^2 (b_true - 8/9) dw = +0.0081 (true - est) [CORRECTED after the run by the independent
%         verification: the forcing must be propagated through the law's own amplification, mu = (1-a_wall)^2 int (b_true - 8/9) dw
%         = +0.057, open loop; measured closed loop -0.0030 / -0.0005 at the end of the descent], the response a fraction of it
%         (the e4-line input's response was 10-40% of its running sum); hold slope and hold sd as in the b_true arms
%         (no first-order model drift in a stationary hold: E[f dw] and 1/2 f' E[dw^2] cancel);
%     (2) prod - lockb paired difference small: sqrt(P55) shrinks by a few percent at most (CRLB/prior 0.86), so b_hat
%         stays within its prior and the two arms agree within the hold sd; if |paired diff| > hold sd the slot-5
%         dynamics are doing something the derivation does not have;
%     (3) hist: est - true POSITIVE at the wall (the Euler ratchet and the uncompensated second-order means, seed 7 of
%         the b_true Euler arm gave +0.019 / +0.006), i.e. the historical near-wall over-estimate has the opposite
%         sign to the model error it was hiding.
%   Opponent for (1): the sign comes out positive => the path-integral reading of the model error is wrong (e.g. the
%   y2 leg re-anchors the level to the readout during the descent faster than the input accumulates).
%   Output: prod_ladder_<traj>.mat (t, E, AH, AT, HB, hd, B (b_hat), sP5, slope per arm) | EXPIRES: with the production rung
%   | 產線改動不會自動跟上
function out = run_prod_ladder(traj, seeds, arms)
    if nargin < 1 || isempty(traj); traj = 'canon'; end
    if nargin < 2 || isempty(seeds); seeds = 1:10; end
    if nargin < 3 || isempty(arms); arms = {'lockb','prod','hist'}; end
    traj = lower(traj);
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    pc = physical_constants();  EXTRA = 4.0;
    switch traj
        case 'meng'
            OV = struct('trajectory_type','osc','h_init',15,'h_bottom',2.5,'amplitude',0, ...
                        'frequency',1,'n_cycles',1,'t_hold',0.5,'t_descend_override',10,'T_sim',12.5 + EXTRA,'h_min',2.475);
            cfg0 = OV;  SHORT = 1.0;
        case 'canon'
            cfg0 = canonical_scenario(0.05, 1.1, 'deep');  SHORT = 1.3;
            OV = struct('T_sim', cfg0.T_sim + EXTRA);  cfg0.T_sim = cfg0.T_sim + EXTRA;
    end
    w0bar = cfg0.h_init / pc.R;  [~, cp] = calc_correction_functions(w0bar);  at = 1/cp;
    ws0 = 1 + w0bar - 1/((8/9)*(1 - at));
    t1 = cfg0.t_hold;  t2 = t1 + cfg0.t_descend_override;  t3 = t2 + cfg0.n_cycles/cfg0.frequency;
    BLOCKS = struct('law_exact_step',true,'pred_mean2',true,'nw_mcorr',true,'pred_mean2_e4',true);
    [~, cpw] = calc_correction_functions(cfg0.h_bottom / pc.R);  aw = 1/cpw;   % b_true at the hold height (oracle, discriminator arm only)
    dw = 1e-4; [~, cpp] = calc_correction_functions(cfg0.h_bottom / pc.R + dw); [~, cpm] = calc_correction_functions(cfg0.h_bottom / pc.R - dw);
    b_wall = ((1/cpp - 1/cpm) / (2*dw)) / (1 - aw)^2;
    LOCKW = BLOCKS;  LOCKW.lock_b = true;  LOCKW.b_init = b_wall;
    ARMDEF = struct('lockb', struct('arm','bmid','cc',BLOCKS), ...
                    'lockw', struct('arm','bmid','cc',LOCKW), ...   % DISCRIMINATOR (09-04): b locked at b_true(w_hold) = the wall value; if the hold-slope
                    ...                                              % difference lockb - curve (-0.18 e-6/step) is the slope mismatch AT THE HOLD HEIGHT it -> 0 here
                    'prod',  struct('arm','best','cc',BLOCKS), ...
                    'hist',  struct('arm','best','cc',struct('law_exact_step',false,'pred_mean2',false,'nw_mcorr',false,'pred_mean2_e4',false)));   % explicit OFF: the pre-09-04 recipe (defaults are ON since 09-04 evening)
    fprintf('[%s prod ladder] b_true at the hold height w = %.3f: %.5f (lockw arm)\n', traj, cfg0.h_bottom / pc.R, b_wall);
    fname = fullfile(od, sprintf('prod_ladder_%s.mat', traj));
    if exist(fname, 'file'); out = load(fname); else; out = struct(); end
    out.traj = traj;  out.seeds = seeds;  out.phases = [t1 t2 t3 cfg0.T_sim];  out.t_hold = t3;  out.ws0 = ws0;
    nS = numel(seeds);
    fprintf('[%s prod ladder] ws0 %.5f | hold from %.2f to %.2f s | seeds %s | arms %s\n', traj, ws0, t3, cfg0.T_sim, mat2str(seeds), strjoin(arms, ','));
    for ia = 1:numel(arms)
        A = ARMDEF.(arms{ia});  cc = A.cc;  cc.ws0_perp = ws0;
        o = struct('arm', A.arm, 'ctrl_const_override', cc, 'config_override', OV, 'scenario', 'deep', ...
                   'verbose', false, 'seeds', seeds, 'log_P_full', false);
        clear run_formC_b motion_control_law_formC_b;
        txt = evalc('R = run_formC_b(o);');
        li = regexp(txt, '[^\n]*ARM USES b_init[^\n]*', 'match');  if ~isempty(li); fprintf('   %s\n', strtrim(li{1})); end
        li = regexp(txt, '[^\n]*lock_b[^\n]*', 'match');  if ~isempty(li); fprintf('   %s\n', strtrim(li{1})); end
        t = R.runs{1}.tout(:);  N = numel(t);  E = zeros(N, nS); AH = E; AT = E; HB = E;  B = E;  P5 = E;
        for q = 1:nS
            rr = R.runs{q}; ad = rr.a_hat_out(1,3)/rr.a_bar_hat_out(1,3);
            AH(:,q) = rr.a_bar_hat_out(:,3); AT(:,q) = rr.a_true_out(:,3)/ad; E(:,q) = AH(:,q) - AT(:,q);
            HB(:,q) = rr.h_bar_true_out(:,1);  B(:,q) = rr.b_hat_out(:,3);  P5(:,q) = rr.P_b_out(:,3);   % P_b_out is already sqrt(P55) (driver line 1246); the 09-04 first run double-rooted it, mats fixed up afterwards
        end
        hd = R.runs{1}.p_d_out(:,3)/R.runs{1}.R;  clear R;
        m3 = t > cfg0.T_sim - 3;  k = (1:sum(m3)).';  sl = zeros(1,nS); for q = 1:nS; p = polyfit(k, E(m3,q), 1); sl(q) = p(1); end
        out.(arms{ia}) = struct('t', t, 'E', E, 'AH', AH, 'AT', AT, 'HB', HB, 'hd', hd, 'B', B, 'sP5', P5, 'slope', sl);
        mh = t > t3;  ms = t > t3 & t <= t3 + SHORT;  pm = mean(E(ms,:),1);  i2 = find(t > t2, 1);  i3 = find(t > t3, 1);
        fprintf('[%s %-5s] health: min w %.4f | min a_hat %.5f | NaN %d | b_hat [%.4f, %.4f], b_hat(end) %.4f +- %.4f (seeds), sqrt(P55) %.4f -> %.4f\n', ...
            traj, arms{ia}, min(HB(:)), min(AH(:)), sum(~isfinite(E(:))), min(B(:)), max(B(:)), mean(B(end,:)), std(B(end,:)), P5(2,1), mean(P5(end,:)));
        fprintf('[%s %-5s] est-true: end of descent %+.5f (SEM %.5f) | hold start %+.5f | short-hold (%.1f s) %+.5f (SEM %.5f) | whole-hold %+.5f | hold sd %.5f | last-3-s slope %+.3f e-6/step (SEM %.3f)\n', ...
            traj, arms{ia}, mean(E(i2,:)), std(E(i2,:))/sqrt(nS), mean(E(i3,:)), SHORT, mean(pm), std(pm)/sqrt(nS), mean(E(mh,:),'all'), mean(std(E(mh,:),0,2)), 1e6*mean(sl), 1e6*std(sl)/sqrt(nS));
    end
    % paired comparisons against the b_true curve arm (A) and between arms
    fA = fullfile(od, sprintf('btrue_e4_%s.mat', traj));
    if exist(fA, 'file') && isfield(out, 'lockb')
        SA = load(fA);  dE = out.lockb.E - SA.e4.E;  t = out.lockb.t;  i2 = find(t > out.phases(2), 1);  i3 = find(t > out.phases(3), 1);
        fprintf('[%s] PAIRED lockb - b_true(curve): end of descent %+.5f (SEM %.5f) | hold start %+.5f (SEM %.5f) | whole hold %+.5f | slope diff %+.3f e-6/step (SEM %.3f)\n', ...
            traj, mean(dE(i2,:)), std(dE(i2,:))/sqrt(nS), mean(dE(i3,:)), std(dE(i3,:))/sqrt(nS), mean(dE(t > out.phases(3),:),'all'), ...
            1e6*mean(out.lockb.slope - SA.e4.slope), 1e6*std(out.lockb.slope - SA.e4.slope)/sqrt(nS));
    end
    if exist(fA, 'file') && isfield(out, 'lockw')
        SA = load(fA);  dE = out.lockw.E - SA.e4.E;  t = out.lockw.t;  i2 = find(t > out.phases(2), 1);  i3 = find(t > out.phases(3), 1);
        fprintf('[%s] PAIRED lockw - b_true(curve): end of descent %+.5f (SEM %.5f) | hold start %+.5f (SEM %.5f) | whole hold %+.5f | slope diff %+.3f e-6/step (SEM %.3f)\n', ...
            traj, mean(dE(i2,:)), std(dE(i2,:))/sqrt(nS), mean(dE(i3,:)), std(dE(i3,:))/sqrt(nS), mean(dE(t > out.phases(3),:),'all'), ...
            1e6*mean(out.lockw.slope - SA.e4.slope), 1e6*std(out.lockw.slope - SA.e4.slope)/sqrt(nS));
    end
    if isfield(out, 'lockb') && isfield(out, 'prod')
        dE = out.prod.E - out.lockb.E;  t = out.lockb.t;  i3 = find(t > out.phases(3), 1);
        fprintf('[%s] PAIRED prod - lockb: hold start %+.5f (SEM %.5f) | whole hold %+.5f | slope diff %+.3f e-6/step (SEM %.3f) | hold sd %.5f vs %.5f\n', ...
            traj, mean(dE(i3,:)), std(dE(i3,:))/sqrt(nS), mean(dE(t > out.phases(3),:),'all'), 1e6*mean(out.prod.slope - out.lockb.slope), ...
            1e6*std(out.prod.slope - out.lockb.slope)/sqrt(nS), mean(std(out.prod.E(t > out.phases(3),:),0,2)), mean(std(out.lockb.E(t > out.phases(3),:),0,2)));
    end
    save(fname, '-struct', 'out', '-v7.3');
    fprintf('[%s prod ladder] saved prod_ladder_%s.mat\n', traj, traj);
end
