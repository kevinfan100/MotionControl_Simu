% PURPOSE (2026-09-04): predict, from the filter's own P log, the mean drift the b_true arm still carries
%   after pred_mean2 (+ nw_mcorr), because its slope reads the ESTIMATED GAIN while pred_mean2's start-point
%   term was derived for a slope that reads the estimated HEIGHT.
%   Error dynamics (e = true - estimate) of the row-4 step in the b_true arm, a_bar' = b (1 - a_hat)^2:
%       a_bar'_true = b (1 - a_bar_w)^2 = a_bar' - 2 b (1 - a_hat) e4 + O(e^2)
%       e4[k+1] - e4[k] = a_bar' e_D - 2 b (1 - a_hat) e4 (dW_hat + e_D) + a_bar'' dW_hat e_D + 1/2 a_bar'' e_D^2 + O(3)
%   with e_D = u = (1 - lc) e3 + alpha (e8 + e9) + F_dw e4 + w_T   [nw_mcorr: - (1 - lc) e1 in place of the n_w share].
%   Mean the truth carries:            T = -2 b (1 - a_hat) Cov(e4, u)                         (gain-reading start point)
%   Mean pred_mean2 adds (code):       C = -a_bar''_law Cov(e3, u) = +2 b (1 - a_hat) a_bar' Cov(e3, u)   (height-reading start point)
%   (Jensen 1/2 a'' Var(u) is common to both writings; the curvature-difference term is 0 without app_known.)
%   Remaining drift per step (est - true) = C - T = 2 b (1 - a_hat) Cov(e4 + a_bar' e3, u) = 2 b (1 - a_hat) Cov(z, u),
%   z = e4 + a_bar' e3 the law-invariant combination. It vanishes iff P34 = -a_bar' P33 (gain error slaved to the
%   position error through the law), which is why pred_mean2 already took most of the b_true bias (09-02).
%   Reports, per segment, in e-6/step (est - true): whole = 2b(1-a)Cov(e4,u), code = 2b(1-a)a'Cov(e3,u), resid = whole - code,
%   each for the base u and the nw_mcorr u (same base P). The hold resid is the PRE-REGISTERED prediction of the
%   remaining last-3-s slope of run_btrue_nw_mcorr.m's nwmcorr arm (the base arm adds the a'_true-family M drift on top).
%   Runs the base arm (b_true@true + exact + pred_mean2, hold +4 s) with log_P_full on.
%   CAVEAT (09-04, found by the third-arm run): the driver does not log F_dw; the controller's F_dw is the
%   deterministic-mirror force with its (1-lc) two-step history, F_dw_det = f_det + (1-lc)(f_det[k-1] + f_det[k-2]).
%   This probe approximates it by the realised force history f + (1-lc)(f[k-1] + f[k-2]) (raw form); the first version
%   used the last force alone and underestimated the F_dw shares ~2.7x. The EXACT input the flag adds is the code's own
%   pred_mean2_out difference between the arms (btrue_e4_<traj>.mat, field pm2); use that for numbers, this probe for
%   the segment anatomy. Hold values are unaffected (F_dw ~ 0 there). | EXPIRES: with the b_true rung
function out = probe_btrue_e4_line(traj, seeds)
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
            cfg0 = OV;
        case 'canon'
            cfg0 = canonical_scenario(0.05, 1.1, 'deep');  OV = struct('T_sim', cfg0.T_sim + EXTRA);  cfg0.T_sim = cfg0.T_sim + EXTRA;
    end
    w0bar = cfg0.h_init / pc.R;  [~, cp] = calc_correction_functions(w0bar);  at = 1/cp;
    ws0 = 1 + w0bar - 1/((8/9)*(1 - at));
    t1 = cfg0.t_hold;  t2 = t1 + cfg0.t_descend_override;  t3 = t2 + cfg0.n_cycles/cfg0.frequency;
    cc = struct('ws0_perp',ws0,'law_exact_step',true,'pred_mean2',true,'nw_mcorr',false,'pred_mean2_e4',false,'fe44_Aa_scale',1);   % explicit since the 09-04 production defaults turned these ON
    o = struct('arm','best','b_true',true,'b_true_at','true','ctrl_const_override',cc, ...
               'config_override',OV,'scenario','deep','verbose',false,'seeds',seeds,'log_P_full',true);
    clear run_formC_b motion_control_law_formC_b;
    evalc('R = run_formC_b(o);');
    lc = R.cfg.lambda_c;  alpha = 1 - lc;  nS = numel(seeds);
    t = R.runs{1}.tout(:);  N = numel(t);
    % TERM(:,:,v): v = 1 whole/base, 2 code/base, 3 resid/base, 4 whole/mcorr, 5 code/mcorr, 6 resid/mcorr  (est - true per step)
    TERM = zeros(N, nS, 6);  E = zeros(N, nS);  ZC = zeros(N, nS, 2);   % ZC: Cov(z,e3) and Cov(z,u_base)
    for q = 1:nS
        rr = R.runs{q};  P = rr.P_full_out;  np = size(P, 2);
        ah = rr.a_bar_hat_out(:,3);  bu = rr.b_hat_out(:,3);  fb = rr.f_bar_out(:,3);  apv = rr.a_prime_out(:,3);
        ad = rr.a_hat_out(1,3)/rr.a_bar_hat_out(1,3);  E(:,q) = ah - rr.a_true_out(:,3)/ad;
        for k = 2:N
            Pc = squeeze(P(k-1, :, :, 3));
            Fdw = fb(k-1);  if k >= 3; Fdw = Fdw + alpha * fb(k-2); end;  if k >= 4; Fdw = Fdw + alpha * fb(k-3); end   % raw F_dw of step k-1
            a1 = 1 - ah(k-1);  b = bu(k);  ap = b * a1^2;  A2 = 2 * b * a1;   % slope the law used at step k (a' = b (1 - a_hat)^2)
            c4 = alpha * Pc(3,4) + Fdw * Pc(4,4);   c3 = alpha * Pc(3,3) + Fdw * Pc(3,4);          % Cov(e4,u), Cov(e3,u) base
            if np >= 9
                c4 = c4 + alpha * (Pc(4,8) + Pc(4,9));  c3 = c3 + alpha * (Pc(3,8) + Pc(3,9));
            end
            c4m = c4 - alpha * Pc(1,4);  c3m = c3 - alpha * Pc(1,3);                              % nw_mcorr u
            TERM(k,q,:) = A2 * [c4, ap*c3, c4 + ap*c3, c4m, ap*c3m, c4m + ap*c3m];
            ZC(k,q,:) = [Pc(3,4) + ap*Pc(3,3), c4 + ap*c3];
        end
    end
    clear R;
    LAB = {'whole base 2b(1-a)Cov(e4,u)', 'code  base 2b(1-a)a''Cov(e3,u)', 'RESID base 2b(1-a)Cov(z,u)', ...
           'whole mcorr', 'code  mcorr', 'RESID mcorr'};
    SEG = {'hold0', t <= t1; 'far half', t > t1 & t <= t1 + 0.5*(t2-t1); 'near half', t > t1 + 0.5*(t2-t1) & t <= t2; ...
           'osc/last', t > t2 & t <= t3; 'hold 1st s', t > t3 & t <= t3 + 1; 'hold last 3 s', t > cfg0.T_sim - 3};
    fprintf('[%s e4-line] lambda_c %.2f | seeds %s | est - true drift per step (e-6/step, SEM over seeds), cum = sum over run [a_bar]\n', traj, lc, mat2str(seeds));
    for v = 1:6
        T = TERM(:,:,v);
        fprintf('  %-32s | ', LAB{v});
        for s = 1:size(SEG,1)
            m = SEG{s,2};  ps = mean(T(m,:), 1);
            fprintf('%s %+.3f(%.3f) ', SEG{s,1}, 1e6*mean(ps), 1e6*std(ps)/sqrt(nS));
        end
        cs = cumsum(T, 1);
        fprintf('| cum @t3 %+.5f  @end %+.5f\n', mean(cs(find(t > t3, 1), :)), mean(cs(end, :)));
    end
    mh = t > t3;
    fprintf('  hold: mean Cov(z,e3) = P34 + a''P33 = %+.3e | mean Cov(z,u_base) %+.3e\n', mean(ZC(mh,:,1), 'all'), mean(ZC(mh,:,2), 'all'));
    % measured E for the same seeds (noisy at 10 seeds; the run_btrue_nw_mcorr numbers are the ones to compare)
    m3 = t > cfg0.T_sim - 3;  k = (1:sum(m3)).';  sl = zeros(1,nS); for q = 1:nS; p = polyfit(k, E(m3,q), 1); sl(q) = p(1); end
    fprintf('  base arm measured here: last-3-s slope %+.3f e-6/step (SEM %.3f) | short-hold (1st 1.3 s) %+.5f\n', ...
        1e6*mean(sl), 1e6*std(sl)/sqrt(nS), mean(E(t > t3 & t <= t3 + 1.3, :), 'all'));
    out = struct('traj', traj, 'seeds', seeds, 't', t, 'TERM', TERM, 'ZC', ZC, 'E', E, 'phases', [t1 t2 t3 cfg0.T_sim], 'lab', {LAB});
    save(fullfile(od, sprintf('probe_btrue_e4_line_%s.mat', traj)), '-struct', 'out', '-v7.3');
    fprintf('[%s e4-line] saved probe_btrue_e4_line_%s.mat\n', traj, traj);
end
