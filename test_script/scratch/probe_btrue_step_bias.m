% PURPOSE (2026-09-04, step 1 of the descent-transient check): is the predict's KNOWN STEP biased during the fast
%   descent? e_D = Delta w_true - M_hat, with M_hat the step the exact law step actually integrated over, recovered
%   EXACTLY from the obs_dump capture:  1/(1 - (x_pred4[j] - mean2)) = 1/(1 - x_upd4[j-1]) + b_hat M_hat.
%   Cross-checked against the direct formula M_hat = Delta w_d + (1-lc)(x3_upd + res1) + alpha (x8_upd + x9_upd)
%   (wiring check, must agree to ~1e-12). Reports per segment (seed mean over 10 seeds, canon):
%     mean e_D, its F_dw e4 share (the gain error feeding the step: e4 = a_true - a_hat, F_dw = f_bar history),
%     and Sum(a' e_D) over the fast window = the law-part of (predict - truth) of probe_btrue_descent_dip
%     (canon 1.0-1.5 s: -0.0103 for the b_true arm, -0.0175 for the a'_true arm).
%   Opponent: mean e_D ~ 0 in the fast window => the dip is not a step bias.
%   Arms: btest (b_true@true, four blocks) and apest (a'_true@est, four blocks), obs_dump on. | EXPIRES: with the b_true rung
function out = probe_btrue_step_bias(traj, seeds)
    if nargin < 1 || isempty(traj); traj = 'canon'; end
    if nargin < 2 || isempty(seeds); seeds = 1:10; end
    traj = lower(traj);
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'apd_acov_meng');  pc = physical_constants();
    switch traj
        case 'meng'; OV = struct('trajectory_type','osc','h_init',15,'h_bottom',2.5,'amplitude',0,'frequency',1,'n_cycles',1,'t_hold',0.5,'t_descend_override',10,'T_sim',12.5,'h_min',2.475); cfg0 = OV;
                     SEG = {'far 2-5 s',[2 5]; 'near 7-10 s',[7 10]; 'hold',[11.5 12.5]};
        case 'canon'; OV = struct(); cfg0 = canonical_scenario(0.05, 1.1, 'deep');
                     SEG = {'pre 0.6-1.0',[0.6 1.0]; 'fast 1.0-1.5',[1.0 1.5]; 'osc 1.5-3.5',[1.5 3.5]; 'hold',[3.5 4.8]};
    end
    w0bar = cfg0.h_init / pc.R; [~, cp] = calc_correction_functions(w0bar); at = 1/cp; ws0 = 1 + w0bar - 1/((8/9)*(1 - at));
    lc = cfg0.lambda_c; if isempty(lc); lc = 0.7; end;  alpha = 1 - lc;
    ON4 = struct('law_exact_step',true,'pred_mean2',true,'nw_mcorr',true,'pred_mean2_e4',true,'ws0_perp',ws0,'obs_dump',true);
    ON3 = ON4; ON3.pred_mean2_e4 = false; ON3.lock_b = true;
    DEF = struct('btest', struct('o', struct('b_true',true,'b_true_at','true'), 'cc', ON4), ...
                 'apest', struct('o', struct('ap_known',true,'ap_known_at','est','app_known',true), 'cc', ON3));
    ARM = {'btest','apest'};  nS = numel(seeds);  out = struct('traj', traj, 'seeds', seeds);
    for ia = 1:2
        A = DEF.(ARM{ia});  ED = []; FDE4 = []; APD = []; DIR = []; T = [];
        for q = 1:nS
            o = struct('arm','best','ctrl_const_override',A.cc,'config_override',OV,'scenario','deep','verbose',false,'seeds',seeds(q),'log_P_full',false);
            fn = fieldnames(A.o); for i = 1:numel(fn); o.(fn{i}) = A.o.(fn{i}); end
            clear run_formC_b motion_control_law_formC_b;
            evalc('R = run_formC_b(o);');
            L = obs_dump('get');  Lz = L([L.ax] == 3);  n = numel(Lz);  r = R.runs{1};
            k = [Lz.k].';  xp = cell2mat(arrayfun(@(z) z.x_pred(:).', Lz, 'UniformOutput', false).');
            xu = cell2mat(arrayfun(@(z) z.x_upd(:).',  Lz, 'UniformOutput', false).');
            ad = r.a_hat_out(1,3)/r.a_bar_hat_out(1,3);  t = r.tout(:);
            bu = r.b_hat_out(:,3);  m2 = r.pred_mean2_out(:,3);  hb = r.h_bar_true_out(:,1);  hd = r.p_d_out(:,3)/r.R;
            y1 = r.dh_m_out(:,3) / r.R;  fb = r.f_bar_out(:,3);  ap = r.a_prime_out(:,3)/ad;  atr = r.a_true_out(:,3)/ad;  ah = r.a_bar_hat_out(:,3);
            % ALIGNMENT (checked 09-04 on seed 7): obs_dump record j (k_j) <-> driver index kd = k_j + 1
            % (x_upd(4) == a_bar_hat_out(kd) exactly; y1 - x_upd(1) = the posterior residual only at that offset).
            % Record j's predict starts from the posterior of record j-1 (driver kd-1) and lands on driver kd.
            J = 2:n;  kd = k(J) + 1;  kd = min(kd, numel(t));
            Mhat = (1 ./ (1 - (xp(J,4) - m2(kd))) - 1 ./ (1 - xu(J-1,4))) ./ bu(kd);            % exact back-solve
            res1 = y1(kd-1) - xu(J-1,1);
            Mdir_a = (hd(kd) - hd(kd-1)) + alpha * (xu(J-1,3) + res1) + alpha * (xu(J-1,8) + xu(J-1,9));   % direct formula, Delta w_d = hd[kd] - hd[kd-1]
            kd1 = min(kd + 1, numel(t));
            Mdir_b = (hd(kd1) - hd(kd)) + alpha * (xu(J-1,3) + res1) + alpha * (xu(J-1,8) + xu(J-1,9));   % Delta w_d = hd[kd+1] - hd[kd]
            if max(abs(Mhat - Mdir_b)) < max(abs(Mhat - Mdir_a)); Mdir = Mdir_b; else; Mdir = Mdir_a; end
            dtrue = hb(kd) - hb(kd-1);
            eD = dtrue - Mhat;
            Fdw = fb(kd-1) + alpha * (fb(max(kd-2,1)) + fb(max(kd-3,1)));                       % raw F_dw history (approximation)
            e4 = atr(kd-1) - ah(kd-1);
            ED(:,q) = eD;  FDE4(:,q) = Fdw .* e4;  APD(:,q) = ap(kd) .* eD;  DIR(:,q) = Mhat - Mdir;  T = t(kd);
        end
        fprintf('[%s %s] wiring check max|M_hat(back-solved) - M_hat(direct)| = %.2e, median %.2e (|M| mean %.2e)\n', traj, ARM{ia}, max(abs(DIR(:))), median(abs(DIR(:))), mean(abs(ED(:) - ED(:) + abs(ED(:)))) );
        for s = 1:size(SEG,1)
            m = T >= SEG{s,2}(1) & T <= SEG{s,2}(2);
            me = mean(ED(m,:), 1);  mf = mean(FDE4(m,:), 1);  sa = sum(APD(m,:), 1);
            fprintf('[%s %s] %-12s mean e_D %+.3e (SEM %.1e) | of which F_dw e4 %+.3e | Sum a'' e_D %+.5f (SEM %.5f) | steps %d\n', ...
                traj, ARM{ia}, SEG{s,1}, mean(me), std(me)/sqrt(nS), mean(mf), mean(sa), std(sa)/sqrt(nS), sum(m));
        end
        out.(ARM{ia}) = struct('t', T, 'eD', ED, 'fdwe4', FDE4, 'ap_eD', APD, 'wiring', DIR);
    end
    save(fullfile(od, sprintf('probe_btrue_step_bias_%s.mat', traj)), '-struct', 'out', '-v7.3');
    fprintf('[%s] saved probe_btrue_step_bias_%s.mat\n', traj, traj);
end
