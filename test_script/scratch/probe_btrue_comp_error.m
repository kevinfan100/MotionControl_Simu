% PURPOSE (2026-09-04): does the descent dip of the b_true arm equal the ERROR of the second-order compensation, i.e. the
%   difference between the true cross-seed moments and the filter's P-based moments that pred_mean2 (+ e4 line) uses?
%   Per step (obs_dump, exact): e_D = Delta w_true - M_hat (M_hat back-solved from the exact step), e4 = a_true - a_hat,
%   e3 = dw3_true - dw3_hat (all at the start of the step), F_dw = -F(3,4) from the captured Jacobian, P from P_upd of the
%   previous record. The code adds per step   mean2 = A_a cov_e4u_P + 1/2 a''_law var_u_P   with
%   cov_e4u_P = (1-lc)(P34 - P14) + alpha(P48 + P49) + F_dw P44,  var_u_P = the pred_mean2 expression (nw_mcorr form).
%   The truth carries (seed mean)   A_a Cov_seeds(e4, e_D) + 1/2 a'' E_seeds[e_D^2]  (+ the terms that vanish in mean).
%   Reports, per window: Sum over steps of  A_a (Cov_true - cov_P)  and  1/2 a'' (Var_true - var_P)  and their total,
%   against the measured dip (canon 1.0-1.5 s: predict - truth -0.0103; dip -0.0105). Also the honesty of P44, P34 there.
%   20 seeds (cross-seed moments per step are noisy; the window sums average them). | EXPIRES: with the b_true rung
function out = probe_btrue_comp_error(traj, seeds)
    if nargin < 1 || isempty(traj); traj = 'canon'; end
    if nargin < 2 || isempty(seeds); seeds = 1:20; end
    traj = lower(traj);
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'apd_acov_meng');  pc = physical_constants();
    switch traj
        case 'meng'; OV = struct('trajectory_type','osc','h_init',15,'h_bottom',2.5,'amplitude',0,'frequency',1,'n_cycles',1,'t_hold',0.5,'t_descend_override',10,'T_sim',12.5,'h_min',2.475); cfg0 = OV;
                     SEG = {'far 2-5 s',[2 5]; 'mid 5-7 s',[5 7]; 'near 7-10 s',[7 10]; 'osc 10.5-11.5',[10.5 11.5]; 'hold',[11.5 12.5]};
        case 'canon'; OV = struct(); cfg0 = canonical_scenario(0.05, 1.1, 'deep');
                     SEG = {'pre 0.6-1.0',[0.6 1.0]; 'fast 1.0-1.5',[1.0 1.5]; 'osc 1.5-3.5',[1.5 3.5]; 'hold',[3.5 4.8]};
    end
    w0bar = cfg0.h_init / pc.R; [~, cp] = calc_correction_functions(w0bar); at = 1/cp; ws0 = 1 + w0bar - 1/((8/9)*(1 - at));
    if isfield(cfg0, 'lambda_c'); lc = cfg0.lambda_c; else; lc = 0.7; end;  alpha = 1 - lc;   % Meng override struct carries no lambda_c (house value 0.7)
    cc = struct('law_exact_step',true,'pred_mean2',true,'nw_mcorr',true,'pred_mean2_e4',true,'ws0_perp',ws0,'obs_dump',true);
    nS = numel(seeds);  ED = []; E4 = []; E3 = []; CP = []; VP = []; AA = []; APP = []; P44 = []; P34 = []; T = []; M2 = [];
    for q = 1:nS
        o = struct('arm','best','b_true',true,'b_true_at','true','ctrl_const_override',cc,'config_override',OV,'scenario','deep','verbose',false,'seeds',seeds(q),'log_P_full',false);
        clear run_formC_b motion_control_law_formC_b;
        evalc('R = run_formC_b(o);');
        L = obs_dump('get');  Lz = L([L.ax] == 3);  n = numel(Lz);  r = R.runs{1};
        k = [Lz.k].';  xp = cell2mat(arrayfun(@(z) z.x_pred(:).', Lz, 'UniformOutput', false).');
        xu = cell2mat(arrayfun(@(z) z.x_upd(:).',  Lz, 'UniformOutput', false).');
        ad = r.a_hat_out(1,3)/r.a_bar_hat_out(1,3);  t = r.tout(:);
        bu = r.b_hat_out(:,3);  m2 = r.pred_mean2_out(:,3);  hb = r.h_bar_true_out(:,1);  hd = r.p_d_out(:,3)/r.R;
        atr = r.a_true_out(:,3)/ad;  ap = r.a_prime_out(:,3)/ad;
        J = 2:n;  kd = min(k(J) + 1, numel(t));                                   % record j <-> driver index kd
        Mhat = (1 ./ (1 - (xp(J,4) - m2(kd))) - 1 ./ (1 - xu(J-1,4))) ./ bu(kd);
        eD = (hb(kd) - hb(kd-1)) - Mhat;
        e4 = atr(kd-1) - xu(J-1,4);
        e3 = (hd(kd-1) - hb(kd-1)) - xu(J-1,3);                                     % dw3_true - dw3_hat at the start of the step
        nrec = numel(J);  cp_ = zeros(nrec,1); vp_ = cp_; aa_ = cp_; app_ = cp_; p44 = cp_; p34 = cp_;
        for jj = 1:nrec
            j = J(jj);  Pc = Lz(j-1).P_upd;  F = Lz(j).F;  Fdw = -F(3,4);  np = size(Pc,1);
            ah = xu(j-1,4);  b = bu(kd(jj));  apk = b * (1 - ah)^2;  A_a = -2 * b * (1 - ah);  app = -2 * apk^2 / (1 - ah);
            c4 = alpha * Pc(3,4) + Fdw * Pc(4,4);  if np >= 9; c4 = c4 + alpha * (Pc(4,8) + Pc(4,9)); end;  c4 = c4 - alpha * Pc(1,4);
            R1 = Lz(j).R(1);  s2n = R1;
            if np >= 9
                vu = alpha^2 * Pc(3,3) + alpha^2 * (Pc(8,8) + Pc(9,9) + 2*Pc(8,9)) + 2*alpha*alpha*(Pc(3,8) + Pc(3,9)) + Fdw^2 * Pc(4,4) ...
                   + 2*Fdw*(alpha*Pc(3,4) + alpha*(Pc(4,8) + Pc(4,9)));
                vu = vu + alpha^2 * Pc(1,1) - 2*alpha^2*(Pc(1,3) + Pc(1,8) + Pc(1,9)) - 2*Fdw*alpha*Pc(1,4);   % nw_mcorr form, s2T (thermal) added below
            else
                vu = alpha^2*Pc(3,3) + Fdw^2*Pc(4,4) + 2*alpha*Fdw*Pc(3,4) + alpha^2*Pc(1,1) - 2*alpha^2*Pc(1,3) - 2*Fdw*alpha*Pc(1,4);
            end
            cp_(jj) = c4;  vp_(jj) = vu;  aa_(jj) = A_a;  app_(jj) = app;  p44(jj) = Pc(4,4);  p34(jj) = Pc(3,4);
        end
        ED(:,q) = eD; E4(:,q) = e4; E3(:,q) = e3; CP(:,q) = cp_; VP(:,q) = vp_; AA(:,q) = aa_; APP(:,q) = app_; P44(:,q) = p44; P34(:,q) = p34; M2(:,q) = m2(kd); T = t(kd);
        fprintf('[%s] seed %2d done (%d records)\n', traj, seeds(q), n);
    end
    % cross-seed moments per step vs the filter's
    covT = mean((E4 - mean(E4,2)) .* (ED - mean(ED,2)), 2) * nS/(nS-1);    % Cov_seeds(e4, e_D)
    varT = mean(ED.^2, 2);                                                   % E[e_D^2] (thermal included)
    covP = mean(CP, 2);  Aa = mean(AA, 2);  App = mean(APP, 2);
    % the P-based var_u lacks the thermal share s2T (kappa_T a) in this reconstruction: compare the e4 line only and the Jensen
    % via the thermal-free part: E[e_D^2] - thermal is not separable here, so report the e4 line exactly and the Jensen with the
    % thermal share estimated as the hold-window value of E[e_D^2] - var_P (printed separately).
    fprintf('[%s] per-window sums (est - true sign = minus the truth''s mean increment): A_a*(Cov_true - Cov_P) = compensation error of the e4 line\n', traj);
    for s = 1:size(SEG,1)
        m = T >= SEG{s,2}(1) & T <= SEG{s,2}(2);
        errE4 = sum(Aa(m) .* (covT(m) - covP(m)));
        honP44 = sqrt(mean(var(E4(m,:),0,2)) / mean(P44(m,:),'all'));
        fprintf('[%s] %-14s Sum A_a Cov_true %+.5f | Sum A_a Cov_P %+.5f | e4-line comp. error (est-true) %+.5f | sqrt(Var(e4)/P44) %.2f | mean Cov_true(e4,eD) %+.2e vs Cov_P %+.2e | steps %d\n', ...
            traj, SEG{s,1}, sum(Aa(m).*covT(m)), sum(Aa(m).*covP(m)), -errE4, honP44, mean(covT(m)), mean(covP(m)), sum(m));
    end
    out = struct('traj', traj, 'seeds', seeds, 't', T, 'eD', ED, 'e4', E4, 'e3', E3, 'covP', CP, 'varP', VP, 'Aa', AA, 'app', APP, 'P44', P44, 'P34', P34, 'mean2', M2);
    save(fullfile(od, sprintf('probe_btrue_comp_error_%s.mat', traj)), '-struct', 'out', '-v7.3');
    fprintf('[%s] saved probe_btrue_comp_error_%s.mat\n', traj, traj);
end
