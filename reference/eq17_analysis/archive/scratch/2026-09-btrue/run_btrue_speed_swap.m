% FORK OF test_script/scratch/run_btrue_aa_off.m (2026-09-06) | PURPOSE: discriminator for WHY the kappa = 0.5 b_true arm keeps
%   3-4x more near-wall spread on Meng than on canon at the SAME height (btrue_spread_vs_height.png: at kappa = 1 the two
%   trajectories overlap vs w/R; at kappa = 0.5 Meng stays 0.012-0.018 while canon drops to 0.003-0.005).
%   Same b_true arm (slope at a_hat, b_true at the previous true height, four blocks, seed-at-truth), 10 seeds, only the
%   descent duration is swapped:
%     mengfast    Meng ramp 15 -> 2.5 um in 1 s   (band speed ~5.6 R/s, like canon's 5.3)      T_sim 3.5 s
%     canonslow   canonical deep, descent 10 s    (band speed ~2.1 R/s)                          T_sim 13.8 s
%     canonvslow  canonical deep, descent 38 s    (band speed ~0.56 R/s, like Meng's 0.42-0.56)  T_sim 41.8 s
%   PRE-REGISTERED (sigma_seed of (a_hat - a)/a_nom vs w/R, descent only, read at w/R = 2.0 / 1.5 / 1.2):
%     A  step-count (time) mechanism: the ordering follows the approach SPEED, not the trajectory --
%        mengfast ~ canon-std (0.002-0.005) < canonslow < canonvslow ~ Meng-std (0.012-0.018) at kappa 0.5;
%        at kappa 1 all arms still overlap (the path-integral term A_a*M dominates there).
%     B  trajectory-specific mechanism (start height 6.67 R vs 22.2 R, prior, tracking): mengfast stays with Meng-std and
%        canonslow / canonvslow stay with canon-std regardless of speed.
%   Health before reading anything: min w_true > 1.0 R, no NaN, min a_hat > 0.031 (floor), descent step count = t_desc / Ts.
%   SECOND ROUND (A refuted: mengfast stays with Meng, canonslow stays with canon): swap the START HEIGHT instead --
%     canonlow   canon shape, h_init 15 um (6.67 R)     menghigh   Meng ramp, h_init 50 um (22.2 R)
%   prediction: if the start height (prior envelope [h_bottom, h_init] and/or the far-field path) is the cause, canonlow rises
%   to Meng's level and menghigh drops to canon's level at kappa 0.5.
%   Output btrue_speed_swap_<arm>.mat (fields k100 / k050: t E AH HB hd sP E_l) | EXPIRES: with the b_true rung | 產線改動不會自動跟上
function out = run_btrue_speed_swap(arm, kappas, seeds)
    if nargin < 2 || isempty(kappas); kappas = [1 0.5]; end
    if nargin < 3 || isempty(seeds); seeds = 1:10; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'apd_acov_meng');  pc = physical_constants();
    switch lower(arm)
        case 'mengfast'
            OV = struct('trajectory_type','osc','h_init',15,'h_bottom',2.5,'amplitude',0,'frequency',1,'n_cycles',1,'t_hold',0.5,'t_descend_override',1,'T_sim',3.5,'h_min',2.475); cfg0 = OV;
        case 'canonslow'
            cfg0 = canonical_scenario(0.05, 1.1, 'deep');  OV = struct('t_descend_override', 10, 'T_sim', 0.5 + 10 + 2 + 1.3);
        case 'canonvslow'
            cfg0 = canonical_scenario(0.05, 1.1, 'deep');  OV = struct('t_descend_override', 38, 'T_sim', 0.5 + 38 + 2 + 1.3);
        case 'canonlow'      % second discriminator (after A was refuted by mengfast / canonslow): canon shape but Meng's START HEIGHT 15 um (6.67 R)
            cfg0 = canonical_scenario(0.05, 1.1, 'deep');  OV = struct('h_init', 15);
        case 'menghigh'      % mirror: Meng ramp (amplitude 0, 10 s) from canon's start height 50 um (22.2 R)
            OV = struct('trajectory_type','osc','h_init',50,'h_bottom',2.5,'amplitude',0,'frequency',1,'n_cycles',1,'t_hold',0.5,'t_descend_override',10,'T_sim',12.5,'h_min',2.475); cfg0 = OV;
        case {'mengp0a', 'mengp0b'}   % THIRD ROUND: Meng std ramp, but P44[0] forced to canon's value -- (a) canon's own P44[0] = 0.00026^2,
                                      % (b) canon's sqrt(P44) when it reaches Meng's start height 6.67 R at kappa 0.5 = 0.00089^2.
                                      % P44[0] = (a_bar'[0])^2 Pf_w0_std^2 + Pf_a_floor^2 (controller header); Pf_w0_std = 0, Pf_a_floor = the value.
                                      % prediction: if the start-height effect is P44[0], (b) falls onto the canon family at kappa 0.5.
            OV = struct('trajectory_type','osc','h_init',15,'h_bottom',2.5,'amplitude',0,'frequency',1,'n_cycles',1,'t_hold',0.5,'t_descend_override',10,'T_sim',12.5,'h_min',2.475); cfg0 = OV;
            if strcmp(lower(arm), 'mengp0a'); XC = struct('Pf_w0_std', 0, 'Pf_a_floor', 0.00026); else; XC = struct('Pf_w0_std', 0, 'Pf_a_floor', 0.00089); end
        otherwise, error('arm %s', arm);
    end
    if ~exist('XC', 'var'); XC = struct(); end
    fn = fieldnames(OV); for i = 1:numel(fn); cfg0.(fn{i}) = OV.(fn{i}); end
    w0bar = cfg0.h_init / pc.R; [~, cp] = calc_correction_functions(w0bar); at = 1/cp; ws0 = 1 + w0bar - 1/((8/9)*(1 - at));
    t3 = cfg0.t_hold + cfg0.t_descend_override + cfg0.n_cycles/cfg0.frequency;  nS = numel(seeds);
    out = struct('arm', arm, 'seeds', seeds, 't_hold', t3, 't_desc', cfg0.t_descend_override, 'kappas', kappas);
    for ik = 1:numel(kappas)
        cc = struct('law_exact_step',true,'pred_mean2',true,'nw_mcorr',true,'pred_mean2_e4',true,'fe44_Aa_scale',kappas(ik),'ws0_perp',ws0);
        fx = fieldnames(XC); for i = 1:numel(fx); cc.(fx{i}) = XC.(fx{i}); end
        o = struct('arm','best','b_true',true,'b_true_at','true','ctrl_const_override',cc,'config_override',OV,'scenario','deep','verbose',false,'seeds',seeds,'log_P_full',false);
        clear run_formC_b motion_control_law_formC_b;
        evalc('R = run_formC_b(o);');
        t = R.runs{1}.tout(:);  N = numel(t);  E = zeros(N,nS); AH = E; HB = E; SP = E; L41 = E; L31 = E; AP = E;
        for q = 1:nS
            r = R.runs{q}; ad = r.a_hat_out(1,3)/r.a_bar_hat_out(1,3);
            AH(:,q) = r.a_bar_hat_out(:,3); E(:,q) = AH(:,q) - r.a_true_out(:,3)/ad; HB(:,q) = r.h_bar_true_out(:,1);
            SP(:,q) = r.P_a_out(:,3)/ad;  L41(:,q) = r.K_a_y1_out(:,3);  L31(:,q) = r.K_dx_y1_out(:,3);  AP(:,q) = r.a_prime_out(:,3)/ad;
        end
        hd = R.runs{1}.p_d_out(:,3)/R.runs{1}.R;  clear R;
        key = sprintf('k%03d', round(100*kappas(ik)));
        out.(key) = struct('t', t, 'E', E, 'AH', AH, 'HB', HB, 'hd', hd, 'sP', SP, 'E_l', L41 + AP .* L31);
        Ts = t(2) - t(1);  v = [0; diff(hd)];  i0 = find(v < -1e-7, 1);  i1 = find(hd <= min(hd) + 1e-9, 1);  nd = i1 - i0 + 1;
        band = HB < 2.5 & repmat(t >= t(i0) & t <= t(i1), 1, nS);
        fprintf('[%s kappa %.2f] HEALTH: min w_true %.4f R | NaN %d | min a_hat %.4f | descent steps %d (expected %d) | band speed %.3f R/s | band time %.2f s\n', ...
            arm, kappas(ik), min(HB(:)), nnz(isnan(E)), min(AH(:)), nd, round(cfg0.t_descend_override/Ts), mean(abs(v(any(band,2))))/Ts, sum(any(band,2))*Ts);
        x = mean(HB(i0:i1,:), 2);  s = std(E(i0:i1,:), 0, 2);  sp = mean(SP(i0:i1,:), 2);  el = mean(out.(key).E_l(i0:i1,:), 2);
        [xu, iu] = unique(x);
        fprintf('[%s kappa %.2f] descent sigma_seed at w/R 2.0 / 1.5 / 1.2: %.4f %.4f %.4f | sqrtP44 %.4f %.4f %.4f | l41 + a'' l31 %+.3f %+.3f %+.3f | hold sd %.4f, hold mean %+.4f (SEM %.4f)\n', ...
            arm, kappas(ik), interp1(xu, s(iu), [2.0 1.5 1.2]), interp1(xu, sp(iu), [2.0 1.5 1.2]), interp1(xu, el(iu), [2.0 1.5 1.2]), ...
            mean(std(E(t > t3,:), 0, 2)), mean(E(t > t3,:), 'all'), std(mean(E(t > t3,:), 1))/sqrt(nS));
    end
    save(fullfile(od, sprintf('btrue_speed_swap_%s.mat', arm)), '-struct', 'out', '-v7.3');
    fprintf('[%s] saved btrue_speed_swap_%s.mat\n', arm, arm);
end
