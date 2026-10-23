% STATUS: ACTIVE (scratch) | PURPOSE: is the +3 % Var(dh_mr) excess seen at the
%   FAR-FIELD hold with a_hat-driven control (verify_cdpmr_hold_estimated,
%   15 seeds, 2 sigma) real? Same hold (w_bar 6.67, 4.5 s, window 2.5-4.5 s),
%   more seeds, both arms paired: control on a_hat vs control on a_true.
%   The pole is lambda_c in both (a_hat/a = 1.000 there), so any paired
%   difference is what a FLUCTUATING gain in the law does that a fixed one
%   does not.
function out = check_farfield_var_excess(seeds, wbar)
    if nargin < 1 || isempty(seeds); seeds = 1:45; end
    if nargin < 2 || isempty(wbar); wbar = 6.67; end      % any height: the same paired check near the wall
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    pc = physical_constants(); ax = 3; h = wbar*pc.R; T0 = 2.5;
    ov = struct('trajectory_type','osc','h_init',h,'h_bottom',h,'amplitude',0,'frequency',1,'n_cycles',1,'t_hold',0.5,'t_descend_override',0.01,'T_sim',4.5,'h_min',1.1*pc.R);
    nS = numel(seeds); V = zeros(2, nS); RB = V; EH = V; GSD = V; VP = zeros(1, nS);
    for c = 1:2
        clear run_formC_b motion_control_law_formC_b;
        if c == 1; [~, O] = evalc("run_formC_b(struct('arm','best','seeds',seeds,'config_override',ov));");
        else;      [~, O] = evalc("run_formC_b(struct('arm','best','seeds',seeds,'config_override',ov,'a_ctrl_override','true'));"); end
        for q = 1:nS
            r = O.runs{q}; t = r.tout(:); m = t >= T0; cc = r.ctrl_const; P = r.meta.params_value;
            M = r.dh_m_out(:,ax); N = numel(M); md = zeros(N+1,1); for k = 1:N; md(k+1) = (1-cc.a_pd)*md(k) + cc.a_pd*M(k); end; R = M - md(2:end);
            a = mean(r.a_true_out(m,ax)); F = cc.C_dpmr*4*P.ctrl.k_B*P.ctrl.T*a + cc.C_n*P.ctrl.sigma2_noise(ax);
            V(c,q) = var(R(m))/F; RB(c,q) = mean(r.a_xm_out(m,ax))/a - 1; EH(c,q) = mean(r.a_hat_out(m,ax))/a - 1;
            GSD(c,q) = std(r.a_hat_out(m,ax))/a;                       % how much the applied gain fluctuates
            if c == 1                                                   % pole prediction from this seed's a_hat
                a_pd = cc.a_pd; lc = cc.lambda_c; g = a/mean(r.a_hat_out(m,ax)); le = 1 - g*(1-lc);
                Cd = (1-a_pd)^2*(2*(1-a_pd)*(1-le)/(1-(1-a_pd)*le) + 2/((2-a_pd)*(1+le)*(1-(1-a_pd)*le)));
                Cn = (1-a_pd)^2*(2/(2-a_pd) + 2*(1-a_pd)^2*a_pd*(1-le)/((2-a_pd)*(1-(1-a_pd)*le)) + 2*(1-le)^2/((2-a_pd)*(1+le)*(1-(1-a_pd)*le)));
                VP(q) = (Cd*4*P.ctrl.k_B*P.ctrl.T*a + Cn*P.ctrl.sigma2_noise(ax))/F;
            end
        end
    end
    d = V(1,:) - V(2,:);
    fprintf('hold at w_bar %.2f, %d seeds paired, window %.1f-%.1f s\n', wbar, nS, T0, 4.5);
    fprintf('  Var/formula : a_hat-driven %.4f +- %.4f | true-gain %.4f +- %.4f | paired diff %+.4f +- %.4f (t = %.1f)\n', mean(V(1,:)), std(V(1,:))/sqrt(nS), mean(V(2,:)), std(V(2,:))/sqrt(nS), mean(d), std(d)/sqrt(nS), mean(d)/(std(d)/sqrt(nS)));
    fprintf('  readout a_m/a-1 : %+.2f%% +- %.2f | %+.2f%% +- %.2f\n', 100*mean(RB(1,:)), 100*std(RB(1,:))/sqrt(nS), 100*mean(RB(2,:)), 100*std(RB(2,:))/sqrt(nS));
    fprintf('  a_hat/a-1 (a_hat-driven): %+.2f%% +- %.2f ; sd of the applied gain within the window: %.2f%% of a\n', 100*mean(EH(1,:)), 100*std(EH(1,:))/sqrt(nS), 100*mean(GSD(1,:)));
    fprintf('  pole prediction of Var/formula for the a_hat-driven arm: %.4f  (measured paired excess over true-gain: %+.4f)\n', mean(VP), mean(d));
    out = struct('V', V, 'RB', RB, 'EH', EH, 'GSD', GSD, 'VP', VP, 'seeds', seeds, 'wbar', wbar);
    save(fullfile(root, 'test_results', 'apd_acov_meng', sprintf('check_var_excess_w%.2f.mat', wbar)), 'out', '-v7.3');
end
