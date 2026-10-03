% STATUS: ACTIVE (scratch) | PURPOSE: explain the two Meng-vs-canon differences of the
%   a'_true arm 4 (2026-09-02). [假說] both come from the WALL PRIOR the seed-at-truth
%   arm keeps while its real initial spread is zero: on Meng the prior maps to
%   sqrt(P44[0]) = 2.75e-3 (start 6.67 R), on canon to 2.6e-4 (start 22 R).
%   (1) honesty 0.34 on the Meng descent: sd(E)^2 should equal P44(t) - P44[0] (the
%       prior part rides along with Phi ~ 1 because A_a = 0).
%   (2) hold +0.0020 vs +0.0006: pred_mean2 consumes P34, which carries the same fake
%       prior on Meng. 對手: D3 (orthogonality) scales with near-wall dwell.
%   分開: rerun Meng arm 4 with Pf_w0_std = 0.0105 (prior ~ canon's). If hold -> ~+0.0006
%   and honesty -> ~1, one cause; if hold stays +0.002, the dwell hypothesis stands.
function probe_aptrue_meng_prior()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    S = load(fullfile(od, 'arms10_seedtruth_comp_meng.mat')); d = S.aptrue; t = d.t(:);
    sP = d.sP; sdE = std(d.E,0,2); P0 = sP(1)^2;
    fprintf('(1) Meng arm4: sqrt(P44[0]) %.2e | t: sd(E) | sqrt(P44) | sqrt(P44^2 - P44[0]) | ratio sd/sqrt(P44-P0)\n', sP(1));
    for tt = [2 4 6 8 9 10 11 12.4]
        k = find(t>=tt,1); q = sqrt(max(sP(k)^2 - P0, 0));
        fprintf('     t=%5.1f  %.2e  %.2e  %.2e  %.2f\n', t(k), sdE(k), sP(k), q, sdE(k)/max(q,1e-12));
    end
    pc = physical_constants();
    OV = struct('trajectory_type','osc','h_init',15,'h_bottom',2.5,'amplitude',0, ...
                'frequency',1,'n_cycles',1,'t_hold',0.5,'t_descend_override',10,'T_sim',12.5,'h_min',2.475);
    w0bar = OV.h_init / pc.R; [~, cp] = calc_correction_functions(w0bar); at = 1/cp;
    ws0 = 1 + w0bar - 1/((8/9)*(1 - at));
    for pf = [0.111 0.0105]
        o = struct('arm','best','config_override',OV,'scenario','deep','verbose',false,'seeds',1:10,'log_P_full',false, ...
                   'ap_known',true,'ap_known_at','est','app_known',true,'Pf_w0_std',pf, ...
                   'ctrl_const_override',struct('lock_b',true,'ws0_perp',ws0,'law_exact_step',true,'pred_mean2',true));
        clear run_formC_b motion_control_law_formC_b;
        evalc('R = run_formC_b(o);');
        t = R.runs{1}.tout(:); nS = 10; E = zeros(numel(t), nS); P = E; M2 = E;
        for q = 1:nS
            r = R.runs{q}; ad = r.a_hat_out(1,3)/r.a_bar_hat_out(1,3);
            E(:,q) = r.a_bar_hat_out(:,3) - r.a_true_out(:,3)/ad; P(:,q) = r.P_a_out(:,3)/ad; M2(:,q) = r.pred_mean2_out(:,3);
        end
        md = t>0.5 & t<=10.5; mh = t>10.6;
        fprintf('(2) Pf_w0_std %.4f: sqrt(P44[0]) %.2e | descend honesty %.2f | hold E %+.5f (SEM %.5f) | hold honesty %.2f | mean2 sum %+.5f | E(0) %+.1e\n', ...
            pf, mean(P(1,:)), mean(std(E(md,:),0,2))/mean(P(md,:),'all'), mean(E(mh,:),'all'), std(mean(E(mh,:),1))/sqrt(nS), ...
            mean(std(E(mh,:),0,2))/mean(P(mh,:),'all'), mean(sum(M2,1)), mean(E(1,:)));
        clear R;
    end
end
