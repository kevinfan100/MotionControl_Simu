% STATUS: ACTIVE (scratch) | PURPOSE: discriminator for the b_true spread chain
%   (2026-08-31): P44 inflated -> l42 10x -> y2 noise recycled into a_hat.
%   Arm = b_true, seed-at-truth, y2_on = false, 30 seeds, Meng 10 s + hold.
%   Wiring check printed first: l42 must be identically 0 (y2_off consumed at
%   motion_control_law_formC_b:1394).
%   PREDICTIONS (written before the run):
%     if y2 is the main spread injector -> nearwall sd drops toward the
%       a'_true level (~0.2-0.6%);
%     bias: open -- y2 was the main counterweight to the law push (ledger
%       -3.7% in hold), so the +0.8% could grow when y2 is removed.
function out = run_btrue_y2off_arm()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    w0bar = 15/2.25; [~, cp] = calc_correction_functions(w0bar);
    ws0 = 1 + w0bar - 1/((8/9)*(1 - 1/cp));
    OV = struct('trajectory_type','osc','h_init',15,'h_bottom',2.5,'amplitude',0, ...
                'frequency',1,'n_cycles',1,'t_hold',0.5,'t_descend_override',10, ...
                'T_sim',12.5,'h_min',2.475);
    o = struct('arm','best','b_true',true,'b_true_at','true','y2_on',false, ...
               'ctrl_const_override',struct('ws0_perp',ws0), ...
               'config_override',OV,'scenario','deep','verbose',false,'seeds',1:30);
    clear run_formC_b motion_control_law_formC_b;
    evalc('R = run_formC_b(o);');
    t = R.runs{1}.tout(:); nS = 30; E = zeros(numel(t), nS); AH = E; P44 = E; L42 = E;
    for q = 1:nS
        r = R.runs{q}; ad = r.a_hat_out(1,3)/r.a_bar_hat_out(1,3);
        AH(:,q) = r.a_bar_hat_out(:,3); E(:,q) = AH(:,q) - r.a_true_out(:,3)/ad;
        P44(:,q) = r.P_a_out(:,3)/ad; L42(:,q) = r.K_a_y2_out(:,3);
    end
    fprintf('wiring check: max|l42| = %.3e (must be 0)\n', max(abs(L42(:))));
    m1 = t>=7.5 & t<=10.5; m2 = t>10.5;
    fprintf('[btrue y2off seed=truth] nearwall mean %+.5f (SEM %.5f) sd %.5f | hold mean %+.5f (SEM %.5f) sd %.5f\n', ...
        mean(E(m1,:),'all'), std(mean(E(m1,:),1))/sqrt(nS), mean(std(E(m1,:),0,2)), ...
        mean(E(m2,:),'all'), std(mean(E(m2,:),1))/sqrt(nS), mean(std(E(m2,:),0,2)));
    fprintf('sqrtP44: nearwall %.5f | hold %.5f\n', mean(P44(m1,:),'all'), mean(P44(m2,:),'all'));
    out = struct('t',t,'E',E,'sdE',std(AH,0,2),'sP',mean(P44,2));
    S = out; save(fullfile(od,'arm30_btrue_y2off.mat'), '-struct', 'S', '-v7.3');
end
