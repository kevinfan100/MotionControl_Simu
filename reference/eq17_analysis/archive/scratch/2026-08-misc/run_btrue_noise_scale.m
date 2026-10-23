% STATUS: ACTIVE (scratch) | PURPOSE: noise-power scaling discriminator for the
%   b_true bias (2026-08-31). Hypothesis A (variance-driven, "one event"):
%   bias is the law's curvature rectifying the spread -> bias ~ noise power
%   (amplitude x0.5 -> bias x0.25; x2 -> x4). Hypothesis B (signed input,
%   velocity x lag residual, deterministic): bias unchanged under noise scaling
%   (gains are scale-invariant when Q and R scale together).
%   Arm = b_true, seed-at-truth, Meng 10 s + 2 s hold, 30 seeds per scale.
%   Knobs: config.T_scale (plant thermal + filter kBT, added 2026-08-31,
%   default 1 bit-identical) and meas_noise_std x k (config field).
%   Noise AMPLITUDE k in {0.5, 1, 2}: T_scale = k^2, meas_noise_std * k.
%   k = 1 arm must reproduce the existing seed-truth numbers (negative control).
function out = run_btrue_noise_scale()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    w0bar = 15/2.25; [~, cp] = calc_correction_functions(w0bar);
    ws0 = 1 + w0bar - 1/((8/9)*(1 - 1/cp));
    MEAS0 = [0.00062; 0.00057; 0.00331];
    K = [0.5 1 2]; out = struct('K', K);
    for i = 1:numel(K)
        k = K(i);
        OV = struct('trajectory_type','osc','h_init',15,'h_bottom',2.5,'amplitude',0, ...
                    'frequency',1,'n_cycles',1,'t_hold',0.5,'t_descend_override',10, ...
                    'T_sim',12.5,'h_min',2.475, ...
                    'T_scale',k^2,'meas_noise_std',MEAS0*k);
        o = struct('arm','best','b_true',true,'b_true_at','true', ...
                   'ctrl_const_override',struct('ws0_perp',ws0), ...
                   'config_override',OV,'scenario','deep','verbose',false,'seeds',1:30);
        clear run_formC_b motion_control_law_formC_b;
        evalc('R = run_formC_b(o);');
        t = R.runs{1}.tout(:); nS = 30; E = zeros(numel(t), nS); AH = E;
        for q = 1:nS
            r = R.runs{q}; ad = r.a_hat_out(1,3)/r.a_bar_hat_out(1,3);
            AH(:,q) = r.a_bar_hat_out(:,3); E(:,q) = AH(:,q) - r.a_true_out(:,3)/ad;
        end
        m1 = t>=7.5 & t<=10.5; m2 = t>10.5;
        fprintf('[k=%.1f] nearwall bias %+.5f (SEM %.5f) sd %.5f | hold bias %+.5f (SEM %.5f) sd %.5f\n', ...
            k, mean(E(m1,:),'all'), std(mean(E(m1,:),1))/sqrt(nS), mean(std(E(m1,:),0,2)), ...
            mean(E(m2,:),'all'), std(mean(E(m2,:),1))/sqrt(nS), mean(std(E(m2,:),0,2)));
        out.(sprintf('k%02d', round(10*k))) = struct('t',t,'E',E);
        clear R;
    end
    S = out; save(fullfile(od,'arm30_btrue_noise_scale.mat'), '-struct', 'S', '-v7.3');
    fprintf('saved arm30_btrue_noise_scale.mat\n');
end
