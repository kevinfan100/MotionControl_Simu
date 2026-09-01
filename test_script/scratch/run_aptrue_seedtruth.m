% STATUS: ACTIVE (scratch) | PURPOSE: discriminating arm for the L0 bias
%   attribution (2026-08-31). Claim under test: the flat -0.14% mean error of
%   the a'_true@cmd arm is ENTIRELY the seed truncation (far-field first-order
%   anchor 1 - (9/8)/w_bar at finite w_bar0 = 6.667). Move the seed onto the
%   full-series truth via ctrl_const.ws0_perp (the nominal wall enters the
%   seed line and nowhere else) and rerun the same arm.
%   PREDICTION if attribution right: mean E(t) ~ 0 whole run (within the
%   marginal hold drift -0.065%). If the estimator had a systematic
%   non-correcting defect, the bias would rebuild toward -0.14%.
function out = run_aptrue_seedtruth()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    w0bar = 15/2.25; [~, cp] = calc_correction_functions(w0bar); at = 1/cp;
    w0 = w0bar - 1/((8/9)*(1 - at)); ws0 = 1 + w0;
    fprintf('truth %.6f -> ws0_perp = %.6f (seed check %.6f)\n', at, ws0, 1 - 1/((8/9)*(w0bar - w0)));
    OV = struct('trajectory_type','osc','h_init',15,'h_bottom',2.5,'amplitude',0, ...
                'frequency',1,'n_cycles',1,'t_hold',0.5,'t_descend_override',10, ...
                'T_sim',12.5,'h_min',2.475);
    o = struct('arm','best','ap_known',true,'ap_known_at','cmd', ...
               'ctrl_const_override',struct('lock_b',true,'ws0_perp',ws0), ...
               'config_override',OV,'scenario','deep','verbose',false,'seeds',1:30);
    clear run_formC_b motion_control_law_formC_b;
    evalc('R = run_formC_b(o);');
    t = R.runs{1}.tout(:); nS = numel(o.seeds); E = zeros(numel(t), nS);
    for q = 1:nS
        r = R.runs{q}; ad = r.a_hat_out(1,3)/r.a_bar_hat_out(1,3);
        E(:,q) = r.a_bar_hat_out(:,3) - r.a_true_out(:,3)/ad;
    end
    m1 = t>=7.5 & t<=10.5; m2 = t>10.5;
    fprintf('SEED-AT-TRUTH aptrue@cmd: E(t=0) %+.5f | nearwall %+.5f (SEM %.5f) | hold %+.5f (SEM %.5f)\n', ...
        mean(E(1,:)), mean(E(m1,:),'all'), std(mean(E(m1,:),1))/sqrt(nS), ...
        mean(E(m2,:),'all'), std(mean(E(m2,:),1))/sqrt(nS));
    Em = mean(E,2);
    save(fullfile(od,'arm30_aptrue_seedtruth.mat'), 't','E','Em','ws0','-v7.3');
    out = struct('t',t,'E',E,'ws0',ws0);
end
