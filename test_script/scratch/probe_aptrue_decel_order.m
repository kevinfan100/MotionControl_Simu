% STATUS: ACTIVE (scratch) | PURPOSE: is the Meng deceleration-phase residual of the
%   a'_true arm 4 (+0.0018 present at dwell start, made in t = 8-10.5, living in the
%   update channel) FIRST order (deterministic: unmodeled velocity-dependent tracking
%   lag => biased y1 innovation) or SECOND order (noise rectification)? (2026-09-02)
%   Zero-noise run (T_scale 0, meas 0): first order => residual stays; second => 0.
%   k = 2 noise (T_scale 4, meas x2): first order => same; second => x4.
function probe_aptrue_decel_order()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    pc = physical_constants(); MEAS0 = [0.00062; 0.00057; 0.00331];
    w0bar = 15/pc.R; [~, cp] = calc_correction_functions(w0bar); at = 1/cp; ws0 = 1 + w0bar - 1/((8/9)*(1 - at));
    K = [0.01 0.5]; SEEDS = {1:3, 1:10};   % k=0 rejected by the driver (sigma2_n_s > 0); 0.01 = 1e-4 noise power
    for i = 1:3
        k = K(i);
        OV = struct('trajectory_type','osc','h_init',15,'h_bottom',2.5,'amplitude',0,'frequency',1,'n_cycles',1, ...
                    't_hold',0.5,'t_descend_override',10,'T_sim',12.5,'h_min',2.475,'T_scale',k^2,'meas_noise_std',MEAS0*k);
        o = struct('arm','best','config_override',OV,'scenario','deep','verbose',false,'seeds',SEEDS{i},'log_P_full',false, ...
                   'ap_known',true,'ap_known_at','est','app_known',true, ...
                   'ctrl_const_override',struct('lock_b',true,'ws0_perp',ws0,'law_exact_step',true,'pred_mean2',true));
        clear run_formC_b motion_control_law_formC_b;
        try
            evalc('R = run_formC_b(o);');
        catch ME
            fprintf('k=%g failed: %s\n', k, ME.message); continue;
        end
        t = R.runs{1}.tout(:); nS = numel(SEEDS{i}); E = zeros(numel(t), nS);
        for q = 1:nS; r = R.runs{q}; ad = r.a_hat_out(1,3)/r.a_bar_hat_out(1,3); E(:,q) = r.a_bar_hat_out(:,3) - r.a_true_out(:,3)/ad; end
        m = mean(E,2); sem = std(E,0,2)/sqrt(nS);
        fprintf('k=%g (%d seeds): m(t=8) %+.5f | m(9) %+.5f | m(10) %+.5f | m(10.5) %+.5f | hold %+.5f (SEM %.5f) | NaN %d\n', ...
            k, nS, m(find(t>=8,1)), m(find(t>=9,1)), m(find(t>=10,1)), m(find(t>=10.5,1)), mean(E(t>10.6,:),'all'), std(mean(E(t>10.6,:),1))/sqrt(nS), sum(~isfinite(E(:))));
        clear R;
    end
end
