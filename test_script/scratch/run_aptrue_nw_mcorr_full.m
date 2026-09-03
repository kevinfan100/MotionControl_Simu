% FORK OF test_script/scratch/run_aptrue_nw_mcorr.m (2026-09-03) | PURPOSE: the same two arms
%   (base = exact + @est + pred_mean2; nwmcorr = + nw_mcorr) at the STANDARD run length with the
%   full compact schema (a_hat, a_true, hd, h_bar_true per seed) for the single-vs-seeds comparison
%   figure plot_aptrue_two_traj_nw_mcorr.m. Same 10 seeds, both trajectories.
%   Output: aptrue_nw_mcorr_full_<traj>.mat | EXPIRES: with the parent
function run_aptrue_nw_mcorr_full(seeds)
    if nargin < 1 || isempty(seeds); seeds = 1:10; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'apd_acov_meng');  pc = physical_constants();
    for tr = {'canon','meng'}
        traj = tr{1};
        switch traj
            case 'meng'; OV = struct('trajectory_type','osc','h_init',15,'h_bottom',2.5,'amplitude',0,'frequency',1,'n_cycles',1,'t_hold',0.5,'t_descend_override',10,'T_sim',12.5,'h_min',2.475); cfg0 = OV;
            case 'canon'; OV = struct(); cfg0 = canonical_scenario(0.05, 1.1, 'deep');
        end
        w0bar = cfg0.h_init / pc.R; [~, cp] = calc_correction_functions(w0bar); at = 1/cp; ws0 = 1 + w0bar - 1/((8/9)*(1 - at));
        t3 = cfg0.t_hold + cfg0.t_descend_override + cfg0.n_cycles/cfg0.frequency;
        ARM = {'base','nwmcorr'};  out = struct('traj', traj, 'seeds', seeds, 't_hold', t3);  nS = numel(seeds);
        for a = 1:2
            cc = struct('lock_b',true,'ws0_perp',ws0,'law_exact_step',true,'pred_mean2',true);
            if a == 2; cc.nw_mcorr = true; end
            o = struct('arm','best','ap_known',true,'ap_known_at','est','app_known',true,'ctrl_const_override',cc, ...
                       'config_override',OV,'scenario','deep','verbose',false,'seeds',seeds,'log_P_full',false);
            clear run_formC_b motion_control_law_formC_b;
            evalc('R = run_formC_b(o);');
            t = R.runs{1}.tout(:);  N = numel(t);  E = zeros(N,nS); AH = E; AT = E; HB = E;
            for q = 1:nS
                r = R.runs{q}; ad = r.a_hat_out(1,3)/r.a_bar_hat_out(1,3);
                AH(:,q) = r.a_bar_hat_out(:,3); AT(:,q) = r.a_true_out(:,3)/ad; E(:,q) = AH(:,q) - AT(:,q); HB(:,q) = r.h_bar_true_out(:,1);
            end
            hd = R.runs{1}.p_d_out(:,3)/R.runs{1}.R;  clear R;
            out.(ARM{a}) = struct('t', t, 'E', E, 'AH', AH, 'AT', AT, 'HB', HB, 'hd', hd);
            fprintf('[%s %-7s] hold est-true %+.5f (SEM %.5f) | sd %.5f\n', traj, ARM{a}, mean(mean(E(t>t3,:),1)), std(mean(E(t>t3,:),1))/sqrt(nS), mean(std(E(t>t3,:),0,2)));
        end
        save(fullfile(od, sprintf('aptrue_nw_mcorr_full_%s.mat', traj)), '-struct', 'out', '-v7.3');
        fprintf('[%s] saved aptrue_nw_mcorr_full_%s.mat\n', traj, traj);
    end
end
