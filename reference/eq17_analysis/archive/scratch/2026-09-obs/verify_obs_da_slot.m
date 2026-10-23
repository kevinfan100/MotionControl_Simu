% PURPOSE (2026-09-07, E2 gates 4-5): observability of the (b_hat, da) pair in formC_b with slot 6 = additive disturbance
%   (ctrl_const.da_slot). Runs one seed of the bhd arm (estimated b, seed-at-truth P44[0], da free) with obs_dump on and feeds
%   verify_state_observability with free = [1 2 3 4 5 6 8 9], negative control = slot 7 (inert), priors from the run's ctrl_const;
%   then the same arm WITHOUT da (bhp0, free = [1 2 3 4 5 8 9]) so the CRLB of b before / after adding da is on one page.
%   Gate 3 (paper, 0903 tex S11): da and b alias when a' M is constant over the window; expect the worst windows on the uniform
%   far-field descent, separation near the wall and at turning points.  Output verify_obs_da_slot_<traj>.mat
function out = verify_obs_da_slot(traj, seed, win)
    if nargin < 1 || isempty(traj); traj = 'canon'; end
    if nargin < 2 || isempty(seed); seed = 7; end
    if nargin < 3 || isempty(win); win = 500; end       % gate-3 check: on the slow Meng ramp a 500-step window spans 0.13 R, a' dw barely changes
    traj = lower(traj);
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'apd_acov_meng');  pc = physical_constants();
    switch traj
        case 'meng'; OV = struct('trajectory_type','osc','h_init',15,'h_bottom',2.5,'amplitude',0,'frequency',1,'n_cycles',1,'t_hold',0.5,'t_descend_override',10,'T_sim',12.5,'h_min',2.475); cfg0 = OV; p0 = 3e-4;
                     SEG = {'hold0', 0, 0.5; 'far', 0.5, 7; 'near', 7, 10.5; 'hold', 10.5, 12.5};
        case 'canon'; OV = struct(); cfg0 = canonical_scenario(0.05, 1.1, 'deep'); p0 = 1e-5;
                     SEG = {'hold0', 0, 0.5; 'descent', 0.5, 1.5; 'osc', 1.5, 3.5; 'hold', 3.5, 4.8};
    end
    w0bar = cfg0.h_init / pc.R; [~, cp] = calc_correction_functions(w0bar); at = 1/cp; ws0 = 1 + w0bar - 1/((8/9)*(1 - at));
    base = struct('law_exact_step',true,'pred_mean2',true,'nw_mcorr',true,'pred_mean2_e4',true,'fe44_Aa_scale',1,'ws0_perp',ws0,'Pf_w0_std',0,'Pf_a_floor',p0,'obs_dump',true);
    ARMS = {'bhd', true, [1 2 3 4 5 6 8 9]; 'bhp0', false, [1 2 3 4 5 8 9]};
    out = struct('traj', traj, 'seed', seed);
    for ia = 1:2
        cc = base; cc.da_slot = ARMS{ia,2};
        o = struct('arm','best','ctrl_const_override',cc,'config_override',OV,'scenario','deep','verbose',false,'seeds',seed,'log_P_full',false);
        clear run_formC_b motion_control_law_formC_b;
        evalc('R = run_formC_b(o);');
        L = obs_dump('get');  r = R.runs{1};  ad = r.a_hat_out(1,3)/r.a_bar_hat_out(1,3);  cc_run = r.ctrl_const;
        free = ARMS{ia,3};  ps = nan(1, numel(free));  lab = cell(1, numel(free));
        names = {'dw1','dw2','dw3','a_w','b','da','w_s','m1','m2'};
        for i = 1:numel(free); lab{i} = names{free(i)}; end
        ps(free == 4) = r.P_a_out(1,3)/ad;  ps(free == 5) = cc_run.Pf_b_std;  if any(free == 6); ps(free == 6) = cc_run.Pf_da_std(1); end
        cfgv = struct('axis', 3, 'free', free, 'neg_ctrl', 7, 'prior_std', ps, 'labels', {lab}, 'window', win, 't_end', cfg0.T_sim, 'segments', {SEG});
        fprintf('\n===== [%s %s] gates 4-5 (free slots %s, negative control slot 7) =====\n', traj, ARMS{ia,1}, mat2str(free));
        res = verify_state_observability(L, cfgv);
        out.(ARMS{ia,1}) = res;
    end
    save(fullfile(od, sprintf('verify_obs_da_slot_%s_w%d.mat', traj, win)), '-struct', 'out', '-v7.3');
    fprintf('[%s] saved verify_obs_da_slot_%s.mat\n', traj, traj);
end
