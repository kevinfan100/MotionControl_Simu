% PURPOSE (2026-09-20): gates 4 and 5 of the observability workflow for the offset-law state delta (0920_estb_delta_law.tex,
%   version A: x = [dw1 dw2 dw3 (a_w derived) (slot 5 locked) . delta]). Runs the instrumented driver (ctrl_const.obs_dump)
%   on the plane and on the cell plant (cell_cperp_delta), canon deep, one seed, then verify_state_observability on the free
%   slots [1 2 3 7] with the LOCKED slot 5 as the negative control (pinned by construction, must come back Inf).
%   Gate 5 compares the CRLB of delta against its prior 0.5 R per window and per segment (hold0 / descent / osc / hold);
%   the paper gates (0920 tex, Observability) predict: y1 row alone loses delta in hold (F_dw = 0) and far field (a' -> 0),
%   y2 row alone keeps it wherever a'(g_d) != 0, so hold near the wall must still show information through y2.
%   Output: console + test_results/temp_delta_two_walls/verify_obs_delta_<plant>_w<win>.mat | EXPIRES: with the delta-law line
function out = verify_obs_delta(seed, win)
    if nargin < 1 || isempty(seed); seed = 7; end
    if nargin < 2 || isempty(win);  win = 500; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration')); addpath(here);
    od = fullfile(root, 'test_results', 'temp_delta_two_walls');  if ~exist(od, 'dir'); mkdir(od); end
    cfg0 = canonical_scenario(0.05, 1.1, 'deep');
    SEG = {'hold0', 0, 0.5; 'descent', 0.5, 1.5; 'osc', 1.5, 3.5; 'hold', 3.5, 4.8};
    DEL = struct('plane', 0.0, 'cell', 1.05);  SIG = 0.5;
    out = struct('seed', seed, 'win', win);
    for pl = {'plane', 'cell'}
        cc = struct('ma2_aug', false, 'nw_mcorr', false, 'pred_mean2', false, 'pred_mean2_e4', false, 'law_exact_step', false, ...
                    'lock_b', true, 'b_ceil', 1.5, 'b_floor', 0.2, 'Pf_a_floor', 1e-5, 'delta_state', true, 'gain_algebraic', true, ...
                    'amodel_fe', true, 'amodel_h', true, 'delta_init', DEL.(pl{1}), 'Pf_delta_std', SIG, 'ws0_perp', 0, 'Pf_w0_std', 0, ...
                    'b_init', 8/9, 'Pf_b_std', 0, 'obs_dump', true);
        o = struct('arm', 'best', 'ctrl_const_override', cc, 'scenario', 'deep', 'verbose', false, 'seeds', seed, 'log_P_full', false);
        if strcmp(pl{1}, 'cell'); o.plant_cperp = @cell_cperp_delta; end
        clear run_formC_b motion_control_law_formC_b;
        evalc('R = run_formC_b(o);');
        L = obs_dump('get');
        free = [1 2 3 7];  ps = [NaN NaN NaN SIG];  lab = {'dw1', 'dw2', 'dw3', 'delta'};
        cfgv = struct('axis', 3, 'free', free, 'neg_ctrl', 5, 'prior_std', ps, 'labels', {lab}, 'window', win, ...
                      't_end', cfg0.T_sim, 'segments', {SEG});
        fprintf('\n===== [%s plant, seed %d] delta_state gates 4-5 (free slots %s, negative control = locked slot 5) =====\n', pl{1}, seed, mat2str(free));
        res = verify_state_observability(L, cfgv);
        out.(pl{1}) = res;
        save(fullfile(od, sprintf('verify_obs_delta_%s_w%d.mat', pl{1}, win)), '-struct', 'res', '-v7.3');
    end
end
