% STATUS: ACTIVE (scratch) | PURPOSE: TEST 2 -- on the Meng 10 s ramp, the
%   a_hat error of three arms side by side: production (b estimated),
%   b_true (law reads the true b, slot 5 locked; opts.b_true), and a'_true
%   (law reads the true slope outright; opts.ap_known). If the error and its
%   seed spread survive b_true but not a'_true, the slope path is where the
%   error enters; if they survive both, the y1 feedback path is.
%   The a'_true arm LOCKS slot 5 (lock_b) so b_hat cannot wander into P45
%   (2026-08-27: unlocked it did, sd 0.0042 vs 0.0018 locked).
%   CAVEAT (08-24): ap_known also removes the a_bar dependence of a' (A_a = 0),
%   so it changes more than one thing -- read it as the "everything about the
%   slope known" bound, not as a clean single-factor arm.
function out = run_arm_compare_btrue_aptrue(seeds, no_hold)
    if nargin < 1 || isempty(seeds); seeds = [7 11 23 42 101 777]; end
    if nargin < 2 || isempty(no_hold); no_hold = false; end   % true: stop 50 ms after arrival at the trough
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'apd_acov_meng'); pc = physical_constants();
    ov = struct('trajectory_type','osc','h_init',15.0,'h_bottom',2.5,'amplitude',0,'frequency',1,'n_cycles',1, ...
                't_hold',0.5,'t_descend_override',10.0,'T_sim',12.5,'h_min',1.1*pc.R);
    if no_hold; ov.T_sim = 10.55; end
    ARMS = { 'production (b estimated)', struct();
             'b_{true}  (law reads true b)', struct('b_true', true);
             'a''_{true} (true slope, b locked)', struct('ap_known', true, 'ctrl_const_override', struct('lock_b', true)) };
    nA = size(ARMS,1); O = cell(1,nA); NAMES = ARMS(:,1).'; W = cell(1,nA);
    for c = 1:nA
        o = ARMS{c,2}; o.arm = 'best'; o.seeds = seeds; o.config_override = ov;
        clear run_formC_b motion_control_law_formC_b;
        evalc("O{c} = run_formC_b(o);");
        W{c} = [0.5 5.5; 5.5 10.5; 11.5 12.5]; if no_hold; W{c} = W{c}(1:2,:); end
        fprintf('ran %s\n', NAMES{c});
    end
    tag = ''; if no_hold; tag = '_nohold'; end
    fig = fullfile(od, ['arm_compare_btrue_aptrue' tag '.png']);
    D = plot_seed_spread_cols(O, NAMES, fig, W, {'far half', 'near half', 'hold'});
    save(fullfile(od, ['arm_compare_btrue_aptrue' tag '.mat']), 'O', 'seeds', 'ARMS', '-v7.3');
    out = struct('D', {D}, 'file', fig);
end
