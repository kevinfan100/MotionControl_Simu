function out = run_meng_btrue_euler_vs_exact(seeds)
%RUN_MENG_BTRUE_EULER_VS_EXACT  The arm_four_evalpoint gains page, redone as a
%   before/after pair on the Meng 10 s ramp: both columns run the b_true oracle
%   with b evaluated at the particle's TRUE height (b_true_at = 'true'); the
%   only difference is the law integrator:
%     column 1  BEFORE  forward Euler        a+ = a + b(1-a)^2 M
%     column 2  AFTER   exact step           1/(1-a+) = 1/(1-a) + b M
%   Page layout = plot_seed_gain_cols (row 1 a_hat - a per seed + mean,
%   rows 2-5 gains l31 l32 l41 l42, row 6 trajectory).
% STATUS: ACTIVE | requested comparison (2026-08-31)

    if nargin < 1 || isempty(seeds); seeds = [7 11 23 42 101 777]; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'law_exact_step');
    if ~exist(od, 'dir'); mkdir(od); end
    pc = physical_constants();
    ov = struct('trajectory_type','osc','h_init',15.0,'h_bottom',2.5,'amplitude',0,'frequency',1,'n_cycles',1, ...
                't_hold',0.5,'t_descend_override',10.0,'T_sim',12.5,'h_min',1.1*pc.R);
    ARMS = { 'b_{true}(TRUE height)  Euler  (before)', struct(); ...
             'b_{true}(TRUE height)  exact step  (after)', struct('law_exact_step', true) };
    nA = size(ARMS,1);  O = cell(1,nA);  NAMES = ARMS(:,1).';  W = cell(1,nA);
    for c = 1:nA
        o = struct('arm', 'best', 'ap_src', 'post', 'seeds', seeds, 'verbose', false, ...
                   'b_true', true, 'b_true_at', 'true', 'config_override', ov, ...
                   'ctrl_const_override', ARMS{c,2});
        clear run_formC_b motion_control_law_formC_b;
        evalc("O{c} = run_formC_b(o);");
        W{c} = [0.5 5.5; 5.5 10.5; 11.5 12.5];
        fprintf('ran %s\n', NAMES{c});
    end
    fig = fullfile(od, 'arm_btrue_euler_vs_exact_meng_gains.png');
    D = plot_seed_gain_cols(O, NAMES, fig, W, {'far half', 'near half', 'hold'});
    save(fullfile(od, 'arm_btrue_euler_vs_exact_meng.mat'), 'O', 'seeds', 'ARMS', '-v7.3');
    out = struct('D', {D}, 'file', fig);
end
