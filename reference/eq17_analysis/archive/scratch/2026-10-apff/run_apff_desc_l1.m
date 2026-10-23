% STATUS: SCRATCH -- 2026-10-16 (user): closed loop, Layer-1 first-order estimator (5state_noap production, eps 1e-5) with the gain
%   feed-forward a[k] = a[k-1] + a'*del_pd[k-1] (ctrl_const.ap_ff): 'off' (Layer 1), 'lpf' (low-passed difference of its own a_hat,
%   beta 2/51), 'true' (driver slope, upper bound), 'nominal' (10-20: law uses a_nom, estimator runs unused). Constant-velocity descent 22.2 -> 1.10 R in 1 s + 0.2 s hold, lambda_c 0.4
%   (the run_mc_apslot_compare 'desc' case). Cache test_results/temp_1009_ws/apff_<arm>_<wall>_s<seed>.mat.
%   check = true: re-runs 'off' seed 1 plane and compares with the cache written before ap_ff existed (bit-identical).
function run_apff_desc_l1(arms, walls, seeds, check)
    if nargin < 1 || isempty(arms); arms = {'off', 'lpf', 'true'}; end
    if nargin < 2 || isempty(walls); walls = {'plane', 'cell'}; end
    if nargin < 3 || isempty(seeds); seeds = 1:5; end
    if nargin < 4; check = false; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);
    addpath(genpath(fullfile(root, 'model')));  addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'temp_1009_ws');  pc = physical_constants();  a_nom = pc.Ts / pc.gamma_N;
    if check
        r0 = load(fullfile(od, 'apdiff_desc_l1_plane_s1.mat'));  r1 = local_run('off', 'plane', 1, pc, a_nom);
        fprintf('bit-identical check (ap_ff off vs pre-change cache): a_hat %d, h %d\n', isequal(r0.a_hat, r1.a_hat), isequal(r0.h, r1.h));
    end
    for ia = 1:numel(arms)
        for iw = 1:numel(walls)
            for s = seeds
                fn = fullfile(od, sprintf('apff_%s_%s_s%d.mat', arms{ia}, walls{iw}, s));
                if exist(fn, 'file'); continue; end
                r = local_run(arms{ia}, walls{iw}, s, pc, a_nom);  save(fn, '-struct', 'r');
            end
        end
    end
end

function r = local_run(arm, wall, s, pc, a_nom)
    cfg = canonical_scenario(0.05, 1.10, 'deep');  cfg.eq17_variant = '5state_noap';
    ov = struct('trajectory_type', 'ramp_descent', 'h_init', 50, 'h_bottom', 1.10 * pc.R, 't_hold', 0, ...
                't_descend_override', 1.0, 'T_sim', 1.2, 'lambda_c', 0.4);
    fo = fieldnames(ov);  for q = 1:numel(fo); cfg.(fo{q}) = ov.(fo{q}); end
    cc = struct('lf_mode', 'const', 'lambda_f', 1, 'gain_order', 1, 'seed_mode', 'true_h0', 'q44_r2_frac', 1e-5);
    if ~any(strcmp(arm, {'off', 'nominal'})); cc.ap_ff = arm; end
    if strcmp(arm, 'lpf'); cc.ap_ff_thr = 0.0066 * pc.R; end   % 10-16: a' kept while the low-passed commanded step < 0.0066 R/step (hold)
    o = struct('seed', s, 'collect_diag', true, 'ctrl_const_override', cc);
    if strcmp(wall, 'cell'); o.plant_cperp = @cell_cperp_delta; end
    if strcmp(arm, 'nominal'); o.a_ctrl_const = a_nom * ones(3, 1); end   % 10-20 deck: control law uses the nominal motion gain (a-hat = 1)
    clear motion_control_law_eq17_5state_noap;
    S = run_pure_simulation(cfg, o);
    r = struct('t', S.tout(:), 'a_true', S.a_true_out(:, 3) / a_nom, 'a_hat', S.diag.a_hat(:, 3) / a_nom, ...
               'h_d', S.p_d_out(:, 3) / pc.R, 'h', S.p_true_out(:, 3) / pc.R, 'dxm', S.diag.delta_x_m(:, 3));
    r.ap_hat = NaN(size(r.t));  if isfield(S.diag, 'a_prime_hat'); r.ap_hat = S.diag.a_prime_hat(:, 3) / a_nom * pc.R; end   % per R
end
