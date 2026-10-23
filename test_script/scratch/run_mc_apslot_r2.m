% STATUS: SCRATCH -- 2026-10-18 (user): a' as a state (formC_b ap_slot), raw y2 vs whitened y2, nothing else changed.
%   Arm = apP002F050 of run_mc_apslot_compare (a'[0] = 1/22.2^2, sqrt P'[0] 0.002, Q77 = P_fix^2 I7, sqrt P_fix 0.05, fe_cmd_eval on).
%   Only ctrl_const.y2_whiten differs (false: y2 = a_bar_m itself, IF(s); true: production, y2 = whitened increment, IF(1)).
%   Trajectory 'desc' (22.2 -> 1.10 R in 1 s, constant velocity, hold 0.2 s), plane wall, lambda_c 0.4, seeds 1:5.
%   Output test_results/temp_1009_ws/apr2_<raw|white>_plane_desc.mat.
%   Sanity: the 'white' arm must reproduce a_hat of apcmp_apP002F050_plane_desc.mat bit for bit.
function run_mc_apslot_r2(seeds, arms)
    if nargin < 1 || isempty(seeds); seeds = 1:5; end
    if nargin < 2 || isempty(arms); arms = {'raw', 'white'}; end   % 10-18: 'white_q0' = whitened, Q77 off (q77_adapt_P0 0)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration')); addpath(here);
    od = fullfile(root, 'test_results', 'temp_1009_ws');  pc = physical_constants();  Ts = pc.Ts;
    OV = struct('trajectory_type', 'ramp_descent', 'h_init', 50, 'h_bottom', 1.10 * pc.R, 't_hold', 0, ...
                't_descend_override', 1.0, 'T_sim', 1.2, 'lambda_c', 0.4);
    cc = struct('ma2_aug', false, 'nw_mcorr', false, 'pred_mean2', false, 'pred_mean2_e4', false, 'law_exact_step', false, ...
                'lock_b', false, 'b_ceil', 1.5, 'b_floor', 0.2, 'gain_algebraic', false, 'amodel_fe', false, 'amodel_h', false, ...
                'ws0_perp', 0, 'Pf_w0_std', 0, 'Pf_a_floor', 0.002, ...
                'plaw_dc_slope', true, 'plaw_p', 2, 'c_init', 0, 'Pf_c_std', 1e-6, 'ap_slot', true, 'fe_cmd_eval', true, ...
                'delta_init', 1 / 22.2^2, 'Pf_delta_std', 0.002, 'plaw_g_min', 0.5, 'plaw_delta_lim', [-1, 10], ...
                'plaw_seed_cross', false, 'q77_adapt_P0', 0.05);
    for ia = 1:numel(arms)
        c = cc;  c.y2_whiten = startsWith(arms{ia}, 'white');  if endsWith(arms{ia}, '_q0'); c.q77_adapt_P0 = 0; end
        if endsWith(arms{ia}, '_fin'); c.q77_adapt_P0 = 0.4;  c.q77_hscale = 1.1; end   % 10-18: final a'-state Q (10-16 apP002F400H110: sqrt P_fix 0.4 x (1.1/h_d)^2)
        if startsWith(arms{ia}, 'y2off'); c.y2_whiten = true;  c.y2_off = true; end   % 10-18: y1 only (discriminator)
        o = struct('arm', 'best', 'ctrl_const_override', c, 'config_override', OV, 'scenario', 'deep', 'verbose', false, ...
                   'seeds', seeds, 'log_P_full', false);
        clear run_formC_b motion_control_law_formC_b;
        tic;  evalc('Rr = run_formC_b(o);');  tr = toc;
        ns = numel(Rr.runs);  n = numel(Rr.runs{1}.h_bar_d_out);
        res = struct('t', (0:n-1)' * Ts, 'arm', arms{ia}, 'seeds', seeds, 'cc', c);
        [res.wd, res.h_true, res.a_true, res.ap_true, res.a_hat, res.ap_hat, res.sp44, res.da_y2, res.da_y1, ...
         res.dap_y2, res.dap_y1, res.nu2, res.nu1, res.R2, res.Q44, res.S2, res.S1, res.sp77] = deal(zeros(n, ns));
        for q = 1:ns
            S = Rr.runs{q};  hb = max(S.h_bar_true_out(:), 1.001);  an = S.a_nom;
            res.wd(:, q) = S.h_bar_d_out(:);  res.h_true(:, q) = hb;
            res.a_hat(:, q)  = S.a_bar_hat_out(:, 3);
            res.ap_hat(:, q) = S.a_prime_out(:, 3) / an;            % um/pN -> a-bar per R
            res.sp44(:, q)   = S.P_a_out(:, 3) / an;                % sqrt p44 in a-bar units
            res.nu2(:, q) = S.innov_y2_out(:, 3);  res.nu1(:, q) = S.innov_y1_out(:, 3);  res.R2(:, q) = S.R2_out(:, 3);  res.Q44(:, q) = S.Q44_out(:, 3);  res.S2(:, q) = S.S2_out(:, 3);  res.S1(:, q) = S.S1_out(:, 3);  res.sp77(:, q) = S.P_ws_out(:, 3);   % sqrt P of slot 7 (a')
            res.da_y2(:, q)  = S.K_a_y2_out(:, 3) .* S.innov_y2_out(:, 3);   % l42 e_am  (a-bar per step)
            res.da_y1(:, q)  = S.K_a_y1_out(:, 3) .* S.innov_y1_out(:, 3);   % l41 e_x1
            res.dap_y2(:, q) = S.dws_y2_out(:, 3);  res.dap_y1(:, q) = S.dws_y1_out(:, 3);   % slot 7 (a') updates
            for i = 1:n
                [~, cp] = calc_correction_functions(hb(i));  res.a_true(i, q) = 1 / cp;
                [~, cp1] = calc_correction_functions(hb(i) + 1e-4);  [~, cp0] = calc_correction_functions(max(hb(i) - 1e-4, 1.0005));
                res.ap_true(i, q) = (1 / cp1 - 1 / cp0) / (hb(i) + 1e-4 - max(hb(i) - 1e-4, 1.0005));
            end
        end
        save(fullfile(od, sprintf('apr2_%s_plane_desc.mat', arms{ia})), '-struct', 'res');
        fprintf('%-5s (%.0f s): min h %.4f | NaN %d | a_hat at floor %d | ap_hat[1] %.5f\n', arms{ia}, tr, min(res.h_true(:)), ...
                nnz(~isfinite(res.a_hat)), nnz(res.a_hat <= 0.05 + 1e-9), res.ap_hat(1, 1));
    end
    ref = fullfile(od, 'apcmp_apP002F050_plane_desc.mat');
    if exist(ref, 'file')
        A = load(ref);  W = load(fullfile(od, 'apr2_white_plane_desc.mat'));
        fprintf('sanity: white arm vs apcmp_apP002F050 a_hat isequal = %d (max diff %.2e)\n', isequal(A.a_hat, W.a_hat), max(abs(A.a_hat(:) - W.a_hat(:))));
    end
end
