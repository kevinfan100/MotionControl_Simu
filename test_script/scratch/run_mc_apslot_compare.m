% STATUS: SCRATCH -- 2026-10-14 (user-approved test): slope a' as a STATE VALUE (formC_b ap_slot) vs the two function estimators,
%   same trajectory and walls. Trajectory = the Section 4/5 'tri' case: 22.2 -> 1.10 R in 1 s, then triangle 1.10 <-> 10 R at 2 Hz for
%   2 s (canonical_scenario 'ramp_tri', f 2, top 10 R); lambda_c 0.4; plane init on every wall; seeds 1:5.
%   Arms: 'conly' = Section 6 (c-only (1-c)/h^2, c[0] = 0, sqrt P_cc[0] 0.1, q55 'ratio' P_fix 0.15, as run_mc_s5_conly / run_plaw_walls f1p2);
%         's8'    = Section 8 (1/(h - w_s)^2, w_s-hat[0] = 0, prior 0.3);
%         'apNN'  = ap_slot: a'[0] = 1/22.2^2 (plane far field), sqrt P'[0] 0.05, Q77 = P_fix^2 I7 with sqrt P_fix = NN/100, fe_cmd_eval on.
%   Output test_results/temp_1009_ws/apcmp_<arm>_<plant>.mat (t, wd, h_true, a_true, a_hat, ap_hat).
function run_mc_apslot_compare(arms, plants, seeds, traj)
    if nargin < 1 || isempty(arms); arms = {'conly', 's8', 'ap05', 'ap20'}; end
    if nargin < 2 || isempty(plants); plants = {'plane', 'cell'}; end
    if nargin < 3 || isempty(seeds); seeds = 1:5; end
    if nargin < 4 || isempty(traj); traj = 'tri'; end   % 10-14: 'desc' = constant-velocity 22.2 -> 1.10 R in 1 s, hold 0.2 s (Section 8)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration')); addpath(here);
    od = fullfile(root, 'test_results', 'temp_1009_ws');  pc = physical_constants();  Ts = pc.Ts;  R = pc.R;
    OV = struct('frequency', 2, 'n_cycles', 4, 'amplitude', 0.5 * (10 - 1.10) * R, 'lambda_c', 0.4);  scen = 'ramp_tri';  ttag = '';
    if strcmp(traj, 'desc')
        OV = struct('trajectory_type', 'ramp_descent', 'h_init', 50, 'h_bottom', 1.10 * R, 't_hold', 0, 't_descend_override', 1.0, 'T_sim', 1.2, 'lambda_c', 0.4);
        scen = 'deep';  ttag = '_desc';
    end
    base = struct('ma2_aug', false, 'nw_mcorr', false, 'pred_mean2', false, 'pred_mean2_e4', false, 'law_exact_step', false, ...
                  'lock_b', false, 'b_ceil', 1.5, 'b_floor', 0.2, 'gain_algebraic', false, 'amodel_fe', false, 'amodel_h', false, ...
                  'ws0_perp', 0, 'Pf_w0_std', 0, 'Pf_a_floor', 0.002);
    for ia = 1:numel(arms)
        cc = base;
        switch arms{ia}
            case 'conly'
                cc.plaw_cp_slope = true;  cc.c_init = 0;  cc.Pf_c_std = 0.1;  cc.p_init = 2;  cc.Pf_p_std = 1e-6;  cc.plaw_p_lim = [1.2, 4];
                cc.plaw_seed_cross = true;  cc.q55_adapt = 'ratio';  cc.q55_adapt_P0 = 0.15;
            case 'f4fc'                                  % 10-15: (delta, p) law a' = 1/(h - w_s)^p, plane init w_s 0, p 2; prior w_s 0.3, p 0.1; fe_cmd_eval on
                cc.plaw_dp_slope = true;  cc.p_init = 2;  cc.Pf_p_std = 0.1;  cc.plaw_p_lim = [1.2, 4];  cc.delta_init = 0;  cc.Pf_delta_std = 0.3;
                cc.plaw_g_min = 0.5;  cc.plaw_delta_lim = [-5, 5];  cc.plaw_seed_cross = true;  cc.fe_cmd_eval = true;
            case {'f4fcQ05', 'f4fcQ10', 'f4fcQ20', 'f4fcQ40'}   % 10-15: f4fc + Q on slot 5 (= p): q55_adapt 'ratio', Q_pp = P_fix^2 I_k, sqrt P_fix = NN/100
                cc.plaw_dp_slope = true;  cc.p_init = 2;  cc.Pf_p_std = 0.1;  cc.plaw_p_lim = [1.2, 4];  cc.delta_init = 0;  cc.Pf_delta_std = 0.3;
                cc.plaw_g_min = 0.5;  cc.plaw_delta_lim = [-5, 5];  cc.plaw_seed_cross = true;  cc.fe_cmd_eval = true;
                cc.q55_adapt = 'ratio';  cc.q55_adapt_P0 = str2double(arms{ia}(6:end)) / 100;
            case 's8fc'                                  % 10-15: S8 with fe_cmd_eval (p pinned at 2)
                cc.plaw_dc_slope = true;  cc.plaw_p = 2;  cc.c_init = 0;  cc.Pf_c_std = 1e-6;  cc.delta_init = 0;  cc.Pf_delta_std = 0.3;
                cc.plaw_g_min = 0.5;  cc.plaw_delta_lim = [-5, 5];  cc.plaw_seed_cross = true;  cc.fe_cmd_eval = true;
            case 's8'                                    % plane init: w_s-hat[0] = 0, prior 0.3
                cc.plaw_dc_slope = true;  cc.plaw_p = 2;  cc.c_init = 0;  cc.Pf_c_std = 1e-6;  cc.delta_init = 0;  cc.Pf_delta_std = 0.3;
                cc.plaw_g_min = 0.5;  cc.plaw_delta_lim = [-5, 5];  cc.plaw_seed_cross = true;
            otherwise   % 'apNN' (prior 0.05, sqrt P_fix NN/100) or 'apP<prior x1000>F<P_fix x1000>' (10-14 sweep)
                cc.plaw_dc_slope = true;  cc.plaw_p = 2;  cc.c_init = 0;  cc.Pf_c_std = 1e-6;  cc.ap_slot = true;  cc.fe_cmd_eval = true;
                cc.delta_init = 1 / 22.2^2;  cc.Pf_delta_std = 0.05;  cc.plaw_g_min = 0.5;  cc.plaw_delta_lim = [-1, 10];  cc.plaw_seed_cross = false;
                tk = regexp(arms{ia}, '^apP(\d+)F(\d+)$', 'tokens', 'once');
                if isempty(tk); cc.q77_adapt_P0 = str2double(arms{ia}(3:end)) / 100;
                else; cc.Pf_delta_std = str2double(tk{1}) / 1000;  cc.q77_adapt_P0 = str2double(tk{2}) / 1000; end
        end
        for ip = 1:numel(plants)
            o = struct('arm', 'best', 'ctrl_const_override', cc, 'config_override', OV, 'scenario', scen, 'verbose', false, ...
                       'seeds', seeds, 'log_P_full', false);
            pcp = [];  if strcmp(plants{ip}, 'cell'); pcp = @cell_cperp_delta;  o.plant_cperp = pcp; end
            clear run_formC_b motion_control_law_formC_b;
            tic;  evalc('Rr = run_formC_b(o);');  tr = toc;
            ns = numel(Rr.runs);  n = numel(Rr.runs{1}.h_bar_d_out);
            res = struct('t', (0:n-1)' * Ts, 'arm', arms{ia}, 'plant', plants{ip}, 'seeds', seeds, 'cc', cc);
            [res.wd, res.h_true, res.a_true, res.a_hat, res.ap_hat, res.w_hat, res.p_hat] = deal(zeros(n, ns));
            for q = 1:ns
                S = Rr.runs{q};  hb = max(S.h_bar_true_out(:), 1.001);
                res.wd(:, q) = S.h_bar_d_out(:);  res.h_true(:, q) = hb;  res.a_hat(:, q) = S.a_bar_hat_out(:, 3);  res.ap_hat(:, q) = S.a_prime_out(:, 3);
                res.w_hat(:, q) = -S.ws_hat_out(:, 3);  res.p_hat(:, q) = S.b_hat_out(:, 3);   % 10-15: slot 7 (w_s = -delta), slot 5 (p or c)
                if isempty(pcp); for i = 1:n; [~, cp] = calc_correction_functions(hb(i)); res.a_true(i, q) = 1 / cp; end
                else; res.a_true(:, q) = 1 ./ pcp(hb); end
            end
            save(fullfile(od, sprintf('apcmp_%s_%s%s.mat', arms{ia}, plants{ip}, ttag)), '-struct', 'res');
            fprintf('%-6s %-5s (%.0f s): min h %.4f | NaN %d | a_hat at floor %d\n', arms{ia}, plants{ip}, tr, min(res.h_true(:)), nnz(~isfinite(res.a_hat)), nnz(res.a_hat <= 0.05 + 1e-9));
        end
    end
end
