% STATUS: SCRATCH -- 2026-10-13, motion_ctrl Section 9 (user): z-axis gain regulation to a constant a_d with the S8 estimator.
%   Outer law (run_formC_b opts.gain_reg): Delta x_d[k] = (1 - lambda_a)(a_d - a_hat[k-1]) / a_hat'[k-1], Delta p_d = R Delta x_d w_hat.
%   First case: plane wall, a_d = 0.1, lambda_a = 1 - Ts / tau_a with tau_a = 0.1 s, start parked at 22.2 R (h_init 50 um), no saturation,
%   no height floor. Estimator: S8 = formC_b plaw_dc_slope with c pinned at 0 (numerator 1), delta_init = -w_s-hat[0] = 0,
%   Pf_delta_std 0.3, plaw_delta_lim [-5, 5]. lambda_c 0.4. Output test_results/temp_1009_ws/gainreg_<plant>.mat.
function run_mc_s9_gainreg(plants, seeds, a_d, tau_a, T_sim, ws0, extra, tag, dith)
    if nargin < 1 || isempty(plants); plants = {'plane'}; end
    if nargin < 2 || isempty(seeds); seeds = 1; end
    if nargin < 3 || isempty(a_d); a_d = 0.1; end
    if nargin < 4 || isempty(tau_a); tau_a = 0.1; end
    if nargin < 5 || isempty(T_sim); T_sim = 1.5; end
    if nargin < 6 || isempty(ws0); ws0 = 0; end   % 10-14: estimator start w_s-hat[0] (plant wall at 0)
    if nargin < 7 || isempty(extra); extra = struct(); end   % 10-14: extra ctrl_const overrides (e.g. struct('fe_cmd_eval', true))
    if nargin < 8; tag = ''; end
    if nargin < 9; dith = []; end   % 10-14: run_formC_b opts.cmd_dither (known sine on the command), [] = off
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration')); addpath(here);
    od = fullfile(root, 'test_results', 'temp_1009_ws');
    pc = physical_constants();  Ts = pc.Ts;  R = pc.R;
    lam_a = 1 - Ts / tau_a;
    cc = struct('ma2_aug', false, 'nw_mcorr', false, 'pred_mean2', false, 'pred_mean2_e4', false, ...
                'law_exact_step', false, 'lock_b', false, 'b_ceil', 1.5, 'b_floor', 0.2, ...
                'gain_algebraic', false, 'amodel_fe', false, 'amodel_h', false, 'ws0_perp', 0, 'Pf_w0_std', 0, ...
                'Pf_a_floor', 0.002, 'plaw_seed_cross', true, 'plaw_dc_slope', true, 'plaw_p', 2, 'c_init', 0, 'Pf_c_std', 1e-6, ...
                'delta_init', -ws0, 'Pf_delta_std', 0.3, 'plaw_g_min', 0.5, 'plaw_delta_lim', [-5, 5]);
    fn = fieldnames(extra);  for i = 1:numel(fn); cc.(fn{i}) = extra.(fn{i}); end
    OV = struct('trajectory_type', 'positioning', 'h_init', 50, 'h_bottom', 50, 'amplitude', 0, 't_hold', 0, ...
                't_descend_override', 0, 'T_sim', T_sim, 'lambda_c', 0.4);
    for ip = 1:numel(plants)
        o = struct('arm', 'best', 'ctrl_const_override', cc, 'config_override', OV, 'scenario', 'deep', 'verbose', false, ...
                   'seeds', seeds, 'log_P_full', false, 'gain_reg', struct('a_d', a_d, 'lambda_a', lam_a), 'cmd_dither', dith);
        if strcmp(plants{ip}, 'cell'); o.plant_cperp = @cell_cperp_delta; end
        clear run_formC_b motion_control_law_formC_b;
        tic;  evalc('Rr = run_formC_b(o);');  tr = toc;
        ns = numel(Rr.runs);  S1 = Rr.runs{1};  n = numel(S1.h_bar_d_out);
        res = struct('ws0', ws0, 't', (0:n-1)' * Ts, 'a_d', a_d, 'lambda_a', lam_a, 'tau_a', tau_a, 'seeds', seeds, 'plant', plants{ip});
        [res.wd, res.h_true, res.a_true, res.a_hat, res.w_hat, res.dxm, res.ap_hat, res.ap_true, res.sPa, res.sPw, res.p_hat] = deal(zeros(n, ns));
        for q = 1:ns
            S = Rr.runs{q};  hb = S.h_bar_true_out(:);
            res.wd(:, q) = S.h_bar_d_out(:);  res.h_true(:, q) = hb;  res.a_hat(:, q) = S.a_bar_hat_out(:, 3);
            res.w_hat(:, q) = -S.ws_hat_out(:, 3);  res.dxm(:, q) = S.dh_m_out(:, 3);
            res.ap_hat(:, q) = S.a_prime_out(:, 3);  res.ap_true(:, q) = S.a_prime_true_out(:, 3);
            res.sPa(:, q) = S.P_a_out(:, 3) / S.a_nom;  res.p_hat(:, q) = S.b_hat_out(:, 3);  if isfield(S, 'sP_ws_out'); res.sPw(:, q) = S.sP_ws_out(:, 3); end   % 10-14: sqrt P_aa (a-bar units)   % same driver units; only the ratio kappa is read
            if strcmp(plants{ip}, 'cell'); res.a_true(:, q) = 1 ./ cell_cperp_delta(max(hb, 1.001));
            else; for i = 1:n; [~, cp] = calc_correction_functions(max(hb(i), 1.001)); res.a_true(i, q) = 1 / cp; end; end
        end
        save(fullfile(od, sprintf('gainreg_%s_tau%03d%s.mat', plants{ip}, round(1000 * tau_a), [local_wtag(ws0), local_ttag(T_sim), tag])), '-struct', 'res');
        e = res.a_true - a_d;  i63 = find(abs(mean(e, 2)) <= 0.37 * abs(mean(e(1, :))), 1);  t63 = NaN;  if ~isempty(i63); t63 = res.t(i63); end
        fprintf('%s ws0 %+.1f tau %.2f (%.0f s): lambda_a %.5f | min h_true %.4f | NaN %d | a_true end %.4f (a_d %.2f) | h_d end %.3f | t to 63%% %.3f s | last 0.5 s mean a_true - a_d %+.4f sd %.4f\n', ...
                plants{ip}, ws0, tau_a, tr, lam_a, min(res.h_true(:)), nnz(~isfinite(res.a_hat(:))), mean(res.a_true(end, :)), a_d, mean(res.wd(end, :)), t63, ...
                mean(mean(e(res.t > T_sim - 0.5, :))), mean(std(e(res.t > T_sim - 0.5, :), 0, 1)));
    end
end

function s = local_wtag(ws0)
    if ws0 == 0; s = ''; elseif ws0 < 0; s = sprintf('_wm%g', -ws0); else; s = sprintf('_wp%g', ws0); end
end

function s = local_ttag(T_sim)
    s = '';  if T_sim ~= 1.5; s = sprintf('_T%g', T_sim); end   % 10-14: longer runs keep their own file
end
