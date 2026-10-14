% STATUS: SCRATCH -- 2026-10-13, motion_ctrl Section 8 (user 10-13): slope with unit numerator 1/(h - w_s)^2, the wall position w_s
%   estimated (c removed), q = 0 on w_s. Question: from a wrong initial w_s-hat, does it converge on a constant-velocity descent?
%   Code: formC_b plaw_dc_slope with c pinned (c_init 0, Pf_c_std 1e-6, no q55_adapt), slot 7 = delta = -w_s, delta_init = -w_s-hat[0],
%   Pf_delta_std = 1.0 (same prior for every start), plaw_delta_lim widened to [-5, 5] (default [-0.9, 3] = w_s in [-3, 0.9]).
%   Plants: plane (production polynomial, set wall position 0) and cell (cell_cperp_delta: the plane slid by 1.05 R).
%   Trajectory: h_d 22.2 -> 1.10 R at constant velocity in 1 s, parked 0.2 s; lambda_c 0.4.
%   Output test_results/temp_1009_ws/wsonly_<plant>_<init tag>.mat.
function run_1009_wsonly(inits, plants, seeds, opt)
    if nargin < 1 || isempty(inits); inits = [-2 -1 -0.5 0 0.5]; end
    if nargin < 2 || isempty(plants); plants = {'plane', 'cell'}; end
    if nargin < 3 || isempty(seeds); seeds = 1:10; end
    if nargin < 4; opt = struct(); end                                  % 10-13 descent-error arms: opt.sigw (prior sqrt P_w[0], default 1.0),
    if ~isfield(opt, 'sigw'); opt.sigw = 1.0; end                      %   opt.cnum (pinned c: numerator 1 - c, default 0 = numerator 1;
    if ~isfield(opt, 'cnum'); opt.cnum = 0; end                        %   -0.125 = numerator 9/8), opt.tag (file suffix)
    if ~isfield(opt, 'tag'); opt.tag = ''; end
    if ~isfield(opt, 'q77'); opt.q77 = 0; end
    if ~isfield(opt, 'gate'); opt.gate = false; end                    % 10-13: formC_b ws_gate_info (slot-7 information gate, ratio 1)                          % 10-13: sqrt(P_fix) of the slot-7 ratio Q (formC_b q77_adapt_P0), 0 = off
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration')); addpath(here);
    od = fullfile(root, 'test_results', 'temp_1009_ws');
    pc = physical_constants();  Ts = pc.Ts;  R = pc.R;
    OV = struct('trajectory_type', 'ramp_descent', 'h_init', 50, 'h_bottom', 1.10 * R, 't_hold', 0, 't_descend_override', 1.0, ...
                'T_sim', 1.2, 'amplitude', 0, 'frequency', 1, 'n_cycles', 1, 'lambda_c', 0.4);
    for ip = 1:numel(plants)
        pcp = [];  if strcmp(plants{ip}, 'cell'); pcp = @cell_cperp_delta; end
        for ii = 1:numel(inits)
            w0 = inits(ii);
            cc = struct('ma2_aug', false, 'nw_mcorr', false, 'pred_mean2', false, 'pred_mean2_e4', false, ...
                        'law_exact_step', false, 'lock_b', false, 'b_ceil', 1.5, 'b_floor', 0.2, ...
                        'gain_algebraic', false, 'amodel_fe', false, 'amodel_h', false, 'ws0_perp', 0, 'Pf_w0_std', 0, ...
                        'Pf_a_floor', 0.002, 'plaw_seed_cross', true, ...
                        'plaw_dc_slope', true, 'plaw_p', 2, 'c_init', opt.cnum, 'Pf_c_std', 1e-6, ...
                        'delta_init', -w0, 'Pf_delta_std', opt.sigw, 'plaw_g_min', 0.5, 'plaw_delta_lim', [-5, 5]);
            if opt.q77 > 0; cc.q77_adapt_P0 = opt.q77; end
            if opt.gate; cc.ws_gate_info = true;  cc.ws_gate_ratio = 1; end
            if isfield(opt, 'fecmd') && opt.fecmd; cc.fe_cmd_eval = true; end   % 10-14: formC_b fe_cmd_eval (P-predict Jacobian at the commanded point)
            o = struct('arm', 'best', 'ctrl_const_override', cc, 'config_override', OV, 'scenario', 'deep', 'verbose', false, ...
                       'seeds', seeds, 'log_P_full', false);
            if ~isempty(pcp); o.plant_cperp = pcp; end
            clear run_formC_b motion_control_law_formC_b;
            tic;  evalc('Rr = run_formC_b(o);');  tr = toc;
            res = local_collect(Rr, pcp, Ts);  res.w0 = w0;  res.plant = plants{ip};  res.seeds = seeds;  res.cc = cc;
            tag = strrep(strrep(sprintf('%+g', w0), '+', 'p'), '-', 'm');  tag = strrep(tag, '.', 'd');
            save(fullfile(od, sprintf('wsonly_%s_%s%s.mat', plants{ip}, tag, opt.tag)), '-struct', 'res');
            fprintf('%-5s%s w_s-hat[0] %+4.1f (%.0f s) | min h_true %.4f | NaN %d | a_hat<=0.05 %d | c_hat max|.| %.1e | w_s-hat end %+.3f +- %.3f, sqrtP %.3f | range [%+.2f %+.2f]\n', ...
                    plants{ip}, opt.tag, w0, tr, min(res.h_true(:)), nnz(~isfinite(res.a_hat(:))), nnz(res.a_hat(:) <= 0.05 + 1e-9), max(abs(res.c_hat(:))), ...
                    mean(res.w_hat(end, :)), std(res.w_hat(end, :)), mean(res.sP_w(end, :)), min(res.w_hat(:)), max(res.w_hat(:)));
        end
    end
end

function res = local_collect(Rr, pcp, Ts)
    ns = numel(Rr.runs);  S1 = Rr.runs{1};  n = numel(S1.h_bar_d_out);
    res = struct('t', (0:n-1)' * Ts, 'wd', S1.h_bar_d_out(:));
    [res.h_true, res.a_true, res.a_hat, res.c_hat, res.w_hat, res.sP_w, res.dxm] = deal(zeros(n, ns));
    for q = 1:ns
        S = Rr.runs{q};  hb = S.h_bar_true_out(:);
        res.h_true(:, q) = hb;  res.a_true(:, q) = local_a(hb, pcp);  res.a_hat(:, q) = S.a_bar_hat_out(:, 3);
        res.c_hat(:, q) = S.b_hat_out(:, 3);  res.w_hat(:, q) = -S.ws_hat_out(:, 3);  res.sP_w(:, q) = S.P_ws_out(:, 3);
        res.dxm(:, q) = S.dh_m_out(:, 3);
    end
end

function a = local_a(h, pcp)
    h = max(h, 1.001);
    if isempty(pcp)
        a = zeros(size(h));  for i = 1:numel(h); [~, c] = calc_correction_functions(h(i)); a(i) = 1 / c; end
    else
        a = 1 ./ pcp(h);
    end
end
