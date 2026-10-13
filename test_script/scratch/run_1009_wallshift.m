% STATUS: SCRATCH -- 2026-10-12, c-only (motion_ctrl Section 6, unchanged) with a wall-position error in the PLANT only (user 10-12).
%   Plant: plane polynomial with its wall at w_s = DELTA (estimator coordinate), c_perp(h) = 1 / D_perp(1 / max(h - DELTA, 1));
%   the handle feeds the mobility, the thermal force (calc_thermal_force plant_cperp) and the logged truth. The estimator still assumes
%   the wall at 0 (law f1p2: slope (1-c)/h^2, q55 = P_fix^2 I, sqrt(P_fix) 0.15, c-hat[0] = 0, sqrt(p55[0]) 0.10, lambda_c 0.4).
%   Trajectory: commanded h_d 22.2 -> bottom in 1 s at constant velocity, then parked 0.2 s; bottom = 1.10 + max(DELTA, 0)
%   (DELTA > 0: the true gap at the bottom is 1.10; DELTA < 0: the commanded bottom stays 1.10, the true gap is 1.10 - DELTA).
%   Check: DELTA = 0 with the handle vs the production plane path (plant_cperp = []), same seed.
%   Output test_results/temp_1009_ws/wallshift_<tag>.mat (t, wd, h_true, a_true, a_hat, c_hat, sP_c, dxm, c_need, delta, bottom).
function run_1009_wallshift(deltas, seeds, pfix, do_check)
    if nargin < 1 || isempty(deltas); deltas = [0 -2 -4 2 4]; end
    if nargin < 2 || isempty(seeds); seeds = 1:10; end
    if nargin < 3 || isempty(pfix); pfix = 0.15; end              % sqrt(P_fix) of q55 = P_fix^2 I (10-12: per-case sweep, reference arm)
    if nargin < 4 || isempty(do_check); do_check = true; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration')); addpath(here);
    od = fullfile(root, 'test_results', 'temp_1009_ws');  if ~exist(od, 'dir'); mkdir(od); end
    pc = physical_constants();  Ts = pc.Ts;  R = pc.R;
    cc = struct('ma2_aug', false, 'nw_mcorr', false, 'pred_mean2', false, 'pred_mean2_e4', false, ...
                'law_exact_step', false, 'lock_b', false, 'b_ceil', 1.5, 'b_floor', 0.2, ...
                'gain_algebraic', false, 'amodel_fe', false, 'amodel_h', false, 'ws0_perp', 0, 'Pf_w0_std', 0, ...
                'c_init', 0, 'Pf_c_std', 0.10, 'Pf_a_floor', 0.002, 'plaw_seed_cross', true, ...
                'q55_adapt', 'ratio', 'q55_adapt_P0', pfix, ...
                'plaw_cp_slope', true, 'p_init', 2, 'Pf_p_std', 1e-6, 'plaw_p_lim', [1.2, 4]);       % = run_1009_ws arm 'conly'
    for id = 1:numel(deltas)
        DE = deltas(id);  bot = 1.10 + max(DE, 0);
        OV = struct('trajectory_type', 'ramp_descent', 'h_init', 50, 'h_bottom', bot * R, 't_hold', 0, 't_descend_override', 1.0, ...
                    'T_sim', 1.2, 'amplitude', 0, 'frequency', 1, 'n_cycles', 1, 'lambda_c', 0.4);
        pcp = @(h) local_cperp(h, DE);
        o = struct('arm', 'best', 'ctrl_const_override', cc, 'config_override', OV, 'scenario', 'deep', 'verbose', false, ...
                   'seeds', seeds, 'log_P_full', false, 'plant_cperp', pcp);
        clear run_formC_b motion_control_law_formC_b;
        tic;  evalc('Rr = run_formC_b(o);');  tr = toc;
        res = local_collect(Rr, pcp, Ts);  res.delta = DE;  res.bottom = bot;  res.seeds = seeds;
        tag = strrep(sprintf('%+g', DE), '+', 'p');  tag = strrep(tag, '-', 'm');
        res.pfix = pfix;  if abs(pfix - 0.15) > 1e-12; tag = sprintf('%s_pf%03d', tag, round(100 * pfix)); end
        save(fullfile(od, sprintf('wallshift_%s.mat', tag)), '-struct', 'res');
        gap = res.h_true - max(1 + DE, 1);                                  % distance from the true contact
        fprintf('DELTA %+g (bottom h_d %.2f, %.0f s): min true gap above contact %.4f R | NaN %d | a_hat <= 0.05: %d steps | c_hat range [%+.2f %+.2f]\n', ...
                DE, bot, tr, min(gap(:)), nnz(~isfinite(res.a_hat(:))), nnz(res.a_hat(:) <= 0.05 + 1e-9), min(res.c_hat(:)), max(res.c_hat(:)));
    end
    if ~do_check; return; end
    % instrument check: DELTA = 0 through the handle vs the production plane path, seed 1
    o.plant_cperp = [];  o = rmfield(o, 'plant_cperp');  o.seeds = seeds(1);
    o.config_override.h_bottom = 1.10 * R;
    clear run_formC_b motion_control_law_formC_b;  evalc('R0 = run_formC_b(o);');
    S0 = load(fullfile(od, 'wallshift_p0.mat'));
    d = max(abs(R0.runs{1}.a_bar_hat_out(:, 3) - S0.a_hat(:, 1)));
    fprintf('check DELTA = 0: handle vs production plane, max |d a_hat| = %.2e (seed %d)\n', d, seeds(1));
end

function cp = local_cperp(h, DE)
    u = 1 ./ max(h - DE, 1.0);
    a = 1 - (9/8) * u + (1/2) * u.^3 - (57/100) * u.^4 + (1/5) * u.^5 + (7/200) * u.^11 - (1/25) * u.^12;
    cp = 1 ./ a;
end

function res = local_collect(Rr, pcp, Ts)
    ns = numel(Rr.runs);  S1 = Rr.runs{1};  n = numel(S1.h_bar_d_out);
    res = struct('t', (0:n-1)' * Ts, 'wd', S1.h_bar_d_out(:));
    [res.h_true, res.a_true, res.a_hat, res.c_hat, res.sP_c, res.dxm] = deal(zeros(n, ns));
    for q = 1:ns
        S = Rr.runs{q};  hb = S.h_bar_true_out(:);
        res.h_true(:, q) = hb;  res.a_true(:, q) = 1 ./ pcp(hb);  res.a_hat(:, q) = S.a_bar_hat_out(:, 3);
        res.c_hat(:, q) = S.b_hat_out(:, 3);  res.sP_c(:, q) = S.P_b_out(:, 3);  res.dxm(:, q) = S.dh_m_out(:, 3);
    end
    hd = res.wd;  hd(1) = hd(2);  dh = 1e-4;                                % needed c in the estimator's coordinate: 1 - h_d^2 a'_plant(h_d)
    ap = (1 ./ pcp(hd + dh) - 1 ./ pcp(hd - dh)) / (2 * dh);
    res.c_need = 1 - hd.^2 .* ap;
end
