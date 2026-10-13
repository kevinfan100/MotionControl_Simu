% STATUS: SCRATCH -- 2026-10-11, runs for 1009_conly_wall_position.tex: the c-only estimator of motion_ctrl Section 6 with the
%   wall position w_s added as a constant state. Settings of motion_ctrl Section 6: lambda_c 0.4, c-hat[0] = 0, sqrt(p55[0]) = 0.10,
%   q55 = P_fix^2 I (q55_adapt 'ratio', sqrt(P_fix) = 0.15), y2 whitening and echo factor ON, ma2_aug / nw_mcorr OFF.
%   scen 'fixtop' : parked at 22.2 R (h_init 50 um), 2 s, no motion                                  (user 10-11)
%   scen 'fixbot' : parked at 1.10 R, 2 s, no motion                                                 (user 10-11)
%   scen 'desc'   : 22.2 -> 1.10 R at constant velocity in 1 s (the descent of ramp_tri), then parked at 1.10 R for 0.2 s (user 10-11; hold shortened from 1 s, not the focus)
%   scen 'tri10'  : 1 s descent, triangle 1.10 <-> 10 R at 2 Hz for 2 s (motion_ctrl Fig. 4.3 / 6.3 / 6.4)
%   arm 'ws'     : run_plaw_walls law 'f2' ctrl_const (slope (1-c)/(h + delta)^2, slot 7 = delta = -w_s), delta-hat[0] = 0
%                  (plane init, not the band fit 0.104), sqrt(P[0]) on delta 0.64 (run_plaw_walls f2 default), q77 = 0
%   arm 'conly'  : run_plaw_walls law 'f1p2' ctrl_const (slope (1-c)/h^2, slot 7 = p pinned at 2)       = motion_ctrl Section 6
%   arm 'wslock' : 'ws' with the delta prior 1e-6 (instrument check: must land on 'conly')
%   arm 'wsnocross' : 'ws' with the seed cross terms P45[0], P47[0] zeroed (plaw_seed_cross false; discriminator for the fixed-bottom w_s move)
%   Checks 10-11 on 'tri10': 'conly' reproduces the cached Section 6 runs f1p2_<plant>_PI_mcs5_tri_ratio_rtri_f2_top1000_lc040.mat
%   bit for bit; 'wslock' vs 'conly' max |d a-hat| 1.3e-11.
%   Output test_results/temp_1009_ws/<scen>_<arm>_<plant>.mat (fields as run_plaw_walls local_collect, plus w_hat = -delta-hat,
%   sP_w, q55 = Q55 added per step, so the information on c of a step is I = q55 / P_fix^2).
function run_1009_ws(scens, seeds, plants, arms)
    if nargin < 1 || isempty(scens); scens = {'fixtop', 'fixbot', 'desc'}; end
    if nargin < 2 || isempty(seeds); seeds = 1; end
    if nargin < 3 || isempty(plants); plants = {'plane', 'cell'}; end
    if nargin < 4 || isempty(arms); arms = {'ws'}; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration')); addpath(here);
    od = fullfile(root, 'test_results', 'temp_1009_ws');  if ~exist(od, 'dir'); mkdir(od); end
    pc = physical_constants();  Ts = pc.Ts;  R = pc.R;
    H_TOP = 22.2;  H_BOT = 1.10;  LAMC = 0.4;  SIG_C = 0.10;  SIG_D = 0.64;  P_FIX = 0.15;
    for is = 1:numel(scens)
        switch scens{is}
            case {'fixtop', 'fixbot'}                                   % run_plaw_walls kind 'fixed' (positioning), lambda_c added
                hf = H_TOP;  if strcmp(scens{is}, 'fixbot'); hf = H_BOT; end
                if hf == H_TOP; hf = 50 / R; end                        % the descent's start height (h_init 50 um)
                OV = struct('trajectory_type', 'positioning', 'h_init', hf * R, 'T_sim', 2, 'h_bottom', hf * R, 'amplitude', 0, ...
                            't_hold', 0, 't_descend_override', 0, 'lambda_c', LAMC);
                scen = 'deep';
            case 'desc'
                OV = struct('trajectory_type', 'ramp_descent', 'h_init', 50, 'h_bottom', H_BOT * R, 't_hold', 0, 't_descend_override', 1.0, ...
                            'T_sim', 1.2, 'amplitude', 0, 'frequency', 1, 'n_cycles', 1, 'lambda_c', LAMC);
                scen = 'deep';
            case 'tri10'
                OV = struct('frequency', 2, 'n_cycles', 4, 'amplitude', 0.5 * (10 - H_BOT) * R, 'lambda_c', LAMC);   % run_plaw_walls ramp_tri branch
                scen = 'ramp_tri';
            otherwise; error('unknown scenario %s', scens{is});
        end
        for ip = 1:numel(plants)
            switch plants{ip}
                case 'plane'; plant_cperp = [];
                case 'cell';  plant_cperp = @cell_cperp_delta;
                otherwise; error('unknown plant %s', plants{ip});
            end
            for ia = 1:numel(arms)
                cc = struct('ma2_aug', false, 'nw_mcorr', false, 'pred_mean2', false, 'pred_mean2_e4', false, ...
                            'law_exact_step', false, 'lock_b', false, 'b_ceil', 1.5, 'b_floor', 0.2, ...
                            'gain_algebraic', false, 'amodel_fe', false, 'amodel_h', false, 'ws0_perp', 0, 'Pf_w0_std', 0, ...
                            'c_init', 0, 'Pf_c_std', SIG_C, 'Pf_a_floor', 0.002, 'plaw_seed_cross', true, ...
                            'q55_adapt', 'ratio', 'q55_adapt_P0', P_FIX);
                switch arms{ia}
                    case 'conly'
                        cc.plaw_cp_slope = true;  cc.p_init = 2;  cc.Pf_p_std = 1e-6;  cc.plaw_p_lim = [1.2, 4];
                    case {'ws', 'wslock', 'wsnocross'}
                        cc.plaw_dc_slope = true;  cc.plaw_p = 2;  cc.delta_init = 0;  cc.Pf_delta_std = SIG_D;
                        cc.plaw_g_min = 0.5;  cc.plaw_delta_lim = [-0.9, 3];
                        if strcmp(arms{ia}, 'wslock'); cc.Pf_delta_std = 1e-6; end
                        if strcmp(arms{ia}, 'wsnocross'); cc.plaw_seed_cross = false; end
                    otherwise; error('unknown arm %s', arms{ia});
                end
                o = struct('arm', 'best', 'ctrl_const_override', cc, 'config_override', OV, 'scenario', scen, 'verbose', false, ...
                           'seeds', seeds, 'log_P_full', false);
                if ~isempty(plant_cperp); o.plant_cperp = plant_cperp; end
                clear run_formC_b motion_control_law_formC_b;
                tic;  evalc('Rr = run_formC_b(o);');  tr = toc;
                res = local_collect(Rr, plant_cperp, Ts);
                res.scen = scens{is};  res.arm = arms{ia};  res.plant = plants{ip};  res.seeds = seeds;  res.cc = cc;  res.P_fix = P_FIX;
                save(fullfile(od, sprintf('%s_%s_%s.mat', scens{is}, arms{ia}, plants{ip})), '-struct', 'res');
                fprintf('%-6s %-6s %-6s (%.0f s) | min h_true %.4f | NaN %d | end: c_hat %+.3f sqrtP %.3f | w_hat %+.3f sqrtP %.3f\n', scens{is}, arms{ia}, ...
                        plants{ip}, tr, min(res.h_true(:)), nnz(~isfinite(res.a_hat(:))), res.c_hat(end, 1), res.sP_c(end, 1), res.w_hat(end, 1), res.sP_w(end, 1));
            end
        end
    end
end

function res = local_collect(Rr, plant_cperp, Ts)
    ns = numel(Rr.runs);  S1 = Rr.runs{1};  n = numel(S1.h_bar_d_out);
    res = struct('t', (0:n-1)' * Ts, 'wd', S1.h_bar_d_out(:));
    [res.h_true, res.a_true, res.a_hat, res.c_hat, res.sP_c, res.w_hat, res.sP_w, res.dxm, res.q55] = deal(zeros(n, ns));
    for q = 1:ns
        S = Rr.runs{q};  hb = S.h_bar_true_out(:);
        res.h_true(:, q) = hb;  res.a_true(:, q) = local_a_plant(hb, plant_cperp);  res.a_hat(:, q) = S.a_bar_hat_out(:, 3);
        res.c_hat(:, q) = S.b_hat_out(:, 3);  res.sP_c(:, q) = S.P_b_out(:, 3);          % slot 5 = c
        res.w_hat(:, q) = -S.ws_hat_out(:, 3);  res.sP_w(:, q) = S.P_ws_out(:, 3);       % slot 7 = delta = -w_s (f1p2: p, not used)
        res.dxm(:, q) = S.dh_m_out(:, 3);                                                % y1 = delta z_m [um]
        if isfield(S, 'q55a_q_out'); res.q55(:, q) = S.q55a_q_out(:, 3); end              % Q55 added at this step's predict = P_fix^2 I[k-1]
    end
    hd = max(res.wd, 1.0);  dh = 1e-4;                                                   % c(h_d) = 1 - h_d^2 a'_plant(h_d), wall at w_s = 0
    ap = (local_a_plant(hd + dh, plant_cperp) - local_a_plant(hd - dh, plant_cperp)) / (2 * dh);
    res.c_eff_d = 1 - hd.^2 .* ap;  res.ap_d = ap;
end

function a = local_a_plant(h, plant_cperp)
    h = max(h, 1.001);
    if isempty(plant_cperp)
        a = zeros(size(h));
        for i = 1:numel(h); [~, c] = calc_correction_functions(h(i)); a(i) = 1 / c; end
    else
        a = 1 ./ plant_cperp(h);
    end
end
