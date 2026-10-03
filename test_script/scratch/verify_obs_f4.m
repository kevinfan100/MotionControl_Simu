% PURPOSE (2026-09-23): gates 4-5 of the observability workflow plus the derivation-vs-code check for the (delta, p) coordinate of the
%   two-constant slope law (flag plaw_dp_slope, 0923_estb_f4_slope_dp.tex): x = [dw1 dw2 dw3 a_w p . delta] (slots 5 = p, 7 = delta), slot 6 inert
%   (negative control). From the obs_dump log:
%     (1) derivation vs code, ratios that cancel the unknown per-step scalars M, F_dw, nabla, H2_scale (tex S5, S7):
%           F_e(4,7) / F_e(4,5)                        = (p/g)/ln g     every step with commanded motion   (-(p a'/g) M over -a' ln g M)
%           F_e(4,7) / (F_e(4,3) - (1-lambda_c) a')    = -1             (the slope sees e_delta - e_dw3 only)
%           H2(7) / H2(3)                              = -1
%           H2(5) / H2(3)                              = -g ln g / p
%         evaluated with an INDEPENDENT law at the logged (g_hat, c_hat) of the previous update and w_d[k+1] (obs_dump alignment).
%     (2) gate 4: rank of the free block [1 2 3 4 5 7] along the true trajectory; negative control = inert slot 6 (must be Inf).
%     (3) gate 5: CRLB of c and delta against their priors (0.10, 0.64) per window and segment; a_w against its P44[0].
%   Plants plane / cell / sphere (plane-tail plant), plane init (delta, p) = (+0.059, 1.950), canon deep, one seed.
%   Output: console + test_results/temp_plaw_walls/verify_obs_f4_<plant>_w<win>.mat  FORK OF verify_obs_f2.m | EXPIRES: with the F4 line
function out = verify_obs_f4(seed, win, plants)
    if nargin < 1 || isempty(seed); seed = 7; end
    if nargin < 2 || isempty(win);  win = 500; end
    if nargin < 3 || isempty(plants); plants = {'plane', 'cell', 'sphere'}; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration')); addpath(here);
    od = fullfile(root, 'test_results', 'temp_plaw_walls');  if ~exist(od, 'dir'); mkdir(od); end
    cfg0 = canonical_scenario(0.05, 1.1, 'deep');  LAM = cfg0.lambda_c;
    SEG = {'hold0', 0, 0.5; 'descent', 0.5, 1.5; 'osc', 1.5, 3.5; 'hold', 3.5, 4.8};
    SIG_P = 0.10;  SIG_D = 0.64;  P_PLANE = 1.950;  D_PLANE = 0.059;  G_MIN = 0.5;
    law_ap = @(g, p) 1 ./ g.^p;
    out = struct('seed', seed, 'win', win);
    for pl = plants
        cc = struct('ma2_aug', false, 'nw_mcorr', false, 'pred_mean2', false, 'pred_mean2_e4', false, 'law_exact_step', false, ...
                    'lock_b', false, 'gain_algebraic', false, 'amodel_fe', false, 'amodel_h', false, ...
                    'plaw_dp_slope', true, 'p_init', P_PLANE, 'Pf_p_std', SIG_P, 'delta_init', D_PLANE, 'Pf_delta_std', SIG_D, 'plaw_p_lim', [1.2 4], ...
                    'ws0_perp', 0, 'Pf_w0_std', 0, 'Pf_a_floor', 0.002, 'plaw_g_min', G_MIN, 'obs_dump', true);
        o = struct('arm', 'best', 'ctrl_const_override', cc, 'scenario', 'deep', 'verbose', false, 'seeds', seed, 'log_P_full', false);
        switch pl{1}
            case 'cell';   o.plant_cperp = @cell_cperp_delta;
            case 'sphere'; o.plant_cperp = @sphere_cperp_fig11_ptail;
        end
        clear run_formC_b motion_control_law_formC_b;
        evalc('R = run_formC_b(o);');
        L = obs_dump('get');  S1 = R.runs{1};  wd = S1.h_bar_d_out(:);
        Lz = L([L.ax] == 3);  n = numel(Lz);
        r75 = nan(n, 1);  r75e = nan(n, 1);  r73 = nan(n, 1);  rH73 = nan(n, 1);  rH53 = nan(n, 1);  rH53e = nan(n, 1);
        for i = 2:n
            k = Lz(i).k + 1;  xp = Lz(i - 1).x_upd;                   % obs_dump alignment: the step's w_d is h_bar_d_out(k+1)
            if k < 1 || k > numel(wd); continue; end
            g = max(wd(k) - xp(3) + xp(7), G_MIN);  p = xp(5);  ap = law_ap(g, p);
            F = Lz(i).F;  d43 = F(4, 3) - (1 - LAM) * ap;
            if abs(F(4, 5)) > 1e-12; r75(i) = F(4, 7) / F(4, 5);  r75e(i) = (p / g) / log(g); end
            if abs(d43) > 1e-9;      r73(i) = F(4, 7) / d43; end
            H2 = Lz(i).H{2};
            if ~isempty(H2) && abs(H2(3)) > 1e-12
                rH73(i) = H2(7) / H2(3);  rH53(i) = H2(5) / H2(3);  rH53e(i) = -g * log(g) / p;
            end
        end
        e75 = abs(r75 - r75e) ./ max(abs(r75e), 1e-9);  e73 = abs(r73 + 1);  eH73 = abs(rH73 + 1);  eH53 = abs(rH53 - rH53e) ./ max(abs(rH53e), 1e-9);
        fprintf('\n===== [%s plant, seed %d] plaw_dp_slope derivation vs code =====\n', pl{1}, seed);
        fprintf('  F_e(4,7)/F_e(4,5) vs (p/g)/ln g                : %d motion steps, max rel err %.2e (median %.2e)\n', nnz(isfinite(e75)), max(e75, [], 'omitnan'), median(e75, 'omitnan'));
        fprintf('  F_e(4,7)/(F_e(4,3) - (1-lc) a'') vs -1         : %d motion steps, max abs err %.2e\n', nnz(isfinite(e73)), max(e73, [], 'omitnan'));
        fprintf('  H2(7)/H2(3) vs -1                              : %d steps with nabla, max abs err %.2e\n', nnz(isfinite(eH73)), max(eH73, [], 'omitnan'));
        fprintf('  H2(5)/H2(3) vs -g ln g/p                       : %d steps with nabla, max rel err %.2e | F_e(5,5) = %.3f F_e(7,7) = %.3f\n', ...
                nnz(isfinite(eH53)), max(eH53, [], 'omitnan'), Lz(end).F(5, 5), Lz(end).F(7, 7));
        free = [1 2 3 4 5 7];  ps = [NaN NaN NaN NaN SIG_P SIG_D];  lab = {'dw1', 'dw2', 'dw3', 'a_w', 'p', 'delta'};
        cfgv = struct('axis', 3, 'free', free, 'neg_ctrl', 6, 'prior_std', ps, 'labels', {lab}, 'window', win, ...
                      't_end', cfg0.T_sim, 'segments', {SEG});
        fprintf('===== [%s plant] gates 4-5 (free slots %s, negative control = inert slot 6) =====\n', pl{1}, mat2str(free));
        res = verify_state_observability(L, cfgv);
        res.jac_check = struct('e75_max', max(e75, [], 'omitnan'), 'e73_max', max(e73, [], 'omitnan'), 'eH73_max', max(eH73, [], 'omitnan'), 'eH53_max', max(eH53, [], 'omitnan'));
        out.(pl{1}) = res;
        save(fullfile(od, sprintf('verify_obs_f4_%s_w%d.mat', pl{1}, win)), '-struct', 'res', '-v7.3');
    end
end
