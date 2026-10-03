% PURPOSE (2026-09-21): gates 4-5 of the observability workflow plus the derivation-vs-code check for version 2 (flag plaw_c_slope,
%   0921_estb_v2_slope_c.tex): x = [dw1 dw2 dw3 a_w c], slot 7 inert (negative control). From the obs_dump log:
%     (1) derivation vs code, ratios that cancel the unknown per-step scalars M, F_dw, nabla, H2_scale:
%           F_e(4,5) / (F_e(4,3) - (1-lambda_c) a')  = -h/(p (1-c))      every step with commanded motion   (tex S5: -M/h^p over M p a'/h)
%           H2(5) / H2(3)                            = -h/(p (1-c))      every step with nabla != 0          (tex S6: nabla/h^p over -nabla p a'/h)
%         evaluated with an INDEPENDENT law at the logged (h_hat, c_hat) of the previous update and w_d[k+1] (obs_dump alignment, C71).
%     (2) gate 4: rank of the free block [1 2 3 4 5] along the true trajectory; negative control = inert slot 7 (must be Inf).
%     (3) gate 5: CRLB of c against its prior 0.5 per window and segment; a_w against its P44[0].
%   Plants plane / cell / sphere, plane init (c = +0.037, p = 2.042), canon deep, one seed.
%   Output: console + test_results/temp_plaw_walls/verify_obs_v2_<plant>_w<win>.mat | EXPIRES: with the version-2 line
function out = verify_obs_v2(seed, win, plants)
    if nargin < 1 || isempty(seed); seed = 7; end
    if nargin < 2 || isempty(win);  win = 500; end
    if nargin < 3 || isempty(plants); plants = {'plane', 'cell', 'sphere'}; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration')); addpath(here);
    od = fullfile(root, 'test_results', 'temp_plaw_walls');  if ~exist(od, 'dir'); mkdir(od); end
    cfg0 = canonical_scenario(0.05, 1.1, 'deep');  LAM = cfg0.lambda_c;  pc = physical_constants();  Ts = pc.Ts;
    SEG = {'hold0', 0, 0.5; 'descent', 0.5, 1.5; 'osc', 1.5, 3.5; 'hold', 3.5, 4.8};
    P0 = 2.042;  SIG = 0.5;  C_PLANE = 0.0372;
    law_ap = @(h, c) (1 - c) ./ max(h, 1).^P0;
    out = struct('seed', seed, 'win', win);
    for pl = plants
        cc = struct('ma2_aug', false, 'nw_mcorr', false, 'pred_mean2', false, 'pred_mean2_e4', false, 'law_exact_step', false, ...
                    'lock_b', false, 'gain_algebraic', false, 'amodel_fe', false, 'amodel_h', false, ...
                    'plaw_c_slope', true, 'plaw_p', P0, 'c_init', C_PLANE, 'Pf_c_std', SIG, 'ws0_perp', 0, 'Pf_w0_std', 0.5, 'Pf_a_floor', 0.01, ...
                    'obs_dump', true);
        o = struct('arm', 'best', 'ctrl_const_override', cc, 'scenario', 'deep', 'verbose', false, 'seeds', seed, 'log_P_full', false);
        switch pl{1}
            case 'cell';   o.plant_cperp = @cell_cperp_delta;
            case 'sphere'; o.plant_cperp = @sphere_cperp_fig11;
        end
        clear run_formC_b motion_control_law_formC_b;
        evalc('R = run_formC_b(o);');
        L = obs_dump('get');  S1 = R.runs{1};  wd = S1.h_bar_d_out(:);
        Lz = L([L.ax] == 3);  n = numel(Lz);
        rF = nan(n, 1);  rF_exp = nan(n, 1);  rH = nan(n, 1);  rH_exp = nan(n, 1);
        for i = 2:n
            k = Lz(i).k + 1;  xp = Lz(i - 1).x_upd;                   % obs_dump alignment: the step's w_d is h_bar_d_out(k+1) (C71)
            if k < 1 || k > numel(wd); continue; end
            h = max(wd(k) - xp(3), 1);  c = xp(5);  ap = law_ap(h, c);  r_exp = -h / (P0 * (1 - c));
            F = Lz(i).F;  d43 = F(4, 3) - (1 - LAM) * ap;
            if abs(d43) > 1e-9 && abs(F(4, 5)) > 1e-12; rF(i) = F(4, 5) / d43;  rF_exp(i) = r_exp; end
            H2 = Lz(i).H{2};
            if ~isempty(H2) && abs(H2(3)) > 1e-12 && abs(H2(5)) > 1e-12; rH(i) = H2(5) / H2(3);  rH_exp(i) = r_exp; end
        end
        eF = abs(rF - rF_exp) ./ max(abs(rF_exp), 1e-9);  eH = abs(rH - rH_exp) ./ max(abs(rH_exp), 1e-9);
        fprintf('\n===== [%s plant, seed %d] plaw_c_slope derivation vs code =====\n', pl{1}, seed);
        fprintf('  F_e(4,5)/(F_e(4,3) - (1-lc) a'') vs -h/(p(1-c)) : %d motion steps, max rel err %.2e (median %.2e) | F_e(4,4) - (1 + a'' F_dw): checked implicitly | F_e(5,5) = %.3f\n', ...
                nnz(isfinite(eF)), max(eF, [], 'omitnan'), median(eF, 'omitnan'), Lz(end).F(5, 5));
        fprintf('  H2(5)/H2(3) vs -h/(p(1-c))                        : %d steps with nabla, max rel err %.2e (median %.2e) | H2(4) = %.3f (must be 1 x scale)\n', ...
                nnz(isfinite(eH)), max(eH, [], 'omitnan'), median(eH, 'omitnan'), Lz(end).H{2}(4) / max(abs(Lz(end).H{2}(4)), eps));
        free = [1 2 3 4 5];  ps = [NaN NaN NaN NaN SIG];  lab = {'dw1', 'dw2', 'dw3', 'a_w', 'c'};
        cfgv = struct('axis', 3, 'free', free, 'neg_ctrl', 7, 'prior_std', ps, 'labels', {lab}, 'window', win, ...
                      't_end', cfg0.T_sim, 'segments', {SEG});
        fprintf('===== [%s plant] gates 4-5 (free slots %s, negative control = inert slot 7) =====\n', pl{1}, mat2str(free));
        res = verify_state_observability(L, cfgv);
        res.jac_check = struct('eF_max', max(eF, [], 'omitnan'), 'eH_max', max(eH, [], 'omitnan'), 'nF', nnz(isfinite(eF)), 'nH', nnz(isfinite(eH)));
        out.(pl{1}) = res;
        save(fullfile(od, sprintf('verify_obs_v2_%s_w%d.mat', pl{1}, win)), '-struct', 'res', '-v7.3');
    end
end
