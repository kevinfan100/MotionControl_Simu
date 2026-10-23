% PURPOSE (2026-09-21, user: "confirm the current derivation is right"): three checks of the advisor-law estimator (flag
%   plaw_c_state, version A: x = [dw1 dw2 dw3 (a derived) (slot 5 locked) . c]) from the code that actually runs:
%     (1) derivation vs code (derivation-workflow rule 4, step 2): rebuild the Jacobian RATIOS the tex predicts from the logged
%         state with an INDEPENDENT law implementation and compare with the F_e / H2 the controller built (obs_dump):
%           F_e(3,7) / (F_e(3,3) - lambda_c) = -(da/dc)/a'            every step with commanded motion (F_dw != 0)
%           H2(7) / H2(3)                     = -(da/dc)/a'            hold steps only (walk-back nabla = 0)
%         the unknown per-step scalars F_dw, H2_scale (1 - S) cancel in the ratios; lambda_c = 0.7 (canonical_scenario).
%     (2) gate 4: rank of the free slots [1 2 3 7] along the true trajectory, with the LOCKED slot 5 as the negative control.
%     (3) gate 5: CRLB of c against its prior 0.5 per window and per segment (hold0 / descent / osc / hold).
%   Plants plane / cell (cell_cperp_delta) / sphere (sphere_cperp_fig11), plane init (c = 0), canon deep, one seed.
%   Output: console + test_results/temp_plaw_walls/verify_obs_plaw_<plant>_w<win>.mat | EXPIRES: with the advisor-law test
function out = verify_obs_plaw(seed, win, plants)
    if nargin < 1 || isempty(seed); seed = 7; end
    if nargin < 2 || isempty(win);  win = 500; end
    if nargin < 3 || isempty(plants); plants = {'plane', 'cell', 'sphere'}; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration')); addpath(here);
    od = fullfile(root, 'test_results', 'temp_plaw_walls');  if ~exist(od, 'dir'); mkdir(od); end
    cfg0 = canonical_scenario(0.05, 1.1, 'deep');  LAM = cfg0.lambda_c;
    SEG = {'hold0', 0, 0.5; 'descent', 0.5, 1.5; 'osc', 1.5, 3.5; 'hold', 3.5, 4.8};
    P0 = 1.9666;  SIG = 0.5;
    law_ap  = @(h, c) (1 - c) ./ max(h, 1).^P0;                                  % independent implementation (fit_advisor_law_walls.m)
    law_dac = @(h, c) 1 ./ ((P0 - 1) * max(h, 1).^(P0 - 1));
    out = struct('seed', seed, 'win', win);
    for pl = plants
        cc = struct('ma2_aug', false, 'nw_mcorr', false, 'pred_mean2', false, 'pred_mean2_e4', false, 'law_exact_step', false, ...
                    'lock_b', true, 'b_ceil', 1.5, 'b_floor', 0.2, 'Pf_a_floor', 1e-5, 'plaw_c_state', true, 'gain_algebraic', true, ...
                    'amodel_fe', true, 'amodel_h', true, 'plaw_p', P0, 'c_init', 0, 'Pf_c_std', SIG, 'ws0_perp', 0, 'Pf_w0_std', 0, ...
                    'b_init', 8/9, 'Pf_b_std', 0, 'obs_dump', true);
        o = struct('arm', 'best', 'ctrl_const_override', cc, 'scenario', 'deep', 'verbose', false, 'seeds', seed, 'log_P_full', false);
        switch pl{1}
            case 'cell';   o.plant_cperp = @cell_cperp_delta;
            case 'sphere'; o.plant_cperp = @sphere_cperp_fig11;
        end
        clear run_formC_b motion_control_law_formC_b;
        evalc('R = run_formC_b(o);');
        L = obs_dump('get');  S1 = R.runs{1};  wd = S1.h_bar_d_out(:);  pc = physical_constants();  Ts = pc.Ts;
        % ---- (1) derivation vs code: ratios from the logged state
        Lz = L([L.ax] == 3);  n = numel(Lz);
        % the ratio -(da/dc)/a' = -h/((p-1)(1-c)) is linear in h, so a one-step misalignment between the logged w_d and the
        % step's own w_bar_d shows up as ~dh/h per step (0.1-0.4 % in the descent); scan the alignment shift of w_d (-1, 0, +1)
        % and report all three -- the derivation is confirmed when ONE shift gives machine precision.
        SH = [-1 0 1];  eF_sh = nan(1, numel(SH));  medF_sh = nan(1, numel(SH));
        rH = nan(n, 1);  rH_exp = nan(n, 1);  tk = nan(n, 1);
        for is = 1:numel(SH)
            rF = nan(n, 1);  rF_exp = nan(n, 1);
            for i = 2:n
                k = Lz(i).k + SH(is);  xp = Lz(i - 1).x_upd;          % the step's Jacobians are built at x_curr = previous x_upd and w_d[k]
                if k < 1 || k > numel(wd); continue; end
                h = wd(k) - xp(3) - 0;  c = xp(7);  tk(i) = (Lz(i).k - 1) * Ts;
                ap = law_ap(h, c);  dac = law_dac(h, c);
                F = Lz(i).F;  d33 = F(3, 3) - LAM;
                if abs(d33) > 1e-9; rF(i) = F(3, 7) / d33;  rF_exp(i) = -dac / ap; end
                if SH(is) == 0
                    H2 = Lz(i).H{2};
                    if ~isempty(H2) && tk(i) >= 3.5 && abs(H2(3)) > 1e-12; rH(i) = H2(7) / H2(3);  rH_exp(i) = -dac / ap; end
                end
            end
            eF = abs(rF - rF_exp) ./ max(abs(rF_exp), 1e-9);  eF_sh(is) = max(eF, [], 'omitnan');  medF_sh(is) = median(eF, 'omitnan');  nF = nnz(isfinite(eF));
        end
        eH = abs(rH - rH_exp) ./ max(abs(rH_exp), 1e-9);  [eF_best, ib] = min(eF_sh);  eF = eF_sh(2);
        fprintf('\n===== [%s plant, seed %d] plaw_c_state derivation vs code =====\n', pl{1}, seed);
        fprintf('  F_e(3,7)/(F_e(3,3)-lambda_c) vs -(da/dc)/a'' : %d motion steps | max rel err by w_d shift {-1, 0, +1} = %.2e / %.2e / %.2e (median %.2e / %.2e / %.2e) -> best shift %+d: %.2e | F_e(7,7) = %.3f\n', ...
                nF, eF_sh(1), eF_sh(2), eF_sh(3), medF_sh(1), medF_sh(2), medF_sh(3), SH(ib), eF_best, Lz(end).F(7, 7));
        fprintf('  H2(7)/H2(3) (hold, nabla = 0) vs -(da/dc)/a''  : %d hold steps with y2, max rel err %.2e (median %.2e) | y2 gated steps %d / %d\n', ...
                nnz(isfinite(eH)), max(eH, [], 'omitnan'), median(eH, 'omitnan'), nnz(arrayfun(@(s) isempty(s.H{2}), Lz)), n);
        % ---- (2)+(3) gates 4 and 5
        free = [1 2 3 7];  ps = [NaN NaN NaN SIG];  lab = {'dw1', 'dw2', 'dw3', 'c'};
        cfgv = struct('axis', 3, 'free', free, 'neg_ctrl', 5, 'prior_std', ps, 'labels', {lab}, 'window', win, ...
                      't_end', cfg0.T_sim, 'segments', {SEG});
        fprintf('===== [%s plant] gates 4-5 (free slots %s, negative control = locked slot 5) =====\n', pl{1}, mat2str(free));
        res = verify_state_observability(L, cfgv);
        res.jac_check = struct('eF_max_by_shift', eF_sh, 'eF_best', eF_best, 'best_shift', SH(ib), 'eH_max', max(eH, [], 'omitnan'), 'nF', nF, 'nH', nnz(isfinite(eH)));
        out.(pl{1}) = res;
        save(fullfile(od, sprintf('verify_obs_plaw_%s_w%d.mat', pl{1}, win)), '-struct', 'res', '-v7.3');
    end
end
