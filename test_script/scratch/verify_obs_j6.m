% PURPOSE (2026-10-07, branch test/conly-meng-law): gates 4-5 of the observability workflow for the c-only slope law (f1p2:
%   x = [dw1 dw2 dw3 a_w c . p], slot 5 = c free, slot 7 = p pinned by its ~0 prior, slot 6 inert = negative control) under the
%   journal control law (control_law 'j6', 1007_conly_meng_law.tex) and, side by side, the current law ('eq17'); ramp_tri 1 Hz,
%   plane init, plants plane / cell / sphere, one seed. From the obs_dump log, j6 only:
%     (1) derivation vs code (tex S5, S7), ratios that cancel the per-step scalars:
%           F_e(3,3)                      = 1
%           F_e(3,4) * a_hat / Delta_w_d  = -1      (F_e(3,4) = -f_det = -Delta_w_d / a_hat, the PREVIOUS step's command and posterior)
%           F_e(4,3) / F_e(4,5)           = -2 (1-c)/h_hat
%           H2(5) / H2(3)                 = -h_hat / (2 (1-c))
%     (2) gate 4: rank of the free block [1 2 3 4 5] along the trajectory; negative control = inert slot 6 (CRLB must be Inf);
%     (3) gate 5: CRLB of c against its prior 0.6 per window and segment (descent 0-1 s, triangle 1-3 s).
%   Output: console + test_results/temp_conly_meng/verify_obs_j6_<plant>_<law>.mat.  FORK OF verify_obs_f2.m | EXPIRES: with this line
function out = verify_obs_j6(seed, win, plants, laws)
    if nargin < 1 || isempty(seed); seed = 1; end
    if nargin < 2 || isempty(win);  win = 500; end
    if nargin < 3 || isempty(plants); plants = {'plane', 'cell', 'sphere'}; end
    if nargin < 4 || isempty(laws); laws = {'eq17', 'j6'}; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration')); addpath(here);
    od = fullfile(root, 'test_results', 'temp_conly_meng');  if ~exist(od, 'dir'); mkdir(od); end
    pc = physical_constants();  R = pc.R;
    SEG = {'descent', 0, 1.0; 'triangle', 1.0, 3.0};
    SIG_C = 0.6;  C_PLANE = 0.0533;
    out = struct('seed', seed, 'win', win);
    for pl = plants
        for lw = laws
            cc = struct('ma2_aug', false, 'nw_mcorr', false, 'pred_mean2', false, 'pred_mean2_e4', false, 'law_exact_step', false, ...
                        'lock_b', false, 'gain_algebraic', false, 'amodel_fe', false, 'amodel_h', false, ...
                        'plaw_cp_slope', true, 'c_init', C_PLANE, 'Pf_c_std', SIG_C, 'p_init', 2, 'Pf_p_std', 1e-6, 'plaw_p_lim', [1.2, 4], ...
                        'ws0_perp', 0, 'Pf_w0_std', 0, 'Pf_a_floor', 0.002, 'obs_dump', true, 'control_law', lw{1});
            OV = struct('frequency', 1, 'n_cycles', 2);
            o = struct('arm', 'best', 'ctrl_const_override', cc, 'config_override', OV, 'scenario', 'ramp_tri', 'verbose', false, ...
                       'seeds', seed, 'log_P_full', false);
            switch pl{1}
                case 'cell';   o.plant_cperp = @cell_cperp_delta;
                case 'sphere'; o.plant_cperp = @sphere_cperp_fig11;
            end
            clear run_formC_b motion_control_law_formC_b;
            evalc('Rr = run_formC_b(o);');
            L = obs_dump('get');  S1 = Rr.runs{1};  wd = S1.h_bar_d_out(:);  Ts = pc.Ts;
            Lz = L([L.ax] == 3);  n = numel(Lz);
            fprintf('\n===== [%s plant | %s] %d records, T = %.2f s =====\n', pl{1}, lw{1}, n, n * Ts);
            if strcmp(lw{1}, 'j6')
                r33 = nan(n, 1);  r34 = nan(n, 1);  r43 = nan(n, 1);  rH = nan(n, 1);
                for i = 2:n
                    F = Lz(i).F;  H = Lz(i).H;  xp = Lz(i - 1).x_upd;  k = Lz(i).k + 1;     % obs_dump alignment: the step's w_d is h_bar_d_out(k+1) (verify_obs_f2)
                    if k + 1 > numel(wd) || k < 2; continue; end
                    h_hat = wd(k) - xp(3);  c_hat = xp(5);  a_hat = max(xp(4), 0.05);        % previous posterior at this step's w_d (code: h_v2 = w_bar_d - x_curr(3))
                    dwd = wd(k) - wd(k - 1);                                                    % the previous step's command increment
                    r33(i) = F(3, 3);
                    if abs(dwd) > 1e-9; r34(i) = F(3, 4) * a_hat / dwd; end
                    if abs(F(4, 5)) > 1e-12; r43(i) = F(4, 3) / F(4, 5) * h_hat / (-2 * (1 - c_hat)); end
                    H2 = H{2};
                    if ~isempty(H2) && abs(H2(3)) > 1e-12; rH(i) = H2(5) / H2(3) * (-2 * (1 - c_hat)) / h_hat; end
                end
                fprintf('  derivation vs code: F_e(3,3) = 1: max|dev| %.1e | F_e(3,4) a/Dw = -1: median %.4f (iqr %.1e, %d steps) | F_e(4,3)/F_e(4,5) = -2(1-c)/h: median %.4f (iqr %.1e) | H2(5)/H2(3) = -h/(2(1-c)): median %.4f (iqr %.1e)\n', ...
                        max(abs(r33 - 1), [], 'omitnan'), median(r34, 'omitnan'), local_iqr(r34), nnz(~isnan(r34)), median(r43, 'omitnan'), local_iqr(r43), median(rH, 'omitnan'), local_iqr(rH));
            end
            free = [1 2 3 4 5];  ps = [NaN NaN NaN NaN SIG_C];  lab = {'dw1', 'dw2', 'dw3', 'a_w', 'c'};
            cfgv = struct('axis', 3, 'free', free, 'neg_ctrl', 6, 'prior_std', ps, 'labels', {lab}, 'window', win, ...
                          'Ts', Ts, 't_end', n * Ts, 'segments', {SEG});
            res = verify_state_observability(L, cfgv);
            fn = fullfile(od, sprintf('verify_obs_j6_%s_%s.mat', pl{1}, lw{1}));  save(fn, 'res', 'cfgv');
            out.(pl{1}).(lw{1}) = res;
        end
    end
end

function v = local_iqr(x)
    x = sort(x(~isnan(x)));  n = numel(x);
    if n < 4; v = NaN; return; end
    v = x(ceil(0.75 * n)) - x(ceil(0.25 * n));
end
