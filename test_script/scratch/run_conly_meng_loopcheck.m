% PURPOSE (2026-10-07, branch test/conly-meng-law): stage-3 loop check of the journal control law (control_law 'j6') against
%   1007_conly_meng_law.tex S7, with the gain the law uses LOCKED AT TRUTH (run_formC_b a_ctrl_override 'true') so only the
%   loop is on trial, not the estimator. Positioning at opts.heights (plane plant) for opts.T_hold s, both laws, paired seeds.
%   Per law x height (mean +- SEM over seeds, z axis, last T_hold - 1 s):
%     (a) lag-1 autocorrelation of the posterior dw3_hat          expect lambda_c = 0.70 (the mixed timing gives 0.73)
%     (b) Var(dw_m)/sigma2_n against the closed form of the law   j6: C_dpmr q + (1 + K1) r ; eq17: C_dpmr q + 2/(1+lc) r
%     (c) a_m reading / a_true - 1                                 expect ~0 for each law with its own constants; also the
%         j6 loop read with the OLD constants (offline EWMA of dx_r) -> the predicted +0.7 / +2.0 / +5.8 % bias
%     (d) y1 innovation: lag 1-3 autocorrelation and Var(innov)/mean(S1)   expect |rho| < 2/sqrt(N), ratio 1 +- 0.1
%   Figure: dx_m [nm] vs t, rows = heights, columns = laws, seed 1 (figure-style.md). Output test_results/temp_conly_meng/.
%   The f1p2 recipe mirrors run_plaw_walls.m local_ctrl_const (case 'f1p2'); keep them in step.
function out = run_conly_meng_loopcheck(opts)
    if nargin < 1; opts = struct(); end
    if ~isfield(opts, 'heights'); opts.heights = [22.2, 2.0, 1.10]; end
    if ~isfield(opts, 'T_hold');  opts.T_hold  = 3.0; end
    if ~isfield(opts, 'seeds');   opts.seeds   = 1:10; end
    if ~isfield(opts, 'laws');    opts.laws    = {'eq17', 'j6'}; end
    if ~isfield(opts, 'plot');    opts.plot    = true; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration')); addpath(here);
    od = fullfile(root, 'test_results', 'temp_conly_meng');  if ~exist(od, 'dir'); mkdir(od); end
    pc = physical_constants();  Ts = pc.Ts;  R = pc.R;  kBT = pc.k_B * pc.T;
    a_o = pc.Ts / (pc.gamma_N * pc.R);  kappa_T = 4 * kBT * a_o / R;       % normalization-boundary.md U0
    lc = 0.7;  apd = 0.05;  a_cov = 0.05;  sig_n = 0.00331;  r = (sig_n / R)^2;
    C_dpmr_raw = 2 + 1 / (1 - lc^2);
    cc_prod = build_eq17_6state_constants(struct('lambda_c', lc, 'sigma2_n_s', sig_n^2 * ones(3, 1), 'kBT', kBT, 'a_cov', a_cov, 'a_pd', apd, 'd', 2));
    % --- f1p2 recipe (run_plaw_walls local_ctrl_const, plane init) ---
    cc = struct('ma2_aug', false, 'nw_mcorr', false, 'pred_mean2', false, 'pred_mean2_e4', false, ...
                'law_exact_step', false, 'lock_b', false, 'b_ceil', 1.5, 'b_floor', 0.2, 'Pf_a_floor', 0.002);
    cc.gain_algebraic = false;  cc.amodel_fe = false;  cc.amodel_h = false;
    cc.ws0_perp = 0;  cc.Pf_w0_std = 0;
    cc.plaw_cp_slope = true;  cc.c_init = 0.0533;  cc.Pf_c_std = 0.6;
    cc.p_init = 2;  cc.Pf_p_std = 1e-6;  cc.plaw_p_lim = [1.2, 4];
    nH = numel(opts.heights);  nL = numel(opts.laws);  ns = numel(opts.seeds);
    out = struct('heights', opts.heights, 'laws', {opts.laws}, 'seeds', opts.seeds);
    M = nan(nL, nH, ns, 7);          % (a) rho1(dw3_hat)  (b) Var(dw_m)/r  (b') closed form  (c) a_m bias  (c') old-const bias  (d) rho1(innov)  (d') Var(innov)/S1
    rho3 = nan(nL, nH, ns, 3);
    TS = cell(nL, nH);
    for il = 1:nL
        law = opts.laws{il};  cci = cc;  cci.control_law = law;
        for ih = 1:nH
            h = opts.heights(ih);
            OV = struct('trajectory_type', 'positioning', 'h_init', h * R, 'T_sim', opts.T_hold, 'h_bottom', h * R, ...
                        'amplitude', 0, 't_hold', 0, 't_descend_override', 0);
            o = struct('arm', 'best', 'ctrl_const_override', cci, 'config_override', OV, 'scenario', 'deep', 'verbose', false, ...
                       'seeds', opts.seeds, 'log_P_full', false, 'a_ctrl_override', 'true');
            clear run_formC_b motion_control_law_formC_b;
            evalc('Rr = run_formC_b(o);');
            for q = 1:ns
                S = Rr.runs{q};  n = numel(S.p_m_out(:, 3));  t = (0:n-1)' * Ts;  win = t >= 1.0;
                a_true = S.a_true_out(:, 3) / S.a_nom;  ab = mean(a_true(win));          % true normalized gain at this height
                qv = kappa_T * ab;
                dwm = S.dh_m_out(:, 3) / R;                                              % dw_m = y1 [-]
                x3  = S.delta_x_hat_3_out(:, 3) / R;                                     % posterior dw3_hat [-]
                am  = S.a_xm_out(:, 3) / S.a_nom;                                        % a_m reading [-]
                nu  = S.innov_y1_out(:, 3);  S1 = S.S1_out(:, 3);
                dxr = S.dx_r_out(:, 3) / R;                                              % IIR residual the readout squares
                M(il, ih, q, 1) = local_acf(x3(win), 1);
                M(il, ih, q, 2) = var(dwm(win)) / r;
                m  = (qv + sqrt(qv^2 + 4 * qv * r)) / 2;  K1 = m / (m + r);
                if strcmp(law, 'j6'); M(il, ih, q, 3) = (C_dpmr_raw * qv + (1 + K1) * r) / r;
                else;                 M(il, ih, q, 3) = (C_dpmr_raw * qv + 2 / (1 + lc) * r) / r; end
                M(il, ih, q, 4) = mean(am(win)) / ab - 1;
                s2 = 0;  s2v = zeros(n, 1);                                              % offline EWMA of dx_r^2 -> old-constant reading
                for k = 1:n; s2 = (1 - a_cov) * s2 + a_cov * dxr(k)^2;  s2v(k) = s2; end
                am_old = (s2v - cc_prod.C_n * r) / (cc_prod.C_dpmr * kappa_T);
                M(il, ih, q, 5) = mean(am_old(win)) / ab - 1;
                M(il, ih, q, 6) = local_acf(nu(win), 1);  rho3(il, ih, q, :) = [local_acf(nu(win), 1), local_acf(nu(win), 2), local_acf(nu(win), 3)];
                M(il, ih, q, 7) = var(nu(win)) / mean(S1(win));
                if q == 1; TS{il, ih} = struct('t', t, 'dxm_nm', dwm * R * 1e3, 'K1', K1, 'ab', ab); end
            end
        end
    end
    out.M = M;  out.rho3 = rho3;  out.TS = TS;
    % --- report ---
    lab = {'(a) rho1(dw3_hat)   [0.70]', '(b) Var(dw_m)/r     [closed form]', '(b'') closed form', '(c) a_m/a_true - 1  [0]', ...
           '(c'') old consts  a_m/a_true - 1', '(d) rho1(innov y1) [0]', '(d'') Var(innov)/S1 [1]'};
    N = nnz((0:round(opts.T_hold / Ts) - 1) * Ts >= 1.0);
    fprintf('\nLOOP CHECK  gain locked at truth, plane, z axis, %d seeds, window t >= 1 s (N = %d, 2/sqrt(N) = %.3f)\n', ns, N, 2 / sqrt(N));
    fprintf('%-36s', '');  for il = 1:nL; for ih = 1:nH; fprintf('%-9s@%-5.4g', opts.laws{il}, opts.heights(ih)); end; end;  fprintf('\n');
    for j = 1:7
        fprintf('%-36s', lab{j});
        for il = 1:nL; for ih = 1:nH
            v = squeeze(M(il, ih, :, j));  fprintf('%+8.4f+-%.4f ', mean(v), std(v) / sqrt(ns));
        end; end
        fprintf('\n');
    end
    fprintf('%-36s', 'K1 at height');  for il = 1:nL; for ih = 1:nH; fprintf('%-15.4f', TS{il, ih}.K1); end; end; fprintf('\n');
    fprintf('%-36s', 'a_true at height');  for il = 1:nL; for ih = 1:nH; fprintf('%-15.4f', TS{il, ih}.ab); end; end; fprintf('\n');
    save(fullfile(od, 'loopcheck.mat'), '-struct', 'out');
    % --- figure: dx_m [nm] vs t, rows = heights, cols = laws, seed 1 ---
    if opts.plot
        fig = figure('Visible', 'off', 'Position', [100 100 1100 250 * nH]);
        for ih = 1:nH
            yl = 0;
            for il = 1:nL; yl = max(yl, max(abs(TS{il, ih}.dxm_nm))); end
            for il = 1:nL
                ax = subplot(nH, nL, (ih - 1) * nL + il);
                plot(TS{il, ih}.t, TS{il, ih}.dxm_nm, 'Color', [0.4 0.7 1.0]);  box on;  grid off;
                ylim([-yl, yl] * 1.05);  xlabel('t [s]');  ylabel('\deltax_m [nm]');
                legend(ax, sprintf('%s, w = %.4g R, seed 1', opts.laws{il}, opts.heights(ih)), 'Location', 'northoutside', 'Orientation', 'horizontal');
            end
        end
        fn = fullfile(od, 'loopcheck_dxm.png');  exportgraphics(fig, fn, 'Resolution', 150);  close(fig);
        fprintf('figure -> %s\n', fn);
    end
end

function r = local_acf(x, lag)
    x = x(:) - mean(x);  n = numel(x);
    r = (x(1:n-lag)' * x(1+lag:n)) / (x' * x);
end
