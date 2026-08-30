function out = check_formB_echo_S_measure(opts)
%CHECK_FORMB_ECHO_S_MEASURE  Measure the y2 self-echo sensitivity S from closed-loop runs.
%   FORK OF nothing -- new instrument | PURPOSE: put a re-runnable measurement
%   behind the echo constants S_T/S_n that motion_control_law_formB_ws.m:356-380
%   computes from the Lyapunov model, which today have NO acceptance script in
%   the repo (the 2026-08-01 paired-forcing measurement was never committed).
%   EXPIRES: when this is promoted to test_script/integration/verify_formB_echo_S.m
%
%   METHOD (paired forced mismatch). S is defined as
%       S := dln Var(dw_r) / dln a_hat_ctrl   at  a_hat_ctrl = a_true,
%   so it is measurable directly: hold at a fixed height (positioning mode, where
%   the trajectory term of the control law vanishes IDENTICALLY, making the loop
%   exactly the 6-state model the constants come from), feed the control law a
%   deliberately wrong gain a_true*(1 +/- eps) via a_ctrl_override, and read the
%   log-slope of Var(dw_r). Same seed on both sides of the pair, so the chi^2
%   draws cancel. dw_r is taken from dx_r_out, which is the controller's own
%   (1-a_pd)(dw_m - dw_bar_m) -- the identical definition the Lyapunov output row
%   uses, so no re-derivation sits between model and measurement.
%
%   ARMS (each isolates one Lyapunov solve):
%     T   thermal ON, sensor noise OFF   -> S_T alone   (model +0.328123)
%     N   thermal OFF, sensor noise ON   -> S_n alone   (model -0.243727)
%     M22 both ON, far field h_bar 22.2  -> blend       (model (S_T a + S_n xi)/(a + xi))
%     M2  both ON, trough  h_bar 2.0     -> blend, where xi/a is 2x larger
%
%   INVARIANCE CHECK (free): S_T depends only on (lambda_c, a_pd, d) -- NOT on the
%   gain value -- so all three axes and both heights must return the same S_T.
%
%   COEFFICIENT CROSS-CHECK: arm M also reports the READOUT-level slope from
%   a_xm_out. Its ratio to the variance-level slope must equal (a + xi)/a, the
%   factor at issue in check_formB_echo_coefficient.
%
%   opts: .seeds (default 1:8) .T_sim (3.0) .eps_list ([0.05 0.10 0.20])
%         .save_fig (true) .verbose (true)

    if nargin < 1; opts = struct(); end
    if ~isfield(opts, 'seeds');    opts.seeds    = 1:8;               end
    if ~isfield(opts, 'T_sim');    opts.T_sim    = 3.0;               end
    if ~isfield(opts, 'eps_list'); opts.eps_list = [0.05 0.10 0.20];  end
    if ~isfield(opts, 'save_fig'); opts.save_fig = true;              end
    if ~isfield(opts, 'verbose');  opts.verbose  = true;              end
    if ~isfield(opts, 'arms_sel'); opts.arms_sel = {'T','N','M22','M2'}; end
    if ~isfield(opts, 'prev');     opts.prev     = [];                end

    [script_dir, ~, ~] = fileparts(mfilename('fullpath'));
    project_root = fileparts(fileparts(script_dir));
    addpath(genpath(fullfile(project_root, 'model')));
    addpath(genpath(fullfile(project_root, 'test_script')));

    T_SETTLE = 1.0;                 % [s] discard before variance window
    H_FAR    = 50.0;                % [um] h_bar = 22.2
    H_NEAR   = 4.5;                 % [um] h_bar = 2.0

    % model-side reference (recomputed here, not copied)
    S_T_MODEL = -0.243727; %#ok<NASGU>  placeholder overwritten below
    [S_T_MODEL, S_n_MODEL] = local_model_S(0.7, 0.05);

    arms = struct( ...
        'tag',     {'T',       'N',       'M22',    'M2'}, ...
        'thermal', {true,      false,     true,     true}, ...
        'noise',   {false,     true,      true,     true}, ...
        'h_init',  {H_FAR,     H_FAR,     H_FAR,    H_NEAR}, ...
        'eps',     {opts.eps_list, 0.10,  0.10,     0.10});

    if isempty(opts.prev)
        out = struct('arms', {{}}, 'S_T_model', S_T_MODEL, 'S_n_model', S_n_MODEL, 'opts', opts);
    else
        out = opts.prev;   % accumulate across calls (arms run in batches)
    end

    for ia = 1:numel(arms)
        A = arms(ia);
        if ~any(strcmp(A.tag, opts.arms_sel)); continue; end
        res = struct('tag', A.tag, 'eps', A.eps, 'h_init', A.h_init, ...
                     'S', [], 'S_sem', [], 'S_read', [], 'S_read_sem', [], ...
                     'a_bar', [], 'xi_bar', [], 'V0', [], 'Y0', [], 'ratio_exact', [], 'a_true', []);
        [ctr, abar, xibar] = local_center(A, opts, T_SETTLE);
        res.a_bar = abar;  res.xi_bar = xibar;
        res.V0 = ctr.V0;  res.Y0 = ctr.Y0;  res.ratio_exact = ctr.ratio_exact;
        res.a_true = ctr.a_true;
        for ie = 1:numel(A.eps)
            ep = A.eps(ie);
            [S_seed, Sr_seed] = local_measure(A, ep, opts, T_SETTLE);
            res.S(ie, :)      = mean(S_seed, 1);
            res.S_sem(ie, :)  = std(S_seed, 0, 1) / sqrt(size(S_seed, 1));
            res.S_read(ie, :) = mean(Sr_seed, 1, 'omitnan');
            res.S_read_sem(ie, :) = std(Sr_seed, 0, 1, 'omitnan') / sqrt(size(Sr_seed, 1));
            if opts.verbose
                fprintf(['[%-3s] eps=%4.2f  h=%5.1f um  S(x,y,z) = ' ...
                         '%+.4f+-%.4f  %+.4f+-%.4f  %+.4f+-%.4f\n'], ...
                        A.tag, ep, A.h_init, ...
                        res.S(ie,1), res.S_sem(ie,1), res.S(ie,2), res.S_sem(ie,2), ...
                        res.S(ie,3), res.S_sem(ie,3));
            end
        end
        out.arms{end+1} = res;
    end

    local_report(out, S_T_MODEL, S_n_MODEL);
    if opts.save_fig
        out.fig = local_plot(out, project_root, S_T_MODEL, S_n_MODEL);
    end
end


function [cfg, a_true, a_bar_nom, xi_bar, C_n, s2n, R] = local_setup(A, T_sim)
%LOCAL_SETUP  Scenario config plus the true gain and sensor floor at the height.
    cfg = local_hold_cfg(A, T_sim);
    P   = calc_simulation_params(cfg);  Pv = P.Value;
    R   = Pv.common.R;
    a_nom = Pv.common.Ts / Pv.common.gamma_N;          % [um/pN] far-field
    [c_par, c_perp] = calc_correction_functions(A.h_init / R);
    a_true = [a_nom / c_par; a_nom / c_par; a_nom / c_perp];

    a_o     = Pv.ctrl.Ts / (Pv.ctrl.gamma * R);
    kappa_T = 4 * (Pv.ctrl.k_B * Pv.ctrl.T / R) * a_o;
    s2n     = (cfg.meas_noise_std(:)).^2 / R^2;
    C_dpmr  = 3.160954;  C_n = 1.109329;               % g=1 values, self-checked elsewhere
    xi_bar  = (C_n / C_dpmr) * s2n / kappa_T;
    a_bar_nom = a_true(:) / (a_o * R);
end


function [S_seed, Sread_seed] = local_measure(A, ep, opts, T_SETTLE)
%LOCAL_MEASURE  Paired +/-eps forced-mismatch runs; returns per-seed log-slopes.
    [cfg, a_true] = local_setup(A, opts.T_sim);
    ns = numel(opts.seeds);
    S_seed     = zeros(ns, 3);
    Sread_seed = nan(ns, 3);
    for q = 1:ns
        sd = opts.seeds(q);
        sp = run_formB_ws(cfg, struct('seed', sd, 'a_ctrl_override', a_true * (1 + ep)));
        sm = run_formB_ws(cfg, struct('seed', sd, 'a_ctrl_override', a_true * (1 - ep)));
        w  = sp.tout > T_SETTLE;
        Vp = var(sp.dx_r_out(w, :), 0, 1);
        Vm = var(sm.dx_r_out(w, :), 0, 1);
        S_seed(q, :) = (log(Vp) - log(Vm)) / (2 * ep);
        Yp = mean(sp.a_xm_out(w, :), 1);
        Ym = mean(sm.a_xm_out(w, :), 1);
        ok = (Yp > 0) & (Ym > 0);
        Sread_seed(q, ok) = (log(Yp(ok)) - log(Ym(ok))) / (2 * ep);
    end
end


function [ctr, a_bar_nom, xi_bar] = local_center(A, opts, T_SETTLE)
%LOCAL_CENTER  eps = 0 runs: the LEVEL check the coefficient argument rests on.
%   The readout/variance slope ratio is algebraically sigma2/(sigma2 - C_n*s2n)
%   -- exact, independent of whether the loop sits at the nominal variance
%   level. Computing it from the MEASURED level removes that assumption; the
%   nominal prediction (a+xi)/a is reported alongside for comparison.
    [cfg, a_true, a_bar_nom, xi_bar, C_n, s2n, R] = local_setup(A, opts.T_sim);
    ns = numel(opts.seeds);
    V0 = zeros(ns, 3);  Y0 = zeros(ns, 3);
    for q = 1:ns
        s0 = run_formB_ws(cfg, struct('seed', opts.seeds(q), 'a_ctrl_override', a_true));
        w  = s0.tout > T_SETTLE;
        V0(q, :) = var(s0.dx_r_out(w, :), 0, 1) / R^2;      % normalized [-]
        Y0(q, :) = mean(s0.a_xm_out(w, :), 1);              % [um/pN]
    end
    ctr = struct();
    ctr.V0 = mean(V0, 1);
    ctr.Y0 = mean(Y0, 1);
    ctr.a_true = a_true(:).';
    ctr.ratio_exact = ctr.V0 ./ (ctr.V0 - (C_n * s2n(:)).');
end


function cfg = local_hold_cfg(A, T_sim)
%LOCAL_HOLD_CFG  Pure hold at h_init. In positioning mode the control law's
%   trajectory term is identically zero, so the loop IS the model's hold loop.
    cfg = user_config();
    cfg.trajectory_type   = 'positioning';
    cfg.h_init            = A.h_init;
    cfg.T_sim             = T_sim;
    cfg.ctrl_enable       = true;
    cfg.thermal_enable    = A.thermal;
    cfg.meas_noise_enable = A.noise;
    cfg.meas_noise_std    = [0.00062; 0.00057; 0.00331];   % [um] per axis
    cfg.lambda_c          = 0.7;
    cfg.a_pd              = 0.05;
    cfg.a_cov             = 0.05;
    cfg.h_min             = 1.1 * 2.25;
    cfg.h_bar_safe        = 1.5;
end


function [S_T, S_n] = local_model_S(lambda_c, a_pd)
%LOCAL_MODEL_S  The controller's own init computation, recomputed here.
    ep = 1e-4;
    S_T = (log(local_v(1/(1+ep),1,lambda_c,a_pd)) - log(local_v(1/(1-ep),1,lambda_c,a_pd)))/(2*ep);
    S_n = (log(local_v(1/(1+ep),2,lambda_c,a_pd)) - log(local_v(1/(1-ep),2,lambda_c,a_pd)))/(2*ep);
end


function v = local_v(gE, iN, lambda_c, a_pd)
    alE = 1 - lambda_c;
    AE = zeros(6); BqE = zeros(6,1); BnE = zeros(6,1);
    AE(1,1)=1; AE(1,3)=-gE*alE; AE(1,4)=-gE*alE; AE(1,5)=-gE*alE;
    BnE(1)=-gE*alE; BqE(1)=1;
    AE(2,1)=1; AE(3,2)=1;
    AE(4,3)=-alE; AE(4,4)=-alE; AE(4,5)=-alE; BnE(4)=-alE;
    AE(5,4)=1; AE(6,3)=a_pd; AE(6,6)=1-a_pd; BnE(6)=a_pd;
    if iN == 1; QE = BqE*BqE.'; extraE = 0; else; QE = BnE*BnE.'; extraE = (1-a_pd)^2; end
    XE = reshape((eye(36) - kron(AE,AE)) \ QE(:), 6, 6);
    cE = zeros(1,6); cE(3) = 1-a_pd; cE(6) = -(1-a_pd);
    v = cE*XE*cE.' + extraE;
end


function local_report(out, S_T, S_n)
    fprintf('\n===== model side =====\n');
    fprintf('S_T = %+.6f   S_n = %+.6f\n', S_T, S_n);
    fprintf('\n===== arm verdicts (z axis unless noted) =====\n');
    for k = 1:numel(out.arms)
        r = out.arms{k};
        for ie = 1:numel(r.eps)
            switch r.tag
                case 'T';  pred = S_T * [1 1 1];
                case 'N';  pred = S_n * [1 1 1];
                otherwise
                    pred = (S_T * r.a_bar(:).' + S_n * r.xi_bar(:).') ./ (r.a_bar(:).' + r.xi_bar(:).');
            end
            for ax = [1 3]
                dev = r.S(ie, ax) - pred(ax);
                nsig = abs(dev) / max(r.S_sem(ie, ax), eps);
                fprintf('%-4s eps=%4.2f ax=%d : meas %+.4f +- %.4f  model %+.4f  dev %+.4f (%.1f sigma) %s\n', ...
                        r.tag, r.eps(ie), ax, r.S(ie,ax), r.S_sem(ie,ax), pred(ax), dev, nsig, ...
                        ternary(nsig <= 2, 'PASS', 'CHECK'));
            end
        end
        if any(strcmp(r.tag, {'M22','M2'}))
            ratio_nom  = (r.a_bar(:).' + r.xi_bar(:).') ./ r.a_bar(:).';
            ratio_meas = r.S_read(1, :) ./ r.S(1, :);
            fprintf('%-4s ax3 level check: readout Y0 %.6g vs a_true %.6g  (rel %+.2f%%)\n', ...
                    r.tag, r.Y0(3), r.a_true(3), 100*(r.Y0(3)/r.a_true(3) - 1));
            fprintf(['%-4s ax3 readout/variance slope ratio: meas %.4f | ' ...
                     'exact-from-measured-level %.4f | nominal (a+xi)/a %.4f\n'], ...
                    r.tag, ratio_meas(3), r.ratio_exact(3), ratio_nom(3));
        end
    end
end


function s = ternary(c, a, b)
    if c; s = a; else; s = b; end
end


function fpath = local_plot(out, project_root, S_T, S_n)
    COL_MODEL = [0.8 0 0]; COL_MEAS = [0.45 0.55 0.95]; COL_MEAS2 = [0 0.2 0.9];
    FS = 18; LFS = 13; AXLW = 2.0;
    out_dir = fullfile(project_root, 'test_results', 'echo_S');
    if ~exist(out_dir, 'dir'); mkdir(out_dir); end

    f = figure('Position', [80 80 1150 760], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    % --- panel 1: arm T, eps linearity, x and z ---
    rT = out.arms{1};
    nexttile; hold on;
    hm = yline(S_T, '-', 'Color', COL_MODEL, 'LineWidth', 2.0, ...
               'DisplayName', 'S_T model (Lyapunov)');
    h1 = errorbar(rT.eps, rT.S(:,1), rT.S_sem(:,1), 'o-', 'Color', COL_MEAS, ...
                  'MarkerFaceColor', COL_MEAS, 'LineWidth', 2.0, 'DisplayName', 'measured x');
    h2 = errorbar(rT.eps, rT.S(:,3), rT.S_sem(:,3), 's-', 'Color', COL_MEAS2, ...
                  'MarkerFaceColor', COL_MEAS2, 'LineWidth', 2.0, 'DisplayName', 'measured z');
    ylabel('S_T', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel('forced mismatch \epsilon', 'FontSize', FS, 'FontWeight', 'bold');
    legend([hm h1 h2], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
           'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;

    % --- panel 2: all arms on z, measured vs model ---
    nexttile; hold on;
    tags = {}; meas = []; msem = []; pred = [];
    for k = 1:numel(out.arms)
        r = out.arms{k};
        ie = find(abs(r.eps - 0.10) < 1e-9, 1);
        if isempty(ie); ie = 1; end
        switch r.tag
            case 'T'; p = S_T;
            case 'N'; p = S_n;
            otherwise
                p = (S_T*r.a_bar(3) + S_n*r.xi_bar(3)) / (r.a_bar(3) + r.xi_bar(3));
        end
        tags{end+1} = r.tag; %#ok<AGROW>
        meas(end+1) = r.S(ie,3); msem(end+1) = r.S_sem(ie,3); pred(end+1) = p; %#ok<AGROW>
    end
    xk = 1:numel(tags);
    hp = plot(xk, pred, 'd', 'Color', COL_MODEL, 'MarkerFaceColor', COL_MODEL, ...
              'MarkerSize', 12, 'LineStyle', 'none', 'DisplayName', 'model');
    hq = errorbar(xk, meas, msem, 'o', 'Color', COL_MEAS2, 'MarkerFaceColor', COL_MEAS2, ...
                  'MarkerSize', 9, 'LineWidth', 2.0, 'LineStyle', 'none', 'DisplayName', 'measured');
    yline(0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off');
    set(gca, 'XTick', xk, 'XTickLabel', tags, 'XLim', [0.5, numel(tags)+0.5]);
    ylabel('S  (z axis)', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel('arm', 'FontSize', FS, 'FontWeight', 'bold');
    legend([hp hq], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
           'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;

    fpath = fullfile(out_dir, 'fig_echo_S_measure.png');
    exportgraphics(f, fpath, 'Resolution', 150);
    close(f);
    fprintf('\nfigure -> %s\n', fpath);
end
