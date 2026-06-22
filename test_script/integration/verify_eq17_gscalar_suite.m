function out = verify_eq17_gscalar_suite(scenario, opts)
%VERIFY_EQ17_GSCALAR_SUITE  gscalar (pooled scalar-gain) vs 4-state baseline.
%
%   out = verify_eq17_gscalar_suite('pos_h50')   % static hold h_bar~22
%   out = verify_eq17_gscalar_suite('pos_h10')   % static hold h_bar~4.4
%   out = verify_eq17_gscalar_suite('osc_1hz')   % 1 Hz near-wall oscillation
%   out = verify_eq17_gscalar_suite('summary')   % read 3 cached metrics, bar chart
%
%   Mirrors verify_eq17_4state_suite, but the BASELINE is the 4-state controller
%   (production) and the candidate is 'gscalar'. Same 3 scenarios, same seeds,
%   identical pure-MATLAB noise per seed. Writes figures into
%   test_results/verify_gscalar/<scenario>/ and caches metrics.mat.
%
%   Headline (the legitimate "50%->5%" claim) uses the ACROSS-SEED noise metric
%   ahat_noise_rel = mean_steady( std_seed(a_hat) / a_true ): the deterministic
%   a_true oscillation cancels across seeds, leaving only noise-driven spread.
%   (The per-seed temporal rel std conflates genuine a_true swing with noise.)
%
%   Pass/fail gates (printed, not thrown -- this is dynamic verification):
%     G-A osc z   : ahat_noise_rel_z(gs) < 0.05  AND  < 4-state
%     G-B all     : |track(gs)-track(4)|/track(4) < 0.05  (all axes)
%     G-C all z   : |abias_z(gs)| < 0.03   (osc near-wall < 0.05)
%     G-D pos z   : ahat_std_rel_z(gs) < 0.05  (absolute)
%     G-F all     : s_hat within 1 +/- 0.05 by t_steady; sqrt(var_s) shrinking
%   G-E (flatness vs height) is the visual proof in fig_ahat_vs_height.
%
%   opts.seeds (default 1:6), opts.beta_s (default 0.002), opts.verbose.
%
%   See also: motion_control_law_eq17_gscalar, verify_eq17_4state_suite,
%             run_pure_simulation

    if nargin < 2; opts = struct(); end
    if ~isfield(opts, 'seeds');   opts.seeds  = 1:6;     end
    if ~isfield(opts, 'beta_s');  opts.beta_s = 0.002;   end
    if ~isfield(opts, 'verbose'); opts.verbose = true;   end

    here = fileparts(mfilename('fullpath'));
    proj = fullfile(here, '..', '..');
    addpath(genpath(proj));
    out_root = fullfile(proj, 'test_results', 'verify_gscalar');

    if strcmpi(scenario, 'summary')
        out = make_summary(out_root, opts);
        return;
    end

    sc = scenario_spec(scenario);
    if isfield(opts, 'T_sim'); sc.T_sim = opts.T_sim; end
    out_dir = fullfile(out_root, scenario);
    if ~exist(out_dir, 'dir'); mkdir(out_dir); end

    cfg4 = build_cfg(sc, '4state',  opts);
    cfgG = build_cfg(sc, 'gscalar', opts);
    A4 = run_variant(cfg4, opts.seeds, sc);
    AG = run_variant(cfgG, opts.seeds, sc);

    % --- figures (gscalar = red, 4-state baseline = blue) ---
    f1 = fig1_gain_estimation(AG, A4, sc, out_dir);
    f2 = fig2_tracking_error(AG, A4, sc, out_dir);
    f3 = fig3_trajectory(AG, sc, out_dir);
    fH = fig_ahat_vs_height(AG, A4, sc, out_dir);
    fS = fig_s_convergence(AG, sc, out_dir);
    figs = {f1, f2, f3, fH, fS};
    if sc.is_pos
        figs = [figs, {fig_var_ahat(AG, A4, sc, out_dir)}];
    else
        figs = [figs, {fig_track_var(AG, A4, sc, out_dir)}];
    end

    % --- gates ---
    G = eval_gates(AG, A4, sc);

    % --- cache metrics ---
    M = struct('scenario', scenario, 'label', sc.label, 'is_pos', sc.is_pos, ...
               'trackG', AG.track_std_nm, 'track4', A4.track_std_nm, ...
               'astdG',  AG.ahat_std_rel, 'astd4',  A4.ahat_std_rel, ...
               'noiseG', AG.ahat_noise_rel, 'noise4', A4.ahat_noise_rel, ...
               'abiasG', AG.ahat_bias,    'abias4', A4.ahat_bias, ...
               'alagG',  AG.ahat_lag_ms,  'alag4',  A4.ahat_lag_ms, ...
               'gates',  G);
    save(fullfile(out_dir, 'metrics.mat'), 'M');

    out = struct('scenario', scenario, 'metrics', M, 'figs', {figs}, 'gates', G);
    if opts.verbose
        print_report(scenario, sc, AG, A4, G, figs, opts);
    end
end


% ====================== scenario / config ======================

function sc = scenario_spec(scenario)
    switch lower(scenario)
        case 'pos_h50'
            sc = struct('name','pos_h50','label','positioning h=50 (h_bar~22)', ...
                        'is_pos',true,'h_init',50,'T_sim',5,'t_steady',1.0);
        case 'pos_h10'
            sc = struct('name','pos_h10','label','positioning h=10 (h_bar~4.4)', ...
                        'is_pos',true,'h_init',10,'T_sim',5,'t_steady',1.0);
        case 'osc_1hz'
            sc = struct('name','osc_1hz','label','osc 1Hz near-wall', ...
                        'is_pos',false,'h_init',50,'T_sim',4,'t_steady',1.5);
        otherwise
            error('verify_eq17_gscalar_suite:badScenario', ...
                  'scenario must be pos_h50 | pos_h10 | osc_1hz | summary');
    end
end


function cfg = build_cfg(sc, variant, opts)
    cfg = user_config();
    cfg.eq17_variant      = variant;
    cfg.h_init            = sc.h_init;
    cfg.ctrl_enable       = true;
    cfg.thermal_enable    = true;
    cfg.meas_noise_enable = true;
    cfg.lambda_c          = 0.7;
    cfg.a_pd              = 0.05;
    cfg.a_cov             = 0.05;
    cfg.meas_noise_std    = [0.00062; 0.00057; 0.00331];   % [um]
    cfg.suppress_xD       = true;
    cfg.use_am_lpf        = false;
    cfg.a_det             = 0.005;
    cfg.T_sim             = sc.T_sim;
    if strcmpi(variant, 'gscalar')
        cfg.beta_s                 = opts.beta_s;
        cfg.use_hbar_meas_for_adet = false;     % default: desired-height a_det
    end
    pc = physical_constants();
    if sc.is_pos
        cfg.trajectory_type = 'positioning';
        cfg.h_bottom        = sc.h_init;
        cfg.amplitude       = 0;
        cfg.h_min           = 1.5 * pc.R;
        cfg.h_bar_safe      = 1.5;
    else
        cfg.trajectory_type    = 'osc';
        cfg.h_bottom           = 2.7;            % h_bar = 1.2 near wall
        cfg.amplitude          = 2.5;
        cfg.frequency          = 1.0;
        cfg.n_cycles           = 2;
        cfg.t_hold             = 0.5;
        cfg.t_descend_override = 1.0;
        cfg.h_min              = 1.05 * pc.R;
        cfg.h_bar_safe         = 1;
    end
end


% ====================== run + metrics ======================

function A = run_variant(cfg, seeds, sc)
    ns = numel(seeds);
    track = nan(ns,3); astd = nan(ns,3); abias = nan(ns,3); lag = nan(ns,3);
    ah_stack = []; at_stack = []; ez_stack = [];
    t_vec = []; hbar_d = []; pd_z = []; pt_z = []; Ts = []; a_xm_s1 = [];
    s_hat_s1 = []; var_s_s1 = []; a_nom_val = []; kBT_val = [];
    for si = 1:ns
        ro = struct('seed', seeds(si), 'verbose', false, 'collect_diag', true, ...
                    'use_true_gain', false);
        s = run_pure_simulation(cfg, ro);
        Ts = s.meta.params_value.common.Ts;
        e  = s.p_d_out(2:end,:) - s.p_true_out(1:end-1,:);     % [N-1 x 3] [um]
        track(si,:) = 1e3 * std(e, 0, 1);                      % [nm]
        a_true = s.a_true_out(2:end,:);
        a_hat  = s.diag.a_hat(2:end,:);
        N = size(a_true,1);
        t  = (1:N)' * Ts;
        win = t >= sc.t_steady;
        astd(si,:)  = std(a_hat(win,:),0,1) ./ mean(a_hat(win,:),1);
        abias(si,:) = mean(a_hat(win,:),1) ./ mean(a_true(win,:),1) - 1;
        for ax = 1:3
            lag(si,ax) = xcorr_lag_ms(a_true(:,ax), a_hat(:,ax), Ts);
        end
        ah_stack = cat(3, ah_stack, a_hat);
        at_stack = cat(3, at_stack, a_true);
        ez_stack = cat(2, ez_stack, e(:,3));
        if si == 1
            t_vec = t;
            P = s.meta.params_value; w = P.wall.w_hat(:);
            hbar_d = (s.p_d_out(2:end,:)*w - P.wall.pz) / P.common.R;
            pd_z = s.p_d_out(2:end,:)*w; pt_z = s.p_true_out(1:end-1,:)*w;
            a_xm_s1 = s.diag.a_xm(2:end,:);
            a_nom_val = Ts / P.ctrl.gamma;
            kBT_val   = P.ctrl.k_B * P.ctrl.T;
            if isfield(s.diag, 's_hat'); s_hat_s1 = s.diag.s_hat(2:end); else; s_hat_s1 = zeros(N,1); end
            if isfield(s.diag, 'var_s'); var_s_s1 = s.diag.var_s(2:end); else; var_s_s1 = zeros(N,1); end
        end
    end
    win = t_vec >= sc.t_steady;
    A.track_std_nm = mean(track,1);
    A.ahat_std_rel = mean(astd,1);
    A.ahat_bias    = mean(abias,1);
    A.ahat_lag_ms  = mean(lag,1);
    A.a_hat_ens    = mean(ah_stack,3);
    A.a_hat_std    = std(ah_stack,0,3);                       % across-seed sigma(a_hat)
    A.a_true_ens   = mean(at_stack,3);
    % across-seed noise metric (deterministic swing cancels): per axis
    A.ahat_noise_rel = mean(A.a_hat_std(win,:) ./ A.a_true_ens(win,:), 1);
    A.a_xm_s1      = a_xm_s1;
    A.s_hat_s1     = s_hat_s1;
    A.var_s_s1     = var_s_s1;
    A.is_gscalar   = any(s_hat_s1 ~= 0);
    A.ez_rms       = sqrt(mean(ez_stack.^2, 2));
    A.ez_var       = var(ez_stack, 0, 2);
    A.a_nom        = a_nom_val;
    A.kBT          = kBT_val;
    A.lambda_c     = cfg.lambda_c;
    A.t = t_vec; A.hbar_d = hbar_d; A.pd_z = pd_z; A.pt_z = pt_z; A.Ts = Ts;
    A.win = win;
end


function lag_ms = xcorr_lag_ms(a_true, a_hat, Ts)
    x = a_true - mean(a_true); y = a_hat - mean(a_hat);
    if std(x) < eps || std(y) < eps; lag_ms = 0; return; end
    maxlag = min(200, numel(x)-1);
    [c, lags] = xcorr(y, x, maxlag, 'coeff');
    [~, im] = max(c); lag_ms = lags(im) * Ts * 1e3;
end


% ====================== gates ======================

function G = eval_gates(AG, A4, sc)
    z = 3;
    G = struct();
    % G-B tracking unchanged (all axes)
    rel = abs(AG.track_std_nm - A4.track_std_nm) ./ A4.track_std_nm;
    G.B = struct('val', max(rel), 'pass', all(rel < 0.05));
    % G-C bias
    thr_c = 0.03; if ~sc.is_pos; thr_c = 0.05; end
    G.C = struct('val', abs(AG.ahat_bias(z)), 'pass', abs(AG.ahat_bias(z)) < thr_c);
    % G-F convergence (seed-1 s_hat near 1 at t_steady, var shrinking)
    if AG.is_gscalar
        idx = find(AG.t >= sc.t_steady, 1, 'first');
        s_at = AG.s_hat_s1(idx);
        shrink = AG.var_s_s1(idx) < AG.var_s_s1(1);
        G.F = struct('s_at_steady', s_at, 'pass', abs(s_at-1) < 0.05 && shrink);
    else
        G.F = struct('s_at_steady', NaN, 'pass', false);
    end
    if sc.is_pos
        % G-D positioning noise (temporal rel std, z, absolute)
        G.D = struct('val', AG.ahat_std_rel(z), 'pass', AG.ahat_std_rel(z) < 0.05);
        G.A = struct('val', AG.ahat_noise_rel(z), 'pass', NaN);   % reported only
    else
        % G-A headline (osc, across-seed noise, z)
        passA = (AG.ahat_noise_rel(z) < 0.05) && (AG.ahat_noise_rel(z) < A4.ahat_noise_rel(z));
        G.A = struct('val', AG.ahat_noise_rel(z), 'val4', A4.ahat_noise_rel(z), 'pass', passA);
        G.D = struct('val', AG.ahat_std_rel(z), 'pass', NaN);     % reported only
    end
end


function print_report(scenario, sc, AG, A4, G, figs, opts)
    ax = {'x','y','z'};
    fprintf('\n[%s : %s, %d seeds]   (gscalar vs 4-state baseline)\n', scenario, sc.label, numel(opts.seeds));
    fprintf('  %-26s %8s %8s %8s\n', 'metric / axis', ax{:});
    fprintf('  %-26s %8.1f %8.1f %8.1f\n', 'track std gscalar [nm]', AG.track_std_nm);
    fprintf('  %-26s %8.1f %8.1f %8.1f\n', 'track std 4-state [nm]', A4.track_std_nm);
    fprintf('  %-26s %8.1f %8.1f %8.1f\n', 'a_hat noise(seed) gs [%]', 100*AG.ahat_noise_rel);
    fprintf('  %-26s %8.1f %8.1f %8.1f\n', 'a_hat noise(seed) 4 [%]', 100*A4.ahat_noise_rel);
    fprintf('  %-26s %8.1f %8.1f %8.1f\n', 'a_hat temporal std gs [%]', 100*AG.ahat_std_rel);
    fprintf('  %-26s %8.1f %8.1f %8.1f\n', 'a_hat bias gscalar [%]', 100*AG.ahat_bias);
    fprintf('  %-26s %8.1f %8.1f %8.1f\n', 'a_hat bias 4-state [%]', 100*A4.ahat_bias);
    fprintf('  --- gates ---\n');
    fns = fieldnames(G);
    for i = 1:numel(fns)
        g = G.(fns{i});
        if islogical(g.pass) || isnumeric(g.pass)
            if isnan(g.pass)
                fprintf('  G-%s : (reported)\n', fns{i});
            else
                tag = 'FAIL'; if g.pass; tag = 'PASS'; end
                fprintf('  G-%s : %s\n', fns{i}, tag);
            end
        end
    end
    for i = 1:numel(figs); fprintf('  [fig] %s\n', figs{i}); end
end


% ====================== figures ======================

function [FS,LFS,AXLW,COL] = fig_style()
    FS = 18; LFS = 13; AXLW = 2.0;
    % gscalar = red (candidate), 4-state = blue (baseline).
    COL.true = [0 0 0]; COL.gs = [0.8 0 0]; COL.s4 = [0 0.2 0.8];
end

function finish_axes(FS, AXLW)
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
end


function fp = fig1_gain_estimation(AG, A4, sc, out_dir)
    FS = 18; LFS = 13; LW = 2.0; LR = 3.0; LO = 2.0;
    COL_REF = [0 0.6 0]; COLG = [0.8 0 0]; COL4 = [0 0.2 0.8];
    t = AG.t; w_mm = max(3, round(0.02 / AG.Ts)); sm = @(v) movmean(v, w_mm);
    gain_cols = [1 3]; gain_lbl = {'a_x', 'a_z'};
    f = figure('Position',[80 80 1100 980],'Color','w','NumberTitle','off','Visible','off');
    tiledlayout(3, 1, 'TileSpacing','compact','Padding','compact');
    for r = 1:2
        c = gain_cols(r); nexttile; hold on;
        hT = plot(t, AG.a_true_ens(:,c),   '-','Color',COL_REF,'LineWidth',LR,'DisplayName','True');
        hG = plot(t, sm(AG.a_hat_ens(:,c)),'-','Color',COLG,   'LineWidth',LO,'DisplayName','gscalar');
        h4 = plot(t, sm(A4.a_hat_ens(:,c)),'-','Color',COL4,   'LineWidth',LO,'DisplayName','4-state');
        ylabel(sprintf('%s  (\\mum/pN)', gain_lbl{r}),'FontSize',FS,'FontWeight','bold');
        lo = min(AG.a_true_ens(:,c))*0.70; hi = max(AG.a_true_ens(:,c))*1.15;
        if hi > lo; ylim([lo hi]); end
        ax = gca; ax.YAxis.Exponent = 0; yl = ylim; ax.YTick = round(linspace(yl(1),yl(2),4),3);
        if r == 1
            legend([hT hG h4],'Location','northoutside','Orientation','horizontal', ...
                   'FontSize',LFS,'FontWeight','bold','Box','on');
        end
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',LW,'Box','on'); grid off;
    end
    nexttile; hold on;
    plot(t, zeros(size(t)), 'k:', 'LineWidth', 1.0, 'HandleVisibility', 'off');
    eG = plot(t, sm(AG.a_hat_ens(:,3)-AG.a_true_ens(:,3))*1e3, '-','Color',COLG,'LineWidth',LO,'DisplayName','gscalar');
    e4 = plot(t, sm(A4.a_hat_ens(:,3)-A4.a_true_ens(:,3))*1e3, '-','Color',COL4,'LineWidth',LO,'DisplayName','4-state');
    ylabel('a_z err  (nm/pN)','FontSize',FS,'FontWeight','bold');
    xlabel('Time (sec)','FontSize',FS,'FontWeight','bold');
    legend([eG e4],'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
    set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',LW,'Box','on'); grid off;
    fp = fullfile(out_dir, sprintf('fig1_gain_estimation_%s.png', sc.name));
    exportgraphics(f, fp, 'Resolution',150); close(f);
end


function fp = fig2_tracking_error(AG, A4, sc, out_dir)
    [FS,LFS,AXLW,COL] = fig_style(); t = AG.t;
    w_mm = max(3, round(0.02 / AG.Ts)); sm = @(v) movmean(v, w_mm);
    f = figure('Position',[80 80 1100 620],'Color','w','NumberTitle','off','Visible','off');
    hold on;
    h4 = plot(t, sm(A4.ez_rms)*1e3, '-', 'Color',COL.s4,'LineWidth',1.8,'DisplayName','4-state');
    hG = plot(t, sm(AG.ez_rms)*1e3, '-', 'Color',COL.gs,'LineWidth',1.8,'DisplayName','gscalar');
    ylabel('z track err RMS (nm)','FontSize',FS,'FontWeight','bold');
    xlabel('Time (sec)','FontSize',FS,'FontWeight','bold');
    legend([hG h4],'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
    finish_axes(FS, AXLW);
    fp = fullfile(out_dir, sprintf('fig2_tracking_error_%s.png', sc.name));
    exportgraphics(f, fp, 'Resolution',150); close(f);
end


function fp = fig_track_var(AG, A4, sc, out_dir)
    [FS,LFS,AXLW,COL] = fig_style(); t = AG.t;
    w_mm = max(3, round(0.02 / AG.Ts)); sm = @(v) movmean(v, w_mm);
    lc = AG.lambda_c; C_dx = 2 + 1/(1 - lc^2);
    var_th = C_dx * 4 * AG.kBT * AG.a_true_ens(:,3);
    f = figure('Position',[80 80 1100 560],'Color','w','NumberTitle','off','Visible','off');
    hold on;
    hT = plot(t, var_th*1e6,        '-','Color',[0 0.6 0],'LineWidth',2.5,'DisplayName','theory');
    h4 = plot(t, sm(A4.ez_var)*1e6, '-','Color',COL.s4,   'LineWidth',1.8,'DisplayName','4-state');
    hG = plot(t, sm(AG.ez_var)*1e6, '-','Color',COL.gs,   'LineWidth',1.8,'DisplayName','gscalar');
    ylabel('z track-err var (nm^2)','FontSize',FS,'FontWeight','bold');
    xlabel('Time (sec)','FontSize',FS,'FontWeight','bold');
    legend([hG h4 hT],'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
    finish_axes(FS, AXLW);
    fp = fullfile(out_dir, sprintf('fig_track_var_%s.png', sc.name));
    exportgraphics(f, fp, 'Resolution',150); close(f);
end


function fp = fig3_trajectory(AG, sc, out_dir)
    [FS,LFS,AXLW,COL] = fig_style(); t = AG.t;
    f = figure('Position',[80 80 1100 520],'Color','w','NumberTitle','off','Visible','off');
    hold on;
    hd = plot(t, AG.pd_z, '-', 'Color',COL.true,'LineWidth',2.2,'DisplayName','p_d (z)');
    hp = plot(t, AG.pt_z, '-', 'Color',COL.gs,  'LineWidth',1.2,'DisplayName','p_{true} (z)');
    ylabel('z position (\mum)','FontSize',FS,'FontWeight','bold');
    xlabel('Time (sec)','FontSize',FS,'FontWeight','bold');
    legend([hd hp],'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
    finish_axes(FS, AXLW);
    fp = fullfile(out_dir, sprintf('fig3_trajectory_%s.png', sc.name));
    exportgraphics(f, fp, 'Resolution',150); close(f);
end


function fp = fig_var_ahat(AG, A4, sc, out_dir)
%FIG_VAR_AHAT  positioning a_hat spread: per-axis temporal rel std (%), gs vs 4.
    [FS,LFS,AXLW,COL] = fig_style();
    f = figure('Position',[80 80 900 620],'Color','w','NumberTitle','off','Visible','off');
    hold on;
    data = [100*A4.ahat_std_rel(:), 100*AG.ahat_std_rel(:)];   % [3 x 2]
    hb = bar(1:3, data, 0.8);
    hb(1).FaceColor = COL.s4; hb(2).FaceColor = COL.gs;
    set(gca, 'XTick', 1:3, 'XTickLabel', {'x','y','z'});
    ylabel('a_{hat} rel. std (%)','FontSize',FS,'FontWeight','bold');
    xlabel('axis','FontSize',FS,'FontWeight','bold');
    legend({'4-state','gscalar'},'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
    finish_axes(FS, AXLW);
    fp = fullfile(out_dir, sprintf('fig_var_ahat_%s.png', sc.name));
    exportgraphics(f, fp, 'Resolution',150); close(f);
end


function fp = fig_ahat_vs_height(AG, A4, sc, out_dir)
%FIG_AHAT_VS_HEIGHT  across-seed relative a_hat noise (sigma/a_true, z) binned by
%   h_bar_d: gscalar flat at all heights, 4-state fans up near the wall.
    [FS,LFS,AXLW,COL] = fig_style();
    win = AG.win;
    hb = AG.hbar_d(win);
    rnG = AG.a_hat_std(win,3) ./ AG.a_true_ens(win,3);
    rn4 = A4.a_hat_std(win,3) ./ A4.a_true_ens(win,3);
    edges = linspace(min(hb), max(hb), 13);
    ctr = 0.5*(edges(1:end-1)+edges(2:end));
    [bG, ~] = bin_mean(hb, rnG, edges);
    [b4, ~] = bin_mean(hb, rn4, edges);
    f = figure('Position',[80 80 1000 620],'Color','w','NumberTitle','off','Visible','off');
    hold on;
    h4 = plot(ctr, 100*b4, '-o','Color',COL.s4,'LineWidth',2.0,'MarkerFaceColor',COL.s4,'DisplayName','4-state');
    hG = plot(ctr, 100*bG, '-o','Color',COL.gs,'LineWidth',2.0,'MarkerFaceColor',COL.gs,'DisplayName','gscalar');
    ylabel('a_z noise  \sigma_{seed}/a_{true} (%)','FontSize',FS,'FontWeight','bold');
    xlabel('desired h_d / R','FontSize',FS,'FontWeight','bold');
    legend([hG h4],'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
    finish_axes(FS, AXLW);
    fp = fullfile(out_dir, sprintf('fig_ahat_vs_height_%s.png', sc.name));
    exportgraphics(f, fp, 'Resolution',150); close(f);
end


function fp = fig_s_convergence(AG, sc, out_dir)
%FIG_S_CONVERGENCE  seed-1 s_hat(t) with +/-sqrt(var_s) band, true s=1 line, and
%   faint raw per-axis a_xm/a_true scatter (the ~45% per-shot floor being pooled).
    [FS,LFS,AXLW,COL] = fig_style(); t = AG.t;
    f = figure('Position',[80 80 1100 560],'Color','w','NumberTitle','off','Visible','off');
    hold on;
    if AG.is_gscalar
        sd = sqrt(max(AG.var_s_s1, 0));
        % faint raw s_meas scatter (z axis): a_xm/a_true (proxy for /a_det)
        sm_raw = AG.a_xm_s1(:,3) ./ max(AG.a_true_ens(:,3), eps);
        plot(t, sm_raw, '.', 'Color',[0.7 0.7 0.85], 'MarkerSize',3, 'HandleVisibility','off');
        fill([t; flipud(t)], [AG.s_hat_s1+sd; flipud(AG.s_hat_s1-sd)], COL.gs, ...
             'FaceAlpha',0.20,'EdgeColor','none','HandleVisibility','off');
        hS = plot(t, AG.s_hat_s1, '-','Color',COL.gs,'LineWidth',2.2,'DisplayName','s_{hat}');
        h1 = plot(t, ones(size(t)), 'k--','LineWidth',1.5,'DisplayName','s_{true}=1');
        ylim([0.5 1.5]);
        legend([hS h1],'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
    else
        text(0.5,0.5,'(non-gscalar variant: no s_{hat})','Units','normalized','HorizontalAlignment','center','FontSize',FS);
    end
    ylabel('scalar s','FontSize',FS,'FontWeight','bold');
    xlabel('Time (sec)','FontSize',FS,'FontWeight','bold');
    finish_axes(FS, AXLW);
    fp = fullfile(out_dir, sprintf('fig_s_convergence_%s.png', sc.name));
    exportgraphics(f, fp, 'Resolution',150); close(f);
end


function [bm, bn] = bin_mean(x, y, edges)
    nb = numel(edges)-1; bm = nan(nb,1); bn = zeros(nb,1);
    for b = 1:nb
        m = x >= edges(b) & x < edges(b+1);
        if b == nb; m = m | (x == edges(end)); end
        bn(b) = sum(m);
        if bn(b) > 0; bm(b) = mean(y(m)); end
    end
end


% ====================== summary ======================

function out = make_summary(out_root, opts)
    [FS,LFS,AXLW,COL] = fig_style();
    scs = {'pos_h50','pos_h10','osc_1hz'};
    M = {}; labels = {};
    for i = 1:numel(scs)
        mf = fullfile(out_root, scs{i}, 'metrics.mat');
        if exist(mf, 'file'); d = load(mf); M{end+1} = d.M; labels{end+1} = scs{i}; end %#ok<AGROW>
    end
    assert(~isempty(M), 'make_summary: no metrics.mat found; run the scenarios first.');
    ns = numel(M); z = 3;
    track4 = arrayfun(@(k) M{k}.track4(z), 1:ns); trackG = arrayfun(@(k) M{k}.trackG(z), 1:ns);
    noise4 = arrayfun(@(k) 100*M{k}.noise4(z), 1:ns); noiseG = arrayfun(@(k) 100*M{k}.noiseG(z), 1:ns);
    bias4  = arrayfun(@(k) 100*abs(M{k}.abias4(z)), 1:ns); biasG = arrayfun(@(k) 100*abs(M{k}.abiasG(z)), 1:ns);

    f = figure('Position',[60 60 1250 760],'Color','w','NumberTitle','off','Visible','off');
    panel = {{'z track std (nm)', track4, trackG}, ...
             {'z a_{hat} noise \sigma_{seed}/a (%)', noise4, noiseG}, ...
             {'z a_{hat} |bias| (%)', bias4, biasG}};
    for p = 1:3
        subplot(1,3,p); hold on;
        hb = bar(1:ns, [panel{p}{2}(:), panel{p}{3}(:)], 0.8);
        hb(1).FaceColor = COL.s4; hb(2).FaceColor = COL.gs;
        set(gca, 'XTick', 1:ns, 'XTickLabel', labels, 'XTickLabelRotation', 20);
        ylabel(panel{p}{1}, 'FontSize', FS, 'FontWeight','bold');
        if p == 2
            legend({'4-state','gscalar'},'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
        end
        finish_axes(FS, AXLW);
    end
    fp = fullfile(out_root, 'fig_summary_gscalar_vs_4state.png');
    exportgraphics(f, fp, 'Resolution',150); close(f);
    out = struct('fig', fp, 'scenarios', {labels});
    if opts.verbose; fprintf('[summary] %s\n', fp); end
end
