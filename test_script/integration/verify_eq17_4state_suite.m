function out = verify_eq17_4state_suite(scenario, opts)
%VERIFY_EQ17_4STATE_SUITE  4-state vs 5-state across scenarios, Category-A
%   figures + positioning a_hat-spread + cross-scenario summary.
%
%   out = verify_eq17_4state_suite('pos_h50')   % static hold h_bar~22
%   out = verify_eq17_4state_suite('pos_h10')   % static hold h_bar~4.4
%   out = verify_eq17_4state_suite('osc_1hz')   % 1 Hz near-wall oscillation
%   out = verify_eq17_4state_suite('summary')   % read 3 cached metrics, bar chart
%
%   Each scenario call runs the 4-state and 5-state controllers (seed ensemble),
%   writes Category-A figures (fig1 gain estimation / fig2 tracking error /
%   fig3 trajectory) into test_results/verify_4state/<scenario>/, plus
%   fig_var_ahat for the positioning scenarios, and caches metrics.mat. The
%   'summary' call reads the three metrics.mat and writes fig_summary.png.
%
%   In positioning, h_d is constant -> Delta_h_d = 0 -> the 4-state feedforward
%   is identically zero; the run is a do-no-harm regression check (the 4-state
%   should match the 5-state). In osc, the feedforward removes the a_hat lag.
%
%   opts.seeds (default 1:6), opts.T_sim (scenario default), opts.verbose.
%
%   See also: motion_control_law_eq17_4state, run_pure_simulation,
%             verify_eq17_4state, compare_gain_6state

    if nargin < 2; opts = struct(); end
    if ~isfield(opts, 'seeds');   opts.seeds = 1:6;     end
    if ~isfield(opts, 'verbose'); opts.verbose = true;  end

    here = fileparts(mfilename('fullpath'));
    proj = fullfile(here, '..', '..');
    addpath(genpath(proj));
    out_root = fullfile(proj, 'test_results', 'verify_4state');

    if strcmpi(scenario, 'summary')
        out = make_summary(out_root, opts);
        return;
    end

    sc = scenario_spec(scenario);
    if isfield(opts, 'T_sim'); sc.T_sim = opts.T_sim; end
    out_dir = fullfile(out_root, scenario);
    if ~exist(out_dir, 'dir'); mkdir(out_dir); end

    cfg4 = build_cfg(sc, '4state');
    cfg5 = build_cfg(sc, '5state');
    A4 = run_variant(cfg4, opts.seeds, sc);
    A5 = run_variant(cfg5, opts.seeds, sc);

    % --- Category-A figures ---
    f1 = fig1_gain_estimation(A4, A5, sc, out_dir);
    f2 = fig2_tracking_error(A4, A5, sc, out_dir);
    f3 = fig3_trajectory(A4, sc, out_dir);
    figs = {f1, f2, f3};
    if sc.is_pos
        figs = [figs, {fig_var_ahat(A4, A5, sc, out_dir)}];
    else
        figs = [figs, {fig_track_var(A4, A5, sc, out_dir)}];
    end

    % --- cache scalar metrics for the summary ---
    M = struct('scenario', scenario, 'label', sc.label, 'is_pos', sc.is_pos, ...
               'track4', A4.track_std_nm, 'track5', A5.track_std_nm, ...
               'astd4', A4.ahat_std_rel, 'astd5', A5.ahat_std_rel, ...
               'abias4', A4.ahat_bias, 'abias5', A5.ahat_bias, ...
               'alag4', A4.ahat_lag_ms, 'alag5', A5.ahat_lag_ms);
    save(fullfile(out_dir, 'metrics.mat'), 'M');

    out = struct('scenario', scenario, 'metrics', M, 'figs', {figs});
    if opts.verbose
        ax = {'x','y','z'};
        fprintf('\n[%s : %s, %d seeds]   (mean over seeds)\n', scenario, sc.label, numel(opts.seeds));
        fprintf('  %-22s %8s %8s %8s\n', 'metric / axis', ax{:});
        fprintf('  %-22s %8.1f %8.1f %8.1f\n', 'track std 4 [nm]', A4.track_std_nm);
        fprintf('  %-22s %8.1f %8.1f %8.1f\n', 'track std 5 [nm]', A5.track_std_nm);
        fprintf('  %-22s %8.1f %8.1f %8.1f\n', 'a_hat std 4 [%]', 100*A4.ahat_std_rel);
        fprintf('  %-22s %8.1f %8.1f %8.1f\n', 'a_hat std 5 [%]', 100*A5.ahat_std_rel);
        fprintf('  %-22s %8.1f %8.1f %8.1f\n', 'a_hat bias 4 [%]', 100*A4.ahat_bias);
        fprintf('  %-22s %8.1f %8.1f %8.1f\n', 'a_hat bias 5 [%]', 100*A5.ahat_bias);
        fprintf('  %-22s %8.1f %8.1f %8.1f\n', 'a_hat lag 4 [ms]', A4.ahat_lag_ms);
        fprintf('  %-22s %8.1f %8.1f %8.1f\n', 'a_hat lag 5 [ms]', A5.ahat_lag_ms);
        for i = 1:numel(figs); fprintf('  [fig] %s\n', figs{i}); end
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
            error('verify_eq17_4state_suite:badScenario', ...
                  'scenario must be pos_h50 | pos_h10 | osc_1hz | summary');
    end
end


function cfg = build_cfg(sc, variant)
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
    pc = physical_constants();
    if sc.is_pos
        cfg.trajectory_type = 'positioning';   % hold at h_init
        cfg.h_bottom        = sc.h_init;        % no descent
        cfg.amplitude       = 0;                % no oscillation
        cfg.h_min           = 1.5 * pc.R;
        cfg.h_bar_safe      = 1.5;              % gate won't fire (h_bar >> 1.5)
    else
        cfg.trajectory_type    = 'osc';
        cfg.h_bottom           = 2.7;           % h_bar = 1.2 near wall
        cfg.amplitude          = 2.5;
        cfg.frequency          = 1.0;
        cfg.n_cycles           = 2;
        cfg.t_hold             = 0.5;
        cfg.t_descend_override = 1.0;
        cfg.h_min              = 1.05 * pc.R;
        cfg.h_bar_safe         = 1;             % gate-free even at trough h_bar=1.2
    end
end


% ====================== run + metrics ======================

function A = run_variant(cfg, seeds, sc)
    ns = numel(seeds);
    track = nan(ns,3); astd = nan(ns,3); abias = nan(ns,3); lag = nan(ns,3);
    ah_stack = []; at_stack = []; ez_stack = [];
    t_vec = []; hbar_d = []; pd_z = []; pt_z = []; Ts = []; a_xm_s1 = [];
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
        win = t >= sc.t_steady;                                % steady window
        astd(si,:)  = std(a_hat(win,:),0,1) ./ mean(a_hat(win,:),1);          % temporal rel std
        abias(si,:) = mean(a_hat(win,:),1) ./ mean(a_true(win,:),1) - 1;      % rel bias
        for ax = 1:3
            lag(si,ax) = xcorr_lag_ms(a_true(:,ax), a_hat(:,ax), Ts);
        end
        ah_stack = cat(3, ah_stack, a_hat);
        at_stack = cat(3, at_stack, a_true);
        ez_stack = cat(2, ez_stack, e(:,3));                   % z tracking error per seed
        if si == 1
            t_vec = t;
            P = s.meta.params_value; w = P.wall.w_hat(:);
            hbar_d = (s.p_d_out(2:end,:)*w - P.wall.pz) / P.common.R;
            pd_z = s.p_d_out(2:end,:)*w; pt_z = s.p_true_out(1:end-1,:)*w;
            a_xm_s1 = s.diag.a_xm(2:end,:);                    % raw IIR gain measurement (seed 1)
            a_nom_val = Ts / P.ctrl.gamma;                    % nominal gain a_nom = Ts/gamma_N
            kBT_val   = P.ctrl.k_B * P.ctrl.T;                % [pN*um]
        end
    end
    A.track_std_nm = mean(track,1);
    A.ahat_std_rel = mean(astd,1);
    A.ahat_bias    = mean(abias,1);
    A.ahat_lag_ms  = mean(lag,1);
    A.a_hat_ens    = mean(ah_stack,3);
    A.a_hat_std    = std(ah_stack,0,3);                       % across-seed sigma(a_hat) = sqrt(Var(a_hat))
    A.a_true_ens   = mean(at_stack,3);
    A.a_xm_s1      = a_xm_s1;
    A.ez_rms       = sqrt(mean(ez_stack.^2, 2));               % [N-1] RMS over seeds [um]
    A.ez_var       = var(ez_stack, 0, 2);                      % [N-1] across-seed Var(e_z) [um^2]
    A.ez_s1        = ez_stack(:,1);                            % seed-1 z tracking error [um]
    A.a_nom        = a_nom_val;
    A.kBT          = kBT_val;
    A.lambda_c     = cfg.lambda_c;
    A.t = t_vec; A.hbar_d = hbar_d; A.pd_z = pd_z; A.pt_z = pt_z; A.Ts = Ts;
end


function lag_ms = xcorr_lag_ms(a_true, a_hat, Ts)
    x = a_true - mean(a_true); y = a_hat - mean(a_hat);
    if std(x) < eps || std(y) < eps; lag_ms = 0; return; end
    maxlag = min(200, numel(x)-1);
    [c, lags] = xcorr(y, x, maxlag, 'coeff');
    [~, im] = max(c); lag_ms = lags(im) * Ts * 1e3;
end


% ====================== figures ======================

function [FS,LFS,AXLW,COL] = fig_style()
    FS = 18; LFS = 13; AXLW = 2.0;
    % Unified scheme: 4-state = red, 5-state = blue (matches fig1 canonical).
    COL.true = [0 0 0]; COL.s4 = [0.8 0 0]; COL.s5 = [0 0.2 0.8];
end

function finish_axes(FS, AXLW)
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
end


function fp = fig1_gain_estimation(A4, A5, sc, out_dir)
%FIG1  Gain estimation a_x (col 1) / a_z (col 3), canonical layout (Measured
%   a_xm / True / 4-state / 5-state), units um/pN, no title. Mirrors
%   make_eq17_6state_figures fig1.
    FS = 18; LFS = 13; LW = 2.0; LR = 3.0; LO = 2.0;
    COL_REF  = [0 0.6 0];               % green : True
    COL4     = [0.8 0 0];               % red   : 4-state
    COL5     = [0 0.2 0.8];             % blue  : 5-state
    t = A4.t;
    w_mm = max(3, round(0.02 / A4.Ts)); sm = @(v) movmean(v, w_mm);
    gain_cols = [1 3]; gain_lbl = {'a_x', 'a_z'};
    f = figure('Position',[80 80 1100 980],'Color','w','NumberTitle','off','Visible','off');
    tiledlayout(3, 1, 'TileSpacing','compact','Padding','compact');
    % --- tiles 1-2: a_x / a_z tracking (True / 4-state / 5-state) ---
    for r = 1:2
        c = gain_cols(r); nexttile; hold on;
        hT = plot(t, A4.a_true_ens(:,c),   '-','Color',COL_REF, 'LineWidth',LR,'DisplayName','True');
        h4 = plot(t, sm(A4.a_hat_ens(:,c)),'-','Color',COL4,    'LineWidth',LO,'DisplayName','4-state');
        h5 = plot(t, sm(A5.a_hat_ens(:,c)),'-','Color',COL5,    'LineWidth',LO,'DisplayName','5-state');
        ylabel(sprintf('%s  (\\mum/pN)', gain_lbl{r}),'FontSize',FS,'FontWeight','bold');
        lo = min(A4.a_true_ens(:,c))*0.70; hi = max(A4.a_true_ens(:,c))*1.15;
        if hi > lo; ylim([lo hi]); end
        ax = gca; ax.YAxis.Exponent = 0; yl = ylim; ax.YTick = round(linspace(yl(1),yl(2),4),3);
        if r == 1
            legend([hT h4 h5],'Location','northoutside','Orientation','horizontal', ...
                   'FontSize',LFS,'FontWeight','bold','Box','on');
        end
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',LW,'Box','on'); grid off;
    end
    % --- tile 3: a_z estimation error (a_hat - a_true), 4-state vs 5-state [nm/pN] ---
    nexttile; hold on;
    plot(t, zeros(size(t)), 'k:', 'LineWidth', 1.0, 'HandleVisibility', 'off');
    e4 = plot(t, sm(A4.a_hat_ens(:,3)-A4.a_true_ens(:,3))*1e3, '-','Color',COL4,'LineWidth',LO,'DisplayName','4-state');
    e5 = plot(t, sm(A5.a_hat_ens(:,3)-A5.a_true_ens(:,3))*1e3, '-','Color',COL5,'LineWidth',LO,'DisplayName','5-state');
    ylabel('a_z err  (nm/pN)','FontSize',FS,'FontWeight','bold');
    xlabel('Time (sec)','FontSize',FS,'FontWeight','bold');
    legend([e4 e5],'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
    set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',LW,'Box','on'); grid off;
    fp = fullfile(out_dir, sprintf('fig1_gain_estimation_%s.png', sc.name));
    exportgraphics(f, fp, 'Resolution',150); close(f);
end


function fp = fig2_tracking_error(A4, A5, sc, out_dir)
%FIG2  z-axis tracking error: cross-seed RMS envelope sqrt(mean_seeds e_z^2)(t),
%   4-state vs 5-state. (Single-seed traces are noise-dominated and overlap; the
%   envelope shows the small aggregate edge.)
    [FS,LFS,AXLW,COL] = fig_style(); t = A4.t;
    w_mm = max(3, round(0.02 / A4.Ts)); sm = @(v) movmean(v, w_mm);
    f = figure('Position',[80 80 1100 620],'Color','w','NumberTitle','off','Visible','off');
    hold on;
    h5 = plot(t, sm(A5.ez_rms)*1e3, '-', 'Color',COL.s5,'LineWidth',1.8,'DisplayName','5-state');
    h4 = plot(t, sm(A4.ez_rms)*1e3, '-', 'Color',COL.s4,'LineWidth',1.8,'DisplayName','4-state');
    ylabel('z track err RMS (nm)','FontSize',FS,'FontWeight','bold');
    xlabel('Time (sec)','FontSize',FS,'FontWeight','bold');
    legend([h4 h5],'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
    finish_axes(FS, AXLW);
    fp = fullfile(out_dir, sprintf('fig2_tracking_error_%s.png', sc.name));
    exportgraphics(f, fp, 'Resolution',150); close(f);
end


function fp = fig_track_var(A4, A5, sc, out_dir)
%FIG_TRACK_VAR  z tracking-error variance vs time: 4-state / 5-state (across-seed
%   Var(e_z)) against the closed-loop theory C_dx*4kBT*a_z(t), with the
%   closed-loop factor C_dx = 2 + 1/(1-lambda_c^2). The theory assumes a correct
%   gain; the 5-state's near-wall a_hat lag drives its tracking variance above it.
    [FS,LFS,AXLW,COL] = fig_style(); t = A4.t;
    w_mm = max(3, round(0.02 / A4.Ts)); sm = @(v) movmean(v, w_mm);
    lc = A4.lambda_c; C_dx = 2 + 1/(1 - lc^2);
    var_th = C_dx * 4 * A4.kBT * A4.a_true_ens(:,3);          % [um^2] theory Var(e_z)(t)
    f = figure('Position',[80 80 1100 560],'Color','w','NumberTitle','off','Visible','off');
    hold on;
    hT = plot(t, var_th*1e6,         '-','Color',[0 0.6 0],'LineWidth',2.5,'DisplayName','theory');
    h5 = plot(t, sm(A5.ez_var)*1e6,  '-','Color',COL.s5,   'LineWidth',1.8,'DisplayName','5-state');
    h4 = plot(t, sm(A4.ez_var)*1e6,  '-','Color',COL.s4,   'LineWidth',1.8,'DisplayName','4-state');
    ylabel('z track-err var (nm^2)','FontSize',FS,'FontWeight','bold');
    xlabel('Time (sec)','FontSize',FS,'FontWeight','bold');
    legend([h4 h5 hT],'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
    finish_axes(FS, AXLW);
    fp = fullfile(out_dir, sprintf('fig_track_var_%s.png', sc.name));
    exportgraphics(f, fp, 'Resolution',150); close(f);
end


function fp = fig3_trajectory(A4, sc, out_dir)
%FIG3  desired vs true z position.
    [FS,LFS,AXLW,COL] = fig_style(); t = A4.t;
    f = figure('Position',[80 80 1100 520],'Color','w','NumberTitle','off','Visible','off');
    hold on;
    hd = plot(t, A4.pd_z, '-', 'Color',COL.true,'LineWidth',2.2,'DisplayName','p_d (z)');
    hp = plot(t, A4.pt_z, '-', 'Color',COL.s4, 'LineWidth',1.2,'DisplayName','p_{true} (z)');
    ylabel('z position (\mum)','FontSize',FS,'FontWeight','bold');
    xlabel('Time (sec)','FontSize',FS,'FontWeight','bold');
    legend([hd hp],'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
    finish_axes(FS, AXLW);
    fp = fullfile(out_dir, sprintf('fig3_trajectory_%s.png', sc.name));
    exportgraphics(f, fp, 'Resolution',150); close(f);
end


function fp = fig_var_ahat(A4, A5, sc, out_dir)
%FIG_VAR_AHAT  positioning a_hat spread: per-axis relative std (%), 4 vs 5.
    [FS,LFS,AXLW,COL] = fig_style();
    f = figure('Position',[80 80 900 620],'Color','w','NumberTitle','off','Visible','off');
    hold on;
    data = [100*A5.ahat_std_rel(:), 100*A4.ahat_std_rel(:)];   % [3 x 2] x/y/z
    hb = bar(1:3, data, 0.8);
    hb(1).FaceColor = COL.s5; hb(2).FaceColor = COL.s4;
    set(gca, 'XTick', 1:3, 'XTickLabel', {'x','y','z'});
    ylabel('a_{hat} rel. std (%)','FontSize',FS,'FontWeight','bold');
    xlabel('axis','FontSize',FS,'FontWeight','bold');
    legend({'5-state','4-state'},'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
    finish_axes(FS, AXLW);
    fp = fullfile(out_dir, sprintf('fig_var_ahat_%s.png', sc.name));
    exportgraphics(f, fp, 'Resolution',150); close(f);
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
    ns = numel(M);
    % z-axis scalar per scenario (4 vs 5): track std, a_hat std, a_hat lag
    z = 3;
    track4 = arrayfun(@(k) M{k}.track4(z), 1:ns); track5 = arrayfun(@(k) M{k}.track5(z), 1:ns);
    bias4  = arrayfun(@(k) 100*abs(M{k}.abias4(z)), 1:ns); bias5 = arrayfun(@(k) 100*abs(M{k}.abias5(z)), 1:ns);
    % lag is only meaningful in osc; a_hat is flat in positioning -> zero it
    lagmask = double(arrayfun(@(k) ~M{k}.is_pos, 1:ns));
    lag4   = arrayfun(@(k) M{k}.alag4(z), 1:ns) .* lagmask; lag5 = arrayfun(@(k) M{k}.alag5(z), 1:ns) .* lagmask;

    f = figure('Position',[60 60 1250 760],'Color','w','NumberTitle','off','Visible','off');
    panel = {{'z track std (nm)', track5, track4}, ...
             {'z a_{hat} |bias| (%)', bias5, bias4}, ...
             {'z a_{hat} lag (ms)', lag5, lag4}};
    for p = 1:3
        subplot(1,3,p); hold on;
        hb = bar(1:ns, [panel{p}{2}(:), panel{p}{3}(:)], 0.8);
        hb(1).FaceColor = COL.s5; hb(2).FaceColor = COL.s4;
        set(gca, 'XTick', 1:ns, 'XTickLabel', labels, 'XTickLabelRotation', 20);
        ylabel(panel{p}{1}, 'FontSize', FS, 'FontWeight','bold');
        if p == 2
            legend({'5-state','4-state'},'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
        end
        finish_axes(FS, AXLW);
    end
    fp = fullfile(out_root, 'fig_summary_4v5.png');
    exportgraphics(f, fp, 'Resolution',150); close(f);
    out = struct('fig', fp, 'scenarios', {labels});
    if opts.verbose; fprintf('[summary] %s\n', fp); end
end
