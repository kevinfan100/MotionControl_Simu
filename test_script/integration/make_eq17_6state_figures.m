function make_eq17_6state_figures(sig, meta, out_dir)
%MAKE_EQ17_6STATE_FIGURES  Canonical verification figures for the 6-state
%   RevisedControl_Vpersonal controller (shared by all scenarios).
%
%   make_eq17_6state_figures(sig, meta, out_dir)
%
%   Produces two publication-style figures (EXP/thesis style: role colors,
%   grid off, tiledlayout, stats-in-title, legend northoutside) and saves
%   them as PNG into out_dir:
%       fig1_gain_estimation.png  - a_x, a_z : Measured / True / Estimated
%       fig2_tracking_error.png   - per-axis tracking error + IIR-LP overlay
%                                    + delta_x_D^d (disturbance est) on right axis
%
%   The same figures serve positioning (h=50, h=10) and ramp scenarios; only
%   the gain-figure y-limits and titles adapt via meta.kind.
%
%   ----- Inputs -----
%   sig : struct of seed-1 time series (all columns ordered [x y z])
%       .t        [N x1]  time [s]
%       .idx      [N x1 logical]  plot/steady window (t >= t_warmup)
%       .a_xm     [N x3]  raw IIR gain measurement a_xm           [um/pN]
%       .a_hat    [N x3]  EKF gain estimate                        [um/pN]
%       .a_true   [N x3]  physics ground-truth gain (para,para,perp) [um/pN]
%       .err      [N x3]  tracking error (p_m - p_d)               [nm]
%       .del_pmd  [N x3]  IIR low-pass of measured error, sign-matched to err [nm]
%       .xD_nm    [N x3]  delta_x_D^d disturbance estimate          [nm]
%   meta : struct
%       .name     char    scenario tag (e.g. 'h50','h10','ramp'), used in console
%       .kind     char    'positioning' | 'ramp'
%   out_dir : char  output directory (created if missing)
%
%   See also: verify_eq17_6state, run_eq17_6state_all

    if ~exist(out_dir, 'dir'); mkdir(out_dir); end

    % ---- canonical EXP/thesis style (matches verify_eq6_h50_figures) ----
    COL_REF  = [0 0.6 0];               % green  : True
    COL_OUT  = [0.8 0 0];               % red    : Estimated
    COL_ERR  = [0 0.2 0.8];             % blue   : Error
    COL_MEAS = [0.45 0.55 0.95 0.30];   % light blue (alpha) : Measured a_xm
    COL_LP   = [0 0 0];                 % black  : IIR low-pass overlay
    COL_DST  = [0.85 0.45 0];           % orange : delta_x_D^d estimate
    FS = 18; LFS = 14; LW = 2.0;
    LR = 3.0; LO = 2.0; LM = 0.5;       % true / estimated / measured line widths

    t   = sig.t;
    idx = sig.idx;
    t0  = t(find(idx, 1, 'first'));
    t1  = max(t);
    is_ramp = strcmpi(meta.kind, 'ramp');

    % ================= FIG 1 : gain estimation (a_x, a_z) =================
    f1 = figure('Position', [80 80 1100 720], 'Color', 'w', ...
                'Name', sprintf('eq17 6-state %s : gain', meta.name), 'NumberTitle', 'off');
    tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    gain_cols = [1 3];                  % a_x = col 1, a_z = col 3
    gain_lbl  = {'a_x', 'a_z'};
    for r = 1:2
        c = gain_cols(r);
        nexttile; hold on;
        plot(t(idx), sig.a_xm(idx, c),  '-', 'Color', COL_MEAS, 'LineWidth', LM, 'DisplayName', 'Measured');
        plot(t(idx), sig.a_true(idx, c),'-', 'Color', COL_REF,  'LineWidth', LR, 'DisplayName', 'True');
        plot(t(idx), sig.a_hat(idx, c), '-', 'Color', COL_OUT,  'LineWidth', LO, 'DisplayName', 'Estimated');

        rel  = (sig.a_hat(idx, c) - sig.a_true(idx, c)) ./ sig.a_true(idx, c);
        bias = 100 * mean(rel);
        sd   = 100 * std(rel);
        if is_ramp
            title(sprintf('%s:   mean rel-err %+.2f%%     std %.2f%%', gain_lbl{r}, bias, sd), ...
                  'FontSize', FS, 'FontWeight', 'bold');
            lo = min(sig.a_true(idx, c)) * 0.70;
            hi = max(sig.a_true(idx, c)) * 1.15;
        else
            title(sprintf('%s:   bias %+.2f%%     std %.2f%%', gain_lbl{r}, bias, sd), ...
                  'FontSize', FS, 'FontWeight', 'bold');
            tm = mean(sig.a_true(idx, c));
            lo = 0.6 * tm; hi = 1.4 * tm;
        end
        ylabel(sprintf('%s  (\\mum/pN)', gain_lbl{r}), 'FontSize', FS, 'FontWeight', 'bold');
        ylim([lo hi]);
        if r == 2, xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold'); end
        if r == 1
            legend('Location', 'northoutside', 'Orientation', 'horizontal', ...
                   'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'off');
        end
        xlim([t0 t1]);
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', LW, 'Box', 'on'); grid off;
    end
    disable_toolbars(f1);
    exportgraphics(f1, fullfile(out_dir, 'fig1_gain_estimation.png'), 'Resolution', 150);

    % ============ FIG 2 : tracking error + LP + disturbance (3x1) ============
    f2 = figure('Position', [80 80 1100 920], 'Color', 'w', ...
                'Name', sprintf('eq17 6-state %s : tracking', meta.name), 'NumberTitle', 'off');
    tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    ymax = max(max(abs(sig.err(idx, :))));
    axn  = {'x', 'y', 'z'};
    for k = 1:3
        nexttile;
        yyaxis left; hold on;
        he = plot(t(idx), sig.err(idx, k),     '-', 'Color', COL_ERR, 'LineWidth', 0.6, 'DisplayName', 'error');
        hl = plot(t(idx), sig.del_pmd(idx, k), '-', 'Color', COL_LP,  'LineWidth', LO,  'DisplayName', '\delta x_{md} (LP)');
        ylabel(sprintf('%s error (nm)', axn{k}), 'FontSize', FS, 'FontWeight', 'bold');
        ylim([-ymax ymax] * 1.05); set(gca, 'YColor', 'k');

        yyaxis right;
        hd = plot(t(idx), sig.xD_nm(idx, k), '-', 'Color', COL_DST, 'LineWidth', LO, 'DisplayName', '\delta x_D^d est');
        ylabel('\delta x_D^d (nm)', 'FontSize', FS - 2, 'FontWeight', 'bold'); set(gca, 'YColor', COL_DST);
        xdm = max(0.5, max(abs(sig.xD_nm(idx, k))));
        ylim([-xdm xdm] * 1.2);

        mv = mean(sig.err(idx, k)); sv = std(sig.err(idx, k));
        title(sprintf('%s:  err mean %+.2f / std %.2f nm    |\\delta x_D^d| %.2f nm', ...
              axn{k}, mv, sv, mean(abs(sig.xD_nm(idx, k)))), 'FontSize', FS - 2, 'FontWeight', 'bold');
        if k == 3, xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold'); end
        if k == 1
            legend([he hl hd], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
                   'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'off');
        end
        xlim([t0 t1]);
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', LW, 'Box', 'on'); grid off;
    end
    disable_toolbars(f2);
    exportgraphics(f2, fullfile(out_dir, 'fig2_tracking_error.png'), 'Resolution', 150);

    close(f1); close(f2);
end


function disable_toolbars(fig)
%DISABLE_TOOLBARS  Hide axes interaction toolbars so exportgraphics does not
%   render them (avoids the "Exported image displays axes toolbar" warning).
    axs = findall(fig, 'Type', 'Axes');
    for ii = 1:numel(axs)
        axs(ii).Toolbar.Visible = 'off';
    end
end
