function make_figures(out, a_true, scenario, out_dir)
%MAKE_FIGURES  Two verification figures for the 6-state package.
%
%   make_figures(out, a_true, scenario, out_dir)
%
%   Produces (thesis style: role colors, grid off, legend northoutside) and
%   saves as PNG into out_dir (filenames carry the scenario so a multi-
%   scenario verify run does not overwrite):
%       fig1_gain_<scenario>.png      a_hat vs a_true (a_x and a_z)
%       fig2_tracking_<scenario>.png  per-axis tracking error (p_m - p_d)
%
%   Inputs:
%       out      - run_simulation output (ekf_out = [a_x a_y a_z h_bar])
%       a_true   - [N x3] physics ground-truth gain [x y z] (um/pN)
%       scenario - 'h50' | 'h10' | 'ramp2p7'
%       out_dir  - output directory (created if missing)
%
%   Note: the standalone controller has no diag output (decision 6), so
%   these figures use only the logged signals -- a_hat vs a_true and the
%   tracking error. The raw a_xm / IIR-LP / disturbance overlays of the
%   mother repo's figure set need diag and are intentionally omitted.
%
%   See also: verify_standalone, run_simulation

    if ~exist(out_dir, 'dir'); mkdir(out_dir); end

    COL_REF = [0 0.6 0];      % green : True
    COL_OUT = [0.8 0 0];      % red   : Estimated
    COL_ERR = [0 0.2 0.8];    % blue  : Error
    FS = 18; LFS = 14; LW = 2.0; LR = 3.0; LO = 2.0;

    t = out.tout;
    Ts = t(2) - t(1);
    n_warm = round(out.meta.params.traj.t_hold / Ts);
    idx = false(numel(t), 1); idx(n_warm+1:end) = true;
    t0 = t(find(idx, 1, 'first')); t1 = max(t);
    is_ramp = strcmpi(scenario, 'ramp2p7');
    a_hat = out.ekf_out(:, 1:3);

    % ================= FIG 1 : gain estimation (a_x, a_z) =================
    f1 = figure('Position', [80 80 1100 720], 'Color', 'w', ...
                'Name', sprintf('6-state %s : gain', scenario), 'NumberTitle', 'off');
    tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    cols = [1 3]; lbl = {'a_x', 'a_z'};
    for r = 1:2
        c = cols(r);
        nexttile; hold on;
        plot(t(idx), a_true(idx, c), '-', 'Color', COL_REF, 'LineWidth', LR, 'DisplayName', 'True');
        plot(t(idx), a_hat(idx, c),  '-', 'Color', COL_OUT, 'LineWidth', LO, 'DisplayName', 'Estimated');
        rel = (a_hat(idx, c) - a_true(idx, c)) ./ a_true(idx, c);
        b = 100 * mean(rel); sd = 100 * std(rel);
        if is_ramp
            title(sprintf('%s:   mean rel-err %+.2f%%     std %.2f%%', lbl{r}, b, sd), ...
                  'FontSize', FS, 'FontWeight', 'bold');
            lo = min(a_true(idx, c)) * 0.70; hi = max(a_true(idx, c)) * 1.15;
        else
            title(sprintf('%s:   bias %+.2f%%     std %.2f%%', lbl{r}, b, sd), ...
                  'FontSize', FS, 'FontWeight', 'bold');
            tm = mean(a_true(idx, c)); lo = 0.6 * tm; hi = 1.4 * tm;
        end
        ylabel(sprintf('%s  (\\mum/pN)', lbl{r}), 'FontSize', FS, 'FontWeight', 'bold');
        ylim([lo hi]); xlim([t0 t1]);
        if r == 2, xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold'); end
        if r == 1
            legend('Location', 'northoutside', 'Orientation', 'horizontal', ...
                   'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'off');
        end
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', LW, 'Box', 'on'); grid off;
    end
    disable_toolbars(f1);
    exportgraphics(f1, fullfile(out_dir, sprintf('fig1_gain_%s.png', scenario)), 'Resolution', 150);

    % ================= FIG 2 : tracking error (x, y, z) =================
    err = (out.p_m_out - out.p_d_out) * 1e3;     % nm
    f2 = figure('Position', [80 80 1100 920], 'Color', 'w', ...
                'Name', sprintf('6-state %s : tracking', scenario), 'NumberTitle', 'off');
    tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    ymax = max(max(abs(err(idx, :)))); axn = {'x', 'y', 'z'};
    for k = 1:3
        nexttile; hold on;
        plot(t(idx), err(idx, k), '-', 'Color', COL_ERR, 'LineWidth', 0.6, 'DisplayName', 'error');
        mv = mean(err(idx, k)); sv = std(err(idx, k));
        title(sprintf('%s:  mean %+.2f / std %.2f nm', axn{k}, mv, sv), ...
              'FontSize', FS, 'FontWeight', 'bold');
        ylabel(sprintf('%s error (nm)', axn{k}), 'FontSize', FS, 'FontWeight', 'bold');
        ylim([-ymax ymax] * 1.05); xlim([t0 t1]);
        if k == 3, xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold'); end
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', LW, 'Box', 'on'); grid off;
    end
    disable_toolbars(f2);
    exportgraphics(f2, fullfile(out_dir, sprintf('fig2_tracking_%s.png', scenario)), 'Resolution', 150);

    close(f1); close(f2);
end


function disable_toolbars(fig)
    axs = findall(fig, 'Type', 'Axes');
    for ii = 1:numel(axs); axs(ii).Toolbar.Visible = 'off'; end
end
