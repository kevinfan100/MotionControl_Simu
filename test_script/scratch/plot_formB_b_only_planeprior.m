% FORK OF test_script/scratch/plot_formB_c3_prior028_cellplant.m @ untracked-2026-08-05 |
% PURPOSE: single-free-parameter identifiability arm -- TRUE z curve is the p = 1 member of
% the page-2 constructed family (b, p, ws) = (0.13, 1, 1) while the controller carries the
% rigid-plane anchors and frees ONLY b (p and w_s locked AT THEIR TRUE VALUES).  Zero
% representation error, zero b-p coupling, zero b-w_s degeneracy: the only obstacle left is
% the prior width itself.  Figure for formB_selected_figs.tex | EXPIRES: when the arm is
% formally chartered or the prior question is settled | 產線改動不會自動跟上
%
% NOTE the plant curve b = 0.13 is a CONSTRUCTED test curve (page-2 hand-added family,
% all passing a_bar = 0.70 at gap 0.3 R), NOT a fitted cell boundary -- physical cell fits
% sit at b = 0.5-1.06.  The page therefore tests the ESTIMATOR's prior structure, not the
% physics of curved boundaries (pages 3-4 already cover that).
%
% Lab figure convention (.claude/rules/figure-style.md): no grid, no title, box on,
% legend northoutside horizontal, True = red / Estimate = blue / Measured = light blue,
% FontSize 18 bold, exportgraphics Resolution 150, statistics to the console only.

here = fileparts(mfilename('fullpath'));
proj = fullfile(here, '..', '..');
addpath(genpath(fullfile(proj, 'model')));
addpath(fullfile(proj, 'test_script', 'integration'));
fig_dir = fullfile(proj, 'reference', 'eq17_analysis', 'derivation', 'figures');

AX_Z         = 3;
NM           = 1e3;
B_ANCHOR     = 9/8;            % rigid-plane reflection coefficient = the seed
ERR_BAND_PCT = 2;
SEEDS_SHOWN  = [7 11];
PLANT_LAW    = struct('b', 0.13, 'p', 1, 'ws', 1);   % page-2 p = 1 constructed curve

FS         = 18;
RED        = 'r';
BLUE       = 'b';
LIGHT_BLUE = [0.45 0.72 0.95];
BAND_HUE   = [0.78 0.85 0.95];
BUD_HUE    = [0.30 0.30 0.30];

mat_file = fullfile(proj, 'test_results', 'run_formB_ws_bonly_planeprior.mat');
if exist(mat_file, 'file')
    S = load(mat_file);  out = S.out;
else
    o = struct();
    o.seeds = SEEDS_SHOWN;
    o.plant_gain_law = PLANT_LAW;
    % b free; p and w_s locked AT TRUTH -> the single-parameter test
    o.ctrl_const_override = struct('lock_b', false, 'lock_p', true, 'lock_ws', true);
    out = run_formB_ws(o);
end

tb = [out.cfg.t_hold, out.cfg.t_hold + out.cfg.t_descend_override];

fprintf('\n=== single-free-parameter arm: TRUE b = %.3f, seed b = %.4f ===\n', ...
        PLANT_LAW.b, B_ANCHOR);

for target = SEEDS_SHOWN
    q = find(out.seeds == target, 1);
    r = out.runs{q};

    t    = r.tout(:);
    aT   = r.a_true_out(:, AX_Z);
    aH   = r.a_hat_out(:, AX_Z);
    aXM  = r.a_xm_out(:, AX_Z);
    errz = 100 * (aH - aT) ./ aT;

    bH   = r.b_hat_out(:, AX_Z);
    sPb  = r.P_b_out(:, AX_Z);           % driver stores SQRT of the posterior variance

    % P[0] traversal budget, run-time form of the house criterion
    %   (b_hat[k] - b_hat[0])^2 <= P_bb[0] - P_bb[k]
    % masked below 2 % of P[0]: there both numerator and denominator are still
    % at round-off and the ratio is a 0/0 artefact, not a budget statement
    denom = sPb(1)^2 - sPb.^2;
    ratio = nan(size(t));
    ok    = denom > 0.02 * sPb(1)^2;
    ratio(ok) = (bH(ok) - bH(1)).^2 ./ denom(ok);

    % how far the truth sits from the seed in prior widths
    n_sigma = abs(B_ANCHOR - PLANT_LAW.b) / sPb(1);

    % panels 1-2 auto-widen: the mismatch magnitude is not known a priori
    a_lim   = 5 * ceil((max(aT * NM) + 2) / 5);
    err_lim = max(14, 5 * ceil((max(abs(errz)) + 1) / 5));
    if err_lim > 14
        fprintf('NOTE seed %d: error panel widened to +-%g %% (house +-14).\n', target, err_lim);
    end

    fig = figure('Color', 'w', 'Position', [80 40 900 1280]);

    % --- 1. gain -----------------------------------------------------------
    ax1 = subplot(4, 1, 1); hold(ax1, 'on');
    hx = plot(ax1, t, aXM * NM, 'Color', LIGHT_BLUE, 'LineWidth', 0.5);
    ht = plot(ax1, t, aT  * NM, RED,  'LineWidth', 2.6);
    he = plot(ax1, t, aH  * NM, BLUE, 'LineWidth', 2.0);
    ylabel(ax1, 'a_z  [nm/pN]', 'FontSize', FS, 'FontWeight', 'bold');
    legend([ht he hx], {'a_{true} (b = 0.13 plant)', 'a_{hat}', 'a_{wm} (raw)'}, ...
           'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS - 4);
    ylim(ax1, [0 max(24, a_lim)]); style_ax(ax1, FS); xlim(ax1, [t(1) t(end)]); add_phase(ax1, tb);

    % --- 2. gain error -----------------------------------------------------
    ax2 = subplot(4, 1, 2); hold(ax2, 'on');
    h0  = plot(ax2, t, zeros(size(t)), RED, 'LineWidth', 2.0);
    he2 = plot(ax2, t, errz, BLUE, 'LineWidth', 2.0);
    plot(ax2, t,  ERR_BAND_PCT * ones(size(t)), 'k--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
    plot(ax2, t, -ERR_BAND_PCT * ones(size(t)), 'k--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
    ylabel(ax2, 'a_z error  [%]', 'FontSize', FS, 'FontWeight', 'bold');
    legend([h0 he2], {'zero', '(a_{hat}-a_{true})/a_{true}'}, ...
           'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS - 4);
    ylim(ax2, err_lim * [-1 1]); style_ax(ax2, FS); xlim(ax2, [t(1) t(end)]); add_phase(ax2, tb);

    % --- 3. the single free parameter, with its own claimed width ----------
    ax3 = subplot(4, 1, 3); hold(ax3, 'on');
    tf  = [t; flipud(t)];
    bf  = [bH + sPb; flipud(bH - sPb)];
    hbd = fill(ax3, tf, bf, BAND_HUE, 'EdgeColor', 'none', 'FaceAlpha', 0.85);
    hbh = plot(ax3, t, bH, BLUE, 'LineWidth', 2.2);
    hsd = plot(ax3, t, B_ANCHOR * ones(size(t)), 'k--', 'LineWidth', 1.6);
    htr = plot(ax3, t, PLANT_LAW.b * ones(size(t)), RED, 'LineStyle', '--', 'LineWidth', 2.0);
    ylabel(ax3, 'b  [-]', 'FontSize', FS, 'FontWeight', 'bold');
    legend([htr hbh hbd hsd], ...
           {'true b = 0.13', 'b_{hat}', ...
            sprintf('\\pm sqrt(P_{bb}) = %.4f', sPb(1)), 'plane anchor 9/8'}, ...
           'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS - 7);
    ylim(ax3, [0 1.35]); style_ax(ax3, FS); xlim(ax3, [t(1) t(end)]); add_phase(ax3, tb);

    % --- 4. P[0] traversal budget -----------------------------------------
    ax4 = subplot(4, 1, 4); hold(ax4, 'on');
    need = nan(size(t));
    need(ok) = (B_ANCHOR - PLANT_LAW.b)^2 ./ denom(ok);   % same mask as the realised trace
    hnd = plot(ax4, t, need, RED, 'LineStyle', ':', 'LineWidth', 2.2);
    hrt = plot(ax4, t, ratio, 'Color', BUD_HUE, 'LineWidth', 2.2);
    hlm = plot(ax4, t, ones(size(t)), RED, 'LineWidth', 2.0);
    set(ax4, 'YScale', 'log');
    ylabel(ax4, 'P[0] budget ratio', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel(ax4, 'time  [s]', 'FontSize', FS, 'FontWeight', 'bold');
    legend([hlm hrt hnd], {'budget limit = 1', 'realised (b_{hat}-b_0)^2/\DeltaP', ...
           'needed to reach truth'}, 'Location', 'northoutside', ...
           'Orientation', 'horizontal', 'FontSize', FS - 7);
    ylim(ax4, [1e-1 1e6]); set(ax4, 'YTick', 10.^(0:2:6));
    style_ax(ax4, FS); xlim(ax4, [t(1) t(end)]); add_phase(ax4, tb);

    outp = fullfile(fig_dir, sprintf('formB_bonly_planeprior_seed%d.png', target));
    drawnow;
    exportgraphics(fig, outp, 'Resolution', 150);
    close(fig);

    m = out.metrics.rows(q, :);
    fprintf('FIGURE saved: %s\n', outp);
    fprintf('  seed %d: descent peak %.2f %%, osc RMS %.2f %%, hold mean %+.2f %%\n', ...
            target, m(1), m(2), m(3));
    fprintf('  b_hat  %.4f -> %.4f   (moved %.4f; truth needs %.4f)\n', ...
            bH(1), bH(end), abs(bH(end) - bH(1)), abs(B_ANCHOR - PLANT_LAW.b));
    fprintf('  sqrt(P_bb)  %.4f -> %.4f   truth sits %.1f prior widths from the seed\n', ...
            sPb(1), sPb(end), n_sigma);
    fprintf('  budget ratio realised %.3f (house limit 1) | needed to reach truth %.1f\n', ...
            m(4), (B_ANCHOR - PLANT_LAW.b)^2 / max(sPb(1)^2 - sPb(end)^2, eps));
end


function style_ax(ax, FS)
    set(ax, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 1.3, 'Box', 'on');
    grid(ax, 'off');
    ax.Toolbar = [];
end

function add_phase(ax, tb)
    yl = ylim(ax);
    for x = tb
        plot(ax, [x x], yl, '--', 'Color', [0.55 0.55 0.55], 'LineWidth', 1.0, ...
             'HandleVisibility', 'off');
    end
    ylim(ax, yl);
end
