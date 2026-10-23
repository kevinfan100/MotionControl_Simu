% FORK OF test_script/scratch/plot_formB_ws_compare.m @ bdd7a67 | PURPOSE: config-3
% (estimate b, p, w_s) rerun with the calibrated-grade w_s prior 0.028 instead of the
% house 0.111, same 4-panel layout as doc page c3, seeds 7/11, for the figure-digest
% doc formB_selected_figs.tex | EXPIRES: when the digest doc is retired | 產線改動不會自動跟上
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
B_ANCHOR     = 9/8;
P_ANCHOR     = 1;
ERR_BAND_PCT = 2;
WS_DEV_YLIM  = [-26 8];        % kept identical to doc c3 so the two pages compare 1:1
SEEDS_SHOWN  = [7 11];
PRIOR_WS     = 0.028;          % calibrated-grade sqrt(P_ws)[0] (doc prior-sweep pages)

FS         = 18;
RED        = 'r';
BLUE       = 'b';
LIGHT_BLUE = [0.45 0.72 0.95];
P_HUE      = [0.85 0.33 0.10];
WS_HUE     = [0.13 0.55 0.13];

o = struct();
o.seeds = SEEDS_SHOWN;
o.ctrl_const_override = struct('lock_b', false, 'lock_p', false, 'lock_ws', false, ...
                               'Pf_ws_std', PRIOR_WS);
out = run_formB_ws(o);

series = {'b_hat_out', 'b_{hat}', BLUE; 'p_hat_out', 'p_{hat}', P_HUE; ...
          'ws_hat_out', 'ws_{hat}', WS_HUE};
tb = [out.cfg.t_hold, out.cfg.t_hold + out.cfg.t_descend_override];

for target = SEEDS_SHOWN
    q = find(out.seeds == target, 1);
    r = out.runs{q};

    t    = r.tout(:);
    aT   = r.a_true_out(:, AX_Z);
    aH   = r.a_hat_out(:, AX_Z);
    aXM  = r.a_xm_out(:, AX_Z);
    errz = 100 * (aH - aT) ./ aT;

    fig = figure('Color', 'w', 'Position', [80 40 900 1280]);

    ax1 = subplot(4, 1, 1); hold(ax1, 'on');
    hx = plot(ax1, t, aXM * NM, 'Color', LIGHT_BLUE, 'LineWidth', 0.5);
    ht = plot(ax1, t, aT  * NM, RED,  'LineWidth', 2.6);
    he = plot(ax1, t, aH  * NM, BLUE, 'LineWidth', 2.0);
    ylabel(ax1, 'a_z  [nm/pN]', 'FontSize', FS, 'FontWeight', 'bold');
    legend([ht he hx], {'a_{true}', 'a_{hat}', 'a_{wm} (raw)'}, ...
           'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS - 4);
    ylim(ax1, [0 24]); style_ax(ax1, FS); xlim(ax1, [t(1) t(end)]); add_phase(ax1, tb);

    ax2 = subplot(4, 1, 2); hold(ax2, 'on');
    h0  = plot(ax2, t, zeros(size(t)), RED, 'LineWidth', 2.0);
    he2 = plot(ax2, t, errz, BLUE, 'LineWidth', 2.0);
    plot(ax2, t,  ERR_BAND_PCT * ones(size(t)), 'k--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
    plot(ax2, t, -ERR_BAND_PCT * ones(size(t)), 'k--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
    ylabel(ax2, 'a_z error  [%]', 'FontSize', FS, 'FontWeight', 'bold');
    legend([h0 he2], {'zero', '(a_{hat}-a_{true})/a_{true}'}, ...
           'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS - 4);
    ylim(ax2, [-14 14]); style_ax(ax2, FS); xlim(ax2, [t(1) t(end)]); add_phase(ax2, tb);

    ax3 = subplot(4, 1, 3); hold(ax3, 'on');
    nser = size(series, 1);
    hs   = gobjects(nser, 1);
    lbl  = cell(nser, 1);
    for s = 1:nser
        y = r.(series{s, 1})(:, AX_Z);
        hs(s)  = plot(ax3, t, y, 'Color', series{s, 3}, 'LineWidth', 1.8);
        lbl{s} = [series{s, 2} ' (diag only)'];
    end
    ha = plot(ax3, t, B_ANCHOR * ones(size(t)), 'k--', 'LineWidth', 1.2);
    plot(ax3, t, P_ANCHOR * ones(size(t)), 'k:', 'LineWidth', 1.4, 'HandleVisibility', 'off');
    ylabel(ax3, 'free params', 'FontSize', FS, 'FontWeight', 'bold');
    legend([hs; ha], [lbl; {'anchors 9/8, 1'}], 'Location', 'northoutside', ...
           'Orientation', 'horizontal', 'FontSize', FS - 7);
    ylim(ax3, [0.70 1.30]); style_ax(ax3, FS); xlim(ax3, [t(1) t(end)]); add_phase(ax3, tb);

    ws_dev = 100 * (r.ws_hat_out(:, AX_Z) - P_ANCHOR);
    ax4 = subplot(4, 1, 4); hold(ax4, 'on');
    hz  = plot(ax4, t, zeros(size(t)), RED, 'LineWidth', 2.0);
    hd  = plot(ax4, t, ws_dev, 'Color', WS_HUE, 'LineWidth', 2.0);
    ylabel(ax4, '(ws_{hat}-1)  [% of R]', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel(ax4, 'time  [s]', 'FontSize', FS, 'FontWeight', 'bold');
    legend([hz hd], {'true wall (ws = 1)', 'ws_{hat} deviation (diag only)'}, ...
           'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS - 6);
    ylim(ax4, WS_DEV_YLIM); style_ax(ax4, FS); xlim(ax4, [t(1) t(end)]); add_phase(ax4, tb);

    outp = fullfile(fig_dir, sprintf('formB_c3_prior028_seed%d.png', target));
    drawnow;
    exportgraphics(fig, outp, 'Resolution', 150);
    close(fig);

    m  = out.metrics.rows(q, :);
    ws = r.ws_hat_out(:, AX_Z);
    fprintf('FIGURE saved: %s\n', outp);
    fprintf('  seed %d: descent peak %.3f %%, osc RMS %.3f %%, hold mean %+.3f %%, b budget %.3f, p budget %.3f\n', ...
            target, m(1), m(2), m(3), m(4), m(5));
    fprintf('  ws_hat range [%.4f %.4f], final %.4f (departure %+.1f %%)\n', ...
            min(ws), max(ws), ws(end), 100 * (ws(end) - 1));
    fprintf('  b_hat final %.4f, p_hat final %.4f\n', ...
            r.b_hat_out(end, AX_Z), r.p_hat_out(end, AX_Z));
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
