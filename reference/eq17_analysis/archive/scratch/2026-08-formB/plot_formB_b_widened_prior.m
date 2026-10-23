% FORK OF test_script/scratch/plot_formB_b_only_planeprior.m @ untracked-2026-08-05 |
% PURPOSE: same single-free-parameter arm (plant = constructed p = 1 curve b = 0.13, p and
% w_s locked at truth), but the b prior is WIDENED so the truth is inside it.  Answers the
% question the plane-prior page cannot: once the prior stops forbidding the move, does the
% filter actually walk b to the truth, and what does the widening cost?
% | EXPIRES: when the b-prior width is derived from a declared boundary family
% | 產線改動不會自動跟上
%
% ⚠ THE WIDTHS HERE ARE ASSUMED, NOT DERIVED (user instruction 2026-08-05: "先假設比我的
% 曲線寬一點的範圍").  They are a provisional ladder chosen to bracket the distance the
% estimate must travel, |9/8 - 0.13| = 0.995 -- NOT a physical prior.  A committed prior
% width must come from a declared boundary family (see formB_mainline_review_2026-08-05.md).
%
% Caveat recorded on the face of it: a linear Gaussian prior on b with mean 9/8 and
% sqrt(P) = 1.0 places ~13 % of its mass at b < 0, which is unphysical (b is a length).
% A wide prior on b probably wants a log parametrisation; that is a state-level change and
% is NOT attempted here.
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
B_TRUE       = 0.13;
ERR_BAND_PCT = 2;
SEEDS_SHOWN  = [7 11];
PLANT_LAW    = struct('b', B_TRUE, 'p', 1, 'ws', 1);
WIDTH_LADDER = [0.10 0.25 0.50 1.00];    % assumed, see header
WIDTH_SHOWN  = 1.00;                     % the page figure

FS         = 18;
RED        = 'r';
BLUE       = 'b';
LIGHT_BLUE = [0.45 0.72 0.95];
BAND_HUE   = [0.78 0.85 0.95];
BUD_HUE    = [0.30 0.30 0.30];

fprintf('\n=== widened-b-prior ladder (plant b = %.3f, seed %.4f, travel %.4f) ===\n', ...
        B_TRUE, B_ANCHOR, abs(B_ANCHOR - B_TRUE));

% NOTE run_formB_ws tags its .mat by ARM FLAGS ONLY -- Pf_b_std is not in the tag, so every
% width in the ladder (and the narrow-prior page-6 arm) writes the SAME
% run_formB_ws_t1_y2on_plantlaw.mat.  This arm therefore keeps its own copy; the page-6
% plotter's cache of that name is stale after this script runs and must be regenerated.
cache = fullfile(proj, 'test_results', 'formB_bwide_shown.mat');

out_shown = [];
if exist(cache, 'file')
    C = load(cache);  out_shown = C.out_shown;
    fprintf('  (cached shown-width arm loaded; ladder skipped)\n');
end
for w = WIDTH_LADDER * double(isempty(out_shown))
    o = struct();
    o.seeds          = SEEDS_SHOWN;
    o.plant_gain_law = PLANT_LAW;
    o.ctrl_const_override = struct('lock_b', false, 'lock_p', true, 'lock_ws', true, ...
                                   'Pf_b_std', w);
    out = run_formB_ws(o);

    bf = zeros(numel(SEEDS_SHOWN), 1);
    for q = 1:numel(SEEDS_SHOWN)
        bf(q) = out.runs{q}.b_hat_out(end, AX_Z);
    end
    m = out.metrics.rows;
    fprintf(['  sqrt(P_bb) = %.2f (truth at %.1f sigma) : b_hat final %.3f / %.3f | ' ...
             'desc %.2f/%.2f %% hold %+.2f/%+.2f %% | budget %.2f/%.2f\n'], ...
            w, abs(B_ANCHOR - B_TRUE) / w, bf(1), bf(2), ...
            m(1, 1), m(2, 1), m(1, 3), m(2, 3), m(1, 4), m(2, 4));

    if w == WIDTH_SHOWN; out_shown = out; end
end

if ~exist(cache, 'file')
    fprintf(['\n  budget criterion reading: to move %.3f lawfully the prior must satisfy\n' ...
             '  sqrt(P[0]) >= the travel itself, i.e. sqrt(P_bb) >= %.2f\n'], ...
            abs(B_ANCHOR - B_TRUE), abs(B_ANCHOR - B_TRUE));
    save(cache, 'out_shown');
end

% ---------------- page figure at the shown width ----------------
out = out_shown;
tb  = [out.cfg.t_hold, out.cfg.t_hold + out.cfg.t_descend_override];

for target = SEEDS_SHOWN
    q = find(out.seeds == target, 1);
    r = out.runs{q};

    t    = r.tout(:);
    aT   = r.a_true_out(:, AX_Z);
    aH   = r.a_hat_out(:, AX_Z);
    aXM  = r.a_xm_out(:, AX_Z);
    errz = 100 * (aH - aT) ./ aT;
    bH   = r.b_hat_out(:, AX_Z);
    sPb  = r.P_b_out(:, AX_Z);

    err_lim = max(14, 5 * ceil((max(abs(errz)) + 1) / 5));
    a_lim   = 5 * ceil((max(aT * NM) + 2) / 5);

    fig = figure('Color', 'w', 'Position', [80 40 900 980]);

    ax1 = subplot(3, 1, 1); hold(ax1, 'on');
    hx = plot(ax1, t, aXM * NM, 'Color', LIGHT_BLUE, 'LineWidth', 0.5);
    ht = plot(ax1, t, aT  * NM, RED,  'LineWidth', 2.6);
    he = plot(ax1, t, aH  * NM, BLUE, 'LineWidth', 2.0);
    ylabel(ax1, 'a_z  [nm/pN]', 'FontSize', FS, 'FontWeight', 'bold');
    legend([ht he hx], {'a_{true} (b = 0.13 plant)', 'a_{hat}', 'a_{wm} (raw)'}, ...
           'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS - 4);
    ylim(ax1, [0 max(24, a_lim)]); style_ax(ax1, FS);
    xlim(ax1, [t(1) t(end)]); add_phase(ax1, tb);

    ax2 = subplot(3, 1, 2); hold(ax2, 'on');
    h0  = plot(ax2, t, zeros(size(t)), RED, 'LineWidth', 2.0);
    he2 = plot(ax2, t, errz, BLUE, 'LineWidth', 2.0);
    plot(ax2, t,  ERR_BAND_PCT * ones(size(t)), 'k--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
    plot(ax2, t, -ERR_BAND_PCT * ones(size(t)), 'k--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
    ylabel(ax2, 'a_z error  [%]', 'FontSize', FS, 'FontWeight', 'bold');
    legend([h0 he2], {'zero', '(a_{hat}-a_{true})/a_{true}'}, ...
           'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS - 4);
    ylim(ax2, err_lim * [-1 1]); style_ax(ax2, FS);
    xlim(ax2, [t(1) t(end)]); add_phase(ax2, tb);

    ax3 = subplot(3, 1, 3); hold(ax3, 'on');
    tf  = [t; flipud(t)];
    bf2 = [bH + sPb; flipud(bH - sPb)];
    hbd = fill(ax3, tf, bf2, BAND_HUE, 'EdgeColor', 'none', 'FaceAlpha', 0.85);
    hbh = plot(ax3, t, bH, BLUE, 'LineWidth', 2.2);
    hsd = plot(ax3, t, B_ANCHOR * ones(size(t)), 'k--', 'LineWidth', 1.6);
    htr = plot(ax3, t, B_TRUE * ones(size(t)), RED, 'LineStyle', '--', 'LineWidth', 2.0);
    plot(ax3, t, zeros(size(t)), 'Color', [0.6 0.6 0.6], 'LineWidth', 1.0, ...
         'HandleVisibility', 'off');
    ylabel(ax3, 'b  [-]', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel(ax3, 'time  [s]', 'FontSize', FS, 'FontWeight', 'bold');
    legend([htr hbh hbd hsd], {'true b = 0.13', 'b_{hat}', ...
            sprintf('\\pm sqrt(P_{bb}) = %.2f (ASSUMED)', sPb(1)), 'plane anchor 9/8'}, ...
           'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS - 7);
    ylim(ax3, [-0.4 2.4]); style_ax(ax3, FS);
    xlim(ax3, [t(1) t(end)]); add_phase(ax3, tb);

    outp = fullfile(fig_dir, sprintf('formB_bwide_seed%d.png', target));
    drawnow;
    exportgraphics(fig, outp, 'Resolution', 150);
    close(fig);

    m = out.metrics.rows(q, :);
    fprintf('FIGURE saved: %s\n', outp);
    fprintf('  seed %d: desc %.2f %%, osc %.2f %%, hold %+.2f %% | b_hat %.4f -> %.4f\n', ...
            target, m(1), m(2), m(3), bH(1), bH(end));
    fprintf('  sqrt(P_bb) %.3f -> %.3f | budget realised %.2f\n', sPb(1), sPb(end), m(4));
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
