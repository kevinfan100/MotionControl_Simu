% FORK OF test_script/scratch/plot_formB_ws_arm1.m @ untracked-2026-07-31 (itself a fork of
% test_script/integration/plot_5state_powerlaw.m @ 9a082e3) | PURPOSE: three-panel figures for
% the Form B lock-ladder configs P2 / P3, panel (c) generalized to an arbitrary parameter set |
% EXPIRES: when the Form B lock ladder is decided | 產線改動不會自動跟上
%
% plot_formB_ws_compare.m -- seed-7 / seed-11 figures for the whole lock ladder.
%   arm1 (doc c1): w_s locked at 1, estimate (b, p)   -- 3 panels
%   P2   (doc c2): lock_p = true,   estimate (b, w_s) -- 4 panels
%   P3   (doc c3): nothing locked,  estimate (b, p, w_s) -- 4 panels
%   Panels (a) and (b) are identical to the arm-1 figures; panel (c) is driven by a spec
%   list so one routine serves both configs (and any future lock combination).
%
%   Lab figure convention (.claude/rules/figure-style.md): no grid, no title, box on,
%   legend northoutside horizontal, True = red / Estimate = blue / Measured = light blue,
%   FontSize 18 bold, exportgraphics Resolution 150, statistics to the console only.
%   Panel (c) carries several estimated parameters at once, so it departs from the
%   three-role colour rule: b_hat keeps Estimate blue, the others take separate hues.
%   Every panel-(c) series is DIAGNOSTIC ONLY -- none is a controller output.
%
%   Outputs (test_results/, gitignored); doc copies keep the c1/c2/c3 names:
%       formB_arm1_seed7.png  formB_arm1_seed11.png   -> figures/formB_c1_seed*.png
%       formB_p2_seed7.png    formB_p2_seed11.png     -> figures/formB_c2_seed*.png
%       formB_p3_seed7.png    formB_p3_seed11.png     -> figures/formB_c3_seed*.png
%
%   DRIVER TAG COLLISION (interface wart, not fixed here): run_formB_ws builds its arm tag
%   from opts only, so any ctrl_const_override arm still saves as run_formB_ws_t1_y2on.mat
%   and overwrites the arm-1 file. This script therefore reads its own collision-free
%   copies (run_formB_ws_p2.mat / _p3.mat); if it has to regenerate them it says so, and
%   arm 1 must then be re-run to restore its .mat.

here = fileparts(mfilename('fullpath'));
proj = fullfile(here, '..', '..');
res_dir = fullfile(proj, 'test_results');

AX_Z         = 3;        % wall-normal axis (perpendicular)
NM           = 1e3;      % um/pN -> nm/pN display scale
B_ANCHOR     = 9/8;      % derived far-field reflection coefficient (b seed)
P_ANCHOR     = 1;        % derived far-field power (p seed); also the true wall w_s
ERR_BAND_PCT = 2;        % +-2 % reference lines on the error panel
WS_DEV_YLIM  = [-26 8];  % [% of R] must contain the full seed-11 drag, which
                         % deepened to -24 % under the envelope priors; keep it
                         % wide enough that nothing is silently clipped
SEEDS_SHOWN  = [7 11];

FS         = 18;
RED        = 'r';
BLUE       = 'b';
LIGHT_BLUE = [0.45 0.72 0.95];   % Measured / raw readout
P_HUE      = [0.85 0.33 0.10];   % p_hat
WS_HUE     = [0.13 0.55 0.13];   % ws_hat

% ---- config table: file, regeneration override, panel-(c) series ----
ov_p2 = struct('lock_b', false, 'lock_p', true,  'lock_ws', false, 'Pf_ws_std', 0.111);
ov_p3 = struct('lock_b', false, 'lock_p', false, 'lock_ws', false, 'Pf_ws_std', 0.111);

cfg_list = { ...
    struct('key', 'arm1', 'var', 'out', 'file', 'run_formB_ws_t1_y2on.mat', 'ov', struct(), ...
           'ws_panel', false, ...
           'series', {{'b_hat_out', 'b_{hat}', BLUE; 'p_hat_out', 'p_{hat}', P_HUE}}), ...
    struct('key', 'p2', 'var', 'out_p2', 'file', 'run_formB_ws_p2.mat', 'ov', ov_p2, ...
           'ws_panel', true, ...
           'series', {{'b_hat_out', 'b_{hat}', BLUE; 'ws_hat_out', 'ws_{hat}', WS_HUE}}), ...
    struct('key', 'p3', 'var', 'out_p3', 'file', 'run_formB_ws_p3.mat', 'ov', ov_p3, ...
           'ws_panel', true, ...
           'series', {{'b_hat_out', 'b_{hat}', BLUE; 'p_hat_out', 'p_{hat}', P_HUE; ...
                       'ws_hat_out', 'ws_{hat}', WS_HUE}}) };

for c = 1:numel(cfg_list)
    C = cfg_list{c};
    mat_file = fullfile(res_dir, C.file);
    if exist(mat_file, 'file')
        S = load(mat_file);  out = S.(C.var);
    else
        fprintf('%s missing -- regenerating config %s (this OVERWRITES run_formB_ws_t1_y2on.mat; re-run arm 1 afterwards).\n', ...
                mat_file, upper(C.key));
        o = struct(); o.ctrl_const_override = C.ov;
        out = run_formB_ws(o);
        S = struct(C.var, out);  save(mat_file, '-struct', 'S');
    end
    seeds = out.seeds;
    cfg   = out.cfg;
    tb    = [cfg.t_hold, cfg.t_hold + cfg.t_descend_override];   % phase boundaries [s]

    for target = SEEDS_SHOWN
        q = find(seeds == target, 1);
        assert(~isempty(q), 'plot_formB_ws_compare:seedMissing', 'seed %d not in config %s.', ...
               target, upper(C.key));
        r = out.runs{q};

        t    = r.tout(:);
        aT   = r.a_true_out(:, AX_Z);      % [um/pN] true motion gain
        aH   = r.a_hat_out(:, AX_Z);       % [um/pN] EKF estimate
        aXM  = r.a_xm_out(:, AX_Z);        % [um/pN] raw gain readout
        errz = 100 * (aH - aT) ./ aT;      % [%]

        % configs that estimate w_s get a dedicated deviation panel below (c)
        n_row = 3 + double(C.ws_panel);
        fig = figure('Color', 'w', 'Position', [80 40 900 1000 + 280 * double(C.ws_panel)]);

        % (a) gain: true / estimate / raw readout
        ax1 = subplot(n_row, 1, 1); hold(ax1, 'on');
        hx = plot(ax1, t, aXM * NM, 'Color', LIGHT_BLUE, 'LineWidth', 0.5);
        ht = plot(ax1, t, aT  * NM, RED,  'LineWidth', 2.6);
        he = plot(ax1, t, aH  * NM, BLUE, 'LineWidth', 2.0);
        ylabel(ax1, 'a_z  [nm/pN]', 'FontSize', FS, 'FontWeight', 'bold');
        legend([ht he hx], {'a_{true}', 'a_{hat}', 'a_{wm} (raw)'}, ...
               'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS - 4);
        ylim(ax1, [0 24]); style_ax(ax1, FS); xlim(ax1, [t(1) t(end)]); add_phase(ax1, tb);

        % (b) relative gain error
        ax2 = subplot(n_row, 1, 2); hold(ax2, 'on');
        h0  = plot(ax2, t, zeros(size(t)), RED, 'LineWidth', 2.0);
        he2 = plot(ax2, t, errz, BLUE, 'LineWidth', 2.0);
        plot(ax2, t,  ERR_BAND_PCT * ones(size(t)), 'k--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
        plot(ax2, t, -ERR_BAND_PCT * ones(size(t)), 'k--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
        ylabel(ax2, 'a_z error  [%]', 'FontSize', FS, 'FontWeight', 'bold');
        legend([h0 he2], {'zero', '(a_{hat}-a_{true})/a_{true}'}, ...
               'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS - 4);
        ylim(ax2, [-14 14]); style_ax(ax2, FS); xlim(ax2, [t(1) t(end)]); add_phase(ax2, tb);

        % (c) the free shape / wall parameters against their anchors
        ax3 = subplot(n_row, 1, 3); hold(ax3, 'on');
        nser = size(C.series, 1);
        hs   = gobjects(nser, 1);
        lbl  = cell(nser, 1);
        for s = 1:nser
            y = r.(C.series{s, 1})(:, AX_Z);
            hs(s)  = plot(ax3, t, y, 'Color', C.series{s, 3}, 'LineWidth', 1.8);
            lbl{s} = [C.series{s, 2} ' (diag only)'];
        end
        ha = plot(ax3, t, B_ANCHOR * ones(size(t)), 'k--', 'LineWidth', 1.2);
        plot(ax3, t, P_ANCHOR * ones(size(t)), 'k:', 'LineWidth', 1.4, 'HandleVisibility', 'off');
        ylabel(ax3, 'free params', 'FontSize', FS, 'FontWeight', 'bold');
        legend([hs; ha], [lbl; {'anchors 9/8, 1'}], 'Location', 'northoutside', ...
               'Orientation', 'horizontal', 'FontSize', FS - 7);
        ylim(ax3, [0.70 1.30]); style_ax(ax3, FS); xlim(ax3, [t(1) t(end)]); add_phase(ax3, tb);
        if ~C.ws_panel
            xlabel(ax3, 'time  [s]', 'FontSize', FS, 'FontWeight', 'bold');
        end

        % (d) wall-position deviation, the quantity the drag lives in
        if C.ws_panel
            ws_dev = 100 * (r.ws_hat_out(:, AX_Z) - P_ANCHOR);   % [% of R]
            ax4 = subplot(n_row, 1, 4); hold(ax4, 'on');
            hz  = plot(ax4, t, zeros(size(t)), RED, 'LineWidth', 2.0);
            hd  = plot(ax4, t, ws_dev, 'Color', WS_HUE, 'LineWidth', 2.0);
            ylabel(ax4, '(ws_{hat}-1)  [% of R]', 'FontSize', FS, 'FontWeight', 'bold');
            xlabel(ax4, 'time  [s]', 'FontSize', FS, 'FontWeight', 'bold');
            legend([hz hd], {'true wall (ws = 1)', 'ws_{hat} deviation (diag only)'}, ...
                   'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS - 6);
            ylim(ax4, WS_DEV_YLIM); style_ax(ax4, FS); xlim(ax4, [t(1) t(end)]);
            add_phase(ax4, tb);
        end

        outp = fullfile(res_dir, sprintf('formB_%s_seed%d.png', C.key, target));
        drawnow;   % let the layout settle: without it the first export of a run
                   % crops taller and the doc pages show mismatched figure sizes
        exportgraphics(fig, outp, 'Resolution', 150);
        close(fig);

        m  = out.metrics.rows(q, :);
        ws = r.ws_hat_out(:, AX_Z);
        fprintf('FIGURE saved: %s\n', outp);
        fprintf('CAPTION: Form B config %s seed %d, z axis: gain tracking, relative error, and the free parameters over the hold-descend-oscillate-hold scenario.\n', ...
                upper(C.key), target);
        fprintf('  seed %d: descent peak %.3f %%, osc RMS %.3f %%, hold mean %+.3f %%, b budget %.3f, p budget %.3f\n', ...
                target, m(1), m(2), m(3), m(4), m(5));
        fprintf('  ws_hat range [%.4f %.4f], final %.4f (true wall = %g, departure %+.1f %%)\n', ...
                min(ws), max(ws), ws(end), P_ANCHOR, 100 * (ws(end) - P_ANCHOR) / P_ANCHOR);
        fprintf('  a_wm raw clipped above %g nm/pN on %.1f %% of samples\n', 24, 100 * mean(aXM * NM > 24));
    end
end


function style_ax(ax, FS)
    set(ax, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 1.3, 'Box', 'on');
    grid(ax, 'off');
    ax.Toolbar = [];   % keep the interactive toolbar out of exportgraphics
end

function add_phase(ax, tb)
    yl = ylim(ax);
    for x = tb
        plot(ax, [x x], yl, '--', 'Color', [0.55 0.55 0.55], 'LineWidth', 1.0, ...
             'HandleVisibility', 'off');
    end
    ylim(ax, yl);
end
