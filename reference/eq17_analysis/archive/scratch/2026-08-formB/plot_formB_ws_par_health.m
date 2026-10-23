% plot_formB_ws_par_health.m -- Stage 1a doc page c6: x/y gain-estimation
%   health before/after the parallel-axis law (par_law).
%   PURPOSE: the x/y filters used to run the PERPENDICULAR Form B law
%   (b=9/8, p=1) against the parallel truth c_para -- Goldman log at contact
%   pushes the best representation's origin inside the wall -- so their gain
%   estimates carried a systematic -22..-26 % hold bias and their ws walked
%   to ~0.6. After: driver-derived parallel package (b,p,ws0 locked, origin
%   rides the z-axis ws estimate one-way).
%   EXPIRES: superseded when a c-page regeneration ladder moves to
%   integration/.
%   Data: test_results/run_formB_ws_p2.mat  (out_p2, BEFORE: all-perp law)
%         test_results/run_formB_ws_p2_par.mat (out_p2_par, AFTER)
%   Style: .claude/rules/figure-style.md.

here = fileparts(mfilename('fullpath'));
proj = fullfile(here, '..', '..');
res_dir = fullfile(proj, 'test_results');

FS = 18; RED = 'r'; BLUE = 'b';
LIGHT_BLUE = [0.45 0.72 0.95];
SEEDS_SHOWN = [7 11];

SB = load(fullfile(res_dir, 'run_formB_ws_p2.mat'));      befo = SB.out_p2;
SA = load(fullfile(res_dir, 'run_formB_ws_p2_par.mat'));  afte = SA.out_p2_par;
seeds = befo.seeds;
tb = [befo.cfg.t_hold, befo.cfg.t_hold + befo.cfg.t_descend_override, ...
      befo.cfg.t_hold + befo.cfg.t_descend_override + befo.cfg.n_cycles/befo.cfg.frequency];

ax_lbl = {'a_x error  [%]', 'a_y error  [%]'};
for target = SEEDS_SHOWN
    q = find(seeds == target, 1);
    rb = befo.runs{q};  ra = afte.runs{q};
    t = rb.tout(:);

    fig = figure('Color', 'w', 'Position', [80 60 900 760]); drawnow;
    for ax = 1:2
        eb = 100 * (rb.a_hat_out(:,ax) - rb.a_true_out(:,ax)) ./ rb.a_true_out(:,ax);
        ea = 100 * (ra.a_hat_out(:,ax) - ra.a_true_out(:,ax)) ./ ra.a_true_out(:,ax);
        axh = subplot(2, 1, ax); hold(axh, 'on');
        hb = plot(axh, t, eb, 'Color', LIGHT_BLUE, 'LineWidth', 1.2);
        ha = plot(axh, t, ea, BLUE, 'LineWidth', 1.8);
        hz = plot(axh, t([1 end]), [0 0], RED, 'LineWidth', 2.0);
        ylabel(axh, ax_lbl{ax}, 'FontSize', FS, 'FontWeight', 'bold');
        if ax == 2
            xlabel(axh, 'time  [s]', 'FontSize', FS, 'FontWeight', 'bold');
        end
        legend([hz hb ha], {'zero', 'before (perp law)', 'after (parallel law)'}, ...
               'Location', 'northoutside', 'Orientation', 'horizontal', ...
               'FontSize', FS - 5);
        ylim(axh, [-42 12]); xlim(axh, [t(1) t(end)]);
        set(axh, 'FontSize', FS - 2, 'FontWeight', 'bold', 'Box', 'on');
        for x = tb
            plot(axh, [x x], ylim(axh), '--', 'Color', [0.6 0.6 0.6], ...
                 'LineWidth', 1.0, 'HandleVisibility', 'off');
        end
    end
    outp = fullfile(res_dir, sprintf('formB_c6_seed%d.png', target));
    drawnow; exportgraphics(fig, outp, 'Resolution', 150); close(fig);
    fprintf('FIGURE saved: %s\n', outp);
    for ax = 1:2
        eb = 100 * (rb.a_hat_out(:,ax) - rb.a_true_out(:,ax)) ./ rb.a_true_out(:,ax);
        ea = 100 * (ra.a_hat_out(:,ax) - ra.a_true_out(:,ax)) ./ ra.a_true_out(:,ax);
        wd = t>=0.5 & t<1.5; wo = t>=1.6 & t<3.5; wh = t>=3.8;
        fprintf('  seed %d ax %d: desc pk %.2f->%.2f  osc RMS %.2f->%.2f  hold %.2f->%.2f %%\n', ...
                target, ax, max(abs(eb(wd))), max(abs(ea(wd))), ...
                sqrt(mean(eb(wo).^2)), sqrt(mean(ea(wo).^2)), mean(eb(wh)), mean(ea(wh)));
    end
end
