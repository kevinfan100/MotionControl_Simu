% plot_formB_ws_injection.m -- S11 wall-offset injection: does the estimator
%   converge to a TRUE wall at 1 + delta (plant-side shift, nominal frame kept)?
%   PURPOSE: first closed-loop injection figures (chat 2026-08-01); one figure
%   per delta: (a) ws_hat(t) all seeds vs true wall, (b) a_z relative error.
%   EXPIRES: superseded when the S11 protocol goes to integration/.
%   Data: test_results/run_formB_ws_inject_p05.mat (out_ip) / _m05.mat (out_im),
%   produced by run_formB_ws with opts.ws_inject = +/-0.05 (C2 config).
%   Style: .claude/rules/figure-style.md (no grid/title, box on, legend
%   northoutside horizontal, True = red / Estimate = blue, FS18 bold, 150 dpi).

here = fileparts(mfilename('fullpath'));
proj = fullfile(here, '..', '..');
res_dir = fullfile(proj, 'test_results');

AX_Z = 3; FS = 18;
RED  = 'r'; BLUE = 'b';
SEED_BLUES = [0.00 0.20 0.75; 0.10 0.45 0.90; 0.35 0.60 0.95; ...
              0.00 0.30 0.55; 0.25 0.35 0.80; 0.50 0.70 1.00];

cases = { ...
    struct('file', 'run_formB_ws_inject_p05.mat', 'var', 'out_ip', ...
           'delta', +0.05, 'png', 'formB_ws_inject_p05.png'), ...
    struct('file', 'run_formB_ws_inject_m05.mat', 'var', 'out_im', ...
           'delta', -0.05, 'png', 'formB_ws_inject_m05.png') };

for c = 1:numel(cases)
    C = cases{c};
    S = load(fullfile(res_dir, C.file));  out = S.(C.var);
    seeds = out.seeds;  n_s = numel(seeds);
    t  = out.runs{1}.tout(:);
    tb = [out.cfg.t_hold, out.cfg.t_hold + out.cfg.t_descend_override, ...
          out.cfg.t_hold + out.cfg.t_descend_override + out.cfg.n_cycles/out.cfg.frequency];
    ws_true = 1 + C.delta;

    fig = figure('Color', 'w', 'Position', [80 60 900 760]); drawnow;

    % (a) ws_hat(t), all seeds, vs injected true wall
    ax1 = subplot(2, 1, 1); hold(ax1, 'on');
    hs = gobjects(1, n_s);
    for q = 1:n_s
        hs(q) = plot(ax1, t, out.runs{q}.ws_hat_out(:, AX_Z), ...
                     'Color', SEED_BLUES(q, :), 'LineWidth', 1.4);
    end
    ht = plot(ax1, t([1 end]), ws_true * [1 1], RED, 'LineWidth', 2.6);
    h0 = plot(ax1, t([1 end]), [1 1], 'k:', 'LineWidth', 1.6);
    ylabel(ax1, 'ws_{hat}  [-]', 'FontSize', FS, 'FontWeight', 'bold');
    legend(ax1, [ht h0 hs(1)], ...
           {sprintf('true wall  1%+.2f', C.delta), 'seed / nominal (1)', ...
            sprintf('ws_{hat}  (%d seeds)', n_s)}, ...
           'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS - 4);
    ylim(ax1, [0.75 1.15]); xlim(ax1, [t(1) t(end)]);
    set(ax1, 'FontSize', FS - 2, 'FontWeight', 'bold', 'Box', 'on');
    for x = tb; plot(ax1, [x x], ylim(ax1), '--', 'Color', [0.6 0.6 0.6], ...
                     'LineWidth', 1.0, 'HandleVisibility', 'off'); end

    % (b) a_z relative error, all seeds
    ax2 = subplot(2, 1, 2); hold(ax2, 'on');
    for q = 1:n_s
        aT = out.runs{q}.a_true_out(:, AX_Z);
        e  = 100 * (out.runs{q}.a_hat_out(:, AX_Z) - aT) ./ aT;
        plot(ax2, t, e, 'Color', SEED_BLUES(q, :), 'LineWidth', 1.2);
    end
    hz = plot(ax2, t([1 end]), [0 0], RED, 'LineWidth', 2.0);
    ylabel(ax2, 'a_z error  [%]', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel(ax2, 'time  [s]', 'FontSize', FS, 'FontWeight', 'bold');
    legend(ax2, hz, {'zero'}, 'Location', 'northoutside', ...
           'Orientation', 'horizontal', 'FontSize', FS - 4);
    ylim(ax2, [-14 14]); xlim(ax2, [t(1) t(end)]);
    set(ax2, 'FontSize', FS - 2, 'FontWeight', 'bold', 'Box', 'on');
    for x = tb; plot(ax2, [x x], ylim(ax2), '--', 'Color', [0.6 0.6 0.6], ...
                     'LineWidth', 1.0, 'HandleVisibility', 'off'); end

    out_png = fullfile(res_dir, C.png);
    exportgraphics(fig, out_png, 'Resolution', 150);
    fprintf('FIGURE saved: %s\n', out_png);

    % console stats (figures carry no title/statistics per house style)
    wf = zeros(1, n_s);
    for q = 1:n_s; wf(q) = out.runs{q}.ws_hat_out(end, AX_Z); end
    fprintf('  delta %+0.2f: ws finals ', C.delta); fprintf('%.4f ', wf);
    fprintf('| mean %.4f (target %.2f) | response %.0f%% | std %.4f\n', ...
            mean(wf), ws_true, 100 * (mean(wf) - 1) / C.delta, std(wf));
end
