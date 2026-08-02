function out = run_formB_ws_calibration(opts)
% STATUS: ACTIVE | SSOT for the w_bar_s calibration-chain protocol (Stage 3,
%   2026-08-02). Promoted from test_script/scratch/run_formB_ws_calib_chain.m.
%
%RUN_FORMB_WS_CALIBRATION  Sequential posterior handoff for w_bar_s.
%   w_bar_s is a SETUP CONSTANT: run n's posterior (ws_hat, sqrtP_ws) becomes
%   run n+1's prior. Per-run scatter is the honest posterior realization
%   (single-run filter is at the CRLB, formB_ws_ref S8 notes); the chain
%   averages it out at 1/sqrt(n) while the claimed sqrtP tracks the spread --
%   the root cure for the per-seed scatter (doc page c5).
%
%   out = run_formB_ws_calibration(opts), all fields optional:
%       .n_chain    6        independent chains (fresh noise per run)
%       .n_stage    8        calibration runs per chain
%       .delta      +0.05    injected TRUE wall offset [R] (plant side)
%       .operational true    one extra run per chain on the calibrated prior
%       .prior0     0.111    session-start prior width
%       .make_figs  false    export doc-page c5 figures (chain + operational)
%       .fig_names  {'formB_c5_chain.png','formB_c5_operational.png'}
%       .verbose    false
%   Seeds: run seed = 10000*chain + stage (operational: 10000*chain + 9999) --
%   disjoint across every chain/stage by construction.
%   Config: the C2 arm (lock_p, estimate b + w_s) of run_formB_ws.
%
%   Output: .ws_chain/.P_chain (n_chain x n_stage), .ws_op/.desc_op/.P_op
%   (n_chain x 1), .spread/.claimed/.bias (1 x n_stage), .delta/.opts.
%   Saved to test_results/run_formB_ws_calibration_<tag>.mat (gitignored).
%
%   See also: verify_formB_s11_calibration, run_formB_ws

    if nargin < 1; opts = struct(); end
    if ~isfield(opts, 'n_chain');     opts.n_chain = 6;        end
    if ~isfield(opts, 'n_stage');     opts.n_stage = 8;        end
    if ~isfield(opts, 'delta');       opts.delta = 0.05;       end
    if ~isfield(opts, 'operational'); opts.operational = true; end
    if ~isfield(opts, 'prior0');      opts.prior0 = 0.111;     end
    if ~isfield(opts, 'make_figs');   opts.make_figs = false;  end
    if ~isfield(opts, 'fig_names')
        opts.fig_names = {'formB_c5_chain.png', 'formB_c5_operational.png'};
    end
    if ~isfield(opts, 'verbose');     opts.verbose = false;    end

    here = fileparts(mfilename('fullpath'));
    res_dir = fullfile(here, '..', '..', 'test_results');
    AX_Z = 3;
    base_ov = struct('lock_b', false, 'lock_p', true, 'lock_ws', false);
    nc = opts.n_chain; ns = opts.n_stage;

    ws_chain = zeros(nc, ns); P_chain = zeros(nc, ns);
    ws_op = nan(nc, 1); desc_op = nan(nc, 1); P_op = nan(nc, 1);
    t_op = []; ws_op_traj = cell(nc, 1);
    for c = 1:nc
        ws0 = 1; Pws0 = opts.prior0;
        for k = 1:ns
            o = struct(); o.seeds = 10000*c + k; o.ws_inject = opts.delta;
            o.verbose = false;
            o.ctrl_const_override = base_ov;
            o.ctrl_const_override.Pf_ws_std = Pws0;
            o.ctrl_const_override.ws_init   = ws0;
            r = local_quiet_run(o, opts.verbose);
            ws0  = r.runs{1}.ws_hat_out(end, AX_Z);
            Pws0 = r.runs{1}.P_ws_out(end, AX_Z);
            ws_chain(c, k) = ws0;  P_chain(c, k) = Pws0;
        end
        if opts.operational
            o = struct(); o.seeds = 10000*c + 9999; o.ws_inject = opts.delta;
            o.verbose = false;
            o.ctrl_const_override = base_ov;
            o.ctrl_const_override.Pf_ws_std = P_chain(c, ns);
            o.ctrl_const_override.ws_init   = ws_chain(c, ns);
            r = local_quiet_run(o, opts.verbose);
            ws_op(c)   = r.runs{1}.ws_hat_out(end, AX_Z);
            P_op(c)    = r.runs{1}.P_ws_out(end, AX_Z);
            desc_op(c) = r.metrics.rows(1, 1);
            ws_op_traj{c} = r.runs{1}.ws_hat_out(:, AX_Z);
            t_op = r.runs{1}.tout(:);
        end
        fprintf('chain %2d/%2d: ws by run ', c, nc);
        fprintf('%.4f ', ws_chain(c, :)); fprintf('\n');
    end

    out = struct();
    out.opts = opts; out.delta = opts.delta;
    out.ws_chain = ws_chain; out.P_chain = P_chain;
    out.ws_op = ws_op; out.P_op = P_op; out.desc_op = desc_op;
    out.spread  = std(ws_chain, 0, 1);
    out.claimed = mean(P_chain, 1);
    out.bias    = mean(ws_chain, 1) - (1 + opts.delta);
    tag = sprintf('n%d_s%d_d%+04.0f', nc, ns, 1000*opts.delta);
    save(fullfile(res_dir, ['run_formB_ws_calibration_' tag '.mat']), 'out', '-v7.3');
    fprintf('spread by run:  '); fprintf('%.4f ', out.spread);  fprintf('\n');
    fprintf('claimed sqrtP:  '); fprintf('%.4f ', out.claimed); fprintf('\n');
    fprintf('bias vs truth:  '); fprintf('%+.4f ', out.bias);   fprintf('\n');
    if opts.operational
        fprintf('operational: ws err std %.4f | desc pk mean %.3f %%\n', ...
                std(ws_op - (1 + opts.delta)), mean(desc_op));
    end

    if opts.make_figs
        local_make_figs(out, t_op, ws_op_traj, res_dir, opts.fig_names);
    end
end


function r = local_quiet_run(o, verbose)
    if verbose
        r = run_formB_ws(o);
    else
        [~, r] = evalc('run_formB_ws(o)');
    end
end


function local_make_figs(out, t_op, ws_op_traj, res_dir, fig_names)
% Doc-page c5 figures (figure-style.md: no grid/title, box on, legend
% northoutside horizontal, True = red, FS18 bold, 150 dpi).
    FS = 18; nc = size(out.ws_chain, 1); ns = size(out.ws_chain, 2);
    ws_true = 1 + out.delta;
    blues = [linspace(0.0, 0.5, nc).', linspace(0.2, 0.7, nc).', ...
             linspace(0.55, 1.0, nc).'];

    figA = figure('Color', 'w', 'Position', [80 60 900 620]); drawnow;
    axA = axes(figA); hold(axA, 'on');
    hs = gobjects(1, nc);
    for c = 1:nc
        hs(c) = plot(axA, 1:ns, out.ws_chain(c, :), '-o', ...
                     'Color', blues(c, :), 'LineWidth', 1.3, ...
                     'MarkerSize', 4, 'MarkerFaceColor', blues(c, :));
    end
    ht = plot(axA, [1 ns], ws_true * [1 1], 'r', 'LineWidth', 2.6);
    h0 = plot(axA, [1 ns], [1 1], 'k:', 'LineWidth', 1.6);
    hb = plot(axA, 1:ns, ws_true + out.claimed, 'k--', 'LineWidth', 1.2);
    plot(axA, 1:ns, ws_true - out.claimed, 'k--', 'LineWidth', 1.2, ...
         'HandleVisibility', 'off');
    ylabel(axA, 'ws_{hat} after run  [-]', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel(axA, 'calibration run #', 'FontSize', FS, 'FontWeight', 'bold');
    legend([ht h0 hb hs(1)], {sprintf('true wall %.2f', ws_true), ...
           'nominal (1)', 'claimed \pm\surdP', ...
           sprintf('chains (%d, fresh noise each run)', nc)}, ...
           'Location', 'northoutside', 'Orientation', 'horizontal', ...
           'FontSize', FS - 6);
    xlim(axA, [1 ns]); xticks(axA, 1:ns);
    set(axA, 'FontSize', FS - 2, 'FontWeight', 'bold', 'Box', 'on');
    pngA = fullfile(res_dir, fig_names{1});
    drawnow; exportgraphics(figA, pngA, 'Resolution', 150); close(figA);
    fprintf('FIGURE saved: %s\n', pngA);

    figB = figure('Color', 'w', 'Position', [80 60 900 620]); drawnow;
    axB = axes(figB); hold(axB, 'on');
    for c = 1:nc
        hs(c) = plot(axB, t_op, 100 * (ws_op_traj{c} - 1), ...
                     'Color', blues(c, :), 'LineWidth', 1.2);
    end
    ht = plot(axB, t_op([1 end]), 100 * out.delta * [1 1], 'r', 'LineWidth', 2.6);
    h0 = plot(axB, t_op([1 end]), [0 0], 'k:', 'LineWidth', 1.6);
    ylabel(axB, '(ws_{hat}-1)  [% of R]', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel(axB, 'time  [s]', 'FontSize', FS, 'FontWeight', 'bold');
    legend([ht h0 hs(1)], {sprintf('true wall (%+.0f)', 100 * out.delta), ...
           'nominal (0)', ...
           sprintf('operational runs, calibrated prior (%d chains)', nc)}, ...
           'Location', 'northoutside', 'Orientation', 'horizontal', ...
           'FontSize', FS - 6);
    ylim(axB, [-26 8]); xlim(axB, [t_op(1) t_op(end)]);
    set(axB, 'FontSize', FS - 2, 'FontWeight', 'bold', 'Box', 'on');
    for x = [0.5 1.5 3.5]
        plot(axB, [x x], ylim(axB), '--', 'Color', [0.6 0.6 0.6], ...
             'LineWidth', 1.0, 'HandleVisibility', 'off');
    end
    pngB = fullfile(res_dir, fig_names{2});
    drawnow; exportgraphics(figB, pngB, 'Resolution', 150); close(figB);
    fprintf('FIGURE saved: %s\n', pngB);
end
