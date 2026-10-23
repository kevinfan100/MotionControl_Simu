function res = run_formB_amp_depth_sweep(opts)
%RUN_FORMB_AMP_DEPTH_SWEEP  Can beta_hat traverse 9/8 -> 1, and what does it
%   look like while it does?
%
%   STATUS: ACTIVE -- follow-up to run_formB_amp_bonly_arms. Feeds
%   reference/eq17_analysis/derivation/formB_amp_bonly_probe.tex.
%
%   The amplitude writing's effective constant runs b_eff(w) = w(c-1)/c from
%   1 at contact to 9/8 in the far field, so the FULL 9/8 -> 1 range is only
%   ever demanded by a trajectory that actually approaches contact. On the
%   canonical scenario (trough w_bar = 2.0) the truth never asks for less
%   than b_eff = 1.059, so beta_hat has no reason to go near 1 and the
%   envelope prior 0.0708 would not lawfully permit it anyway
%   ((9/8 - 1)^2 / P[0] = 3.1 > 1).
%
%   This sweep lowers the trough and lets the SAME derived chain size the
%   prior each time. Nothing is tuned: the envelope is a config fact and the
%   prior is the sup of the same truth read-off on it. The question is
%   whether the chain is self-consistent -- whether a trajectory that needs a
%   bigger traversal automatically declares a prior that permits it.
%
%   Output -> test_results/temp_formB_amp_depth.mat (gitignored)
%   Figures -> derivation/figures/formB_amp_depth_{traverse,reach}.png
%
%   ~4 s per run; 4 depths x 6 seeds ~= 100 s.

    if nargin < 1; opts = struct(); end
    if ~isfield(opts, 'seeds');  opts.seeds  = [7 11 23 42 101 777]; end
    if ~isfield(opts, 'depths'); opts.depths = [2.00, 1.60, 1.35, 1.20]; end
    if ~isfield(opts, 'save');   opts.save   = true; end

    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));
    addpath(genpath(fullfile(root, 'test_script')));
    fig_dir = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');

    pc = physical_constants();
    B_ANCHOR = 9/8; AX_Z = 3;
    seeds = opts.seeds(:).'; dep = opts.depths(:).';
    n_s = numel(seeds); n_d = numel(dep);

    fprintf('=== amplitude arm: how deep must the trajectory go for beta to traverse? ===\n');
    fprintf('plant = real plane wall; %d seeds x %d trough depths\n\n', n_s, n_d);
    fprintf(['trough  env_lo  prior    b_eff(trough)  demanded  lawful   ' ...
             'demanded/     beta_hat        gate\n']);
    fprintf(['h_bar           sqrtP0                 travel    budget   ' ...
             'prior         final           duty %%\n']);

    res = struct('depths', dep, 'seeds', seeds);
    res.runs = cell(n_d, n_s);
    res.tab  = nan(n_d, n_s, 6);   % b_end sqrtP0 sqrtPend budget gate hold
    res.info = nan(n_d, 4);        % env_lo prior beff_trough demanded

    for d = 1:n_d
        cfg = local_cfg(dep(d), pc);
        env_lo = cfg.h_bottom / pc.R - 0.1;
        env_hi = cfg.h_init   / pc.R + 1.0;
        [sPbb, floor_a] = local_amp_priors(env_lo, env_hi);
        beff_tr = local_beff(dep(d));
        demanded = abs(B_ANCHOR - beff_tr);
        res.info(d, :) = [env_lo, sPbb, beff_tr, demanded];

        ov = struct('law_form_amp', true, 'lock_b', false, 'lock_p', true, ...
                    'lock_ws', true, 'p_init', 1, 'b_init', B_ANCHOR, ...
                    'Pf_b_std', sPbb, 'Pf_a_floor', floor_a);
        for q = 1:n_s
            s = run_formB_ws(cfg, struct('seed', seeds(q), 'ctrl_const_override', ov));
            P0 = s.P_b_out(1, AX_Z); Pe = s.P_b_out(end, AX_Z);
            trav = s.b_hat_out(end, AX_Z) - s.b_hat_out(1, AX_Z);
            gate = 100 * mean(s.h_bar_true_out < cfg.h_bar_safe);
            t = s.tout(:);
            e = 100 * (s.a_hat_out(:, AX_Z) - s.a_true_out(:, AX_Z)) ./ s.a_true_out(:, AX_Z);
            t3 = cfg.t_hold + cfg.t_descend_override + cfg.n_cycles / cfg.frequency;
            res.tab(d, q, :) = [s.b_hat_out(end, AX_Z), P0, Pe, ...
                                trav^2 / max(P0^2 - Pe^2, eps), gate, ...
                                mean(e(t > t3 + 0.3))];
            res.runs{d, q} = struct('t', t, 'b', s.b_hat_out(:, AX_Z), ...
                                    'P', s.P_b_out(:, AX_Z), ...
                                    'w', s.h_bar_d_out(:), 'cfg', cfg);
        end
        T = squeeze(res.tab(d, :, :));
        lawful = sqrt(max(mean(T(:,2)).^2 - mean(T(:,3)).^2, 0));
        fprintf(['%5.2f   %5.2f   %.4f   %.4f        %.4f    %.4f   ' ...
                 '%6.2f       %.4f+-%.4f  %4.1f\n'], ...
                dep(d), env_lo, sPbb, beff_tr, demanded, lawful, ...
                demanded / sPbb, mean(T(:,1)), std(T(:,1)), mean(T(:,5)));
    end

    % ---- the arithmetic the sweep is testing --------------------------
    fprintf('\nglobal sup |b_eff - 9/8| (contact -> far field) = %.4f = the 12.5%% anchor tension\n', ...
            abs(B_ANCHOR - local_beff(1.001)));
    fprintf('to traverse the FULL 9/8 -> 1 lawfully the budget needs sqrt(P[0]) >= %.4f\n', 1/8);

    if opts.save
        save(fullfile(root, 'test_results', 'temp_formB_amp_depth.mat'), 'res', '-v7.3');
    end
    local_figs(res, fig_dir, B_ANCHOR);
end

% --------------------------------------------------------------------------
function local_figs(res, fig_dir, B_ANCHOR)
    FS = 17; BLUE = [0 0.2 0.9]; RED = [0.8 0 0]; BAND = [0.80 0.86 0.96];
    dep = res.depths; n_d = numel(dep); n_s = numel(res.seeds);

    f = figure('Position', [40 40 1250 880], 'Color', 'w', 'Visible', 'off');
    for d = 1:n_d
        ax = subplot(2, 2, d); hold(ax, 'on');
        r1 = res.runs{d, 1}; t = r1.t;
        tf = [t; flipud(t)];
        fill(ax, tf, [r1.b + r1.P; flipud(r1.b - r1.P)], BAND, ...
             'EdgeColor', 'none', 'FaceAlpha', 0.55, 'HandleVisibility', 'off');
        w = r1.w; be = nan(size(w));
        ok = w > 1.001;
        be(ok) = arrayfun(@local_beff, w(ok));
        ht = plot(ax, t, be, '--', 'Color', RED, 'LineWidth', 2.0);
        for q = 1:n_s
            hb = plot(ax, res.runs{d, q}.t, res.runs{d, q}.b, '-', ...
                      'Color', BLUE, 'LineWidth', 1.1);
        end
        ha = plot(ax, [t(1) t(end)], B_ANCHOR * [1 1], 'k--', 'LineWidth', 1.3);
        h1 = plot(ax, [t(1) t(end)], [1 1], '-', 'Color', [0.35 0.35 0.35], 'LineWidth', 1.3);
        ylim(ax, [0.93 1.22]); xlim(ax, [t(1) t(end)]);
        ylabel(ax, '\beta  [-]', 'FontSize', FS, 'FontWeight', 'bold');
        xlabel(ax, 'time  [s]', 'FontSize', FS, 'FontWeight', 'bold');
        text(ax, 0.03, 0.10, sprintf('trough h_{bar} = %.2f   prior %.4f', ...
             dep(d), res.info(d, 2)), 'Units', 'normalized', ...
             'FontSize', FS - 3, 'FontWeight', 'bold');
        if d == 1
            legend([ht hb ha h1], {'b_{eff} demanded', '\beta estimates', ...
                    'anchor 9/8', 'contact value 1'}, 'Location', 'northoutside', ...
                    'Orientation', 'horizontal', 'FontSize', FS - 6);
        end
        set(ax, 'FontSize', FS - 4, 'FontWeight', 'bold', 'LineWidth', 1.3, 'Box', 'on');
        grid(ax, 'off');
    end
    out = fullfile(fig_dir, 'formB_amp_depth_traverse.png');
    exportgraphics(f, out, 'Resolution', 150); close(f);
    fprintf('wrote %s\n', out);

    % reachability: demanded travel vs declared prior vs achieved
    f = figure('Position', [60 60 1000 560], 'Color', 'w', 'Visible', 'off');
    ax = axes(f); hold(ax, 'on');
    dem = res.info(:, 4); pri = res.info(:, 2);
    ach = abs(B_ANCHOR - mean(squeeze(res.tab(:, :, 1)), 2));
    sd  = std(squeeze(res.tab(:, :, 1)), 0, 2);
    h1 = plot(ax, dep, pri, '-o', 'Color', [0.35 0.35 0.35], 'LineWidth', 2.2, ...
              'MarkerFaceColor', [0.85 0.85 0.85], 'MarkerSize', 9);
    h2 = plot(ax, dep, dem, '--s', 'Color', RED, 'LineWidth', 2.2, ...
              'MarkerFaceColor', [0.97 0.8 0.8], 'MarkerSize', 9);
    h3 = errorbar(ax, dep, ach, sd, '-o', 'Color', BLUE, 'LineWidth', 2.2, ...
                  'MarkerFaceColor', BLUE, 'MarkerSize', 9, 'CapSize', 6);
    h4 = plot(ax, [min(dep) max(dep)], [1/8 1/8], ':', 'Color', [0.5 0 0.6], 'LineWidth', 2.0);
    set(ax, 'XDir', 'reverse');
    xlabel(ax, 'trough  h_{bar}  (deeper to the right)', 'FontSize', FS, 'FontWeight', 'bold');
    ylabel(ax, 'distance from 9/8  [-]', 'FontSize', FS, 'FontWeight', 'bold');
    legend([h1 h2 h3 h4], {'declared prior sqrt(P[0])', 'travel the truth demands', ...
            'travel achieved', 'full traverse 9/8 \rightarrow 1 = 1/8'}, ...
           'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS - 6, ...
           'NumColumns', 2);
    set(ax, 'FontSize', FS - 3, 'FontWeight', 'bold', 'LineWidth', 1.3, 'Box', 'on');
    grid(ax, 'off');
    out = fullfile(fig_dir, 'formB_amp_depth_reach.png');
    exportgraphics(f, out, 'Resolution', 150); close(f);
    fprintf('wrote %s\n', out);
end

function b = local_beff(w)
    [~, c] = calc_correction_functions(w, true);
    b = w * (c - 1) / c;
end

function [sPbb, floor_a] = local_amp_priors(h_lo, h_hi)
    h = linspace(h_lo, h_hi, 20001).';
    c = zeros(size(h));
    for i = 1:numel(h)
        [~, c(i)] = calc_correction_functions(h(i), true);
    end
    b_eff = h .* (c - 1) ./ c;
    sPbb  = max(abs(b_eff - 9/8));
    floor_a = max(abs((1 - (9/8) ./ h) - 1 ./ c));
end

function cfg = local_cfg(trough_hbar, pc)
    cfg = user_config();
    cfg.trajectory_type = 'osc';
    cfg.h_init    = 50;
    cfg.h_bottom  = trough_hbar * pc.R;
    cfg.amplitude = 2.5;
    cfg.frequency = 1;    cfg.n_cycles = 2;
    cfg.t_hold    = 0.5;  cfg.t_descend_override = 1.0;  cfg.T_sim = 4.8;
    cfg.h_min     = min(1.1 * pc.R, cfg.h_bottom - 1e-6);
    cfg.ctrl_enable = true; cfg.thermal_enable = true; cfg.meas_noise_enable = true;
    cfg.lambda_c = 0.7;   cfg.a_pd = 0.05;  cfg.a_cov = 0.05;
    cfg.meas_noise_std = [0.00062; 0.00057; 0.00331];
    cfg.h_bar_safe = 1.5;
end
