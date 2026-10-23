function res = plot_formB_BC_single_seed(opts)
%PLOT_FORMB_BC_SINGLE_SEED  One seed, four arms, everything on one page.
%
%   STATUS: ACTIVE -- feeds
%   reference/eq17_analysis/derivation/formB_amp_bonly_probe.tex.
%
%   The ensemble figures answer "on average"; this one answers "what actually
%   happens in a single run". Four arms on ONE seed, same noise realisation:
%       B free / B locked / C free / C locked
%   locked = the constant frozen at the anchor 9/8, i.e. NO parameter is
%   estimated at all. The free-minus-locked difference within a row is
%   exactly what the estimator contributed on this run.
%
%   Row 1  the gain itself against the truth
%   Row 2  the gain error, with the +-2 % band
%   Row 3  the carried constant against b_eff(w(t)) -- the constant the truth
%          demands at the height the particle is at, i.e. the value that would
%          make the law exact right now
%
%   Figure -> derivation/figures/formB_BC_single_seed<seed>.png

    if nargin < 1; opts = struct(); end
    if ~isfield(opts, 'seed'); opts.seed = 7; end

    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));
    addpath(genpath(fullfile(root, 'test_script')));
    fig_dir = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');

    pc = physical_constants(); ANCH = 9/8; AX_Z = 3;
    cfg = local_cfg(pc);
    [pB, fB] = local_priors(cfg, pc, 'len');
    [pC, fC] = local_priors(cfg, pc, 'amp');

    ARMS = { ...
      'B free',   struct('lock_b', false, 'lock_p', true, 'lock_ws', true, ...
                         'Pf_b_std', pB, 'Pf_a_floor', fB), 'len'; ...
      'B locked', struct('lock_b', true,  'lock_p', true, 'lock_ws', true, ...
                         'Pf_b_std', pB, 'Pf_a_floor', fB), 'len'; ...
      'C free',   struct('law_form_amp', true, 'lock_b', false, 'lock_p', true, ...
                         'lock_ws', true, 'p_init', 1, 'b_init', ANCH, ...
                         'Pf_b_std', pC, 'Pf_a_floor', fC), 'amp'; ...
      'C locked', struct('law_form_amp', true, 'lock_b', true,  'lock_p', true, ...
                         'lock_ws', true, 'p_init', 1, 'b_init', ANCH, ...
                         'Pf_b_std', pC, 'Pf_a_floor', fC), 'amp' };
    n_a = size(ARMS, 1);

    R = cell(1, n_a);
    fprintf('=== 單 seed %d，四臂 ===\n', opts.seed);
    for a = 1:n_a
        s = run_formB_ws(cfg, struct('seed', opts.seed, 'ctrl_const_override', ARMS{a,2}));
        R{a} = s;
        m = local_metrics(s, cfg, AX_Z);
        fprintf('  %-10s  desc %6.3f %%   osc %6.3f %%   hold %+6.3f %%   b_end %.4f\n', ...
                ARMS{a,1}, m.desc, m.osc, m.hold, s.b_hat_out(end, AX_Z));
    end

    t = R{1}.tout(:); w = R{1}.h_bar_d_out(:);
    c = nan(size(w));
    for i = 1:numel(w)
        if w(i) > 1.01; [~, c(i)] = calc_correction_functions(w(i), true); end
    end
    beff = struct('len', (c - 1) .* (w - 1), 'amp', w .* (c - 1) ./ c);

    % ---- figure ---------------------------------------------------------
    FS = 17; RED = [0.8 0 0]; CB = [0 0.2 0.9]; CC = [0.45 0.55 0.95];
    GREY = [0.55 0.55 0.55]; tb = [0.5 1.5 3.5];
    col = {CB, CB, CC, CC};  sty = {'-', '--', '-', '--'};
    f = figure('Position', [30 30 1500 980], 'Color', 'w', 'Visible', 'off');
    for a = 1:n_a
        s = R{a}; ad = 1;
        % row 1 gain
        ax = subplot(3, 4, a); hold(ax, 'on');
        h1 = plot(ax, t, s.a_true_out(:, AX_Z), '-', 'Color', RED, 'LineWidth', 2.0);
        h2 = plot(ax, t, s.a_hat_out(:, AX_Z), sty{a}, 'Color', col{a}, 'LineWidth', 2.0);
        ylim(ax, [0 0.018]);
        if a == 1
            ylabel(ax, 'a_z  [\mum/pN]', 'FontSize', FS - 2, 'FontWeight', 'bold');
            legend([h1 h2], {'true', 'estimate'}, 'Location', 'northoutside', ...
                   'Orientation', 'horizontal', 'FontSize', FS - 6);
        end
        text(ax, 0.05, 0.90, ARMS{a,1}, 'Units', 'normalized', 'Color', col{a}, ...
             'FontSize', FS - 3, 'FontWeight', 'bold');
        local_ax(ax, FS, tb, t);
        % row 2 error
        e = 100 * (s.a_hat_out(:, AX_Z) - s.a_true_out(:, AX_Z)) ./ s.a_true_out(:, AX_Z);
        ax = subplot(3, 4, 4 + a); hold(ax, 'on');
        plot(ax, t, zeros(size(t)), '-', 'Color', RED, 'LineWidth', 1.4);
        plot(ax, t,  2 * ones(size(t)), 'k--', 'LineWidth', 0.9);
        plot(ax, t, -2 * ones(size(t)), 'k--', 'LineWidth', 0.9);
        plot(ax, t, e, sty{a}, 'Color', col{a}, 'LineWidth', 2.0);
        ylim(ax, [-12 8]);
        if a == 1
            ylabel(ax, 'a_z error  [%]', 'FontSize', FS - 2, 'FontWeight', 'bold');
        end
        local_ax(ax, FS, tb, t);
        % row 3 constant vs demanded
        ax = subplot(3, 4, 8 + a); hold(ax, 'on');
        be = beff.(ARMS{a,3});
        h3 = plot(ax, t, be, '--', 'Color', RED, 'LineWidth', 1.8);
        h4 = plot(ax, t, s.b_hat_out(:, AX_Z), sty{a}, 'Color', col{a}, 'LineWidth', 2.2);
        h5 = plot(ax, [t(1) t(end)], ANCH * [1 1], ':', 'Color', GREY, 'LineWidth', 1.6);
        ylim(ax, [1.02 1.17]);
        if a == 1
            ylabel(ax, 'constant  [-]', 'FontSize', FS - 2, 'FontWeight', 'bold');
            legend([h3 h4 h5], {'b_{eff} demanded', 'carried', 'anchor 9/8'}, ...
                   'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS - 7);
        end
        xlabel(ax, 'time  [s]', 'FontSize', FS - 3, 'FontWeight', 'bold');
        local_ax(ax, FS, tb, t);
    end
    out = fullfile(fig_dir, sprintf('formB_BC_single_seed%d.png', opts.seed));
    exportgraphics(f, out, 'Resolution', 150); close(f);
    fprintf('wrote %s\n', out);
    res = struct('runs', {R}, 'beff', beff, 't', t);
end

% --------------------------------------------------------------------------
function local_ax(ax, FS, tb, t)
    yl = ylim(ax);
    for x = tb
        plot(ax, [x x], yl, '--', 'Color', [0.6 0.6 0.6], 'LineWidth', 0.9, ...
             'HandleVisibility', 'off');
    end
    ylim(ax, yl); xlim(ax, [t(1) t(end)]);
    set(ax, 'FontSize', FS - 6, 'FontWeight', 'bold', 'LineWidth', 1.2, 'Box', 'on');
    grid(ax, 'off');
end

function m = local_metrics(s, cfg, ax)
    t = s.tout(:);
    e = 100 * (s.a_hat_out(:, ax) - s.a_true_out(:, ax)) ./ s.a_true_out(:, ax);
    t1 = cfg.t_hold; t2 = t1 + cfg.t_descend_override;
    t3 = t2 + cfg.n_cycles / cfg.frequency;
    m.desc = max(abs(e(t > t1 & t <= t2)));
    m.osc  = sqrt(mean(e(t > t2 + 0.2 & t <= t3).^2));
    m.hold = mean(e(t > t3 + 0.3));
end

function [sP, fl] = local_priors(cfg, pc, which)
    h = linspace(cfg.h_bottom / pc.R - 0.1, cfg.h_init / pc.R + 1.0, 20001).';
    c = zeros(size(h));
    for i = 1:numel(h); [~, c(i)] = calc_correction_functions(h(i), true); end
    if strcmp(which, 'len')
        b = (c - 1) .* (h - 1);  a = 1 - (1 + (h - 1) / (9/8)).^(-1);
    else
        b = h .* (c - 1) ./ c;   a = 1 - (9/8) ./ h;
    end
    sP = max(abs(b - 9/8));  fl = max(abs(a - 1 ./ c));
end

function cfg = local_cfg(pc)
    cfg = user_config();
    cfg.trajectory_type = 'osc';
    cfg.h_init = 50; cfg.h_bottom = 4.5; cfg.amplitude = 2.5;
    cfg.frequency = 1; cfg.n_cycles = 2;
    cfg.t_hold = 0.5; cfg.t_descend_override = 1.0; cfg.T_sim = 4.8;
    cfg.h_min = 1.1 * pc.R;
    cfg.ctrl_enable = true; cfg.thermal_enable = true; cfg.meas_noise_enable = true;
    cfg.lambda_c = 0.7; cfg.a_pd = 0.05; cfg.a_cov = 0.05;
    cfg.meas_noise_std = [0.00062; 0.00057; 0.00331];
    cfg.h_bar_safe = 1.5;
end
