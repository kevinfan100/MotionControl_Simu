function res = run_formB_BC_estimator_worth(opts)
%RUN_FORMB_BC_ESTIMATOR_WORTH  What does estimating the constant actually buy?
%
%   STATUS: ACTIVE -- feeds
%   reference/eq17_analysis/derivation/formB_amp_bonly_probe.tex.
%
%   The counter-intuitive reading so far: Form B's constant barely moves, yet
%   its gain is the accurate one. Two different things are being conflated:
%       (1) how RIGHT the final number is        -- Form B wins
%       (2) how much the FILTER did to get there -- Form C wins
%   The gain is computed from the constant, so its error depends only on (1).
%   Whether the filter worked for that number does not enter.
%
%   This measures (2) directly, by removing the estimator: freeze the constant
%   at the anchor 9/8 so NO parameter is estimated at all, and compare with
%   the arm that estimates it. The paired difference is exactly what the
%   estimator contributes. If Form B's numbers are unchanged, the estimator is
%   provably doing nothing on this scenario -- which is the honest reading of
%   "b did not move".
%
%     B free    length writing, b free, prior 0.0157
%     B locked  length writing, b FROZEN at 9/8      (zero free parameters)
%     C free    amplitude writing, beta free, prior 0.0708
%     C locked  amplitude writing, beta FROZEN at 9/8
%
%   Output -> test_results/temp_formB_worth.mat (gitignored)
%   Figure -> derivation/figures/formB_BC_estimator_worth.png

    if nargin < 1; opts = struct(); end
    if ~isfield(opts, 'seeds')
        opts.seeds = [7 11 23 42 101 777 31 53 89 137 211 313];
    end
    if ~isfield(opts, 'save'); opts.save = true; end

    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));
    addpath(genpath(fullfile(root, 'test_script')));
    fig_dir = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');

    pc = physical_constants();
    ANCH = 9/8; AX_Z = 3;
    seeds = opts.seeds(:).'; n_s = numel(seeds);
    cfg = local_cfg(pc);
    [pB, fB] = local_priors(cfg, pc, 'len');
    [pC, fC] = local_priors(cfg, pc, 'amp');

    ARMS = { ...
      'B free',   struct('lock_b', false, 'lock_p', true, 'lock_ws', true, ...
                         'Pf_b_std', pB, 'Pf_a_floor', fB); ...
      'B locked', struct('lock_b', true,  'lock_p', true, 'lock_ws', true, ...
                         'Pf_b_std', pB, 'Pf_a_floor', fB); ...
      'C free',   struct('law_form_amp', true, 'lock_b', false, 'lock_p', true, ...
                         'lock_ws', true, 'p_init', 1, 'b_init', ANCH, ...
                         'Pf_b_std', pC, 'Pf_a_floor', fC); ...
      'C locked', struct('law_form_amp', true, 'lock_b', true,  'lock_p', true, ...
                         'lock_ws', true, 'p_init', 1, 'b_init', ANCH, ...
                         'Pf_b_std', pC, 'Pf_a_floor', fC) };
    n_a = size(ARMS, 1);

    fprintf('=== 估測那個常數，到底買到了什麼？（%d seeds）===\n', n_s);
    fprintf('plant = 真實平面牆 c_perp；兩個寫法各自的誠實 prior：B %.4f  C %.4f\n\n', pB, pC);
    fprintf('%-10s  %-16s %-16s %-16s %s\n', 'arm', 'desc peak %', 'osc RMS %', ...
            'hold mean %', 'b final');

    res = struct('arms', {ARMS(:, 1).'}, 'seeds', seeds);
    res.tab = nan(n_a, n_s, 4);
    for a = 1:n_a
        for q = 1:n_s
            s = run_formB_ws(cfg, struct('seed', seeds(q), ...
                                         'ctrl_const_override', ARMS{a, 2}));
            m = local_metrics(s, cfg, AX_Z);
            res.tab(a, q, :) = [m.desc, m.osc, m.hold, s.b_hat_out(end, AX_Z)];
        end
        T = squeeze(res.tab(a, :, :));
        fprintf('%-10s  %6.3f +- %-7.3f %6.3f +- %-7.3f %+6.3f +- %-7.3f %.4f\n', ...
                ARMS{a, 1}, mean(T(:,1)), std(T(:,1)), mean(T(:,2)), std(T(:,2)), ...
                mean(T(:,3)), std(T(:,3)), mean(T(:,4)));
    end

    % ---- what the estimator contributed, paired ------------------------
    nm = {'desc peak', 'osc RMS', 'hold |mean|'};
    fprintf('\n=== 估測器的貢獻 = 自由臂 - 鎖死臂（配對，負 = 估測有幫助）===\n');
    fprintf('%-12s %-24s %-24s\n', '', 'Form B', 'Form C');
    for k = 1:3
        if k < 3
            dB = squeeze(res.tab(1,:,k)) - squeeze(res.tab(2,:,k));
            dC = squeeze(res.tab(3,:,k)) - squeeze(res.tab(4,:,k));
        else
            dB = abs(squeeze(res.tab(1,:,3))) - abs(squeeze(res.tab(2,:,3)));
            dC = abs(squeeze(res.tab(3,:,3))) - abs(squeeze(res.tab(4,:,3)));
        end
        fprintf('%-12s %+7.3f  t=%+5.2f       %+7.3f  t=%+5.2f\n', nm{k}, ...
                mean(dB), mean(dB)/(std(dB)/sqrt(n_s)), ...
                mean(dC), mean(dC)/(std(dC)/sqrt(n_s)));
    end

    if opts.save
        save(fullfile(root, 'test_results', 'temp_formB_worth.mat'), 'res', '-v7.3');
    end
    local_fig(res, ARMS, fig_dir);
end

% --------------------------------------------------------------------------
function local_fig(res, ARMS, fig_dir)
    FS = 17; CB = [0 0.2 0.9]; CC = [0.45 0.55 0.95];
    M = {'descent peak |e_a|  [%]', 'oscillation RMS e_a  [%]', 'final hold |mean e_a|  [%]'};
    n_s = numel(res.seeds);
    f = figure('Position', [40 40 1250 430], 'Color', 'w', 'Visible', 'off');
    for k = 1:3
        ax = subplot(1, 3, k); hold(ax, 'on');
        V = cell(1, 4);
        for a = 1:4
            v = squeeze(res.tab(a, :, min(k, 3)));
            if k == 3; v = abs(v); end
            V{a} = v;
        end
        xs = [1 2 3.4 4.4];
        col = {CB, CB, CC, CC};
        for a = 1:4
            plot(ax, xs(a) * ones(1, n_s), V{a}, 'o', 'Color', col{a}, ...
                 'MarkerFaceColor', col{a} * 0 + 1, 'MarkerSize', 6);
            plot(ax, xs(a) + [-0.18 0.18], mean(V{a}) * [1 1], '-', ...
                 'Color', col{a}, 'LineWidth', 3.5);
        end
        for q = 1:n_s
            plot(ax, xs(1:2), [V{1}(q) V{2}(q)], '-', 'Color', [0.85 0.85 0.9]);
            plot(ax, xs(3:4), [V{3}(q) V{4}(q)], '-', 'Color', [0.85 0.85 0.9]);
        end
        xlim(ax, [0.5 5.0]);
        set(ax, 'XTick', xs, 'XTickLabel', {'free', 'locked', 'free', 'locked'});
        ylabel(ax, M{k}, 'FontSize', FS - 5, 'FontWeight', 'bold');
        text(ax, 0.22, 1.06, 'Form B', 'Units', 'normalized', 'Color', CB, ...
             'FontSize', FS - 4, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
        text(ax, 0.78, 1.06, 'Form C', 'Units', 'normalized', 'Color', CC, ...
             'FontSize', FS - 4, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
        set(ax, 'FontSize', FS - 5, 'FontWeight', 'bold', 'LineWidth', 1.3, 'Box', 'on');
        grid(ax, 'off');
    end
    out = fullfile(fig_dir, 'formB_BC_estimator_worth.png');
    exportgraphics(f, out, 'Resolution', 150); close(f);
    fprintf('wrote %s\n', out);
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
