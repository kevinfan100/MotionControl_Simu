function res = run_formB_amp_gate_aligned(opts)
%RUN_FORMB_AMP_GATE_ALIGNED  Re-run the traversal question with the prior
%   domain and the observation domain made to agree.
%
%   STATUS: ACTIVE -- supersedes run_formB_amp_depth_sweep for any conclusion
%   about how far beta can travel. Feeds
%   reference/eq17_analysis/derivation/formB_amp_bonly_probe.tex.
%
%   THE DEFECT THIS FIXES. y2 is the only measurement carrying a parameter
%   column (H1 = [1 0 ... 0] has none), and it is gated off below
%   h_bar_safe = 1.5. The prior, however, was derived on
%   [trough - 0.1, h_init + 1]. For a trough below 1.6 those two domains stop
%   agreeing: the chain prices evidence the filter is forbidden to collect,
%   and the estimate's extra travel is prior, not learning.
%
%   FOUR ARMS. Same plant (real plane wall), same seeds, same everything else.
%
%     A  trough 1.60, gate 1.50, prior on [1.50, 23.22]
%        ALIGNED BY CONSTRUCTION -- env_lo lands exactly on the gate, so the
%        two domains coincide and nothing is gated. This is the deepest
%        trough that needs no fix at all.
%     B  trough 1.20, gate 1.50, prior on [1.10, 23.22]
%        THE DEFECT, kept as the reference: 39% of the run gated, prior 1.30x
%        the observable demand.
%     C  trough 1.20, gate 1.10, prior on [1.10, 23.22]
%        ALIGNED BY LOWERING THE GATE -- the filter is allowed to measure
%        everywhere its prior is priced. Asks: does the evidence exist?
%     D  trough 1.20, gate 1.50, prior on [1.50, 23.22]
%        ALIGNED BY CLIPPING THE PRIOR to what can be observed. Asks: how much
%        of B's travel was the over-wide prior alone?
%
%   B vs C isolates the gate; B vs D isolates the prior width; C vs D is the
%   two legitimate ways to align, which should not agree if the deep evidence
%   is real.
%
%   Output -> test_results/temp_formB_amp_gate.mat (gitignored)
%   Figure -> derivation/figures/formB_amp_gate_aligned.png

    if nargin < 1; opts = struct(); end
    if ~isfield(opts, 'seeds'); opts.seeds = [7 11 23 42 101 777 31 53]; end
    if ~isfield(opts, 'save');  opts.save  = true; end

    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));
    addpath(genpath(fullfile(root, 'test_script')));
    fig_dir = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');

    pc = physical_constants();
    B_ANCHOR = 9/8; AX_Z = 3;
    seeds = opts.seeds(:).'; n_s = numel(seeds);

    %          name        trough  gate   prior_lo
    ARMS = { 'A aligned',  1.60,   1.50,  1.50; ...
             'B defect',   1.20,   1.50,  1.10; ...
             'C gate low', 1.20,   1.10,  1.10; ...
             'D prior cut',1.20,   1.50,  1.50 };
    n_a = size(ARMS, 1);

    fprintf('=== amplitude arm: prior domain vs observation domain ===\n');
    fprintf('plant = real plane wall; %d seeds; the only difference between arms is\n', n_s);
    fprintf('the trough, the y2 gate height, and the domain the prior is derived on.\n\n');
    fprintf(['arm           trough  gate  prior_lo  sqrtP0   observable  demanded  ' ...
             'beta_hat          shrink  gate  hold\n']);
    fprintf(['                                                demand              ' ...
             '                  %%       duty%%  %%\n']);

    res = struct('arms', {ARMS(:, 1).'}, 'seeds', seeds, 'spec', cell2mat(ARMS(:, 2:4)));
    res.runs = cell(n_a, n_s);
    res.tab  = nan(n_a, n_s, 6);

    for a = 1:n_a
        trough = ARMS{a, 2}; gate = ARMS{a, 3}; plo = ARMS{a, 4};
        cfg = local_cfg(trough, gate, pc);
        env_hi = cfg.h_init / pc.R + 1.0;
        [sPbb, floor_a] = local_amp_priors(plo, env_hi);
        [sObs, ~]       = local_amp_priors(max(gate, plo), env_hi);   % what can be seen
        demanded = abs(B_ANCHOR - local_beff(trough));

        ov = struct('law_form_amp', true, 'lock_b', false, 'lock_p', true, ...
                    'lock_ws', true, 'p_init', 1, 'b_init', B_ANCHOR, ...
                    'Pf_b_std', sPbb, 'Pf_a_floor', floor_a);
        for q = 1:n_s
            s = run_formB_ws(cfg, struct('seed', seeds(q), 'ctrl_const_override', ov));
            P0 = s.P_b_out(1, AX_Z); Pe = s.P_b_out(end, AX_Z);
            t = s.tout(:);
            e = 100 * (s.a_hat_out(:, AX_Z) - s.a_true_out(:, AX_Z)) ./ s.a_true_out(:, AX_Z);
            t3 = cfg.t_hold + cfg.t_descend_override + cfg.n_cycles / cfg.frequency;
            res.tab(a, q, :) = [s.b_hat_out(end, AX_Z), P0, Pe, ...
                                100 * (1 - Pe / P0), ...
                                100 * mean(s.h_bar_true_out < gate), ...
                                mean(e(t > t3 + 0.3))];
            res.runs{a, q} = struct('t', t, 'b', s.b_hat_out(:, AX_Z), ...
                                    'P', s.P_b_out(:, AX_Z));
        end
        T = squeeze(res.tab(a, :, :));
        fprintf(['%-12s  %.2f    %.2f  %.2f      %.4f   %.4f      %.4f    ' ...
                 '%.4f+-%.4f  %5.1f   %4.1f  %+5.2f\n'], ...
                ARMS{a, 1}, trough, gate, plo, sPbb, sObs, demanded, ...
                mean(T(:,1)), std(T(:,1)), mean(T(:,4)), mean(T(:,5)), mean(T(:,6)));
        res.info(a, :) = [sPbb, sObs, demanded];
    end

    % ---- the three comparisons the design was built for ------------------
    bf = mean(squeeze(res.tab(:, :, 1)), 2);
    fprintf('\ntravel from the anchor (mean over %d seeds):\n', n_s);
    for a = 1:n_a
        fprintf('  %-12s  %+.5f\n', ARMS{a, 1}, bf(a) - B_ANCHOR);
    end
    fprintf('\n  B - D  = %+.5f   <- what the OVER-WIDE PRIOR alone bought\n', bf(2) - bf(4));
    fprintf('  C - B  = %+.5f   <- what OPENING THE GATE bought on top\n', bf(3) - bf(2));
    fprintf('  C - A  = %+.5f   <- deep + aligned, against the deepest arm that\n', bf(3) - bf(1));
    fprintf('                        needed no fix at all\n');

    if opts.save
        save(fullfile(root, 'test_results', 'temp_formB_amp_gate.mat'), 'res', '-v7.3');
    end
    local_fig(res, ARMS, fig_dir, B_ANCHOR, seeds);
end

% --------------------------------------------------------------------------
function local_fig(res, ARMS, fig_dir, B_ANCHOR, seeds)
    FS = 17; BLUE = [0 0.2 0.9]; BAND = [0.80 0.86 0.96];
    n_a = size(ARMS, 1); tb = [0.5 1.5 3.5];
    f = figure('Position', [40 40 1250 880], 'Color', 'w', 'Visible', 'off');
    for a = 1:n_a
        ax = subplot(2, 2, a); hold(ax, 'on');
        r1 = res.runs{a, 1}; t = r1.t; tf = [t; flipud(t)];
        fill(ax, tf, [r1.b + r1.P; flipud(r1.b - r1.P)], BAND, ...
             'EdgeColor', 'none', 'FaceAlpha', 0.55, 'HandleVisibility', 'off');
        for q = 1:numel(seeds)
            hb = plot(ax, res.runs{a, q}.t, res.runs{a, q}.b, '-', ...
                      'Color', BLUE, 'LineWidth', 1.1);
        end
        ha = plot(ax, [t(1) t(end)], B_ANCHOR * [1 1], 'k--', 'LineWidth', 1.3);
        h1 = plot(ax, [t(1) t(end)], [1 1], '-', 'Color', [0.35 0.35 0.35], 'LineWidth', 1.3);
        ho = plot(ax, [t(1) t(end)], (B_ANCHOR - res.info(a, 2)) * [1 1], ':', ...
                  'Color', [0.8 0 0], 'LineWidth', 2.0);
        ylim(ax, [0.93 1.20]); xlim(ax, [t(1) t(end)]);
        ylabel(ax, '\beta  [-]', 'FontSize', FS, 'FontWeight', 'bold');
        xlabel(ax, 'time  [s]', 'FontSize', FS, 'FontWeight', 'bold');
        text(ax, 0.03, 0.12, sprintf('%s   trough %.2f  gate %.2f', ...
             ARMS{a,1}, ARMS{a,2}, ARMS{a,3}), 'Units', 'normalized', ...
             'FontSize', FS - 5, 'FontWeight', 'bold');
        text(ax, 0.03, 0.04, sprintf('prior %.4f   observable %.4f', ...
             res.info(a,1), res.info(a,2)), 'Units', 'normalized', ...
             'FontSize', FS - 6);
        if a == 1
            legend([hb ha ho h1], {'\beta estimates', 'anchor 9/8', ...
                    'observable floor', 'contact value 1'}, ...
                   'Location', 'northoutside', 'Orientation', 'horizontal', ...
                   'FontSize', FS - 6);
        end
        set(ax, 'FontSize', FS - 4, 'FontWeight', 'bold', 'LineWidth', 1.3, 'Box', 'on');
        grid(ax, 'off');
        yl = ylim(ax);
        for x = tb
            plot(ax, [x x], yl, '--', 'Color', [0.55 0.55 0.55], ...
                 'LineWidth', 1.0, 'HandleVisibility', 'off');
        end
        ylim(ax, yl);
    end
    out = fullfile(fig_dir, 'formB_amp_gate_aligned.png');
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
    b_eff   = h .* (c - 1) ./ c;
    sPbb    = max(abs(b_eff - 9/8));
    floor_a = max(abs((1 - (9/8) ./ h) - 1 ./ c));
end

function cfg = local_cfg(trough_hbar, gate, pc)
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
    cfg.h_bar_safe = gate;
end
