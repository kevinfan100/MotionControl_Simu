% STATUS: ACTIVE (figure script for the Form C baseline four-arm ablation)
%          data producer test_script/integration/run_formC_base_ablation.m.
%          Regenerates the runs when called with no argument, so the committed
%          figures never depend on a gitignored .mat.
% EXPIRES: with run_formC_base_ablation.
function plot_formC_base_ablation(out)
%PLOT_FORMC_BASE_ABLATION  One page per seed, four arms overlaid.
%
%   plot_formC_base_ablation(out)   % out from run_formC_base_ablation
%   plot_formC_base_ablation()      % re-runs the ablation first
%
%   Row 1  normalized gain a_bar: true (red) vs the four arms' estimates.
%   Row 2  relative gain error 100*(a_hat - a_true)/a_true [%], four arms.
%   House style (plot_var_ahat_6state): 18 pt bold, 2.0 pt axes, no grid,
%   box on, legend northoutside horizontal, no title, statistics to console.
%
%   Writes reference/eq17_analysis/derivation/figures/formC_base_ablation_s<NN>.png

    if nargin < 1 || isempty(out)
        out = run_formC_base_ablation(struct('plot', false));
    end

    here = fileparts(mfilename('fullpath'));
    project_root = fileparts(fileparts(here));
    fig_dir = fullfile(project_root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    if ~exist(fig_dir, 'dir'); mkdir(fig_dir); end

    AX = out.axis;
    arms = out.arms;
    n_arms = numel(arms);

    % --- style ---------------------------------------------------------
    COL_TRUE = [0.80 0.00 0.00];                 % True = red (house rule)
    COL_ARM  = [0.00 0.20 0.90;                  % A0 nominal  = blue (estimate)
                0.93 0.49 0.06;                  % A1 y2 OFF   = orange
                0.10 0.60 0.30;                  % A2 fdet OFF = green
                0.45 0.10 0.65];                 % A3 both OFF = purple
    LS_ARM   = {'-', '-', '-', '--'};
    FS = 18; LFS = 13; AXLW = 2.0; LW = 2.0;

    for q = 1:numel(out.seeds)
        seed = out.seeds(q);
        t    = out.res{1}.runs{q}.tout(:);
        a_nom = out.res{1}.runs{q}.a_nom;
        ab_true = out.res{1}.runs{q}.a_true_out(:, AX) / a_nom;   % = 1/c_perp

        f = figure('Position', [80 80 1200 820], 'Color', 'w', ...
                   'NumberTitle', 'off', 'Visible', 'off');
        tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

        % ---- row 1: gain tracking ----
        nexttile; hold on;
        h = gobjects(n_arms + 1, 1);
        h(1) = plot(t, ab_true, '-', 'Color', COL_TRUE, 'LineWidth', 2.6, ...
                    'DisplayName', 'true');
        for a = 1:n_arms
            ab = out.res{a}.runs{q}.a_bar_hat_out(:, AX);
            h(a+1) = plot(t, ab, LS_ARM{a}, 'Color', COL_ARM(a, :), 'LineWidth', LW, ...
                          'DisplayName', sprintf('%s %s', arms(a).tag, arms(a).name));
        end
        local_phase_marks(out.windows);
        xlim([0 t(end)]);
        ylabel('$\bar{a}_z$ [-]', 'Interpreter', 'latex', 'FontSize', FS, 'FontWeight', 'bold');
        legend(h, 'Location', 'northoutside', 'Orientation', 'horizontal', ...
               'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on', 'NumColumns', 5);
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');
        grid off;

        % ---- row 2: relative gain error ----
        nexttile; hold on;
        for a = 1:n_arms
            s  = out.res{a}.runs{q};
            aT = s.a_true_out(:, AX);
            e  = 100 * (s.a_hat_out(:, AX) - aT) ./ max(aT, eps);
            plot(t, e, LS_ARM{a}, 'Color', COL_ARM(a, :), 'LineWidth', LW);
        end
        yline(0, '-', 'Color', COL_TRUE, 'LineWidth', 1.6, 'HandleVisibility', 'off');
        local_phase_marks(out.windows);
        xlim([0 t(end)]);
        xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        ylabel('rel. gain error [%]', 'FontSize', FS, 'FontWeight', 'bold');
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');
        grid off;

        fname = fullfile(fig_dir, sprintf('formC_base_ablation_s%02d.png', seed));
        exportgraphics(f, fname, 'Resolution', 150);
        close(f);
        fprintf('figure written: %s\n', fname);
    end
end


function local_phase_marks(w)
%LOCAL_PHASE_MARKS  Faint phase boundaries (descent start, osc start, hold
%   start); not legend entries, no labels, no grid.
    tb = [w.descent(1), w.descent(2), w.osc(2)];
    for i = 1:numel(tb)
        xline(tb(i), '-', 'Color', [0.72 0.72 0.72], 'LineWidth', 1.0, ...
              'HandleVisibility', 'off');
    end
end
