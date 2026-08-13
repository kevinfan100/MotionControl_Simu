function plot_formC_anchor_bias_scatter(R, opts)
%PLOT_FORMC_ANCHOR_BIAS_SCATTER  Where the law's error lives: bias or scatter.
%
%   One tile per arm (2x2). Each tile overlays all 8 seeds' gain relative
%   error, plus the across-seed mean in heavy blue. The whole point is visual:
%
%     law fed the TRUE a_bar    -> the 8 traces LIE ON TOP OF EACH OTHER and
%                                  sit far from zero. Deterministic bias. The
%                                  9/8 anchor scales the whole bundle down.
%     law fed its OWN ESTIMATE  -> the 8 traces FAN OUT around zero. Scatter.
%                                  The anchor does nothing visible.
%
%   Style copied from plot_var_ahat_6state (house canonical), not rebuilt.

    if nargin < 2; opts = struct(); end
    if ~isfield(opts, 'out_dir')
        [sd, ~, ~] = fileparts(mfilename('fullpath'));
        opts.out_dir = fullfile(fileparts(fileparts(sd)), ...
                       'reference', 'eq17_analysis', 'derivation', 'figures');
    end
    FS = 18; LFS = 14; AXLW = 1.4;
    COL_HAT = [0 0.20 0.85]; COL_SEED = [0.55 0.72 0.95];

    arms  = {'exo_b1', 'exo_b98', 'state_b1', 'state_b98'};
    names = {'law @ TRUE a,  b = 1',   'law @ TRUE a,  b = 9/8', ...
             'law @ OWN est., b = 1',  'law @ OWN est., b = 9/8'};

    f = figure('Position', [60 60 1500 900], 'Color', 'w', ...
               'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    for j = 1:numel(arms)
        A = R.(arms{j}); n = numel(A.runs);
        E = []; tcol = [];
        for q = 1:n
            r = A.runs{q};
            a_disp = r.a_hat_out(1, 3) / r.a_bar_hat_out(1, 3);
            at = r.a_true_out(:, 3) / a_disp;
            E(:, q) = 100 * (r.a_bar_hat_out(:, 3) - at) ./ at;   %#ok<AGROW>
            if isempty(tcol); tcol = r.tout(:); end
        end
        nexttile; hold on;
        hs = plot(tcol, E, '-', 'Color', COL_SEED, 'LineWidth', 0.8);
        hm = plot(tcol, mean(E, 2), '-', 'Color', COL_HAT, 'LineWidth', 2.6);
        yline(0, '-', 'Color', [0.35 0.35 0.35], 'LineWidth', 1.0);
        xlim([0 tcol(end)]); ylim([-22 22]);
        ylabel('(\^a_z - a_{true})/a_{true}   (%)', 'FontSize', FS, 'FontWeight', 'bold');
        if j >= 3; xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold'); end
        legend([hm hs(1)], {sprintf('%s   mean', names{j}), '8 seeds'}, ...
               'Location', 'northoutside', 'Orientation', 'horizontal', ...
               'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');
        grid off;
        fprintf('%-22s  hold bias %+7.2f %%   across-seed sd %5.2f %%\n', ...
                names{j}, mean(E(end-200:end, :), 'all'), ...
                std(mean(E(end-200:end, :), 1)));
    end
    out = fullfile(opts.out_dir, 'formC_anchor_bias_vs_scatter.png');
    exportgraphics(f, out, 'Resolution', 150);
    close(f);
    fprintf('wrote %s\n', out);
end
