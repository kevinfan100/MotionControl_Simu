function plot_formC_b_trajectory(R, opts)
%PLOT_FORMC_B_TRAJECTORY  Does b_hat actually move?
%
%   Left  : the whole run. 8 seeds of the free-b arm, the filter's own
%           +/- sqrt(P55) band about the seed, and b_true evaluated along the
%           TRUE height for reference.
%   Right : the first 0.1 s. If b_hat converged on information it would walk;
%           if it jumped it will show as a single step.
%
%   Style copied from plot_var_ahat_6state (house canonical).

    if nargin < 2; opts = struct(); end
    if ~isfield(opts, 'out_dir')
        [sd, ~, ~] = fileparts(mfilename('fullpath'));
        opts.out_dir = fullfile(fileparts(fileparts(sd)), ...
                       'reference', 'eq17_analysis', 'derivation', 'figures');
    end
    FS = 18; LFS = 13; AXLW = 1.4;
    COL_SEED = [0.55 0.72 0.95]; COL_TRUE = [0.85 0.10 0.10];
    COL_BAND = [0.20 0.35 0.85];

    A = R.best; n = numel(A.runs);
    B = []; C = []; t = [];
    for q = 1:n
        r = A.runs{q};
        B(:, q) = r.b_hat_out(:, 3);            %#ok<AGROW>
        C(:, q) = sqrt(r.P_b_out(:, 3));        %#ok<AGROW>
        if isempty(t); t = r.tout(:); end
    end
    % b_true along the realised height of seed 1
    hb = R.best.runs{1}.h_bar_true_out;
    if size(hb, 2) >= 3; hb = hb(:, 3); else; hb = hb(:, 1); end
    bt = nan(size(hb));
    for i = 1:numel(hb)
        if isfinite(hb(i)) && hb(i) > 1
            [~, cp, d] = calc_correction_functions(hb(i), true);
            at = 1 / cp;  apt = -d.dc_perp_dh / cp^2;
            bt(i) = (1 - at)^2 / apt;
        end
    end

    f = figure('Position', [60 60 1500 640], 'Color', 'w', ...
               'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for pane = 1:2
        nexttile; hold on;
        hbnd = fill([t; flipud(t)], [B(:,1) - C(:,1); flipud(B(:,1) + C(:,1))], ...
                    COL_BAND, 'FaceAlpha', 0.18, 'EdgeColor', 'none');
        hs = plot(t, B, '-', 'Color', COL_SEED, 'LineWidth', 1.0);
        ht = plot(t, bt, '-', 'Color', COL_TRUE, 'LineWidth', 2.4);
        yline(1, '--', 'Color', [0.35 0.35 0.35], 'LineWidth', 1.2);
        if pane == 1
            xlim([0 t(end)]);
        else
            xlim([0 0.10]);
        end
        ylim([0.94 1.22]);
        ylabel('b', 'FontSize', FS, 'FontWeight', 'bold');
        xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        if pane == 1
            legend([hs(1) ht hbnd], {'\^b, 8 seeds', 'b_{true} along the true height', ...
                   '\^b \pm \surdP_{55}'}, 'Location', 'northoutside', ...
                   'Orientation', 'horizontal', 'FontSize', LFS, ...
                   'FontWeight', 'bold', 'Box', 'on');
        else
            legend(hs(1), {'first 0.10 s'}, 'Location', 'northoutside', ...
                   'Orientation', 'horizontal', 'FontSize', LFS, ...
                   'FontWeight', 'bold', 'Box', 'on');
        end
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');
        grid off;
    end
    out = fullfile(opts.out_dir, 'formC_b_trajectory.png');
    exportgraphics(f, out, 'Resolution', 150); close(f);
    fprintf('b_hat  seed %.5f -> step2 %.5f -> end %.5f\n', B(1,1), B(2,1), B(end,1));
    fprintf('b_true along the run: [%.5f, %.5f]\n', min(bt), max(bt));
    fprintf('final gap  b_hat - b_true = %+.5f  (%.1f x sqrt(P55))\n', ...
            B(end,1) - bt(end), abs(B(end,1)-bt(end)) / C(end,1));
    fprintf('wrote %s\n', out);
end
