function plot_formC_belief_vs_command(R, opts)
%PLOT_FORMC_BELIEF_VS_COMMAND  One figure for where a_bar' may be evaluated.
%
%   Both arms are given the SAME deterministic seed offset (+0.0056 in a_bar,
%   identical across all seeds, carrying no random information). They differ
%   only in where the gain slope is read:
%
%     from the true height  -- the model cannot follow the belief, so it
%                              contradicts it: the offset is erased.
%     from the belief       -- the model follows, so the offset is never
%                              contradicted, and the motion turns a common-mode
%                              error into seed-dependent scatter.
%
%   Mean over seeds as the heavy line, +-1 across-seed sd as the band, so both
%   the bias and the scatter are in the same panel.
%
%   Style copied from plot_var_ahat_6state (house canonical).

    if nargin < 2; opts = struct(); end
    if ~isfield(opts, 'out_dir')
        [sd_, ~, ~] = fileparts(mfilename('fullpath'));
        opts.out_dir = fullfile(fileparts(fileparts(sd_)), ...
                       'reference', 'eq17_analysis', 'derivation', 'figures');
    end
    FS = 18; LFS = 14; AXLW = 1.4;
    C = struct('state', [0.85 0.10 0.10], 'exo', [0.20 0.35 0.85]);
    L = struct('state', 'a''  from  \^a_w', 'exo', 'a''  from  w_{bar}');

    t = R.state.runs{1}.tout(:); n = numel(t);
    % e_theta = theta - theta_hat throughout the derivation, so the plotted
    % quantity is e_a_bar_w, not its negative.
    cols = @(S) reshape(cell2mat(cellfun(@(r) ...
        r.a_true_out(:,3)/(r.a_hat_out(1,3)/r.a_bar_hat_out(1,3)) - ...
        r.a_bar_hat_out(:,3), S(:).', 'uni', 0)), n, []);

    f = figure('Position', [60 60 1250 660], 'Color', 'w', ...
               'NumberTitle', 'off', 'Visible', 'off');
    hold on;
    hb = gobjects(2,1); hm = gobjects(2,1); k = 0;
    for nm = {'exo', 'state'}
        k = k + 1; E = cols(R.(nm{1}).runs);
        mu = mean(E, 2); sg = std(E, 0, 2);
        hb(k) = fill([t; flipud(t)], [mu - sg; flipud(mu + sg)], C.(nm{1}), ...
                     'FaceAlpha', 0.22, 'EdgeColor', 'none');
        hm(k) = plot(t, mu, '-', 'Color', C.(nm{1}), 'LineWidth', 3.0);
    end
    yline(0, '-', 'Color', [0.55 0.55 0.55], 'LineWidth', 1.0);
    xlim([0 t(end)]); ylim([-0.015 0.026]);
    xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    ylabel('e_{a_w}', 'FontSize', FS, 'FontWeight', 'bold');
    legend([hm(2) hm(1)], {L.state, L.exo}, ...
           'Location', 'northoutside', 'Orientation', 'horizontal', ...
           'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');
    grid off;
    out = fullfile(opts.out_dir, 'formC_belief_vs_command.png');
    exportgraphics(f, out, 'Resolution', 150); close(f);
    fprintf('wrote %s\n', out);
end
