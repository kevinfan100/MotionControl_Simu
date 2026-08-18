function plot_formC_b_payoff(R, opts)
%PLOT_FORMC_B_PAYOFF  Why estimating b does not pay, in two panels.
%
%   Left  : the filter's own uncertainty on b, normalised to its prior. If b
%           were being learned this falls. Overlaid: how far b_hat actually
%           travelled, in units of that same prior width.
%   Right : what b is WORTH. The band is the gain error that one full prior
%           width of b error would produce, |dA/db| * sqrt(P55[0]) / a_bar,
%           against the gain error the run actually makes. If the band is a
%           sliver inside the error, a perfect b buys back only that sliver.
%
%   P_b_out is ALREADY a standard deviation -- do not take its square root.
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
    t = A.runs{1}.tout(:);
    S = []; B = []; E = []; AT = [];
    for q = 1:n
        r = A.runs{q};
        S(:, q) = r.P_b_out(:, 3);                        %#ok<AGROW>
        B(:, q) = r.b_hat_out(:, 3);                      %#ok<AGROW>
        a_disp = r.a_hat_out(1, 3) / r.a_bar_hat_out(1, 3);
        at = r.a_true_out(:, 3) / a_disp;
        AT(:, q) = at;                                    %#ok<AGROW>
        E(:, q) = 100 * (r.a_bar_hat_out(:, 3) - at) ./ at;   %#ok<AGROW>
    end
    s0 = mean(S(1, :));

    f = figure('Position', [60 60 1500 640], 'Color', 'w', ...
               'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    % ---- panel 1: is b being learned? ----
    nexttile; hold on;
    hu = plot(t, mean(S, 2) / s0, '-', 'Color', COL_BAND, 'LineWidth', 2.8);
    hd = plot(t, abs(B - B(1, :)) / s0, '-', 'Color', COL_SEED, 'LineWidth', 1.0);
    yline(1, '--', 'Color', [0.35 0.35 0.35], 'LineWidth', 1.2);
    xlim([0 t(end)]); ylim([0 1.08]);
    ylabel('in units of the b prior', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    legend([hu hd(1)], {'\surdP_{55} / \surdP_{55}[0]   (falls if b is learned)', ...
           '|\^b - \^b[0]| / \surdP_{55}[0],  8 seeds'}, ...
           'Location', 'northoutside', 'Orientation', 'vertical', ...
           'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');
    grid off;

    % ---- panel 2: what is b worth? ----
    nexttile; hold on;
    payoff = 100 * abs((1 - AT(:, 1)) / mean(B(1, :))) * s0 ./ AT(:, 1);
    hb = fill([t; flipud(t)], [payoff; -flipud(payoff)], COL_TRUE, ...
              'FaceAlpha', 0.35, 'EdgeColor', 'none');
    he = plot(t, E, '-', 'Color', COL_SEED, 'LineWidth', 0.8);
    yline(0, '-', 'Color', [0.35 0.35 0.35], 'LineWidth', 1.0);
    xlim([0 t(end)]); ylim([-20 20]);
    ylabel('gain error   (%)', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    legend([hb he(1)], {'what ONE prior width of b is worth', ...
           'the error the run actually makes, 8 seeds'}, ...
           'Location', 'northoutside', 'Orientation', 'vertical', ...
           'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');
    grid off;

    out = fullfile(opts.out_dir, 'formC_b_payoff.png');
    exportgraphics(f, out, 'Resolution', 150); close(f);
    fprintf('sqrt(P55) %.6f -> %.6f  (shrink %.2f %%)\n', s0, mean(S(end,:)), ...
            100*(1-mean(S(end,:))/s0));
    fprintf('b travel  max %.3f sigma   |  payoff peak %.2f %%  vs run RMS %.2f %%\n', ...
            max(max(abs(B - B(1,:))))/s0, max(payoff), sqrt(mean(E(:).^2)));
    fprintf('wrote %s\n', out);
end
