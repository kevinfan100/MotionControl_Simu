function plot_formC_scatter_predictability(R, opts)
%PLOT_FORMC_SCATTER_PREDICTABILITY  Can you know the scatter BEFORE the run?
%
%   Two tiles. Each overlays the 8 seeds' gain error on the filter's own
%   +/- sqrt(P44) band. If the band contains the seeds exactly, the filter's
%   claimed uncertainty IS the scatter and can be read off in advance.
%
%     law fed the commanded/true argument -> rho = 1.00, the band fits.
%     law fed its own estimate            -> rho = 1.8, the seeds burst out.
%
%   Style copied from plot_var_ahat_6state (house canonical).

    if nargin < 2; opts = struct(); end
    if ~isfield(opts, 'out_dir')
        [sd, ~, ~] = fileparts(mfilename('fullpath'));
        opts.out_dir = fullfile(fileparts(fileparts(sd)), ...
                       'reference', 'eq17_analysis', 'derivation', 'figures');
    end
    FS = 18; LFS = 14; AXLW = 1.4;
    COL_SEED = [0.55 0.72 0.95]; BANDC = [0.20 0.35 0.85];

    arms  = {'exo_b98', 'state_b1'};
    names = {'law @ commanded argument', 'law @ OWN estimate'};

    f = figure('Position', [60 60 1500 760], 'Color', 'w', ...
               'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    for j = 1:numel(arms)
        A = R.(arms{j}); E = []; C = []; tcol = [];
        for q = 1:numel(A.runs)
            r = A.runs{q};
            a_disp = r.a_hat_out(1, 3) / r.a_bar_hat_out(1, 3);
            at = r.a_true_out(:, 3) / a_disp;
            E(:, q) = 100 * (r.a_bar_hat_out(:, 3) - at) ./ at;   %#ok<AGROW>
            C(:, q) = 100 * sqrt(r.P_a_out(:, 3)) ./ at;          %#ok<AGROW>
            if isempty(tcol); tcol = r.tout(:); end
        end
        % Seeds are compared to the band about their OWN common mean, since
        % sqrt(P44) prices the SCATTER and never the model bias.
        mu = mean(E, 2); sd_claim = mean(C, 2);
        nexttile; hold on;
        hb = fill([tcol; flipud(tcol)], [mu - sd_claim; flipud(mu + sd_claim)], ...
                  BANDC, 'FaceAlpha', 0.22, 'EdgeColor', 'none');
        hs = plot(tcol, E, '-', 'Color', COL_SEED, 'LineWidth', 0.8);
        xlim([0 tcol(end)]); ylim([-25 25]);
        ylabel('(\^a_z - a_{true})/a_{true}   (%)', 'FontSize', FS, 'FontWeight', 'bold');
        xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        legend([hb hs(1)], {sprintf('%s:  mean \\pm claimed \\surdP_{44}', names{j}), ...
               '8 seeds'}, 'Location', 'northoutside', 'Orientation', 'horizontal', ...
               'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');
        grid off;
        N = size(E, 1); w = round(0.75 * N):N;
        fprintf('%-26s  realized sd %5.2f %%   claimed %5.2f %%   rho %4.2f\n', ...
                names{j}, mean(std(E(w, :), 0, 2)), mean(sd_claim(w)), ...
                mean(std(E(w, :), 0, 2)) / mean(sd_claim(w)));
    end
    out = fullfile(opts.out_dir, 'formC_scatter_predictability.png');
    exportgraphics(f, out, 'Resolution', 150);
    close(f);
    fprintf('wrote %s\n', out);
end
