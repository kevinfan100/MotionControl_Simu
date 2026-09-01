function plot_formC_b_true_curve(opts)
%PLOT_FORMC_B_TRUE_CURVE  The exact autonomous law's varying factor.
%
%   Eliminating w_bar between a_bar(w_bar) and a_bar'(w_bar) gives an EXACT
%   autonomous law, a_bar' = (1-a_bar)^2 / b_true(a_bar). This plots b_true
%   against a_bar: it is 1 at contact, 9/8 in the far field, and peaks in
%   between. The model of formC_state_b.tex replaces this curve by a constant,
%   so the plot IS the approximation being made.
%
%   Style copied from plot_var_ahat_6state (house canonical).

    if nargin < 1; opts = struct(); end
    if ~isfield(opts, 'out_dir')
        [sd, ~, ~] = fileparts(mfilename('fullpath'));
        opts.out_dir = fullfile(fileparts(fileparts(sd)), ...
                       'reference', 'eq17_analysis', 'derivation', 'figures');
    end
    if ~isfield(opts, 'env'); opts.env = [1.900, 23.222]; end
    FS = 18; LFS = 14; AXLW = 1.4;
    COL_TRUE = [0.85 0.10 0.10]; COL_BAND = [0.55 0.72 0.95];

    w = logspace(log10(1.0001), log10(300), 20001).';
    a = zeros(size(w)); ap = zeros(size(w));
    for i = 1:numel(w)
        [~, cp, d] = calc_correction_functions(w(i), true);
        a(i)  = 1 / cp;
        ap(i) = -d.dc_perp_dh / cp^2;
    end
    b = (1 - a).^2 ./ ap;
    in = w >= opts.env(1) & w <= opts.env(2);

    f = figure('Position', [80 80 1150 720], 'Color', 'w', ...
               'NumberTitle', 'off', 'Visible', 'off');
    hold on;
    hb = fill([a(in); flipud(a(in))], ...
              [min(b(in)) * ones(sum(in), 1); flipud(max(b(in)) * ones(sum(in), 1))], ...
              COL_BAND, 'FaceAlpha', 0.30, 'EdgeColor', 'none');
    ht = plot(a, b, '-', 'Color', COL_TRUE, 'LineWidth', 2.6);
    h1 = yline(1,     '--', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.4);
    h2 = yline(9/8,   '--', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.4);
    text(0.02, 1.004,  ' b = 1  (contact)',   'FontSize', LFS, 'FontWeight', 'bold');
    text(0.72, 1.1215, ' b = 9/8  (far field)', 'FontSize', LFS, 'FontWeight', 'bold');
    [bmx, imx] = max(b);
    plot(a(imx), bmx, 'o', 'Color', COL_TRUE, 'MarkerFaceColor', 'w', ...
         'LineWidth', 2.0, 'MarkerSize', 9);
    xlim([0 1]); ylim([0.98 1.18]);
    xlabel('a / a_{nom}   (a_{nom} = T_s/\gamma_N)', 'FontSize', FS, 'FontWeight', 'bold');
    ylabel('b_{true}  =  (1-a)^2 / a''', 'FontSize', FS, 'FontWeight', 'bold');
    legend([ht hb], {'b_{true}(a) -- the exact law''s varying factor', ...
           sprintf('working range,  w_{bar} \\in [%.2f, %.2f]', opts.env)}, ...
           'Location', 'northoutside', 'Orientation', 'horizontal', ...
           'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');
    grid off;

    out = fullfile(opts.out_dir, 'formC_b_true_curve.png');
    exportgraphics(f, out, 'Resolution', 150); close(f);
    fprintf('b_true: contact %.5f | far field %.5f | max %.5f at a = %.4f\n', ...
            b(1), b(end), bmx, a(imx));
    fprintf('working range: a in [%.4f, %.4f], b_true in [%.5f, %.5f]\n', ...
            min(a(in)), max(a(in)), min(b(in)), max(b(in)));
    fprintf('  -> b_init %.5f   sqrt(P55[0]) %.5f\n', ...
            0.5*(min(b(in))+max(b(in))), 0.5*(max(b(in))-min(b(in))));
    fprintf('wrote %s\n', out);
end
