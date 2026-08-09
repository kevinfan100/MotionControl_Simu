function res = plot_formB_truth_pair(opts)
%PLOT_FORMB_TRUTH_PAIR  The two truth functions the document estimates against.
%
%   STATUS: ACTIVE -- feeds formB_amp_functions p.6 (inserted after the five
%   form-comparison pages, before the closed-loop pages).
%
%   Pages 1-5 compare the three WRITINGS on one truth (c_perp) and deliberately
%   omit a_bar itself, because along b(w_bar) all three writings collapse onto
%   a_true and the page would carry no information about the writings.  This
%   page carries the complementary fact: a_true is not one curve but two, and
%   the later closed-loop pages run the same filter against each in turn.
%
%       a_bar = 1/c ,   c from calc_correction_functions (two-sphere series)
%
%   House style of plot_formB_form_compare: latex interpreter, no grid, box on,
%   legend northoutside horizontal, Resolution 200, 1000x680.
%
%   Figure -> derivation/figures/a_perp_para.png

    if nargin < 1; opts = struct(); end
    if ~isfield(opts, 'gap_max'); opts.gap_max = 10;   end
    if ~isfield(opts, 'save');    opts.save    = true; end

    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    addpath(fullfile(root, 'model', 'wall_effect'));
    fig_dir = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');

    G = linspace(0.1, opts.gap_max, 4000).';      % same grid as pages 1-5
    w = 1 + G;
    cA = zeros(size(w)); cP = zeros(size(w));
    for i = 1:numel(w); [cA(i), cP(i)] = calc_correction_functions(w(i), true); end
    aP = 1 ./ cP;   aA = 1 ./ cA;

    % ---- console ---------------------------------------------------------
    fprintf('\n[truth pair]  a_bar = 1/c\n');
    for g = [0.1 1 3.222 opts.gap_max]
        fprintf('  gap %6.3f   a_perp %.4f   a_para %.4f   ratio %.3f\n', ...
                g, interp1(G, aP, g), interp1(G, aA, g), ...
                interp1(G, aA, g) / interp1(G, aP, g));
    end
    fprintf('  far-field coefficient  (1-a)*w_bar :  perp %.4f -> 9/8 = %.4f', ...
            (1 - aP(end)) * w(end), 9/8);
    fprintf('   para %.4f -> 9/16 = %.4f\n', (1 - aA(end)) * w(end), 9/16);
    res = struct('G', G, 'a_perp', aP, 'a_para', aA);
    if ~opts.save; return; end

    % ---- house style -----------------------------------------------------
    C_PERP = [0.8 0 0];  C_PARA = [0 0.2 0.9];
    FS = 20; LFS = 16; AXLW = 2.0; LW = 2.2;

    f = figure('Position', [80 80 1000 680], 'Color', 'w', ...
               'NumberTitle', 'off', 'Visible', 'off');
    hold on;
    yline(1, '-', 'Color', [0.45 0.45 0.45], 'LineWidth', 1.0, 'HandleVisibility', 'off');
    xline(1, '--', 'Color', [0.62 0.62 0.62], 'LineWidth', 1.4, 'HandleVisibility', 'off');
    h(1) = plot(G, aP, '-', 'Color', C_PERP, 'LineWidth', LW, ...
                'DisplayName', '$\bar{a}_{\perp} = 1/c_{\perp}$');
    h(2) = plot(G, aA, '-', 'Color', C_PARA, 'LineWidth', LW, ...
                'DisplayName', '$\bar{a}_{\parallel} = 1/c_{\parallel}$');
    xlim([min(G) max(G)]); ylim([0 1.05]);
    xlabel('$\bar{w}-\bar{w}_s$', 'Interpreter', 'latex', 'FontSize', FS);
    ylabel('$\bar{a}$', 'Interpreter', 'latex', 'FontSize', FS);
    legend(h, 'Interpreter', 'latex', 'Location', 'northoutside', ...
           'Orientation', 'horizontal', 'FontSize', LFS, 'Box', 'on');
    set(gca, 'FontSize', FS, 'LineWidth', AXLW, 'Box', 'on', ...
             'TickLabelInterpreter', 'latex');
    grid off;
    out = fullfile(fig_dir, 'a_perp_para.png');
    exportgraphics(f, out, 'Resolution', 200, 'Padding', 'figure');
    close(f);
    fprintf('  wrote %s\n', out);
end
