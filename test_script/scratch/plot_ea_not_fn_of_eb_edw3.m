% PURPOSE (2026-09-10, user: "why can't e_a in row 3 be written with e_b and e_dw3 alone?"): the counter-example as a
%   two-panel walk. Same b, same height all the way down (e_dw3 = 0, e_b = 0); the only difference is the starting gain,
%   0.0001 low at w = 22.2. The law carries that start error down the canon descent and it grows to ~0.034 at the trough:
%   e_a has a part that is neither a height error nor a slope error (where the curve sits = w_s), so it is not a function
%   of (e_dw3, e_b). Pure law, no simulation. Output test_results/apd_acov_meng/ea_not_fn_of_eb_edw3.png
%   | EXPIRES: with the b line | 產線改動不會自動跟上
function plot_ea_not_fn_of_eb_edw3()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    b = 0.8844;  ws = 1 - 1/b;  w0 = 22.222;  da0 = 1e-4;                 % canon chord line (contact at w = 1)
    w = logspace(log10(1.10), log10(w0), 600);
    u = b*(w - ws);  a_true = 1 - 1./u;                                   % true gain along the path
    u0 = b*(w0 - ws);  a0 = 1 - 1/u0;  uh0 = 1/(1 - (a0 - da0));          % estimator starts 1e-4 low, same b, same height
    a_hat = 1 - 1./(u - (u0 - uh0));
    ea = a_true - a_hat;  explained = zeros(size(w));                     % -a' e_dw3 + ((1-a)/b) e_b with both errors 0
    fprintf('start w = %.1f : a_true %.5f  a_hat %.5f  e_a %.5f\n', w(end), a_true(end), a_hat(end), ea(end));
    fprintf('trough w = %.2f: a_true %.5f  a_hat %.5f  e_a %.5f  (%.0f%% of a_true)\n', w(1), a_true(1), a_hat(1), ea(1), 100*ea(1)/a_true(1));
    FS = 15; LFS = 11; AXLW = 1.8;
    f = figure('Units','inches','Position',[0 0 12 4.8],'Color','w','Visible','off');
    tl = tiledlayout(1, 2, 'TileSpacing','compact','Padding','compact');
    nexttile(tl); hold on; box on;
    h1 = plot(w, a_true, '-', 'Color', [0.85 0 0], 'LineWidth', 2.4);
    h2 = plot(w, a_hat, '--', 'Color', [0 0.35 0.85], 'LineWidth', 2.0);
    set(gca, 'XScale', 'log', 'XDir', 'reverse', 'XTick', [1.1 2 3 5 10 22.2], 'FontSize', FS, 'LineWidth', AXLW); xlim([1.1 w0]); ylim([0 1]);
    xlabel('$\bar w$', 'Interpreter','latex', 'FontSize', FS); ylabel('$\bar a_w$', 'Interpreter','latex', 'FontSize', FS);
    legend([h1 h2], {'true $\bar a_w$', 'estimate $\hat{\bar a}_w$ (start $-10^{-4}$, same $b$, same height)'}, ...
           'Interpreter','latex', 'Location','northoutside','Orientation','horizontal','FontSize', LFS);
    nexttile(tl); hold on; box on;
    h3 = plot(w, ea, '-', 'Color', [0 0.35 0.85], 'LineWidth', 2.4);
    h4 = plot(w, explained, '-', 'Color', [0.5 0.5 0.5], 'LineWidth', 2.0);
    set(gca, 'XScale', 'log', 'XDir', 'reverse', 'XTick', [1.1 2 3 5 10 22.2], 'FontSize', FS, 'LineWidth', AXLW); xlim([1.1 w0]);
    xlabel('$\bar w$', 'Interpreter','latex', 'FontSize', FS); ylabel('$e_{\bar a_w}$', 'Interpreter','latex', 'FontSize', FS);
    legend([h3 h4], {'$e_{\bar a_w}$', '$-\bar a''\,e_{\delta\bar w_3} + \frac{1-\hat{\bar a}_w}{\hat b}\,e_b$ \ ($e_{\delta\bar w_3} = e_b = 0$)'}, ...
           'Interpreter','latex', 'Location','northoutside','Orientation','horizontal','FontSize', LFS);
    exportgraphics(f, fullfile(od, 'ea_not_fn_of_eb_edw3.png'), 'Resolution', 150); close(f);
    fprintf('figure: %s\n', fullfile(od, 'ea_not_fn_of_eb_edw3.png'));
end
