% PURPOSE (2026-09-11, user: "why do the same a model written two ways give such different results?"): the same law
%   a(w, b) = 1 - 1/(b (w - w_s)) under a change of b, drawn two ways. Left = the original derivation: the a state holds
%   the gain at the current height, so a new b turns the curve about the CURRENT point. Right = the a-model substitution:
%   w_s is held, so a new b turns the curve about w_s and the gain at the current height jumps at once.
%   b 0.884 -> 0.984 (+0.1, exaggerated so the two curves separate on the page; the reply quotes +0.01). Canon chord line,
%   current point w = 1.10. Pure law, no simulation. Output test_results/apd_acov_meng/pivot_original_vs_amodel.png
%   | EXPIRES: with the b line | 產線改動不會自動跟上
function plot_pivot_original_vs_amodel()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    b0 = 0.8844;  b1 = b0 + 0.1;  ws = 1 - 1/b0;  wk = 1.10;
    w = linspace(1.0, 3.4, 400);
    a_of = @(u) 1 - 1./u;
    u0 = b0*(wk - ws);  ak = a_of(u0);                                  % current point on the chord line
    orig0 = a_of(u0 + b0*(w - wk));  orig1 = a_of(u0 + b1*(w - wk));     % original: pivot at the current point
    amod0 = a_of(b0*(w - ws));       amod1 = a_of(b1*(w - ws));          % a model: pivot at w_s
    for db = [0.01 0.1]
        fprintf('b %+.2f at w = %.2f: original gain change %+.5f | a model gain change %+.5f\n', db, wk, ...
                a_of(u0) - ak, a_of((b0+db)*(wk - ws)) - ak);
    end
    FS = 15; LFS = 11; AXLW = 1.8;  C0 = [0 0.35 0.85];  C1 = [0.9 0.5 0];
    f = figure('Units','inches','Position',[0 0 12 4.8],'Color','w','Visible','off');
    tl = tiledlayout(1, 2, 'TileSpacing','compact','Padding','compact');
    nexttile(tl); hold on; box on;
    h1 = plot(w, orig0, '-', 'Color', C0, 'LineWidth', 2.2);  h2 = plot(w, orig1, '--', 'Color', C1, 'LineWidth', 2.2);
    h3 = plot(wk, ak, 'ko', 'MarkerSize', 9, 'MarkerFaceColor', 'k');
    set(gca, 'FontSize', FS, 'LineWidth', AXLW); xlim([1.0 3.4]); ylim([0 0.75]);
    xlabel('$\bar w$', 'Interpreter','latex', 'FontSize', FS); ylabel('$\bar a$', 'Interpreter','latex', 'FontSize', FS);
    legend([h1 h2 h3], {'$b = 0.884$', '$b = 0.984$', 'current point'}, 'Interpreter','latex', ...
           'Location','northoutside','Orientation','horizontal','FontSize', LFS);
    nexttile(tl); hold on; box on;
    h4 = plot(w, amod0, '-', 'Color', C0, 'LineWidth', 2.2);  h5 = plot(w, amod1, '--', 'Color', C1, 'LineWidth', 2.2);
    h6 = plot(wk, ak, 'ko', 'MarkerSize', 9, 'MarkerFaceColor', 'k');
    plot(wk, a_of(b1*(wk - ws)), 'o', 'Color', C1, 'MarkerSize', 9, 'LineWidth', 2);
    set(gca, 'FontSize', FS, 'LineWidth', AXLW); xlim([1.0 3.4]); ylim([0 0.75]);
    xlabel('$\bar w$', 'Interpreter','latex', 'FontSize', FS);
    legend([h4 h5 h6], {'$b = 0.884$', '$b = 0.984$', 'current point'}, 'Interpreter','latex', ...
           'Location','northoutside','Orientation','horizontal','FontSize', LFS);
    exportgraphics(f, fullfile(od, 'pivot_original_vs_amodel.png'), 'Resolution', 150); close(f);
    fprintf('figure: %s\n', fullfile(od, 'pivot_original_vs_amodel.png'));
end
