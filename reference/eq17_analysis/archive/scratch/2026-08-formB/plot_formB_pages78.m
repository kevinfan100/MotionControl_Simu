function plot_formB_pages78(opts)
%PLOT_FORMB_PAGES78  Gain error on each truth function, both writings.
%
%   STATUS: ACTIVE -- reads test_results/temp_formB_two_boundaries.mat.
%   Pages 7 and 8 appended to formB_amp_functions.
%
%   One quantity per page (the gain error), one figure, four traces:
%   each writing with its shape constant locked at its anchor (thick, the
%   clean writing-versus-writing comparison, no estimator involved) and free
%   (thin). Single seed; the 12-seed statistics live in the page's table.
%
%   Figures -> derivation/figures/formB_page7_perp.png
%              derivation/figures/formB_page8_para.png

    if nargin < 1; opts = struct(); end
    if ~isfield(opts, 'ylim'); opts.ylim = {[-9 4], [-16 6]}; end

    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    fig_dir = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    L = load(fullfile(root, 'test_results', 'temp_formB_two_boundaries.mat'));
    r = L.res;

    NAME = {'formB_page7_perp.png', 'formB_page8_para.png'};
    FS = 20; CB = [0 0.2 0.9]; CC = [0.45 0.55 0.95]; RED = [0.8 0 0];
    tb = [0.5 1.5 3.5];

    for ib = 1:2
        t = r.trace{ib,1,1}.t;
        f = figure('Position', [60 60 1150 680], 'Color', 'w', 'Visible', 'off');
        ax = axes(f); hold(ax, 'on');
        plot(ax, t, zeros(size(t)), '-', 'Color', RED, 'LineWidth', 1.6, ...
             'HandleVisibility', 'off');
        plot(ax, t,  2 * ones(size(t)), 'k--', 'LineWidth', 1.0, 'HandleVisibility', 'off');
        plot(ax, t, -2 * ones(size(t)), 'k--', 'LineWidth', 1.0, 'HandleVisibility', 'off');
        h = gobjects(1, 4);
        h(1) = plot(ax, t, r.trace{ib,1,2}.e, '-', 'Color', CB, 'LineWidth', 2.6);
        h(2) = plot(ax, t, r.trace{ib,2,2}.e, '-', 'Color', CC, 'LineWidth', 2.6);
        h(3) = plot(ax, t, r.trace{ib,1,1}.e, '-', 'Color', CB, 'LineWidth', 1.0);
        h(4) = plot(ax, t, r.trace{ib,2,1}.e, '-', 'Color', CC, 'LineWidth', 1.0);
        ylim(ax, opts.ylim{ib}); xlim(ax, [t(1) t(end)]);
        xlabel(ax, 'time  [s]', 'FontSize', FS, 'FontWeight', 'bold');
        ylabel(ax, 'a_z error  [%]', 'FontSize', FS, 'FontWeight', 'bold');
        legend(h, {'Form B  constant locked at its anchor', ...
                   'Form C  constant locked at its anchor', ...
                   'Form B  constant free', 'Form C  constant free'}, ...
               'Location', 'northoutside', 'Orientation', 'horizontal', ...
               'FontSize', FS - 6, 'NumColumns', 2, 'Box', 'on');
        yl = ylim(ax);
        for x = tb
            plot(ax, [x x], yl, '--', 'Color', [0.6 0.6 0.6], 'LineWidth', 1.0, ...
                 'HandleVisibility', 'off');
        end
        ylim(ax, yl);
        set(ax, 'FontSize', FS - 4, 'FontWeight', 'bold', 'LineWidth', 1.6, 'Box', 'on');
        grid(ax, 'off');
        out = fullfile(fig_dir, NAME{ib});
        exportgraphics(f, out, 'Resolution', 150); close(f);
        fprintf('wrote %s\n', out);
    end

    % ---- the numbers that go on the pages -------------------------------
    BN = {'c_perp', 'c_para'}; WN = {'B', 'C'}; LN = {'free', 'locked'};
    for ib = 1:2
        fprintf('\n--- %s, 12 seeds ---\n', BN{ib});
        fprintf('%-10s  %-16s %-16s %s\n', 'arm', 'desc %', 'osc %', 'hold %');
        for iw = 1:2
            for il = [2 1]
                T = squeeze(r.tab(ib, iw, il, :, :));
                fprintf('%s %-8s  %6.3f +- %-6.3f  %6.3f +- %-6.3f  %+6.3f +- %-6.3f\n', ...
                        WN{iw}, LN{il}, mean(T(:,1)), std(T(:,1)), ...
                        mean(T(:,2)), std(T(:,2)), mean(T(:,3)), std(T(:,3)));
            end
        end
        Tb = squeeze(r.tab(ib,1,2,:,:)); Tc = squeeze(r.tab(ib,2,2,:,:));
        fprintf('locked ratio B/C:  desc %.2f   osc %.2f   hold %.2f\n', ...
                mean(Tb(:,1))/mean(Tc(:,1)), mean(Tb(:,2))/mean(Tc(:,2)), ...
                abs(mean(Tb(:,3)))/abs(mean(Tc(:,3))));
    end
end
