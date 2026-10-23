% STATUS: ACTIVE (scratch) | PURPOSE: figure for run_ainit_offset_pair.m. One figure
%   per trajectory, two columns sharing the y axis per row (A4 a'_true compensated |
%   BT b_true exact). Row 1: cross-seed mean error (a_hat - a_bar) for every seed
%   offset delta, one line per delta. Row 2: the same divided by its own E(0)
%   (delta ~= 0 only): 1 = nothing corrected, 0 = fully corrected. Console prints
%   the hold recovery per delta. Notation: a_hat_z / a_nom, delta in % of a_bar(0).
function plot_ainit_offset_pair(traj)
    if nargin < 1 || isempty(traj); traj = 'canon'; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    S = load(fullfile(od, sprintf('ainit_offset_pair_%s.mat', traj)));
    t = S.t(:); DEL = S.DEL; nD = numel(DEL); FS = 13;
    cmap = [0.55 0 0; 0.85 0.2 0.2; 1.0 0.55 0.55; 0 0 0; 0.55 0.75 1.0; 0.2 0.45 0.9; 0 0.1 0.55];
    if nD ~= 7; cmap = interp1(linspace(0,1,7), cmap, linspace(0,1,nD)); end
    f = figure('Units','inches','Position',[0 0 14 8.5], 'Color','w', 'Visible','off');
    tl = tiledlayout(2, 2, 'TileSpacing','compact', 'Padding','compact');
    ax = gobjects(2,2); H = gobjects(1,nD); LG = cell(1,nD);
    for a = 1:2
        ax(1,a) = nexttile(a); hold on;
        for i = 1:nD
            E = S.E{a,i}; if isempty(E); continue; end
            H(i) = plot(t, mean(E,2), '-', 'Color', cmap(i,:), 'LineWidth', 1.6);
            LG{i} = sprintf('\\delta = %+.1f%%', 100*DEL(i));
        end
        yline(0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 0.8, 'HandleVisibility','off');
        ylabel('mean (\^a_z - a_z) / a_{nom}', 'FontSize', FS, 'FontWeight','bold');
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',1.4,'Box','on'); grid off;
        text(0.02, 0.95, S.arms{a}, 'Units','normalized', 'FontSize', FS, 'FontWeight','bold', 'VerticalAlignment','top', 'Interpreter','none');
        ax(2,a) = nexttile(2+a); hold on;
        for i = 1:nD
            E = S.E{a,i}; if isempty(E) || DEL(i) == 0; continue; end
            m = mean(E,2); plot(t, m / m(1), '-', 'Color', cmap(i,:), 'LineWidth', 1.6);
        end
        yline(0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 0.8); yline(1, '--', 'Color', [0.4 0.4 0.4], 'LineWidth', 0.8);
        ylabel('E(t) / E(0)   (1 = uncorrected)', 'FontSize', FS, 'FontWeight','bold');
        xlabel('time  [s]', 'FontSize', FS, 'FontWeight','bold');
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',1.4,'Box','on'); grid off;
    end
    linkaxes(ax(1,:), 'y'); linkaxes(ax(2,:), 'y');
    ok = isgraphics(H); legend(ax(1,1), H(ok), LG(ok), 'Location','northoutside','Orientation','horizontal', ...
        'FontSize', 10, 'FontWeight','bold', 'Box','on', 'NumColumns', nD);
    png = fullfile(od, sprintf('ainit_offset_pair_%s.png', traj));
    exportgraphics(f, png, 'Resolution', 150); close(f); fprintf('saved %s\n', png);
end
