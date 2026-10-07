% PURPOSE (2026-10-05, user: "why do we need I at all?"): scalar walk of the variance of c with the same data every step (I = 2),
%   from three start values. Left: Q55 = what the update just removed (P^f - P) -> P stays wherever it started.
%   Right: Q55 = P_fix^2 I (what the update WOULD remove at P_fix) -> every start goes to P_fix = 0.04.   STATUS: SCRATCH
function plot_why_I_walk()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    I = 2;  Pfix = 0.04;  N = 150;  P0 = [0.01 0.04 0.09];  C = {[0 0.2 0.9], [0.5 0.5 0.5], [0.9 0.5 0]};
    FS = 18;  LFS = 13;  AXLW = 2.0;
    f = figure('Position', [40 40 1600 600], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for panel = 1:2
        ax = nexttile(tl);  hold on; box on;  h = gobjects(0);
        for j = 1:numel(P0)
            Pf = zeros(1, N + 1);  Pf(1) = P0(j);
            for k = 1:N
                P = 1 / (1 / Pf(k) + I);                                  % update: 1/P = 1/P^f + I
                if panel == 1; Q = Pf(k) - P; else; Q = Pfix^2 * I; end   % left: give back what was removed; right: P_fix^2 I
                Pf(k + 1) = P + Q;                                         % prediction
            end
            h(end + 1) = plot(ax, 0:N, Pf, '-', 'Color', C{j}, 'LineWidth', 2.2, 'DisplayName', sprintf('start P^f_{55} = %.2f', P0(j))); %#ok<AGROW>
        end
        plot(ax, [0 N], [Pfix Pfix], ':', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.4);
        xlim(ax, [0 N]);  ylim(ax, [0 0.1]);  xlabel(ax, 'step (same data every step, I = 2)', 'FontSize', FS, 'FontWeight', 'bold');
        if panel == 1; ylabel(ax, 'P^f_{55}', 'FontSize', FS, 'FontWeight', 'bold'); end
        lg = legend(ax, h, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'AutoUpdate', 'off');
        if panel == 1; lg.Title.String = 'Q_{55} = P^f_{55} - P_{55} (give back what was removed)'; else; lg.Title.String = 'Q_{55} = P_{fix}^2 I, P_{fix} = 0.04 (dotted)'; end
        set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(ax, 'off');
    end
    out = fullfile(fd, 'fig_why_I_walk.png');  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end
