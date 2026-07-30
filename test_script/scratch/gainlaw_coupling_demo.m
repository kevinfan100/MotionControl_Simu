%GAINLAW_COUPLING_DEMO  The b-p coupling of Form B, shown as curves.
%
%   Form B:  a = 1 - [ b/((w-w_s)+b) ]^p ,  b = scale (length), p = exponent.
%   Three parameter sets from the floor of the error valley: for each b the best
%   p and w_s were found separately.  b spans 1.75x, p spans 1.46x,
%   yet the three gain curves lie within a few percent of one another and of the
%   truth.  That is the operational meaning of the coupling: the gain curve does
%   not distinguish them, so b and p cannot be reported as separate physical
%   readings -- only the combination that runs across the valley (p/b) is
%   determined by the data.
%
%   Usage:  gainlaw_coupling_demo

clear; clc;
here = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(here));
addpath(fullfile(root, 'model', 'wall_effect'));
FIGDIR = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');

N = 3000;  w = logspace(log10(1.1), log10(10), N)';  g = w - 1;
D = zeros(N,1);
for i = 1:N, [~, c] = calc_correction_functions(w(i)); D(i) = 1/c; end
mdl = @(bb,pp,s) 1 - (bb./(max(w - s, 0) + bb)).^pp;

% valley floor: for each b, the best (p, w_s)
BS = [0.80 1.05 1.40];
opt  = optimset('TolX',1e-12,'TolFun',1e-12,'MaxFunEvals',5e4,'MaxIter',5e4);
PAR  = zeros(numel(BS), 3);
for k = 1:numel(BS)
    x = fminsearch(@(x) max(abs(mdl(BS(k), exp(x(1)), x(2))./D - 1)) ...
                        + 1e6*max(0, x(2) - 1.09)^2, [0 1], opt);
    PAR(k,:) = [BS(k), exp(x(1)), x(2)];
end
fprintf('%8s %8s %8s %10s %10s\n', 'b', 'p', 'ws', 'p/b', 'sup %');
for k = 1:numel(BS)
    e = max(abs(mdl(PAR(k,1), PAR(k,2), PAR(k,3))./D - 1));
    fprintf('%8.2f %8.4f %8.4f %10.4f %9.3f%%\n', PAR(k,:), PAR(k,2)/PAR(k,1), 100*e);
end
fprintf('b spans %.2fx, p spans %.2fx, p/b spans %.2fx\n', ...
        max(PAR(:,1))/min(PAR(:,1)), max(PAR(:,2))/min(PAR(:,2)), ...
        max(PAR(:,2)./PAR(:,1))/min(PAR(:,2)./PAR(:,1)));

COL_TRUE = [0.8 0 0];
CL = [0.15 0.30 0.85; 0.45 0.60 0.95; 0.70 0.80 0.99];
FS = 18; LFS = 12; AXLW = 2.0;
fig = figure('Position', [80 80 1500 660], 'Color', 'w', ...
             'NumberTitle', 'off', 'Visible', 'off');
tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile; hold on;
ht = plot(g, D, '-', 'Color', COL_TRUE, 'LineWidth', 3.0, 'DisplayName', 'True');
hs = gobjects(1, numel(BS));
for k = 1:numel(BS)
    hs(k) = plot(g, mdl(PAR(k,1), PAR(k,2), PAR(k,3)), '--', 'Color', CL(k,:), ...
        'LineWidth', 2.2, ...
        'DisplayName', sprintf('b=%.2f, p=%.2f', PAR(k,1), PAR(k,2)));
end
set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
xlim([0 9]); ylim([0 1]);
xlabel('w - w_s', 'FontSize', FS, 'FontWeight', 'bold');
ylabel('motion gain  a / a_o', 'FontSize', FS, 'FontWeight', 'bold');
legend([ht hs], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
       'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');

nexttile; hold on;
yline(0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off');
h2 = gobjects(1, numel(BS));
for k = 1:numel(BS)
    h2(k) = plot(g, 100*(mdl(PAR(k,1), PAR(k,2), PAR(k,3))./D - 1), '--', ...
        'Color', CL(k,:), 'LineWidth', 2.4, ...
        'DisplayName', sprintf('b=%.2f, p=%.2f', PAR(k,1), PAR(k,2)));
end
set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
xlim([0.1 9]); ylim([-4 4]);
xlabel('w - w_s', 'FontSize', FS, 'FontWeight', 'bold');
ylabel('gain error  (%)', 'FontSize', FS, 'FontWeight', 'bold');
legend(h2, 'Location', 'northoutside', 'Orientation', 'horizontal', ...
       'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');

exportgraphics(fig, fullfile(FIGDIR, 'gainlaw_coupling_demo.png'), 'Resolution', 150);
close(fig);
fprintf('figure -> %s\n', fullfile(FIGDIR, 'gainlaw_coupling_demo.png'));
