%GAINLAW_CELL_VS_PLANE_TRUTH  The truth curve itself: rigid plane vs a cell.
%
%   Everything so far has been fitted against ONE truth: the rigid, flat,
%   no-slip wall (Brenner 1961), which is what calc_correction_functions ships.
%   A cell is neither flat nor rigid nor strictly no-slip.  This script isolates
%   the FIRST of those three -- curvature -- because it is the only one with an
%   exact solution available (Jeffrey & Onishi 1984, two rigid spheres).  Softness
%   and the glycocalyx are separate corrections and are NOT included here.
%
%   Coordinate is the same in both cases: w = probe centre to the boundary
%   SURFACE, divided by the probe radius, so contact is at w = 1 for every cell
%   radius and the curves are directly comparable.
%
%   WHY THE CURVES SEPARATE.  Near contact the lubrication resistance scales with
%   the reduced radius R_eff = R*Rc/(R+Rc):
%       plane  c_perp -> 1/eps            (R_eff = R)
%       cell   c_perp -> (R_eff/R)^2/eps = [lam/(1+lam)]^2 / eps,  lam = Rc/R
%   A finite cell has R_eff < R, so LESS fluid has to be squeezed out, the drag
%   is smaller and the gain is HIGHER than the plane at the same gap.  The effect
%   is largest near contact and vanishes in the far field.
%
%   Usage:  gainlaw_cell_vs_plane_truth

clear; clc;
here = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(here));
addpath(fullfile(root, 'model', 'wall_effect'));
addpath(fullfile(root, 'model', 'config'));
addpath(here);
FIGDIR = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');

pc = physical_constants();  R = pc.R;
RC = [4 7 10];                      % cell radii [um]  (RBC ~ 4, typical ~7-10)
W_LO = 1.02;  W_HI = 10;  N = 4000;
w = logspace(log10(W_LO), log10(W_HI), N)';  g = w - 1;

Dpl = zeros(N,1);
for i = 1:N, [~, c] = calc_correction_functions(w(i)); Dpl(i) = 1/c; end
Dc = zeros(N, numel(RC));
for k = 1:numel(RC)
    Dc(:,k) = 1 ./ build_truth_two_sphere(w, RC(k)/R);
end

fprintf('\nprobe radius R = %.3f um\n', R);
fprintf('\n%8s %10s', 'gap', 'plane');
for k = 1:numel(RC), fprintf('%12s', sprintf('Rc=%gum', RC(k))); end
fprintf('   |');
for k = 1:numel(RC), fprintf('%10s', sprintf('+%%', RC(k))); end
fprintf('\n');
for gg = [0.02 0.05 0.1 0.2 0.5 1 2 5 9]
    [~, ix] = min(abs(g - gg));
    fprintf('%8.2f %10.4f', g(ix), Dpl(ix));
    for k = 1:numel(RC), fprintf('%12.4f', Dc(ix,k)); end
    fprintf('   |');
    for k = 1:numel(RC), fprintf('%9.1f%%', 100*(Dc(ix,k)/Dpl(ix) - 1)); end
    fprintf('\n');
end

fprintf('\nnear-contact prediction  gain ratio -> [(1+lam)/lam]^2, lam = Rc/R:\n');
for k = 1:numel(RC)
    lam = RC(k)/R;
    [~, ix] = min(abs(g - 0.02));
    fprintf('  Rc=%2g um  lam=%5.2f   predicted %6.3f   measured at gap 0.02 %6.3f\n', ...
            RC(k), lam, ((1+lam)/lam)^2, Dc(ix,k)/Dpl(ix));
end

COL_TRUE = [0.8 0 0];
CL = [0.10 0.28 0.85; 0.42 0.56 0.94; 0.68 0.77 0.98];
FS = 18; LFS = 13; AXLW = 2.0;
fig = figure('Position', [80 80 1050 720], 'Color', 'w', ...
             'NumberTitle', 'off', 'Visible', 'off');
hold on;
hp = plot(g, Dpl, '-', 'Color', COL_TRUE, 'LineWidth', 3.0, ...
          'DisplayName', 'rigid plane');
hc = gobjects(1, numel(RC));
for k = 1:numel(RC)
    hc(k) = plot(g, Dc(:,k), '--', 'Color', CL(k,:), 'LineWidth', 2.4, ...
                 'DisplayName', sprintf('cell R_c = %g \\mum', RC(k)));
end
set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');
grid off;  xlim([0 W_HI-1]);  ylim([0 1]);
xlabel('w - w_s', 'FontSize', FS, 'FontWeight', 'bold');
ylabel('motion gain  a / a_o', 'FontSize', FS, 'FontWeight', 'bold');
legend([hp hc], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
       'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');

exportgraphics(fig,fullfile(FIGDIR, 'gainlaw_cell_vs_plane_truth.png'), 'Resolution', 150);
close(fig);
fprintf('\nfigure -> %s\n\n', fullfile(FIGDIR, 'gainlaw_cell_vs_plane_truth.png'));
