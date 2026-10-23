%GAINLAW_B_P_ROLES  What b and p do to Form B's curve, and why they couple.
%
%   Form B:  a = 1 - [ b / ((w - w_s) + b) ]^p
%       b : scale parameter, a LENGTH in units of R (was called lam)
%       p : shape parameter, the exponent          (was called b)
%   Derived values for a rigid plane: b = 9/8 (Lorentz first reflection),
%   p = 1 (Stokes 1/r).
%
%   Writing the gap as g = w - w_s the model depends on it only through g/b,
%   so b is a pure horizontal scaling and p is the shape.  Expanding,
%       g << b :  a      ~ (p/b) g          only the RATIO p/b appears
%       g >> b :  1 - a  ~ b^p g^(-p)       p and b appear separately
%   which is why the two are indistinguishable near the wall and only the far
%   field separates them.
%
%   Usage:  gainlaw_b_p_roles

clear; clc;
here = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(here));
addpath(fullfile(root, 'model', 'wall_effect'));
FIGDIR = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');

WS = 1;  W_LO = 1.1;  W_HI = 10;  N = 20000;
OPT = optimset('MaxFunEvals', 2e5, 'MaxIter', 2e5, 'TolX', 1e-14, 'TolFun', 1e-14);
w = logspace(log10(W_LO), log10(W_HI), N)';  g = w - WS;
D = zeros(N,1);
for i = 1:N, [~, c] = calc_correction_functions(w(i)); D(i) = 1/c; end
mdl = @(bb, pp, gg) 1 - (bb./(gg + bb)).^pp;

f  = @(t) mdl(exp(t(1)), exp(t(2)), g);
gs = @(t) max(abs(f(t)./D - 1));
best = inf; tb = [];
for s = [log(9/8) 0; log(1) 0; log(1.2) log(1.1); log(0.9) log(0.9)]'
    t = fminsearch(gs, fminsearch(@(t) sqrt(mean((f(t)./D-1).^2)), s', OPT), OPT);
    if gs(t) < best, best = gs(t); tb = t; end
end
BF = exp(tb(1)); PF = exp(tb(2));
fprintf('\nw_s pinned at %g, both shape parameters free:\n', WS);
fprintf('  b = %.4f   p = %.4f   sup = %.3f%%\n', BF, PF, 100*best);
fprintf('\n%-22s %-24s %10s %10s\n', 'reading', 'formula', 'fitted', 'derived');
fprintf('%-22s %-24s %10.4f %10.4f\n', 'near-wall slope', 'p / b', PF/BF, 1.0);
fprintf('%-22s %-24s %10.4f %10.4f\n', 'half-gain gap', 'b (2^(1/p) - 1)', ...
        BF*(2^(1/PF) - 1), (9/8)*(2^(1/1) - 1));
fprintf('%-22s %-24s %10.4f %10.4f\n', 'tail exponent', 'p', PF, 1.0);
fprintf('%-22s %-24s %10.4f %10.4f\n', 'tail coefficient', 'b^p', BF^PF, 9/8);

BS = [0.5 1.125 2.5];
PS = [0.5 1.0 2.0];
fprintf('\nsweep b (p = 1)\n%8s', 'gap');
for B = BS, fprintf('%12s', sprintf('b=%.3g', B)); end
fprintf('%12s\n', 'truth');
for gg = [0.1 0.5 1.125 3 9]
    fprintf('%8.3f', gg);
    for B = BS, fprintf('%12.4f', mdl(B, 1, gg)); end
    [~, c] = calc_correction_functions(gg + WS); fprintf('%12.4f\n', 1/c);
end
fprintf('\nsweep p (b = 9/8)\n%8s', 'gap');
for P = PS, fprintf('%12s', sprintf('p=%.3g', P)); end
fprintf('%12s\n', 'truth');
for gg = [0.1 0.5 1.125 3 9]
    fprintf('%8.3f', gg);
    for P = PS, fprintf('%12.4f', mdl(9/8, P, gg)); end
    [~, c] = calc_correction_functions(gg + WS); fprintf('%12.4f\n', 1/c);
end

COL_TRUE = [0.8 0 0]; FS = 18; LFS = 13; AXLW = 2.0;
CL = [0.20 0.35 0.85; 0.45 0.60 0.95; 0.70 0.78 0.98];
fig = figure('Position', [80 80 1500 660], 'Color', 'w', ...
             'NumberTitle', 'off', 'Visible', 'off');
tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile; hold on;
ht = plot(g, D, '-', 'Color', COL_TRUE, 'LineWidth', 2.8, 'DisplayName', 'True');
hs = gobjects(1, numel(BS));
for k = 1:numel(BS)
    hs(k) = plot(g, mdl(BS(k), 1, g), '--', 'Color', CL(k,:), ...
                 'LineWidth', 2.2, 'DisplayName', sprintf('b = %.3g', BS(k)));
end
set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
xlim([0 W_HI-1]); ylim([0 1]);
xlabel('w - w_s', 'FontSize', FS, 'FontWeight', 'bold');
ylabel('motion gain  a / a_o', 'FontSize', FS, 'FontWeight', 'bold');
legend([ht hs], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
       'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');

nexttile; hold on;
ht2 = plot(g, D, '-', 'Color', COL_TRUE, 'LineWidth', 2.8, 'DisplayName', 'True');
hp = gobjects(1, numel(PS));
for k = 1:numel(PS)
    hp(k) = plot(g, mdl(9/8, PS(k), g), '--', 'Color', CL(k,:), ...
                 'LineWidth', 2.2, 'DisplayName', sprintf('p = %.3g', PS(k)));
end
set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
xlim([0 W_HI-1]); ylim([0 1]);
xlabel('w - w_s', 'FontSize', FS, 'FontWeight', 'bold');
ylabel('motion gain  a / a_o', 'FontSize', FS, 'FontWeight', 'bold');
legend([ht2 hp], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
       'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');

exportgraphics(fig, fullfile(FIGDIR, 'gainlaw_b_p_roles.png'), 'Resolution', 150);
close(fig);
fprintf('\nfigure -> %s\n', fullfile(FIGDIR, 'gainlaw_b_p_roles.png'));
