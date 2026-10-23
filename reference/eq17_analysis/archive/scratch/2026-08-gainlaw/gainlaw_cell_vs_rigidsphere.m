%GAINLAW_CELL_VS_RIGIDSPHERE  Rigid sphere (Jeffrey & Onishi) vs a REAL cell.
%
%   WHAT J&O ACTUALLY GIVES.  Jeffrey & Onishi, JFM 139 (1984) 261-290, solve
%   two UNEQUAL RIGID IMPERMEABLE NO-SLIP spheres in Stokes flow and tabulate
%   ten scalar resistance functions (X^A, Y^A, Y^B, X^C, Y^C, X^G, Y^G, Y^H,
%   X^M, Y^M) plus the matching mobility functions.  We use exactly one of them,
%   X^A_11: the axisymmetric force on sphere 1 due to its own motion.  So every
%   "cell" curve drawn so far IS the rigid-sphere answer -- the paper contains
%   no cell physics at all.  It supplies CURVATURE and nothing else.
%
%   WHAT A REAL CELL ADDS.  Three corrections, none of them in J&O, sizes taken
%   from the literature review (see the numbers printed below):
%     (1) GLYCOCALYX  the hydrodynamic surface sits L above the lipid membrane,
%         L = 0.4-0.6 um in vivo.  Against distance-from-membrane this is a pure
%         horizontal shift -- and L is LARGER than our closest approach.
%     (2) ELASTICITY  the surface retreats under the squeeze pressure by
%         w0 = H (H_EHD/H)^(5/2),  H_EHD = [3pi/sqrt2 * mu U a^(3/2)/E*]^(2/5)
%         (leading-compliance, elastic half-space).
%     (3) SLIP        Vinogradova 1995 one-slipping-surface factor
%         f1* = (1/4){1 + (3H/2b)[(1+H/4b) ln(1+4b/H) - 1]}, applied to the drag.
%
%   THIS COMPOSITION IS NOT A PUBLISHED RESULT.  It is a first-order
%   superposition of three separately-derived corrections, each valid on its own
%   but never solved together.  It is here to compare MAGNITUDES, not to be
%   fitted against.  The elastic term in particular is history-dependent in
%   reality (dimple formation) and cannot be a static function of the gap.
%
%   Usage:  gainlaw_cell_vs_rigidsphere

clear; clc;
here = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(here));
addpath(fullfile(root, 'model', 'wall_effect'));
addpath(fullfile(root, 'model', 'config'));
addpath(here);
FIGDIR = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');

pc = physical_constants();  R = pc.R;                 % 2.25 um
RC   = 7;                                             % cell radius [um]
LGX  = 0.5;                                           % glycocalyx thickness [um]
ESTR = 1e3;                                           % E* [Pa]
BSLP = 2e-3;                                          % slip length [um]
FREQ = 1;  AMPL = 2.5;                                % trajectory [Hz], [um]
U    = 2*pi*FREQ*AMPL;                                % peak speed [um/s]
MU   = pc.gamma_N/(6*pi*R);                           % [pN s /um^2] = [Pa s]

lgx  = LGX/R;                                         % all in probe radii
bslp = BSLP/R;
% H_EHD in metres then normalised:  mu[Pa s] U[m/s] a[m]
H_EHD_m = ( (3*pi/sqrt(2)) * (MU*1e-3/1e-3) * ... % mu in Pa s (MU is already Pa s)
            (U*1e-6) * (R*1e-6)^1.5 / ESTR )^(2/5);
hehd = H_EHD_m*1e6/R;

fprintf('\nprobe R = %.3f um,  U_peak = %.2f um/s,  mu = %.4g Pa s\n', R, U, MU);
fprintf('cell Rc = %g um  (lam = %.3f)\n', RC, RC/R);
fprintf('glycocalyx L   = %.3f um = %.4f R\n', LGX, lgx);
fprintf('H_EHD (E*=%gPa) = %.1f nm  = %.4f R\n', ESTR, H_EHD_m*1e9, hehd);
fprintf('slip length b  = %.1f nm  = %.5f R\n', BSLP*1e3, bslp);
fprintf('our lowest gap = %.4f R = %.0f nm  -> %s\n', 2.5/R-1, (2.5-R)*1e3, ...
        ternary(2.5/R-1 < lgx, 'INSIDE the glycocalyx', 'outside'));

% ---- the three corrections, one at a time -------------------------------
gm = logspace(log10(0.02), log10(9), 6000)';          % gap from the MEMBRANE
aR = again(gm, RC/R);                                 % rigid sphere, no cell physics
aG = again(max(gm - lgx, 1e-9), RC/R);                % + glycocalyx offset
aE = again(max(gm - lgx, 1e-9) .* (1 + (hehd./max(gm-lgx,1e-9)).^2.5), RC/R);
aS = aE ./ fslip(max(gm - lgx, 1e-9), bslp);          % + slip (drag x f1*)

fprintf('\n=== gain, against distance from the MEMBRANE ===\n');
fprintf('%8s %10s %10s %10s %10s | %10s %10s %10s\n', 'gap[R]', 'rigid', ...
        '+glyx', '+elast', '+slip', 'glyx %', 'elast %', 'slip %');
for q = [0.25 0.3 0.5 1 2 5 9]
    [~, i] = min(abs(gm - q));
    fprintf('%8.2f %10.4f %10.4f %10.4f %10.4f | %9.1f%% %9.2f%% %9.2f%%\n', ...
        gm(i), aR(i), aG(i), aE(i), aS(i), 100*(aG(i)/aR(i)-1), ...
        100*(aE(i)/aG(i)-1), 100*(aS(i)/aE(i)-1));
end
fprintf('\n  glyx %% is vs the rigid sphere; elast %% and slip %% are each vs the\n');
fprintf('  previous column, so they read as successive corrections.\n');

COL_TRUE = [0.8 0 0];
CL = [0.10 0.28 0.85; 0.45 0.60 0.95; 0.70 0.80 0.99];
FS = 18; LFS = 12; AXLW = 2.0;
fig = figure('Position', [80 80 1100 730], 'Color', 'w', ...
             'NumberTitle', 'off', 'Visible', 'off');
hold on;
%   shaded = inside the glycocalyx, where no hydrodynamic model applies;
%   dotted = the lowest point of our own trajectory, which falls in there.
fill([0 lgx lgx 0], [0 0 1 1], [0.88 0.88 0.88], ...
     'EdgeColor', 'none', 'HandleVisibility', 'off');
xline(2.5/R - 1, ':', 'Color', [0.30 0.30 0.30], 'LineWidth', 2.5, ...
      'HandleVisibility', 'off');
h1 = plot(gm, aR, '-',  'Color', COL_TRUE, 'LineWidth', 3.0, ...
          'DisplayName', 'rigid sphere (J&O)');
h2 = plot(gm, aG, '--', 'Color', CL(1,:), 'LineWidth', 2.4, ...
          'DisplayName', '+ glycocalyx');
h3 = plot(gm, aS, '-.', 'Color', CL(3,:), 'LineWidth', 2.4, ...
          'DisplayName', '+ elasticity + slip');
set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');
grid off;  xlim([0 4]);  ylim([0 1]);
xlabel('gap from the membrane   (R)', 'FontSize', FS, 'FontWeight', 'bold');
ylabel('motion gain  a / a_o', 'FontSize', FS, 'FontWeight', 'bold');
legend([h1 h2 h3], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
       'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
exportgraphics(fig, fullfile(FIGDIR, 'gainlaw_cell_vs_rigidsphere.png'), 'Resolution', 150);
close(fig);
fprintf('\nfigure -> %s\n\n', fullfile(FIGDIR, 'gainlaw_cell_vs_rigidsphere.png'));


function a = again(gap, lam)
    a = 1 ./ build_truth_two_sphere(gap + 1, lam);
end

function f = fslip(H, b)
%FSLIP  Vinogradova (1995) one-slipping-surface correction factor.
    x = 4*b./H;
    f = 0.25*(1 + (6./x).*((1 + 1./x).*log(1 + x) - 1));
end

function o = ternary(c, a, b)
    if c, o = a; else, o = b; end
end
