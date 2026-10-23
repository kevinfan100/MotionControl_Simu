%GAINLAW_FIT_CELL  Can Form B's (b, p) absorb a CELL instead of a plane?
%
%   Same family, same metric, same range as the plane study; only the truth is
%   swapped for the two-sphere result (Jeffrey & Onishi 1984).
%
%   THE TWO ANCHORS ARE NOT THE SAME AS THE PLANE'S.
%   near wall   the reduced radius R_eff = R*Rc/(R+Rc) sets the squeeze film,
%               so  a ~ [(1+lam)/lam]^2 * gap,  lam = Rc/R, giving
%                   p/b = [(1+lam)/lam]^2                    (plane: 1)
%   far field   a plane reflects a sphere's Stokeslet through an image system
%               and the deficit decays as 1/gap.  A finite sphere held fixed
%               only re-reflects, so the deficit decays as 1/gap^2:
%                   1-a ~ (9/4) lam / gap^2   =>  p = 2, b = (3/2) sqrt(lam)
%               (plane: p = 1, b = 9/8)
%   Those two are checked numerically below before any fitting, because if the
%   far-field exponent really is 2 then the plane's derived p = 1 cannot be
%   carried over and the whole prior has to be re-derived for a cell.
%
%   Usage:  gainlaw_fit_cell

clear; clc;
here = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(here));
addpath(fullfile(root, 'model', 'wall_effect'));
addpath(fullfile(root, 'model', 'config'));
addpath(here);

pc = physical_constants();  R = pc.R;
RC = [4 7 10];
W_LO = 1.1;  W_HI = 10;  N = 8000;
OPT = optimset('MaxFunEvals', 2e5, 'MaxIter', 2e5, 'TolX', 1e-14, 'TolFun', 1e-14);

% ---- (0) far-field exponent: is the cell deficit really 1/gap^2? ---------
fprintf('\n=== far-field decay exponent  -d ln(1-a) / d ln(gap) ===\n');
fprintf('%10s %10s', 'gap', 'plane');
for k = 1:numel(RC), fprintf('%12s', sprintf('Rc=%gum', RC(k))); end
fprintf('\n');
for gq = [10 30 100 300 1000]
    ww = gq*[1 1.01] + 1;
    dp = zeros(1,2);
    for j = 1:2, [~, c] = calc_correction_functions(ww(j)); dp(j) = 1 - 1/c; end
    fprintf('%10g %10.3f', gq, -diff(log(dp))/diff(log(ww-1)));
    for k = 1:numel(RC)
        dc = 1 - 1./build_truth_two_sphere(ww, RC(k)/R);
        fprintf('%12.3f', -diff(log(dc))/diff(log(ww-1)));
    end
    fprintf('\n');
end
fprintf('  plane -> 1 (image system);  cell -> 2 (re-reflection only)\n');

fprintf('\n=== the two anchors, per boundary ===\n');
fprintf('%-14s %14s %14s %14s %14s\n', 'boundary', 'near p/b', 'far p', ...
        'far b (p=2)', 'near b (p=2)');
fprintf('%-14s %14.4f %14d %14s %14s\n', 'plane', 1, 1, '9/8 = 1.1250', '1.0000');
for k = 1:numel(RC)
    lam = RC(k)/R;
    fprintf('%-14s %14.4f %14d %14.4f %14.4f\n', sprintf('cell %g um', RC(k)), ...
            ((1+lam)/lam)^2, 2, 1.5*sqrt(lam), 2/((1+lam)/lam)^2);
end
fprintf('  plane: the two b differ by 12.5%%.  cell: they differ by the\n');
fprintf('  ratio in the last two columns -- far larger.\n');

% ---- (1) best-case fit of Form B to each boundary ------------------------
w = logspace(log10(W_LO), log10(W_HI), N)';
Dpl = zeros(N,1);
for i = 1:N, [~, c] = calc_correction_functions(w(i)); Dpl(i) = 1/c; end

fB = @(t, ww) 1 - (exp(t(1))./(max(ww - t(3), 0) + exp(t(1)))).^exp(t(2));
SEED = [log(9/8) 0 1.00; log(0.5) log(1.2) 1.00; log(1) log(1.5) 0.95; ...
        log(2) log(2) 0.90; log(0.4) log(0.9) 1.05; log(2.6) log(2) 1.00];

fprintf('\n=== Form B best fit, w in [%.1f, %g], minimax ===\n', W_LO, W_HI);
fprintf('%-14s %10s %10s %10s %12s %12s\n', 'boundary', 'b', 'p', 'p/b', ...
        'ws-1 [um]', 'sup [%]');
[s, t] = fitB(fB, Dpl, w, W_LO, OPT, SEED);
fprintf('%-14s %10.4f %10.4f %10.4f %12.4f %12.4f\n', 'plane', ...
        exp(t(1)), exp(t(2)), exp(t(2))/exp(t(1)), (t(3)-1)*R, 100*s);
BP = zeros(numel(RC), 2);
for k = 1:numel(RC)
    Dk = 1 ./ build_truth_two_sphere(w, RC(k)/R);
    [s, t] = fitB(fB, Dk, w, W_LO, OPT, SEED);
    BP(k,:) = [exp(t(1)) exp(t(2))];
    fprintf('%-14s %10.4f %10.4f %10.4f %12.4f %12.4f\n', ...
            sprintf('cell %g um', RC(k)), exp(t(1)), exp(t(2)), ...
            exp(t(2))/exp(t(1)), (t(3)-1)*R, 100*s);
end

fprintf('\nfitted p/b against the near-wall anchor [(1+lam)/lam]^2:\n');
for k = 1:numel(RC)
    lam = RC(k)/R;
    fprintf('  cell %2g um : fitted %.4f   anchor %.4f   ratio %.3f\n', ...
            RC(k), BP(k,2)/BP(k,1), ((1+lam)/lam)^2, ...
            (BP(k,2)/BP(k,1))/(((1+lam)/lam)^2));
end

% ---- (2) what a PLANE-tuned model predicts on a cell ---------------------
fprintf('\n=== using the plane-tuned model on a cell (no refit) ===\n');
[~, tpl] = fitB(fB, Dpl, w, W_LO, OPT, SEED);
fprintf('%10s', 'gap');
for k = 1:numel(RC), fprintf('%14s', sprintf('Rc=%gum err', RC(k))); end
fprintf('\n');
for gq = [0.1 0.5 1 2 5 9]
    [~, ix] = min(abs(w - 1 - gq));
    fprintf('%10.2f', gq);
    for k = 1:numel(RC)
        Dk = 1 ./ build_truth_two_sphere(w(ix), RC(k)/R);
        fprintf('%13.1f%%', 100*(fB(tpl, w(ix))/Dk - 1));
    end
    fprintf('\n');
end

% ---- (3) figure: two (b, p) that reproduce a cell ------------------------
%   Same treatment as the plane study's gain panel.  Each cell truth is drawn
%   solid and the Form-B curve carrying its fitted (b, p) is drawn dashed on
%   top; the plane is kept as a thin reference so the size of the curvature
%   effect stays visible.  The two cells bracket the usable range.
%   Only the model lines are drawn -- the cell truths they were fitted to are
%   left out, because at this scale each fit sits on its truth and the extra
%   curves add nothing.  Reference is the rigid plane, as in the plane study.
%   Rc = 9 um is used instead of 10 because 10 returns b = 1.003, just over 1;
%   the two curves are visually identical.
FIGDIR   = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
FIGCELLS = [9 2.5];                              % both give b < 1
COL_TRUE = [0.8 0 0];
COL_F    = [0.10 0.28 0.85; 0.55 0.68 0.96];
FS = 18; LFS = 13; AXLW = 2.0;
g = w - 1;

fprintf('\n=== figure members ===\n');
FP = zeros(numel(FIGCELLS), 2);
for j = 1:numel(FIGCELLS)
    Dk = 1 ./ build_truth_two_sphere(w, FIGCELLS(j)/R);
    [sj, tj] = fitB(fB, Dk, w, W_LO, OPT, SEED);
    FP(j,:) = [exp(tj(1)) exp(tj(2))];
    fprintf('  cell %.1f um : b=%.4f  p=%.4f  ws-1=%+.4f um  sup=%.3f%%\n', ...
            FIGCELLS(j), FP(j,1), FP(j,2), (tj(3)-1)*R, 100*sj);
end

% ---- (3b) a boundary whose wall effect fades far sooner (user request) ---
%   Origin stays ON the surface, so the gain still collapses to zero at
%   contact as it must; the curve is only lifted so that it already reaches
%   A_TARGET by a gap of G_TARGET.  That is one equation for two unknowns, so
%   the far-field decay p is kept at the plane's value and b carries the
%   request:
%       1 - [b/(G + b)]^p = A   =>   r = (1-A)^(1/p),  b = r G / (1 - r)
%   Physically this is a boundary whose hydrodynamic reach is much shorter
%   than a rigid plane's -- the deficit is spent within a fraction of a radius
%   instead of over several.
%   One point does not pin the pair: requiring a_bar(G) = A leaves a whole
%   one-parameter family, one member per p:
%       r = (1-A)^(1/p),   b = r G / (1 - r)
%   They all cross at (G, A) and differ in what happens either side of it --
%   p is the far-field decay exponent (1 = plane image system, 2 = finite
%   sphere re-reflection) and p/b is the slope with which the gain leaves
%   contact, so low p means a steeper rise and a longer far-field tail.
A_TARGET  = 0.70;          % gain the curve should already have reached ...
G_TARGET  = 0.3;           % ... by this gap [radii]
P_LIST    = [1, 2, 3];     % one curve per far-field exponent
r_floor   = (1 - A_TARGET).^(1 ./ P_LIST);
B_FLOOR   = r_floor * G_TARGET ./ (1 - r_floor);
COL_FLOOR = [0.10 0.45 0.10; 0.30 0.65 0.20; 0.55 0.80 0.35];
fprintf('\n=== short-reach family through (gap %.2f, a_bar %.2f) ===\n', ...
        G_TARGET, A_TARGET);
fprintf('  %6s %10s %14s %16s\n', 'p', 'b', 'slope p/b', 'far field 1-a');
for j = 1:numel(P_LIST)
    fprintf('  %6.2f %10.4f %14.2f %16s\n', P_LIST(j), B_FLOOR(j), ...
            P_LIST(j)/B_FLOOR(j), sprintf('~ g^-%.0f', P_LIST(j)));
end
fprintf('  %6s %10.4f %14.2f %16s   (rigid plane)\n', '1', 9/8, 1/(9/8), '~ g^-1');
fprintf('\n  %6s', 'gap');
for j = 1:numel(P_LIST); fprintf(' %10s', sprintf('p=%.0f', P_LIST(j))); end
fprintf(' %12s\n', 'rigid plane');
for gq = [0 0.1 0.3 0.5 1.3 3 9]
    fprintf('  %6.1f', gq);
    for j = 1:numel(P_LIST)
        fprintf(' %10.3f', 1 - (B_FLOOR(j)/(gq + B_FLOOR(j)))^P_LIST(j));
    end
    fprintf(' %12.3f\n', 1 - (9/8/(gq + 9/8))^1);
end

fig = figure('Position', [80 80 1050 720], 'Color', 'w', ...
             'NumberTitle', 'off', 'Visible', 'off');
hold on;
ht = plot(g, Dpl, '-', 'Color', COL_TRUE, 'LineWidth', 3.0, ...
          'DisplayName', 'True');
hf = gobjects(1, numel(FIGCELLS) + numel(P_LIST));
for j = 1:numel(FIGCELLS)
    hf(j) = plot(g, 1 - (FP(j,1)./(g + FP(j,1))).^FP(j,2), '--', ...
                 'Color', COL_F(j,:), 'LineWidth', 2.6, ...
                 'DisplayName', sprintf('b=%.2f, p=%.2f', FP(j,1), FP(j,2)));
end
for j = 1:numel(P_LIST)
    hf(numel(FIGCELLS) + j) = ...
        plot(g, 1 - (B_FLOOR(j)./(g + B_FLOOR(j))).^P_LIST(j), '-.', ...
             'Color', COL_FLOOR(j,:), 'LineWidth', 2.6, ...
             'DisplayName', sprintf('b=%.2f, p=%.2f', B_FLOOR(j), P_LIST(j)));
end
set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');
grid off;  xlim([0 W_HI-1]);  ylim([0 1]);
plot([0 G_TARGET], A_TARGET*[1 1], ':', 'Color', COL_FLOOR(1,:), ...
     'LineWidth', 1.4, 'HandleVisibility', 'off');
plot(G_TARGET*[1 1], [0 A_TARGET], ':', 'Color', COL_FLOOR(1,:), ...
     'LineWidth', 1.4, 'HandleVisibility', 'off');
xlabel('w - w_s', 'FontSize', FS, 'FontWeight', 'bold');
ylabel('motion gain  a / a_o', 'FontSize', FS, 'FontWeight', 'bold');
legend([ht hf], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
       'FontSize', LFS - 2, 'FontWeight', 'bold', 'Box', 'on');
exportgraphics(fig, fullfile(FIGDIR, 'gainlaw_cell_fit.png'), 'Resolution', 150);
close(fig);
fprintf('\nfigure -> %s\n\n', fullfile(FIGDIR, 'gainlaw_cell_fit.png'));


function [sup, tb] = fitB(f, D, w, lo, opt, seeds)
    e  = @(t) f(t, w) ./ D - 1;
    gd = @(t) 1e6 * max(0, t(3) - (lo - 1e-3))^2;
    fr = @(t) sqrt(mean(e(t).^2)) + gd(t);
    fs = @(t) max(abs(e(t))) + gd(t);
    best = inf; tb = [];
    for s = 1:size(seeds,1)
        t = fminsearch(fs, fminsearch(fr, seeds(s,:), opt), opt);
        t = fminsearch(fs, t, opt);
        if fs(t) < best, best = fs(t); tb = t; end
    end
    sup = max(abs(e(tb)));
end
