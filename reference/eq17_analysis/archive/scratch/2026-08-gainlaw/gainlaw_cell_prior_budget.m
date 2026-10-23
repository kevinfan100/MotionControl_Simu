%GAINLAW_CELL_PRIOR_BUDGET  What would a CELL boundary cost in prior width?
%
%   The plane case fixes (b, p) = (9/8, 1) from two asymptotic limits that
%   AGREE to 12.5 %, and the interpolation gap over the planned envelope IS the
%   prior width (run_formB_ws/local_envelope_priors).  This script runs the
%   identical construction against the exact two-sphere truth (Jeffrey & Onishi
%   1984, build_truth_two_sphere) with the CELL anchors:
%       near wall   p/b = [(1+lam)/lam]^2          (reduced-radius squeeze film)
%       far field   p = 2,  b = (3/2) sqrt(lam)    (re-reflection only)
%   so the question "can we just re-anchor for a cell?" gets a number instead
%   of an opinion.
%
%   Read-off formulas (same algebra as the plane, general anchor (b0, p0)):
%       g      = w - 1,   a_bar_true = 1/c
%       b_eff  = g / ( (c/(c-1))^(1/p0) - 1 )                 level reading
%       p_eff  = -c' (g + b0) / ( c (c-1) )                   slope reading
%   Prior width = sup |theta_eff - theta_0| over the PLANNED envelope, exactly
%   as the plane version does.  Nothing here is tuned.
%
%   Usage:  gainlaw_cell_prior_budget

clear; clc;
here = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(here));
addpath(fullfile(root, 'model', 'wall_effect'));
addpath(fullfile(root, 'model', 'config'));
addpath(here);

pc = physical_constants();  R = pc.R;
W_LO = 1.90;  W_HI = 23.22;      % planned envelope of the canonical scenario
N    = 4001;
RC   = [2.5 4 7 9 10];           % cell radii [um]

w = linspace(W_LO, W_HI, N).';
g = w - 1;

fprintf('\nprobe radius R = %.2f um; envelope w_bar in [%.2f, %.2f]\n', R, W_LO, W_HI);

% ---- reference: the plane, its own anchors, its own prior width -----------
[cpl, dcpl] = truth_plane(w);
[bpl, ppl, flpl] = read_off(w, cpl, dcpl, 9/8, 1);
fprintf('\n=== PLANE (published anchors 9/8, 1) ===\n');
fprintf('  sup|b_eff - b0| = %.4f   sup|p_eff - p0| = %.4f   shape floor %.5f\n', ...
        bpl, ppl, flpl);
fprintf('  (driver reports 0.0157 / 0.0245 on this envelope)\n');

% ---- the cells ------------------------------------------------------------
fprintf('\n=== CELL anchors, per radius ===\n');
fprintf('%8s %7s | %9s %9s %9s | %9s %9s\n', 'Rc[um]', 'lam', ...
        'b far', 'b near', 'tension', 'p far', 'p near');
for k = 1:numel(RC)
    lam  = RC(k)/R;
    bfar = 1.5*sqrt(lam);                 % far field, with p = 2
    bnear = 2/((1+lam)/lam)^2;            % near wall, same p = 2 via p/b anchor
    fprintf('%8.1f %7.3f | %9.4f %9.4f %8.2fx | %9d %9s\n', RC(k), lam, ...
            bfar, bnear, max(bfar/bnear, bnear/bfar), 2, '(p/b fix)');
end
fprintf('  plane for comparison: b far 1.1250, b near 1.0000, tension 1.12x\n');

fprintf('\n=== prior width the cell anchors would demand on this envelope ===\n');
fprintf('%8s %7s | %10s %10s %10s | %8s\n', 'Rc[um]', 'lam', ...
        'sqrt Pbb', 'sqrt Ppp', 'floor a', 'vs plane');
for k = 1:numel(RC)
    lam = RC(k)/R;
    b0  = 1.5*sqrt(lam);  p0 = 2;
    [c, dc] = truth_cell(w, lam);
    [sb, sp, fl] = read_off(w, c, dc, b0, p0);
    fprintf('%8.1f %7.3f | %10.4f %10.4f %10.5f | %7.1fx\n', ...
            RC(k), lam, sb, sp, fl, sb/bpl);
end

% ---- how far outside the PLANE prior does each cell truth sit? ------------
fprintf('\n=== each cell read through the PLANE anchors (what today''s filter sees) ===\n');
fprintf('%8s %7s | %10s %10s | %12s\n', 'Rc[um]', 'lam', 'sup|db|', 'sup|dp|', ...
        'db / 0.0157');
for k = 1:numel(RC)
    lam = RC(k)/R;
    [c, dc] = truth_cell(w, lam);
    [sb, sp] = read_off(w, c, dc, 9/8, 1);
    fprintf('%8.1f %7.3f | %10.4f %10.4f | %11.0fx\n', RC(k), lam, sb, sp, sb/bpl);
end

% ---- gain error of a plane-anchored law on each cell ----------------------
fprintf('\n=== gain error, plane-anchored law vs cell truth [%% of a] ===\n');
a_plane = 1 - (1 + g/(9/8)).^(-1);
fprintf('%8s %7s | %10s %10s\n', 'Rc[um]', 'lam', 'sup err', 'err at contact');
for k = 1:numel(RC)
    lam = RC(k)/R;
    c = truth_cell(w, lam);
    e = 100*(a_plane - 1./c) .* c;      % relative to the true gain 1/c
    fprintf('%8.1f %7.3f | %9.1f%% %9.1f%%\n', RC(k), lam, max(abs(e)), e(1));
end
% ---- ROUTE B: the parallel-axis construction applied to a cell -----------
%   Stage 1a's precedent: instead of the two asymptotic anchors, fit the law
%   offline to the published truth on the planned envelope with the ORIGIN
%   FREE, and take the prior width from how much the constants move when the
%   inputs move (envelope ends, and here also the cell radius, which is a
%   measured quantity).  Same charter status as the x/y law: offline, published
%   curve, no tuning.
fprintf('\n=== ROUTE B: offline minimax fit per cell (origin free), Stage-1a style ===\n');
fprintf('%8s %7s | %8s %8s %8s | %9s | %s\n', 'Rc[um]', 'lam', 'b', 'p', 'ws0', ...
        'fit sup', 'widths [db dp dws] from env +-0.1 / Rc +-10%');
TH0 = [0.5 1.16 1.0];
for k = 1:numel(RC)
    lam = RC(k)/R;
    [th, sup] = fit_cell(w, lam, TH0);
    probes = {[W_LO+0.1 W_HI lam], [W_LO-0.1 W_HI lam], [W_LO W_HI 1.1*lam], ...
              [W_LO W_HI 0.9*lam]};
    dth = zeros(numel(probes), 3);
    for j = 1:numel(probes)
        wp = linspace(probes{j}(1), probes{j}(2), N).';
        dth(j, :) = fit_cell(wp, probes{j}(3), th) - th;
    end
    width = max(abs(dth), [], 1);
    fprintf('%8.1f %7.3f | %8.4f %8.4f %8.4f | %8.3f%% | [%.4f %.4f %.4f]\n', ...
            RC(k), lam, th(1), th(2), th(3), 100*sup, width(1), width(2), width(3));
end
fprintf('  x/y plane-parallel reference (in production): b 0.5217 p 0.9770 ws0 0.5918\n');
fprintf('  widths [0.0103 0.0061 0.0145], sup 0.038%%\n\n');


% ---------------------------------------------------------------------------
function [c, dc] = truth_plane(w)
    c = zeros(size(w));  dc = zeros(size(w));
    for i = 1:numel(w)
        [~, c(i), dv] = calc_correction_functions(w(i), true);
        dc(i) = dv.dc_perp_dh;
    end
end

function [c, dc] = truth_cell(w, lam)
    c = build_truth_two_sphere(w, lam);
    c = c(:);
    if nargout > 1
        h = 1e-5;
        dc = (build_truth_two_sphere(w + h, lam) - build_truth_two_sphere(w - h, lam))/(2*h);
        dc = dc(:);
    end
end

function [th, sup] = fit_cell(w, lam, th0)
%FIT_CELL  3-parameter minimax fit of Form B to the cell gain 1/c on [w].
    a_target = 1 ./ build_truth_two_sphere(w, lam);
    a_target = a_target(:);
    obj = @(t) max(abs(1 - (1 + max(w - t(3), 0.01)/t(1)).^(-t(2)) - a_target));
    so  = optimset('TolX', 1e-10, 'TolFun', 1e-12, 'MaxFunEvals', 2e4, ...
                   'MaxIter', 2e4, 'Display', 'off');
    th  = fminsearch(obj, th0, so);
    th  = fminsearch(obj, th,  so);
    sup = obj(th);
end

function [sup_b, sup_p, floor_a] = read_off(w, c, dc, b0, p0)
    g = w - 1;
    b_eff = g ./ ((c ./ (c - 1)).^(1/p0) - 1);
    p_eff = -dc .* (g + b0) ./ (c .* (c - 1));
    sup_b = max(abs(b_eff - b0));
    sup_p = max(abs(p_eff - p0));
    a_anch = 1 - (1 + g/b0).^(-p0);
    floor_a = max(abs(a_anch - 1./c));
end
