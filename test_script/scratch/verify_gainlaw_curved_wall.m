%VERIFY_GAINLAW_CURVED_WALL  S2: does the gain law survive a CURVED boundary?
%
%   The plane is one wall; a cell is a sphere.  This script asks whether Form B
%       a = 1 - [ lam / ((w - w_s) + lam) ]^b
%   still fits, whether w_s still lands on contact, and whether lam moves to the
%   value the curvature predicts.
%
%   Truth: build_truth_two_sphere (Jeffrey & Onishi 1984, all-separations form),
%   validated separately by running that file with no arguments.
%
%   PREDICTION under test.  Near contact the two-sphere lubrication resistance is
%       c_perp -> [lam_JO/(1+lam_JO)]^2 / eps  =  (R_eff/R)^2 / eps
%   so a ~ eps (1+lam_JO)^2/lam_JO^2, while Form B gives a ~ eps/lam.  Hence
%       lam  ->  (R_eff/R)^2 = [lam_JO/(1+lam_JO)]^2
%   R_eff = R*Rc/(R+Rc) is the standard sphere-sphere reduced radius.  For the
%   plane (lam_JO -> inf) this is 1, not 9/8, because 9/8 is a FAR-field number
%   while this is a NEAR-field one; the fit sees a blend of the two.
%
%   Usage:  verify_gainlaw_curved_wall

clear; clc;
here = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(here));
addpath(fullfile(root, 'model', 'wall_effect'));
addpath(fullfile(root, 'model', 'config'));
addpath(here);

pc    = physical_constants();
R     = pc.R;                       % probe radius [um]
W_LO  = 1.1;  W_HI = 10;  N = 8000;
OPT   = optimset('MaxFunEvals', 2e5, 'MaxIter', 2e5, ...
                 'TolX', 1e-14, 'TolFun', 1e-14);
RC    = [5 7 10 20];                % cell radii [um]

w = logspace(log10(W_LO), log10(W_HI), N)';

% Form B : theta = [ln lam, ln b, w_s]
fB  = @(t) 1 - (exp(t(1))./(max(w - t(3), 0) + exp(t(1)))).^exp(t(2));
% Form A : theta = [ln b, ln(p+1), w_s]
fA  = @(t) 1 - exp(-(exp(t(1))/exp(t(2))).*max(w - t(3), 1e-12).^exp(t(2)));

fprintf('\n=================================================================\n');
fprintf(' S2  CURVED RIGID BOUNDARY   probe R = %.2f um,  w in [%.1f, %g]\n', R, W_LO, W_HI);
fprintf('=================================================================\n');
fprintf('\n%9s %8s %9s | %8s %8s %9s %9s | %8s %9s\n', ...
        'Rc[um]', 'lam_JO', 'lam pred', ...
        'B: lam', 'B: b', 'B: ws', 'B: sup%', 'A: sup%', 'A: ws');

for rc = [RC inf]
    if isinf(rc)
        D = zeros(N,1);
        for i = 1:N, [~, cc] = calc_correction_functions(w(i)); D(i) = 1/cc; end
        LJ = inf; pred = 1;                       % plane: near-field lam -> 1
    else
        LJ   = rc/R;
        D    = 1 ./ build_truth_two_sphere(w, LJ);
        pred = (LJ/(1+LJ))^2;
    end

    FB = fit_minimax(fB, D, W_LO, OPT, ...
         [log(max(pred,0.2)) 0 1.00; log(1) 0 0.95; log(1.06) log(0.96) 1.00; ...
          log(0.6) log(0.9) 1.02; log(1.2) log(1.1) 0.90]);
    FA = fit_minimax(fA, D, W_LO, OPT, ...
         [log(0.4) log(0.65) 1.05; log(1) log(1) 1.00; log(0.7) log(0.4) 0.95; ...
          log(0.9) log(0.8) 1.05]);

    fprintf('%9s %8.3f %9.4f | %8.4f %8.4f %9.4f %9.3f | %8.3f %9.4f\n', ...
        ternstr(isinf(rc), 'plane', sprintf('%.0f', rc)), LJ, pred, ...
        exp(FB.t(1)), exp(FB.t(2)), FB.t(3), 100*FB.sup, 100*FA.sup, FA.t(3));
end

fprintf('\n  lam pred = [lam_JO/(1+lam_JO)]^2 = (R_eff/R)^2   (near-field prediction)\n');
fprintf('  ws should stay at 1 (contact) for every boundary\n');

% --- how far does the gain curve itself move? ----------------------------
fprintf('\n=== how different is a cell from a plane? (true gain) ===\n');
fprintf('%8s', 'w');  for rc = RC, fprintf('%10s', sprintf('Rc=%g', rc)); end
fprintf('%10s\n', 'plane');
for ww = [1.1 1.5 2 3 5 10]
    fprintf('%8.1f', ww);
    for rc = RC, fprintf('%10.4f', 1/build_truth_two_sphere(ww, rc/R)); end
    [~, cc] = calc_correction_functions(ww);
    fprintf('%10.4f\n', 1/cc);
end


function F = fit_minimax(f, D, w_lo, opt, seeds)
    guard = @(t) 1e6*max(0, t(end) - (w_lo - 1e-3))^2;
    gs = @(t) max(abs(f(t)./D - 1)) + guard(t);
    gr = @(t) sqrt(mean((f(t)./D - 1).^2)) + guard(t);
    best = inf; tb = [];
    for k = 1:size(seeds,1)
        t = fminsearch(gs, fminsearch(gr, seeds(k,:), opt), opt);
        t = fminsearch(gs, t, opt);
        if gs(t) < best, best = gs(t); tb = t; end
    end
    F.t = tb; F.sup = max(abs(f(tb)./D - 1));
end

function s = ternstr(c, a, b)
    if c, s = a; else, s = b; end
end
