%GAINLAW_SHAPE_EVIDENCE_PARA  The same shape argument, run on the PARALLEL axis.
%
%   Companion to gainlaw_shape_evidence.m (perpendicular).  Same two forms,
%   same two procedures, same acceptance metric -- only the truth changes.
%
%       Form A   a = 1 - exp( -(b/(p+1)) (w - w_s)^(p+1) )
%       Form B   a = 1 - [ b / ((w - w_s) + b) ]^p
%
%   WHY THE PARALLEL AXIS IS NOT A RERUN.  The perpendicular argument rested on
%   two closed-form anchors that pin (b, p) with no fitting:
%       near wall   a -> gap          (Brenner exact, D'_perp(1) = 1)  => b = 1
%       far field   1-a -> (9/8)/gap  (Lorentz first reflection)       => b = 9/8
%   and the whole result was that these two disagree by only 12.5%.
%   On the parallel axis the far-field anchor merely changes value (9/16), but
%   the NEAR-WALL ANCHOR DOES NOT EXIST: the exact parallel mobility vanishes
%   logarithmically (Goldman, Cox & Brenner 1967),
%       a_para ~ 1 / ( -(8/15) ln(eps) + 0.9588 ),
%   which is not a power of the gap.  Both forms vanish like a power, so no
%   choice of (b, p) reproduces the contact behaviour.  This script measures how
%   far that structural mismatch propagates into the operating envelope.
%
%   SECTION 0 IS NOT OPTIONAL.  The shipped c_para is the 5-term Faxen series,
%   valid for w >~ 2; at contact it returns a FINITE gain (0.424) where the true
%   parallel gain is 0.  Any failure reported below w = 2 must therefore be
%   attributed correctly: to the form, or to the truth curve being out of its
%   own domain.  Section 0 measures the gap between the shipped polynomial and
%   the Goldman asymptote so the two causes are not confused.
%
%   OUTPUTS
%     console : S0 truth-domain audit, S1 anchors, S2 pointwise travel,
%               S3 global minimax on two ranges, S4 lower-bound sweep,
%               S5 b-p coupling
%     figures : reference/eq17_analysis/derivation/figures/
%                 gainlaw_para_gain_and_error.png
%                 gainlaw_para_constant_B.png
%
%   Usage:  gainlaw_shape_evidence_para

clear; clc;
here = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(here));
addpath(fullfile(root, 'model', 'wall_effect'));
addpath(fullfile(root, 'model', 'config'));
FIGDIR = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');

pc     = physical_constants();
R_UM   = pc.R;
LAM_PA = 9/16;              % far-field reflection coefficient, PARALLEL
LAM_PE = 9/8;               % ... perpendicular, for side-by-side reference
W_LO   = 1.1;
W_HI   = 10;
N_SAMP = 20000;
OPT    = optimset('MaxFunEvals', 2e5, 'MaxIter', 2e5, ...
                  'TolX', 1e-14, 'TolFun', 1e-14);

% =====================================================================
% S0  truth-domain audit -- is the shipped c_para usable where we fit?
% =====================================================================
fprintf('\n=====================================================================\n');
fprintf(' S0  TRUTH DOMAIN -- shipped Faxen series vs Goldman log asymptote\n');
fprintf('=====================================================================\n');
gold = @(w) 1 ./ (-(8/15)*log(w - 1) + 0.9588);     % a_para, Goldman leading
fprintf('\n%8s %12s %12s %10s   %12s\n', 'w', 'a_para ship', 'a_para GCB', ...
        'diff[%]', 'a_perp ship');
for w = [1.001 1.01 1.05 1.1 1.2 1.5 2.0 3.0]
    ap = truth_at(w, 'para');  ae = truth_at(w, 'perp');
    fprintf('%8.3f %12.4f %12.4f %9.1f%%   %12.4f\n', w, ap, gold(w), ...
            100*(ap/gold(w) - 1), ae);
end
fprintf('\n  contact values:  shipped a_para(1) = %.4f   true = 0 (log law)\n', ...
        truth_at(1 + 1e-12, 'para'));
fprintf('  => the shipped polynomial does NOT vanish at contact.  Below w ~ 1.2\n');
fprintf('     it is out of domain; failures there are shared blame.\n');

fprintf('\n  near-wall effective slope  a/gap  (Form A/B both force a ~ (p/b)*gap):\n');
fprintf('%8s %14s %14s\n', 'gap', 'para  a/gap', 'perp  a/gap');
for g = [0.5 0.2 0.1 0.05 0.02 0.01]
    fprintf('%8.3f %14.3f %14.3f\n', g, truth_at(1+g,'para')/g, truth_at(1+g,'perp')/g);
end
fprintf('  perp -> 1 (a finite anchor).  para diverges -> no near-wall anchor.\n');

% =====================================================================
% S1  the two anchors, on the parallel axis
% =====================================================================
fprintf('\n=====================================================================\n');
fprintf(' S1  ANCHORS -- what (b, p) are pinned to, with no fitting\n');
fprintf('=====================================================================\n');
fprintf('\n  far field   1-a -> (lam/gap)^p    lam_para = 9/16 = %.4f\n', LAM_PA);
fprintf('              exponent 1 (Stokes 1/r)  =>  p = 1,  b = 9/16\n');
fprintf('  near wall   Form A/B both give  a ~ (p/b) gap\n');
fprintf('              perp : a/gap -> 1        =>  p/b = 1   (b = 1 when p = 1)\n');
fprintf('              para : a/gap -> infinity =>  NO anchor\n');
fprintf('\n  over-determination of b:\n');
fprintf('    perp :  b = 1      (near)  vs  b = 9/8  (far)   ->  %.1f%% tension\n', ...
        100*(LAM_PE - 1));
fprintf('    para :  b = ?      (none)  vs  b = 9/16 (far)   ->  one anchor lost\n');
fprintf('    the perp argument SURVIVED because both anchors existed and nearly\n');
fprintf('    agreed.  On para the near-wall side is a different function class.\n');

% =====================================================================
% S2  pointwise solve -- does every height want the same (b, p)?
% =====================================================================
fprintf('\n=====================================================================\n');
fprintf(' S2  POINTWISE SOLVE  (w_s pinned at 1, both shape parameters free)\n');
fprintf('=====================================================================\n');
wg = logspace(log10(W_LO), log10(W_HI), N_SAMP)';
[pA_pa, bA_pa, pB_pa, bB_pa] = deal(zeros(N_SAMP,1));
[pA_pe, bA_pe, pB_pe, bB_pe] = deal(zeros(N_SAMP,1));
for i = 1:N_SAMP
    [pA_pa(i), bA_pa(i), pB_pa(i), bB_pa(i)] = solve_local(wg(i), 'para');
    [pA_pe(i), bA_pe(i), pB_pe(i), bB_pe(i)] = solve_local(wg(i), 'perp');
end
span = @(v) max(v) - min(v);
fprintf('\n%-24s %10s %10s %10s   %10s\n', 'demanded parameter', ...
        sprintf('@w=%.1f', W_LO), sprintf('@w=%g', W_HI), 'travel', 'perp travel');
fprintf('%-24s %10.4f %10.4f %10.4f   %10.4f\n', 'Form A  p', pA_pa(1), pA_pa(end), ...
        span(pA_pa), span(pA_pe));
fprintf('%-24s %10.4f %10.4f %10.4f   %10.4f\n', 'Form A  b', bA_pa(1), bA_pa(end), ...
        span(bA_pa), span(bA_pe));
fprintf('%-24s %10.4f %10.4f %10.4f   %10.4f\n', 'Form B  p', pB_pa(1), pB_pa(end), ...
        span(pB_pa), span(pB_pe));
fprintf('%-24s %10.4f %10.4f %10.4f   %10.4f\n', 'Form B  b', bB_pa(1), bB_pa(end), ...
        span(bB_pa), span(bB_pe));
ix2 = find(wg >= 2, 1);
fprintf('\n  restricted to w >= 2:  Form B  p travel %.4f (was %.4f),', ...
        span(pB_pa(ix2:end)), span(pB_pa));
fprintf(' b travel %.4f (was %.4f)\n', span(bB_pa(ix2:end)), span(bB_pa));

% =====================================================================
% S3  global minimax on two ranges
% =====================================================================
MODELS = build_models_para(LAM_PA);
RANGES = [1.1 10; 1.5 10; 2.0 10];
fprintf('\n=====================================================================\n');
fprintf(' S3  GLOBAL MINIMAX (parallel truth)\n');
fprintf('=====================================================================\n');
FITS = cell(size(RANGES,1), numel(MODELS));
for r = 1:size(RANGES,1)
    [wf, Df] = truth_grid(RANGES(r,1), RANGES(r,2), N_SAMP, 'para');
    fprintf('\n  w in [%.1f, %g]\n', RANGES(r,1), RANGES(r,2));
    fprintf('  %-24s %-34s %9s %12s %8s\n', 'form', 'parameters', 'sup[%]', ...
            'ws bias[um]', 'equiosc');
    for m = 1:numel(MODELS)
        F = run_fit(MODELS(m), wf, Df, RANGES(r,1), OPT);
        FITS{r,m} = F;
        fprintf('  %-24s %-34s %9.4f %12.4f %5d/%-2d\n', MODELS(m).name, ...
                MODELS(m).describe(F.theta), 100*F.sup, (F.ws-1)*R_UM, ...
                F.n_alt, MODELS(m).np + 1);
    end
end
fprintf('\n  reference (perpendicular, same metric, [1.1,10]):\n');
fprintf('    Form A 4.256%%   Form B 0.429%%\n');

% =====================================================================
% S3b  w_s PINNED at the true contact -- separates two different failures
% =====================================================================
%   With w_s free, a form that cannot bend enough near the wall can hide the
%   mismatch by sliding its own origin into the wall.  Pinning w_s = 1 removes
%   that escape route, so the residual sup is the shape error alone.  The
%   difference between the two columns is exactly what w_s was being spent on.
fprintf('\n=====================================================================\n');
fprintf(' S3b  w_s PINNED AT 1  (shape error with no origin shift)\n');
fprintf('=====================================================================\n');
fprintf('\n%-10s %-10s %14s %14s %14s\n', 'axis', 'form', 'sup ws free[%]', ...
        'sup ws=1 [%]', 'penalty');
for ax = {'para', 'perp'}
    lamx = LAM_PA; if strcmp(ax{1}, 'perp'), lamx = LAM_PE; end
    [wf, Df] = truth_grid(W_LO, W_HI, N_SAMP, ax{1});
    MP = build_models_para(lamx);
    MPIN = build_models_pinned(lamx);
    for m = 1:2
        Ff = run_fit(MP(m),   wf, Df, W_LO, OPT);
        Fp = run_fit(MPIN(m), wf, Df, W_LO, OPT);
        fprintf('%-10s %-10s %14.4f %14.4f %13.1fx\n', ax{1}, MP(m).name, ...
                100*Ff.sup, 100*Fp.sup, Fp.sup/Ff.sup);
    end
end
fprintf('\n  a large penalty = the free fit was buying accuracy with w_s,\n');
fprintf('  i.e. the reported surface position is a fitting artefact.\n');

% =====================================================================
% S4  lower-bound sweep -- where does the structure break?
% =====================================================================
fprintf('\n=====================================================================\n');
fprintf(' S4  LOWER-BOUND SWEEP (upper bound fixed at %g)\n', W_HI);
fprintf('=====================================================================\n');
LO  = [1.05 1.1 1.2 1.35 1.5 1.75 2.0 2.5 3.0];
MPE = build_models_para(LAM_PE);
SW  = zeros(numel(LO), 6);
for j = 1:numel(LO)
    [wj, Dj] = truth_grid(LO(j), W_HI, N_SAMP, 'para');
    FA = run_fit(MODELS(1), wj, Dj, LO(j), OPT);
    FB = run_fit(MODELS(2), wj, Dj, LO(j), OPT);
    [~, De] = truth_grid(LO(j), W_HI, N_SAMP, 'perp');
    FE = run_fit(MPE(2), wj, De, LO(j), OPT);
    SW(j,:) = [100*FA.sup, (FA.ws-1)*R_UM, 100*FB.sup, (FB.ws-1)*R_UM, ...
               100*FE.sup, (FE.ws-1)*R_UM];
end
fprintf('\n%8s | %9s %11s | %9s %11s | %9s %11s\n', 'w_lo', ...
        'A/pa sup%', 'A/pa ws[um]', 'B/pa sup%', 'B/pa ws[um]', ...
        'B/pe sup%', 'B/pe ws[um]');
for j = 1:numel(LO)
    fprintf('%8.2f | %9.3f %11.3f | %9.3f %11.3f | %9.3f %11.3f\n', LO(j), SW(j,:));
end
fprintf('\n  the perpendicular column is the control: same form, same metric,\n');
fprintf('  and its w_s stays put while the parallel one walks into the wall.\n');

% =====================================================================
% S5  b-p coupling on the usable range
% =====================================================================
fprintf('\n=====================================================================\n');
fprintf(' S5  b-p COUPLING (Form B, parallel)\n');
fprintf('=====================================================================\n');
fprintf('\n%-14s %10s %10s %12s %12s\n', 'range', 'cond(F)', 'corr(b,p)', ...
        'corr(b,ws)', 'corr(p,ws)');
for r = 1:size(RANGES,1)
    [wf, ~] = truth_grid(RANGES(r,1), RANGES(r,2), N_SAMP, 'para');
    C = coupling(MODELS(2).f, FITS{r,2}.theta, wf);
    fprintf('[%4.1f,%4.1f]   %10.3g %10.4f %12.4f %12.4f\n', RANGES(r,1), ...
            RANGES(r,2), C.cond, C.c12, C.c13, C.c23);
end
fprintf('\n  reference (perpendicular, [1.1,10]): corr(b,p) = +0.986\n');

% =====================================================================
% S6  JOINT FIT -- one shared w_s, per-axis (b, p)
% =====================================================================
%   This is how the estimator would actually run: the surface position is a
%   property of the wall, not of the axis, so the two axes must agree on it
%   while keeping their own shape constants.  The question is whether adding
%   the parallel axis helps locate the wall or corrupts the perpendicular
%   reading that already worked.
fprintf('\n=====================================================================\n');
fprintf(' S6  JOINT FIT  (w_s shared, per-axis b and p, form B)\n');
fprintf('=====================================================================\n');
for r = 1:size(RANGES,1)
    lo = RANGES(r,1);  hi = RANGES(r,2);
    [wj, Dpa] = truth_grid(lo, hi, N_SAMP, 'para');
    [~,  Dpe] = truth_grid(lo, hi, N_SAMP, 'perp');
    fB  = @(b, p, s, w) 1 - (b ./ (max(w - s, 0) + b)).^p;
    epa = @(t) fB(exp(t(1)), exp(t(2)), t(5), wj) ./ Dpa - 1;
    epe = @(t) fB(exp(t(3)), exp(t(4)), t(5), wj) ./ Dpe - 1;
    gd  = @(t) 1e6 * max(0, t(5) - (lo - 1e-3))^2;
    jr  = @(t) sqrt(mean([epa(t); epe(t)].^2)) + gd(t);
    js  = @(t) max([max(abs(epa(t))), max(abs(epe(t)))]) + gd(t);
    SEED = [log(LAM_PA) 0 log(LAM_PE) 0 1.00; ...
            log(0.4) log(0.9) log(1.05) log(0.95) 0.95; ...
            log(0.32) log(0.82) log(1.06) log(0.96) 0.99; ...
            log(0.6) log(1.1) log(1.2) log(1.1) 0.90];
    best = inf; tb = [];
    for s = 1:size(SEED,1)
        t = fminsearch(js, fminsearch(jr, SEED(s,:), OPT), OPT);
        t = fminsearch(js, t, OPT);
        if js(t) < best, best = js(t); tb = t; end
    end
    % perpendicular alone, same range, for the comparison
    Fpe = run_fit_axis(fB, Dpe, wj, lo, OPT, LAM_PE);
    fprintf('\n  w in [%.1f, %g]\n', lo, hi);
    fprintf('    joint : ws bias %+8.4f um | para b=%.4f p=%.4f sup %.4f%%', ...
            (tb(5)-1)*R_UM, exp(tb(1)), exp(tb(2)), 100*max(abs(epa(tb))));
    fprintf(' | perp b=%.4f p=%.4f sup %.4f%%\n', exp(tb(3)), exp(tb(4)), ...
            100*max(abs(epe(tb))));
    fprintf('    perp alone : ws bias %+8.4f um | b=%.4f p=%.4f sup %.4f%%\n', ...
            (Fpe.ws-1)*R_UM, exp(Fpe.theta(1)), exp(Fpe.theta(2)), 100*Fpe.sup);
end
fprintf('\n  if the joint ws bias is worse than perp alone, the parallel axis is\n');
fprintf('  a net liability for locating the wall.\n');

% =====================================================================
% FIGURES
% =====================================================================
COL_TRUE = [0.8 0 0]; COL_A = [0 0.2 0.9]; COL_B = [0.45 0.55 0.95];
FS = 18; LFS = 13; AXLW = 2.0; BANDC = [0.93 0.86 0.86];

[wz, Dz] = truth_grid(W_LO, W_HI, 4000, 'para');
gA = MODELS(1).f(FITS{1,1}.theta, wz);
gB = MODELS(2).f(FITS{1,2}.theta, wz);
gz = wz - 1;

f1 = figure('Position', [80 80 1500 680], 'Color', 'w', ...
            'NumberTitle', 'off', 'Visible', 'off');
tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile; hold on;
ht = plot(gz, Dz, '-',  'Color', COL_TRUE, 'LineWidth', 2.6, 'DisplayName', 'True');
hA = plot(gz, gA, '--', 'Color', COL_A, 'LineWidth', 2.2, 'DisplayName', 'form A');
hB = plot(gz, gB, '-.', 'Color', COL_B, 'LineWidth', 2.2, 'DisplayName', 'form B');
set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
xlim([0 W_HI-1]); ylim([0 1]);
xlabel('w - w_s', 'FontSize', FS, 'FontWeight', 'bold');
ylabel('motion gain  a / a_o', 'FontSize', FS, 'FontWeight', 'bold');
legend([ht hA hB], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
       'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');

nexttile; hold on;
yline(0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off');
xline(1, ':', 'Color', [0.35 0.35 0.35], 'LineWidth', 2.0, 'HandleVisibility', 'off');
h2A = plot(gz, 100*(gA./Dz - 1), '--', 'Color', COL_A, 'LineWidth', 2.4, ...
           'DisplayName', 'form A');
h2B = plot(gz, 100*(gB./Dz - 1), '-.', 'Color', COL_B, 'LineWidth', 2.4, ...
           'DisplayName', 'form B');
set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
xlim([W_LO-1 W_HI-1]);
xlabel('w - w_s', 'FontSize', FS, 'FontWeight', 'bold');
ylabel('gain error  (%)', 'FontSize', FS, 'FontWeight', 'bold');
legend([h2A h2B], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
       'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
exportgraphics(f1, fullfile(FIGDIR, 'gainlaw_para_gain_and_error.png'), 'Resolution', 150);
close(f1);

% THE FAILURE FIGURE.  The gain panel above shows both forms fitting the
% parallel curve to well under 1%, which is the trap: that accuracy is bought
% by moving the model's own origin into the wall.  Left panel removes the
% purchase (w_s held at the true contact) and the parallel error jumps an order
% of magnitude while the perpendicular one barely moves.  Right panel shows the
% same defect as a drift: the fitted surface position walks steadily into the
% wall as near-wall data is removed, whereas the perpendicular fit stays put.
[wp, Dpa1] = truth_grid(W_LO, W_HI, 4000, 'para');
[~,  Dpe1] = truth_grid(W_LO, W_HI, 4000, 'perp');
MPIN_PA = build_models_pinned(LAM_PA);  MPIN_PE = build_models_pinned(LAM_PE);
FpinPA  = run_fit(MPIN_PA(2), wp, Dpa1, W_LO, OPT);
FpinPE  = run_fit(MPIN_PE(2), wp, Dpe1, W_LO, OPT);
ePA = 100*(MPIN_PA(2).f(FpinPA.theta, wp)./Dpa1 - 1);
ePE = 100*(MPIN_PE(2).f(FpinPE.theta, wp)./Dpe1 - 1);

f3 = figure('Position', [80 80 1500 680], 'Color', 'w', ...
            'NumberTitle', 'off', 'Visible', 'off');
tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile; hold on;
yline(0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off');
h3a = plot(wp - 1, ePA, '-',  'Color', COL_TRUE, 'LineWidth', 2.6, ...
           'DisplayName', 'parallel');
h3b = plot(wp - 1, ePE, '-.', 'Color', COL_B, 'LineWidth', 2.6, ...
           'DisplayName', 'perpendicular');
set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
xlim([W_LO-1 W_HI-1]);
xlabel('w - w_s', 'FontSize', FS, 'FontWeight', 'bold');
ylabel('gain error, w_s fixed  (%)', 'FontSize', FS, 'FontWeight', 'bold');
legend([h3a h3b], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
       'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');

nexttile; hold on;
yline(0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off');
h4a = plot(LO, SW(:,4), '-o', 'Color', COL_TRUE, 'LineWidth', 2.4, ...
           'MarkerSize', 7, 'MarkerFaceColor', COL_TRUE, 'DisplayName', 'parallel');
h4b = plot(LO, SW(:,6), '-s', 'Color', COL_B, 'LineWidth', 2.4, ...
           'MarkerSize', 7, 'MarkerFaceColor', COL_B, 'DisplayName', 'perpendicular');
set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
xlabel('fit lower bound  w_{lo}', 'FontSize', FS, 'FontWeight', 'bold');
ylabel('fitted w_s bias  (\mum)', 'FontSize', FS, 'FontWeight', 'bold');
legend([h4a h4b], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
       'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
exportgraphics(f3, fullfile(FIGDIR, 'gainlaw_para_ws_artifact.png'), 'Resolution', 150);
close(f3);

% demanded-parameter travel, Form B, parallel vs perpendicular on one axis pair
f2 = figure('Position', [80 80 1000 900], 'Color', 'w', ...
            'NumberTitle', 'off', 'Visible', 'off');
tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
PANEL = {pB_pa, bB_pa; pB_pe, bB_pe};
ANCH  = {1, LAM_PA; 1, LAM_PE};
YLAB  = {'form B:  p', 'form B:  b'};
for t = 1:2
    nexttile; hold on;
    lo = min(PANEL{1,t}); hi = max(PANEL{1,t});
    fill([W_LO W_HI W_HI W_LO], [lo lo hi hi], BANDC, ...
         'EdgeColor', 'none', 'HandleVisibility', 'off');
    yline(ANCH{1,t}, '--', 'Color', [0.35 0.35 0.35], 'LineWidth', 1.8, ...
          'HandleVisibility', 'off');
    hpa = plot(wg, PANEL{1,t}, '-', 'Color', COL_TRUE, 'LineWidth', 3.0, ...
               'DisplayName', 'parallel');
    hpe = plot(wg, PANEL{2,t}, '-', 'Color', COL_B, 'LineWidth', 2.4, ...
               'DisplayName', 'perpendicular');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');
    grid off; xlim([W_LO W_HI]);
    ylabel(YLAB{t}, 'FontSize', FS, 'FontWeight', 'bold');
    if t == 1
        legend([hpa hpe], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
               'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    else
        xlabel('w', 'FontSize', FS, 'FontWeight', 'bold');
    end
end
exportgraphics(f2, fullfile(FIGDIR, 'gainlaw_para_constant_B.png'), 'Resolution', 150);
close(f2);

fprintf('\nfigures -> %s\n', FIGDIR);
fprintf('  gainlaw_para_gain_and_error.png  gainlaw_para_ws_artifact.png  gainlaw_para_constant_B.png\n\n');


% =====================================================================
% helpers
% =====================================================================
function [abar, L, dL] = truth_at(w, ax)
%TRUTH_AT  exact normalised gain on one axis, plus L = -ln(1-a) and dL/dgap.
    [cpa, cpe, dv] = calc_correction_functions(w, true);
    if strcmp(ax, 'para'), c = cpa; dc = dv.dc_para_dh;
    else,                  c = cpe; dc = dv.dc_perp_dh;  end
    abar = 1/c;
    y    = 1 - abar;
    L    = -log(y);
    dL   = (-dc / c^2) / y;
end

function [w, D] = truth_grid(lo, hi, n, ax)
    w = logspace(log10(lo), log10(hi), n)';
    D = zeros(n,1);
    for i = 1:n, D(i) = truth_at(w(i), ax); end
end

function [pA, bA, pB, bB] = solve_local(w, ax)
%SOLVE_LOCAL  parameters that make each form locally exact at one height.
%   Form A:  L = (b/(p+1)) g^(p+1),  L' = b g^p     -> k = p+1 = g L'/L
%   Form B:  L = p ln(1+g/b),        L' = p/(g+b)   -> (1+u)ln(1+u)/u = L/(L' g)
    [~, L, dL] = truth_at(w, ax);
    g  = w - 1;
    k  = g * dL / L;
    pA = k - 1;   bA = k * L / g^k;
    Fu = @(u) (1 + u) .* log(1 + u) ./ u;
    u  = fzero(@(x) Fu(x) - L/(dL*g), [1e-9 1e6]);
    bB = g/u;     pB = dL * (bB + g);
end

function M = build_models_para(lam)
%BUILD_MODELS_PARA  same two forms as the perpendicular study, parallel anchor.
    M(1).name = 'form A (slide)';
    M(1).np   = 3;                          % [ln b, ln(p+1), w_s]
    M(1).f    = @(t, w) 1 - exp(-(exp(t(1))/exp(t(2))) .* ...
                                max(w - t(3), 1e-12).^exp(t(2)));
    M(1).seed = [log(1.5) log(0.5) 1.00; log(2) log(0.3) 0.95; ...
                 log(1) log(1) 1.00;    log(3) log(0.2) 1.05; ...
                 log(0.8) log(0.6) 0.90; log(4) log(0.15) 1.08];
    M(1).describe = @(t) sprintf('b=%.3f p=%+.3f ws=%.4f', ...
                                 exp(t(1)), exp(t(2)) - 1, t(3));

    M(2).name = 'form B';
    M(2).np   = 3;                          % [ln b, ln p, w_s]
    M(2).f    = @(t, w) 1 - (exp(t(1)) ./ ...
                (max(w - t(3), 0) + exp(t(1)))).^exp(t(2));
    M(2).seed = [log(lam) 0 1.00; log(0.3) 0 0.95; log(0.6) log(1.2) 1.00; ...
                 log(0.2) log(0.8) 1.05; log(1) log(0.6) 0.90; ...
                 log(0.4) log(1.5) 1.08];
    M(2).describe = @(t) sprintf('b=%.4f p=%.4f ws=%.4f', ...
                                 exp(t(1)), exp(t(2)), t(3));

    M(3).name = 'form B, b,p derived';
    M(3).np   = 1;                          % [w_s], b = 9/16 and p = 1 pinned
    M(3).f    = @(t, w) max(w - t(1), 0) ./ (max(w - t(1), 0) + lam);
    M(3).seed = [1.00; 0.95; 1.05; 0.85];
    M(3).describe = @(t) sprintf('ws=%.4f   (b=9/16, p=1 both derived)', t(1));
end

function M = build_models_pinned(lam)
%BUILD_MODELS_PINNED  the same two forms with w_s frozen at the true contact.
%   theta keeps a trailing dummy so run_fit's guard and F.ws stay valid; the
%   dummy is held far below the guard threshold and never enters the model.
    M(1).name = 'form A';
    M(1).np   = 2;
    M(1).f    = @(t, w) 1 - exp(-(exp(t(1))/exp(t(2))) .* ...
                                max(w - 1, 1e-12).^exp(t(2)));
    M(1).seed = [log(1.5) log(0.5) 0; log(2) log(0.3) 0; log(1) log(1) 0; ...
                 log(3) log(0.2) 0; log(0.8) log(0.6) 0];
    M(1).describe = @(t) sprintf('b=%.3f p=%+.3f', exp(t(1)), exp(t(2)) - 1);

    M(2).name = 'form B';
    M(2).np   = 2;
    M(2).f    = @(t, w) 1 - (exp(t(1)) ./ (max(w - 1, 0) + exp(t(1)))).^exp(t(2));
    M(2).seed = [log(lam) 0 0; log(0.3) 0 0; log(0.6) log(1.2) 0; ...
                 log(1.1) log(0.95) 0; log(0.4) log(1.5) 0];
    M(2).describe = @(t) sprintf('b=%.4f p=%.4f', exp(t(1)), exp(t(2)));
end

function F = run_fit_axis(fB, D, w, lo, opt, lam)
%RUN_FIT_AXIS  single-axis form-B minimax, packed as [ln b, ln p, w_s].
    e  = @(t) fB(exp(t(1)), exp(t(2)), t(3), w) ./ D - 1;
    gd = @(t) 1e6 * max(0, t(3) - (lo - 1e-3))^2;
    fr = @(t) sqrt(mean(e(t).^2)) + gd(t);
    fs = @(t) max(abs(e(t))) + gd(t);
    SEED = [log(lam) 0 1.00; log(1) 0 0.95; log(1.06) log(0.96) 1.00; ...
            log(1.2) log(1.1) 0.90];
    best = inf; tb = [];
    for s = 1:size(SEED,1)
        t = fminsearch(fs, fminsearch(fr, SEED(s,:), opt), opt);
        t = fminsearch(fs, t, opt);
        if fs(t) < best, best = fs(t); tb = t; end
    end
    F.theta = tb;  F.sup = max(abs(e(tb)));  F.ws = tb(3);
end

function F = run_fit(M, w, D, w_lo, opt)
    relerr = @(t) M.f(t, w) ./ D - 1;
    guard  = @(t) 1e6 * max(0, t(end) - (w_lo - 1e-3))^2;
    frms   = @(t) sqrt(mean(relerr(t).^2)) + guard(t);
    fsup   = @(t) max(abs(relerr(t)))      + guard(t);
    best = inf; theta = [];
    for s = 1:size(M.seed, 1)
        t = fminsearch(fsup, fminsearch(frms, M.seed(s,:), opt), opt);
        t = fminsearch(fsup, t, opt);
        if fsup(t) < best, best = fsup(t); theta = t; end
    end
    r = relerr(theta);
    F.theta = theta;  F.sup = max(abs(r));  F.ws = theta(end);
    F.n_alt = count_alternations(r, F.sup);
end

function C = coupling(f, t, w)
%COUPLING  Fisher information of the log-parameterised model at the optimum.
    d = 1e-6;  J = zeros(numel(w), numel(t));
    for q = 1:numel(t)
        tp = t; tm = t;  tp(q) = t(q) + d;  tm(q) = t(q) - d;
        J(:,q) = (f(tp,w) - f(tm,w)) / (2*d);
    end
    F = J'*J;  S = inv(F);
    cc = @(i,j) S(i,j) / sqrt(S(i,i)*S(j,j));
    C.cond = cond(F);  C.c12 = cc(1,2);  C.c13 = cc(1,3);  C.c23 = cc(2,3);
end

function n = count_alternations(r, sup)
    d  = diff(r);
    ix = find(d(1:end-1) .* d(2:end) < 0) + 1;
    ix = [1; ix(:); numel(r)];
    ix = ix(abs(r(ix)) >= 0.9 * sup);
    if isempty(ix), n = 0; return; end
    sg = sign(r(ix));
    n  = 1 + sum(sg(1:end-1) .* sg(2:end) < 0);
end
