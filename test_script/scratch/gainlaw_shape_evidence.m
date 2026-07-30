%GAINLAW_SHAPE_EVIDENCE  Evidence for the gain-law SHAPE argument (perp axis).
%
%   Produces every number and figure behind the claim that the slide form
%
%       a_bar = 1 - exp( -(b/(p+1)) * (w_bar - w_s)^(p+1) )        [power form]
%
%   cannot carry constant parameters, while
%
%       a_bar = 1 - ( 1 + (w_bar - w_s)/lam )^(-b)                 [log form]
%
%   can. Both live inside a_bar = 1 - exp(-G); only G differs (power vs log
%   growth). The truth is the exact perpendicular correction c_perp(h_bar)
%   (Brenner 1961) already shipped in model/wall_effect.
%
%   TWO PROCEDURES (they answer different questions):
%     P1 pointwise solve  - solve the model's own equations at each height for
%                           the parameters that would make it locally exact.
%                           NO optimisation, so no "you fitted it badly" attack
%                           surface. Uses ANALYTIC derivatives, not differences.
%     P2 global minimax   - one parameter set shared by all samples; minimax
%                           (not RMS) because the claim is an UPPER BOUND on
%                           what the family can do. Equioscillation is checked
%                           so "the optimiser did not converge" is ruled out.
%
%   OUTPUTS
%     console : G1 pointwise tables + travel, G2/G3 fits, S sweep
%     figures : reference/eq17_analysis/derivation/figures/
%                 gainlaw_deficit_tail.png   deficit vs gap, log-log
%                 gainlaw_tradeoff.png       ws bias and sup vs fit lower bound
%
%   Usage:  gainlaw_shape_evidence

clear; clc;
here = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(here));
addpath(fullfile(root, 'model', 'wall_effect'));
addpath(fullfile(root, 'model', 'config'));
FIGDIR = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');

pc      = physical_constants();
R_UM    = pc.R;                 % probe radius [um] -> converts w_s bias to um
LAM_FF  = 9/8;                  % far-field reflection coefficient, perpendicular
W_LO    = 1.1;                  % acceptance range: operating envelope
W_HI    = 10;
N_SAMP  = 20000;
OPT     = optimset('MaxFunEvals', 2e5, 'MaxIter', 2e5, ...
                   'TolX', 1e-14, 'TolFun', 1e-14);

% =====================================================================
% G1  pointwise solve  (P1)
% =====================================================================
fprintf('\n');
fprintf('=====================================================================\n');
fprintf(' G1  POINTWISE SOLVE -- does every height want the same parameters?\n');
fprintf('=====================================================================\n');

W_SHOW = [1.1 1.5 2 3 5 10];
fprintf('\n[G1a] log form, lam=9/8 and w_s=1 fixed, one unknown b:\n');
fprintf('      b = -ln(1-a_bar) / ln(1 + (w-1)/lam)      (value only, no derivative)\n\n');
fprintf('%8s %10s %12s %14s %12s\n', 'w_bar', 'a_bar', '-ln(1-a)', 'ln(1+eps/lam)', 'solved b');
for w = W_SHOW
    [abar, ~, ~] = truth_at(w);
    L = -log(1 - abar);  D = log(1 + (w - 1)/LAM_FF);
    fprintf('%8.1f %10.4f %12.4f %14.4f %12.4f\n', w, abar, L, D, L/D);
end

fprintf('\n[G1b] power form, w_s=1 fixed, two unknowns (b,p):\n');
fprintf('      needs value AND slope ->  p+1 = eps*L''/L ,  b = (p+1)*L/eps^(p+1)\n\n');
fprintf('%8s %10s %14s %14s\n', 'w_bar', 'a_bar', 'solved p', 'solved b');
for w = W_SHOW
    [abar, L, dL] = truth_at(w);
    eps_ = w - 1;  k = eps_ * dL / L;
    fprintf('%8.1f %10.4f %14.4f %14.4f\n', w, abar, k - 1, k * L / eps_^k);
end

% --- travel across the whole acceptance range -------------------------
wg = logspace(log10(W_LO), log10(W_HI), N_SAMP)';
[b_log, p_pow, b_pow] = deal(zeros(N_SAMP, 1));
for i = 1:N_SAMP
    [abar, L, dL] = truth_at(wg(i));
    eps_ = wg(i) - 1;  k = eps_ * dL / L;
    b_log(i) = L / log(1 + eps_/LAM_FF);
    p_pow(i) = k - 1;   % A: p with (b,p) both free
    b_pow(i) = k * L / eps_^k;
end
% --- SYMMETRIC treatment: BOTH forms get two free shape parameters ---------
%   A: (b,p) from value+slope.  B: (b,lam) from value+slope.
%   B's solve: L = b ln(1+eps/lam), L' = b/(lam+eps).  With u := eps/lam,
%       (1+u) ln(1+u) / u = L / (L' eps),
%   whose left side is monotone in u, so (lam,b) is unique.
%   RESULT, and it must not be hidden: with two free parameters BOTH forms are
%   sloppy (A 0.519/0.351, B 0.560/0.416).  B is not better here.  The reason is
%   that B's near-wall anchor pins only the RATIO b/lam = 1, not the individual
%   values, so near the wall the pair wanders along that ray -- the same
%   collinearity that made a free extra scale unacceptable.  The discriminator
%   is NOT "who is flatter with two free parameters"; it is what happens when
%   each form's own DERIVED constant is used (see G1d).
[lamB2, bB2] = deal(zeros(N_SAMP, 1));
Fu = @(u) (1 + u) .* log(1 + u) ./ u;
for i = 1:N_SAMP
    [~, L, dL] = truth_at(wg(i));
    eps_ = wg(i) - 1;
    u = fzero(@(x) Fu(x) - L/(dL*eps_), [1e-9 1e6]);
    lamB2(i) = eps_/u;   bB2(i) = dL*(lamB2(i) + eps_);
end

span = @(v) max(v) - min(v);
fprintf('\n[G1c] parameter travel over w_bar in [%.2f, %g]:\n', W_LO, W_HI);
fprintf('   -- both forms with TWO free shape parameters (w_s pinned) --\n');
fprintf('   A  p   : %+.4f -> %+.4f   travel %.4f\n', p_pow(1), p_pow(end), span(p_pow));
fprintf('   A  b   : %+.4f -> %+.4f   travel %.4f\n', b_pow(1), b_pow(end), span(b_pow));
fprintf('   B  lam : %+.4f -> %+.4f   travel %.4f\n', lamB2(1), lamB2(end), span(lamB2));
fprintf('   B  b   : %+.4f -> %+.4f   travel %.4f\n', bB2(1), bB2(end), span(bB2));
fprintf('   => with two free parameters NEITHER form is constant.\n');
fprintf('   -- one free parameter, the other set to its DERIVED value --\n');
fprintf('   power form  p : %+.4f -> %+.4f   travel %.4f\n', p_pow(1), p_pow(end), span(p_pow));
fprintf('   power form  b : %+.4f -> %+.4f   travel %.4f\n', b_pow(1), b_pow(end), span(b_pow));
fprintf('   log   form  b : %+.4f -> %+.4f   travel %.4f\n', b_log(1), b_log(end), span(b_log));
fprintf('   ratio (power p travel) / (log b travel) = %.1fx\n', span(p_pow)/span(b_log));

% =====================================================================
% G2 + G3  global minimax fit  (P2) -- one run, two readings
% =====================================================================
fprintf('\n');
fprintf('=====================================================================\n');
fprintf(' G2/G3  GLOBAL MINIMAX FIT on w_bar in [%.2f, %g]\n', W_LO, W_HI);
fprintf('=====================================================================\n');
[wf, Df] = truth_grid(W_LO, W_HI, N_SAMP);

MODELS = build_models(LAM_FF);
fits = struct([]);
fprintf('\n%-28s %-30s %9s %11s %7s %6s\n', 'form', 'parameters', 'sup[%]', ...
        'ws bias[um]', 'equiosc', 'starts');
for m = 1:numel(MODELS)
    F = run_fit(MODELS(m), wf, Df, W_LO, OPT);
    fits(m).name = MODELS(m).name;  fits(m).F = F;
    fprintf('%-28s %-30s %9.4f %11.4f %4d/%-2d %5d/%d\n', MODELS(m).name, ...
            MODELS(m).describe(F.theta), 100*F.sup, (F.ws - 1)*R_UM, ...
            F.n_alt, MODELS(m).np + 1, F.n_hit, F.n_start);
end
fprintf('\n  ws bias column = (fitted w_s - true wall 1.0) * R, R = %.2f um\n', R_UM);
fprintf('  equiosc = alternating near-extremal error points found / required\n');
fprintf('  starts  = multi-start runs landing on the reported optimum / total\n');

% =====================================================================
% S  sweep the fit lower bound -- does the error move between pockets?
% =====================================================================
fprintf('\n');
fprintf('=====================================================================\n');
fprintf(' S  LOWER-BOUND SWEEP (upper bound fixed at %g)\n', W_HI);
fprintf('=====================================================================\n');
LO_SWEEP = [1.1 1.2 1.35 1.5 1.7 2.0];
SW = zeros(numel(LO_SWEEP), 4);      % [ws_pow sup_pow ws_log sup_log]
for j = 1:numel(LO_SWEEP)
    [wj, Dj] = truth_grid(LO_SWEEP(j), W_HI, N_SAMP);
    Fp = run_fit(MODELS(1), wj, Dj, LO_SWEEP(j), OPT);   % power form
    Fl = run_fit(MODELS(2), wj, Dj, LO_SWEEP(j), OPT);   % log form, b free
    SW(j, :) = [(Fp.ws-1)*R_UM, 100*Fp.sup, (Fl.ws-1)*R_UM, 100*Fl.sup];
end
fprintf('\n%10s | %14s %10s | %14s %10s\n', 'fit lower', ...
        'power ws[um]', 'power sup%', 'log ws[um]', 'log sup%');
for j = 1:numel(LO_SWEEP)
    fprintf('%10.2f | %14.3f %10.3f | %14.3f %10.3f\n', LO_SWEEP(j), SW(j, :));
end
fprintf('\n  power form: ws bias and sup move in OPPOSITE directions\n');
fprintf('              -> one fixed error budget, poured into either pocket\n');

% =====================================================================
% FIGURES
% =====================================================================
COL_TRUE = [0.8 0 0]; COL_POW = [0 0.2 0.9]; COL_LOG = [0.45 0.55 0.95];
FS = 18; LFS = 13; AXLW = 2.0;

% --- F1: two panels. LEFT = structure (tail class, needs the far field to be
%     visible at all). RIGHT = what acceptance actually measures, the relative
%     GAIN error inside the operating envelope. They must be read together: a
%     large deficit error far away is a SMALL gain error there, because the
%     deficit itself is small. The operating band is shaded on both.
gap  = logspace(log10(0.05), log10(100), 4000)';
w_pl = 1 + gap;
Dtru = zeros(size(w_pl));
for i = 1:numel(w_pl), Dtru(i) = truth_at(w_pl(i)); end
def_true = 1 - Dtru;
def_pow  = 1 - MODELS(1).f(fits(1).F.theta, w_pl);
def_log  = 1 - MODELS(5).f(fits(5).F.theta, w_pl);
BAND = [0.90 0.90 0.90];

% MAIN FIGURE -- the [k+1] = [k] test, drawn.
%   A filter state held with Q = 0 IS the horizontal dashed line. The solid
%   curve is the value the exact curve demands at that height. The vertical
%   distance between them is the modelling error the Q = 0 claim hides, so the
%   two panels are given the SAME y-axis height (1.2) and the travel can be
%   compared by eye. Anchors are the closed-form end values of Secs. 2-3.
%   One figure per form, IDENTICAL y-axis span (1.3) so the travel stays
%   comparable when the two are stacked in the document. Grey dashed = the two
%   closed-form anchors (the endpoints the demanded curve is pinned between);
%   red = the constant the exact curve demands at each height; shaded = the
%   travel that a Q = 0 state must pretend does not happen. No numbers on the
%   figure -- they go to the console, per the project figure rules.
%   SYMMETRIC: one figure per form, two panels = the form's two free shape
%   parameters. All four panels share the y-span (1.4) and are centred on their
%   own anchors, so every travel band is directly comparable by eye.
SPAN = 1.4; BANDC = [0.93 0.86 0.86];
plot_two_params(wg, {p_pow, b_pow}, {[0 -1], 1}, ...
    {'power form:  p', 'power form:  b'}, SPAN, W_LO, W_HI, ...
    COL_TRUE, BANDC, FS, LFS, AXLW, fullfile(FIGDIR, 'gainlaw_constant_A.png'));
plot_two_params(wg, {lamB2, bB2}, {LAM_FF, 1}, ...
    {'log form:  \lambda', 'log form:  b'}, SPAN, W_LO, W_HI, ...
    COL_TRUE, BANDC, FS, LFS, AXLW, fullfile(FIGDIR, 'gainlaw_constant_B.png'));

% SUPPORTING FIGURE -- fully linear axes, plotting the gain itself.
%   Left  : the three curves are indistinguishable by eye. That is the honest
%           starting point: a 4% gain error simply cannot be seen on a 0..1
%           axis, so "the fit looks fine" is not evidence of anything.
%   Right : the same comparison under a magnifier. Only here does the power
%           form's failure appear -- and its 4 alternating touches of the
%           +/-sup lines ARE the equioscillation proof that this is the
%           family's theoretical floor, not a lazy optimiser.
[wz, Dz] = truth_grid(W_LO, W_HI, 4000);
g_pow = MODELS(1).f(fits(1).F.theta, wz);
g_log = MODELS(5).f(fits(5).F.theta, wz);   % form B
e_pow = 100 * (g_pow ./ Dz - 1);
e_log = 100 * (g_log ./ Dz - 1);

f1 = figure('Position', [80 80 1500 680], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

%   x axis is the gap w - w_s, evaluated at the TRUE contact w_s = 1 so both
%   forms share one physical coordinate rather than their own fitted origins.
%   Convention: the wall PLANE is at w = 0 and contact at w = 1, hence w >= 1
%   and the gap w - w_s >= 0, with contact at the axis origin.
gz = wz - 1;
nexttile; hold on;
ht = plot(gz, Dz,    '-',  'Color', COL_TRUE, 'LineWidth', 2.6, 'DisplayName', 'True');
hp = plot(gz, g_pow, '--', 'Color', COL_POW,  'LineWidth', 2.2, 'DisplayName', 'form A');
hl = plot(gz, g_log, '-.', 'Color', COL_LOG,  'LineWidth', 2.2, 'DisplayName', 'form B');
set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
xlim([0 W_HI-1]); ylim([0 1]);
xlabel('w - w_s', 'FontSize', FS, 'FontWeight', 'bold');
ylabel('motion gain  a / a_o', 'FontSize', FS, 'FontWeight', 'bold');
legend([ht hp hl], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
       'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');

nexttile; hold on;
yline(0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off');
yline( 100*fits(1).F.sup, ':', 'Color', COL_POW, 'LineWidth', 1.5, 'HandleVisibility', 'off');
yline(-100*fits(1).F.sup, ':', 'Color', COL_POW, 'LineWidth', 1.5, 'HandleVisibility', 'off');
h2p = plot(gz, e_pow, '--', 'Color', COL_POW, 'LineWidth', 2.4, 'DisplayName', 'form A');
h2l = plot(gz, e_log, '-.', 'Color', COL_LOG, 'LineWidth', 2.4, 'DisplayName', 'form B');
set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
xlim([W_LO-1 W_HI-1]); ylim([-5 5]);
xlabel('w - w_s', 'FontSize', FS, 'FontWeight', 'bold');
ylabel('gain error  (%)', 'FontSize', FS, 'FontWeight', 'bold');
legend([h2p h2l], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
       'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');

exportgraphics(f1, fullfile(FIGDIR, 'gainlaw_gain_and_error.png'), 'Resolution', 150);
close(f1);

% APPENDIX FIGURE -- the structural statement (tail class). Kept separate
% because it needs log-log and the far field to be visible at all; the main
% argument for that claim is algebraic, not graphical.
fa = figure('Position', [80 80 900 680], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
hold on;
fill([W_LO W_HI W_HI W_LO], [1e-3 1e-3 1.2 1.2], BAND, ...
     'EdgeColor', 'none', 'HandleVisibility', 'off');
hr = plot(w_pl, LAM_FF ./ gap, ':', 'Color', [0.35 0.35 0.35], 'LineWidth', 1.8, ...
          'DisplayName', '(9/8)/(w-w_s)');
hat = plot(w_pl, def_true, '-',  'Color', COL_TRUE, 'LineWidth', 2.4, 'DisplayName', 'True');
hap = plot(w_pl, def_pow,  '--', 'Color', COL_POW,  'LineWidth', 2.2, 'DisplayName', 'form A');
hal = plot(w_pl, def_log,  '-.', 'Color', COL_LOG,  'LineWidth', 2.2, 'DisplayName', 'form B');
set(gca, 'XScale', 'log', 'YScale', 'log', 'FontSize', FS, 'FontWeight', 'bold', ...
         'LineWidth', AXLW, 'Box', 'on'); grid off;
xlim([1.05 101]); ylim([1e-3 1.2]);
xlabel('w', 'FontSize', FS, 'FontWeight', 'bold');
ylabel('gain deficit  1 - a', 'FontSize', FS, 'FontWeight', 'bold');
legend([hat hap hal hr], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
       'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
exportgraphics(fa, fullfile(FIGDIR, 'gainlaw_deficit_tail.png'), 'Resolution', 150);
close(fa);

% --- F2: trade-off, two stacked panels sharing the sweep axis ---------
f2 = figure('Position', [80 80 1000 760], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile; hold on;
hp1 = plot(LO_SWEEP, SW(:,1), '-o', 'Color', COL_POW, 'LineWidth', 2.2, ...
           'MarkerSize', 7, 'MarkerFaceColor', COL_POW, 'DisplayName', 'form A');
hl1 = plot(LO_SWEEP, SW(:,3), '-s', 'Color', COL_LOG, 'LineWidth', 2.2, ...
           'MarkerSize', 7, 'MarkerFaceColor', COL_LOG, 'DisplayName', 'form B');
yline(0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off');
ylabel('w_s bias  (\mum)', 'FontSize', FS, 'FontWeight', 'bold');
legend([hp1 hl1], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
       'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;

nexttile; hold on;
plot(LO_SWEEP, SW(:,2), '-o', 'Color', COL_POW, 'LineWidth', 2.2, ...
     'MarkerSize', 7, 'MarkerFaceColor', COL_POW);
plot(LO_SWEEP, SW(:,4), '-s', 'Color', COL_LOG, 'LineWidth', 2.2, ...
     'MarkerSize', 7, 'MarkerFaceColor', COL_LOG);
ylabel('sup rel. gain error  (%)', 'FontSize', FS, 'FontWeight', 'bold');
xlabel('fit lower bound  w_{lo}', 'FontSize', FS, 'FontWeight', 'bold');
set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;

exportgraphics(f2, fullfile(FIGDIR, 'gainlaw_tradeoff.png'), 'Resolution', 150);
close(f2);

fprintf('\nfigures -> %s\n', FIGDIR);
fprintf('  gainlaw_constant_A.png / _B.png (main)  gainlaw_gain_and_error.png  gainlaw_tradeoff.png  gainlaw_deficit_tail.png\n\n');


% =====================================================================
% helpers
% =====================================================================
function [abar, L, dL] = truth_at(w)
%TRUTH_AT  exact normalised gain and the derived quantities at one height.
%   a_bar = 1/c_perp,  L = -ln(1-a_bar),  dL = dL/d(gap)  [analytic]
    [~, c, dv] = calc_correction_functions(w, true);
    abar = 1/c;
    y    = 1 - abar;                      % gain deficit
    L    = -log(y);
    dabar_dw = -dv.dc_perp_dh / c^2;      % analytic, never finite differences
    dL   = dabar_dw / y;
end

function [w, D] = truth_grid(lo, hi, n)
%TRUTH_GRID  log-spaced heights and the exact a_bar there.
%   log spacing because the variation is grossly non-uniform: on [1.1,10]
%   the sub-interval [1.1,2] holds 48% of the total gain swing in 10% of a
%   linear axis.
    w = logspace(log10(lo), log10(hi), n)';
    D = zeros(n, 1);
    for i = 1:n, D(i) = truth_at(w(i)); end
end

function M = build_models(lam_ff)
%BUILD_MODELS  the candidate shapes, all of the form a = 1 - exp(-G).
%   theta is packed so every free parameter is unconstrained in R:
%   positive quantities are carried as logs, w_s directly.
    M(1).name = 'power form (slide)';
    M(1).np   = 3;                                  % [ln b, ln(p+1), w_s]
    M(1).f    = @(t, w) 1 - exp(-(exp(t(1))/exp(t(2))) .* ...
                                max(w - t(3), 1e-12).^exp(t(2)));
    M(1).seed = [log(0.4) log(0.5) 1.02; log(1) log(1) 1.00; ...
                 log(0.7) log(0.3) 0.90; log(0.9) log(0.8) 1.05; ...
                 log(0.5) log(0.6) 1.04; log(0.35) log(0.45) 0.95];
    M(1).describe = @(t) sprintf('b=%.3f p=%+.3f ws=%.4f', ...
                                 exp(t(1)), exp(t(2)) - 1, t(3));

    M(2).name = 'log form (b free)';
    M(2).np   = 2;                                  % [ln b, w_s], lam pinned
    M(2).f    = @(t, w) 1 - (1 + max(w - t(2), 1e-12)/lam_ff).^(-exp(t(1)));
    M(2).seed = [log(1) 1.00; log(0.95) 0.90; log(1.1) 1.05; log(0.9) 0.98];
    M(2).describe = @(t) sprintf('b=%.4f  lam=9/8 pinned  ws=%.4f', ...
                                 exp(t(1)), t(2));

    M(3).name = 'log form (b=1, lam free)';
    M(3).np   = 2;                                  % [ln lam, w_s], b pinned 1
    M(3).f    = @(t, w) max(w - t(2), 1e-12) ./ ...
                        (max(w - t(2), 1e-12) + exp(t(1)));
    M(3).seed = [log(9/8) 1.00; log(1) 0.95; log(1.2) 1.05; log(0.9) 0.98];
    M(3).describe = @(t) sprintf('lam=%.4f  b=1 pinned   ws=%.4f', ...
                                 exp(t(1)), t(2));

    % ADOPTED FORM.  Both shape constants come from derivation, so the only
    % estimated quantity is the contact position itself:
    %   b   = 1     Stokes 1/r decay of a point-force disturbance
    %   lam = 9/8   Lorentz first reflection = leading Brenner coefficient
    % a = 1 - [lam/(w - ws + lam)]^b  reduces to  (w-ws)/((w-ws)+lam).
    % With one free parameter the b/lam collinearity (cos ~ 0.98) cannot arise.
    M(4).name = 'adopted: b=1, lam=9/8';
    M(4).np   = 1;                                  % [w_s]
    M(4).f    = @(t, w) max(w - t(1), 1e-12) ./ (max(w - t(1), 1e-12) + lam_ff);
    M(4).seed = [1.00; 0.95; 1.05; 0.90];
    M(4).describe = @(t) sprintf('ws=%.4f   (b, lam both derived)', t(1));

    % FORM B, in the compact writing.  b is the LENGTH scale (units of R),
    % p is the exponent -- the same two symbols Form A uses, so the two forms
    % can be read side by side:
    %     a = 1 - [ b / ((w - w_s) + b) ]^p
    % At w = w_s the bracket is b/b = 1 and a = 0 for ANY p, so w_s IS the
    % contact position (nominal 1) -- the same convention as Form A and the
    % slide's "w_s - 1: unknown surface position".
    % Derived values for a rigid plane: p = 1 (Stokes 1/r) and b = 9/8 (Lorentz
    % first reflection), from 1 - a -> (b/gap)^p in the far field.  Near the
    % wall a ~ (p/b)*gap, so only the RATIO p/b is fixed there; that shared duty
    % is why the two are ~98% correlated.  p = 1 reduces it to gap/(gap+b).
    % theta = [ln b, ln p, w_s]; w_s LAST so run_fit's guard and F.ws see it.
    M(5).name = 'form B: 1-[b/X]^p';
    M(5).np   = 3;
    M(5).f    = @(t, w) 1 - (exp(t(1)) ./ ...
                (max(w - t(3), 0) + exp(t(1)))).^exp(t(2));
    M(5).seed = [log(9/8) 0 1.00; log(1) 0 0.95; log(1.06) log(0.96) 1.00; ...
                 log(1.2) log(1.1) 0.90; log(1) log(1.05) 1.05];
    M(5).describe = @(t) sprintf('b=%.4f p=%.4f ws=%.4f', ...
                                 exp(t(1)), exp(t(2)), t(3));
end

function F = run_fit(M, w, D, w_lo, opt)
%RUN_FIT  two-stage multi-start minimax fit + equioscillation check.
%   Stage 1 RMS (smooth, converges easily) seeds stage 2 minimax (the honest
%   upper bound on what the family can do). A quadratic guard keeps w_s below
%   the first sample so the model stays real over the data.
    relerr = @(t) M.f(t, w) ./ D - 1;
    guard  = @(t) 1e6 * max(0, t(end) - (w_lo - 1e-3))^2;
    frms   = @(t) sqrt(mean(relerr(t).^2)) + guard(t);
    fsup   = @(t) max(abs(relerr(t)))      + guard(t);

    best = inf; theta = [];
    scores = zeros(size(M.seed, 1), 1);
    for s = 1:size(M.seed, 1)
        t = fminsearch(fsup, fminsearch(frms, M.seed(s, :), opt), opt);
        t = fminsearch(fsup, t, opt);                 % polish
        scores(s) = fsup(t);
        if scores(s) < best, best = scores(s); theta = t; end
    end

    r = relerr(theta);
    F.theta  = theta;
    F.sup    = max(abs(r));
    F.ws     = theta(end);
    F.n_hit  = sum(scores < best * 1.01);             % starts reaching the optimum
    F.n_start= numel(scores);
    F.n_alt  = count_alternations(r, F.sup);
end

function plot_two_params(w, thetas, anchors, ylabs, span, w_lo, w_hi, ...
                         col, bandc, fs, lfs, axlw, outfile)
%PLOT_TWO_PARAMS  One form, both of its free shape parameters, stacked.
%   Grey dashed = the closed-form anchors that parameter is pinned between.
%   Red = the value the exact curve demands at that height.  Shaded = the travel
%   a Q = 0 state must pretend does not happen.  Fixed y-span so the shaded
%   heights are comparable across all panels of both figures.
    f = figure('Position', [80 80 1000 900], 'Color', 'w', ...
               'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    for t = 1:2
        th = thetas{t};  an = anchors{t};
        lo = min(th); hi = max(th); mid = mean(an);
        nexttile; hold on;
        fill([w_lo w_hi w_hi w_lo], [lo lo hi hi], bandc, ...
             'EdgeColor', 'none', 'HandleVisibility', 'off');
        for a = an
            yline(a, '--', 'Color', [0.35 0.35 0.35], 'LineWidth', 1.8, ...
                  'HandleVisibility', 'off');
        end
        h = plot(w, th, '-', 'Color', col, 'LineWidth', 3.0, ...
                 'DisplayName', 'demanded by the exact curve');
        set(gca, 'FontSize', fs, 'FontWeight', 'bold', 'LineWidth', axlw, ...
            'Box', 'on'); grid off;
        xlim([w_lo w_hi]); ylim(mid + 0.5*span*[-1 1]);
        ylabel(ylabs{t}, 'FontSize', fs, 'FontWeight', 'bold');
        if t == 1
            legend(h, 'Location', 'northoutside', 'Orientation', 'horizontal', ...
                   'FontSize', lfs, 'FontWeight', 'bold', 'Box', 'on');
        else
            xlabel('w', 'FontSize', fs, 'FontWeight', 'bold');
        end
    end
    exportgraphics(f, outfile, 'Resolution', 150);
    close(f);
end

function plot_constant_demand(w, theta, anchors, span, w_lo, w_hi, ylab, leg, ...
                              col, bandc, fs, lfs, axlw, outfile)
%PLOT_CONSTANT_DEMAND  The [k+1] = [k] question for one form, one figure.
%   The y-axis is centred on the demanded curve and given a FIXED span so that
%   two forms drawn this way can be compared by eye. The shaded strip is the
%   demanded travel; a Q = 0 state replaces that strip by a single value.
    lo = min(theta); hi = max(theta);
    mid = mean(anchors);        % centre on the anchor pair, so BOTH stay in view
    f = figure('Position', [80 80 1000 620], 'Color', 'w', ...
               'NumberTitle', 'off', 'Visible', 'off');
    hold on;
    fill([w_lo w_hi w_hi w_lo], [lo lo hi hi], bandc, ...
         'EdgeColor', 'none', 'HandleVisibility', 'off');
    for a = anchors
        yline(a, '--', 'Color', [0.35 0.35 0.35], 'LineWidth', 1.8, ...
              'HandleVisibility', 'off');
    end
    h = plot(w, theta, '-', 'Color', col, 'LineWidth', 3.0, 'DisplayName', leg);
    set(gca, 'FontSize', fs, 'FontWeight', 'bold', 'LineWidth', axlw, 'Box', 'on');
    grid off;
    xlim([w_lo w_hi]);
    ylim(mid + 0.5*span*[-1 1]);
    xlabel('w', 'FontSize', fs, 'FontWeight', 'bold');
    ylabel(ylab, 'FontSize', fs, 'FontWeight', 'bold');
    legend(h, 'Location', 'northoutside', 'Orientation', 'horizontal', ...
           'FontSize', lfs, 'FontWeight', 'bold', 'Box', 'on');
    exportgraphics(f, outfile, 'Resolution', 150);
    close(f);
end

function n = count_alternations(r, sup)
%COUNT_ALTERNATIONS  near-extremal sign alternations of the error curve.
%   A true minimax optimum equioscillates at least (np+1) times; finding them
%   is what rules out "the optimiser simply did not converge".
    d  = diff(r);
    ix = find(d(1:end-1) .* d(2:end) < 0) + 1;        % interior local extrema
    ix = [1; ix(:); numel(r)];                         % endpoints count too
    ix = ix(abs(r(ix)) >= 0.9 * sup);
    if isempty(ix), n = 0; return; end
    sg = sign(r(ix));
    n  = 1 + sum(sg(1:end-1) .* sg(2:end) < 0);
end
