%GAINLAW_PARA_ORIGIN_SHIFT  Why the parallel axis cannot report the wall.
%
%   One picture for the whole result.  Both panels show the exact motion gain
%   (red) and the best form-B fit (blue), each drawn from ITS OWN origin w_s so
%   the origins can be compared directly against the true contact at w = 1.
%
%   perpendicular : the truth leaves contact with a finite slope (Brenner,
%                   a ~ gap), which the model can reproduce, so the fitted
%                   origin lands on the wall.
%   parallel      : the truth leaves contact with an INFINITE slope (Goldman,
%                   a ~ 1/ln(1/gap)).  No member of the family can do that, so
%                   the fit slides its origin into the wall until the part of
%                   its curve that is already steep covers the physical range.
%                   The gain then matches, but the origin is no longer the wall.
%
%   Usage:  gainlaw_para_origin_shift

clear; clc;
here = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(here));
addpath(fullfile(root, 'model', 'wall_effect'));
addpath(fullfile(root, 'model', 'config'));
FIGDIR = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');

pc   = physical_constants();  R_UM = pc.R;
W_LO = 1.1;  W_HI = 10;  N = 8000;
OPT  = optimset('MaxFunEvals', 2e5, 'MaxIter', 2e5, 'TolX', 1e-14, 'TolFun', 1e-14);
fB   = @(b, p, s, w) 1 - (b ./ (max(w - s, 0) + b)).^p;

wf = logspace(log10(W_LO), log10(W_HI), N)';
Dpa = zeros(N,1);  Dpe = zeros(N,1);
for i = 1:N
    [cpa, cpe] = calc_correction_functions(wf(i));
    Dpa(i) = 1/cpa;  Dpe(i) = 1/cpe;
end
tPA = fit3(fB, Dpa, wf, W_LO, OPT, [log(0.5) 0 1; log(0.3) log(0.8) 0.9; log(0.2) log(0.7) 0.95]);
tPE = fit3(fB, Dpe, wf, W_LO, OPT, [log(9/8) 0 1; log(1) log(0.95) 0.98; log(1.2) log(1.1) 0.95]);

fprintf('\n%-16s %10s %12s %14s\n', 'axis', 'ws_hat', 'offset[um]', '%% of radius');
fprintf('%-16s %10.4f %12.4f %13.1f%%\n', 'perpendicular', tPE(3), ...
        (tPE(3)-1)*R_UM, 100*(1-tPE(3)));
fprintf('%-16s %10.4f %12.4f %13.1f%%\n', 'parallel', tPA(3), ...
        (tPA(3)-1)*R_UM, 100*(1-tPA(3)));
fprintf('\nprobe radius R = %.3f um\n', R_UM);

COL_TRUE = [0.8 0 0];  COL_FIT = [0 0.2 0.9];  COL_WALL = [0.35 0.35 0.35];
FS = 18; LFS = 13; AXLW = 2.0;
X_LO = 0.70;  X_HI = 4.0;
wt = linspace(1, X_HI, 3000)';                    % truth exists only for w >= 1
Dt = zeros(size(wt));

fig = figure('Position', [80 80 1500 660], 'Color', 'w', ...
             'NumberTitle', 'off', 'Visible', 'off');
tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

for k = 1:2
    if k == 1, t = tPE; ttl = 'perpendicular'; else, t = tPA; ttl = 'parallel'; end
    for i = 1:numel(wt)
        [cpa, cpe] = calc_correction_functions(max(wt(i), 1 + 1e-12));
        if k == 1, Dt(i) = 1/cpe; else, Dt(i) = 1/cpa; end
    end
    wm = linspace(t(3), X_HI, 3000)';             % model drawn from its OWN origin

    nexttile; hold on;
    % the wall region, so "inside the wall" is visible as such
    fill([X_LO 1 1 X_LO], [0 0 1 1], [0.90 0.90 0.90], ...
         'EdgeColor', 'none', 'HandleVisibility', 'off');
    hw = xline(1, '-', 'Color', COL_WALL, 'LineWidth', 3.0, ...
               'DisplayName', 'true contact');
    hs = xline(t(3), ':', 'Color', COL_FIT, 'LineWidth', 3.0, ...
               'DisplayName', 'fitted origin  w_s');
    ht = plot(wt, Dt, '-',  'Color', COL_TRUE, 'LineWidth', 3.0, ...
              'DisplayName', 'True');
    hf = plot(wm, fB(exp(t(1)), exp(t(2)), t(3), wm), '--', 'Color', COL_FIT, ...
              'LineWidth', 2.4, 'DisplayName', 'fitted model');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');
    grid off;  xlim([X_LO X_HI]);  ylim([0 1]);
    xlabel(sprintf('w        [ %s ]', ttl), 'FontSize', FS, 'FontWeight', 'bold');
    ylabel('motion gain  a / a_o', 'FontSize', FS, 'FontWeight', 'bold');
    legend([ht hf hw hs], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
           'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
end

exportgraphics(fig, fullfile(FIGDIR, 'gainlaw_para_origin_shift.png'), 'Resolution', 150);
close(fig);
fprintf('figure -> %s\n\n', fullfile(FIGDIR, 'gainlaw_para_origin_shift.png'));


function tb = fit3(fB, D, w, lo, opt, seeds)
    e  = @(t) fB(exp(t(1)), exp(t(2)), t(3), w) ./ D - 1;
    gd = @(t) 1e6 * max(0, t(3) - (lo - 1e-3))^2;
    fr = @(t) sqrt(mean(e(t).^2)) + gd(t);
    fs = @(t) max(abs(e(t))) + gd(t);
    best = inf; tb = [];
    for s = 1:size(seeds, 1)
        t = fminsearch(fs, fminsearch(fr, seeds(s,:), opt), opt);
        t = fminsearch(fs, t, opt);
        if fs(t) < best, best = fs(t); tb = t; end
    end
end
