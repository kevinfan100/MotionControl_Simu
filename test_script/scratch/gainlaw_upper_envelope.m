%GAINLAW_UPPER_ENVELOPE  Form-B members that stay ABOVE the exact gain curve.
%
%   w_s is FIXED at the true contact (1); only (b, p) are chosen.  Wanted:
%   curves that lie above the truth everywhere in the operating range and that
%   fall away from the far-field value 1 more slowly than the truth does as the
%   wall is approached.  Both requirements come from the same two limits:
%
%       near wall   a ~ (p/b)(w-w_s)   vs truth  a ~ (w-w_s)     -> p/b > 1
%       far field   1-a ~ b^p (w-w_s)^(-p)  vs  (9/8)/(w-w_s)    -> p > 1,
%                                                        or p = 1 with b < 9/8
%
%   The second is what "falls off more slowly" means: the gain deficit must be
%   smaller than the truth's at every gap, so the gain sits higher.
%
%   Note the two derived anchors bracket the truth:  (b,p) = (1,1) is the
%   near-wall anchor and lies above;  (9/8,1) is the far-field anchor and lies
%   below.  The 12.5% tension of the two-anchor argument is exactly this gap.
%
%   Usage:  gainlaw_upper_envelope

clear; clc;
here = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(here));
addpath(fullfile(root, 'model', 'wall_effect'));
FIGDIR = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');

W_LO = 1.1;  W_HI = 10;  N = 4000;
w = logspace(log10(W_LO), log10(W_HI), N)';  g = w - 1;
D = zeros(N,1);
for i = 1:N, [~, c] = calc_correction_functions(w(i)); D(i) = 1/c; end
mdl = @(b, p, gg) 1 - (1 + gg/b).^(-p);

% ---- sweep: which (b,p) stay above the truth everywhere? -----------------
PV = 1.00:0.02:2.40;
BV = 0.30:0.01:1.60;
above = false(numel(PV), numel(BV));
excess = nan(numel(PV), numel(BV));
for ip = 1:numel(PV)
    for ib = 1:numel(BV)
        if PV(ip)/BV(ib) <= 1, continue; end          % near-wall requirement
        r = mdl(BV(ib), PV(ip), g) ./ D - 1;
        if all(r > 0), above(ip,ib) = true; excess(ip,ib) = max(r); end
    end
end
fprintf('\n%d of %d (b,p) pairs on the grid stay strictly above the truth\n', ...
        sum(above(:)), numel(above));
fprintf('tightest excess on the grid: %.3f%%\n', 100*min(excess(:)));

% ---- tightest member: largest b at p = 1 that still clears the truth -----
%   p must be >= 1 (a slower tail would eventually cross below), so at p = 1
%   the bound tightens as b grows until it touches.  The touch point sits at
%   the lower edge of the range, which makes this b range-dependent; extending
%   the range to contact drives it back to the near-wall anchor b = 1.
okab = @(b,p) all(mdl(b,p,g)./D - 1 > 0);
lo = 1.0; hi = 9/8;
for k = 1:80
    mid = (lo+hi)/2;
    if okab(mid,1), lo = mid; else, hi = mid; end
end
b_crit = lo;
fprintf('\ntightest at p=1: b = %.4f  (near-wall anchor 1, far-field anchor 9/8)\n', b_crit);

% ---- representative members, ordered by how far above they sit -----------
%   The last two are deliberately exaggerated: b well below 1 pushes the whole
%   curve up, and the excess is unbounded (as p grows the model tends to 1 at
%   every gap), so "more extreme" is a choice, not a limit of the family.
CAND = [b_crit 1.000;     % tightest on this range, touches at w = 1.1
        1.000  1.000;     % near-wall anchor; clears the truth at every gap
        0.900  1.000;     % same tail slope, smaller coefficient
        0.800  1.100;
        0.600  1.400;
        0.400  1.800];
fprintf('\n%8s %8s %8s %12s %12s %14s\n', 'b', 'p', 'p/b', 'tail exp', ...
        'max exc[%]', 'min exc[%]');
for k = 1:size(CAND,1)
    r = mdl(CAND(k,1), CAND(k,2), g) ./ D - 1;
    fprintf('%8.3f %8.3f %8.3f %12.3f %12.3f %14.4f\n', CAND(k,1), CAND(k,2), ...
            CAND(k,2)/CAND(k,1), CAND(k,2), 100*max(r), 100*min(r));
end
fprintf('\ntail exp > 1 (truth is 1) = the deficit dies faster, so the gain\n');
fprintf('falls away from 1 more slowly than the truth as the wall is approached.\n');

% the far-field anchor, for contrast: it sits BELOW
rff = mdl(9/8, 1, g) ./ D - 1;
fprintf('\nfar-field anchor (b,p) = (9/8, 1):  excess in [%.3f, %.3f] %% -> below\n', ...
        100*min(rff), 100*max(rff));

% ---- figure, document style ---------------------------------------------
%   The gain panel, with the truth and the two most exaggerated members only:
%   both have b well below 1, so the whole curve is lifted, and p above 1 keeps
%   the tail from crossing back under.
SHOW = [0.600 1.400; 0.400 1.800];
COL_TRUE = [0.8 0 0];
CL = [0.10 0.28 0.85; 0.55 0.68 0.96];
FS = 18; LFS = 13; AXLW = 2.0;
fig = figure('Position', [80 80 1050 720], 'Color', 'w', ...
             'NumberTitle', 'off', 'Visible', 'off');
hold on;
ht = plot(g, D, '-', 'Color', COL_TRUE, 'LineWidth', 3.0, 'DisplayName', 'True');
hs = gobjects(1, size(SHOW,1));
for k = 1:size(SHOW,1)
    hs(k) = plot(g, mdl(SHOW(k,1), SHOW(k,2), g), '--', 'Color', CL(k,:), ...
        'LineWidth', 2.6, ...
        'DisplayName', sprintf('b=%.2f, p=%.2f', SHOW(k,1), SHOW(k,2)));
end
set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');
grid off;  xlim([0 W_HI-1]);  ylim([0 1]);
xlabel('w - w_s', 'FontSize', FS, 'FontWeight', 'bold');
ylabel('motion gain  a / a_o', 'FontSize', FS, 'FontWeight', 'bold');
legend([ht hs], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
       'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');

exportgraphics(fig, fullfile(FIGDIR, 'gainlaw_upper_envelope.png'), 'Resolution', 150);
close(fig);
fprintf('\nfigure -> %s\n\n', fullfile(FIGDIR, 'gainlaw_upper_envelope.png'));
