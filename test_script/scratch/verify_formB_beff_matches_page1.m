%VERIFY_FORMB_BEFF_MATCHES_PAGE1  Are the red b_eff curves on pp.7-8 the
%   same function page 1 plots?
%
%   STATUS: ACTIVE -- run after any change to plot_formB_closedloop_pages.m
%   or plot_formB_form_compare.m.  Prints five checks, no assertions to tune:
%
%     C1  the two runs share one trajectory (so plotting the blind run's
%         b_hat against the other file's w_bar is legitimate), and how much
%         of that trajectory lies outside page 1's plotted gap range
%     C2  the figure's formula against the page-1 generator's, same w_bar
%     C3  substituting b_eff back into each law returns a_true
%     C4  the red curve is drawn at the COMMANDED height; how far it would
%         move if drawn at the true height instead
%     C5  the priors quoted on the pages are envelope sups, not trajectory sups
%
here = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(here));
addpath(genpath(fullfile(root,'model'))); addpath(genpath(fullfile(root,'test_script')));
cd(root);
r  = load('test_results/temp_formB_two_boundaries.mat');  r  = r.res;
rb = load('test_results/temp_formB_blind_para.mat');      rb = rb.res;

w  = r.trace{1,1,1}.w;   wb = rb.trace{3}.w;
d1 = max(abs(w - wb));
fprintf('C1  len %d / %d   max|w_p6run - w_p7run| = %.3e\n', numel(w), numel(wb), d1);
ok  = w > 1.01;
gmn = min(w(ok)) - 1;  gmx = max(w(ok)) - 1;  fo = 100*mean((w(ok)-1) > 10);
fprintf('C1  gap on trajectory [%.4f %.4f];  page 1 plots [0.1 10];  outside = %.1f %%\n', ...
        gmn, gmx, fo);

cP = nan(size(w)); cA = nan(size(w));
for i = 1:numel(w)
    if ok(i); [cA(i), cP(i)] = calc_correction_functions(w(i), true); end
end
NM = {'c_perp','c_para'};
for tr = 1:2
    if tr == 1; c = cP; else; c = cA; end
    mB = (c - 1) .* (w - 1);      mC = w .* (c - 1) ./ c;      % what the figure draws
    a  = 1 ./ c;   G = w - 1;
    pB = G .* (1 - a) ./ a;       pC = (1 + G) .* (1 - a);     % page-1 generator
    e1 = max(abs(mB(ok) - pB(ok)));  e2 = max(abs(mC(ok) - pC(ok)));
    aB = 1 - mB ./ ((w - 1) + mB);   aC = 1 - mC ./ (1 + w - 1);
    e3 = max(abs(aB(ok) - a(ok)));   e4 = max(abs(aC(ok) - a(ok)));
    iF = find(w == max(w), 1);  iT = find(ok & w == min(w(ok)), 1);
    fprintf('C2  %s  max|figure - page1 formula|   B %.2e   C %.2e\n', NM{tr}, e1, e2);
    fprintf('C3  %s  max|law(b_eff) - a_true|      B %.2e   C %.2e\n', NM{tr}, e3, e4);
    fprintf('    far  w=%6.2f  b_eff  B %.4f  C %.4f\n', w(iF), mB(iF), mC(iF));
    fprintf('    tro  w=%6.2f  b_eff  B %.4f  C %.4f   swing B %.4f  C %.4f\n', ...
            w(iT), mB(iT), mC(iT), max(mB(ok))-min(mB(ok)), max(mC(ok))-min(mC(ok)));
end

s = run_formB_ws(rb.cfg, struct('seed', 7, 'ctrl_const_override', ...
      struct('lock_b', false, 'lock_p', true, 'lock_ws', true, 'b_init', 9/8, ...
             'Pf_b_std', 0.7505, 'Pf_a_floor', 0.5697), ...
      'plant_cperp', @(hb) calc_correction_functions(hb, true)));

wd = s.h_bar_d_out(:); wt = s.h_bar_true_out(:);
m  = wd > 1.01 & wt > 1.01;   wd = wd(m); wt = wt(m);
cd_ = zeros(size(wd)); ct = zeros(size(wt));
for i = 1:numel(wd)
    cd_(i) = calc_correction_functions(wd(i), true);
    ct(i)  = calc_correction_functions(wt(i), true);
end
bD = (cd_-1).*(wd-1);  bT = (ct-1).*(wt-1);          % writing B
cD = wd.*(cd_-1)./cd_; cT = wt.*(ct-1)./ct;          % writing C
fprintf('C4 commanded vs TRUE height, c_para, seed 7\n');
fprintf('   max|w_true - w_cmd| = %.4f   rms = %.4f\n', ...
        max(abs(wt-wd)), sqrt(mean((wt-wd).^2)));
fprintf('   b_eff shift  B  max %.4f (%.1f %% of the 0.165 swing)   C  max %.4f (%.1f %% of 0.0069)\n', ...
        max(abs(bT-bD)), 100*max(abs(bT-bD))/0.1648, ...
        max(abs(cT-cD)), 100*max(abs(cT-cD))/0.0069);

% and over the FINAL HOLD only, which is where the page's claim is read
th = s.tout(m); hold_m = th > 3.8;
fprintf('   final hold only:  B %.4f (%.1f %%)   C %.4f (%.1f %%)\n', ...
        max(abs(bT(hold_m)-bD(hold_m))), 100*max(abs(bT(hold_m)-bD(hold_m)))/0.1648, ...
        max(abs(cT(hold_m)-cD(hold_m))), 100*max(abs(cT(hold_m)-cD(hold_m)))/0.0069);

% envelope sup vs achieved-trajectory sup, both writings both truths
pc = physical_constants();
he = linspace(1.90, 23.22, 20001).';
cpe = zeros(size(he)); cae = zeros(size(he));
for i = 1:numel(he); [cae(i), cpe(i)] = calc_correction_functions(he(i), true); end
for tr = 1:2
    if tr == 1; c = cpe; nm = 'c_perp'; else; c = cae; nm = 'c_para'; end
    sB = max(abs((c-1).*(he-1) - 9/8));  sC = max(abs(he.*(c-1)./c - 9/8));
    fprintf('C5 envelope [1.90 23.22] sup|b_eff - 9/8|  %s   B %.4f   C %.4f\n', nm, sB, sC);
end
