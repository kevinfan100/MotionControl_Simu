% STATUS: SCRATCH -- 2026-10-25 (user): paired 10-seed discrimination of the a_zz bias: readout a-bar_m,zz vs estimate a-hat_zz, default
%   sensor noise vs all axes 0.6 nm, same window 5-10 s. Paired difference (default - equal) per seed removes the shared thermal part.
%   Criterion (written before): readout paired difference ~ estimate paired difference (both ~ +0.008) -> readout formula (C_n Sigma_n);
%   readout ~ 0 but estimate ~ +0.008 -> estimator weighting. Computed angle, 15 um, 10 s, parfor.
here = fileparts(mfilename('fullpath'));  addpath(here);
Rr = 2.25;  seeds = 1:10;  n = numel(seeds);
if ~exist('extra', 'var'); extra = struct(); end           % 10-25: extra = struct('r6_diag', true) -> diagonal R6 arm
if isempty(gcp('nocreate')); parpool('Processes', min(10, feature('numcores'))); end
M = NaN(n, 2);  E = NaN(n, 2);  W1 = NaN(n, 2);  W2 = NaN(n, 2);  arms = {[], [0.6; 0.6; 0.6] * 1e-3};
for c = 1:2
    nz = arms{c};  m = NaN(n, 1);  e = NaN(n, 1);  w1 = NaN(n, 1);  w2 = NaN(n, 1);
    parfor i = 1:n
        o = struct('theta_y', 20, 'h0', 15 / Rr, 'seeds', seeds(i), 'T', 10, 'aprime', true, 'tag', 'pr');
        if ~isempty(nz); o.noise_um = nz; end
        fx = fieldnames(extra);  for q = 1:numel(fx); o.(fx{q}) = extra.(fx{q}); end
        R = run_t3_xyz_fixed(o);  r = R.runs{1};  kk = find(r.t >= 5);
        m(i) = mean(r.am(kk, 3)) - r.A_true(3, 3);  w1(i) = mean(r.am(r.t < 1, 3)) - r.A_true(3, 3);  w2(i) = mean(r.am(r.t >= 1 & r.t < 5, 3)) - r.A_true(3, 3);  e(i) = mean(r.a(kk, 3)) - r.A_true(3, 3);
    end
    M(:, c) = m;  E(:, c) = e;  W1(:, c) = w1;  W2(:, c) = w2;
end
pf = @(v) sprintf('%+.4f (SEM %.4f, %+.1f SEM)', mean(v), std(v) / sqrt(n), mean(v) / (std(v) / sqrt(n)));
fprintf('readout 0-1 s: paired diff %s | 1-5 s: paired diff %s\n', pf(W1(:, 1) - W1(:, 2)), pf(W2(:, 1) - W2(:, 2)));
fprintf('readout  a_m,zz - a_zz : default %s | equal %s | paired diff %s\n', pf(M(:, 1)), pf(M(:, 2)), pf(M(:, 1) - M(:, 2)));
fprintf('estimate a-hat_zz - a_zz: default %s | equal %s | paired diff %s\n', pf(E(:, 1)), pf(E(:, 2)), pf(E(:, 1) - E(:, 2)));
