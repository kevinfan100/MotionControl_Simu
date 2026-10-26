% STATUS: SCRATCH -- 2026-10-25 (user): is the single-sample readout a-bar_m itself biased (C_n Sigma_n subtraction), or only the estimate?
%   Computed angle, 15 um, seeds 1-10, 10 s, default noise vs all axes 0.6 nm (parfor). Per element: mean over 2-10 s and seeds of
%   (a-bar_m - a-bar) with its SEM (per-seed means), next to the estimate's bias. Readout biased -> C_n is wrong (readout chain).
here = fileparts(mfilename('fullpath'));  addpath(here);
Rr = 2.25;  seeds = 1:10;  n = numel(seeds);  nm = {'xx', 'yy', 'zz', 'xy', 'xz', 'yz'};  ij = [1 1; 2 2; 3 3; 1 2; 1 3; 2 3];
arms = {[], [0.6; 0.6; 0.6] * 1e-3};  lab = {'default noise [0.62 0.57 3.31] nm', 'all axes 0.6 nm'};
if isempty(gcp('nocreate')); parpool('Processes', min(10, feature('numcores'))); end
for c = 1:2
    Bm = NaN(n, 6);  Be = NaN(n, 6);  nz = arms{c};
    parfor i = 1:n
        o = struct('theta_y', 20, 'h0', 15 / Rr, 'seeds', seeds(i), 'T', 10, 'aprime', true, 'tag', 'rb');
        if ~isempty(nz); o.noise_um = nz; end
        R = run_t3_xyz_fixed(o);  r = R.runs{1};  at = arrayfun(@(j) r.A_true(ij(j, 1), ij(j, 2)), 1:6);
        k2 = find(r.t >= 2);  k5 = find(r.t >= 5);
        Bm(i, :) = mean(r.am(k2, :) - at, 1);  Be(i, :) = mean(r.a(k5, :) - at, 1);
    end
    fprintf('\n%s\n', lab{c});
    for j = 1:6
        fprintf('  a_%s: readout a_m - a mean %+.4f (SEM %.4f, %+.1f SEM) | estimate bias %+.4f (%+.1f SEM)\n', nm{j}, mean(Bm(:, j)), ...
                std(Bm(:, j)) / sqrt(n), mean(Bm(:, j)) / (std(Bm(:, j)) / sqrt(n)), mean(Be(:, j)), mean(Be(:, j)) / (std(Be(:, j)) / sqrt(n)));
    end
end
