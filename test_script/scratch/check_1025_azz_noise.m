% STATUS: SCRATCH -- 2026-10-25 (user): discriminate the a-hat_zz bias (+0.008, honesty 1.5). [hypothesis] the larger z sensor noise
%   (3.31 nm vs ~0.6 nm on x, y) is handled wrongly in the readout (C_n Sigma_n subtraction) or in R; rival: a plant effect along z
%   unrelated to the sensor. Arms on the same seeds 1-10: default noise vs all axes 0.6 nm. Computed angle, 15 um, 10 s, parfor.
%   If the bias and the honesty excess vanish with equal noise -> sensor-noise handling; if they stay -> the rival.
here = fileparts(mfilename('fullpath'));  addpath(here);
Rr = 2.25;  seeds = 1:10;  n = numel(seeds);  nm = {'xx', 'yy', 'zz', 'xy', 'xz', 'yz'};  ij = [1 1; 2 2; 3 3; 1 2; 1 3; 2 3];
arms = {[], [0.6; 0.6; 0.6] * 1e-3};  lab = {'default noise [0.62 0.57 3.31] nm', 'all axes 0.6 nm'};
if isempty(gcp('nocreate')); parpool('Processes', min(10, feature('numcores'))); end
for c = 1:2
    E = NaN(n, 6);  Er = NaN(n, 6);  S = NaN(n, 6);  nz = arms{c};
    parfor i = 1:n
        o = struct('theta_y', 20, 'h0', 15 / Rr, 'seeds', seeds(i), 'T', 10, 'aprime', true, 'tag', 'azz');
        if ~isempty(nz); o.noise_um = nz; end
        R = run_t3_xyz_fixed(o);  r = R.runs{1};  kk = find(r.t >= 5);
        at = arrayfun(@(j) r.A_true(ij(j, 1), ij(j, 2)), 1:6);  e = r.a(kk, :) - at;
        E(i, :) = mean(e, 1);  Er(i, :) = sqrt(mean(e.^2, 1));  S(i, :) = mean(r.sPa(kk, :), 1);
    end
    bias = mean(E, 1);  sem = std(E, 0, 1) / sqrt(n);  hon = sqrt(mean(Er.^2, 1)) ./ mean(S, 1);
    fprintf('\n%s\n', lab{c});
    for j = 1:6
        fprintf('  a_%s: bias %+.4f (%+.1f SEM) | rms err %.4f | mean sqrtP %.4f | honesty %.2f\n', nm{j}, bias(j), bias(j) / sem(j), ...
                sqrt(mean(Er(:, j).^2)), mean(S(:, j)), hon(j));
    end
end
