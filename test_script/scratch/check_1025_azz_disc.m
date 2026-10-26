% STATUS: SCRATCH -- 2026-10-25 (user): two discriminations of the a-hat_zz bias / overconfidence (default sensor noise, computed angle,
%   15 um, seeds 1-10, 10 s, parfor). Arms: base | r6_if_axis (hypothesis 1: R6 time-correlation factor per axis, thermal + noise) |
%   decouple_pa (hypothesis 2: cross covariance dp - A-hat set to 0 each step). Per element: bias (SEM multiple), honesty.
here = fileparts(mfilename('fullpath'));  addpath(here);
Rr = 2.25;  seeds = 1:10;  n = numel(seeds);  nm = {'xx', 'yy', 'zz', 'xy', 'xz', 'yz'};  ij = [1 1; 2 2; 3 3; 1 2; 1 3; 2 3];
if ~exist('which_arms', 'var'); which_arms = 1:3; end     % 10-25: which_arms = [1 4] -> base vs no A-bar' (hypothesis 3)
flags = {{}, {'r6_if_axis'}, {'decouple_pa'}, {'no_aprime'}};  lab = {'base', 'H1: R6 IF per axis', 'H2: dp - A cross covariance cut', 'H3: no A-bar'' states'};
if isempty(gcp('nocreate')); parpool('Processes', min(10, feature('numcores'))); end
for c = which_arms
    E = NaN(n, 6);  Er = NaN(n, 6);  S = NaN(n, 6);  fl = flags{c};
    parfor i = 1:n
        o = struct('theta_y', 20, 'h0', 15 / Rr, 'seeds', seeds(i), 'T', 10, 'aprime', true, 'tag', 'disc');
        for q = 1:numel(fl); o.(fl{q}) = true; end
        if isfield(o, 'no_aprime'); o.aprime = false; o = rmfield(o, 'no_aprime'); end
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
