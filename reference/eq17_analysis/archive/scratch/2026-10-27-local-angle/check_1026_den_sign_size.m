% STATUS: SCRATCH -- 2026-10-26 (user): is the no-gate divergence of the local version caused by the sign or by the size of the
%   denominator a_par-hat - a_perp-hat? Same setting as the no-gate run of plot_1026_local_info.m (20 deg wall, 15 um, theta-hat[0] =
%   (0, 15) deg, gains at the seeds, prior 10 deg, seeds 1-10, 10 s), no gate. Only the denominator of the angle step changes
%   (delta-theta-hat and the F_e angle rows, run_t3_xyz_fixed opts.den_mode):
%   (0) as derived; (1) |gap| -- sign fixed, size untouched; (2) sign(gap) max(|gap|, g/2) -- size fixed, sign untouched;
%   (3) max(gap, g/2) -- both fixed. g = true a_par - a_perp at 15 um (0.083): an oracle floor, diagnostic only.
%   Criteria written before the run: seeds leaving the wall (|theta-hat - truth| > 60 deg). Sign is the cause if (1) removes the
%   divergence and (2) does not; size is the cause if the reverse. Also printed for arm (0): sign and size of the gap at the first
%   step whose angle step exceeds 5 deg.
here = fileparts(mfilename('fullpath'));  addpath(here);
seeds = 1:10;  n = numel(seeds);  g = 0.9160 - 0.8327;
base = struct('theta_y', 20, 'h0', 15 / 2.25, 'T', 10, 'aprime', true, 'th0_deg', [0 15], 'gain0', 'seed', 'form', 'local');
o = {base, base, base, base};
o{2}.den_mode = 'abs';  o{3}.den_mode = 'floor';  o{3}.den_floor = g / 2;  o{4}.den_mode = 'both';  o{4}.den_floor = g / 2;
nm = {'(0) as derived', '(1) |gap|: sign fixed', '(2) size floored at g/2', '(3) both fixed'};
if isempty(gcp('nocreate')); parpool('Processes', min(10, feature('numcores'))); end
runs = cell(4, n);
for c = 1:4
    rc = cell(1, n);  oc = o{c};
    parfor i = 1:n; oi = oc;  oi.seeds = seeds(i);  oi.tag = sprintf('ds%d', c);  R = run_t3_xyz_fixed(oi);  rc{i} = R.runs{1}; end
    runs(c, :) = rc;
end
for c = 1:4
    bad = cellfun(@(r) any(abs(r.thy(1:r.kend) - 20) > 60 | abs(r.thx(1:r.kend)) > 60), runs(c, :));
    e10 = cellfun(@(r) 20 - r.thy(r.kend), runs(c, ~bad));  x10 = cellfun(@(r) -r.thx(r.kend), runs(c, ~bad));
    fprintf('%-26s | seeds leaving the wall %d / %d (%s) | others: rms err at 10 s theta_y %.1f theta_x %.1f deg\n', nm{c}, sum(bad), n, ...
            mat2str(seeds(bad)), sqrt(mean(e10.^2)), sqrt(mean(x10.^2)));
end
fprintf('arm (0): first step with |angle step| > 5 deg -> gap there (true g = %.3f)\n', g);
for i = 1:n
    r = runs{1, i};  k = 1:r.kend;  st = abs(diff([r.thy(k) r.thx(k)]));  kb = find(max(st, [], 2) > 5, 1);
    if isempty(kb); fprintf('  seed %2d: no step > 5 deg\n', seeds(i)); continue; end
    gp = r.dgap(kb + 1);
    fprintf('  seed %2d: t = %.4f s, gap-hat %+.4f (%s, %.2f x g)\n', seeds(i), r.t(kb + 1), gp, ...
            ternary(gp < 0, 'negative', ternary(gp < g / 2, 'positive, below g/2', 'positive, above g/2')), gp / g);
end
function s = ternary(c, a, b); if c; s = a; else; s = b; end; end
