% STATUS: SCRATCH -- 2026-10-26 (user): does delta-theta-hat do anything in the local version? Paired seeds 1-10, 20 deg wall, 15 um,
%   theta-hat[0] = (0, 15) deg, gains at the seeds, 10 s. Arms: (A) document version with the gate 2 sqrt P; (B) delta-theta-hat = 0
%   (angle rows of F_e = I, so the angle moves only by L e), no gate; (C) as B with the gate.
%   Criteria written before the run: delta-theta-hat is useless here if B matches A within the seed noise (paired difference of the
%   angle error at 1 / 2 / 5 / 10 s, mean +- SEM, contains 0) and B does not leave the wall without the gate.
here = fileparts(mfilename('fullpath'));  addpath(here);
seeds = 1:10;  n = numel(seeds);  T = 10;
base = struct('theta_y', 20, 'h0', 15 / 2.25, 'T', T, 'aprime', true, 'th0_deg', [0 15], 'gain0', 'seed', 'form', 'local');
o = {base, base, base};  o{1}.gap_gate = 2;  o{2}.no_dth = true;  o{3}.no_dth = true;  o{3}.gap_gate = 2;
nm = {'A: document + gate', 'B: no delta-theta, no gate', 'C: no delta-theta + gate'};
if isempty(gcp('nocreate')); parpool('Processes', min(10, feature('numcores'))); end
runs = cell(3, n);
for c = 1:3
    rc = cell(1, n);  oc = o{c};
    parfor i = 1:n; oi = oc;  oi.seeds = seeds(i);  oi.tag = sprintf('nd%d', c);  R = run_t3_xyz_fixed(oi);  rc{i} = R.runs{1}; end
    runs(c, :) = rc;
end
t = runs{1, 1}.t;  ks = arrayfun(@(s) find(t >= s - 1e-9, 1), [1 2 5 10]);
EY = zeros(3, n, 4);  EX = EY;
for c = 1:3
    for i = 1:n
        r = runs{c, i};  EY(c, i, :) = 20 - r.thy(ks);  EX(c, i, :) = -r.thx(ks);
    end
    left = sum(cellfun(@(r) any(abs(r.thy(1:r.kend) - 20) > 60), runs(c, :)));
    fprintf('%-28s | rms theta_y err 1/2/5/10 s %s | rms theta_x %s | seeds leaving the wall %d\n', nm{c}, ...
            mat2str(round(squeeze(sqrt(mean(EY(c, :, :).^2, 2)))', 1)), mat2str(round(squeeze(sqrt(mean(EX(c, :, :).^2, 2)))', 1)), left);
end
for c = 2:3
    d = squeeze(abs(EY(c, :, :)) - abs(EY(1, :, :)));  dx = squeeze(abs(EX(c, :, :)) - abs(EX(1, :, :)));
    fprintf('paired |err| %s minus A: theta_y %s +- %s | theta_x %s +- %s deg (1/2/5/10 s, mean +- SEM)\n', nm{c}(1), ...
            mat2str(round(mean(d), 2)), mat2str(round(std(d) / sqrt(n), 2)), mat2str(round(mean(dx), 2)), mat2str(round(std(dx) / sqrt(n), 2)));
end
