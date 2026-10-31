% STATUS: SCRATCH -- 2026-10-27 (user): is the local version (1026 document; run_t3_xyz_fixed form 'local', one-sided gate 2 sqrt P) worse
%   than the 1025 estimated angle (form 'estimated')? Paired seeds 1-10, wall theta_y = 20 deg, theta-hat[0] = 0, gains at the seeds for both,
%   prior 10 deg, slope q = 0, 10 s. Scenarios: fixed 15 um, fixed 5 um, circle 3 um 8 Hz at 15 um.
%   Criteria: rms angle error at 1/2/5/10 s, time both angles stay < 5 deg (median), seeds leaving the wall; paired |error| difference.
here = fileparts(mfilename('fullpath'));  addpath(here);
Rr = 2.25;  T = 10;  seeds = 1:10;  n = numel(seeds);
sc = {struct('h0', 15 / Rr, 'nm', 'fixed 15 um'), struct('h0', 5 / Rr, 'nm', 'fixed 5 um'), struct('h0', 15 / Rr, 'nm', 'circle 15 um', 'dither', [3 8 1])};
fm = {'estimated', 'local'};  ns = numel(sc);  nf = numel(fm);
if isempty(gcp('nocreate')); parpool('Processes', min(10, feature('numcores'))); end
jobs = cell(1, ns * nf * n);
parfor q = 1:ns * nf * n
    is = 1 + floor((q - 1) / (nf * n));  rr = q - (is - 1) * nf * n;  f = 1 + floor((rr - 1) / n);  i = rr - (f - 1) * n;
    s = sc{is};
    o = struct('theta_x', 0, 'theta_y', 20, 'h0', s.h0, 'T', T, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'form', fm{f}, ...
               'seeds', seeds(i), 'tag', 'cmp');
    if strcmp(fm{f}, 'local'); o.gap_gate = 2;  o.gate_one = true; end
    if isfield(s, 'dither'); o.dither = s.dither; end
    R = run_t3_xyz_fixed(o);  jobs{q} = R.runs{1};
end
for is = 1:ns
    E = cell(1, nf);
    for f = 1:nf
        rs = jobs((is - 1) * nf * n + (f - 1) * n + (1:n));  t = rs{1}.t;  ks = arrayfun(@(s) find(t >= s - 1e-9, 1), [1 2 5 10]);
        EY = NaN(4, n);  EX = EY;  tin = NaN(1, n);  bad = false(1, n);
        for i = 1:n
            r = rs{i};  k = 1:r.kend;  ey = 20 - r.thy(k);  ex = -r.thx(k);  bad(i) = any(abs(ey) > 60 | abs(ex) > 60) || r.kend < numel(r.t);
            kk = ks(ks <= r.kend);  EY(1:numel(kk), i) = ey(kk);  EX(1:numel(kk), i) = ex(kk);
            last = find(max(abs(ey), abs(ex)) > 5, 1, 'last');  if isempty(last); tin(i) = 0; elseif last < numel(k); tin(i) = r.t(last + 1); end
        end
        E{f} = {EY, EX};
        fprintf('%-13s %-10s | bad %d | rms theta_y 1/2/5/10 s %s | theta_x %s | both < 5 deg median %.2f s (%d seeds never)\n', sc{is}.nm, fm{f}, sum(bad), ...
                mat2str(round(sqrt(mean(EY(:, ~bad).^2, 2, 'omitnan'))', 1)), mat2str(round(sqrt(mean(EX(:, ~bad).^2, 2, 'omitnan'))', 1)), ...
                median(tin, 'omitnan'), sum(isnan(tin)));
    end
    dy = abs(E{2}{1}) - abs(E{1}{1});  dx = abs(E{2}{2}) - abs(E{1}{2});
    fprintf('%-13s paired |err| local - estimated: theta_y %s +- %s | theta_x %s +- %s (1/2/5/10 s, mean +- SEM)\n', sc{is}.nm, ...
            mat2str(round(mean(dy, 2, 'omitnan')', 2)), mat2str(round(std(dy, 0, 2, 'omitnan')' / sqrt(n), 2)), ...
            mat2str(round(mean(dx, 2, 'omitnan')', 2)), mat2str(round(std(dx, 0, 2, 'omitnan')' / sqrt(n), 2)));
end
