% STATUS: SCRATCH -- 2026-10-28 (user): test of the a' fix -- slope a'_33 predicted with the height from the plane law
%   a'_33[k+1] = a'_33 - 2 a'_33^2 / (1 - a_33) M_w (opts.ap_predict), slope P_fix law (L) unchanged. Settled flow (void hemisphere / sphere
%   radius 15 um, start 10 deg off the top, hold 10 s, regulation tau_a 2 s to a_d 0.05, scan from 17 s). Cases: raster over half of the
%   hemisphere at 5 um/s (angle P_fix 2) and 40 um/s (4); full turn of the sphere at 40 um/s (4). Arms: L | L + prediction. Seeds 1-3.
%   Console: done / first off the sphere, a'_33 error at 17 s, mean ratio a'_33-hat / true slope during the scan, gap max, w-hat error.
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
Rr = 2.25;  ad = 0.05;  rb = 15;  tH = 10;  tS = 17;  ph0 = 10;  Ts = 1 / 1612;
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  Lturn = 2 * pi * (rb + hs * Rr);
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, ...
              'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', tH, 'tau_a', 2, 'a_d', ad, ...
              't_scan', tS, 'ap_q', true, 'q_theta_from', tS, 'wall', 'hemi', 'hemi_r', rb, 'hemi_void', true, 'ap_floor_law', true, ...
              'scan_dir', [1 0], 'hemi_start_deg', ph0, 'ap_pfix_law', true, 'tag', 'pr');
cases = {'raster 5 um/s', 'raster 40 um/s', 'full turn 40 um/s'};  arms = {'L', 'L + a'' prediction'};
[IC, IA, ID] = ndgrid(1:3, 1:2, 1:3);  res = cell(size(IC));
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
parfor q = 1:numel(IC)
    o = base;  o.seeds = ID(q);  if IA(q) == 2; o.ap_predict = true; end
    switch IC(q)
        case 1; sp = 5;  o.sP_fix_theta_deg = 2;  o.raster = [12 2 7];  o.scan_len = 1e9;  Tsc = 40;
        case 2; sp = 40; o.sP_fix_theta_deg = 4;  o.raster = [12 2 7];  o.scan_len = 1e9;  Tsc = 6;
        case 3; sp = 40; o.sP_fix_theta_deg = 4;  o.scan_plane_n = [0 1 0];  o.scan_len = Lturn;  Tsc = Lturn / sp;
    end
    o.du_d = sp * Ts / Rr;  o.T = tS + Tsc + 0.2;
    R = run_t3_xyz_fixed(o);  r = R.runs{1};
    te = tS + Tsc;  if isfield(r, 'rs_tend') && ~isnan(r.rs_tend); te = r.rs_tend; end
    ks = find(r.t >= tS & r.t <= te & (1:numel(r.t))' <= r.kend);  gap = r.h(ks) * Rr;
    bad = find(gap > 5 | isnan(r.aN(ks)), 1);  if isempty(bad); kon = numel(ks); else; kon = bad - 1; end
    kk = ks(1:max(kon, 1));  i5 = kk(1:20:end);  sv = zeros(numel(i5), 1);
    for i = 1:numel(i5); h = max(r.h(i5(i)), 1.0003); [~, c1] = calc_correction_functions(h + 1e-4); [~, c0] = calc_correction_functions(h - 1e-4); sv(i) = (1 / c1 - 1 / c0) / 2e-4; end
    kS = ks(1);  h = max(r.h(kS), 1.0003);  [~, c1] = calc_correction_functions(h + 1e-4);  [~, c0] = calc_correction_functions(h - 1e-4);
    wn = @(tx, ty) [sind(ty) .* cosd(tx), -sind(tx), cosd(ty) .* cosd(tx)];
    ew = acosd(min(1, sum(wn(r.thx(kk), r.thy(kk)) .* wn(r.th_tr(kk, 1), r.th_tr(kk, 2)), 2)));
    res{q} = [kon / numel(ks), r.apN(kS) - (1 / c1 - 1 / c0) / 2e-4, mean(r.apN(i5) ./ sv), max(gap(1:max(kon, 1))), mean(ew), rms(r.aN(kk) - r.aN_tr(kk))];
end
for ic = 1:3
    for ia = 1:2
        M = cell2mat(squeeze(res(ic, ia, :)));
        fprintf('%-18s %-20s: on the sphere %s | a''33 err at 17 s %s | a''33-hat / true (scan) %s | max gap %s um | w-hat err %s deg | rms a33 err %s\n', cases{ic}, arms{ia}, ...
                mat2str(round(M(:, 1)', 2)), mat2str(round(M(:, 2)', 2)), mat2str(round(M(:, 3)', 2)), mat2str(round(M(:, 4)', 2)), mat2str(round(M(:, 5)', 1)), mat2str(round(M(:, 6)', 3)));
    end
end

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
