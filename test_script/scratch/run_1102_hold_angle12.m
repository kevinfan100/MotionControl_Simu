% STATUS: SCRATCH -- 2026-11-02 (user): angle convergence in the 10 s hold at 15 um, with the terms of the angle step. theta-hat_x[k+1] = theta-hat_x + dth_u, theta-hat_y[k+1] = theta-hat_y + dth_v / cos theta-hat_x; dth_u = a_vw-hat / g-hat, dth_v = -a_uw-hat / g-hat, g-hat = a_11-hat - a_33-hat (1029 tex); gate g-hat > 2 sqrt P_gap. Numerators = off_k1 (before the angle step). Same options as the scans, seeds 1-5, hold, 1 s descent to a_d .07, 1 s at the bottom (t 0-12 s).
%   Sanity: true angles 0 far above the floor; g true .083 at 15 um; dth_u = numerator / denominator from the logs. MATLAB window 15. Output fig_1102_hold_angle12.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');  Rr = 2.25;  Ts = 1 / 1612;  ad = 0.07;  NS = 5;  FS = 13;  cT = [0.85 0 0];  cE = [0 0.25 0.85];  th = char(952);  DL = char(948);  AB = char(257);  hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 0.06, 'a_d', ad, 'descent_v', 15 - hs * Rr, ...
              'ap_q', true, 'wall', 'fillet', 'bead_r', 5, 'fillet_rho', 4, 'ap_floor_law', true, 'scan_ramp', 0.5, 'scan_len', 1e6, 'sP_fix_theta_deg', 2, 'q_geo', [0.9 0.9], 'descent_T', 1, 'post_wait', [0 1], 'tag', 'ho', 'du_d', 40 * Ts / Rr, 'kappa_state', true, 'kappa_from_scan', true, 'kappa_pfix', 0.3, 'raster_local', [36 1 27 0.05], 'bead_start', [18; -13], 'T', 12);
cf = fullfile(fo, 'temp_hold_angle12.mat');
if isfile(cf)
    load(cf, 'rs');
else
rs = cell(NS, 1);
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
pctRunOnAll warning('off', 'all');
parfor q = 1:NS
    o = base;  o.seeds = q;  R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t(:), 'thx', r.thx(:), 'thy', r.thy(:), 'th_tr', r.th_tr, 'off', r.off_k1, 'off_tr', r.off_tr, 'g', r.dgap(:), 'sg', r.sgap(:), 'gate', r.gate(:), 'dth', r.dth, 'aT', r.aT(:), 'aN', r.aN(:), 'aT_tr', r.aT_tr(:), 'aN_tr', r.aN_tr(:), 'sP', r.sPth, 'kend', r.kend);
end
save(fullfile(fo, 'temp_hold_angle12.mat'), 'rs', '-v7.3');
end
fprintf('seed | th_x th_y at 10 s | at 11 s | at 12 s | sP at 10, 11, 12 s | g-hat at 10, 11, 12 s (true) | sd dth_u dth_v 11-12 s\n');
for q = 1:NS
    r = rs{q};  n = find(isfinite(r.g), 1, 'last');  t = r.t;  i1 = find(t >= 10, 1);  i2 = find(t >= 11, 1);  i3 = n;  w = t >= 11 & (1:numel(t))' <= n;  dthv = r.dth(:, 2) .* cosd(r.thx);
    fprintf('%d | %+.2f %+.2f | %+.2f %+.2f | %+.2f %+.2f | %.2f, %.2f, %.2f | %.3f (%.3f), %.3f (%.3f), %.3f (%.3f) | %.4f %.4f\n', q, r.thx(i1), r.thy(i1), r.thx(i2), r.thy(i2), r.thx(i3), r.thy(i3), r.sP(i1, 1), r.sP(i2, 1), r.sP(i3, 1), r.g(i1), r.aT_tr(i1) - r.aN_tr(i1), r.g(i2), r.aT_tr(i2) - r.aN_tr(i2), r.g(i3), r.aT_tr(i3) - r.aN_tr(i3), std(r.dth(w, 1)), std(dthv(w)));
end
r = rs{1};  n = find(isfinite(r.g), 1, 'last');  k = (1:2:n)';  t = r.t(k);  on = r.gate(1:n) > 0;  chk = r.off(on, 2) ./ r.g(on) * 180 / pi - r.dth(on, 1);  fprintf('check dth_u = numerator / denominator: max diff %.2e deg\n', max(abs(chk)));
if any(findall(0, 'Type', 'figure') == 15); close(15); end
f = figure(15);  set(f, 'Color', 'w', 'Position', [40 40 1800 1120], 'Visible', 'on', 'Name', 'angle in the hold and the descent, seed 1');  tl = tiledlayout(4, 2, 'TileSpacing', 'compact', 'Padding', 'compact', 'TileIndexing', 'rowmajor');
a = nexttile(tl);  hold on;  h1 = plot(t, r.th_tr(k, 1), 'Color', cT, 'LineWidth', 1.3);  h2 = plot(t, r.thx(k), 'Color', cE, 'LineWidth', 0.8);  ylabel([th '_x  [deg]']);  legend([h1 h2], {'True', 'Estimate'}, 'Location', 'northoutside', 'Orientation', 'horizontal');  local_s(a, t, FS);
a = nexttile(tl);  hold on;  plot(t, r.th_tr(k, 2), 'Color', cT, 'LineWidth', 1.3);  plot(t, r.thy(k), 'Color', cE, 'LineWidth', 0.8);  ylabel([th '_y  [deg]']);  local_s(a, t, FS);
a = nexttile(tl);  hold on;  plot(t, r.dth(k, 1), 'Color', cE, 'LineWidth', 0.5);  ylabel([DL th '_u  [deg]']);  local_s(a, t, FS);
a = nexttile(tl);  hold on;  plot(t, r.dth(k, 2) .* cosd(r.thx(k)), 'Color', cE, 'LineWidth', 0.5);  ylabel([DL th '_v  [deg]']);  local_s(a, t, FS);
a = nexttile(tl);  hold on;  plot(t, r.off_tr(k, 2), 'Color', cT, 'LineWidth', 1.3);  plot(t, r.off(k, 2), 'Color', cE, 'LineWidth', 0.5);  ylabel([AB '_{vw}  numerator of ' DL th '_u']);  local_s(a, t, FS);
a = nexttile(tl);  hold on;  plot(t, r.off_tr(k, 1), 'Color', cT, 'LineWidth', 1.3);  plot(t, r.off(k, 1), 'Color', cE, 'LineWidth', 0.5);  ylabel([AB '_{uw}  numerator of ' DL th '_v']);  local_s(a, t, FS);
a = nexttile(tl);  hold on;  h1 = plot(t, r.aT_tr(k) - r.aN_tr(k), 'Color', cT, 'LineWidth', 1.3);  h2 = plot(t, r.g(k), 'Color', cE, 'LineWidth', 0.8);  h3 = plot(t, 2 * r.sg(k), 'k--', 'LineWidth', 1.2);  ylabel(['denominator ' AB '_{11} - ' AB '_{33}']);  xlabel('Time (sec)');  legend([h1 h2 h3], {'True', 'Estimate', '2\surdP gate'}, 'Location', 'north');  ylim([-0.1 0.5]);  local_s(a, t, FS);
a = nexttile(tl);  hold on;  h1 = plot(t, r.aT_tr(k), 'Color', cT, 'LineWidth', 1.3);  h2 = plot(t, r.aT(k), 'Color', cE, 'LineWidth', 0.8);  plot(t, r.aN_tr(k), 'Color', cT, 'LineWidth', 1.3);  plot(t, r.aN(k), 'Color', cE, 'LineWidth', 0.8);  ylim([0 1.05]);  ylabel([AB '_{11} (upper), ' AB '_{33} (lower)']);  xlabel('Time (sec)');  local_s(a, t, FS);
exportgraphics(f, fullfile(fo, 'fig_1102_hold_angle12.png'), 'Resolution', 120);
function local_s(a, t, fs)
    box(a, 'on');  xlim(a, [t(1) t(end)]);  xline(a, 10, 'k:', 'LineWidth', 1, 'HandleVisibility', 'off');  xline(a, 11, 'k:', 'LineWidth', 1, 'HandleVisibility', 'off');  set(a, 'FontSize', fs, 'FontWeight', 'bold', 'LineWidth', 1.2);  set(a.Toolbar, 'Visible', 'off');
end
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
