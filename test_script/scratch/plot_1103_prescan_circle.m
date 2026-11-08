% STATUS: SCRATCH -- 2026-11-03 (user): before the scan, seed 1, rho 4 bead (floor under the start, true angle 0), current settings. Left: hold 10 s at 15 um, descent 1 s. Right: hold 5 s with a horizontal circle 3 um 8 Hz (2 s ramp) in the tangent plane of V-hat, descent 1 s. Rows: theta_x, theta_y (true red, estimate blue) | numerators a_13, a_23 (estimator values, slots 12, 13 at k+1 before the angle step) | denominator a_11 - a_33 (true red, estimate blue). Style: single seed, no bands, no guide lines. Figure in Preview. Output fig_1103_prescan_circle.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');  Rr = 2.25;  Ts = 1 / 1612;  ad = 0.07;  FS = 15;  cT = [0.85 0 0];  cE = [0 0.25 0.85];  th = char(952);  AB = char(257);  HT = [10 5];  hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'tau_a', 0.06, 'a_d', ad, 'descent_v', 15 - hs * Rr, 'descent_T', 1, 'post_wait', [0 0], 'ap_q', true, 'wall', 'fillet', 'bead_r', 5, 'fillet_rho', 4, 'ap_floor_law', true, 'scan_ramp', 0.5, 'scan_len', 1e6, 'sP_fix_theta_deg', 2, 'q_geo', [0.9 0.9], 'tag', 'pc', 'du_d', 40 * Ts / Rr, 'kappa_state', true, 'kappa_from_scan', true, 'kappa_pfix', 0.3, 'raster_local', [36 1 27 0.05], 'bead_start', [18; -13], 'seeds', 1);
rs = cell(2, 1);
parfor j = 1:2
    o = base;  o.hold_until = HT(j);  o.T = HT(j) + 1;  if j == 2; o.dither = [3 8 2];  o.dither_win = [0 HT(j)]; end;  R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{j} = struct('t', r.t(:), 'thx', r.thx(:), 'thy', r.thy(:), 'th_tr', r.th_tr, 'off', r.off_k1, 'g', r.dgap(:), 'gt', r.aT_tr(:) - r.aN_tr(:));
end
f = figure('Color', 'w', 'Position', [40 40 1800 1200], 'Visible', 'off');  tl = tiledlayout(5, 2, 'TileSpacing', 'compact', 'Padding', 'compact', 'TileIndexing', 'columnmajor');  tt = {'hold 10 s, descent 1 s, seed 1', 'circle 3 \mum during a 5 s hold, descent 1 s, seed 1'};
for j = 1:2
    r = rs{j};  n = find(isfinite(r.thx), 1, 'last');  k = (1:2:n)';  t = r.t(k);
    for x = [HT(j) HT(j) + 1]; i = min(find(r.t >= x - 1e-6, 1), n);  fprintf('%s: t %.1f s theta-hat (%+.2f, %+.2f) deg\n', tt{j}, r.t(i), r.thx(i), r.thy(i)); end
    a1 = nexttile(tl);  hold(a1, 'on');  h1 = plot(a1, t, r.th_tr(k, 1), 'Color', cT, 'LineWidth', 1.5);  h2 = plot(a1, t, r.thx(k), 'Color', cE, 'LineWidth', 0.9);  ylim(a1, [-12 12]);  ylabel(a1, [th '_x  [deg]']);  lg = legend(a1, [h1 h2], {'True', 'Estimate'}, 'Location', 'northoutside', 'Orientation', 'horizontal');  title(lg, tt{j});
    a2 = nexttile(tl);  hold(a2, 'on');  plot(a2, t, r.th_tr(k, 2), 'Color', cT, 'LineWidth', 1.5);  plot(a2, t, r.thy(k), 'Color', cE, 'LineWidth', 0.9);  ylim(a2, [-12 12]);  ylabel(a2, [th '_y  [deg]']);
    a3 = nexttile(tl);  plot(a3, t, r.off(k, 1), 'Color', cE, 'LineWidth', 0.7);  ylim(a3, [-0.004 0.004]);  ylabel(a3, ['a' char(770) '_{13}']);
    a4 = nexttile(tl);  plot(a4, t, r.off(k, 2), 'Color', cE, 'LineWidth', 0.7);  ylim(a4, [-0.004 0.004]);  ylabel(a4, ['a' char(770) '_{23}']);
    a5 = nexttile(tl);  hold(a5, 'on');  plot(a5, t, r.gt(k), 'Color', cT, 'LineWidth', 1.5);  plot(a5, t, r.g(k), 'Color', cE, 'LineWidth', 0.9);  ylim(a5, [0 0.5]);  ylabel(a5, ['a' char(770) '_{11} - a' char(770) '_{33}']);  xlabel(a5, 'Time (sec)');
    for a = [a1 a2 a3 a4 a5]; xlim(a, [0 HT(j) + 1]);  box(a, 'on');  set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2); end
end
fn = fullfile(fo, 'fig_1103_prescan_circle.png');  exportgraphics(f, fn, 'Resolution', 130);  close(f);  system(['open -a Preview "' fn '"']);
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
