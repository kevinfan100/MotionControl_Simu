% STATUS: SCRATCH -- 2026-11-02 (user): single seed response at a fixed point, flat wall true (8,-8), no motion and circle 3 um, seed 1. theta-hat | a_13, a_23 in the measurement frame, estimated (A-hat = V-hat Lambda-hat V-hat') against true | a_uw, a_vw logged in the local frame of V-hat (reset by every frame step) against the true element in that frame. Sanity: measured-frame elements converge to the true ones; the local estimate stays near 0 by construction. Figure in Preview. Output fig_1102_circle_single.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');  Rr = 2.25;  Ts = 1 / 1612;  ad = 0.07;  FS = 13;  Tx = 8;  Ty = -8;  AMP = [0 3];  cT = [0.85 0 0];  cE = [0 0.25 0.85];  th = char(952);  AB = char(257);  hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
base = struct('theta_x', Tx, 'theta_y', Ty, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'doc_timing', true, 'theta_out', true, 'gap_gate', 2, 'gate_one', true, 'hold_until', 10, 'tau_a', 0.06, 'a_d', ad, 'descent_v', 15 - hs * Rr, 'descent_T', 1, 'post_wait', [0 1], 'ap_q', true, 'ap_floor_law', true, 'sP_fix_theta_deg', 2, 'q_geo', [0.9 0.9], 'tag', 'cs', 'du_d', 40 * Ts / Rr, 'kappa_state', true, 'kappa_from_scan', true, 'kappa_pfix', 0.3, 'scan_ramp', 0.5, 'scan_len', 1e6, 'T', 10, 'seeds', 1);
f = figure('Color', 'w', 'Position', [40 40 1700 1150], 'Visible', 'off');  tl = tiledlayout(4, 2, 'TileSpacing', 'compact', 'Padding', 'compact', 'TileIndexing', 'columnmajor');
for a = 1:2
    o = base;  if AMP(a) > 0; o.dither = [AMP(a) 8 2];  o.dither_win = [0 10]; end;  R = run_t3_xyz_fixed(o);  r = R.runs{1};  n = find(isfinite(r.thx), 1, 'last');  t = r.t(1:n);  t = t(:);  k = (1:2:n)';
    At = r.A_true;  fprintf('A_true 13, 23: %.4f %.4f; a matrix sample at end: %s\n', At(1, 3), At(2, 3), mat2str(r.a(n, :), 4));
    A13 = r.a(1:n, 5);  A23 = r.a(1:n, 6);
    q1 = nexttile(tl);  hold(q1, 'on');  plot(q1, t(k), Tx * ones(size(k)), 'Color', cT, 'LineWidth', 1.3);  plot(q1, t(k), r.thx(k), 'Color', cE);  plot(q1, t(k), Ty * ones(size(k)), 'Color', cT, 'LineWidth', 1.3);  plot(q1, t(k), r.thy(k), 'Color', [0 0.6 0.2]);  ylim(q1, [-20 20]);  ylabel(q1, [th '_x (blue), ' th '_y (green)  [deg]']);  title(q1, sprintf('circle %g \\mum, seed 1', AMP(a)));
    q2 = nexttile(tl);  hold(q2, 'on');  plot(q2, t(k), At(1, 3) * ones(size(k)), 'Color', cT, 'LineWidth', 1.4);  plot(q2, t(k), A13(k), 'Color', cE, 'LineWidth', 0.7);  ylim(q2, [-0.03 0.03]);  ylabel(q2, [AB '_{13} measurement frame']);
    q3 = nexttile(tl);  hold(q3, 'on');  plot(q3, t(k), At(2, 3) * ones(size(k)), 'Color', cT, 'LineWidth', 1.4);  plot(q3, t(k), A23(k), 'Color', cE, 'LineWidth', 0.7);  ylim(q3, [-0.03 0.03]);  ylabel(q3, [AB '_{23} measurement frame']);
    q4 = nexttile(tl);  hold(q4, 'on');  h1 = plot(q4, t(k), r.off_tr(k, 1), 'Color', cT, 'LineWidth', 1.2);  h2 = plot(q4, t(k), r.off_k1(k, 1), 'Color', cE, 'LineWidth', 0.6);  ylim(q4, [-0.015 0.015]);  ylabel(q4, [AB '_{uw} local frame of V' char(770)]);  xlabel(q4, 'Time (sec)');  legend(q4, [h1 h2], {'True', 'Estimate'}, 'Location', 'northeast');
    for q = [q1 q2 q3 q4]; xlim(q, [0 10]);  box(q, 'on');  set(q, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);  end
    i = (t >= 8);  fprintf('circle %g: 8-10 s rms |true local a_uw| %.4f, |logged estimate| %.5f; mean A13 %.4f A23 %.4f\n', AMP(a), rms(r.off_tr(i, 1)), rms(r.off_k1(i, 1)), mean(A13(i)), mean(A23(i)));
end
fn = fullfile(fo, 'fig_1102_circle_single.png');  exportgraphics(f, fn, 'Resolution', 110);  close(f);  system(['open -a Preview "' fn '"']);
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
