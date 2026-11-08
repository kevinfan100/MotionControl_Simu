% STATUS: SCRATCH -- 2026-11-03 (user): angle convergence with a 1 s and a 5 s descent, single seed (seed 1), flat wall true (8,-8), hold 10 s at 15 um, descent at constant speed to the a_d .07 gap (descent_v = distance / T, um/s), then 1 s at the bottom (no scan, so the scan noise does not cover it). Columns 1 s | 5 s, rows share y: theta_x, theta_y (true red, estimate blue), angle error of w-hat, a_33 (true red, estimate blue). Style: seed 1, no bands, no guide lines (feedback single-seed style). Figure in Preview. Output fig_1103_descent_1v5.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');  Rr = 2.25;  Ts = 1 / 1612;  ad = 0.07;  Tx = 8;  Ty = -8;  TD = [1 5];  FS = 16;  cT = [0.85 0 0];  cE = [0 0.25 0.85];  th = char(952);  AB = char(257);  hc = char(770);  hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
base = struct('theta_x', Tx, 'theta_y', Ty, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'doc_timing', true, 'theta_out', true, 'gap_gate', 2, 'gate_one', true, 'hold_until', 10, 'tau_a', 0.06, 'a_d', ad, 'descent_v', 15 - hs * Rr, 'post_wait', [0 1], 'ap_q', true, 'ap_floor_law', true, 'sP_fix_theta_deg', 2, 'q_geo', [0.9 0.9], 'tag', 'd15', 'du_d', 40 * Ts / Rr, 'kappa_state', true, 'kappa_from_scan', true, 'kappa_pfix', 0.3, 'scan_ramp', 0.5, 'scan_len', 1e6, 'seeds', 1);
rs = cell(2, 1);
parfor j = 1:2
    o = base;  o.descent_T = TD(j);  o.descent_v = (15 - hs * Rr) / TD(j);  o.T = 10 + TD(j) + 1;  R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{j} = struct('t', r.t(:), 'thx', r.thx(:), 'thy', r.thy(:), 'aN', r.aN(:), 'aN_tr', r.aN_tr(:), 'h', r.h(:));
end
f = figure('Color', 'w', 'Position', [40 40 1700 1150], 'Visible', 'off');  tl = tiledlayout(4, 2, 'TileSpacing', 'compact', 'Padding', 'compact', 'TileIndexing', 'columnmajor');
for j = 1:2
    r = rs{j};  n = find(isfinite(r.thx), 1, 'last');  k = (1:2:n)';  t = r.t(k);  e = acosd(min(sum(local_w(r.thx(k), r.thy(k)) .* local_w(Tx * ones(numel(k), 1), Ty * ones(numel(k), 1)), 2), 1));
    for x = [10 10 + TD(j) / 2 10 + TD(j) 11 + TD(j)]; i = find(r.t >= x - 1e-6, 1);  if isempty(i); i = n; end;  fprintf('descent %g s: t %.1f s, gap %.2f um, theta %+.2f %+.2f, angle error %.2f deg\n', TD(j), r.t(i), r.h(i) * Rr, r.thx(i), r.thy(i), acosd(min(sum(local_w(r.thx(i), r.thy(i)) .* local_w(Tx, Ty)), 1))); end
    a1 = nexttile(tl);  hold(a1, 'on');  h1 = plot(a1, t, Tx * ones(size(t)), 'Color', cT, 'LineWidth', 1.6);  h2 = plot(a1, t, r.thx(k), 'Color', cE, 'LineWidth', 1);  ylim(a1, [0 16]);  ylabel(a1, [th '_x  [deg]']);  lg = legend(a1, [h1 h2], {'True', 'Estimate'}, 'Location', 'northoutside', 'Orientation', 'horizontal');  title(lg, sprintf('descent %g s, seed 1', TD(j)));
    a2 = nexttile(tl);  hold(a2, 'on');  plot(a2, t, Ty * ones(size(t)), 'Color', cT, 'LineWidth', 1.6);  plot(a2, t, r.thy(k), 'Color', cE, 'LineWidth', 1);  ylim(a2, [-16 0]);  ylabel(a2, [th '_y  [deg]']);
    a3 = nexttile(tl);  plot(a3, t, e, 'Color', cE, 'LineWidth', 1);  ylim(a3, [0 10]);  ylabel(a3, ['angle error of w' hc '  [deg]']);
    a4 = nexttile(tl);  hold(a4, 'on');  plot(a4, t, r.aN_tr(k), 'Color', cT, 'LineWidth', 1.6);  plot(a4, t, r.aN(k), 'Color', cE, 'LineWidth', 1);  ylim(a4, [0 1]);  ylabel(a4, [AB '_{33}']);  xlabel(a4, 'Time (sec)');
    for a = [a1 a2 a3 a4]; xlim(a, [0 r.t(n)]);  box(a, 'on');  set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2); end
end
fn = fullfile(fo, 'fig_1103_descent_1v5.png');  exportgraphics(f, fn, 'Resolution', 150);  close(f);  system(['open -a Preview "' fn '"']);
function w = local_w(thx, thy)
    tx = thx(:) * pi / 180;  ty = thy(:) * pi / 180;  w = [sin(ty) .* cos(tx), -sin(tx), cos(ty) .* cos(tx)];
end
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
