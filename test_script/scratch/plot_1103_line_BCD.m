% STATUS: SCRATCH -- 2026-11-04 (user): profile of one scan line with the points A-F marked, with the step of that line plotted against the commanded distance along the line (x squeezes the steep sides, so the lower rows use distance), points A, B, C (left equator, top, right equator) marked on both in the measurement frame (F_x, F_z (commanded force, pN)) and theta-hat_y, same x axis. Current flow seed 1 (temp_1103_flow.mat), line closest to the bead centre (line 14, +x). Top: bead and glass (dark gray), probe path (blue), wall recovered with the estimated w-hat (red). A left lower half, B left equator, C top, D right equator, E right lower half / fillet, F floor. Figure in Preview. Output fig_1103_line_BCD.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');  load(fullfile(fo, 'temp_1103_flow.mat'), 'r');  Rr = 2.25;  ad = 0.07;  rb = 5;  rho = 4;  du = 40 / 1612;  MU = [char(956) 'm'];  th = char(952);  FS = 14;  cB = [0 0.3 0.9];  cR = [0.9 0.1 0.1];  cG = [0.25 0.25 0.25];
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  g = Rr * hs;  n = find(isfinite(r.aN(1:r.kend)), 1, 'last') - 1;  t = r.t(1:n);  t = t(:);  p = r.p(1:n, :);  D = r.pd(2:n + 1, :) - r.pd(1:n, :);
ty = [NaN; r.thy(1:n - 1)];  tx = [NaN; r.thx(1:n - 1)];  isv = movmean([0; diff(p(:, 2))], 21) > 0.012;  stepv = conv(double(isv), ones(61, 1), 'same') > 0;  id = cumsum(diff([0; stepv]) == 1) + 1;  id(stepv | t < r.tsc + 0.5 | t > r.rs_tend) = 0;
L = unique(id(id > 0));  my = arrayfun(@(l) mean(p(id == l, 2)), L);  [~, j] = min(abs(my));  k = find(id == L(j));  P = p(k, :);  X = P(:, 1);  T = ty(k);
Wh = [sind(T) .* cosd(tx(k)), -sind(tx(k)), cosd(T) .* cosd(tx(k))];  Q = P - g * Wh;  S = [0; cumsum(vecnorm(diff(r.pd(k, :), 1, 1), 2, 2))];
iA = find(X < -6 & T < -95, 1);  if isempty(iA); [~, iA] = min(T(1:round(numel(T) / 3))); end
[~, iB] = min(abs(T(1:round(numel(T) / 2)) + 90));  [~, iC] = max(P(:, 3));  h2 = round(numel(T) / 2):numel(T);  [~, m] = min(abs(T(h2) - 90));  iD = h2(m);  [~, m] = max(T(h2));  iE = h2(m);  iF = find(X > 11, 1);
I = [iB iC iD];  nm = 'ABC';  for q = 1:numel(I); fprintf('%s: x %+.2f z %.2f um, t %.3f s, theta_y %+.0f deg, dx %+.4f dz %+.4f um\n', nm(q), X(I(q)), P(I(q), 3), t(k(I(q))), T(I(q)), D(k(I(q)), 1), D(k(I(q)), 3)); end
Rt = 2 * sqrt(rb * rho);  at = atan2(Rt, rho - rb);  b = linspace(0, at, 400);  c = linspace(at, 0, 400);  px = [(rb + g) * sin(b), Rt - (rho - g) * sin(c), 20];  pz = [rb + (rb + g) * cos(b), rho - (rho - g) * cos(c), g];  tt = linspace(0, 2 * pi, 400);
f = figure('Color', 'w', 'Position', [40 40 1700 1250], 'Visible', 'off');  tl = tiledlayout(4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');  tl.Children;  xl = [-9 15];
a1 = nexttile(tl, [2 1]);  hold(a1, 'on');  plot(a1, rb * sin(tt), rb + rb * cos(tt), 'Color', cG, 'LineWidth', 2.5);  plot(a1, [-20 20], [0 0], 'Color', cG, 'LineWidth', 2.5);
plot(a1, X, P(:, 3), 'Color', cB, 'LineWidth', 2.2);  plot(a1, Q(:, 1), Q(:, 3), 'Color', cR, 'LineWidth', 2.2);
for q = 1:numel(I); plot(a1, X(I(q)), P(I(q), 3), 'ko', 'MarkerFaceColor', 'y', 'MarkerSize', 10);  text(a1, X(I(q)) + 0.35, P(I(q), 3) + 0.6, nm(q), 'FontSize', 18, 'FontWeight', 'bold'); end
axis(a1, 'equal');  xlim(a1, xl);  xlabel(a1, ['x  [' MU ']']);  ylim(a1, [-0.5 13.5]);  ylabel(a1, ['z  [' MU ']']);
a2 = nexttile(tl);  hold(a2, 'on');  plot(a2, S, T, 'Color', cB, 'LineWidth', 1.5);  yline(a2, 90, 'k:');  yline(a2, -90, 'k:');  yline(a2, 0, 'k:');  ylim(a2, [-120 120]);  ylabel(a2, [th char(770) '_y  [deg]']);
cfg = canonical_scenario(0.05, 1.10, 'ramp_tri', 1);  prm = calc_simulation_params(cfg);  a_nom = prm.Value.common.Ts / prm.Value.common.gamma_N;  a_o = a_nom / prm.Value.common.R;  Fm = r.fa(k, :) / a_o;  Fs = movmean(Fm, 81, 1);
Fb = du ./ (a_nom * r.aT(k));  Px = Fb .* cosd(T);  Pz = -Fb .* sind(T);  phi = atan2d(-Fs(:, 3), Fs(:, 1));
fprintf('force along the line: mean |F| (smoothed) %.2f pN, predicted %.2f pN | rms (force direction - theta_y) %.1f deg\n', mean(hypot(Fs(:, 1), Fs(:, 3))), mean(Fb), rms(wrapTo180(phi - T)));

a3 = nexttile(tl);  hold(a3, 'on');  plot(a3, S, Fm(:, 1), 'Color', [0.75 0.82 0.95], 'LineWidth', 0.5);  plot(a3, S, Fm(:, 3), 'Color', [0.75 0.9 0.75], 'LineWidth', 0.5);  h1 = plot(a3, S, Fs(:, 1), 'Color', cB, 'LineWidth', 2);  h2 = plot(a3, S, Fs(:, 3), 'Color', [0 0.6 0.2], 'LineWidth', 2);
yline(a3, 0, 'k:', 'HandleVisibility', 'off');  ylim(a3, [-12 12]);  ylabel(a3, 'force  [pN]');  xlabel(a3, ['distance along the line (commanded)  [' MU ']']);  legend(a3, [h1 h2], {'F_x', 'F_z'}, 'Location', 'northeast');
for a = [a2 a3]; for q = 1:numel(I); xline(a, S(I(q)), '-', nm(q), 'Color', [0.6 0.6 0.6], 'LineWidth', 1, 'FontSize', 14, 'FontWeight', 'bold', 'LabelVerticalAlignment', 'top', 'LabelOrientation', 'horizontal', 'HandleVisibility', 'off'); end; xlim(a, [0 S(end)]); end
for a = [a1 a2 a3]; box(a, 'on');  set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2); end
fn = fullfile(fo, 'fig_1103_line_BCD.png');  exportgraphics(f, fn, 'Resolution', 130);  close(f);
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
