% STATUS: SCRATCH -- 2026-11-03 (user): as fig_1103_flow_force_local.png in the MEASUREMENT frame (x, y, z): commanded force f_d (logged a_o f_d / a_o, pN) and commanded step p_d[k+1] - p_d[k] (um), current flow seed 1 (temp_1103_flow.mat). Console: in the local frame of V-hat[k] the force split into the reference-step part (a_nom A-hat)^-1 Delta p_d and the rest (tracking error, (1 - lambda_c)(p_d[k] - p-hat)), per axis, to see how much the regulation needs. Figure in Preview. Output fig_1103_flow_force_meas.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');  load(fullfile(fo, 'temp_1103_flow.mat'), 'r');  cfg = canonical_scenario(0.05, 1.10, 'ramp_tri', 1);  params = calc_simulation_params(cfg);  P = params.Value;  a_nom = P.common.Ts / P.common.gamma_N;  a_o = a_nom / P.common.R;  FS = 14;  cE = [0 0.25 0.85];
n = find(isfinite(r.aN(1:r.kend)), 1, 'last');  n = min(n, size(r.fa, 1)) - 1;  t = r.t(1:n);  t = t(:);  f = r.fa(1:n, :) / a_o;  D = r.pd(2:n + 1, :) - r.pd(1:n, :);  Fr = NaN(n, 3);  Fl = NaN(n, 3);
for i = 2:n; c0 = r.thx(i - 1) * pi / 180;  s0 = r.thy(i - 1) * pi / 180;  V = [cos(s0) 0 sin(s0); 0 1 0; -sin(s0) 0 cos(s0)] * [1 0 0; 0 cos(c0) -sin(c0); 0 sin(c0) cos(c0)];  Fl(i, :) = (V' * f(i, :)')';  Fr(i, :) = ((V' * D(i, :)') ./ (a_nom * [r.aT(i - 1); r.aT(i - 1); r.aN(i - 1)]))'; end
m = t >= r.tsc + 0.5 & t <= r.rs_tend;  Fe = Fl - Fr;  ax = {'u', 'v', 'w'};
for q = 1:3; fprintf('scan, local %s: total rms %.2f pN | reference-step part rms %.2f pN (mean %+.2f) | tracking-error part rms %.2f pN\n', ax{q}, rms(Fl(m, q)), rms(Fr(m, q)), mean(Fr(m, q)), rms(Fe(m, q))); end
mh = t < 10;  fprintf('hold, local w: total rms %.2f pN, tracking-error part rms %.2f pN | a_33-hat hold %.3f, scan %.3f\n', rms(Fl(mh & isfinite(Fl(:, 3)), 3)), rms(Fe(mh & isfinite(Fe(:, 3)), 3)), mean(r.aN(mh)), mean(r.aN(m)));
k = (1:3:n)';  L = {'F_x', 'F_y', 'F_z'};  LD = {'\Delta x_d', '\Delta y_d', '\Delta z_d'};
g = figure('Color', 'w', 'Position', [40 40 1900 1000], 'Visible', 'off');  tl = tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact', 'TileIndexing', 'rowmajor');
for q = 1:3
    a = nexttile(tl);  plot(a, t(k), f(k, q), 'Color', cE, 'LineWidth', 0.5);  ylim(a, [-25 25]);  xlim(a, [0 t(end)]);  ylabel(a, [L{q} '  [pN]']);  if q == 3; xlabel(a, 'Time (sec)'); end
    b = nexttile(tl);  plot(b, t(k), D(k, q), 'Color', cE, 'LineWidth', 0.6);  xlim(b, [0 t(end)]);  ylim(b, [-0.03 0.06]);  ylabel(b, [LD{q} '  [\mum per step]']);  if q == 3; xlabel(b, 'Time (sec)'); end
    for c = [a b]; box(c, 'on');  set(c, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2); end
end
fn = fullfile(fo, 'fig_1103_flow_force_meas.png');  exportgraphics(g, fn, 'Resolution', 130);  close(g);  system(['open -a Preview "' fn '"']);
