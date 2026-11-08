% STATUS: SCRATCH -- 2026-11-03 (user): current flow, seed 1 (temp_1103_flow.mat from plot_1103_flow.m). Estimator values (user 11-03, not the measurement-frame matrix): slots 12, 13 a_uw, a_vw in the frame of V-hat (logged off_k1, before the angle step) against the true elements in that frame (off_tr), and slots 10, 11 a_11 - a_33 against true. Force saved alone in a mat file: commanded force f_d [pN] in the measurement frame (runner line 660: f_d = (a_nom A_c) \ (p_d[k+1] - p-hat - lc (p_d[k] - p-hat)), all measurement-frame quantities), and a_o f_d [R] as logged (r.fa).
%   Sanity: a_o f_d / a_o = f_d; true a_13 at 15 um on the flat floor = 0. Figure in Preview. Outputs fig_1103_flow_offdiag.png, temp_force_1103_flow_seed1.mat
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');  load(fullfile(fo, 'temp_1103_flow.mat'), 'r');  FS = 14;  cT = [0.85 0 0];  cE = [0 0.25 0.85];  AB = char(257);
cfg = canonical_scenario(0.05, 1.10, 'ramp_tri', 1);  params = calc_simulation_params(cfg);  P = params.Value;  Ts = P.common.Ts;  Rr = P.common.R;  a_nom = Ts / P.common.gamma_N;  a_o = a_nom / Rr;
n = find(isfinite(r.aN(1:r.kend)), 1, 'last');  t = r.t(1:n);  t = t(:);  k = (1:3:n)';
E13 = r.off_k1(1:n, 1);  E23 = r.off_k1(1:n, 2);  T13 = r.off_tr(1:n, 1);  T23 = r.off_tr(1:n, 2);   % estimator slots 12, 13 (a_uw, a_vw at k+1 before the angle step) and the true elements in the same frame (V-hat)
f = figure('Color', 'w', 'Position', [40 40 1800 1000], 'Visible', 'off');  tl = tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
a1 = nexttile(tl);  hold(a1, 'on');  plot(a1, t(k), E13(k), 'Color', cE, 'LineWidth', 0.7);  ylabel(a1, [AB '_{uw}  slot 12, local of V' char(770)]);  
a2 = nexttile(tl);  hold(a2, 'on');  plot(a2, t(k), E23(k), 'Color', cE, 'LineWidth', 0.7);  ylabel(a2, [AB '_{vw}  slot 13, local of V' char(770)]);
a3 = nexttile(tl);  hold(a3, 'on');  h1 = plot(a3, t(k), r.aT_tr(k) - r.aN_tr(k), 'Color', cT, 'LineWidth', 1.4);  h2 = plot(a3, t(k), r.aT(k) - r.aN(k), 'Color', cE, 'LineWidth', 0.7);  ylabel(a3, [AB '_{11} - ' AB '_{33}  (slots 10, 11)']);  xlabel(a3, 'Time (sec)');  ylim(a3, [-0.05 0.5]);  legend(a3, [h1 h2], {'True', 'Estimate'}, 'Location', 'northeast');
for a = [a1 a2]; ylim(a, [-0.02 0.02]); end;  for a = [a1 a2 a3]; xlim(a, [0 t(end)]);  box(a, 'on');  set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2); end
fn = fullfile(fo, 'fig_1103_flow_offdiag.png');  exportgraphics(f, fn, 'Resolution', 130);  close(f);
for w = {[0 10], [11 16], [17 31]}; m = t >= w{1}(1) & t < w{1}(2);  fprintf('%g-%g s: estimate rms a_uw %.1e a_vw %.1e | true left rms a_uw %.1e a_vw %.1e\n', w{1}, rms(E13(m)), rms(E23(m)), rms(T13(m)), rms(T23(m))); end
fa = r.fa(1:n, :);  f_d = fa / a_o;  frame = 'measurement (x, y, z)';  note = 'f_d [pN]: commanded force of each control step, measurement frame; fa = a_o f_d [R per step]; a_o = Ts / (gamma_N R) [1/pN]; t [s]; seed 1, current flow (plot_1103_flow.m)';
save(fullfile(fo, 'temp_force_1103_flow_seed1.mat'), 't', 'f_d', 'fa', 'a_o', 'Ts', 'Rr', 'frame', 'note');
m = t < 10;  fprintf('a_o %.4e 1/pN, Rr %.3f um | f_d rms in the hold x %.3f y %.3f z %.3f pN; in the scan x %.3f y %.3f z %.3f pN\n', a_o, Rr, rms(f_d(m, 1)), rms(f_d(m, 2)), rms(f_d(m, 3)), rms(f_d(t > 12, 1)), rms(f_d(t > 12, 2)), rms(f_d(t > 12, 3)));
system(['open -a Preview "' fn '"']);
