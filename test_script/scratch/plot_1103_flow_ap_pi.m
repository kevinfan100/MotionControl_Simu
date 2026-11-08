% STATUS: SCRATCH -- 2026-11-03 (user): same run as fig_1103_flow_motion (temp_1103_flow.mat, plot_1103_flow.m, seed 1). Figure 1: slopes a'_33, a'_11 estimate against true (plane law at the true gap). Figure 2: PI regulation: a_33 estimate and true against a_d | error e = a_d - a_33-hat and a_d - a_33 | step of the set height from the P and the I part (nm per step, rebuilt from the runner formulas with the logged a_33-hat, a'_33-hat, integral) | gap true and estimate against the gap at a_d.
%   Sanity: P + I equals the logged change of the set height along w-hat. Figures in Preview. Outputs fig_1103_flow_ap.png, fig_1103_flow_pi.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');  load(fullfile(fo, 'temp_1103_flow.mat'), 'r');  Rr = 2.25;  Ts = 1 / 1612;  ad = 0.07;  tau_a = 0.06;  tau_i = 0.5;  FS = 14;  cT = [0.85 0 0];  cE = [0 0.25 0.85];  AB = char(257);  pr = char(8242);  MU = [char(956) 'm'];
hs = fzero(@(h) 1 / local_c(h, 2) - ad, [1.001 30]);  n = find(isfinite(r.aN(1:r.kend)), 1, 'last');  t = r.t(1:n);  t = t(:);  k = (1:3:n)';
sN = local_slope(r.h(1:n), 2);  sT = local_slope(r.h(1:n), 1);
f = figure('Color', 'w', 'Position', [40 40 1800 800], 'Visible', 'off');  tl = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
a1 = nexttile(tl);  hold(a1, 'on');  h1 = plot(a1, t(k), sN(k), 'Color', cT, 'LineWidth', 1.4);  h2 = plot(a1, t(k), r.apN(k), 'Color', cE, 'LineWidth', 0.7);  ylim(a1, [-1 3]);  ylabel(a1, [AB pr '_{33}']);  legend(a1, [h1 h2], {'True', 'Estimate'}, 'Location', 'northoutside', 'Orientation', 'horizontal');
a2 = nexttile(tl);  hold(a2, 'on');  plot(a2, t(k), sT(k), 'Color', cT, 'LineWidth', 1.4);  plot(a2, t(k), r.apT(k), 'Color', cE, 'LineWidth', 0.7);  ylim(a2, [-0.5 1.5]);  ylabel(a2, [AB pr '_{11}']);  xlabel(a2, 'Time (sec)');
for a = [a1 a2]; xlim(a, [0 t(end)]);  box(a, 'on');  set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2); end
fn1 = fullfile(fo, 'fig_1103_flow_ap.png');  exportgraphics(f, fn1, 'Resolution', 130);  close(f);
m = t >= r.tsc & t <= r.rs_tend;  fprintf('scan: a''_33 est/true median %.2f, a''_11 est/true median %.2f; descent 10-11 s a''_33 est/true %.2f\n', median(r.apN(m) ./ sN(m)), median(r.apT(m) ./ sT(m)), mean(r.apN(t >= 10.5 & t < 11)) / mean(sN(t >= 10.5 & t < 11)));
e = ad - r.aN(1:n);  et = ad - r.aN_tr(1:n);  sl = max(r.apN(1:n), (8 / 9) * (1 - min(r.aN(1:n), 1)).^2);  eI = r.eI(1:n);  eI = eI(:);  sl = sl(:);  e = e(:);  et = et(:);  eI(~isfinite(eI)) = 0;  on = t >= r.tsc;
dP = (Ts / tau_a) * e ./ sl;  dI = Ts^2 / (tau_a * tau_i) * eI ./ sl;  dP(~on) = NaN;  dI(~on) = NaN;  floor_on = r.apN(1:n) < (8 / 9) * (1 - min(r.aN(1:n), 1)).^2;
hg = linspace(1.0005, 40, 6000);  ag = arrayfun(@(h) 1 / local_c(h, 2), hg);  hh = interp1(ag, hg, min(max(r.aN(1:n), ag(1)), ag(end)));  hh = hh(:);
fprintf('scan: mean e %+.4f, mean true error %+.4f, mean |true error| %.4f | P step rms %.2f nm, I step mean %+.2f nm | slope floor active %.0f%% of scan steps | gap true mean %.3f, target %.3f um\n', mean(e(m)), mean(et(m)), mean(abs(et(m))), 1e3 * Rr * rms(dP(m)), 1e3 * Rr * mean(dI(m)), 100 * mean(floor_on(m)), mean(r.h(m)) * Rr, hs * Rr);
f = figure('Color', 'w', 'Position', [40 40 1800 1150], 'Visible', 'off');  tl = tiledlayout(4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
b1 = nexttile(tl);  hold(b1, 'on');  h1 = plot(b1, t(k), r.aN_tr(k), 'Color', cT, 'LineWidth', 1.2);  h2 = plot(b1, t(k), r.aN(k), 'Color', cE, 'LineWidth', 0.7);  h3 = yline(b1, ad, 'k--', 'LineWidth', 1.4);  ylim(b1, [0 0.2]);  ylabel(b1, [AB '_{33}']);  legend(b1, [h1 h2 h3], {'True', 'Estimate', sprintf('%s_d = %.2f', AB, ad)}, 'Location', 'northoutside', 'Orientation', 'horizontal');
b2 = nexttile(tl);  hold(b2, 'on');  plot(b2, t(k), movmean(et(k), 25), 'Color', cT, 'LineWidth', 1.2);  plot(b2, t(k), movmean(e(k), 25), 'Color', cE, 'LineWidth', 1);  yline(b2, 0, 'k:');  ylim(b2, [-0.04 0.04]);  ylabel(b2, [AB '_d - ' AB '_{33}']);
b3 = nexttile(tl);  hold(b3, 'on');  h4 = plot(b3, t(k), 1e3 * Rr * dP(k), 'Color', [0.6 0.6 0.6], 'LineWidth', 0.5);  h5 = plot(b3, t(k), 1e3 * Rr * dI(k), 'Color', [0 0.6 0.2], 'LineWidth', 1.4);  ylim(b3, [-3 3]);  ylabel(b3, ['set height step  [nm]']);  legend(b3, [h4 h5], {'P', 'I'}, 'Location', 'northeast');
b4 = nexttile(tl);  hold(b4, 'on');  plot(b4, t(k), r.h(k) * Rr, 'Color', cT, 'LineWidth', 1.2);  plot(b4, t(k), hh(k) * Rr, 'Color', cE, 'LineWidth', 0.7);  yline(b4, hs * Rr, 'k--', 'LineWidth', 1.4);  yline(b4, Rr, 'k-', 'LineWidth', 1.2);  ylim(b4, [2.2 2.7]);  ylabel(b4, ['gap  [' MU ']']);  xlabel(b4, 'Time (sec)');
for a = [b1 b2 b3 b4]; xlim(a, [9 t(end)]);  box(a, 'on');  set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2); end
fn2 = fullfile(fo, 'fig_1103_flow_pi.png');  exportgraphics(f, fn2, 'Resolution', 130);  close(f);  system(['open -a Preview "' fn1 '" "' fn2 '"']);
function s = local_slope(hb, w)
    s = arrayfun(@(h) (1 / local_c(max(h, 1.0003) + 1e-4, w) - 1 / local_c(max(h, 1.0003) - 1e-4, w)) / 2e-4, hb(:));
end
function c = local_c(h, w)
    [a, b] = calc_correction_functions(h);  c = a;  if w == 2; c = b; end
end
