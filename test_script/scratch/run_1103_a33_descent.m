% STATUS: SCRATCH -- 2026-11-03 (user): why a_33-hat is high during the descent (+.06..+.08 in a 5 s descent, check plot_1103_descent_1v5_seeds.m). 5 s descent, flat wall true (8,-8), seeds 1-5, 1 s at the bottom. Arms: A as is | B gain prediction with the TRUE slopes at the true gap (opts.cheat_slope_pred, reference) | C true angles (th0_deg = truth, sP_theta_deg tiny; angle error removed).
%   [hypothesis] the slope a'_33-hat lags the fast change of a_33, so the prediction lags and a_33-hat stays high. Rival: the angle error (4-6 deg in the descent) leaks into a_33 (descent along the wrong normal). Split: B removes the bias -> slope lag; C removes it -> angle. Also logged: a'_33-hat against the true slope. Figure in Preview. Output fig_1103_a33_descent.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');  Rr = 2.25;  Ts = 1 / 1612;  ad = 0.07;  NS = 5;  Tx = 8;  Ty = -8;  TD = 5;  AB = char(257);  pr = char(8242);  hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
base = struct('theta_x', Tx, 'theta_y', Ty, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'doc_timing', true, 'theta_out', true, 'gap_gate', 2, 'gate_one', true, 'hold_until', 10, 'tau_a', 0.06, 'a_d', ad, 'descent_T', TD, 'descent_v', (15 - hs * Rr) / TD, 'post_wait', [0 1], 'ap_q', true, 'ap_floor_law', true, 'sP_fix_theta_deg', 2, 'q_geo', [0.9 0.9], 'tag', 'a3', 'du_d', 40 * Ts / Rr, 'kappa_state', true, 'kappa_from_scan', true, 'kappa_pfix', 0.3, 'scan_ramp', 0.5, 'scan_len', 1e6, 'T', 10 + TD + 1);
[IA, ID] = ndgrid(1:3, 1:NS);  N = numel(IA);  rs = cell(N, 1);  nm = {'A as is', 'B true slope in the prediction', 'C true angles'};
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end;  pctRunOnAll warning('off', 'all');
parfor q = 1:N
    o = base;  o.seeds = ID(q);  if IA(q) == 2; o.cheat_slope_pred = true; end;  if IA(q) == 3; o.th0_deg = [Tx Ty];  o.sP_theta_deg = 0.01; end;  R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t(:), 'aN', r.aN(:), 'aN_tr', r.aN_tr(:), 'apN', r.apN(:), 'h', r.h(:), 'thx', r.thx(:), 'thy', r.thy(:));
end
f = figure('Color', 'w', 'Position', [40 40 1500 900], 'Visible', 'off');  tl = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');  a1 = nexttile(tl);  hold(a1, 'on');  a2 = nexttile(tl);  hold(a2, 'on');  cc = [0 0 0; 0.85 0 0; 0 0.6 0.2];  hl = gobjects(3, 1);
for a = 1:3
    id = find(IA == a);  n = min(cellfun(@(r) find(isfinite(r.aN), 1, 'last'), rs(id)));  t = rs{id(1)}.t(1:n) - (10 + TD);  D = zeros(n, NS);  S = zeros(n, NS);  St = zeros(n, NS);
    for s = 1:NS; r = rs{id(s)};  D(:, s) = r.aN(1:n) - r.aN_tr(1:n);  S(:, s) = r.apN(1:n);  St(:, s) = local_slope(r.h(1:n)); end
    dm = mean(D, 2);  w1 = t >= -2.5 & t < -0.5;  w2 = t >= 0.2;  e = arrayfun(@(s) acosd(min(sum(local_w(rs{s}.thx(1:n), rs{s}.thy(1:n)) .* local_w(Tx * ones(n, 1), Ty * ones(n, 1)), 2), 1)), id', 'UniformOutput', false);  E = sqrt(mean(cell2mat(e).^2, 2));
    fprintf('%-32s a33 est-true: descent (-2.5..-0.5 s) mean %+.4f, peak %+.4f | bottom (>.2 s) %+.4f | slope est/true in the descent %.2f | angle err in the descent %.2f deg\n', nm{a}, mean(dm(w1)), max(movmean(dm, 81)), mean(dm(w2)), mean(mean(S(w1, :))) / mean(mean(St(w1, :))), rms(E(w1)));
    k = (1:3:n)';  hl(a) = plot(a1, t(k), movmean(dm(k), 15), 'Color', cc(a, :), 'LineWidth', 1.5);  plot(a2, t(k), movmean(mean(S(k, :), 2), 15), 'Color', cc(a, :), 'LineWidth', 1.3);  if a == 1; plot(a2, t(k), mean(St(k, :), 2), 'Color', [0.85 0 0], 'LineStyle', '--', 'LineWidth', 1.6); end
end
yline(a1, 0, 'k:', 'HandleVisibility', 'off');  ylabel(a1, [AB '_{33} estimate - true, mean 5 seeds']);  ylabel(a2, [AB pr '_{33}, mean 5 seeds (dashed: true)']);  xlabel(a2, 'time from the bottom  [s]');  ylim(a1, [-0.04 0.12]);  ylim(a2, [-0.5 6]);
for a = [a1 a2]; xlim(a, [-5.5 1]);  box(a, 'on');  set(a, 'FontSize', 14, 'FontWeight', 'bold', 'LineWidth', 1.2); end;  legend(a1, hl, nm, 'Location', 'northoutside', 'Orientation', 'horizontal');
fn = fullfile(fo, 'fig_1103_a33_descent.png');  exportgraphics(f, fn, 'Resolution', 140);  close(f);  system(['open -a Preview "' fn '"']);
function s = local_slope(hb)
    s = arrayfun(@(h) (1 / local_cp(max(h, 1.0003) + 1e-4) - 1 / local_cp(max(h, 1.0003) - 1e-4)) / 2e-4, hb(:));
end
function w = local_w(thx, thy)
    tx = thx(:) * pi / 180;  ty = thy(:) * pi / 180;  w = [sin(ty) .* cos(tx), -sin(tx), cos(ty) .* cos(tx)];
end
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
