% STATUS: SCRATCH -- 2026-10-26 (user): the local version alone (run_t3_xyz_fixed form 'local', 1026_local_angle_estimator.tex),
%   20 deg wall, fixed point 15 um, theta-hat[0] = (0, 15) deg, gains at the reflection seeds, 10 s.
%   Part A: no gate, seeds 1-10 -- how many runs leave the wall (|theta-hat_y - 20| > 60 deg) and when; figure theta-hat_y and
%   a_par-hat - a_perp-hat per seed. Part B: gate 2 sqrt(P_gap) (test option), seed 1 in full: angles, gains, gain difference with the
%   gate band, off-diagonal elements (estimate and truth on the same axis), sqrt P of the angles vs the angle error, slopes.
%   Console: gain and slope errors over seeds 1-10 (mean, sd, rms / sqrt P) at 1 / 5 / 10 s.
%   Outputs test_results/temp_3d_wsonly/fig_1026_local_nogate.png, fig_1026_local_info_s1.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);
addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  T = 10;  seeds = 1:10;  n = numel(seeds);  FS = 14;  hatc = char(770);  ah = ['a' hatc];  th = ['θ' hatc];
cB = [0 0.3 0.85];  cG = [0 0.5 0.2];  cO = [0.85 0.4 0];
base = struct('theta_y', 20, 'h0', 15 / Rr, 'T', T, 'aprime', true, 'th0_deg', [0 15], 'gain0', 'seed', 'form', 'local');
if isempty(gcp('nocreate')); parpool('Processes', min(10, feature('numcores'))); end
rA = cell(1, n);  rB = cell(1, n);
parfor i = 1:n
    o = base;  o.seeds = seeds(i);  o.tag = 'ng';  R = run_t3_xyz_fixed(o);  rA{i} = R.runs{1};
    o.gap_gate = 2;  o.tag = 'gt';  R = run_t3_xyz_fixed(o);  rB{i} = R.runs{1};
end
% ---- part A: no gate ----
fig = figure('Color', 'w', 'Position', [60 60 1500 800]);  tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
a1 = nexttile;  hold on;  box on;  a2 = nexttile;  hold on;  box on;  nbad = 0;
for i = 1:n
    r = rA{i};  k = 1:r.kend;  t = r.t(k);  kb = find(abs(r.thy(k) - 20) > 60, 1);
    if ~isempty(kb); nbad = nbad + 1; fprintf('no gate, seed %2d: leaves at t = %.3f s, gap there %.3f\n', seeds(i), t(kb), r.dgap(kb));
    else; fprintf('no gate, seed %2d: stays, theta-hat at 10 s (%.2f, %.2f)\n', seeds(i), r.thx(r.kend), r.thy(r.kend)); end
    plot(a1, t, r.thy(k), 'LineWidth', 1);  plot(a2, t, r.dgap(k), 'LineWidth', 1);
end
yline(a1, 20, 'r', 'LineWidth', 2.5);  yline(a2, rA{1}.aT_true(1) - rA{1}.aN_true(1), 'r', 'LineWidth', 2.5);  yline(a2, 0, 'k:');
ylim(a1, [-40 80]);  ylim(a2, [-0.15 0.2]);  ylabel(a1, sprintf('%s_y  [\\circ]', th));
ylabel(a2, sprintf('%s_%s - %s_%s', ah, char(8741), ah, char(8869)));  xlabel(a2, 'Time (sec)');
set([a1 a2], 'XLim', [0 2], 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);
exportgraphics(fig, fullfile(fo, 'fig_1026_local_nogate.png'), 'Resolution', 150);  close(fig);
fprintf('no gate: %d of %d seeds leave the wall\n', nbad, n);
% ---- part B: gate, seed 1 in full ----
r = rB{1};  k = 1:r.kend;  t = r.t(k);
[cpa, cpe] = calc_correction_functions(15 / Rr);  dh = 1e-4;  [cpa2, cpe2] = calc_correction_functions(15 / Rr + dh);
apT = (1 / cpa2 - 1 / cpa) / dh;  apN = (1 / cpe2 - 1 / cpe) / dh;  gT = r.aT_true(1);  gN = r.aN_true(1);
fig = figure('Color', 'w', 'Position', [40 40 1500 1500]);  tiledlayout(6, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
nexttile;  hold on;  box on;  yline(20, 'r', 'LineWidth', 2.5);  yline(0, 'r', 'LineWidth', 2.5);
h1 = plot(t, r.thy(k), 'Color', cB, 'LineWidth', 1.4);  h2 = plot(t, r.thx(k), 'Color', cG, 'LineWidth', 1.4);
ylabel('[\circ]');  ylim([-15 30]);  legend([h1 h2], {sprintf('%s_y', th), sprintf('%s_x', th)}, 'Location', 'northoutside', 'Orientation', 'horizontal');
nexttile;  hold on;  box on;  yline(gT, 'r', 'LineWidth', 2.5);  yline(gN, 'r', 'LineWidth', 2.5);
h1 = plot(t, r.aT(k), 'Color', cB, 'LineWidth', 1.2);  h2 = plot(t, r.aN(k), 'Color', cO, 'LineWidth', 1.2);  ylim([0.7 1.0]);
legend([h1 h2], {sprintf('%s_%s', ah, char(8741)), sprintf('%s_%s', ah, char(8869))}, 'Location', 'northoutside', 'Orientation', 'horizontal');
nexttile;  hold on;  box on;  yline(gT - gN, 'r', 'LineWidth', 2.5);  yline(0, 'k:');
h1 = plot(t, r.dgap(k), 'Color', cB, 'LineWidth', 1.2);  h2 = plot(t, 2 * r.sgap(k), '--', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.2);
plot(t, -2 * r.sgap(k), '--', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.2);  ylim([-0.15 0.25]);
legend([h1 h2], {sprintf('%s_%s - %s_%s', ah, char(8741), ah, char(8869)), sprintf('%s2%sP  (gate band)', char(177), char(8730))}, ...
       'Location', 'northoutside', 'Orientation', 'horizontal');
nexttile;  hold on;  box on;
h1 = plot(t, r.off_tr(k, 1), 'r', 'LineWidth', 2);  h2 = plot(t, r.off_post(k, 1), 'Color', cB, 'LineWidth', 0.8);
h3 = plot(t, r.off_tr(k, 2), 'Color', [1 0.6 0.6], 'LineWidth', 2);  h4 = plot(t, r.off_post(k, 2), 'Color', cG, 'LineWidth', 0.8);  ylim([-0.012 0.012]);
legend([h1 h2 h3 h4], {'a_{uw} true', sprintf('%s_{uw}', ah), 'a_{vw} true', sprintf('%s_{vw}', ah)}, 'Location', 'northoutside', 'Orientation', 'horizontal');
nexttile;  hold on;  box on;
h1 = plot(t, abs(20 - r.thy(k)), 'Color', cB, 'LineWidth', 1);  h2 = plot(t, r.sPth(k, 2), '--', 'Color', cB, 'LineWidth', 2);
h3 = plot(t, abs(r.thx(k)), 'Color', cG, 'LineWidth', 1);  h4 = plot(t, r.sPth(k, 1), '--', 'Color', cG, 'LineWidth', 2);  ylim([0 15]);  ylabel('[\circ]');
legend([h1 h2 h3 h4], {sprintf('|θ_y - %s_y|', th), sprintf('%sP_{θy}', char(8730)), sprintf('|θ_x - %s_x|', th), sprintf('%sP_{θx}', char(8730))}, ...
       'Location', 'northoutside', 'Orientation', 'horizontal');
nexttile;  hold on;  box on;  yline(apT, 'r', 'LineWidth', 2.5);  yline(apN, 'r', 'LineWidth', 2.5);
h1 = plot(t, r.apT(k), 'Color', cB, 'LineWidth', 1.2);  h2 = plot(t, r.apN(k), 'Color', cO, 'LineWidth', 1.2);  xlabel('Time (sec)');
legend([h1 h2], {sprintf('%s′_%s', ah, char(8741)), sprintf('%s′_%s', ah, char(8869))}, 'Location', 'northoutside', 'Orientation', 'horizontal');
set(findall(fig, 'Type', 'axes'), 'XLim', [0 T], 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);
exportgraphics(fig, fullfile(fo, 'fig_1026_local_info_s1.png'), 'Resolution', 150);  close(fig);
% ---- console: gains over seeds (gate arm) ----
ks = arrayfun(@(s) find(rB{1}.t >= s - 1e-9, 1), [1 5 10]);
nm = {'a_par', 'a_perp', 'a_par''', 'a_perp'''};  tru = [gT gN apT apN];
for q = 1:4
    E = zeros(n, 3);  S = E;
    for i = 1:n
        ri = rB{i};  v = {ri.aT, ri.aN, ri.apT, ri.apN};  E(i, :) = tru(q) - v{q}(ks)';
        if q <= 2; S(i, :) = ri.sPa(ks, q)'; end
    end
    fprintf('%-7s true %.4f | error (true - est) at 1/5/10 s mean %s sd %s', nm{q}, tru(q), mat2str(round(mean(E), 4)), mat2str(round(std(E), 4)));
    if q <= 2; fprintf(' | rms / sqrtP %s', mat2str(round(sqrt(mean(E.^2)) ./ mean(S), 2))); end
    fprintf('\n');
end
