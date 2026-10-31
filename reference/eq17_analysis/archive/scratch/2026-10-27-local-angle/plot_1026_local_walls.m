% STATUS: SCRATCH -- 2026-10-26 (user): the local version (run_t3_xyz_fixed form 'local', 1026_local_angle_estimator.tex) at 15 um,
%   estimator always set up for a horizontal wall: theta-hat[0] = (0, 0), gains / slopes at the reflection seeds, off-diagonal 0,
%   sqrt P_theta[0] = 10 deg, gate 2 sqrt P (test option), Q of slopes / off-diagonal / angle = 0, R as the runner. Seeds 1-10, 10 s.
%   True wall set by wall = [theta_x theta_y] (deg) before calling; default (0, 0) = case 1 (negative control: the wall is horizontal).
%   Criteria (case 1, written before the run): theta-hat stays near 0, the angle error stays inside sqrt P (honesty ~ 1), no false tilt.
%   Figure rows: theta-hat_x and theta-hat_y (all seeds, red = truth); rms angle error and mean sqrt P; gain errors (true - estimate,
%   all seeds); gate open fraction over seeds. Output test_results/temp_3d_wsonly/fig_1026_local_wall_<tx>_<ty>.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
if ~exist('wall', 'var'); wall = [0 0]; end
Rr = 2.25;  T = 10;  seeds = 1:10;  n = numel(seeds);  FS = 14;  hatc = char(770);  ah = ['a' hatc];  th = ['θ' hatc];
cB = [0 0.3 0.85];  cG = [0 0.5 0.2];  cO = [0.85 0.4 0];  cBl = [0.6 0.72 1];  cGl = [0.6 0.85 0.65];  cOl = [1 0.75 0.55];
base = struct('theta_x', wall(1), 'theta_y', wall(2), 'h0', 15 / Rr, 'T', T, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', ...
              'form', 'local', 'gap_gate', 2);
if isempty(gcp('nocreate')); parpool('Processes', min(10, feature('numcores'))); end
runs = cell(1, n);
parfor i = 1:n; o = base;  o.seeds = seeds(i);  o.tag = 'lw';  R = run_t3_xyz_fixed(o);  runs{i} = R.runs{1}; end
t = runs{1}.t;  N = numel(t);  gT = runs{1}.aT_true(1);  gN = runs{1}.aN_true(1);
EX = NaN(N, n);  EY = EX;  SX = EX;  SY = EX;  EP = EX;  EN = EX;  GO = EX;
for i = 1:n
    r = runs{i};  k = 1:r.kend;
    EX(k, i) = wall(1) - r.thx(k);  EY(k, i) = wall(2) - r.thy(k);  SX(k, i) = r.sPth(k, 1);  SY(k, i) = r.sPth(k, 2);
    EP(k, i) = gT - r.aT(k);  EN(k, i) = gN - r.aN(k);  GO(k, i) = r.gate(k);
end
fig = figure('Color', 'w', 'Position', [40 40 1500 1350]);  tiledlayout(4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
nexttile;  hold on;  box on;
for i = n:-1:1
    plot(t, runs{i}.thy, 'Color', cBl, 'LineWidth', 0.8);  plot(t, runs{i}.thx, 'Color', cGl, 'LineWidth', 0.8);
end
h1 = plot(t, runs{1}.thy, 'Color', cB, 'LineWidth', 1.6);  h2 = plot(t, runs{1}.thx, 'Color', cG, 'LineWidth', 1.6);
yline(wall(2), 'r', 'LineWidth', 2.5);  yline(wall(1), 'r', 'LineWidth', 2.5);  ylabel('[\circ]');  ylim([-15 15] + mean(wall));
legend([h1 h2], {sprintf('%s_y  (seed 1 dark, seeds 2-10 light)', th), sprintf('%s_x', th)}, 'Location', 'northoutside', 'Orientation', 'horizontal');
nexttile;  hold on;  box on;
h1 = plot(t, sqrt(mean(EY.^2, 2, 'omitnan')), 'Color', cB, 'LineWidth', 2);  h2 = plot(t, mean(SY, 2, 'omitnan'), '--', 'Color', cB, 'LineWidth', 2);
h3 = plot(t, sqrt(mean(EX.^2, 2, 'omitnan')), 'Color', cG, 'LineWidth', 2);  h4 = plot(t, mean(SX, 2, 'omitnan'), '--', 'Color', cG, 'LineWidth', 2);
ylabel('[\circ]');  ylim([0 12]);
legend([h1 h2 h3 h4], {sprintf('rms(θ_y - %s_y)', th), sprintf('%sP_{θy}', char(8730)), sprintf('rms(θ_x - %s_x)', th), sprintf('%sP_{θx}', char(8730))}, ...
       'Location', 'northoutside', 'Orientation', 'horizontal');
nexttile;  hold on;  box on;  yline(0, 'r', 'LineWidth', 2);
for i = n:-1:1; plot(t, EP(:, i), 'Color', cBl, 'LineWidth', 0.8);  plot(t, EN(:, i), 'Color', cOl, 'LineWidth', 0.8); end
h1 = plot(t, mean(EP, 2, 'omitnan'), 'Color', cB, 'LineWidth', 2);  h2 = plot(t, mean(EN, 2, 'omitnan'), 'Color', cO, 'LineWidth', 2);
ylim([-0.06 0.06]);  ylabel('true - estimate');
legend([h1 h2], {sprintf('a_%s - %s_%s  (mean dark)', char(8741), ah, char(8741)), sprintf('a_%s - %s_%s', char(8869), ah, char(8869))}, ...
       'Location', 'northoutside', 'Orientation', 'horizontal');
nexttile;  hold on;  box on;  plot(t, mean(GO, 2, 'omitnan'), 'k', 'LineWidth', 1.5);  ylim([-0.05 1.05]);
ylabel('gate open (fraction of seeds)');  xlabel('Time (sec)');
set(findall(fig, 'Type', 'axes'), 'XLim', [0 T], 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);
exportgraphics(fig, fullfile(fo, sprintf('fig_1026_local_wall_%d_%d.png', wall(1), wall(2))), 'Resolution', 150);  close(fig);
% ---- console ----
ks = arrayfun(@(s) find(t >= s - 1e-9, 1), [1 2 5 10]);
fprintf('wall (%g, %g) deg, 15 um, theta-hat[0] = (0, 0), gate 2 sqrtP\n', wall);
fprintf('  theta_y: mean err %s sd %s | rms %s | mean sqrtP %s | honesty %s\n', mat2str(round(mean(EY(ks, :), 2)', 2)), mat2str(round(std(EY(ks, :), 0, 2)', 2)), ...
        mat2str(round(sqrt(mean(EY(ks, :).^2, 2))', 2)), mat2str(round(mean(SY(ks, :), 2)', 2)), mat2str(round((sqrt(mean(EY(ks, :).^2, 2)) ./ mean(SY(ks, :), 2))', 2)));
fprintf('  theta_x: mean err %s sd %s | rms %s | mean sqrtP %s | honesty %s\n', mat2str(round(mean(EX(ks, :), 2)', 2)), mat2str(round(std(EX(ks, :), 0, 2)', 2)), ...
        mat2str(round(sqrt(mean(EX(ks, :).^2, 2))', 2)), mat2str(round(mean(SX(ks, :), 2)', 2)), mat2str(round((sqrt(mean(EX(ks, :).^2, 2)) ./ mean(SX(ks, :), 2))', 2)));
fprintf('  a_par err mean %s sd %s | a_perp err mean %s sd %s (1/2/5/10 s)\n', mat2str(round(mean(EP(ks, :), 2)', 4)), mat2str(round(std(EP(ks, :), 0, 2)', 4)), ...
        mat2str(round(mean(EN(ks, :), 2)', 4)), mat2str(round(std(EN(ks, :), 0, 2)', 4)));
fprintf('  gate open fraction per seed %s | max |theta-hat| over all seeds and time %.1f deg\n', mat2str(round(mean(GO, 1, 'omitnan'), 2)), ...
        max(abs([EX(:); EY(:)] - 0)));
