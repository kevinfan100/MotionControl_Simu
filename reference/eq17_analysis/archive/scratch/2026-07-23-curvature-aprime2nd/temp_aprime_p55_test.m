%TEMP_APRIME_P55_TEST  Wall-aware P55_0 removes the first-hold a_z oscillation.
%
%   Demonstrates that seeding the weak-observability a'-as-state EKF's a' prior
%   variance with the WALL-CURVE value P55_0 = [a'(h_bar_init)]^2 (tiny in the
%   far field, h_bar_init~22) pins a_hat' confidently at ~0 through the
%   far-field hold. This removes the pent-up prior uncertainty that -- with the
%   default P55_0 = (0.1*a_nom/R)^2 (~2000x too large) -- releases through
%   F_e(4,5)=Delta_h_d into the gain the instant descent starts, causing a_hat_z
%   to overshoot / oscillate ("first-hold oscillation"). The already-on
%   time-varying Q55[k] still lets a_hat' grow once descent brings h_bar down.
%
%     Run A (baseline)  : default P55_0 = (0.1*a_nom/R)^2
%     Run B (wall-aware): P55_0 = oracle_ap(1)^2  (wall curve at h_bar_init)
%
%   Canonical 1Hz scenario, weak-obs a'-as-state (aprime_source='state'),
%   1st-order (RW) a', oracle time-varying Q55, seed 1 -- mirrors
%   temp_aprime_order_compare.m for cfg + oracle_ap + a_z(z) extraction.
%
%   TEMP diagnostic (chat 2026-07-22); nothing committed, delete after use.

[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
project_root = fileparts(script_dir);
addpath(fullfile(project_root, 'model'), fullfile(project_root, 'model', 'config'), ...
        fullfile(project_root, 'model', 'wall_effect'), ...
        fullfile(project_root, 'model', 'thermal_force'), ...
        fullfile(project_root, 'model', 'trajectory'), ...
        fullfile(project_root, 'model', 'controller'), ...
        fullfile(project_root, 'model', 'dual_track'), script_dir);
out_dir = fullfile(project_root, 'test_results', 'temp_aprime_order_figs');
if ~exist(out_dir, 'dir'); mkdir(out_dir); end

SEED = 1;

% ------------------------------------------------------------------
% Canonical 1Hz scenario (= temp_aprime_order_compare.m cfg block)
% ------------------------------------------------------------------
pc = physical_constants(); R = pc.R;
cfg = user_config();
cfg.eq17_variant = '4state'; cfg.trajectory_type = 'osc';
cfg.h_init = 50; cfg.h_bottom = 2.0*R; cfg.amplitude = 2.5;
cfg.frequency = 1; cfg.n_cycles = 4; cfg.t_hold = 0.5; cfg.t_descend_override = 1.0;
cfg.h_min = 1.05*R;
cfg.T_sim = cfg.t_hold + cfg.t_descend_override + cfg.n_cycles/cfg.frequency + 0.5; % 6.0
cfg.ctrl_enable = true; cfg.thermal_enable = true; cfg.meas_noise_enable = true;
cfg.lambda_c = 0.7; cfg.a_pd = 0.05; cfg.a_cov = 0.05;
cfg.meas_noise_std = [0.00062; 0.00057; 0.00331];
cfg.suppress_xD = true; cfg.h_bar_safe = 1;
cfg.use_taylor_gain = true; cfg.aprime_source = 'state';   % weak-obs a'-as-state

Pp = calc_simulation_params(cfg); Pp = Pp.Value;
a_nom = Pp.common.Ts / Pp.ctrl.gamma;   % [um/pN] free-space nominal gain
w = Pp.wall.w_hat; pz = Pp.wall.pz;

% segment boundaries (hold / descent / osc)
t1 = cfg.t_hold; t2 = t1 + cfg.t_descend_override; t3 = t2 + cfg.n_cycles/cfg.frequency;

fprintf('==================================================================\n');
fprintf(' Wall-aware P55_0 vs first-hold a_z oscillation -- canonical 1Hz, seed %d\n', SEED);
fprintf(' segments: hold[0,%.1f] descent[%.1f,%.1f] osc[%.1f,%.1f]\n', t1, t1, t2, t2, t3);
fprintf('==================================================================\n\n');

warning('off', 'MATLAB:nearlySingularMatrix');

% ------------------------------------------------------------------
% Prelim run: desired trajectory -> oracle a'(h_d[k]) -> P55_wall
% ------------------------------------------------------------------
so0 = temp_run_pure_sim_aprime2nd(cfg, struct('seed', SEED, 'collect_diag', true));
t = so0.tout; N = numel(t);
h_d = so0.p_d_out*w - pz;                       % desired height [um]
oracle_ap = zeros(N, 1);
for k = 1:N
    hbd = max(h_d(k)/R, 1.001);
    [~, cp, drv] = calc_correction_functions(hbd, true);
    oracle_ap(k) = -(a_nom/cp) * drv.K_h_perp / R;   % a'(h_d[k]) [um/pN/um]
end
P55_wall    = oracle_ap(1)^2;                   % wall-curve prior variance (tiny, far field)
P55_default = (0.1 * a_nom / R)^2;              % controller default (~2000x too large)

% ------------------------------------------------------------------
% Run A (baseline: default P55_0) and Run B (wall-aware P55_0)
% ------------------------------------------------------------------
cfgA = cfg; cfgA.aprime_order = 1; cfgA.aprime_Q_oracle_tv = true;   % default P55_0
soA = temp_run_pure_sim_aprime2nd(cfgA, struct('seed', SEED, 'collect_diag', true));

cfgB = cfg; cfgB.aprime_order = 1; cfgB.aprime_Q_oracle_tv = true;
cfgB.aprime_state_P0 = P55_wall;                                     % wall-aware P55_0
soB = temp_run_pure_sim_aprime2nd(cfgB, struct('seed', SEED, 'collect_diag', true));

warning('on', 'MATLAB:nearlySingularMatrix');

az_true = soA.a_true_out(:, 3);   % z-axis true gain [um/pN]
azA = soA.diag.a_hat(:, 3);       % z-axis EKF gain, baseline
azB = soB.diag.a_hat(:, 3);       % z-axis EKF gain, wall-aware

% ------------------------------------------------------------------
% First-hold oscillation metric over t in [0.5, 1.2]
% ------------------------------------------------------------------
win = (t >= 0.5) & (t <= 1.2);
stdA   = std(azA(win));                          maxeA = max(abs(azA(win) - az_true(win)));
stdB   = std(azB(win));                          maxeB = max(abs(azB(win) - az_true(win)));

% ------------------------------------------------------------------
% Console report
% ------------------------------------------------------------------
fprintf('  P55_0 prior variance (a'' state, slot 5):\n');
fprintf('    default  (0.1*a_nom/R)^2 = %.6e\n', P55_default);
fprintf('    wall     oracle_ap(1)^2  = %.6e   [a''(h_bar_init=%.1f)]^2\n', ...
        P55_wall, h_d(1)/R);
fprintf('    ratio    default/wall    = %.1f x\n\n', P55_default / P55_wall);

fprintf('  first-hold oscillation metric  (window t in [0.5, 1.2], z-axis, nm/pN):\n');
fprintf('    %-12s %-14s %-14s\n', 'run', 'std(a_hat_z)', 'max|a_hat-a_true|');
fprintf('    %-12s %-14.3f %-14.3f\n', 'baseline',   stdA*1e3, maxeA*1e3);
fprintf('    %-12s %-14.3f %-14.3f\n', 'wall-aware', stdB*1e3, maxeB*1e3);
fprintf('    reduction:  std %.2fx   max-err %.2fx\n\n', stdA/max(stdB,realmin), maxeA/max(maxeB,realmin));

if stdB < stdA && maxeB <= maxeA
    fprintf('  VERDICT: wall-aware P55_0 REDUCED the first-hold a_z oscillation.\n');
    fprintf('           std %.3f -> %.3f nm/pN, max-err %.3f -> %.3f nm/pN.\n\n', ...
            stdA*1e3, stdB*1e3, maxeA*1e3, maxeB*1e3);
else
    fprintf('  VERDICT: wall-aware P55_0 did NOT reduce the oscillation (std %.3f->%.3f, maxerr %.3f->%.3f nm/pN).\n\n', ...
            stdA*1e3, stdB*1e3, maxeA*1e3, maxeB*1e3);
end

% ------------------------------------------------------------------
% Figures (match existing style: bold labels, legend, no grid)
% ------------------------------------------------------------------
GRN = [0 0.6 0]; BLU = [0 0.2 0.8]; RED = [0.8 0 0]; FS = 15; LFS = 12; LW = 1.9;

% --- Fig 1: first-hold zoom (x in [0.4, 1.3]) ---
f1 = figure('Position', [60 60 1150 560], 'Color', 'w', 'Visible', 'off'); hold on;
h0 = plot(t, az_true*1e3, '-', 'Color', GRN, 'LineWidth', 3.0, 'DisplayName', 'a_{z,true}');
h1 = plot(t, azA*1e3,     '-', 'Color', BLU, 'LineWidth', 1.8, 'DisplayName', '\^a_z baseline (default P_{55,0})');
h2 = plot(t, azB*1e3,     '-', 'Color', RED, 'LineWidth', 1.8, 'DisplayName', '\^a_z wall-aware P_{55,0}');
for x = [t1 t2]; xline(x, '--', 'Color', [.6 .6 .6], 'LineWidth', 1.1, 'HandleVisibility', 'off'); end
xlim([0.4 1.3]);
ylabel('a_z (nm/pN)', 'FontSize', FS, 'FontWeight', 'bold');
xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
yl = ylim;
text((0.4+t1)/2, yl(1)+0.9*(yl(2)-yl(1)), 'hold', 'FontSize', LFS, 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
text((t1+1.3)/2, yl(1)+0.9*(yl(2)-yl(1)), 'descent', 'FontSize', LFS, 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
title('First-hold \^a_z: wall-aware P_{55,0} pins \^a''\approx0 -> no pent-up release at descent', ...
      'FontSize', FS-1, 'FontWeight', 'bold');
legend([h0 h1 h2], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
       'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', LW, 'Box', 'on'); grid off;
exportgraphics(f1, fullfile(out_dir, 'aprime_p55_firsthold.png'), 'Resolution', 200); close(f1);
fprintf('  saved: %s\n', fullfile(out_dir, 'aprime_p55_firsthold.png'));

% --- Fig 2: full trajectory (t in [0, 6], y in [0, 20]) ---
f2 = figure('Position', [60 60 1150 560], 'Color', 'w', 'Visible', 'off'); hold on;
g0 = plot(t, az_true*1e3, '-', 'Color', GRN, 'LineWidth', 2.6, 'DisplayName', 'a_{z,true}');
g1 = plot(t, azA*1e3,     '-', 'Color', BLU, 'LineWidth', 1.2, 'DisplayName', '\^a_z baseline (default P_{55,0})');
g2 = plot(t, azB*1e3,     '-', 'Color', RED, 'LineWidth', 1.2, 'DisplayName', '\^a_z wall-aware P_{55,0}');
for x = [t1 t2 t3]; xline(x, '--', 'Color', [.6 .6 .6], 'LineWidth', 1.1, 'HandleVisibility', 'off'); end
xlim([0 t(end)]); ylim([0 20]);
ylabel('a_z (nm/pN)', 'FontSize', FS, 'FontWeight', 'bold');
xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
text(t1/2, 18, 'hold', 'FontSize', LFS, 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
text((t1+t2)/2, 18, 'descent', 'FontSize', LFS, 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
text((t2+t3)/2, 18, 'osc', 'FontSize', LFS, 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
title('Full \^a_z(t): wall-aware stays healthy through descent + oscillation', ...
      'FontSize', FS-1, 'FontWeight', 'bold');
legend([g0 g1 g2], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
       'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', LW, 'Box', 'on'); grid off;
exportgraphics(f2, fullfile(out_dir, 'aprime_p55_full.png'), 'Resolution', 200); close(f2);
fprintf('  saved: %s\n', fullfile(out_dir, 'aprime_p55_full.png'));
