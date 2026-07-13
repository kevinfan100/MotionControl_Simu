%VERIFY_TAYLOR_V_BUDGET  a(p_d - dxhat3) vs a_hat + v-budget attribution (z axis).
%
%   Verifies the 4state_del_hd.tex "e_ax -> v" section on the taylor arm:
%
%   (1) Identity check: a_hat[k] vs a( x_hat[k] ), x_hat = p_d - dxhat3
%       (project convention delta_x = p_d - x). a(.) is the full nonlinear
%       gain a_nom / c_perp(h_bar), not the first-order Taylor map.
%   (2) Discriminating scatter: a_hat - a_true vs a(p_d - dxhat3) - a_true
%       (slope 1 = a_hat follows the estimated position).
%   (3) v budget: v := a_hat - a_det + a'*dxhat3 obeys the exact recursion
%       dv = g1*e_y1 + g2*e_y2 + da'*dxhat3, g_j = L(4,j) + a'*L(3,j);
%       the cumulative sums V1/V2/C3 must close on the measured v.
%
%   Arm A2 (use_taylor_gain, aprime_source='known'), 1 Hz osc, seed 1, same
%   scenario and figure style as plot_aprime_overlay_4state (EXP style).
%   Outputs 5 figures to test_results/aprime_4state/ (gitignored).

clear; clc;

[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
project_root = fileparts(fileparts(script_dir));
addpath(fullfile(project_root, 'model'), fullfile(project_root, 'model', 'config'), ...
        fullfile(project_root, 'model', 'wall_effect'), ...
        fullfile(project_root, 'model', 'thermal_force'), ...
        fullfile(project_root, 'model', 'trajectory'), ...
        fullfile(project_root, 'model', 'controller'), ...
        fullfile(project_root, 'model', 'dual_track'), script_dir);
out_dir = fullfile(project_root, 'test_results', 'aprime_4state');
if ~exist(out_dir, 'dir'); mkdir(out_dir); end

% --- scenario (same as compare_aprime_4state, arm A2) ---
cfg = user_config();
cfg.eq17_variant    = '4state';
cfg.trajectory_type = 'osc';
cfg.h_init = 50; cfg.h_bottom = 2.7; cfg.amplitude = 2.5;
cfg.frequency = 1; cfg.n_cycles = 2;
cfg.t_hold = 0.5; cfg.t_descend_override = 1.0; cfg.T_sim = 4.0;
pc = physical_constants(); cfg.h_min = 1.05 * pc.R;
cfg.ctrl_enable = true; cfg.thermal_enable = true; cfg.meas_noise_enable = true;
cfg.lambda_c = 0.7; cfg.a_pd = 0.05; cfg.a_cov = 0.05;
cfg.meas_noise_std = [0.00062; 0.00057; 0.00331];
cfg.suppress_xD = true; cfg.h_bar_safe = 1;
cfg.use_taylor_gain = true;
cfg.aprime_source   = 'known';

ro = struct('seed', 1, 'verbose', false, 'collect_diag', true);
so = run_pure_simulation(cfg, ro);

t = so.tout;
P = so.meta.params_value;
w_hat = P.wall.w_hat; pz = P.wall.pz; R = P.common.R;
a_nom = P.ctrl.Ts / P.ctrl.gamma;
N_t = numel(t);

% --- a(.) evaluated at the estimated position x_hat = p_d - dxhat3 (z axis) ---
dxhat3_z = so.diag.delta_x_hat_3(:, 3);                 % [um]
hb_hat = max((so.p_d_out * w_hat - dxhat3_z - pz) / R, 1.001);
a_at_xhat = zeros(N_t, 1);
for k = 1:N_t
    [~, c_pe] = calc_correction_functions(hb_hat(k));
    a_at_xhat(k) = a_nom / c_pe;                        % z axis (c_perp)
end
a_hat  = so.diag.a_hat(:, 3);
a_true = so.a_true_out(:, 3);

% --- a_det(h_bar_d) = a(p_d): common reference removed from every curve ---
hbd = max((so.p_d_out * w_hat - pz) / R, 1.001);
a_det_d = zeros(N_t, 1);
for k = 1:N_t
    [~, c_pe] = calc_correction_functions(hbd(k));
    a_det_d(k) = a_nom / c_pe;
end

% --- figure (EXP style, mirrors plot_aprime_overlay_4state gap view) ---
COL_TH = [0 0.6 0]; COL_K = [0.8 0 0]; COL_D = [0 0.35 0.8];
FS = 18; LFS = 14; AXLW = 2.0;
T_END = ceil(t(end));

f = figure('Position', [80 80 1100 480], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
hold on;                                                % --- gap overlay (- a(p_d))
hT = plot(t, (a_true - a_det_d) * 1e3, '-', 'Color', COL_TH, 'LineWidth', 2.2, ...
          'DisplayName', 'a_{true} - a(p_d)');
hM = plot(t, (a_at_xhat - a_det_d) * 1e3, '-', 'Color', COL_D, 'LineWidth', 1.4, ...
          'DisplayName', 'a(p_d - \deltax\_hat_3) - a(p_d)');
hK = plot(t, (a_hat - a_det_d) * 1e3, '-', 'Color', COL_K, 'LineWidth', 1.4, ...
          'DisplayName', 'a\_hat - a(p_d)');
xlim([0 T_END]);
ylabel('a_z - a_z(p_d)  (nm/pN)', 'FontSize', FS, 'FontWeight', 'bold');
xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
legend([hT hM hK], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
       'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;

fout = fullfile(out_dir, 'fig_taylor_ahat_vs_a_at_xhat_z.png');
exportgraphics(f, fout, 'Resolution', 150);
close(f);
fprintf('[ahat_vs_a_at_xhat] %s\n', fout);

% --- console: osc-window stats (z, first cycle discarded, project convention) ---
t_osc0  = cfg.t_hold + cfg.t_descend_override;
osc_dur = cfg.n_cycles / cfg.frequency;
t_disc  = min(1 / cfg.frequency, 0.5 * osc_dur);
t_osc   = (t >= t_osc0 + t_disc) & (t < t_osc0 + osc_dur);
rel_map  = a_hat(t_osc) ./ a_at_xhat(t_osc) - 1;
rel_true = a_hat(t_osc) ./ a_true(t_osc)    - 1;
fprintf('osc window (%.1f-%.1f s), z axis:\n', t_osc0 + t_disc, t_osc0 + osc_dur);
fprintf('  a_hat vs a(p_d - dxhat3): bias %+6.2f%%  std %6.2f%%\n', ...
        100 * mean(rel_map), 100 * std(rel_map));
fprintf('  a_hat vs a_true         : bias %+6.2f%%  std %6.2f%%\n', ...
        100 * mean(rel_true), 100 * std(rel_true));

% ======================================================================
% Figure 2: "which position does a_hat follow?" -- discriminating view.
% The two candidates a(p_d - dxhat3) and a_true only differ where the
% estimated and true deviations disagree; the verification looks at those
% moments only:
%   tile 1: zoom overlay near the trough (gap view, common a_det removed)
%   tile 2: scatter  y = a_hat - a_true  vs  x = a(p_d - dxhat3) - a_true.
%           a_hat follows the estimate -> slope 1; the truth -> slope 0.
% ======================================================================

f2 = figure('Position', [80 80 1100 760], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile; hold on;                                      % --- top: zoom overlay (gap view)
Z0 = 2.30; Z1 = 2.62;                                   % 0.32 s around the trough at 2.5 s
wz = (t >= Z0) & (t <= Z1);
hT = plot(t(wz), (a_true(wz)    - a_det_d(wz)) * 1e3, '-', 'Color', COL_TH, ...
          'LineWidth', 2.2, 'DisplayName', 'a_{true} - a(p_d)');
hM = plot(t(wz), (a_at_xhat(wz) - a_det_d(wz)) * 1e3, '-', 'Color', COL_D, ...
          'LineWidth', 1.4, 'DisplayName', 'a(p_d - \deltax\_hat_3) - a(p_d)');
hK = plot(t(wz), (a_hat(wz)     - a_det_d(wz)) * 1e3, '-', 'Color', COL_K, ...
          'LineWidth', 1.4, 'DisplayName', 'a\_hat - a(p_d)');
xlim([Z0 Z1]);
ylabel('a_z - a_z(p_d)  (nm/pN)', 'FontSize', FS, 'FontWeight', 'bold');
legend([hT hM hK], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
       'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;

nexttile; hold on;                                      % --- bottom: discriminating scatter
xd = (a_at_xhat(t_osc) - a_true(t_osc)) * 1e3;          % candidate disagreement
yd = (a_hat(t_osc)     - a_true(t_osc)) * 1e3;
xd = xd - mean(xd); yd = yd - mean(yd);                 % demean (LF bias -> intercept)
pfit = polyfit(xd, yd, 1);
cc   = corrcoef(xd, yd);
hS = plot(xd, yd, '.', 'Color', [0.55 0.55 0.55], 'MarkerSize', 4, ...
          'DisplayName', sprintf('osc samples (slope %.2f, corr %.2f)', pfit(1), cc(1, 2)));
xl = max(abs(xd)) * [-1 1];
h1 = plot(xl, xl,       '--', 'Color', COL_D, 'LineWidth', 1.8, ...
          'DisplayName', 'slope 1: follows estimate');
h0 = plot(xl, [0 0],    '--', 'Color', COL_TH, 'LineWidth', 1.8, ...
          'DisplayName', 'slope 0: follows truth');
xlim(xl);
xlabel('a(p_d - \deltax\_hat_3) - a_{true}  (nm/pN)', 'FontSize', FS, 'FontWeight', 'bold');
ylabel('a\_hat - a_{true}  (nm/pN)', 'FontSize', FS, 'FontWeight', 'bold');
legend([hS h1 h0], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
       'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;

fout2 = fullfile(out_dir, 'fig_taylor_ahat_follows_which_z.png');
exportgraphics(f2, fout2, 'Resolution', 150);
close(f2);
fprintf('[ahat_follows_which] %s\n', fout2);
fprintf('  discriminating regression (osc window): slope %.3f, corr %.3f\n', pfit(1), cc(1, 2));

% ======================================================================
% v-budget: attribute the identity violation v to its injection channels.
% Exact per-step recursion (linearized frame, z axis):
%   v[k] := a_hat[k] - a_det[k] + a'[k]*dxhat3[k]
%   v[k]-v[k-1] = (a'[k]-a'[k-1])*dxhat3[k-1]          (C3: a' drift)
%               + (L41[k] + a'[k]*L31[k]) * e_y1[k]    (V1: y1 leak, g1*e1)
%               + (L42[k] + a'[k]*L32[k]) * e_y2[k]    (V2: y2 leak, g2*e2)
% Predict terms cancel exactly against the a_det / dxhat3 motion, so the
% account must close (residual ~ 0) or the v model is incomplete.
% ======================================================================
ap  = so.diag.a_prime_used(:, 3);
K41 = so.diag.K_kf_a_y1(:, 3);     K31 = so.diag.K_kf_dx_y1(:, 3);
K42 = so.diag.K_kf_a_y2(:, 3);     K32 = so.diag.K_kf_dx_y2(:, 3);
e1  = so.diag.innovation_y1(:, 3); e2  = so.diag.innovation_y2(:, 3);
a_det_c = so.diag.a_x_det(:, 3);   % controller's own a_det (exact bookkeeping frame)

g1 = K41 + ap .* K31;
g2 = K42 + ap .* K32;
leak1 = g1 .* e1;
leak2 = g2 .* e2;
c3    = [0; diff(ap)] .* [0; dxhat3_z(1:end-1)];

v_lin = a_hat - a_det_c + ap .* dxhat3_z;
k0  = 3;                            % skip init row + first EKF step baseline
idx = (k0:N_t).';
V1 = cumsum(leak1(idx)); V2 = cumsum(leak2(idx)); C3 = cumsum(c3(idx));
v_meas  = v_lin(idx) - v_lin(k0 - 1);
V_sum   = V1 + V2 + C3;
closure = V_sum - v_meas;
fprintf('v-budget closure: max|res| %.3g nm/pN (v range %.3g), rms %.3g\n', ...
        max(abs(closure)) * 1e3, range(v_meas) * 1e3, rms(closure) * 1e3);

% e_ax LF overlay: v should reproduce the LF body of -(a_true - a_hat)
eax_lf = movmean(a_hat - a_true, 42);              % 26 ms MA
eax_lf = eax_lf(idx) - eax_lf(k0 - 1);

% --- fig A: who feeds a_hat (|L(4,:)| per channel) ---
fa = figure('Position', [80 80 1100 480], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
semilogy(t, movmean(abs(K41), 33), '-', 'Color', COL_D, 'LineWidth', 1.6); hold on;
semilogy(t, movmean(abs(K42), 33), '-', 'Color', COL_K, 'LineWidth', 1.6);
xlim([0 T_END]);
ylabel('|L_{4j}|  (-)', 'FontSize', FS, 'FontWeight', 'bold');
xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
legend({'|L_{41}|  (y_1 \rightarrow a\_hat)', '|L_{42}|  (y_2 \rightarrow a\_hat)'}, ...
       'Location', 'northoutside', 'Orientation', 'horizontal', ...
       'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
fouta = fullfile(out_dir, 'fig_taylor_ell_gains_z.png');
exportgraphics(fa, fouta, 'Resolution', 150); close(fa);
fprintf('[ell_gains] %s\n', fouta);

% --- fig B: slip gains g = L4 + a'*L3 (lock-breaking per unit innovation) ---
fb = figure('Position', [80 80 1100 480], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
plot(t, g1, '-', 'Color', COL_D, 'LineWidth', 1.2); hold on;
plot(t, g2, '-', 'Color', COL_K, 'LineWidth', 1.2);
xlim([0 T_END]);
ylabel('g_j = L_{4j} + a''\cdotL_{3j}', 'FontSize', FS, 'FontWeight', 'bold');
xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
legend({'g_1  (y_1)', 'g_2  (y_2)'}, ...
       'Location', 'northoutside', 'Orientation', 'horizontal', ...
       'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
foutb = fullfile(out_dir, 'fig_taylor_slip_gains_z.png');
exportgraphics(fb, foutb, 'Resolution', 150); close(fb);
fprintf('[slip_gains] %s\n', foutb);

% --- fig C: verdict (cumulative budget vs measured v; C3 ~ 0 omitted) ---
fc = figure('Position', [80 80 1100 520], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
hold on;
hV = plot(t(idx), v_meas * 1e3, '-',  'Color', 'k', 'LineWidth', 2.4, ...
          'DisplayName', 'v (measured)');
h1 = plot(t(idx), V1 * 1e3, '-',  'Color', COL_D, 'LineWidth', 1.5, ...
          'DisplayName', 'V_1 = \Sigma g_1e_{y1}');
h2 = plot(t(idx), V2 * 1e3, '-',  'Color', COL_K, 'LineWidth', 1.5, ...
          'DisplayName', 'V_2 = \Sigma g_2e_{y2}');
xlim([0 T_END]);
ylabel('\Delta a_z  (nm/pN)', 'FontSize', FS, 'FontWeight', 'bold');
xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
legend([hV h1 h2], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
       'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
foutc = fullfile(out_dir, 'fig_taylor_v_budget_z.png');
exportgraphics(fc, foutc, 'Resolution', 150); close(fc);
fprintf('[v_budget] %s\n', foutc);

% component totals at epoch boundaries
for tb = [1.5, 3.5, t(end)]
    j = find(t(idx) <= tb, 1, 'last');
    fprintf('  t=%.2f s: v %+7.3f | V1 %+7.3f  V2 %+7.3f  C3 %+7.3f  (nm/pN)\n', ...
            tb, v_meas(j) * 1e3, V1(j) * 1e3, V2(j) * 1e3, C3(j) * 1e3);
end
