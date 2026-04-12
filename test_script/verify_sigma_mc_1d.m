%% verify_sigma_mc_1d.m
% 1D Monte Carlo verification of time-varying Sigma_e recursion
% for ctrl4 (3-state KF observer), matching variance_recursion.tex exactly.
%
% Setup (matches run_simulation.m trajectory):
%   - Oracle a_hat = a[k] (true wall-effect drag)
%   - Time-varying Q_kf[k] = 4*k_B*T*a[k]*diag(0,0,1)   (per .tex R1c)
%   - Online Riccati produces time-varying L[k]
%   - Stats window: t >= 0.2 sec (matches run_simulation.m warmup)
%   - N_mc: 10000 + 500 with independent seeds
%
% Output:
%   - test_results/verify/sigma_mc_1d_results.mat
%   - reference/for_test/fig_sigma_mc_1d_timeseries.png   (2 panels)
%
% Runtime: ~1 sec for N=10000 (vectorized over MC), ~0.1 sec for N=500.

clear; close all; clc;

%% === Paths ===
[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
project_root = fileparts(script_dir);
addpath(fullfile(project_root, 'model'));
addpath(fullfile(project_root, 'model', 'config'));
addpath(fullfile(project_root, 'model', 'wall_effect'));

%% === Physical constants ===
phys = physical_constants();
R_probe = phys.R;
gamma_N = phys.gamma_N;
Ts      = phys.Ts;
k_B     = phys.k_B;
T_temp  = phys.T;

sigma2_dXT_nom = 4*k_B*T_temp*Ts/gamma_N;
a_nom = Ts/gamma_N;

fprintf('sigma2_dXT_nom = %.4e um^2\n', sigma2_dXT_nom);
fprintf('a_nom          = %.4e um/pN\n', a_nom);

%% === Trajectory (matches run_simulation.m defaults) ===
t_hold   = 0.5;     % sec
h_init   = 50;      % um
h_bottom = 2.5;     % um
amplitude = 10;     % um (half-amplitude)
freq     = 1;       % Hz
n_cycles = 3;
t_descend = 1/freq;
T_margin  = 0.3;
T_sim     = t_hold + t_descend + n_cycles/freq + T_margin;
N_total   = round(T_sim/Ts);
t_vec     = (0:N_total-1)*Ts;

p_d = zeros(1, N_total);
for k = 1:N_total
    tk = t_vec(k);
    if tk < t_hold
        p_d(k) = h_init;
    elseif tk < t_hold + t_descend
        u = (tk - t_hold)/t_descend;
        p_d(k) = h_init + (h_bottom - h_init) * (1 - cos(pi*u))/2;
    else
        t_osc = tk - t_hold - t_descend;
        if t_osc < n_cycles/freq
            p_d(k) = h_bottom + amplitude * (1 - cos(2*pi*freq*t_osc));
        else
            p_d(k) = h_bottom;
        end
    end
end

fprintf('T_sim = %.2f s, N_total = %d samples\n', T_sim, N_total);

%% === Wall effect: a[k] per step (oracle, uses true h[k] = p_d[k]) ===
h_k = p_d;
gamma_k = zeros(1, N_total);
a_k     = zeros(1, N_total);
for k = 1:N_total
    h_bar = max(h_k(k)/R_probe, 1.001);
    [~, c_perp] = calc_correction_functions(h_bar);
    gamma_k(k) = gamma_N * c_perp;
    a_k(k)     = Ts / gamma_k(k);
end

fprintf('a_k range: [%.3e, %.3e] (ratio %.1fx)\n', ...
        min(a_k), max(a_k), max(a_k)/min(a_k));

%% === Controller params (ctrl4 observer) ===
lambda_c = 0.7;
kf_R     = 0.05 * sigma2_dXT_nom;
meas_noise_std_z = 0.00331;

Fe_obs = [0, 1, 0; 0, 0, 1; 0, 0, 1];
H_obs  = [1, 0, 0];

%% === Deterministic pre-pass 1: online Riccati for L[k] ===
fprintf('\n[Pre-pass 1] Online Riccati for L[k]...\n');
L_hist = zeros(3, N_total);
Pf = 10 * sigma2_dXT_nom * eye(3);

for k = 1:N_total
    Q_kf_k = 4*k_B*T_temp*a_k(k) * diag([0, 0, 1]);
    Pf_fcst = Fe_obs * Pf * Fe_obs' + Q_kf_k;
    Pf_fcst = 0.5*(Pf_fcst + Pf_fcst');
    S_k = H_obs * Pf_fcst * H_obs' + kf_R;
    L_k = Pf_fcst * H_obs' / S_k;
    L_hist(:, k) = L_k;
    Pf = (eye(3) - L_k*H_obs) * Pf_fcst;
    Pf = 0.5*(Pf + Pf');
end
fprintf('L_hist range: L(1) [%.3f, %.3f], L(3) [%.3f, %.3f]\n', ...
        min(L_hist(1,:)), max(L_hist(1,:)), ...
        min(L_hist(3,:)), max(L_hist(3,:)));

%% === Deterministic pre-pass 2: Sigma_e recursion (theory, R2) ===
fprintf('\n[Pre-pass 2] Sigma_e recursion (R2)...\n');
b_vec = [-1; 0; 0; -1];
Sigma_e = zeros(4,4);
Var_theory = zeros(1, N_total);

for k = 1:N_total
    L_k = L_hist(:, k);
    A_k = [lambda_c, 0,        0, 1-lambda_c;
           0,        -L_k(1),  1, 0;
           0,        -L_k(2),  0, 1;
           0,        -L_k(3),  0, 1];
    Q_cl_k = 4*k_B*T_temp*a_k(k) * (b_vec*b_vec');
    Sigma_e = A_k * Sigma_e * A_k' + Q_cl_k;
    Sigma_e = 0.5*(Sigma_e + Sigma_e');
    Var_theory(k) = Sigma_e(1,1);
end
fprintf('Var_theory range: [%.3e, %.3e]\n', min(Var_theory), max(Var_theory));

%% === MC loop (vectorized over realizations) ===
function [Var_MC, mean_MC] = run_mc_vec(N_mc, N_total, L_hist, a_k, lambda_c, b_vec, k_B_, T_, seed_start)
    rng(seed_start);
    e = zeros(4, N_mc);
    sum_x  = zeros(1, N_total);
    sum_x2 = zeros(1, N_total);

    for k = 1:N_total
        L_k = L_hist(:, k);
        A_k = [lambda_c, 0,        0, 1-lambda_c;
               0,        -L_k(1),  1, 0;
               0,        -L_k(2),  0, 1;
               0,        -L_k(3),  0, 1];
        sigma_eta = sqrt(4*k_B_*T_*a_k(k));
        eta = sigma_eta * randn(1, N_mc);
        q_k = b_vec * eta;

        e = A_k * e + q_k;

        sum_x(k)  = sum(e(1, :));
        sum_x2(k) = sum(e(1, :).^2);
    end

    mean_MC = sum_x / N_mc;
    Var_MC  = sum_x2 / N_mc - mean_MC.^2;
end

fprintf('\n[MC] N = 10000 ...\n');
t0 = tic;
[Var_MC_10000, mean_MC_10000] = run_mc_vec(10000, N_total, L_hist, a_k, lambda_c, b_vec, k_B, T_temp, 1);
fprintf('  done (%.1f sec)\n', toc(t0));

fprintf('\n[MC] N = 500 ...\n');
t0 = tic;
[Var_MC_500, mean_MC_500] = run_mc_vec(500, N_total, L_hist, a_k, lambda_c, b_vec, k_B, T_temp, 99999);
fprintf('  done (%.1f sec)\n', toc(t0));

%% === Statistics window ===
t_warmup = 0.2;
ss_start = round(t_warmup/Ts) + 1;
ss = ss_start:N_total;

abs_err_10000 = abs(Var_MC_10000(ss) - Var_theory(ss));
abs_err_500   = abs(Var_MC_500(ss)   - Var_theory(ss));
rel_err_10000 = abs_err_10000 ./ Var_theory(ss);
rel_err_500   = abs_err_500   ./ Var_theory(ss);

rms_rel_10000 = sqrt(mean(rel_err_10000.^2));
rms_rel_500   = sqrt(mean(rel_err_500.^2));

fprintf('\n=== Stats over t >= 0.2 ===\n');
fprintf('N=10000:  rms rel err = %.2f%%  (chi-sq theory: %.2f%%)\n', ...
        rms_rel_10000*100, sqrt(2/10000)*100);
fprintf('N=500 :  rms rel err = %.2f%%  (chi-sq theory: %.2f%%)\n', ...
        rms_rel_500*100, sqrt(2/500)*100);

%% === Save ===
out_dir = fullfile(project_root, 'test_results', 'verify');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end

save(fullfile(out_dir, 'sigma_mc_1d_results.mat'), ...
    'Var_theory', 'Var_MC_10000', 'Var_MC_500', 'mean_MC_10000', 'mean_MC_500', ...
    'h_k', 'a_k', 'gamma_k', 'L_hist', 't_vec', 'p_d', ...
    'N_total', 'Ts', 'ss', 't_warmup', 'lambda_c', 'kf_R', ...
    'rms_rel_10000', 'rms_rel_500');
fprintf('\nSaved: sigma_mc_1d_results.mat\n');

%% === Figure: 2 panels (h + Var) ===
FONT = 18; LW = 2; AXIS_LW = 1.5; LEGEND_FS = 14;
COL_TH  = [0 0.4470 0.7410];       % blue (recursion)
COL_10K = [0.8500 0.3250 0.0980];  % red (N=10000)
COL_500 = [0.9290 0.6940 0.1250];  % yellow (N=500)

fig1 = figure('Position', [100 100 1400 800], 'Visible', 'off');

ax1 = subplot(2, 1, 1);
plot(t_vec(ss), h_k(ss), 'k-', 'LineWidth', LW);
ylabel('h [um]');
xlim([t_warmup, T_sim]);
set(ax1, 'FontSize', FONT, 'LineWidth', AXIS_LW);

ax2 = subplot(2, 1, 2);
% Draw order (bottom to top): yellow first, red, blue recursion ON TOP
h_y = plot(t_vec(ss), Var_MC_500(ss),   '-', 'Color', COL_500, 'LineWidth', LW-0.5);
hold on;
h_r = plot(t_vec(ss), Var_MC_10000(ss), '-', 'Color', COL_10K, 'LineWidth', LW-0.5);
h_b = plot(t_vec(ss), Var_theory(ss),   '-', 'Color', COL_TH,  'LineWidth', LW+1);
hold off;
xlabel('Time [s]');
ylabel('Var(\delta x) [um^2]');
legend([h_b, h_r, h_y], ...
       {'Var_{recursion}', 'Var_{simulation} (N=10000)', 'Var_{simulation} (N=500)'}, ...
       'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LEGEND_FS);
xlim([t_warmup, T_sim]);
set(ax2, 'FontSize', FONT, 'LineWidth', AXIS_LW);

fig_dir = fullfile(project_root, 'reference', 'for_test');
if ~exist(fig_dir, 'dir'), mkdir(fig_dir); end
saveas(fig1, fullfile(fig_dir, 'fig_sigma_mc_1d_timeseries.png'));
fprintf('Saved: fig_sigma_mc_1d_timeseries.png\n');

fprintf('\nDone.\n');
