% verify_positioning_grid.m
% Comprehensive positioning verification: sweep (h, a_pd, axis, seed) and
% measure R22 ratio against closed-form IF_eff prediction.
%
% Grid:
%   h    ∈ {10, 20, 50} um         (wall correction varies)
%   a_pd ∈ {0.005, 0.05, 0.5}      (LP1 corner cases)
%   axis ∈ {X, Y, Z}                (per-sim free)
%   seeds ∈ {42, 100, 200}          (3 seeds × N_t samples)
%
% Total sims: 3 × 3 × 3 = 27 × ~18s = ~8 min
%
% For each (h, a_pd): compute closed-form ρ_T, ρ_N, ρ_mix, IF_eff_cf.
% From sim trace: compute R22_emp = var(σ²̂ residual) per axis.
% Compare R22 ratio across grid.

clear; close all; clc;
clear motion_control_law motion_control_law_eq17_core ...
      motion_control_law_7state trajectory_generator calc_thermal_force;
[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
project_root = fileparts(fileparts(script_dir));
cd(project_root);
addpath(fullfile(project_root, 'model'));
addpath(fullfile(project_root, 'model', 'config'));
addpath(fullfile(project_root, 'model', 'wall_effect'));
addpath(fullfile(project_root, 'model', 'thermal_force'));
addpath(fullfile(project_root, 'model', 'trajectory'));
addpath(fullfile(project_root, 'model', 'controller'));
addpath(fullfile(project_root, 'model', 'dual_track'));

%% Grid
h_list      = [10, 20, 50];
a_pd_list   = [0.005, 0.05, 0.5];
seed_list   = [42, 100, 200];
ax_names    = {'X', 'Y', 'Z'};
sigma_n_axes  = [0.00062; 0.000057; 0.00331];
sigma2_n_axes = sigma_n_axes.^2;
lambda_c    = 0.7;
a_cov       = 0.05;
s_ewma      = 1 - a_cov;
tau_max     = 30;
T_sim       = 20;

n_h = length(h_list); n_apd = length(a_pd_list); n_seed = length(seed_list);

constants = physical_constants();
kBT       = constants.k_B * constants.T;
R_radius  = constants.R;
gamma_N   = constants.gamma_N;
Ts        = 1/1600;
a_nom     = Ts/gamma_N;

save_dir   = fullfile(project_root, 'test_results', 'learn_variance');
cache_path = fullfile(save_dir, 'positioning_grid_27sims.mat');

%% Run grid sims (cached)
if ~exist(cache_path, 'file')
    fprintf('=== Running %d grid sims (~%.1f min) ===\n', n_h*n_apd*n_seed, ...
            n_h*n_apd*n_seed*18/60);
    delta_x_m_grid    = cell(n_h, n_apd, n_seed);
    sigma2_ctrl_grid  = cell(n_h, n_apd, n_seed);
    tout_ref = [];
    t0_total = tic;
    for ih = 1:n_h
        h_static = h_list(ih);
        for ip = 1:n_apd
            a_pd = a_pd_list(ip);
            for is = 1:n_seed
                seed = seed_list(is);
                fprintf('  h=%dum  a_pd=%.4f  seed=%d ... ', h_static, a_pd, seed);
                config = user_config();
                config.trajectory_type   = 'positioning';
                config.h_init            = h_static;
                config.T_sim             = T_sim;
                config.controller_type   = 'eq17_7state';
                config.iir_warmup_mode   = 'prefill';
                config.thermal_enable    = true;
                config.meas_noise_enable = true;
                config.meas_noise_std    = sigma_n_axes;
                config.a_pd              = a_pd;
                config.a_cov             = a_cov;
                opts = struct('seed', seed, 'verbose', false, 'collect_diag', true);
                clear motion_control_law motion_control_law_eq17_core ...
                      motion_control_law_7state trajectory_generator calc_thermal_force;
                ts = tic;
                simOut = run_pure_simulation(config, opts);
                fprintf('%.1fs\n', toc(ts));
                delta_x_m_grid{ih, ip, is}   = simOut.diag.delta_x_m;
                sigma2_ctrl_grid{ih, ip, is} = simOut.diag.sigma2_dxr_hat;
                if isempty(tout_ref); tout_ref = simOut.tout; end
            end
        end
    end
    fprintf('Total: %.1fs\n', toc(t0_total));
    save(cache_path, 'delta_x_m_grid', 'sigma2_ctrl_grid', 'tout_ref', ...
         'h_list', 'a_pd_list', 'seed_list', 'sigma_n_axes', '-v7.3');
    fprintf('Cached → %s\n', cache_path);
else
    fprintf('Loading cache → %s\n', cache_path);
    load(cache_path);
end

N_t = length(tout_ref);
warmup_idx = tout_ref >= 2;

%% Per-cell analysis: compute R22 ratio + IF_eff
% Storage: [h, a_pd, seed, axis]
ratio_OptA_grid     = zeros(n_h, n_apd, n_seed, 3);
ratio_new_grid      = zeros(n_h, n_apd, n_seed, 3);
IF_eff_cf_grid      = zeros(n_h, n_apd, 3);          % seed-independent (theory)
IF_eff_emp_grid     = zeros(n_h, n_apd, n_seed, 3);
sigma2_mean_grid    = zeros(n_h, n_apd, n_seed, 3);
sigma2_th_grid      = zeros(n_h, n_apd, 3);          % theory mean (seed-independent)

R22_prefactor = 2*a_cov/(2-a_cov);

% Closed-form ρ_T, ρ_N (seed/axis independent — only α, β, c)
for ih = 1:n_h
    h_static = h_list(ih);
    h_bar = h_static / R_radius;
    [c_para, c_perp] = calc_correction_functions(max(h_bar, 1.001));
    a_axes_static = [a_nom/c_para; a_nom/c_para; a_nom/c_perp];

    for ip = 1:n_apd
        a_pd = a_pd_list(ip);
        alpha = 1 - a_pd;
        beta  = lambda_c;
        c     = 1 - beta;

        % R_T(τ), R_N(τ) closed-form
        R_T = zeros(tau_max+2, 1);
        for tau = 0:tau_max+1
            T1 = alpha^(tau+2)/(1-alpha^2);
            if tau <= 3
                T2 = alpha^(5-tau)*c/((1-alpha^2)*(1-alpha*beta));
            else
                T2 = (alpha^2*c)/(alpha-beta) * ...
                     (alpha^(tau-2)/(1-alpha^2) - beta^(tau-2)/(1-alpha*beta));
            end
            T3 = alpha^(tau+5)*c/((1-alpha^2)*(1-alpha*beta));
            R_g = 1/(alpha-beta)^2 * ( ...
                    alpha^(tau+2)/(1-alpha^2) + beta^(tau+2)/(1-beta^2) ...
                  - alpha*beta*(alpha^tau+beta^tau)/(1-alpha*beta));
            T4 = alpha^2*c^2*R_g;
            R_T(tau+1) = T1 - T2 - T3 + T4;
        end
        R_N = zeros(tau_max+1, 1);
        for tau = 0:tau_max
            if tau == 0
                R_N(1) = 2*R_T(1) - 2*R_T(2);
            else
                R_N(tau+1) = 2*R_T(tau+1) - R_T(tau) - R_T(tau+2);
            end
        end
        R_T = R_T(1:tau_max+1);
        C_dpmr_cf = R_T(1);
        C_n_cf    = R_N(1);
        rho_T_cf  = R_T / C_dpmr_cf;
        rho_N_cf  = R_N / C_n_cf;

        % Per-axis weights and IF_eff
        for ax = 1:3
            sig_T = C_dpmr_cf * 4*kBT * a_axes_static(ax);
            sig_n = C_n_cf * sigma2_n_axes(ax);
            sig_tot = sig_T + sig_n;
            w_T = sig_T/sig_tot; w_n = sig_n/sig_tot;
            rho_mix = w_T*rho_T_cf + w_n*rho_N_cf;
            IF_eff_cf_grid(ih, ip, ax) = 1 + 2*sum(rho_mix(2:end).^2 .* s_ewma.^(1:tau_max)');
            sigma2_th_grid(ih, ip, ax) = sig_tot;
        end
    end
end

% OptA reference IF_eff (depends on a_cov, not on a_pd)
denom_e = 1 + 2*(1-lambda_c)^2;
rho_e_1 = (1-lambda_c)*(2-lambda_c)/denom_e;
rho_e_2 = (1-lambda_c)/denom_e;
Var_dx_over_sig_e = (1 + 2*lambda_c*rho_e_1 + 2*lambda_c^2*rho_e_2)/(1-lambda_c^2);
inv_var_ratio = 1/Var_dx_over_sig_e;
rho_dx_1_OA = lambda_c + inv_var_ratio*(rho_e_1 + lambda_c*rho_e_2);
rho_dx_2_OA = lambda_c*rho_dx_1_OA + inv_var_ratio*rho_e_2;
IF_eff_OptA = 1 + 2*(rho_dx_1_OA^2*s_ewma + rho_dx_2_OA^2*s_ewma^2/(1-lambda_c^2*s_ewma));

% Per-cell R22 emp from sim
for ih = 1:n_h
    for ip = 1:n_apd
        a_pd = a_pd_list(ip);
        for is = 1:n_seed
            % Recompute sigma2_ctrl using a_pd for this cell
            % Sigma2_ctrl_grid was generated under THIS a_pd.
            sigma2_ctrl = sigma2_ctrl_grid{ih, ip, is};
            for ax = 1:3
                y = sigma2_ctrl(warmup_idx, ax);
                t = tout_ref(warmup_idx);
                p = polyfit(t, y, 1);
                y_resid = y - polyval(p, t);
                R22_emp = var(y_resid);
                sigma2_mean_grid(ih, ip, is, ax) = mean(y);

                % Use TRUE σ²_dxr_theory based on observed mean (more accurate)
                sigma2_mean_obs = mean(y);
                R22_th_OptA = R22_prefactor * IF_eff_OptA * sigma2_mean_obs^2;
                R22_th_new  = R22_prefactor * IF_eff_cf_grid(ih, ip, ax) * sigma2_mean_obs^2;
                ratio_OptA_grid(ih, ip, is, ax) = R22_emp / R22_th_OptA;
                ratio_new_grid(ih, ip, is, ax)  = R22_emp / R22_th_new;

                % Empirical IF_eff
                % Compute ρ̂ from δp_mr (sandbox IIR with this a_pd)
                delta_x_m = delta_x_m_grid{ih, ip, is};
                dx = delta_x_m(:, ax);
                dpmd_prev = 0; dpmr = zeros(N_t, 1);
                for k = 1:N_t
                    dpmd = (1-a_pd)*dpmd_prev + a_pd*dx(k);
                    dpmr(k) = dx(k) - dpmd;
                    dpmd_prev = dpmd;
                end
                y_pmr = dpmr(warmup_idx);
                N_y = length(y_pmr);
                rho_hat = zeros(tau_max+1, 1);
                for tau = 0:tau_max
                    rho_hat(tau+1) = mean(y_pmr(1:N_y-tau) .* y_pmr(1+tau:N_y));
                end
                rho_hat = rho_hat / rho_hat(1);
                IF_eff_emp_grid(ih, ip, is, ax) = 1 + 2*sum(rho_hat(2:end).^2 .* s_ewma.^(1:tau_max)');
            end
        end
    end
end

% Aggregate across seeds → mean / std per (h, a_pd, axis)
ratio_new_mean = mean(ratio_new_grid, 3);     % [h, a_pd, axis]
ratio_new_std  = std(ratio_new_grid, [], 3);
ratio_OptA_mean = mean(ratio_OptA_grid, 3);
IF_eff_emp_mean = mean(IF_eff_emp_grid, 3);
IF_eff_emp_std  = std(IF_eff_emp_grid, [], 3);

%% Print summary
fprintf('\n========= R22 ratio (new closed-form, mean ± std across seeds) =========\n');
for ih = 1:n_h
    fprintf('\nh = %d um:\n', h_list(ih));
    fprintf('  a_pd  |   X axis        |   Y axis        |   Z axis\n');
    fprintf('--------|-----------------|-----------------|----------------\n');
    for ip = 1:n_apd
        fprintf(' %.4f | %.3f ± %.3f   | %.3f ± %.3f   | %.3f ± %.3f\n', ...
                a_pd_list(ip), ...
                ratio_new_mean(ih, ip, 1), ratio_new_std(ih, ip, 1), ...
                ratio_new_mean(ih, ip, 2), ratio_new_std(ih, ip, 2), ...
                ratio_new_mean(ih, ip, 3), ratio_new_std(ih, ip, 3));
    end
end

fprintf('\n========= IF_eff: closed-form vs empirical (Z axis) =========\n');
fprintf('  h  | a_pd   | IF_eff_cf | IF_eff_emp(mean±std) | diff%%\n');
fprintf('-----|--------|-----------|----------------------|------\n');
for ih = 1:n_h
    for ip = 1:n_apd
        cf_z = IF_eff_cf_grid(ih, ip, 3);
        em_z = IF_eff_emp_mean(ih, ip, 3);
        sd_z = IF_eff_emp_std(ih, ip, 3);
        fprintf(' %3d | %.4f |  %.4f   |   %.4f ± %.4f      | %+.2f%%\n', ...
                h_list(ih), a_pd_list(ip), cf_z, em_z, sd_z, 100*(cf_z/em_z - 1));
    end
end

fprintf('\n========= R22 ratio (OptA) — for comparison =========\n');
for ih = 1:n_h
    fprintf('h=%-3d: ', h_list(ih));
    for ip = 1:n_apd
        fprintf(' a_pd=%.3f X=%.2f Y=%.2f Z=%.2f |', ...
                a_pd_list(ip), ratio_OptA_mean(ih, ip, 1), ...
                ratio_OptA_mean(ih, ip, 2), ratio_OptA_mean(ih, ip, 3));
    end
    fprintf('\n');
end

%% Save aggregate stats
save(fullfile(save_dir, 'verify_positioning_grid.mat'), ...
     'h_list', 'a_pd_list', 'seed_list', 'sigma_n_axes', ...
     'ratio_new_mean', 'ratio_new_std', 'ratio_OptA_mean', ...
     'IF_eff_cf_grid', 'IF_eff_emp_mean', 'IF_eff_emp_std');

%% Plot
set(groot, 'defaultTextInterpreter','tex');
set(groot, 'defaultLegendInterpreter','tex');
set(groot, 'defaultAxesTickLabelInterpreter','tex');

COL_AX  = [0.85 0.20 0.20; 0.20 0.65 0.30; 0.20 0.30 0.85];
COL_APD = [0.0 0.4 0.0; 0.5 0.2 0.7; 0.9 0.4 0.0];

fig = figure('Position', [50 50 1700 950], 'Color', 'w');
tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

% (1) R22 ratio vs h, Z axis, lines for each a_pd
nexttile;
for ip = 1:n_apd
    errorbar(h_list, squeeze(ratio_new_mean(:, ip, 3)), ...
             squeeze(ratio_new_std(:, ip, 3)), 'o-', ...
             'Color', COL_APD(ip,:), 'LineWidth', 2, 'MarkerSize', 9, ...
             'MarkerFaceColor', COL_APD(ip,:), ...
             'DisplayName', sprintf('a\\_pd=%.3f', a_pd_list(ip))); hold on;
end
yline(1.0, 'k--', 'LineWidth', 2);
yline(1.05, 'k:', 'LineWidth', 1); yline(0.95, 'k:', 'LineWidth', 1);
set(gca, 'FontSize', 15, 'FontWeight', 'bold', 'LineWidth', 2);
xlabel('h (um)', 'FontSize', 17);
ylabel('R_{22} ratio (new closed-form)', 'FontSize', 17);
title('(1) R_{22} ratio vs h, Z axis (new closed-form)', 'FontSize', 16);
legend('show', 'FontSize', 13, 'Location', 'best');
grid on; box on;
ylim([0.7 1.2]);

% (2) Same but X axis
nexttile;
for ip = 1:n_apd
    errorbar(h_list, squeeze(ratio_new_mean(:, ip, 1)), ...
             squeeze(ratio_new_std(:, ip, 1)), 's-', ...
             'Color', COL_APD(ip,:), 'LineWidth', 2, 'MarkerSize', 9, ...
             'MarkerFaceColor', COL_APD(ip,:), ...
             'DisplayName', sprintf('a\\_pd=%.3f', a_pd_list(ip))); hold on;
end
yline(1.0, 'k--', 'LineWidth', 2);
yline(1.05, 'k:', 'LineWidth', 1); yline(0.95, 'k:', 'LineWidth', 1);
set(gca, 'FontSize', 15, 'FontWeight', 'bold', 'LineWidth', 2);
xlabel('h (um)', 'FontSize', 17);
ylabel('R_{22} ratio (new closed-form)', 'FontSize', 17);
title('(2) R_{22} ratio vs h, X axis', 'FontSize', 16);
legend('show', 'FontSize', 13, 'Location', 'best');
grid on; box on;
ylim([0.7 1.2]);

% (3) IF_eff_cf vs IF_eff_emp scatter
nexttile;
for ih = 1:n_h
    for ip = 1:n_apd
        for ax = 1:3
            x = IF_eff_cf_grid(ih, ip, ax);
            y = IF_eff_emp_mean(ih, ip, ax);
            sd = IF_eff_emp_std(ih, ip, ax);
            errorbar(x, y, sd, 'o', 'Color', COL_AX(ax,:), ...
                     'MarkerFaceColor', COL_AX(ax,:), 'MarkerSize', 8, ...
                     'LineWidth', 1.5, 'HandleVisibility', 'off'); hold on;
        end
    end
end
% legend axes
for ax = 1:3
    plot(NaN, NaN, 'o', 'Color', COL_AX(ax,:), 'MarkerFaceColor', COL_AX(ax,:), ...
         'MarkerSize', 12, 'DisplayName', sprintf('axis %s', ax_names{ax}));
end
% Y=x reference
xline_min = 0.5; xline_max = 4;
plot([xline_min xline_max], [xline_min xline_max], 'k--', 'LineWidth', 2);
plot([xline_min xline_max], [xline_min xline_max]*1.05, 'k:', 'LineWidth', 1);
plot([xline_min xline_max], [xline_min xline_max]/1.05, 'k:', 'LineWidth', 1);
set(gca, 'FontSize', 15, 'FontWeight', 'bold', 'LineWidth', 2);
xlabel('IF_{eff} closed-form', 'FontSize', 17);
ylabel('IF_{eff} empirical', 'FontSize', 17);
title('(3) IF_{eff} closed-form vs empirical (all cells)', 'FontSize', 16);
legend('show', 'FontSize', 13, 'Location', 'best');
grid on; box on; axis equal;
xlim([xline_min xline_max]); ylim([xline_min xline_max]);

% (4) ratio distribution: box plot across all axes/cells
nexttile;
all_ratios_new = ratio_new_grid(:);
all_ratios_OptA = ratio_OptA_grid(:);
boxplot([all_ratios_OptA, all_ratios_new], 'Labels', {'OptA', 'new closed-form'}, ...
        'Colors', [0.8 0 0; 0 0.55 0], 'symbol', 'k+');
yline(1.0, 'k--', 'LineWidth', 2);
yline(1.05, 'k:', 'LineWidth', 1); yline(0.95, 'k:', 'LineWidth', 1);
set(gca, 'FontSize', 15, 'FontWeight', 'bold', 'LineWidth', 2);
ylabel('R_{22} ratio', 'FontSize', 17);
title(sprintf('(4) Distribution across grid (%d cells)', length(all_ratios_new)), ...
      'FontSize', 16);
grid on; box on;
ylim([0.5 1.2]);

sgtitle(sprintf('Positioning grid verification: 3 h × 3 a\\_pd × 3 axes × 3 seeds = %d cells', ...
                length(all_ratios_new)), 'FontSize', 17, 'FontWeight', 'bold');

out_path = fullfile(save_dir, 'verify_positioning_grid.png');
exportgraphics(fig, out_path, 'Resolution', 150);
fprintf('\nSaved: %s\n', out_path);

set(groot, 'defaultTextInterpreter','remove');
set(groot, 'defaultLegendInterpreter','remove');
set(groot, 'defaultAxesTickLabelInterpreter','remove');
