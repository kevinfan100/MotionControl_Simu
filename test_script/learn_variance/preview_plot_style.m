% preview_plot_style.m  (v2)
% Quick preview of proposed plot style — iteration after first feedback.
% Changes: LaTeX interpreter throughout; thicker/saturated green theory line;
% B3 split into top/bottom dual panel.

clear; close all; clc;

[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
project_root = fileparts(fileparts(script_dir));
cd(project_root);
addpath(fullfile(project_root, 'model'));
addpath(fullfile(project_root, 'model', 'config'));
addpath(fullfile(project_root, 'model', 'wall_effect'));

%% ============== Load data ==============
S = load(fullfile(project_root, 'test_results', 'learn_variance', ...
                  'ramp_50to5_T20_delpm.mat'));
delta_z = S.delta_x_m(:, 3);
tout    = S.tout;
config  = S.config;
N       = length(tout);
Ts      = tout(2) - tout(1);

%% ============== Constants ==============
constants  = physical_constants();
kBT        = constants.k_B * constants.T;
gamma_N    = constants.gamma_N;
R_radius   = constants.R;
a_nom      = Ts / gamma_N;
lambda_c   = config.lambda_c;
sigma2_n_z = config.meas_noise_std(3)^2;

a_pd  = config.a_pd;
a_prd = config.a_prd;
a_cov = config.a_cov;

C_dx = 2*(1-a_pd)*(1-lambda_c) / (1-(1-a_pd)*lambda_c) ...
     + (2/(2-a_pd)) / ((1+lambda_c)*(1-(1-a_pd)*lambda_c));
C_n  = 2 / (1 + lambda_c);

%% ============== Theory curve Var_theory(t) ==============
h_init   = config.h_init;
h_bottom = config.h_bottom;
rate     = (h_init - h_bottom) / config.T_sim;
h_t      = max(h_init - rate * tout, h_bottom);
h_bar_t  = h_t / R_radius;

a_z_t    = zeros(N, 1);
for k = 1:N
    [~, c_perp] = calc_correction_functions(max(h_bar_t(k), 1.001));
    a_z_t(k) = a_nom / c_perp;
end
Var_theory_t = C_dx * 4 * kBT * a_z_t + C_n * sigma2_n_z;

%% ============== Inline IIR chain ==============
del_pmd_init  = 0;
del_pmrd_init = 0;
del_pmr2_init = C_dx * 4 * kBT * a_z_t(1) + C_n * sigma2_n_z;

del_pmd_z      = zeros(N, 1);
del_pmr_z      = zeros(N, 1);
del_pmrd_z     = zeros(N, 1);
del_pmr2_avg_z = zeros(N, 1);
del_pmr_var_z  = zeros(N, 1);

del_pmd_prev   = del_pmd_init;
del_pmrd_prev  = del_pmrd_init;
del_pmr2_prev  = del_pmr2_init;

for k = 1:N
    dpmd  = (1-a_pd)  * del_pmd_prev  + a_pd  * delta_z(k);
    dpmr  = delta_z(k) - dpmd;
    dpmrd = (1-a_prd) * del_pmrd_prev + a_prd * dpmr;
    dpmr2 = (1-a_cov) * del_pmr2_prev + a_cov * dpmr^2;
    dvar  = max(dpmr2 - dpmrd^2, 0);

    del_pmd_z(k)      = dpmd;
    del_pmr_z(k)      = dpmr;
    del_pmrd_z(k)     = dpmrd;
    del_pmr2_avg_z(k) = dpmr2;
    del_pmr_var_z(k)  = dvar;

    del_pmd_prev  = dpmd;
    del_pmrd_prev = dpmrd;
    del_pmr2_prev = dpmr2;
end

%% ============== Style constants (v2 adjustments) ==============
COL_REF    = [0.00 0.65 0.00];        % green - saturated
COL_OUT    = [0.85 0.00 0.00];        % red - estimated/output
COL_ERR    = [0.00 0.20 0.80];        % blue - intermediate
COL_INPUT  = [0.00 0.00 0.00];        % black - input
COL_AUX    = [0.45 0.45 0.45];        % gray - auxiliary

AXIS_LW    = 2.0;
FONT_SIZE  = 18;
TITLE_FS   = 16;
LEGEND_FS  = 14;
LW_PRIMARY = 2.0;
LW_THEORY  = 3.0;     % was 1.8 — thicker
LW_AUX     = 1.2;
LW_LIGHT   = 0.5;

% Default LaTeX interpreter for everything
set(groot, 'defaultTextInterpreter',           'latex');
set(groot, 'defaultLegendInterpreter',         'latex');
set(groot, 'defaultAxesTickLabelInterpreter',  'latex');

save_dir = fullfile(project_root, 'test_results', 'learn_variance');
if ~exist(save_dir, 'dir'); mkdir(save_dir); end

annot_str = sprintf('ramp $50\\rightarrow 5$, $T_{sim}=20$ s, seed$=42$, $\\lambda_c=0.7$');

%% ============== Figure B3: Layer-3 view (dual panel, linear y) ==============
fig_b3 = figure('Position', [50 100 1100 700], 'Color', 'w');
tl = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

% --- Top: instantaneous del_pmr^2 ---
nexttile;
plot(tout, del_pmr_z.^2, 'Color', COL_AUX, 'LineWidth', LW_LIGHT);
set(gca, 'FontSize', FONT_SIZE, 'FontWeight', 'bold', 'LineWidth', AXIS_LW);
ylabel('$\delta x_{mr}^2$ [um$^2$]', 'FontSize', FONT_SIZE);
title('B3 (top): instantaneous squared HP residual', 'FontSize', TITLE_FS);
grid on; box on; xlim([0 config.T_sim]);
set(gca, 'XTickLabel', []);

% --- Bottom: EWMA del_pmr2_avg ---
nexttile;
plot(tout, del_pmr2_avg_z, 'Color', COL_OUT, 'LineWidth', LW_PRIMARY);
set(gca, 'FontSize', FONT_SIZE, 'FontWeight', 'bold', 'LineWidth', AXIS_LW);
xlabel('Time [sec]', 'FontSize', FONT_SIZE);
ylabel('$\overline{\delta x_{mr}^2}$ [um$^2$]', 'FontSize', FONT_SIZE);
title('B3 (bottom): EWMA smoothed (\texttt{a\_cov}$=0.05$)', 'FontSize', TITLE_FS);
grid on; box on; xlim([0 config.T_sim]);

text(0.02, -0.30, annot_str, 'Units', 'normalized', 'FontSize', 11, ...
     'Color', COL_AUX);

exportgraphics(fig_b3, fullfile(save_dir, 'preview_B3_style.png'), 'Resolution', 150);

%% ============== Figure A6: Final variance estimate vs theory ==============
fig_a6 = figure('Position', [200 200 1100 600], 'Color', 'w');

yyaxis left;
p1 = plot(tout, del_pmr_var_z, 'Color', COL_OUT, 'LineWidth', LW_PRIMARY, ...
          'LineStyle', '-');
hold on;
p2 = plot(tout, Var_theory_t,  'Color', COL_REF, 'LineWidth', LW_THEORY, ...
          'LineStyle', '--');
ylabel('$\mathrm{Var}(\delta x_r)$ [um$^2$]', 'FontSize', FONT_SIZE);
ax_left = gca;
set(ax_left, 'YColor', 'k');

yyaxis right;
p3 = plot(tout, a_z_t, 'Color', COL_AUX, 'LineWidth', LW_AUX, 'LineStyle', ':');
ylabel('$a_z$ [um/pN]', 'FontSize', FONT_SIZE, 'Color', COL_AUX);
set(gca, 'YColor', COL_AUX);

set(gca, 'FontSize', FONT_SIZE, 'FontWeight', 'bold', 'LineWidth', AXIS_LW);
xlabel('Time [sec]', 'FontSize', FONT_SIZE);
title('A6: Final variance estimate vs theory --- Z axis', 'FontSize', TITLE_FS);
legend([p1 p2 p3], ...
       {'$\widehat{\sigma^2_{\delta x_r}}$ (IIR estimate)', ...
        '$\sigma^2_{\delta x_r}$ theory (Eq.12)', ...
        '$a_z(t)$'}, ...
       'Location', 'northeast', 'FontSize', LEGEND_FS, 'Box', 'off');
grid on; box on; xlim([0 config.T_sim]);

text(0.02, -0.16, annot_str, 'Units', 'normalized', 'FontSize', 11, ...
     'Color', COL_AUX);

exportgraphics(fig_a6, fullfile(save_dir, 'preview_A6_style.png'), 'Resolution', 150);

% Reset interpreter defaults to factory
set(groot, 'defaultTextInterpreter',          'remove');
set(groot, 'defaultLegendInterpreter',        'remove');
set(groot, 'defaultAxesTickLabelInterpreter', 'remove');

fprintf('\nv2 preview saved:\n');
fprintf('  %s\n', fullfile(save_dir, 'preview_B3_style.png'));
fprintf('  %s\n', fullfile(save_dir, 'preview_A6_style.png'));
