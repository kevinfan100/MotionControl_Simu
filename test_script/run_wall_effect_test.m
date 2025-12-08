%% run_wall_effect_test.m - Wall Effect Module Validation Test
%
% This script validates the Wall Effect module by generating visualization
% plots that show the behavior of correction functions and mobility.
%
% Output: 3 PNG files
%   1. correction_functions.png - c_para, c_perp vs h_bar
%   2. effective_drag.png - gamma_para, gamma_perp vs h_bar
%   3. mobility.png - 1/gamma_para, 1/gamma_perp vs h_bar

clear; close all; clc;

%% Add paths (relative to project root)
[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
project_root = fileparts(script_dir);
addpath(fullfile(project_root, 'model', 'wall_effect'));
addpath(fullfile(project_root, 'model', 'trajectory'));
cd(project_root);

%% Test parameters
R = 2.25;             % Particle radius [um]
gamma_N = 0.0425;     % Stokes drag [pN*sec/um]

% Test range
h_bar_range = linspace(1.1, 20, 200);
h_bar_min = 1.5;  % Safety line

%% Figure style settings
axis_linewidth = 1.5;
xlabel_fontsize = 14;
ylabel_fontsize = 14;
title_fontsize = 16;
tick_fontsize = 12;
legend_fontsize = 11;
line_width = 2;

%% Calculate correction functions over range
N = length(h_bar_range);
c_para_vec = zeros(1, N);
c_perp_vec = zeros(1, N);

for i = 1:N
    [c_para_vec(i), c_perp_vec(i)] = calc_correction_functions(h_bar_range(i));
end

% Derived quantities
gamma_para_vec = gamma_N * c_para_vec;
gamma_perp_vec = gamma_N * c_perp_vec;
mobility_para_vec = 1 ./ gamma_para_vec;
mobility_perp_vec = 1 ./ gamma_perp_vec;

%% Create output directory
timestamp = datestr(now, 'yyyymmdd_HHMMSS');
output_dir = fullfile('test_results', 'wall_effect', ['test_' timestamp]);
mkdir(output_dir);

%% Figure 1: Correction Functions
fig1 = figure('Position', [100 100 800 500], 'Visible', 'off');
plot(h_bar_range, c_para_vec, 'b-', 'LineWidth', line_width);
hold on;
plot(h_bar_range, c_perp_vec, 'r-', 'LineWidth', line_width);
xline(h_bar_min, 'k--', 'LineWidth', 1.5);
yline(1, 'k:', 'LineWidth', 1);
hold off;

xlabel('Normalized distance h/R', 'FontSize', xlabel_fontsize);
ylabel('Correction coefficient', 'FontSize', ylabel_fontsize);
title('Wall Effect Correction Functions', 'FontSize', title_fontsize);
legend({'c_{parallel}', 'c_{perp}', 'h_{min}=1.5', 'c->1'}, ...
    'Location', 'northeast', 'FontSize', legend_fontsize);
grid on;
set(gca, 'FontSize', tick_fontsize, 'LineWidth', axis_linewidth);
xlim([1 20]);
ylim([0.9 max(c_perp_vec)*1.1]);

exportgraphics(fig1, fullfile(output_dir, 'correction_functions.png'), 'Resolution', 150);
close(fig1);

%% Figure 2: Effective Drag
fig2 = figure('Position', [100 100 800 500], 'Visible', 'off');
plot(h_bar_range, gamma_para_vec, 'b-', 'LineWidth', line_width);
hold on;
plot(h_bar_range, gamma_perp_vec, 'r-', 'LineWidth', line_width);
yline(gamma_N, 'k:', 'LineWidth', 1.5);
xline(h_bar_min, 'k--', 'LineWidth', 1.5);
hold off;

xlabel('Normalized distance h/R', 'FontSize', xlabel_fontsize);
ylabel('Effective drag [pN*sec/um]', 'FontSize', ylabel_fontsize);
title('Effective Drag vs Distance', 'FontSize', title_fontsize);
legend({'gamma_{parallel}', 'gamma_{perp}', sprintf('gamma_N=%.4f', gamma_N), 'h_{min}=1.5'}, ...
    'Location', 'northeast', 'FontSize', legend_fontsize);
grid on;
set(gca, 'FontSize', tick_fontsize, 'LineWidth', axis_linewidth);
xlim([1 20]);

exportgraphics(fig2, fullfile(output_dir, 'effective_drag.png'), 'Resolution', 150);
close(fig2);

%% Figure 3: Mobility
fig3 = figure('Position', [100 100 800 500], 'Visible', 'off');
plot(h_bar_range, mobility_para_vec, 'b-', 'LineWidth', line_width);
hold on;
plot(h_bar_range, mobility_perp_vec, 'r-', 'LineWidth', line_width);
yline(1/gamma_N, 'k:', 'LineWidth', 1.5);
xline(h_bar_min, 'k--', 'LineWidth', 1.5);
hold off;

xlabel('Normalized distance h/R', 'FontSize', xlabel_fontsize);
ylabel('Mobility [um/(pN*sec)]', 'FontSize', ylabel_fontsize);
title('Mobility vs Distance', 'FontSize', title_fontsize);
legend({'1/gamma_{parallel}', '1/gamma_{perp}', sprintf('1/gamma_N=%.2f', 1/gamma_N), 'h_{min}=1.5'}, ...
    'Location', 'southeast', 'FontSize', legend_fontsize);
grid on;
set(gca, 'FontSize', tick_fontsize, 'LineWidth', axis_linewidth);
xlim([1 20]);

exportgraphics(fig3, fullfile(output_dir, 'mobility.png'), 'Resolution', 150);
close(fig3);

%% Save result data
result.h_bar_range = h_bar_range;
result.c_para = c_para_vec;
result.c_perp = c_perp_vec;
result.gamma_para = gamma_para_vec;
result.gamma_perp = gamma_perp_vec;
result.mobility_para = mobility_para_vec;
result.mobility_perp = mobility_perp_vec;
result.params.R = R;
result.params.gamma_N = gamma_N;
result.params.h_bar_min = h_bar_min;

save(fullfile(output_dir, 'result.mat'), 'result');

%% Print summary
fprintf('\n=== Wall Effect Test Summary ===\n');
fprintf('Test range: h/R = %.1f to %.1f\n', min(h_bar_range), max(h_bar_range));

fprintf('\nAt h/R = %.1f (safety boundary):\n', h_bar_min);
[c_para_min, c_perp_min] = calc_correction_functions(h_bar_min);
fprintf('  c_para = %.4f, c_perp = %.4f\n', c_para_min, c_perp_min);
fprintf('  gamma_para = %.4f, gamma_perp = %.4f [pN*sec/um]\n', ...
    gamma_N*c_para_min, gamma_N*c_perp_min);

fprintf('\nAt h/R = 10 (far from wall):\n');
[c_para_far, c_perp_far] = calc_correction_functions(10);
fprintf('  c_para = %.4f, c_perp = %.4f\n', c_para_far, c_perp_far);
fprintf('  (Both approaching 1 as expected)\n');

fprintf('\nResults saved to: %s\n', output_dir);
