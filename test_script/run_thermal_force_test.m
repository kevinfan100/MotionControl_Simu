%% run_thermal_force_test.m - Thermal Force Module Validation Test
%
% This script validates the Thermal Force module by:
% 1. Verifying the distribution matches theoretical Gaussian
% 2. Comparing variance at different distances from wall
%
% Output: 2 PNG files
%   1. distribution_histogram.png - F_x, F_y, F_z histograms with theory
%   2. position_comparison.png - Std comparison at h/R=10 vs h/R=1.5

clear; close all; clc;

%% Add paths (relative to project root)
[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
project_root = fileparts(script_dir);
addpath(fullfile(project_root, 'model', 'wall_effect'));
addpath(fullfile(project_root, 'model', 'thermal_force'));
addpath(fullfile(project_root, 'model', 'trajectory'));
cd(project_root);

% === RNG setting ===
% Uncomment the following line to reproduce test results
% rng(42);

%% Test parameters
N = 10000;           % Number of samples
bins = 50;           % Histogram bins
h_bar_far = 10;      % Far from wall
h_bar_near = 1.5;    % Close to wall (safety boundary)

% Get default params
params_slx = calc_simulation_params();
params = params_slx.Value;

%% Figure style settings
axis_linewidth = 1.5;
xlabel_fontsize = 14;
ylabel_fontsize = 14;
title_fontsize = 16;
tick_fontsize = 12;
legend_fontsize = 11;
line_width = 2;

%% Create output directory
timestamp = datestr(now, 'yyyymmdd_HHMMSS');
output_dir = fullfile('test_results', 'thermal_force', ['test_' timestamp]);
mkdir(output_dir);

%% Test 1: Distribution at h_bar = h_bar_far
% Position at h_bar_far
p_test = h_bar_far * params.common.R * params.wall.w_hat;

% Generate N samples
F_th_samples = zeros(3, N);
for i = 1:N
    F_th_samples(:, i) = calc_thermal_force(p_test, params);
end

% Calculate measured statistics
mean_measured = mean(F_th_samples, 2);
std_measured = std(F_th_samples, 0, 2);

% Calculate theoretical values
[c_para, c_perp] = calc_correction_functions(h_bar_far);
C = c_para * (params.wall.u_hat + params.wall.v_hat) + c_perp * params.wall.w_hat;
variance_coeff = params.thermal.variance_coeff;
std_theory = sqrt(variance_coeff * (C.^2));

%% Figure 1: Distribution Histogram
fig1 = figure('Position', [100 100 1200 400], 'Visible', 'off');

labels = {'F_x', 'F_y', 'F_z'};
colors = {[0.2 0.4 0.8], [0.2 0.6 0.4], [0.8 0.3 0.3]};

for k = 1:3
    subplot(1, 3, k);

    data = F_th_samples(k, :);

    % Histogram (normalized as PDF)
    histogram(data, bins, 'Normalization', 'pdf', ...
        'FaceColor', colors{k}, 'EdgeColor', 'white', 'FaceAlpha', 0.7);
    hold on;

    % Theoretical Gaussian curve
    x_range = linspace(min(data)-3*std(data), max(data)+3*std(data), 200);
    y_theory = normpdf(x_range, 0, std_theory(k));
    plot(x_range, y_theory, 'r-', 'LineWidth', line_width);

    hold off;

    xlabel([labels{k} ' [pN]'], 'FontSize', xlabel_fontsize);
    if k == 1
        ylabel('Probability density', 'FontSize', ylabel_fontsize);
    end
    title(sprintf('%s Distribution (h/R=%.0f)', labels{k}, h_bar_far), 'FontSize', title_fontsize);
    legend({'Measured', 'Theory'}, 'Location', 'northeast', 'FontSize', legend_fontsize);
    grid on;
    set(gca, 'FontSize', tick_fontsize, 'LineWidth', axis_linewidth);

    % Add text box with statistics
    text_str = sprintf('Theory: mean=0, std=%.2f\nMeasured: mean=%.3f, std=%.2f', ...
        std_theory(k), mean_measured(k), std_measured(k));
    text(0.05, 0.95, text_str, 'Units', 'normalized', 'VerticalAlignment', 'top', ...
        'FontSize', 10, 'BackgroundColor', 'white', 'EdgeColor', 'black');
end

exportgraphics(fig1, fullfile(output_dir, 'distribution_histogram.png'), 'Resolution', 150);
close(fig1);

%% Test 2: Position Comparison
% Generate samples at h_bar_near
p_near = h_bar_near * params.common.R * params.wall.w_hat;
F_th_near = zeros(3, N);
for i = 1:N
    F_th_near(:, i) = calc_thermal_force(p_near, params);
end
std_near = std(F_th_near, 0, 2);

% Generate samples at h_bar_far (reuse from above)
std_far = std_measured;

% Calculate theoretical values for h_bar_near
[c_para_near, c_perp_near] = calc_correction_functions(h_bar_near);
C_near = c_para_near * (params.wall.u_hat + params.wall.v_hat) + c_perp_near * params.wall.w_hat;
std_theory_near = sqrt(variance_coeff * (C_near.^2));

% Calculate theoretical values for h_bar_far (already have std_theory)

%% Figure 2: Position Comparison (Grouped Bar Chart)
fig2 = figure('Position', [100 100 800 500], 'Visible', 'off');

% Data for bar chart
bar_data = [std_far'; std_near'];  % 2x3 matrix

% Create grouped bar chart
b = bar(bar_data, 'grouped');
b(1).FaceColor = [0.2 0.4 0.8];  % Blue for x
b(2).FaceColor = [0.2 0.6 0.4];  % Green for y
b(3).FaceColor = [0.8 0.3 0.3];  % Red for z

% Add value labels on bars
for i = 1:2
    for j = 1:3
        text(b(j).XEndPoints(i), b(j).YEndPoints(i) + 0.05, ...
            sprintf('%.2f', bar_data(i,j)), ...
            'HorizontalAlignment', 'center', 'FontSize', 10);
    end
end

% Labels and title
set(gca, 'XTickLabel', {sprintf('h/R=%.0f (far)', h_bar_far), sprintf('h/R=%.1f (near)', h_bar_near)});
xlabel('Distance from wall', 'FontSize', xlabel_fontsize);
ylabel('Standard deviation [pN]', 'FontSize', ylabel_fontsize);
title('Thermal Force Std vs Distance from Wall', 'FontSize', title_fontsize);
legend({'sigma_x', 'sigma_y', 'sigma_z'}, 'Location', 'northwest', 'FontSize', legend_fontsize);
grid on;
set(gca, 'FontSize', tick_fontsize, 'LineWidth', axis_linewidth);

% Add ratio text box
ratio_x = std_near(1) / std_far(1);
ratio_y = std_near(2) / std_far(2);
ratio_z = std_near(3) / std_far(3);
text_str = sprintf('Ratio (near/far):\n  sigma_x: %.2fx\n  sigma_y: %.2fx\n  sigma_z: %.2fx', ...
    ratio_x, ratio_y, ratio_z);
text(0.98, 0.95, text_str, 'Units', 'normalized', 'VerticalAlignment', 'top', ...
    'HorizontalAlignment', 'right', 'FontSize', 10, 'BackgroundColor', 'white', 'EdgeColor', 'black');

exportgraphics(fig2, fullfile(output_dir, 'position_comparison.png'), 'Resolution', 150);
close(fig2);

%% Save result data
result.N = N;
result.h_bar_test = [h_bar_far, h_bar_near];
result.std_far = std_far;
result.std_near = std_near;
result.std_theory_far = std_theory;
result.std_theory_near = std_theory_near;
result.mean_measured = mean_measured;
result.params = params;

save(fullfile(output_dir, 'result.mat'), 'result');

%% Print summary
fprintf('\n=== Thermal Force Test Summary ===\n');
fprintf('Number of samples: N = %d\n', N);

fprintf('\nAt h/R = %.0f (far from wall):\n', h_bar_far);
fprintf('  Measured std: [%.3f, %.3f, %.3f] pN\n', std_far);
fprintf('  Theory std:   [%.3f, %.3f, %.3f] pN\n', std_theory);
fprintf('  Error:        [%.1f%%, %.1f%%, %.1f%%]\n', ...
    100*abs(std_far - std_theory)./std_theory);

fprintf('\nAt h/R = %.1f (near wall):\n', h_bar_near);
fprintf('  Measured std: [%.3f, %.3f, %.3f] pN\n', std_near);
fprintf('  Theory std:   [%.3f, %.3f, %.3f] pN\n', std_theory_near);

fprintf('\nRatio (near/far):\n');
fprintf('  sigma_x: %.2fx\n', ratio_x);
fprintf('  sigma_y: %.2fx\n', ratio_y);
fprintf('  sigma_z: %.2fx (perpendicular to wall)\n', ratio_z);

fprintf('\nResults saved to: %s\n', output_dir);
