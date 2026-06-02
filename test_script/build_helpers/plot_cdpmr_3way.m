% plot_cdpmr_3way.m
% Plot three C_dpmr versions vs λ_c:
%   (1) Paper baseline:  C_dpmr = 2 + 1/(1−λc²)
%   (2) Closed-form:     Parseval residue (a_pd dependent)
%   (3) Lyapunov 11-state: paper + HP1 + KF estimator errors
%
% Style matches test/eq17-5state-ekf:figures/theory/cdpmr_3way_alpha005.png

clear; close all; clc;
[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
addpath(script_dir);

%% --- Setup ---
a_pd        = 0.05;
d_delay     = 2;
lambda_grid = linspace(0.40, 0.90, 41);

% Q/R from Positioning_kf_a_merged.pdf §4 (Eq.23, Eq.24)
% State order: [delta_x_1, delta_x_2, delta_x_3, x_D, delta_x_D, a_x, delta_a_x]
Q_kf_scale = [0, 0, 1e4, 1e-1, 0, 1e-4, 0]';
R_kf_scale = [1e-2, 1]';

opts = struct('f0', 0, 'verbose', false, 'Fe_form', 'eq19');

%% --- Sweep ---
C_paper     = zeros(size(lambda_grid));
C_closed    = zeros(size(lambda_grid));
C_lyapunov  = zeros(size(lambda_grid));

for i = 1:length(lambda_grid)
    lc = lambda_grid(i);

    % (1) Paper baseline
    C_paper(i) = 2 + 1/(1 - lc^2);

    % (2) Closed-form Parseval (from Cdpmr_Cn_derivation.tex Eq.15)
    D = 1 - (1-a_pd)*lc;
    term1 = 2 * (1-a_pd) * (1-lc) / D;
    term2 = (2/(2-a_pd)) * 1 / ((1+lc) * D);
    C_closed(i) = (1-a_pd)^2 * (term1 + term2);

    % (3) Lyapunov 11-state
    try
        C_lyapunov(i) = compute_7state_cdpmr_eff_v2( ...
            lc, d_delay, a_pd, Q_kf_scale, R_kf_scale, opts);
    catch ME
        warning('Lyapunov failed at lc=%.3f: %s', lc, ME.message);
        C_lyapunov(i) = NaN;
    end
end

%% --- Plot (mirror existing cdpmr_3way style) ---
fig = figure('Position', [100 100 1100 700], 'Color', 'w');
ax = axes(fig);
hold(ax, 'on');

COL_PAPER    = [0.00 0.45 0.74];   % blue
COL_LYAP     = [0.85 0.10 0.10];   % red
COL_CLOSED   = [0.10 0.65 0.20];   % green

p1 = plot(lambda_grid, C_paper,    '-',  'Color', COL_PAPER,  'LineWidth', 4, ...
          'DisplayName', 'paper 1');
p2 = plot(lambda_grid, C_lyapunov, '-',  'Color', COL_LYAP,   'LineWidth', 4, ...
          'DisplayName', 'paper 3');
p3 = plot(lambda_grid, C_closed,   '-',  'Color', COL_CLOSED, 'LineWidth', 4, ...
          'DisplayName', 'paper 1 with KF');

set(ax, 'FontSize', 22, 'FontWeight', 'bold', 'LineWidth', 2);
set(ax, 'TickLabelInterpreter', 'tex');
xlabel(ax, '\lambda_c', 'FontSize', 28, 'FontWeight', 'bold');
ylabel(ax, 'C_{dpmr}', 'FontSize', 28, 'FontWeight', 'bold');
xlim(ax, [0.40 0.90]);
ylim(ax, [2 8]);

leg = legend([p1 p2 p3], 'Location', 'northoutside', ...
             'Orientation', 'horizontal', ...
             'FontSize', 20, 'FontWeight', 'bold', ...
             'Box', 'on', 'EdgeColor', 'k', 'LineWidth', 1.5);

box(ax, 'on');
grid(ax, 'off');

%% --- Save ---
% script_dir = <repo>/test_script/build_helpers -> repo root is two levels up
repo_root = fileparts(fileparts(script_dir));
out_dir   = fullfile(repo_root, 'reference', 'eq17_analysis', 'figures', 'theory');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end
out_path = fullfile(out_dir, 'cdpmr_3way_apd005.png');
exportgraphics(fig, out_path, 'Resolution', 150);
fprintf('Saved: %s\n', out_path);

%% --- Numerical summary at λc = 0.7 ---
idx_07 = find(abs(lambda_grid - 0.70) < 1e-6, 1);
if ~isempty(idx_07)
    fprintf('\n=== At λc = 0.7 ===\n');
    fprintf('  Paper baseline    : %.4f\n', C_paper(idx_07));
    fprintf('  Lyapunov 11-state : %.4f\n', C_lyapunov(idx_07));
    fprintf('  Closed-form       : %.4f\n', C_closed(idx_07));
end
