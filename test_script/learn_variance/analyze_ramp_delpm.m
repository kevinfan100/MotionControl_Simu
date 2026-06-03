% analyze_ramp_delpm.m
% Segmented verification of delta_x_m from ramp_descent 50->5, T_sim=20s.
% Compares to Meng 2024 (Near-Wall) Fig. 6 right panel:
%   - mean of tracking error should be close to 0 (ideal control)
%   - std should match theoretical prediction along the ramp
%
% Theoretical formula (paper Eq.12):
%   sigma2_dx = (2 + 1/(1-lc^2)) * 4*kBT*a_x  +  (2/(1+lc)) * sigma2_n
%   a_x       = Ts/gamma_N * (1/c)   c = c_para (X,Y) or c_perp (Z)

clear; close all; clc;

[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
project_root = fileparts(fileparts(script_dir));
cd(project_root);

addpath(fullfile(project_root, 'model'));
addpath(fullfile(project_root, 'model', 'config'));
addpath(fullfile(project_root, 'model', 'wall_effect'));

%% Load saved data
load_path = fullfile(project_root, 'test_results', 'learn_variance', ...
                     'ramp_50to5_T20_delpm.mat');
S = load(load_path);
delta_x_m = S.delta_x_m;     % [N x 3]
tout      = S.tout;
config    = S.config;

N = length(tout);
fs = 1/(tout(2)-tout(1));
fprintf('Loaded: N=%d, fs=%.0f Hz, T_sim=%.1fs\n', N, fs, tout(end));

%% Constants (must match user_config / calc_simulation_params)
constants  = physical_constants();
kBT        = constants.k_B * constants.T;
Ts         = constants.Ts;
gamma_N    = constants.gamma_N;
R_radius   = constants.R;
a_nom      = Ts / gamma_N;
lambda_c   = config.lambda_c;
sigma2_n_s = config.meas_noise_std(:).^2;   % 3x1 [um^2]

%% Trajectory: ramp_descent gives h(t) = max(h_init - rate*t, h_bottom)
h_init   = config.h_init;
h_bottom = config.h_bottom;
rate     = (h_init - h_bottom) / config.T_sim;
h_t      = max(h_init - rate * tout, h_bottom);
h_bar_t  = h_t / R_radius;

%% Define segments (10 windows of 2 sec each, span full T_sim)
n_seg = 10;
seg_edges = linspace(0, config.T_sim, n_seg+1);
seg_centers = (seg_edges(1:end-1) + seg_edges(2:end)) / 2;

%% Per-segment statistics (mean, std, theoretical std)
labels = {'X', 'Y', 'Z'};
seg_mean = zeros(n_seg, 3);
seg_std  = zeros(n_seg, 3);
seg_theory_std = zeros(n_seg, 3);
seg_h_mid = zeros(n_seg, 1);

% paper formula coefficients (production lambda_c=0.7)
C_dpmr = 2 + 1/(1 - lambda_c^2);     % 3.96 at lc=0.7
C_n    = 2/(1 + lambda_c);            % 1.18 at lc=0.7

for s = 1:n_seg
    idx = (tout >= seg_edges(s)) & (tout < seg_edges(s+1));
    if s == n_seg
        idx = (tout >= seg_edges(s)) & (tout <= seg_edges(s+1));
    end

    h_mid = mean(h_t(idx));
    h_bar_mid = h_mid / R_radius;
    seg_h_mid(s) = h_mid;

    % Wall correction at segment midpoint
    if h_bar_mid > 1.001
        [c_para, c_perp] = calc_correction_functions(h_bar_mid);
    else
        c_para = NaN; c_perp = NaN;
    end
    a_x_axis = [a_nom/c_para; a_nom/c_para; a_nom/c_perp];   % X,Y use para; Z uses perp

    for ax = 1:3
        seg_mean(s, ax) = mean(delta_x_m(idx, ax));
        seg_std(s, ax)  = std(delta_x_m(idx, ax));
        sigma2_th = C_dpmr * 4 * kBT * a_x_axis(ax) + C_n * sigma2_n_s(ax);
        seg_theory_std(s, ax) = sqrt(max(sigma2_th, 0));
    end
end

%% Print segment table
fprintf('\n=== Segmented analysis (n=%d windows of %.1f s) ===\n', n_seg, seg_edges(2)-seg_edges(1));
fprintf('  paper Eq.12: sigma_dx = sqrt( C_dpmr*4kBT*a_x + C_n*sigma2_n )\n');
fprintf('  C_dpmr=%.3f (lc=%.2f), C_n=%.3f\n\n', C_dpmr, lambda_c, C_n);
fprintf(' seg | t_center | h_mid  | mean_X     mean_Y     mean_Z       | std_X(meas/th)        std_Y(meas/th)        std_Z(meas/th)\n');
fprintf('-----|----------|--------|------------------------------------|---------------------------------------------------------------\n');
for s = 1:n_seg
    fprintf(' %2d  | %5.2f s  | %5.2f  | %+.2e %+.2e %+.2e | %.2e/%.2e   %.2e/%.2e   %.2e/%.2e\n', ...
            s, seg_centers(s), seg_h_mid(s), ...
            seg_mean(s,1), seg_mean(s,2), seg_mean(s,3), ...
            seg_std(s,1), seg_theory_std(s,1), ...
            seg_std(s,2), seg_theory_std(s,2), ...
            seg_std(s,3), seg_theory_std(s,3));
end

%% Plot — Fig 6-style verification
fig = figure('Position', [50 50 1500 800], 'Color', 'w');
tl = tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

for ax = 1:3
    % Left column: time-domain delta_x_m + mean per segment
    nexttile;
    plot(tout, delta_x_m(:, ax), 'Color', [0 0.2 0.8], 'LineWidth', 0.3); hold on;
    % Overlay segment means as red circles
    plot(seg_centers, seg_mean(:, ax), 'ro-', 'MarkerSize', 6, ...
         'MarkerFaceColor', 'r', 'LineWidth', 1.4);
    yline(0, 'k--', 'LineWidth', 0.8);
    grid on;
    ylabel(sprintf('\\delta_{m,%s} [um]', labels{ax}));
    if ax == 1
        title('Time domain + segment mean (red)');
    end
    if ax == 3
        xlabel('Time [sec]');
    end
    xlim([0 config.T_sim]);

    % Right column: per-segment std (measured vs theoretical) — mimics paper Fig 6 right
    nexttile;
    plot(seg_centers, seg_std(:, ax),         'ro-', 'MarkerFaceColor', 'r', ...
         'LineWidth', 1.5, 'MarkerSize', 6); hold on;
    plot(seg_centers, seg_theory_std(:, ax),  'k--', 'LineWidth', 1.5);
    plot(seg_centers, seg_mean(:, ax),        'bs-', 'MarkerFaceColor', 'b', ...
         'LineWidth', 1.2, 'MarkerSize', 5);
    yline(0, 'k:', 'LineWidth', 0.5);
    grid on;
    legend({'meas std', 'theory std', 'mean'}, 'Location', 'best', 'FontSize', 8);
    if ax == 1
        title('Segment std (red) vs theory (black dashed) + mean (blue)');
    end
    if ax == 3
        xlabel('Time [sec]');
    end
    ylabel(sprintf('[um]  axis %s', labels{ax}));
    xlim([0 config.T_sim]);
end
sgtitle(sprintf('Fig 6-style verification: ramp\\_descent 50\\rightarrow 5, T\\_sim=%g s, lc=%.2f', ...
                config.T_sim, lambda_c));

save_dir = fullfile(project_root, 'test_results', 'learn_variance');
plot_path = fullfile(save_dir, 'fig6_verify_ramp_50to5_T20.png');
exportgraphics(fig, plot_path, 'Resolution', 150);
fprintf('\nSaved: %s\n', plot_path);
