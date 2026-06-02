% preview_individual.m
% Render one plot at a time for style review.
% Toggle `which_fig` to switch which figure to render.

clear; close all; clc;

[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
project_root = fileparts(fileparts(script_dir));
cd(project_root);
addpath(fullfile(project_root, 'model'));
addpath(fullfile(project_root, 'model', 'config'));
addpath(fullfile(project_root, 'model', 'wall_effect'));

%% ==== TOGGLE: which figure to render ====
which_fig_list = {'A0','B1','B2','A5','A6'};   % batch render these in sequence
% which_fig_list = {'A6'};   % uncomment for single-fig render

%% ==== Sweep mode ====
sweep_mode   = true;                     % true: loop over sweep_values; false: use config
sweep_param  = 'a_pd';                   % 'a_pd' | 'a_prd' | 'a_cov'
sweep_values = [0.005, 0.05, 0.5];       % 10x span around 0.05

%% ============== Load data + compute all signals ==============
S = load(fullfile(project_root, 'test_results', 'learn_variance', ...
                  'ramp_50to5_T20_delpm.mat'));
delta_z = S.delta_x_m(:, 3);
tout    = S.tout;
config  = S.config;
N       = length(tout);
Ts      = tout(2) - tout(1);

constants  = physical_constants();
kBT        = constants.k_B * constants.T;
gamma_N    = constants.gamma_N;
R_radius   = constants.R;
a_nom      = Ts / gamma_N;
lambda_c   = config.lambda_c;
sigma2_n_z = config.meas_noise_std(3)^2;
%% ==== a_pd-INDEPENDENT precompute (once) ====
C_n  = 2 / (1 + lambda_c);

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

%% ============== Style — match run_simulation.m thesis style ==============
COL_REF   = [0.0 0.6 0.0];           % green - reference / true
COL_OUT   = [0.8 0.0 0.0];           % red   - output / estimated
COL_ERR   = [0.0 0.2 0.8];           % blue  - error / tracking-error
COL_AUX   = [0.45 0.45 0.45];        % gray  - auxiliary

AXIS_LW   = 3.0;
FONT_SIZE = 32;
LEGEND_FS = 24;
LINE_REF  = 4;
LINE_OUT  = 3;
LINE_AUX  = 2;

set(groot, 'defaultTextInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');
set(groot, 'defaultAxesTickLabelInterpreter','latex');

%% ==== Determine sweep_list and base_save ====
switch sweep_param
    case 'a_pd';  short_name = 'apd';
    case 'a_prd'; short_name = 'aprd';
    case 'a_cov'; short_name = 'acov';
    otherwise; error('Unknown sweep_param: %s', sweep_param);
end
if sweep_mode
    sweep_list = sweep_values;
    base_save  = fullfile(project_root, 'test_results', 'learn_variance', ['sweep_' short_name]);
else
    sweep_list = config.(sweep_param);
    base_save  = fullfile(project_root, 'test_results', 'learn_variance');
end
% Default fixed alphas (the swept one is overridden per iteration)
a_pd_default  = 0.05;
a_prd_default = 0.05;
a_cov_default = 0.05;

%% ============== Outer sweep loop ==============
for sweep_idx = 1:length(sweep_list)
close all;   % clean up figures from previous iteration
% Reset all alphas to defaults, then override the swept one
a_pd  = a_pd_default;
a_prd = a_prd_default;
a_cov = a_cov_default;
sweep_val = sweep_list(sweep_idx);
switch sweep_param
    case 'a_pd';  a_pd  = sweep_val;
    case 'a_prd'; a_prd = sweep_val;
    case 'a_cov'; a_cov = sweep_val;
end

% a_pd-dependent recompute — C_dpmr with (1-a_pd)^2 prefactor
C_dpmr_bracket = 2*(1-a_pd)*(1-lambda_c) / (1-(1-a_pd)*lambda_c) ...
              + (2/(2-a_pd)) / ((1+lambda_c)*(1-(1-a_pd)*lambda_c));
C_dx = (1-a_pd)^2 * C_dpmr_bracket;
Var_theory_t = C_dx * 4 * kBT * a_z_t + C_n * sigma2_n_z;

fprintf('\n=== %s = %.4f (a_pd=%.3f, a_prd=%.3f, a_cov=%.3f) ===\n', ...
        sweep_param, sweep_val, a_pd, a_prd, a_cov);
fprintf('  C_dx = %.4f, swept-param window ~ %.1f samples = %.2f ms\n', ...
        C_dx, 2/sweep_val-1, (2/sweep_val-1)*Ts*1000);

% IIR chain
del_pmr2_init = C_dx * 4 * kBT * a_z_t(1) + C_n * sigma2_n_z;
del_pmd_z      = zeros(N, 1);  del_pmr_z      = zeros(N, 1);
del_pmrd_z     = zeros(N, 1);  del_pmr2_avg_z = zeros(N, 1);
del_pmr_var_z  = zeros(N, 1);
del_pmd_prev   = 0;            del_pmrd_prev  = 0;
del_pmr2_prev  = del_pmr2_init;
for k = 1:N
    dpmd  = (1-a_pd)  * del_pmd_prev  + a_pd  * delta_z(k);
    dpmr  = delta_z(k) - dpmd;
    dpmrd = (1-a_prd) * del_pmrd_prev + a_prd * dpmr;
    dpmr2 = (1-a_cov) * del_pmr2_prev + a_cov * dpmr^2;
    dvar  = max(dpmr2 - dpmrd^2, 0);
    del_pmd_z(k)=dpmd; del_pmr_z(k)=dpmr; del_pmrd_z(k)=dpmrd;
    del_pmr2_avg_z(k)=dpmr2; del_pmr_var_z(k)=dvar;
    del_pmd_prev=dpmd; del_pmrd_prev=dpmrd; del_pmr2_prev=dpmr2;
end

% Auto-scale ylim for A5/A6 (variance-domain plots)
ymax_data = max([max(del_pmr2_avg_z), max(del_pmr_var_z), max(Var_theory_t)]);
y_target  = 1.15 * ymax_data;
if     y_target <= 0.0005; ystep = 0.0001;
elseif y_target <= 0.001;  ystep = 0.0002;
elseif y_target <= 0.002;  ystep = 0.0004;
elseif y_target <= 0.004;  ystep = 0.0008;
else;                      ystep = 0.0010;
end
A5A6_ylim_top   = ystep * ceil(y_target / ystep);
A5A6_yticks_vec = 0:ystep:A5A6_ylim_top;
fprintf('  A5/A6 ylim: [0, %.4f] um^2 (step %.4f, %d ticks)\n', ...
        A5A6_ylim_top, ystep, length(A5A6_yticks_vec));

% Per-iteration save folder
if sweep_mode
    sweep_str = strrep(sprintf('%s_%.3f', short_name, sweep_val), '.', 'p');
    save_dir  = fullfile(base_save, sweep_str);
else
    save_dir = base_save;
end
if ~exist(save_dir,'dir'); mkdir(save_dir); end

%% ============== Render loop over which_fig_list ==============
for fig_idx = 1:length(which_fig_list)
which_fig = which_fig_list{fig_idx};

%% ============== A0: del_pm input single panel ==============
if strcmp(which_fig, 'A0')
    fig = figure('Position', [50 50 1400 750], 'Color', 'w');

    plot(tout, delta_z, '-', 'Color', COL_ERR, 'LineWidth', LINE_OUT, ...
         'DisplayName', '$\delta p_m$');
    hold on;
    yline(0, '-', 'Color', COL_AUX, 'LineWidth', LINE_AUX, ...
          'HandleVisibility', 'off');

    set(gca, 'FontSize', FONT_SIZE, 'FontWeight', 'bold', 'LineWidth', AXIS_LW);
    xlabel('Time [sec]', 'FontSize', FONT_SIZE, 'FontWeight', 'bold');
    ylabel('$\delta p_m$ [um]', 'FontSize', FONT_SIZE, 'FontWeight', 'bold');
    grid on; box on;

    % Locked axis range + clean tick labels (no trailing zeros)
    xlim([0 config.T_sim]);
    ylim([-0.15 0.15]);
    yticks(-0.15:0.05:0.15);
    yticklabels({'$-0.15$','$-0.1$','$-0.05$','$0$','$0.05$','$0.1$','$0.15$'});

    legend('Location', 'northoutside', 'Orientation', 'horizontal', ...
           'FontSize', LEGEND_FS, 'FontWeight', 'bold', ...
           'Interpreter', 'latex', 'Box', 'on');

    out_path = fullfile(save_dir, 'preview_A0.png');
    exportgraphics(fig, out_path, 'Resolution', 150);
    fprintf('Saved: %s\n', out_path);
end

%% ============== B1: del_pm + del_pmd overlay (Layer-1) ==============
if strcmp(which_fig, 'B1')
    fig = figure('Position', [50 50 1400 950], 'Color', 'w');

    mean_pm  = mean(delta_z);
    mean_pmd = mean(del_pmd_z);
    if abs(mean_pm)  < 5e-5; s1='0'; else; s1=sprintf('%+.4f', mean_pm);  end
    if abs(mean_pmd) < 5e-5; s2='0'; else; s2=sprintf('%+.4f', mean_pmd); end
    label_pm  = sprintf('$\\delta p_m$ \\quad mean $=\\,%s$ um', s1);
    label_pmd = sprintf('$\\delta p_{md}$ \\quad mean $=\\,%s$ um', s2);

    % Blue first (background, semi-transparent); red second (foreground, solid)
    p1 = plot(tout, delta_z,   '-', 'Color', COL_ERR, 'LineWidth', LINE_OUT, ...
              'DisplayName', label_pm);
    hold on;
    p2 = plot(tout, del_pmd_z, '-', 'Color', COL_OUT, 'LineWidth', LINE_REF, ...
              'DisplayName', label_pmd);
    yline(0, '-', 'Color', COL_AUX, 'LineWidth', LINE_AUX, ...
          'HandleVisibility', 'off');

    % Apply alpha (R2018b+: 4th channel of Color)
    p1.Color(4) = 0.40;     % blue background, semi-transparent
    p2.Color(4) = 1.00;     % red foreground, fully solid

    set(gca, 'FontSize', FONT_SIZE, 'FontWeight', 'bold', 'LineWidth', AXIS_LW);
    xlabel('Time [sec]', 'FontSize', FONT_SIZE, 'FontWeight', 'bold');
    ylabel('[um]', 'FontSize', FONT_SIZE, 'FontWeight', 'bold');
    grid on; box on;
    xlim([0 config.T_sim]);
    ylim([-0.15 0.15]);
    yticks(-0.15:0.05:0.15);
    yticklabels({'$-0.15$','$-0.1$','$-0.05$','$0$','$0.05$','$0.1$','$0.15$'});

    legend([p1 p2], 'Location', 'northoutside', 'NumColumns', 1, ...
           'FontSize', LEGEND_FS, 'FontWeight', 'bold', ...
           'Interpreter', 'latex', 'Box', 'on');

    out_path = fullfile(save_dir, 'preview_B1.png');
    exportgraphics(fig, out_path, 'Resolution', 150);
    fprintf('Saved: %s\n', out_path);
end

%% ============== B2: del_pmr + del_pmrd overlay (Layer-2) ==============
if strcmp(which_fig, 'B2')
    fig = figure('Position', [50 50 1400 950], 'Color', 'w');

    mean_pmr  = mean(del_pmr_z);
    mean_pmrd = mean(del_pmrd_z);
    if abs(mean_pmr)  < 5e-5; s1='0'; else; s1=sprintf('%+.4f', mean_pmr);  end
    if abs(mean_pmrd) < 5e-5; s2='0'; else; s2=sprintf('%+.4f', mean_pmrd); end
    label_pmr  = sprintf('$\\delta p_{mr}$ \\quad mean $=\\,%s$ um',  s1);
    label_pmrd = sprintf('$\\delta p_{mrd}$ \\quad mean $=\\,%s$ um', s2);

    p1 = plot(tout, del_pmr_z,  '-', 'Color', COL_ERR, 'LineWidth', LINE_OUT, ...
              'DisplayName', label_pmr);
    hold on;
    p2 = plot(tout, del_pmrd_z, '-', 'Color', COL_OUT, 'LineWidth', LINE_REF, ...
              'DisplayName', label_pmrd);
    yline(0, '-', 'Color', COL_AUX, 'LineWidth', LINE_AUX, ...
          'HandleVisibility', 'off');

    p1.Color(4) = 0.40;     % blue background
    p2.Color(4) = 1.00;     % red foreground

    set(gca, 'FontSize', FONT_SIZE, 'FontWeight', 'bold', 'LineWidth', AXIS_LW);
    xlabel('Time [sec]', 'FontSize', FONT_SIZE, 'FontWeight', 'bold');
    ylabel('[um]', 'FontSize', FONT_SIZE, 'FontWeight', 'bold');
    grid on; box on;
    xlim([0 config.T_sim]);
    ylim([-0.15 0.15]);
    yticks(-0.15:0.05:0.15);
    yticklabels({'$-0.15$','$-0.1$','$-0.05$','$0$','$0.05$','$0.1$','$0.15$'});

    legend([p1 p2], 'Location', 'northoutside', 'NumColumns', 1, ...
           'FontSize', LEGEND_FS, 'FontWeight', 'bold', ...
           'Interpreter', 'latex', 'Box', 'on');

    out_path = fullfile(save_dir, 'preview_B2.png');
    exportgraphics(fig, out_path, 'Resolution', 150);
    fprintf('Saved: %s\n', out_path);
end

%% ============== A4: del_pmrd^2 single (bias correction term) ==============
if strcmp(which_fig, 'A4')
    fig = figure('Position', [50 50 1400 950], 'Color', 'w');

    pmrd_sq  = del_pmrd_z.^2;
    mean_v   = mean(pmrd_sq);
    max_v    = max(pmrd_sq);

    label_str = sprintf(['$\\delta p_{mrd}^{\\,2}$ \\quad ', ...
                         'mean $=\\,%.6f$ um$^2$ \\quad ', ...
                         'max $=\\,%.4f$ um$^2$'], mean_v, max_v);

    p1 = plot(tout, pmrd_sq, '-', 'Color', COL_OUT, 'LineWidth', LINE_OUT, ...
              'DisplayName', label_str);

    set(gca, 'FontSize', FONT_SIZE, 'FontWeight', 'bold', 'LineWidth', AXIS_LW);
    xlabel('Time [sec]', 'FontSize', FONT_SIZE, 'FontWeight', 'bold');
    ylabel('$\delta p_{mrd}^{\,2}$ [um$^2$]', 'FontSize', FONT_SIZE, 'FontWeight', 'bold');
    grid on; box on;
    xlim([0 config.T_sim]);
    ylim([0 0.0008]);
    yticks(0:0.0002:0.0008);
    % 10^-4 regime: single ×10^-4 indicator at top-left, integer mantissa
    ax = gca; ax.YAxis.Exponent = -4;

    legend(p1, 'Location', 'northoutside', 'NumColumns', 1, ...
           'FontSize', LEGEND_FS, 'FontWeight', 'bold', ...
           'Interpreter', 'latex', 'Box', 'on');

    out_path = fullfile(save_dir, 'preview_A4.png');
    exportgraphics(fig, out_path, 'Resolution', 150);
    fprintf('Saved: %s\n', out_path);
end

%% ============== B3: del_pmr^2 + del_pmr2_avg overlay (Layer-3 EWMA) ==============
if strcmp(which_fig, 'B3')
    fig = figure('Position', [50 50 1400 950], 'Color', 'w');

    pmr_sq         = del_pmr_z.^2;
    mean_pmr_sq    = mean(pmr_sq);          max_pmr_sq    = max(pmr_sq);
    mean_pmr2_avg  = mean(del_pmr2_avg_z);  std_pmr2_avg  = std(del_pmr2_avg_z);

    label_pmr_sq = sprintf(['$\\delta p_{mr}^{\\,2}$ \\quad ', ...
                            'mean $=\\,%.6f$ um$^2$ \\quad ', ...
                            'max $=\\,%.4f$ um$^2$'], mean_pmr_sq, max_pmr_sq);
    label_pmr2_avg = sprintf(['$\\overline{\\delta p_{mr}^{\\,2}}$ \\quad ', ...
                              'mean $=\\,%.6f$ um$^2$ \\quad ', ...
                              'std $=\\,%.6f$ um$^2$'], mean_pmr2_avg, std_pmr2_avg);

    % Blue first (background, semi-transparent); red second (foreground, solid)
    p1 = plot(tout, pmr_sq,         '-', 'Color', COL_ERR, 'LineWidth', LINE_OUT, ...
              'DisplayName', label_pmr_sq);
    hold on;
    p2 = plot(tout, del_pmr2_avg_z, '-', 'Color', COL_OUT, 'LineWidth', LINE_REF, ...
              'DisplayName', label_pmr2_avg);

    p1.Color(4) = 0.40;     % blue background
    p2.Color(4) = 1.00;     % red foreground

    set(gca, 'FontSize', FONT_SIZE, 'FontWeight', 'bold', 'LineWidth', AXIS_LW);
    xlabel('Time [sec]', 'FontSize', FONT_SIZE, 'FontWeight', 'bold');
    ylabel('[um$^2$]', 'FontSize', FONT_SIZE, 'FontWeight', 'bold');
    grid on; box on;
    xlim([0 config.T_sim]);
    ylim([0 0.005]);
    yticks(0:0.001:0.005);
    % 10^-3 range → decimal, no trailing zeros
    yticklabels({'$0$','$0.001$','$0.002$','$0.003$','$0.004$','$0.005$'});
    ax = gca; ax.YAxis.Exponent = 0;     % suppress auto scientific exponent

    legend([p1 p2], 'Location', 'northoutside', 'NumColumns', 1, ...
           'FontSize', LEGEND_FS, 'FontWeight', 'bold', ...
           'Interpreter', 'latex', 'Box', 'on');

    out_path = fullfile(save_dir, 'preview_B3.png');
    exportgraphics(fig, out_path, 'Resolution', 150);
    fprintf('Saved: %s\n', out_path);
end

%% ============== A5: pmr2_avg only (single line) =====
if strcmp(which_fig, 'A5')
    fig = figure('Position', [50 50 1400 950], 'Color', 'w');

    plot(tout, del_pmr2_avg_z, '-', 'Color', COL_OUT, 'LineWidth', LINE_REF, ...
         'DisplayName', '$\overline{\delta p_{mr}^{\,2}}$');

    set(gca, 'FontSize', FONT_SIZE, 'FontWeight', 'bold', 'LineWidth', AXIS_LW);
    xlabel('Time [sec]', 'FontSize', FONT_SIZE, 'FontWeight', 'bold');
    ylabel('$\overline{\delta p_{mr}^{\,2}}$ [um$^2$]', 'FontSize', FONT_SIZE, 'FontWeight', 'bold');
    grid on; box on;
    xlim([0 config.T_sim]);
    ylim([0 A5A6_ylim_top]);
    yticks(A5A6_yticks_vec);
    ax = gca; ax.YAxis.Exponent = -4;     % single ×10^-4 at top-left

    legend('Location', 'northoutside', 'NumColumns', 1, ...
           'FontSize', LEGEND_FS, 'FontWeight', 'bold', ...
           'Interpreter', 'latex', 'Box', 'on');

    out_path = fullfile(save_dir, 'preview_A5.png');
    exportgraphics(fig, out_path, 'Resolution', 150);
    fprintf('Saved: %s\n', out_path);
end

%% ============== A6: var estimate vs theory =====
if strcmp(which_fig, 'A6')
    fig = figure('Position', [50 50 1400 950], 'Color', 'w');

    % Red estimate (semi-transparent) + green theory (dashed)
    p1 = plot(tout, del_pmr_var_z, '-',  'Color', COL_OUT, 'LineWidth', LINE_OUT, ...
              'DisplayName', '$\sigma^2_{\delta p_{mr}}$ (IIR)');
    hold on;
    p2 = plot(tout, Var_theory_t, '--', 'Color', COL_REF, 'LineWidth', LINE_REF, ...
              'DisplayName', '$\sigma^2_{\delta p_{mr}}$ (theory)');

    p1.Color(4) = 0.55;
    p2.Color(4) = 1.00;

    set(gca, 'FontSize', FONT_SIZE, 'FontWeight', 'bold', 'LineWidth', AXIS_LW);
    xlabel('Time [sec]', 'FontSize', FONT_SIZE, 'FontWeight', 'bold');
    ylabel('$\sigma^2_{\delta p_{mr}}$ [um$^2$]', 'FontSize', FONT_SIZE, 'FontWeight', 'bold');
    grid on; box on;
    xlim([0 config.T_sim]);
    ylim([0 A5A6_ylim_top]);
    yticks(A5A6_yticks_vec);
    ax = gca; ax.YAxis.Exponent = -4;

    legend([p1 p2], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
           'FontSize', LEGEND_FS, 'FontWeight', 'bold', ...
           'Interpreter', 'latex', 'Box', 'on');

    out_path = fullfile(save_dir, 'preview_A6.png');
    exportgraphics(fig, out_path, 'Resolution', 150);
    fprintf('Saved: %s\n', out_path);
end

end   % end of for fig_idx loop
end   % end of for sweep_idx loop

%% Reset interpreter defaults
set(groot, 'defaultTextInterpreter','remove');
set(groot, 'defaultLegendInterpreter','remove');
set(groot, 'defaultAxesTickLabelInterpreter','remove');
