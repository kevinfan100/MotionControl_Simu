%% verify_variance.m - Unified Variance Verification for 3 Controllers
%
% Verifies tracking error variance formulas (C_dpm and C_dpmr) for:
%   Controller 1 (type=1): Eq.17 delay compensation
%   Controller 2 (type=2): 3-state observer (le=0, deadbeat)
%   Controller 3 (type=7): 7-state EKF
%
% Conditions: stationary trajectory, no wall effect, known gamma_N,
%             no measurement noise, thermal force ON.
%
% Outputs 6 figures (2 per controller): C_dpm vs lc, C_dpmr vs lc
% Saves to test_results/verify/

clear; close all; clc;
clear motion_control_law motion_control_law_1 motion_control_law_2 ...
      motion_control_law_7state motion_control_law_23state ...
      trajectory_generator calc_thermal_force;

[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
project_root = fileparts(script_dir);
cd(project_root);

addpath(fullfile(project_root, 'model'));
addpath(fullfile(project_root, 'model', 'config'));
addpath(fullfile(project_root, 'model', 'wall_effect'));
addpath(fullfile(project_root, 'model', 'thermal_force'));
addpath(fullfile(project_root, 'model', 'trajectory'));
addpath(fullfile(project_root, 'model', 'controller'));

%% ===== SECTION 1: Configuration =====

ctrl_list  = [1, 2, 7];
lc_list    = [0.4, 0.5, 0.6, 0.7, 0.8, 0.9];
a_var_list = [0.005, 0.05, 0.5];
T_sim      = 20;
rerun_sim  = true;

n_ctrl = length(ctrl_list);
n_lc   = length(lc_list);
n_avar = length(a_var_list);

% Output directory
out_dir = fullfile(project_root, 'test_results', 'verify');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end

% Physical constants for post-processing
c = physical_constants();
sigma2_deltaXT = 4 * c.k_B * c.T * c.Ts / c.gamma_N;  % [um^2]
Ts = c.Ts;

% Figure style (from run_simulation.m)
AXIS_LW   = 3.0;
FONT_SIZE = 32;
LEGEND_FS = 24;
LINE_REF  = 4;
LINE_OUT  = 3;
MK_SIZE   = 12;
COL_REF   = [0.0 0.6 0.0];
COL_OUT   = [0.8 0.0 0.0];
COL_ERR   = [0.0 0.2 0.8];
% Additional colors for 3 a_var lines
COL_AVAR  = {COL_REF, COL_OUT, COL_ERR};
LINE_STYLES = {'-', '--', ':'};
MK_STYLES   = {'o', 's', 'd'};

% Controller labels
ctrl_names = containers.Map([1, 2, 7], ...
    {'Ctrl1 (Eq.17)', 'Ctrl2 (Observer)', 'Ctrl3 (7-state EKF)'});

%% ===== SECTION 2: Run Simulations =====

if rerun_sim
    fprintf('=== Running Simulations ===\n');

    for ci = 1:n_ctrl
        ctrl_type = ctrl_list(ci);

        for li = 1:n_lc
            lc = lc_list(li);
            fprintf('  ctrl=%d, lc=%.1f ... ', ctrl_type, lc);

            % Clear persistent states between runs
            clear motion_control_law motion_control_law_1 ...
                  motion_control_law_2 motion_control_law_7state ...
                  motion_control_law_23state trajectory_generator ...
                  calc_thermal_force;

            % Fixed seed for reproducibility
            rng(42 + li);

            % Configure
            config = user_config();
            config.trajectory_type    = 'positioning';
            config.h_init             = 50;
            config.t_hold             = 0;
            config.enable_wall_effect = false;
            config.ctrl_enable        = true;
            config.controller_type    = ctrl_type;
            config.lambda_c           = lc;
            config.meas_noise_enable  = false;
            config.meas_noise_std     = [0; 0; 0];
            config.thermal_enable     = true;
            config.T_sim              = T_sim;

            % Calculate params
            params = calc_simulation_params(config);
            p0 = params.Value.common.p0;

            % Run Simulink
            assignin('base', 'params', params);
            assignin('base', 'p0', p0);
            assignin('base', 'Ts', Ts);

            simOut = sim(fullfile(project_root, 'model', 'system_model'), ...
                'StopTime', num2str(T_sim), ...
                'SaveTime', 'on', 'TimeSaveName', 'tout', ...
                'SaveOutput', 'on', 'OutputSaveName', 'yout');

            % Extract data
            p_d_out  = simOut.p_d_out;   % [N x 3]
            p_m_out  = simOut.p_m_out;   % [N x 3]
            f_d_out  = simOut.f_d_out;   % [N x 3]

            % Save
            mat_file = fullfile(out_dir, ...
                sprintf('sim_ctrl%d_lc%02d.mat', ctrl_type, round(lc*100)));
            save(mat_file, 'p_d_out', 'p_m_out', 'f_d_out', ...
                 'ctrl_type', 'lc', 'T_sim', 'Ts');

            fprintf('saved.\n');
        end
    end
    fprintf('=== All simulations complete ===\n\n');
end

%% ===== SECTION 3: Post-processing =====

fprintf('=== Post-processing ===\n');

% Storage
C_dpm_sim  = zeros(n_ctrl, n_lc);          % direct variance factor
C_dpmr_sim = zeros(n_ctrl, n_lc, n_avar);  % IIR filtered variance factor

for ci = 1:n_ctrl
    ctrl_type = ctrl_list(ci);

    for li = 1:n_lc
        lc = lc_list(li);

        % Load
        mat_file = fullfile(out_dir, ...
            sprintf('sim_ctrl%d_lc%02d.mat', ctrl_type, round(lc*100)));
        data = load(mat_file);

        p_d = data.p_d_out;  % [N x 3]
        p_m = data.p_m_out;  % [N x 3]
        N_samples = size(p_d, 1);

        % Reconstruct del_pm = p_d[k-2] - p_m[k]
        del_pm = zeros(N_samples, 3);
        for k = 1:N_samples
            if k >= 3
                del_pm(k, :) = p_d(k-2, :) - p_m(k, :);
            else
                del_pm(k, :) = 0;
            end
        end

        % Steady-state window: discard first half
        ss_start = round(N_samples / 2);
        ss_idx = ss_start:N_samples;

        % C_dpm: direct variance (z-axis = column 3)
        var_z = var(del_pm(ss_idx, 3));
        C_dpm_sim(ci, li) = var_z / sigma2_deltaXT;

        % C_dpmr: IIR filtered variance for each a_var
        for ai = 1:n_avar
            a_var = a_var_list(ai);

            % Causal IIR HP filter (current project convention)
            del_pmd = zeros(N_samples, 3);
            del_pmr = zeros(N_samples, 3);
            for k = 2:N_samples
                del_pmd(k, :) = (1 - a_var) * del_pmd(k-1, :) + a_var * del_pm(k, :);
                del_pmr(k, :) = del_pm(k, :) - del_pmd(k, :);
            end

            var_z_iir = var(del_pmr(ss_idx, 3));
            C_dpmr_sim(ci, li, ai) = var_z_iir / sigma2_deltaXT;
        end
    end
end

% --- Theory ---
lc_dense = 0.1:0.005:0.95;

% C_dpm theory: K + 1/(1-lc^2)
%   Controller 1: K = 2
%   Controller 2: K = 3
%   Controller 3: no analytic formula (EKF is nonlinear)
C_dpm_theory = zeros(n_ctrl, length(lc_dense));
K_values = [2, 3, NaN];
has_C_dpm_theory = [true, true, false];

for ci = 1:n_ctrl
    if has_C_dpm_theory(ci)
        K = K_values(ci);
        C_dpm_theory(ci, :) = K + 1./(1 - lc_dense.^2);
    end
end

% C_dpmr theory (corrected formulas with (1-av)^2 prefactor):
%   Controller 1: K=2 (constant)
%     C_dpmr = (1-av)^2 * [2*(1-av)*(1-lc)/(1-(1-av)*lc)
%              + (2/(2-av)) / ((1+lc)*(1-(1-av)*lc))]
%   Controller 2: K_eff(av) = 2 + 2*(1-av)^2/(2-av)
%     C_dpmr = (1-av)^2 * [K_eff*(1-av)*(1-lc)/(1-(1-av)*lc)
%              + (2/(2-av)) / ((1+lc)*(1-(1-av)*lc))]
%   Controller 3: no analytic formula
C_dpmr_theory = zeros(n_ctrl, length(lc_dense), n_avar);
has_C_dpmr_theory = [true, true, false];

for ci = 1:n_ctrl
    if ~has_C_dpmr_theory(ci), continue; end

    for ai = 1:n_avar
        av = a_var_list(ai);

        % Determine K for this controller and a_var
        if ctrl_list(ci) == 1
            K = 2;  % constant for all a_var
        elseif ctrl_list(ci) == 2
            K = 2 + 2*(1-av)^2 / (2-av);  % K_eff depends on a_var
        end

        C_dpmr_theory(ci, :, ai) = (1-av)^2 * ( ...
            K * (1-av) * (1-lc_dense) ./ (1 - (1-av)*lc_dense) ...
            + (2/(2-av)) ./ ((1+lc_dense) .* (1 - (1-av)*lc_dense)));
    end
end

% --- Print results ---
fprintf('\n--- C_dpm Results (z-axis) ---\n');
for ci = 1:n_ctrl
    if has_C_dpm_theory(ci)
        K = K_values(ci);
        fprintf('\n%s (K=%.2f):\n', ctrl_names(ctrl_list(ci)), K);
        fprintf('  lc     sim       theory    err%%\n');
        for li = 1:n_lc
            lc = lc_list(li);
            theory = K + 1/(1 - lc^2);
            sim_val = C_dpm_sim(ci, li);
            err_pct = abs(sim_val - theory) / theory * 100;
            fprintf('  %.1f    %.4f    %.4f    %.1f%%\n', lc, sim_val, theory, err_pct);
        end
    else
        fprintf('\n%s (no analytic formula):\n', ctrl_names(ctrl_list(ci)));
        fprintf('  lc     sim\n');
        for li = 1:n_lc
            fprintf('  %.1f    %.4f\n', lc_list(li), C_dpm_sim(ci, li));
        end
    end
end

fprintf('\n--- C_dpmr Results (z-axis) ---\n');
for ci = 1:n_ctrl
    if has_C_dpmr_theory(ci)
        fprintf('\n%s:\n', ctrl_names(ctrl_list(ci)));
        for ai = 1:n_avar
            av = a_var_list(ai);

            if ctrl_list(ci) == 1
                K = 2;
            elseif ctrl_list(ci) == 2
                K = 2 + 2*(1-av)^2 / (2-av);
            end

            fprintf('  a_var=%.3f (K=%.4f):\n', av, K);
            fprintf('    lc     sim       theory    err%%\n');
            for li = 1:n_lc
                lc = lc_list(li);
                theory = (1-av)^2 * (K*(1-av)*(1-lc)/(1-(1-av)*lc) ...
                       + (2/(2-av)) / ((1+lc)*(1-(1-av)*lc)));
                sim_val = C_dpmr_sim(ci, li, ai);
                err_pct = abs(sim_val - theory) / theory * 100;
                fprintf('    %.1f    %.4f    %.4f    %.1f%%\n', lc, sim_val, theory, err_pct);
            end
        end
    else
        fprintf('\n%s (no analytic formula):\n', ctrl_names(ctrl_list(ci)));
        for ai = 1:n_avar
            av = a_var_list(ai);
            fprintf('  a_var=%.3f:\n', av);
            fprintf('    lc     sim\n');
            for li = 1:n_lc
                fprintf('    %.1f    %.4f\n', lc_list(li), C_dpmr_sim(ci, li, ai));
            end
        end
    end
end

%% ===== SECTION 4: Plot =====

fprintf('\n=== Generating Figures ===\n');

% Style overrides for figure generation
FIG_FONT_SIZE = 28;
FIG_LEGEND_FS = 18;
FIG_LINE_TH   = 3.5;
FIG_MK_SIZE   = 10;
FIG_LINE_SIM  = 2.0;

for ci = 1:n_ctrl
    ctrl_type = ctrl_list(ci);

    % --- Fig 1: C_dpm vs lc ---
    fig1 = figure('Position', [100 100 900 650], 'Visible', 'off');
    ax1 = axes(fig1);
    hold(ax1, 'on');

    if has_C_dpm_theory(ci)
        % Theory line + simulation points
        plot(ax1, lc_dense, C_dpm_theory(ci, :) * sigma2_deltaXT, '-', ...
            'Color', COL_REF, 'LineWidth', FIG_LINE_TH);
        plot(ax1, lc_list, C_dpm_sim(ci, :) * sigma2_deltaXT, 'o', ...
            'Color', COL_OUT, 'MarkerSize', FIG_MK_SIZE, 'LineWidth', FIG_LINE_SIM, ...
            'MarkerFaceColor', COL_OUT);
        legend(ax1, {'Theory', 'Simulation'}, 'FontSize', FIG_LEGEND_FS, ...
            'Location', 'northoutside', 'Orientation', 'horizontal');
    else
        % No theory: simulation points connected with line
        plot(ax1, lc_list, C_dpm_sim(ci, :) * sigma2_deltaXT, '-o', ...
            'Color', COL_OUT, 'LineWidth', FIG_LINE_SIM, ...
            'MarkerSize', FIG_MK_SIZE, 'MarkerFaceColor', COL_OUT);
        legend(ax1, 'Simulation', 'FontSize', FIG_LEGEND_FS, ...
            'Location', 'northoutside', 'Orientation', 'horizontal');
    end

    xlabel(ax1, '\lambda_c', 'FontSize', FIG_FONT_SIZE, 'FontWeight', 'bold');
    ylabel(ax1, 'Variance [um^2]', 'FontSize', FIG_FONT_SIZE, 'FontWeight', 'bold');
    set(ax1, 'FontSize', FIG_FONT_SIZE, 'FontWeight', 'bold', 'LineWidth', AXIS_LW);
    box(ax1, 'on'); grid(ax1, 'off');

    exportgraphics(fig1, fullfile(out_dir, sprintf('fig_ctrl%d_C_dpm.png', ctrl_type)), ...
        'Resolution', 150);
    fprintf('  Saved fig_ctrl%d_C_dpm.png\n', ctrl_type);
    close(fig1);

    % --- Fig 2: C_dpmr vs lc ---
    fig2 = figure('Position', [100 100 900 650], 'Visible', 'off');
    ax2 = axes(fig2);
    hold(ax2, 'on');

    h_leg = gobjects(n_avar, 1);
    leg_labels = cell(n_avar, 1);

    for ai = 1:n_avar
        av = a_var_list(ai);

        if has_C_dpmr_theory(ci)
            % Theory line
            plot(ax2, lc_dense, squeeze(C_dpmr_theory(ci, :, ai)) * sigma2_deltaXT, ...
                '-', 'Color', COL_AVAR{ai}, 'LineWidth', FIG_LINE_TH);
            % Simulation points
            h_leg(ai) = plot(ax2, lc_list, squeeze(C_dpmr_sim(ci, :, ai)) * sigma2_deltaXT, ...
                'o', 'Color', COL_AVAR{ai}, ...
                'MarkerSize', FIG_MK_SIZE, 'LineWidth', FIG_LINE_SIM, ...
                'MarkerFaceColor', COL_AVAR{ai});
        else
            % No theory: lines connecting simulation points
            h_leg(ai) = plot(ax2, lc_list, squeeze(C_dpmr_sim(ci, :, ai)) * sigma2_deltaXT, ...
                '-o', 'Color', COL_AVAR{ai}, 'LineWidth', FIG_LINE_SIM, ...
                'MarkerSize', FIG_MK_SIZE, 'MarkerFaceColor', COL_AVAR{ai});
        end

        leg_labels{ai} = sprintf('a_{var}=%.3f', av);
    end

    xlabel(ax2, '\lambda_c', 'FontSize', FIG_FONT_SIZE, 'FontWeight', 'bold');
    ylabel(ax2, 'Variance [um^2]', 'FontSize', FIG_FONT_SIZE, 'FontWeight', 'bold');
    set(ax2, 'FontSize', FIG_FONT_SIZE, 'FontWeight', 'bold', 'LineWidth', AXIS_LW);
    box(ax2, 'on'); grid(ax2, 'off');

    legend(ax2, h_leg, leg_labels, 'FontSize', FIG_LEGEND_FS, ...
        'Location', 'northoutside', 'Orientation', 'horizontal');

    exportgraphics(fig2, fullfile(out_dir, sprintf('fig_ctrl%d_C_dpmr.png', ctrl_type)), ...
        'Resolution', 150);
    fprintf('  Saved fig_ctrl%d_C_dpmr.png\n', ctrl_type);
    close(fig2);
end

fprintf('\n=== Verification Complete ===\n');
