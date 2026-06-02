%% verify_qr_derivation.m — Compare empirical Q/R vs derived Q/R in Simulink
%
% Runs the 7-state EKF simulation twice under identical trajectory/noise
% conditions:
%   Run 1: EMPIRICAL (pre-derivation) Q = [0;0;1e4;1e-1;0;1e-4;0], R = [1e-2;1.0]
%   Run 2: DERIVED   (beta interpretation) Q = [0;0;1;0;0;Q77;Q77], R = [R11;R22]
%
% Reports 3D RMSE, a_hat tracking error, DARE behavior, and writes a report
% to reference/qr_analysis/qr_verification_report.md.

clear; close all; clc;
clear motion_control_law motion_control_law_23state motion_control_law_7state trajectory_generator;

[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
project_root = fileparts(script_dir);
cd(project_root);

addpath(fullfile(project_root, 'model'));
addpath(fullfile(project_root, 'model', 'config'));
addpath(fullfile(project_root, 'model', 'wall_effect'));
addpath(fullfile(project_root, 'model', 'thermal_force'));
addpath(fullfile(project_root, 'model', 'trajectory'));
addpath(fullfile(project_root, 'model', 'controller'));
addpath(script_dir);

fprintf('===== verify_qr_derivation (3-way A str Simulink comparison) =====\n\n');

%% Shared configuration (same for both runs)
base_config = user_config();
base_config.theta = 0;
base_config.phi   = 0;
base_config.pz    = 0;
base_config.h_min = 1.1 * 2.25;
base_config.enable_wall_effect = true;

base_config.t_hold    = 0.5;
base_config.h_init    = 20;              % default per user_config (was 50 in run_simulation)
base_config.h_bottom  = 2.5;
base_config.amplitude = 2.5;             % default per user_config (was 10 in run_simulation)
base_config.frequency = 1;
base_config.n_cycles  = 3;
base_config.trajectory_type = 'osc';

base_config.ctrl_enable = true;
base_config.controller_type = 7;         % 7-state EKF (derivation target)
base_config.lambda_c = 0.7;
base_config.a_pd  = 0.05;
base_config.a_prd = 0.05;
base_config.a_cov = 0.05;
base_config.epsilon = 0.01;

% NOISE ON per decision D2
base_config.meas_noise_enable = true;
base_config.meas_noise_std    = [0.01; 0.01; 0.01];

base_config.thermal_enable = true;

base_config.Pf_init_diag = [0; 0; 1e-4; 1e-4; 0; 10*(0.0147)^2; 0];
base_config.beta   = 0;
base_config.lamdaF = 1.0;

% Trajectory duration
T_margin = 0.3;
t_descend = 1 / base_config.frequency;
T_traj = base_config.t_hold + t_descend + base_config.n_cycles / base_config.frequency;
base_config.T_sim = T_traj + T_margin;

t_warmup = 0.2;

%% Q/R variants
variants = struct();

% Per-axis Bus: Qz is 9x1 [Q1..Q6; Q7_x; Q7_y; Q7_z], Rz is 6x1
% [R_pos_x; R_pos_y; R_pos_z; R_gain_x; R_gain_y; R_gain_z].
% Variants below use scalar single-axis values replicated across all 3 axes.
expand_Q = @(q7_old) [q7_old(1:6); q7_old(7); q7_old(7); q7_old(7)];
expand_R = @(r_old) [r_old(1); r_old(1); r_old(1); r_old(2); r_old(2); r_old(2)];

variants(1).name = 'empirical';
variants(1).label = 'Empirical (pre-derivation)';
variants(1).Qz = expand_Q([0; 0; 1e4; 1e-1; 0; 1e-4; 0]);
variants(1).Rz = expand_R([1e-2; 1.0]);

% Load derived Q77 scaling and R22 scaling
q77_path = fullfile(project_root, 'test_results', 'verify', 'q77_trajectory.mat');
r22_path = fullfile(project_root, 'test_results', 'verify', 'r22_self_consistent.mat');
q77_data = load(q77_path, 'Q77_scaling');
r22_data = load(r22_path, 'R22_scaling');
sigma2_n = base_config.meas_noise_std(3)^2;
phys = physical_constants();
sigma2_dXT = 4 * phys.k_B * phys.T * (phys.Ts / phys.gamma_N);
R11_derived = sigma2_n / sigma2_dXT;

variants(2).name = 'derived_beta';
variants(2).label = 'Derived (backward-diff beta)';
variants(2).Qz = expand_Q([0; 0; 1; 0; 0; q77_data.Q77_scaling; q77_data.Q77_scaling]);
variants(2).Rz = expand_R([R11_derived; r22_data.R22_scaling]);

% B'-2 white-noise equivalent (Q(6,6) larger, Q(7,7) smaller)
bprime_path = fullfile(project_root, 'test_results', 'verify', 'q_bprime_deployed.mat');
if exist(bprime_path, 'file')
    bp = load(bprime_path);
    variants(3).name = 'derived_Bprime';
    variants(3).label = 'Derived (B''-2 white-noise equiv)';
    variants(3).Qz = expand_Q([0; 0; 1; 0; 0; bp.Q66_deployed_exp; bp.Q77_deployed_exp]);
    variants(3).Rz = expand_R([R11_derived; r22_data.R22_scaling]);

    % Ablation: B' Q + empirical R - isolates whether the remaining gap
    % is explained by R(2,2) = 1.72 (derived) vs 1.0 (empirical).
    variants(4).name = 'Bprime_Remp';
    variants(4).label = 'B'' Q + empirical R';
    variants(4).Qz = expand_Q([0; 0; 1; 0; 0; bp.Q66_deployed_exp; bp.Q77_deployed_exp]);
    variants(4).Rz = expand_R([1e-2; 1.0]);

    % Ablation: empirical Q + derived R - the inverse
    variants(5).name = 'Qemp_Rderived';
    variants(5).label = 'Empirical Q + derived R';
    variants(5).Qz = expand_Q([0; 0; 1e4; 1e-1; 0; 1e-4; 0]);
    variants(5).Rz = expand_R([R11_derived; r22_data.R22_scaling]);
end

fprintf('Variants prepared:\n');
for v = 1:numel(variants)
    fprintf('  [%s] Q=[%s] R=[%s]\n', variants(v).name, ...
            sprintf('%.4g ', variants(v).Qz), ...
            sprintf('%.4g ', variants(v).Rz));
end
fprintf('\n');

%% Run each variant
results = struct();
for v = 1:numel(variants)
    fprintf('===== Running variant: %s =====\n', variants(v).label);

    config = base_config;
    config.Qz_diag_scaling = variants(v).Qz;
    config.Rz_diag_scaling = variants(v).Rz;

    clear motion_control_law_7state trajectory_generator;

    try
        params = calc_simulation_params(config);
    catch ME
        fprintf('  ERROR in calc_simulation_params: %s\n', ME.message);
        results(v).run_ok = false;
        results(v).error_msg = ME.message;
        continue;
    end

    p0 = params.Value.common.p0;
    assignin('base', 'params', params);
    assignin('base', 'p0', p0);
    assignin('base', 'Ts', params.Value.common.Ts);

    T_sim = config.T_sim;
    fprintf('  p0 = [%.3f %.3f %.3f] um,  T_sim = %.2f s\n', p0, T_sim);

    t_sim_start = tic;
    try
        simOut = sim(fullfile(project_root, 'model', 'system_model'), ...
                     'StopTime', num2str(T_sim), ...
                     'SaveTime', 'on', 'TimeSaveName', 'tout', ...
                     'SaveOutput', 'on', 'OutputSaveName', 'yout');
        run_ok = true;
        err_msg = '';
    catch ME
        run_ok = false;
        err_msg = ME.message;
        fprintf('  Simulink ERROR: %s\n', err_msg);
    end
    t_sim = toc(t_sim_start);

    if ~run_ok
        results(v).run_ok = false;
        results(v).error_msg = err_msg;
        results(v).config = config;
        continue;
    end

    %% Extract metrics
    Ts = params.Value.common.Ts;
    p_d_log = simOut.p_d_out';
    p_m_log = simOut.p_m_out';
    f_d_log = simOut.f_d_out';

    N_discrete = size(p_d_log, 2);
    t_sample = (0:(N_discrete-1)) * Ts;

    error_3d = vecnorm(p_m_log - p_d_log, 2, 1);
    error_x = (p_m_log(1,:) - p_d_log(1,:)) * 1000;  % nm
    error_y = (p_m_log(2,:) - p_d_log(2,:)) * 1000;
    error_z = (p_m_log(3,:) - p_d_log(3,:)) * 1000;

    idx_ctrl = t_sample >= t_warmup;

    rmse_x  = rms(error_x(idx_ctrl));
    rmse_y  = rms(error_y(idx_ctrl));
    rmse_z  = rms(error_z(idx_ctrl));
    rmse_3d = rms(error_3d(idx_ctrl)) * 1000;  % nm

    max_x  = max(abs(error_x(idx_ctrl)));
    max_y  = max(abs(error_y(idx_ctrl)));
    max_z  = max(abs(error_z(idx_ctrl)));
    max_3d = max(error_3d(idx_ctrl)) * 1000;

    % EKF a_hat analysis
    ekf_log = simOut.ekf_out';
    a_hat_log = ekf_log(1:2, :);  % [x; z]

    w_hat = params.Value.wall.w_hat;
    pz_wall = params.Value.wall.pz;
    R_particle = params.Value.common.R;
    a_nom = params.Value.common.Ts / params.Value.common.gamma_N;

    a_true_log = zeros(2, N_discrete);
    for k = 1:N_discrete
        h_val = (p_m_log(:,k)' * w_hat - pz_wall) / R_particle;
        if h_val > 1.001
            [c_para_k, c_perp_k] = calc_correction_functions(h_val);
        else
            c_para_k = 15; c_perp_k = 15;  % fallback (saturation)
        end
        a_true_log(:, k) = [a_nom / c_para_k; a_nom / c_perp_k];
    end

    % a_hat vs a_true analysis (post-warmup)
    a_hat_ctrl = a_hat_log(:, idx_ctrl);
    a_true_ctrl = a_true_log(:, idx_ctrl);
    a_hat_err_abs_x = abs(a_hat_ctrl(1,:) - a_true_ctrl(1,:));
    a_hat_err_abs_z = abs(a_hat_ctrl(2,:) - a_true_ctrl(2,:));
    a_hat_err_rel_x = a_hat_err_abs_x ./ max(a_true_ctrl(1,:), eps);
    a_hat_err_rel_z = a_hat_err_abs_z ./ max(a_true_ctrl(2,:), eps);

    a_hat_rel_median_x = median(a_hat_err_rel_x);
    a_hat_rel_median_z = median(a_hat_err_rel_z);
    a_hat_rel_max_x    = max(a_hat_err_rel_x);
    a_hat_rel_max_z    = max(a_hat_err_rel_z);

    % a_hat "frozen" check: is std(a_hat) << std(a_true)?
    a_hat_std_x = std(a_hat_ctrl(1,:));
    a_hat_std_z = std(a_hat_ctrl(2,:));
    a_true_std_x = std(a_true_ctrl(1,:));
    a_true_std_z = std(a_true_ctrl(2,:));
    a_hat_trackratio_x = a_hat_std_x / max(a_true_std_x, eps);
    a_hat_trackratio_z = a_hat_std_z / max(a_true_std_z, eps);

    % NaN check
    nan_count = sum(isnan(p_m_log(:))) + sum(isnan(f_d_log(:))) + sum(isnan(a_hat_log(:)));

    % Store
    results(v).run_ok          = true;
    results(v).label           = variants(v).label;
    results(v).name            = variants(v).name;
    results(v).config          = config;
    results(v).Qz              = variants(v).Qz;
    results(v).Rz              = variants(v).Rz;
    results(v).t_sim_wall_time = t_sim;
    results(v).t_sample        = t_sample;
    results(v).rmse_3d         = rmse_3d;
    results(v).rmse_x          = rmse_x;
    results(v).rmse_y          = rmse_y;
    results(v).rmse_z          = rmse_z;
    results(v).max_3d          = max_3d;
    results(v).max_x           = max_x;
    results(v).max_y           = max_y;
    results(v).max_z           = max_z;
    results(v).a_hat_rel_median_x = a_hat_rel_median_x;
    results(v).a_hat_rel_median_z = a_hat_rel_median_z;
    results(v).a_hat_rel_max_x    = a_hat_rel_max_x;
    results(v).a_hat_rel_max_z    = a_hat_rel_max_z;
    results(v).a_hat_trackratio_x = a_hat_trackratio_x;
    results(v).a_hat_trackratio_z = a_hat_trackratio_z;
    results(v).nan_count       = nan_count;
    results(v).p_d_log         = p_d_log;
    results(v).p_m_log         = p_m_log;
    results(v).a_hat_log       = a_hat_log;
    results(v).a_true_log      = a_true_log;
    results(v).error_3d        = error_3d;
    results(v).error_x         = error_x;
    results(v).error_y         = error_y;
    results(v).error_z         = error_z;
    results(v).f_d_log         = f_d_log;

    fprintf('  Sim wall time: %.1f s\n', t_sim);
    fprintf('  3D RMSE = %.2f nm   Max = %.2f nm\n', rmse_3d, max_3d);
    fprintf('  Per-axis RMSE [x y z] nm: %.2f  %.2f  %.2f\n', rmse_x, rmse_y, rmse_z);
    fprintf('  a_hat median rel error (x, z): %.2f%%  %.2f%%\n', ...
            100*a_hat_rel_median_x, 100*a_hat_rel_median_z);
    fprintf('  a_hat track ratio std(a_hat)/std(a_true) (x, z): %.3f  %.3f\n', ...
            a_hat_trackratio_x, a_hat_trackratio_z);
    fprintf('  NaN count: %d\n\n', nan_count);
end

%% Comparison and acceptance gates
fprintf('\n===== COMPARISON =====\n\n');
n_v = numel(variants);
col_width = 24;
head = sprintf('%-35s |', 'Metric');
for v = 1:n_v
    head = [head, sprintf(' %-*s |', col_width, variants(v).name)];
end
fprintf('%s\n', head);
fprintf('%s\n', repmat('-', 1, length(head)));

labels_fmt = {'3D RMSE (nm)', '3D max (nm)', 'X RMSE (nm)', 'Y RMSE (nm)', 'Z RMSE (nm)', ...
              'a_hat_x median rel %%', 'a_hat_z median rel %%', ...
              'a_hat_x track ratio', 'a_hat_z track ratio', 'NaN count'};
fld_fmt = {'rmse_3d', 'max_3d', 'rmse_x', 'rmse_y', 'rmse_z', ...
           'a_hat_rel_median_x', 'a_hat_rel_median_z', ...
           'a_hat_trackratio_x', 'a_hat_trackratio_z', 'nan_count'};
scale_fmt = [1 1 1 1 1 100 100 1 1 1];

for m = 1:numel(labels_fmt)
    row = sprintf('%-35s |', labels_fmt{m});
    for v = 1:n_v
        if results(v).run_ok
            val = results(v).(fld_fmt{m}) * scale_fmt(m);
            if m == 10
                row = [row, sprintf(' %*d |', col_width, val)];
            else
                row = [row, sprintf(' %*.3f |', col_width, val)];
            end
        else
            row = [row, sprintf(' %*s |', col_width, 'ERR')];
        end
    end
    fprintf('%s\n', row);
end

% Acceptance gates (compare each derived vs empirical = variant 1)
if results(1).run_ok
    r1 = results(1);
    for v = 2:n_v
        if ~results(v).run_ok, continue; end
        r2 = results(v);
        fprintf('\n----- Acceptance gates for variant %s -----\n', variants(v).name);
        gate1 = (r2.rmse_3d <= 1.25 * r1.rmse_3d);
        gate2 = (r2.a_hat_rel_median_z < 0.10);
        gate3 = (r2.a_hat_trackratio_z > 0.5);
        fprintf('  G-1: 3D RMSE <= 1.25 * empirical   %s (%.2f <= %.2f)\n', ...
                tern(gate1, 'PASS', 'FAIL'), r2.rmse_3d, 1.25*r1.rmse_3d);
        fprintf('  G-2: a_hat_z median rel < 10%%      %s (%.2f%%)\n', ...
                tern(gate2, 'PASS', 'FAIL'), 100*r2.a_hat_rel_median_z);
        fprintf('  G-3: a_hat_z track ratio > 0.5     %s (%.3f)\n', ...
                tern(gate3, 'PASS', 'FAIL'), r2.a_hat_trackratio_z);
    end
end

%% Plot comparison figure
fig = figure('Position', [100, 100, 1400, 900], 'Visible', 'off');
colors = {[0.5 0.5 0.5], [0.8 0.0 0.0], [0.0 0.4 0.8], [0.0 0.6 0.3], [0.6 0.3 0.0]};  % 5 colors
legend_labels = cellfun(@(s) s, {results.label}, 'UniformOutput', false);
if all([results.run_ok])
    t_sample = results(1).t_sample;

    % a_hat z-axis tracking
    subplot(3, 2, 1);
    plot(t_sample, results(1).a_true_log(2,:), 'k-', 'LineWidth', 1.5); hold on;
    for v = 1:numel(results)
        plot(t_sample, results(v).a_hat_log(2,:), 'Color', colors{v}, 'LineWidth', 1.2);
    end
    legend([{'a_{true}'}, legend_labels], 'Location', 'northoutside', ...
           'Orientation', 'horizontal', 'FontSize', 8);
    ylabel('a_z (um/pN)');
    xlabel('time [s]');
    title('z-axis motion gain estimation');
    set(gca, 'FontSize', 10);

    % a_hat x-axis tracking
    subplot(3, 2, 2);
    plot(t_sample, results(1).a_true_log(1,:), 'k-', 'LineWidth', 1.5); hold on;
    for v = 1:numel(results)
        plot(t_sample, results(v).a_hat_log(1,:), 'Color', colors{v}, 'LineWidth', 1.2);
    end
    legend([{'a_{true}'}, legend_labels], 'Location', 'northoutside', ...
           'Orientation', 'horizontal', 'FontSize', 8);
    ylabel('a_x (um/pN)');
    xlabel('time [s]');
    title('x-axis motion gain estimation');
    set(gca, 'FontSize', 10);

    % Error 3D
    subplot(3, 2, 3);
    hold on;
    for v = 1:numel(results)
        plot(t_sample, results(v).error_3d*1000, 'Color', colors{v}, 'LineWidth', 1.0);
    end
    legend(legend_labels, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', 8);
    ylabel('3D error [nm]');
    xlabel('time [s]');
    title('3D tracking error');
    set(gca, 'FontSize', 10);

    % Error z
    subplot(3, 2, 4);
    hold on;
    for v = 1:numel(results)
        plot(t_sample, results(v).error_z, 'Color', colors{v}, 'LineWidth', 1.0);
    end
    legend(legend_labels, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', 8);
    ylabel('z error [nm]');
    xlabel('time [s]');
    title('z-axis error');
    set(gca, 'FontSize', 10);

    % f_d z
    subplot(3, 2, 5);
    hold on;
    for v = 1:numel(results)
        plot(t_sample, results(v).f_d_log(3,:), 'Color', colors{v}, 'LineWidth', 1.0);
    end
    legend(legend_labels, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', 8);
    ylabel('f_z [pN]');
    xlabel('time [s]');
    title('Control force (z)');
    set(gca, 'FontSize', 10);

    % Trajectory height
    subplot(3, 2, 6);
    pz_wall = results(1).config.pz;
    R_val = 2.25;
    h_bar_1 = (results(1).p_d_log' * [0;0;1] - pz_wall) / R_val;
    plot(t_sample, h_bar_1, 'k-', 'LineWidth', 1.5);
    ylabel('h_{bar} (reference)');
    xlabel('time [s]');
    title('Reference trajectory h/R');
    set(gca, 'FontSize', 10);

    fig_path = fullfile(project_root, 'reference', 'qr_analysis', 'fig_qr_verification.png');
    saveas(fig, fig_path);
    close(fig);
    fprintf('\nFigure saved: %s\n', fig_path);
end

%% Write markdown report
rep_path = fullfile(project_root, 'reference', 'qr_analysis', 'qr_verification_report.md');
fid = fopen(rep_path, 'w');
if fid > 0
    fprintf(fid, '# Q/R Derivation Simulink Verification Report\n\n');
    fprintf(fid, '**Generated**: %s\n\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
    fprintf(fid, '## Scenario\n\n');
    fprintf(fid, '- Controller: 7-state EKF\n');
    fprintf(fid, '- Trajectory: h_init=%.1f -> h_bottom=%.1f, amp=%.1f um, f=%.1f Hz, %d cycles\n', ...
            base_config.h_init, base_config.h_bottom, base_config.amplitude, ...
            base_config.frequency, base_config.n_cycles);
    fprintf(fid, '- Measurement noise: ON (std=[0.01;0.01;0.01] um)\n');
    fprintf(fid, '- Thermal: ON\n');
    fprintf(fid, '- Duration: %.2f s (warmup %.2f s excluded from stats)\n\n', ...
            base_config.T_sim, t_warmup);
    fprintf(fid, '## Q/R variants compared\n\n');
    for v = 1:numel(variants)
        fprintf(fid, '- **%s**: Q = [%s], R = [%s]\n', variants(v).label, ...
                sprintf('%.4g ', variants(v).Qz), sprintf('%.4g ', variants(v).Rz));
    end
    fprintf(fid, '\n## Results\n\n');
    % Table header
    fprintf(fid, '| Metric |');
    for v = 1:numel(variants), fprintf(fid, ' %s |', variants(v).name); end
    fprintf(fid, '\n|---|');
    for v = 1:numel(variants), fprintf(fid, '---|'); end
    fprintf(fid, '\n');
    row_labels = {'3D RMSE (nm)', '3D max (nm)', 'X RMSE (nm)', 'Y RMSE (nm)', 'Z RMSE (nm)', ...
                  'a_hat_x median rel %%', 'a_hat_z median rel %%', ...
                  'a_hat_x track ratio', 'a_hat_z track ratio', 'NaN count'};
    row_flds = {'rmse_3d', 'max_3d', 'rmse_x', 'rmse_y', 'rmse_z', ...
                'a_hat_rel_median_x', 'a_hat_rel_median_z', ...
                'a_hat_trackratio_x', 'a_hat_trackratio_z', 'nan_count'};
    row_scales = [1 1 1 1 1 100 100 1 1 1];
    for m = 1:numel(row_labels)
        fprintf(fid, '| %s |', row_labels{m});
        for v = 1:numel(variants)
            if results(v).run_ok
                val = results(v).(row_flds{m}) * row_scales(m);
                if m == 10
                    fprintf(fid, ' %d |', val);
                else
                    fprintf(fid, ' %.3f |', val);
                end
            else
                fprintf(fid, ' ERROR |');
            end
        end
        fprintf(fid, '\n');
    end

    fprintf(fid, '\n### Acceptance gates (each derived vs empirical)\n\n');
    if results(1).run_ok
        r1 = results(1);
        for v = 2:numel(variants)
            if ~results(v).run_ok, continue; end
            r2 = results(v);
            fprintf(fid, '**%s**:\n', variants(v).label);
            fprintf(fid, '- G1 (3D RMSE <= 1.25x empirical): %s (%.2f <= %.2f nm)\n', ...
                    tern(r2.rmse_3d <= 1.25 * r1.rmse_3d, 'PASS', 'FAIL'), ...
                    r2.rmse_3d, 1.25*r1.rmse_3d);
            fprintf(fid, '- G2 (a_hat_z median rel err < 10%%): %s (%.2f%%)\n', ...
                    tern(r2.a_hat_rel_median_z < 0.10, 'PASS', 'FAIL'), ...
                    100*r2.a_hat_rel_median_z);
            fprintf(fid, '- G3 (a_hat_z not frozen, track ratio > 0.5): %s (%.3f)\n\n', ...
                    tern(r2.a_hat_trackratio_z > 0.5, 'PASS', 'FAIL'), ...
                    r2.a_hat_trackratio_z);
        end
    end
    fclose(fid);
    fprintf('Report saved: %s\n', rep_path);
end

%% Save .mat
mat_path = fullfile(project_root, 'test_results', 'verify', 'qr_verification.mat');
save(mat_path, 'results', 'variants', 'base_config');
fprintf('Data saved: %s\n', mat_path);

fprintf('\n===== Done =====\n');

function s = tern(cond, a, b)
    if cond, s = a; else, s = b; end
end
