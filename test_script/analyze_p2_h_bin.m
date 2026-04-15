% temp_p2_h_bin_analysis.m
% P2 Step 1: extract empirical tracking-error std per h-bin from task1d dynamic sweep
% Verifies whether the empirical std matches theoretical thermal-limited floor
% at different heights, especially near-wall h ≈ 2.5 µm.
%
% See plan: C:\Users\PME406_01\.claude\plans\glimmering-sniffing-simon.md

clear; close all;

% ---- Paths ----
proj_root = fileparts(fileparts(mfilename('fullpath')));
cd(proj_root);
addpath(fullfile(proj_root, 'model', 'wall_effect'));
fprintf('working dir: %s\n', pwd);

% ---- Constants ----
R = 2.25;                        % um, probe radius (model/config/physical_constants.m)
sigma2_dXT = 2.5189e-4;          % um², 4*k_B*T*a_nom (task1d_diagnostic_report.md)
Fs = 1600;

% ---- Load ----
S = load('test_results/verify/task1d_paper_benchmark_mc.mat');
o = S.out;
d = o.sample_seed_data;

C_dpmr     = o.C_dpmr_eff;          % 3.5381 at lc=0.4
a_nom      = o.a_nom;               % 1.4706e-2 um/pN
IIR_bf     = o.IIR_bias_factor;     % 0.9363

fprintf('Data: lc=%.2f, C_dpmr=%.4f, a_nom=%.4e, IIR_bias_factor=%.4f\n', ...
        o.config.lambda_c, C_dpmr, a_nom, IIR_bf);
fprintf('Sweep: h = [%.2f, %.2f] um, freq = %.1f Hz, T_sim = %.1f s\n', ...
        o.config.h_bottom, o.config.h_init, o.config.frequency, o.config.T_sim);

% ---- Time series ----
t_raw   = d.t(:)';
p_d_raw = d.p_d;          % 3 x N
p_m_raw = d.p_m;          % 3 x N
h_raw   = d.h_k(:)';
a_true  = d.a_true_traj(:)';
N_raw   = numel(t_raw);

% ---- Convention A: dxm[k] = p_d[k-2] - p_m[k] ----
% Drop first 2 samples to align, then also skip transient (before ss_start)
dxm       = p_d_raw(:, 1:end-2) - p_m_raw(:, 3:end);   % 3 x (N-2)
t_dxm     = t_raw(3:end);
h_dxm     = h_raw(3:end);
a_true_z  = a_true(3:end);

ss_start_idx = max(d.ss(1) - 2, 1);     % shift by 2 because of the drop
ss_idx = ss_start_idx:size(dxm, 2);

dxm_ss      = dxm(:, ss_idx);
h_ss        = h_dxm(ss_idx);
a_true_z_ss = a_true_z(ss_idx);

fprintf('\nSteady-state samples: N=%d (%.1f s)\n', numel(ss_idx), numel(ss_idx)/Fs);
fprintf('h_ss range: [%.3f, %.3f] um\n', min(h_ss), max(h_ss));

% ---- h bins ----
bin_edges = [2.47, 3.0, 4.0, 5.0, 7.5, 10, 15, 25, 50.3];
n_bins = numel(bin_edges) - 1;

% Containers
bin_center     = zeros(1, n_bins);
bin_count      = zeros(1, n_bins);
std_emp        = zeros(3, n_bins);
mean_emp       = zeros(3, n_bins);
std_theory_xy  = zeros(1, n_bins);
std_theory_z   = zeros(1, n_bins);
c_para_bin     = zeros(1, n_bins);
c_perp_bin     = zeros(1, n_bins);

for b = 1:n_bins
    mask = (h_ss >= bin_edges(b)) & (h_ss < bin_edges(b+1));
    N_bin = sum(mask);
    bin_count(b) = N_bin;

    if N_bin < 20
        fprintf('WARNING: bin %d [%.2f,%.2f) has only %d samples, skipping\n', ...
                b, bin_edges(b), bin_edges(b+1), N_bin);
        bin_center(b) = NaN;
        std_emp(:, b) = NaN;
        mean_emp(:, b) = NaN;
        std_theory_xy(b) = NaN;
        std_theory_z(b) = NaN;
        continue;
    end

    h_bin = h_ss(mask);
    bin_center(b) = mean(h_bin);

    for axis = 1:3
        vals = dxm_ss(axis, mask);
        mean_emp(axis, b) = mean(vals);
        std_emp(axis, b)  = std(vals, 0);
    end

    % Theoretical at bin center
    h_bar = bin_center(b) / R;
    [cpara, cperp] = calc_correction_functions(h_bar);
    c_para_bin(b) = cpara;
    c_perp_bin(b) = cperp;
    % a_x = a_nom / c_para,  a_z = a_nom / c_perp
    % Var = C_dpmr * 4*k_B*T * a_axis = C_dpmr * sigma2_dXT * (a_axis / a_nom)
    std_theory_xy(b) = sqrt(C_dpmr * sigma2_dXT / cpara);
    std_theory_z(b)  = sqrt(C_dpmr * sigma2_dXT / cperp);
end

% ---- Ratios ----
ratio_x = std_emp(1,:) ./ std_theory_xy;
ratio_y = std_emp(2,:) ./ std_theory_xy;
ratio_z = std_emp(3,:) ./ std_theory_z;

% ---- Print table ----
fprintf('\n=== Per-h-bin results (z-axis decisive) ===\n');
fprintf('%-13s %-6s %-6s %-12s %-13s %-6s %-10s %-10s\n', ...
        'bin[um]', 'N', 'h_c', 'c_perp', 'mean_z[nm]', ...
        'emp_z[nm]', 'thr_z[nm]', 'ratio_z');
for b = 1:n_bins
    if isnan(bin_center(b)); continue; end
    fprintf('[%4.2f,%5.2f)  %5d  %5.2f  %8.3f  %8.2f  %9.2f  %9.2f  %7.3f\n', ...
            bin_edges(b), bin_edges(b+1), bin_count(b), bin_center(b), ...
            c_perp_bin(b), 1000*mean_emp(3,b), 1000*std_emp(3,b), ...
            1000*std_theory_z(b), ratio_z(b));
end

fprintf('\n=== Per-h-bin results (x-axis for comparison) ===\n');
fprintf('%-13s %-6s %-12s %-13s %-6s %-10s %-10s\n', ...
        'bin[um]', 'N', 'c_para', 'mean_x[nm]', 'emp_x[nm]', 'thr_x[nm]', 'ratio_x');
for b = 1:n_bins
    if isnan(bin_center(b)); continue; end
    fprintf('[%4.2f,%5.2f)  %5d  %8.3f  %8.2f  %9.2f  %9.2f  %7.3f\n', ...
            bin_edges(b), bin_edges(b+1), bin_count(b), c_para_bin(b), ...
            1000*mean_emp(1,b), 1000*std_emp(1,b), ...
            1000*std_theory_xy(b), ratio_x(b));
end

% ---- Cross-check: bin-weighted average vs task1d A2 aggregate (+17%) ----
valid = ~isnan(bin_center);
w = bin_count(valid);
overall_emp_z_nm = 1000 * sqrt(sum(w .* std_emp(3, valid).^2) / sum(w));
overall_th_z_nm  = 1000 * sqrt(sum(w .* std_theory_z(valid).^2) / sum(w));
overall_ratio    = overall_emp_z_nm / overall_th_z_nm;
fprintf('\nBin-weighted z-axis: emp=%.2f nm, theory=%.2f nm, ratio=%.3f (task1d A2 said +17%%)\n', ...
        overall_emp_z_nm, overall_th_z_nm, overall_ratio);

% ---- Plot ----
figure('Position', [100, 100, 1000, 750], 'Color', 'w');

% Panel 1: std_emp (markers) + std_theory (lines) vs h, log x-axis
ax1 = subplot(2, 1, 1); hold on;
h_plot = bin_center(valid);
% Theoretical curves: use a dense h sweep for smooth curves
h_dense = logspace(log10(2.47), log10(50.3), 200);
cpara_dense = zeros(size(h_dense));
cperp_dense = zeros(size(h_dense));
for i = 1:numel(h_dense)
    [cp, cq] = calc_correction_functions(h_dense(i)/R);
    cpara_dense(i) = cp;
    cperp_dense(i) = cq;
end
std_th_xy_dense = 1000 * sqrt(C_dpmr * sigma2_dXT ./ cpara_dense);
std_th_z_dense  = 1000 * sqrt(C_dpmr * sigma2_dXT ./ cperp_dense);

cx = [0.00, 0.45, 0.74];   % blue
cy = [0.47, 0.67, 0.19];   % green
cz = [0.85, 0.33, 0.10];   % orange

plot(h_dense, std_th_xy_dense, '-', 'Color', cx, 'LineWidth', 1.5);
plot(h_dense, std_th_xy_dense, '-', 'Color', cy, 'LineWidth', 1.5);  % same theory for y
plot(h_dense, std_th_z_dense,  '-', 'Color', cz, 'LineWidth', 1.5);

plot(h_plot, 1000*std_emp(1, valid), 'o', 'Color', cx, 'MarkerFaceColor', cx, 'MarkerSize', 7);
plot(h_plot, 1000*std_emp(2, valid), 's', 'Color', cy, 'MarkerFaceColor', cy, 'MarkerSize', 7);
plot(h_plot, 1000*std_emp(3, valid), 'd', 'Color', cz, 'MarkerFaceColor', cz, 'MarkerSize', 7);

set(ax1, 'XScale', 'log', 'Box', 'on');
xlabel('h [\mum]'); ylabel('std [nm]');
legend({'theory x (\parallel)', 'theory y (\parallel)', 'theory z (\perp)', ...
        'emp x', 'emp y', 'emp z'}, ...
       'Location', 'northoutside', 'Orientation', 'horizontal', 'NumColumns', 3);
xlim([2.3, 55]);

% Panel 2: ratio per bin, horizontal line at 1
ax2 = subplot(2, 1, 2); hold on;
plot(h_plot, ratio_x(valid), 'o-', 'Color', cx, 'MarkerFaceColor', cx, 'LineWidth', 1.2);
plot(h_plot, ratio_y(valid), 's-', 'Color', cy, 'MarkerFaceColor', cy, 'LineWidth', 1.2);
plot(h_plot, ratio_z(valid), 'd-', 'Color', cz, 'MarkerFaceColor', cz, 'LineWidth', 1.2);
yline(1.0, 'k:', 'LineWidth', 1.0);
yline(1.1, 'k--', 'LineWidth', 0.6, 'Alpha', 0.4);
yline(1.5, 'k--', 'LineWidth', 0.6, 'Alpha', 0.4);

set(ax2, 'XScale', 'log', 'Box', 'on');
xlabel('h [\mum]'); ylabel('empirical / theoretical');
xlim([2.3, 55]);
ylim([0.5, 2.5]);

% ---- Save ----
fig_path = 'reference/for_test/fig_p2_h_bin.png';
exportgraphics(gcf, fig_path, 'Resolution', 150);
fprintf('\nFigure saved: %s\n', fig_path);

mat_path = 'test_results/verify/p2_h_bin.mat';
save(mat_path, 'bin_edges', 'bin_center', 'bin_count', ...
     'std_emp', 'mean_emp', 'std_theory_xy', 'std_theory_z', ...
     'ratio_x', 'ratio_y', 'ratio_z', ...
     'c_para_bin', 'c_perp_bin', ...
     'overall_emp_z_nm', 'overall_th_z_nm', 'overall_ratio', ...
     'C_dpmr', 'a_nom', 'sigma2_dXT', 'R');
fprintf('Data saved: %s\n', mat_path);
