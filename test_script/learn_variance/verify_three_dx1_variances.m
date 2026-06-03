%% verify_three_dx1_variances.m
% Empirical verification of variance ordering for the three views of δx[k-d]:
%   (1) delta_x_m  (paper "δx_m", code measurement, with sensor noise)
%   (2) δx_1       (paper Eq.14 true state, noise-free)
%   (3) δx̂_1      (paper Eq.17 estimator state, KF-filtered)
%
% Theory (memory: project_three_dx1_signals):
%   delta_x_m[k] = δx_1[k] − n[k-d]   (n ⊥ δx_1 at the same lag)
%   δx̂_1[k]    = δx_1[k] − e_1[k]   (KF orthogonality: Cov(δx̂_1, e_1)=0)
% =>  Var(delta_x_m) = Var(δx_1) + σ²_n              ... (A)
%     Var(δx̂_1)    = Var(δx_1) − P_11_ss            ... (B)
%
% Setup: positioning at h_init=50um, T_sim=5s, 3 seeds.
% Steady-state window: t > 1s.

clear; clc;

% --- Path setup ---
[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
project_root = fileparts(fileparts(script_dir));
addpath(fullfile(project_root, 'model'));
addpath(fullfile(project_root, 'model', 'config'));
addpath(fullfile(project_root, 'model', 'wall_effect'));
addpath(fullfile(project_root, 'model', 'thermal_force'));
addpath(fullfile(project_root, 'model', 'trajectory'));
addpath(fullfile(project_root, 'model', 'controller'));
addpath(fullfile(project_root, 'model', 'dual_track'));
addpath(fullfile(project_root, 'model', 'diag'));

seeds = [1, 2, 3];
T_sim = 5.0;
t_ss  = 1.0;

% --- Base config: positioning h=50 ---
config = user_config();
config.h_init            = 50;
config.h_bottom          = 50;
config.amplitude         = 0;
config.trajectory_type   = 'positioning';
config.T_sim             = T_sim;
config.ctrl_enable       = true;
config.meas_noise_enable = true;
config.thermal_enable    = true;
config.lambda_c          = 0.7;
config.a_pd              = 0.05;
config.a_cov             = 0.05;
config.sigma2_w_fD       = 0;
config.meas_noise_std    = [0.00062; 0.000057; 0.00331];   % [x;y;z], um (matches existing tests)
config.controller_type   = 7;

% σ²_n per axis (in um²) — direct from config (sensor noise variance)
sigma2_n = config.meas_noise_std(:).^2;     % 3x1

n_seeds = numel(seeds);
ax_names = {'x', 'y', 'z'};

% Per-seed storage (rows=seed, cols=axis x,y,z)
var_dxm     = zeros(n_seeds, 3);    % empirical Var(delta_x_m)
var_dx1     = zeros(n_seeds, 3);    % empirical Var(δx_1) reconstructed = Var(dxm)−σ²_n
var_dx1_hat = zeros(n_seeds, 3);    % empirical Var(δx̂_1)
P11_ss      = zeros(n_seeds, 3);    % KF posterior P(1,1) at steady-state
P_dx_ss     = zeros(n_seeds, 3);    % P(3,3) for reference
P_a_ss      = zeros(n_seeds, 3);    % P(6,6)

for s = 1:n_seeds
    seed = seeds(s);
    run_opts.seed         = seed;
    run_opts.verbose      = false;
    run_opts.collect_diag = true;

    fprintf('seed %d/%d (=%d) ... ', s, n_seeds, seed);
    t0 = tic;
    simOut = run_pure_simulation(config, run_opts);
    fprintf('done (%.1fs)\n', toc(t0));

    tout = simOut.tout;
    idx_ss = (tout >= t_ss);

    dxm        = simOut.diag.delta_x_m;       % [N x 3]  → x,y,z per axis
    dx1_hat    = simOut.diag.delta_x_hat_1;   % [N x 3]
    P11        = simOut.diag.P_dx1;           % [N x 3]
    P_dx_log   = simOut.diag.P_dx;            % [N x 3]
    P_a_log    = simOut.diag.P_a;             % [N x 3]

    % Empirical variances (steady-state, sample variance)
    var_dxm(s, :)     = var(dxm(idx_ss, :),     0, 1);
    var_dx1_hat(s, :) = var(dx1_hat(idx_ss, :), 0, 1);
    var_dx1(s, :)     = var_dxm(s, :) - sigma2_n.';   % reconstruction via Eq.(A)

    P11_ss(s, :)  = mean(P11(idx_ss, :),      1);
    P_dx_ss(s, :) = mean(P_dx_log(idx_ss, :), 1);
    P_a_ss(s, :)  = mean(P_a_log(idx_ss, :),  1);
end

% --- Aggregate across seeds (mean) ---
v_dxm_m     = mean(var_dxm,     1);
v_dx1_m     = mean(var_dx1,     1);
v_dx1hat_m  = mean(var_dx1_hat, 1);
P11_m       = mean(P11_ss, 1);

% --- Report ---
fprintf('\n');
fprintf('================================================================\n');
fprintf('  Verification: Var ordering of δx[k-d] three views (h=50, %d seeds)\n', n_seeds);
fprintf('================================================================\n');
fprintf('sigma2_n (sensor noise variance) per axis [um^2]:\n');
fprintf('  x: %.4e   y: %.4e   z: %.4e\n\n', sigma2_n(1), sigma2_n(2), sigma2_n(3));

fprintf('Per-axis variances (mean across %d seeds) [um^2]:\n', n_seeds);
fprintf('%-30s %-13s %-13s %-13s\n', 'Quantity', 'x', 'y', 'z');
fprintf('%s\n', repmat('-', 1, 75));
fprintf('%-30s %-13.4e %-13.4e %-13.4e\n', 'Var(delta_x_m)  measurement', v_dxm_m(1), v_dxm_m(2), v_dxm_m(3));
fprintf('%-30s %-13.4e %-13.4e %-13.4e\n', 'Var(δx_1)       true (recon)', v_dx1_m(1), v_dx1_m(2), v_dx1_m(3));
fprintf('%-30s %-13.4e %-13.4e %-13.4e\n', 'Var(δx̂_1)      estimate',     v_dx1hat_m(1), v_dx1hat_m(2), v_dx1hat_m(3));
fprintf('%-30s %-13.4e %-13.4e %-13.4e\n', 'P_11 (KF post cov)',           P11_m(1), P11_m(2), P11_m(3));
fprintf('\n');

fprintf('Ordering check (expect Var(dxm) > Var(δx_1) > Var(δx̂_1)):\n');
for a = 1:3
    ok = v_dxm_m(a) > v_dx1_m(a) && v_dx1_m(a) > v_dx1hat_m(a);
    fprintf('  axis %s: %.4e > %.4e > %.4e   [%s]\n', ...
        ax_names{a}, v_dxm_m(a), v_dx1_m(a), v_dx1hat_m(a), ternary(ok, 'OK', 'FAIL'));
end
fprintf('\n');

fprintf('Relation (A) check  Var(dxm) − Var(δx_1) ?= σ²_n:\n');
for a = 1:3
    diff_A = v_dxm_m(a) - v_dx1_m(a);
    rel_err_A = abs(diff_A - sigma2_n(a)) / sigma2_n(a);
    fprintf('  axis %s: diff=%.4e   σ²_n=%.4e   rel_err=%.2e  (by construction)\n', ...
        ax_names{a}, diff_A, sigma2_n(a), rel_err_A);
end
fprintf('  ↑ Note: this holds exactly by construction since Var(δx_1) := Var(dxm) − σ²_n.\n\n');

fprintf('Relation (B) check  Var(δx_1) − Var(δx̂_1) ?= P_11_ss:\n');
for a = 1:3
    diff_B = v_dx1_m(a) - v_dx1hat_m(a);
    rel_err_B = abs(diff_B - P11_m(a)) / max(P11_m(a), 1e-30);
    ratio_B = diff_B / P11_m(a);
    fprintf('  axis %s: diff=%.4e   P_11=%.4e   ratio=%.3f   rel_err=%.2e\n', ...
        ax_names{a}, diff_B, P11_m(a), ratio_B, rel_err_B);
end

fprintf('\nPer-seed raw values (Var × 1e6, i.e. [(nm)^2]) for transparency:\n');
fprintf('%-6s %-10s %-10s %-10s %-10s %-10s %-10s %-10s %-10s %-10s\n', ...
    'seed', 'Vdxm_x','Vdxm_y','Vdxm_z','Vdx1_x','Vdx1_y','Vdx1_z','Vdxh_x','Vdxh_y','Vdxh_z');
for s = 1:n_seeds
    fprintf('%-6d %-10.4f %-10.4f %-10.4f %-10.4f %-10.4f %-10.4f %-10.4f %-10.4f %-10.4f\n', ...
        seeds(s), var_dxm(s,:)*1e6, var_dx1(s,:)*1e6, var_dx1_hat(s,:)*1e6);
end

fprintf('\n=== Done ===\n');

% ---- helpers ----
function s = ternary(cond, a, b)
    if cond, s = a; else, s = b; end
end
