%% phase3b_acov_sweep.m — Sweep a_cov to find optimal IIR smoothing
%
% Phase 3b's multi-lag test suggested that block sample variance achieves
% ~14% rel std vs IIR (a_cov=0.05) ~43%. This script reruns the IIR
% smoother offline on the Phase 2A data with varying a_cov to confirm the
% rel std improves with smaller a_cov, and documents the bias/variance
% trade-off.

clear; close all; clc;

[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
project_root = fileparts(script_dir);
cd(project_root);

fprintf('===== phase3b_acov_sweep =====\n\n');

%% Load Phase 2A data
p2a = load(fullfile(project_root, 'test_results', 'verify', 'phase2_chisquared_mc.mat'));
r2a = p2a.results;
del_pmr = r2a.del_pmr;
a_nom = r2a.a_nom;
C_dpmr_eff = r2a.C_dpmr_eff;
ss = r2a.ss_start:size(del_pmr, 2);
N = size(del_pmr, 2);

constants = physical_constants();
addpath(fullfile(project_root, 'model'));
addpath(fullfile(project_root, 'model', 'config'));
k_B = constants.k_B; T_temp = constants.T;
Ts = constants.Ts;

%% Sweep a_cov
a_pd = 0.05; a_prd = 0.05;   % keep HP filter fixed
a_cov_list = [0.005, 0.01, 0.02, 0.03, 0.05, 0.1];
fprintf('Sweep a_cov = [%s]\n\n', sprintf('%.3f ', a_cov_list));

fprintf('  a_cov    N_eff   mean(a_m)/a_nom  std(a_m)  rel_std    bias     tau_resp [ms]\n');
fprintf('  -----    -----   ---------------  --------  --------   ------   -------------\n');

results = [];
for i = 1:length(a_cov_list)
    a_cov = a_cov_list(i);
    N_eff = 1/a_cov;
    tau_ms = N_eff * Ts * 1000;   % response time constant

    % IIR variance
    del_pmrd = zeros(3, N);
    del_pmr2_avg = zeros(3, N);
    for k = 2:N
        del_pmrd(:,k) = (1-a_prd)*del_pmrd(:,k-1) + a_prd*del_pmr(:,k);
        del_pmr2_avg(:,k) = (1-a_cov)*del_pmr2_avg(:,k-1) + a_cov*del_pmr(:,k).^2;
    end
    Var_est = max(del_pmr2_avg - del_pmrd.^2, 0);
    a_m = max(Var_est / (C_dpmr_eff * 4 * k_B * T_temp), 0);

    % Statistics over the ss window (z-axis)
    m = mean(a_m(3, ss));
    s = std(a_m(3, ss));
    rel_std = s/m * 100;
    bias_pct = 100*(m - a_nom)/a_nom;

    fprintf('  %5.3f   %5.0f    %9.4f      %.3e   %6.2f%%   %+6.2f%%   %7.1f\n', ...
        a_cov, N_eff, m/a_nom, s, rel_std, bias_pct, tau_ms);

    results(i).a_cov = a_cov;
    results(i).mean = m;
    results(i).std = s;
    results(i).rel_std = rel_std;
    results(i).bias = bias_pct;
    results(i).tau_ms = tau_ms;
end

%% Also test block sample variance at various window lengths
fprintf('\nBlock sample variance (non-IIR):\n');
fprintf('  wlen (samples)  wlen (ms)  rel_std   mean_a_m/a_nom\n');
for wl = [100, 200, 500, 1000, 2000, 5000]
    n_win = floor(length(ss) / wl);
    var_wins = zeros(n_win, 1);
    for w = 1:n_win
        idx = ss((w-1)*wl+1 : w*wl);
        var_wins(w) = var(del_pmr(3, idx));
    end
    a_m_block = var_wins / (C_dpmr_eff * 4 * k_B * T_temp);
    m = mean(a_m_block);
    rel_std_b = std(a_m_block)/m * 100;
    fprintf('  %6d          %8.1f   %6.2f%%   %.4f\n', wl, wl*Ts*1000, rel_std_b, m/a_nom);
end

%% Save
out_dir = fullfile(project_root, 'test_results', 'verify');
save(fullfile(out_dir, 'phase3b_acov_sweep.mat'), 'results');

fprintf('\nDone.\n');
