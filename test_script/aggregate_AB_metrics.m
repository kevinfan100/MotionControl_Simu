% aggregate_AB_metrics.m
% Read all test_results/verify/AB_run_*.mat from parallel runs and print
% aggregate statistics for scenarios A (h=50) and B (h=2.5).

[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
project_root = fileparts(script_dir);
cd(project_root);

files = dir('test_results/verify/AB_run_*.mat');
if isempty(files)
    error('No AB_run_*.mat files found');
end

A = struct('track_x_mean_nm', [], 'track_x_std_nm', [], ...
           'track_z_mean_nm', [], 'track_z_std_nm', [], ...
           'ahat_x_mean_pct', [], 'ahat_x_std_pct', [], ...
           'ahat_z_mean_pct', [], 'ahat_z_std_pct', []);
B = A;
A_meta = []; B_meta = [];

fields = fieldnames(A);
for i = 1:length(files)
    s = load(fullfile(files(i).folder, files(i).name));
    name = files(i).name;
    if abs(s.h_init - 50) < 1e-6
        for j = 1:length(fields), A.(fields{j})(end+1) = s.(fields{j}); end
        if isempty(A_meta)
            A_meta = struct('h_init', s.h_init, 'cpara', s.cpara, 'cperp', s.cperp, ...
                'a_x_true', s.a_x_true, 'a_z_true', s.a_z_true);
        end
    elseif abs(s.h_init - 2.5) < 1e-6
        for j = 1:length(fields), B.(fields{j})(end+1) = s.(fields{j}); end
        if isempty(B_meta)
            B_meta = struct('h_init', s.h_init, 'cpara', s.cpara, 'cperp', s.cperp, ...
                'a_x_true', s.a_x_true, 'a_z_true', s.a_z_true);
        end
    end
end

print_scen = @(name, M, meta) print_one(name, M, meta);

fprintf('\n========== Scenario A: free-space h=50 um ==========\n');
fprintf('  c_para=%.3f c_perp=%.3f a_x_true=%.4e a_z_true=%.4e\n', ...
    A_meta.cpara, A_meta.cperp, A_meta.a_x_true, A_meta.a_z_true);
fprintf('  N seeds = %d\n', length(A.track_x_mean_nm));
print_one(A);

fprintf('\n========== Scenario B: near-wall h=2.5 um ==========\n');
fprintf('  c_para=%.3f c_perp=%.3f a_x_true=%.4e a_z_true=%.4e\n', ...
    B_meta.cpara, B_meta.cperp, B_meta.a_x_true, B_meta.a_z_true);
fprintf('  N seeds = %d\n', length(B.track_x_mean_nm));
print_one(B);

% Save aggregated
out = struct();
out.A = A; out.A_meta = A_meta;
out.B = B; out.B_meta = B_meta;
save('test_results/verify/AB_aggregate.mat', '-struct', 'out');
fprintf('\nSaved test_results/verify/AB_aggregate.mat\n');

function print_one(M)
    fprintf('  Tracking error δx = p_d - p (nm):\n');
    fprintf('    x:   mean = %+6.3f ± %5.3f       std = %5.3f ± %5.3f\n', ...
        mean(M.track_x_mean_nm), std(M.track_x_mean_nm), ...
        mean(M.track_x_std_nm),  std(M.track_x_std_nm));
    fprintf('    z:   mean = %+6.3f ± %5.3f       std = %5.3f ± %5.3f\n', ...
        mean(M.track_z_mean_nm), std(M.track_z_mean_nm), ...
        mean(M.track_z_std_nm),  std(M.track_z_std_nm));
    fprintf('  a_hat error (%% of a_true):\n');
    fprintf('    x:   bias = %+6.3f%% ± %5.3f       std = %5.3f%% ± %5.3f\n', ...
        mean(M.ahat_x_mean_pct), std(M.ahat_x_mean_pct), ...
        mean(M.ahat_x_std_pct),  std(M.ahat_x_std_pct));
    fprintf('    z:   bias = %+6.3f%% ± %5.3f       std = %5.3f%% ± %5.3f\n', ...
        mean(M.ahat_z_mean_pct), std(M.ahat_z_mean_pct), ...
        mean(M.ahat_z_std_pct),  std(M.ahat_z_std_pct));
end
