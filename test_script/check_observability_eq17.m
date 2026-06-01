function check_observability_eq17()
%CHECK_OBSERVABILITY_EQ17  Math-layer observability verification.
%
% Verifies the analytical conclusions in
% reference/eq17_analysis/design.md Section 7 for both:
%
%   * 5-state baseline (1st-order random walk for x_D and a_x)
%       state = [delta_x_1, delta_x_2, delta_x_3, x_D, a_x]'
%
%   * 7-state architecture (2nd-order = integrated random walk)
%       state = [delta_x_1, delta_x_2, delta_x_3, x_D, delta_xD, a_x, delta_a]'
%
% Config B (dual measurement: H = [delta_x_m; a_xm]) is the design
% target. Both should produce rank(O) = n_state for any f_d sequence.
%
% Outputs:
%   * Console: per-case pass/fail tables for both configurations
%   * test_results/eq17_analysis/task01_observability_results.mat

clear; close all; clc;

[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
project_root = fileparts(script_dir);

%% Common parameters
lambda_c = 0.7;

%% Part 1: 5-state (historical Task 01 baseline)
fprintf('=== Part 1: 5-state (1st-order random walk) ===\n');
fprintf('Window N = 6, lambda_c = %.2f, n_state = 5\n\n', lambda_c);

H_A = [1 0 0 0 0];
H_B5 = [1 0 0 0 0; 0 0 0 0 1];

cases_5 = struct( ...
    'id', {}, 'name', {}, 'f_d_seq', {}, 'H', {}, 'expect', {});

cases_5(end+1).id      = 'A1';
cases_5(end).name      = 'Config A, f_d = 0 (positioning hold)';
cases_5(end).f_d_seq   = zeros(1, 6);
cases_5(end).H         = H_A;
cases_5(end).expect    = 4;

cases_5(end+1).id      = 'A2';
cases_5(end).name      = 'Config A, f_d = 2 (constant non-zero)';
cases_5(end).f_d_seq   = 2 * ones(1, 6);
cases_5(end).H         = H_A;
cases_5(end).expect    = 4;

cases_5(end+1).id      = 'A3';
cases_5(end).name      = 'Config A, f_d = linear ramp 1..6';
cases_5(end).f_d_seq   = 1:6;
cases_5(end).H         = H_A;
cases_5(end).expect    = 5;

cases_5(end+1).id      = 'A4';
cases_5(end).name      = 'Config A, f_d = sinusoidal sin(k*pi/3)';
cases_5(end).f_d_seq   = sin((1:6) * pi / 3);
cases_5(end).H         = H_A;
cases_5(end).expect    = 5;

% Window-vs-PE boundary cases. For N=6, only f_d_seq(1..N-3)=(1..3) affects
% the (x_D, a_x) sub-block of O. So a *single* variation must fall within
% indices 1..3 to grant observability; a variation at index >=4 does not
% help at this window length. A5/A6/A7 form a controlled sweep of this
% boundary.

cases_5(end+1).id      = 'A5';
cases_5(end).name      = 'Config A, early variation [1 2 1 1 1 1] (idx 2 in PE window)';
cases_5(end).f_d_seq   = [1 2 1 1 1 1];
cases_5(end).H         = H_A;
cases_5(end).expect    = 5;

cases_5(end+1).id      = 'A6';
cases_5(end).name      = 'Config A, mid variation [1 1 2 1 1 1] (idx 3 in PE window)';
cases_5(end).f_d_seq   = [1 1 2 1 1 1];
cases_5(end).H         = H_A;
cases_5(end).expect    = 5;

cases_5(end+1).id      = 'A7';
cases_5(end).name      = 'Config A, late variation [1 1 1 2 1 1] (idx 4 outside PE window)';
cases_5(end).f_d_seq   = [1 1 1 2 1 1];
cases_5(end).H         = H_A;
cases_5(end).expect    = 4;

cases_5(end+1).id      = 'B1';
cases_5(end).name      = 'Config B, f_d = 0 (positioning hold)';
cases_5(end).f_d_seq   = zeros(1, 6);
cases_5(end).H         = H_B5;
cases_5(end).expect    = 5;

cases_5(end+1).id      = 'B2';
cases_5(end).name      = 'Config B, f_d = 2 (constant non-zero)';
cases_5(end).f_d_seq   = 2 * ones(1, 6);
cases_5(end).H         = H_B5;
cases_5(end).expect    = 5;

cases_5(end+1).id      = 'B3';
cases_5(end).name      = 'Config B, f_d = linear ramp 1..6';
cases_5(end).f_d_seq   = 1:6;
cases_5(end).H         = H_B5;
cases_5(end).expect    = 5;

[results_5state, all_pass_5] = run_cases(cases_5, @build_F_e_5state, 5, lambda_c);

%% Part 2: 7-state architecture (current design target, Config B only)
fprintf('\n\n=== Part 2: 7-state (2nd-order = integrated random walk) ===\n');
fprintf('Window N = 7, lambda_c = %.2f, n_state = 7, dual feedback only\n\n', lambda_c);

H_B7 = [1 0 0 0 0 0 0; 0 0 0 0 0 1 0];

cases_7 = struct( ...
    'id', {}, 'name', {}, 'f_d_seq', {}, 'H', {}, 'expect', {});

cases_7(end+1).id      = '7B1';
cases_7(end).name      = '7-state Config B, f_d = 0 (most stringent: hold + no PE)';
cases_7(end).f_d_seq   = zeros(1, 7);
cases_7(end).H         = H_B7;
cases_7(end).expect    = 7;

cases_7(end+1).id      = '7B2';
cases_7(end).name      = '7-state Config B, f_d = 2 (constant non-zero)';
cases_7(end).f_d_seq   = 2 * ones(1, 7);
cases_7(end).H         = H_B7;
cases_7(end).expect    = 7;

cases_7(end+1).id      = '7B3';
cases_7(end).name      = '7-state Config B, f_d = ramp 1..7';
cases_7(end).f_d_seq   = 1:7;
cases_7(end).H         = H_B7;
cases_7(end).expect    = 7;

[results_7state, all_pass_7] = run_cases(cases_7, @build_F_e_7state, 7, lambda_c);

% Inspect the most stringent 7-state case
idx = find(strcmp({results_7state.id}, '7B1'), 1);
if ~isempty(idx)
    fprintf('\n--- 7B1 O matrix (f_d=0, the hardest test) -- rank = %d ---\n', ...
        results_7state(idx).rank_O);
    disp_O(results_7state(idx).O);
end

%% Save results
output_dir = fullfile(project_root, 'test_results', 'eq17_analysis');
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end
save_path = fullfile(output_dir, 'task01_observability_results.mat');
save(save_path, 'results_5state', 'results_7state', ...
                'all_pass_5', 'all_pass_7', 'lambda_c');

%% Overall summary
fprintf('\n=== Overall Summary ===\n');
if all_pass_5
    fprintf('  5-state: ALL %d cases PASS\n', numel(cases_5));
else
    fprintf('  5-state: SOME CASES FAILED\n');
end
if all_pass_7
    fprintf('  7-state: ALL %d cases PASS\n', numel(cases_7));
else
    fprintf('  7-state: SOME CASES FAILED\n');
end
fprintf('  Results saved to: %s\n', save_path);

end

%% ========== Helpers ==========

function [results, all_pass] = run_cases(cases, build_F_e_fn, n_state, lambda_c)
%RUN_CASES  Loop test cases, build O, check rank vs expectation.

results = struct( ...
    'id', {}, 'name', {}, 'f_d_seq', {}, 'rank_O', {}, ...
    'expect', {}, 'pass', {}, 'O', {});

all_pass = true;

for i = 1:numel(cases)
    c = cases(i);
    O = build_obs_matrix(c.f_d_seq, c.H, build_F_e_fn, lambda_c, n_state);
    r = rank(O);
    pass = (r == c.expect);

    results(end+1).id      = c.id;
    results(end).name      = c.name;
    results(end).f_d_seq   = c.f_d_seq;
    results(end).rank_O    = r;
    results(end).expect    = c.expect;
    results(end).pass      = pass;
    results(end).O         = O;

    if pass
        status = '   PASS';
    else
        status = '** FAIL';
        all_pass = false;
    end
    fprintf('[%s] %-3s  rank = %d (expect %d)  %s\n', ...
        status, c.id, r, c.expect, c.name);
end

end

function O = build_obs_matrix(f_d_seq, H, build_F_e_fn, lambda_c, n_state)
%BUILD_OBS_MATRIX  LTV observability matrix construction.
%
%   O = [ H * I              ;
%         H * F_e[k_0]       ;
%         H * F_e[k_0+1] * F_e[k_0] ;
%         ...                                ]
%
%   F_e[k] is constructed via build_F_e_fn(f_d_seq(m), lambda_c).
%   Note that f_d_seq(N) is computed but the resulting Phi is not
%   multiplied by H again, so it does not affect O.

N = length(f_d_seq);
Phi = eye(n_state);
O = zeros(0, n_state);

for m = 1:N
    O = [O; H * Phi];                        %#ok<AGROW>
    F_e = build_F_e_fn(f_d_seq(m), lambda_c);
    Phi = F_e * Phi;
end

end

function F = build_F_e_5state(f_d, lambda_c)
%BUILD_F_E_5STATE  5x5 augmented system matrix (1st-order random walk).
%
%   State: [delta_x_1, delta_x_2, delta_x_3, x_D, a_x]'
%   Time-varying entry: F(3,5) = -f_d.

F = [0 1 0        0  0;
     0 0 1        0  0;
     0 0 lambda_c -1 -f_d;
     0 0 0        1  0;
     0 0 0        0  1];

end

function F = build_F_e_7state(f_d, lambda_c)
%BUILD_F_E_7STATE  7x7 augmented system matrix (2nd-order / integrated RW).
%
%   State: [delta_x_1, delta_x_2, delta_x_3, x_D, delta_xD, a_x, delta_a]'
%   Time-varying entry: F(3,6) = -f_d.
%
%   Cross-couplings:
%       F(4,5) = 1  (x_D[k+1] = x_D[k] + delta_xD[k])
%       F(6,7) = 1  (a_x[k+1] = a_x[k] + delta_a[k])

F = zeros(7);
F(1, 2) = 1;
F(2, 3) = 1;
F(3, 3) = lambda_c;
F(3, 4) = -1;
F(3, 6) = -f_d;
F(4, 4) = 1;  F(4, 5) = 1;
F(5, 5) = 1;
F(6, 6) = 1;  F(6, 7) = 1;
F(7, 7) = 1;

end

function disp_O(O)
%DISP_O  Compact print of an observability matrix.
fmt = '%8.4f ';
for r = 1:size(O, 1)
    fprintf(fmt, O(r, :));
    fprintf('\n');
end

end
