function check_observability_eq17()
%CHECK_OBSERVABILITY_EQ17 Math-layer observability verification for Eq.17 + 5-state EKF.
%
% Verifies the analytical conclusions in
% reference/eq17_analysis/design.md Section 7:
%
%   Config A (single measurement: H_A = [1 0 0 0 0]):
%       rank(O) = 4 when f_d is constant in the PE window (incl. f_d = 0)
%       rank(O) = 5 when f_d varies in the PE window
%       PE window = f_d_seq(1..N-3); variations at indices >= N-2 do not
%       help at window length N (see test cases A5..A7).
%
%   Config B (dual measurement: H_B = [1 0 0 0 0; 0 0 0 0 1]):
%       rank(O) = 5 unconditionally (no PE required, mathematical layer)
%
% Notes:
%   * Window length N = 6. The construction uses F_e built from
%     f_d_seq(1)..f_d_seq(N-1); f_d_seq(N) is computed but does not
%     affect the final O matrix.
%   * Config B assumes nominal a_xm measurement. IIR-induced R_2 effects
%     (warm-up, low SNR, fast a_x variation, near-wall) can degrade B
%     toward A behaviour at the *practical* (Gramian) layer; that is the
%     subject of Task 02, not this rank test.
%
% Outputs:
%   * Console: per-case rank vs expectation, plus selected O matrices.
%   * test_results/eq17_analysis/task01_observability_results.mat (gitignored)

clear; close all; clc;

[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
project_root = fileparts(script_dir);

%% System parameters
lambda_c = 0.7;                          % closed-loop pole
N        = 6;                            % observation window length
n_state  = 5;

H_A = [1 0 0 0 0];                       % Config A: y = delta_x_m
H_B = [1 0 0 0 0; 0 0 0 0 1];            % Config B: y = [delta_x_m; a_xm]

%% Test cases
cases = struct( ...
    'id',      {}, ...
    'name',    {}, ...
    'f_d_seq', {}, ...
    'H',       {}, ...
    'expect',  {});

cases(end+1).id      = 'A1';
cases(end).name      = 'Config A, f_d = 0 (positioning hold)';
cases(end).f_d_seq   = zeros(1, N);
cases(end).H         = H_A;
cases(end).expect    = 4;

cases(end+1).id      = 'A2';
cases(end).name      = 'Config A, f_d = 2 (constant non-zero)';
cases(end).f_d_seq   = 2 * ones(1, N);
cases(end).H         = H_A;
cases(end).expect    = 4;

cases(end+1).id      = 'A3';
cases(end).name      = 'Config A, f_d = linear ramp 1..N';
cases(end).f_d_seq   = 1:N;
cases(end).H         = H_A;
cases(end).expect    = 5;

cases(end+1).id      = 'A4';
cases(end).name      = 'Config A, f_d = sinusoidal sin(k*pi/3)';
cases(end).f_d_seq   = sin((1:N) * pi / 3);
cases(end).H         = H_A;
cases(end).expect    = 5;

% Window-vs-PE boundary cases. For N=6, only f_d_seq(1..N-3)=(1..3) affects
% the (x_D, a_x) sub-block of O. So a *single* variation must fall within
% indices 1..3 to grant observability; a variation at index >=4 does not
% help at this window length. A5/A6/A7 form a controlled sweep of this
% boundary.

cases(end+1).id      = 'A5';
cases(end).name      = 'Config A, early variation [1 2 1 1 1 1] (idx 2 in PE window)';
cases(end).f_d_seq   = [1 2 1 1 1 1];
cases(end).H         = H_A;
cases(end).expect    = 5;

cases(end+1).id      = 'A6';
cases(end).name      = 'Config A, mid variation [1 1 2 1 1 1] (idx 3 in PE window)';
cases(end).f_d_seq   = [1 1 2 1 1 1];
cases(end).H         = H_A;
cases(end).expect    = 5;

cases(end+1).id      = 'A7';
cases(end).name      = 'Config A, late variation [1 1 1 2 1 1] (idx 4 outside PE window for N=6)';
cases(end).f_d_seq   = [1 1 1 2 1 1];
cases(end).H         = H_A;
cases(end).expect    = 4;

cases(end+1).id      = 'B1';
cases(end).name      = 'Config B, f_d = 0 (positioning hold)';
cases(end).f_d_seq   = zeros(1, N);
cases(end).H         = H_B;
cases(end).expect    = 5;

cases(end+1).id      = 'B2';
cases(end).name      = 'Config B, f_d = 2 (constant non-zero)';
cases(end).f_d_seq   = 2 * ones(1, N);
cases(end).H         = H_B;
cases(end).expect    = 5;

cases(end+1).id      = 'B3';
cases(end).name      = 'Config B, f_d = linear ramp 1..N';
cases(end).f_d_seq   = 1:N;
cases(end).H         = H_B;
cases(end).expect    = 5;

%% Verification loop
fprintf('=== Math-layer Observability Verification ===\n');
fprintf('Window N = %d, lambda_c = %.2f, n_state = %d\n\n', N, lambda_c, n_state);

results = struct( ...
    'id',      {}, ...
    'name',    {}, ...
    'f_d_seq', {}, ...
    'rank_O',  {}, ...
    'expect',  {}, ...
    'pass',    {}, ...
    'O',       {});

all_pass = true;

for i = 1:numel(cases)
    c = cases(i);
    O = build_obs_matrix(c.f_d_seq, c.H, lambda_c);
    r = rank(O);
    pass = (r == c.expect);

    results(end+1).id    = c.id;
    results(end).name    = c.name;
    results(end).f_d_seq = c.f_d_seq;
    results(end).rank_O  = r;
    results(end).expect  = c.expect;
    results(end).pass    = pass;
    results(end).O       = O;

    if pass
        status = '   PASS';
    else
        status = '** FAIL';
        all_pass = false;
    end

    fprintf('[%s] %-3s  rank = %d (expect %d)  %s\n', ...
        status, c.id, r, c.expect, c.name);
end

fprintf('\n');
if all_pass
    fprintf('=== ALL %d cases PASS ===\n', numel(cases));
else
    fprintf('=== SOME CASES FAILED ===\n');
end

%% Inspection: print O matrix for representative cases
inspect_ids = {'A1', 'A4', 'B1'};
for i = 1:numel(inspect_ids)
    idx = find(strcmp({results.id}, inspect_ids{i}), 1);
    if isempty(idx); continue; end
    r = results(idx);
    fprintf('\n--- O matrix for %s (%s) -- rank = %d ---\n', r.id, r.name, r.rank_O);
    disp_O(r.O);
end

%% Save results
output_dir = fullfile(project_root, 'test_results', 'eq17_analysis');
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end
save_path = fullfile(output_dir, 'task01_observability_results.mat');
save(save_path, 'results', 'lambda_c', 'N', 'cases', 'all_pass');
fprintf('\nResults saved to: %s\n', save_path);

end

%% ========== Helper functions ==========

function O = build_obs_matrix(f_d_seq, H, lambda_c)
%BUILD_OBS_MATRIX  LTV observability matrix.
%
%   O = [ H * I              ;
%         H * F_e[k_0]       ;
%         H * F_e[k_0+1] * F_e[k_0] ;
%         ...                                ]
%
%   F_e[k] depends on f_d_seq(m) for m = 1..N. Note that f_d_seq(N) is
%   never used in the final O (because the last F_e is computed but the
%   resulting Phi is not multiplied by H again). This is preserved to
%   keep the loop simple; callers are expected to know that f_d_seq(N) is
%   informational only.

n_state = 5;
N = length(f_d_seq);
Phi = eye(n_state);
O = zeros(0, n_state);

for m = 1:N
    O = [O; H * Phi];                        %#ok<AGROW>
    F_e = build_F_e(f_d_seq(m), lambda_c);
    Phi = F_e * Phi;
end

end

function F_e = build_F_e(f_d, lambda_c)
%BUILD_F_E  5x5 augmented system transition matrix for Eq.17 + 5-state EKF.
%
%   State: [delta_x_1, delta_x_2, delta_x_3, x_D, a_x]'
%   Time-varying entry: F_e(3,5) = -f_d.

F_e = [0 1 0        0  0;
       0 0 1        0  0;
       0 0 lambda_c -1 -f_d;
       0 0 0        1  0;
       0 0 0        0  1];

end

function disp_O(O)
%DISP_O  Compact print of an observability matrix.
fmt = '%8.4f ';
for r = 1:size(O, 1)
    fprintf(fmt, O(r, :));
    fprintf('\n');
end

end
