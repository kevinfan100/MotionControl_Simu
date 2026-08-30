% L2 LOO battery -- run the five paired arms (seeds 1:100) and save the
% per-channel N x 3 x nseeds arrays under test_results/l2_loo/.
% Pre-registration: test_script/scratch/l2_loo_prereg.txt (written first).
% STATUS: ACTIVE (scratch, L2 instrument) | PURPOSE: LOO re-verification of
%   y2_echo_corr / ma2_aug / fe_row4_full on the formC_b deep base |
%   EXPIRES: stacked-fix audit closes.
% Usage: define ARMS_TO_RUN (cell of arm names) before run(), else all five.
% The baseline arm checks seed 7 bit-for-bit against the L0 fixture BEFORE
% saving; a mismatch aborts the whole battery.
cd('/Users/kevin/code/MotionControl_Simu-motion-test');
root = pwd;
p = strsplit(genpath(fullfile(root, 'model')), pathsep);
p = p(~cellfun('isempty', p)); p = p(~contains(p, [filesep 'archive']));
addpath(p{:});
addpath(fullfile(root, 'test_script', 'integration'));
addpath(fullfile(root, 'test_script', 'build_helpers'));
addpath(fullfile(root, 'test_script', 'scratch'));

SEEDS   = 1:100;
OUT_DIR = fullfile(root, 'test_results', 'l2_loo');
if ~exist(OUT_DIR, 'dir'); mkdir(OUT_DIR); end
FIX_FILE = fullfile(root, 'test_results', 'l0_fixture', 'fixture_formC_b.mat');

F = {'a_bar_hat_out', 'a_true_out', 'a_xm_out', 'R2_out', 'innov_y1_out', ...
     'innov_y2_out', 'K_a_y1_out', 'K_a_y2_out', 'K_b_y1_out', 'K_b_y2_out', ...
     'b_hat_out', 'P_b_out', 'P_a_out', 'gate_out', 'h_bar_d_out', ...
     'a_hat_out', 'P41_out', 'h_bar_true_out'};

ALL = { ...
    'base',     struct(); ...
    'echo_off', struct('y2_echo_corr', false); ...
    'ma2_off',  struct('ma2_aug', false); ...
    'fe4_off',  struct('fe_row4_full', false); ...
    'both_off', struct('y2_echo_corr', false, 'ma2_aug', false)};
if ~exist('ARMS_TO_RUN', 'var'); ARMS_TO_RUN = ALL(:, 1).'; end

for a = 1:size(ALL, 1)
    name = ALL{a, 1};
    if ~any(strcmp(name, ARMS_TO_RUN)); continue; end
    ov = ALL{a, 2};
    clear run_formC_b motion_control_law_formC_b;
    fprintf('\n=== L2 LOO ARM %s : override %s ===\n', name, jsonencode(ov));
    opts = struct('seeds', SEEDS, 'arm', 'best', 'ctrl_const_override', ov);
    tic; D = run_formC_b(opts); el = toc;

    ns = numel(D.runs); N = numel(D.runs{1}.tout);
    S = struct();
    for i = 1:numel(F)
        nc = size(D.runs{1}.(F{i}), 2);
        S.(F{i}) = zeros(N, nc, ns);
        for s = 1:ns; S.(F{i})(:, :, s) = double(D.runs{s}.(F{i})); end
    end
    S.t = D.runs{1}.tout(:); S.K = D.runs{1}.ctrl_const; S.ctrl_const = S.K;
    S.a_nom = D.runs{1}.a_nom; S.seeds = SEEDS; S.arm = name; S.override = ov;
    S.metrics = D.metrics; S.cfg = D.cfg;
    S.windows = D.metrics.windows;

    if strcmp(name, 'base')
        L = load(FIX_FILE); ref = L.fixture.data.seed7.a_bar_hat_out;
        cur = D.runs{find(SEEDS == 7, 1)}.a_bar_hat_out;
        if ~isequal(size(cur), size(ref)); d = Inf; else; d = max(abs(cur(:) - ref(:))); end
        fprintf('FIXTURE CHECK seed 7 a_bar_hat_out: max|diff| = %.3e (fixture commit %s)\n', ...
                d, L.fixture.commit(1:7));
        S.fixture_maxdiff = d;
        if d ~= 0
            save(fullfile(OUT_DIR, 'l2_base_100_FAILED_FIXTURE.mat'), '-struct', 'S', '-v7.3');
            error('l2_loo:fixture', 'baseline seed 7 is NOT bit-identical to the fixture (max|diff| %.3e). STOP.', d);
        end
        fprintf('FIXTURE CHECK: PASS (bit-identical)\n');
    end
    fn = fullfile(OUT_DIR, sprintf('l2_%s_100.mat', name));
    save(fn, '-struct', 'S', '-v7.3');
    fprintf('ARM DONE %s: %d seeds, %.1f s (%.2f s/seed) -> %s\n', name, ns, el, el/ns, fn);
end
fprintf('\nL2 LOO ARMS COMPLETE\n');
