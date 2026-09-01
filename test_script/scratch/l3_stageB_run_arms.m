% L3 Stage B -- run B0 ('left', regenerated), B1 ('mid') and B2 ('trap'), seeds
% 1:100, arm 'best', canonical deep, paired by seed.
% Registration: test_script/scratch/l3_stageB_prereg.txt (read first).
% STATUS: ACTIVE (scratch, L3 instrument) | PURPOSE: paired 100-seed arms for the
%   predict_quad flag | EXPIRES: Stage B closes.
% Saves the same channel set as l2_loo_run_arms.m (+ delta_x_hat_3_out) under
% test_results/l3_stageB/l3_{left,mid,trap}_100.mat (left = B0 regenerated on the
% current production, see provenance block). Never reads run_formC_b_*.mat.
% B0 gate: seed 7 of the regenerated left arm must be bit-identical to the fixture
% (1a70599) AND to the fresh 'left' run saved by l3_stageB_w234.m.
cd('/Users/kevin/code/MotionControl_Simu-motion-test');
root = pwd;
p = strsplit(genpath(fullfile(root, 'model')), pathsep);
p = p(~cellfun('isempty', p)); p = p(~contains(p, [filesep 'archive']));
addpath(p{:});
addpath(fullfile(root, 'test_script', 'integration'));
addpath(fullfile(root, 'test_script', 'build_helpers'));
addpath(fullfile(root, 'test_script', 'scratch'));

SEEDS   = 1:100;
OUT_DIR = fullfile(root, 'test_results', 'l3_stageB');
if ~exist(OUT_DIR, 'dir'); mkdir(OUT_DIR); end
B0_FILE  = fullfile(root, 'test_results', 'l2_loo', 'l2_base_100.mat');
FIX_FILE = fullfile(root, 'test_results', 'l0_fixture', 'fixture_formC_b.mat');
W234     = fullfile(OUT_DIR, 'l3_w234_seed7.mat');

% ---------------------------------------------------------------- B0 provenance
% The L2 baseline (l2_base_100.mat, 2026-08-25) predates the 1a70599 production
% change (fixture remade 2026-08-27), so it is NOT a valid pair for the current
% code. Report its distance; B0 is REGENERATED below as the 'left' arm under
% test_results/l3_stageB/l3_left_100.mat and gated against the fixture.
m0 = matfile(B0_FILE);
s0 = m0.seeds;  i7 = find(s0 == 7, 1);  assert(~isempty(i7), 'seed 7 not in L2 base');
b0_7 = m0.a_bar_hat_out(:, :, i7);
FX = load(FIX_FILE);  fx_7 = FX.fixture.data.seed7.a_bar_hat_out;
Wx = load(W234, 'left');  fr_7 = Wx.left.a_bar_hat_out;
fprintf('L2 BASE PROVENANCE seed 7 a_bar_hat_out: |l2_base - fixture(1a70599)| = %.3e ; |l2_base - fresh left run (post-patch)| = %.3e ; |fixture - fresh left| = %.3e\n', ...
        max(abs(b0_7(:) - fx_7(:))), max(abs(b0_7(:) - fr_7(:))), max(abs(fx_7(:) - fr_7(:))));
assert(max(abs(fx_7(:) - fr_7(:))) == 0, 'l3_stageB:fixture', 'fresh left run is not bit-identical to the fixture -- STOP');

F = {'a_bar_hat_out', 'a_true_out', 'a_xm_out', 'R2_out', 'innov_y1_out', ...
     'innov_y2_out', 'K_a_y1_out', 'K_a_y2_out', 'K_b_y1_out', 'K_b_y2_out', ...
     'b_hat_out', 'P_b_out', 'P_a_out', 'gate_out', 'h_bar_d_out', ...
     'a_hat_out', 'P41_out', 'h_bar_true_out', 'delta_x_hat_3_out'};

ARMS = {'left', struct('predict_quad', 'left'); 'mid', struct('predict_quad', 'mid'); 'trap', struct('predict_quad', 'trap')};
if ~exist('ARMS_TO_RUN', 'var'); ARMS_TO_RUN = ARMS(:, 1).'; end
for a = 1:size(ARMS, 1)
    name = ARMS{a, 1};
    if ~any(strcmp(name, ARMS_TO_RUN)); continue; end
    ov = ARMS{a, 2};
    clear run_formC_b motion_control_law_formC_b;
    fprintf('\n=== L3 STAGE B ARM %s : override %s ===\n', name, jsonencode(ov));
    opts = struct('seeds', SEEDS, 'arm', 'best', 'ctrl_const_override', ov);
    tic; [~, D] = evalc('run_formC_b(opts)'); el = toc;

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
    S.predict_quad = name;
    % seed 7 of this arm must equal the W234 single-seed run of the same rule
    d7 = max(abs(reshape(S.a_bar_hat_out(:, :, find(SEEDS == 7, 1)), [], 1) - Wx_rule(W234, name)));
    fprintf('ARM %s seed 7 vs W234 single-seed run (obs_dump on): max|diff| = %.3e [0 expected]\n', name, d7);
    S.w234_seed7_maxdiff = d7;
    if strcmp(name, 'left')
        d_fx = max(abs(reshape(S.a_bar_hat_out(:, :, find(SEEDS == 7, 1)), [], 1) - fx_7(:)));
        fprintf('B0 GATE (regenerated left arm) seed 7 vs fixture 1a70599: max|diff| = %.3e [must be 0]\n', d_fx);
        S.fixture_maxdiff = d_fx;
        assert(d_fx == 0 && d7 == 0, 'l3_stageB:B0', 'regenerated B0 seed 7 is not bit-identical -- STOP');
    end
    fn = fullfile(OUT_DIR, sprintf('l3_%s_100.mat', name));
    save(fn, '-struct', 'S', '-v7.3');
    fprintf('ARM DONE %s: %d seeds, %.1f s (%.2f s/seed) -> %s\n', name, ns, el, el/ns, fn);
    fprintf('  driver summary: hold mean %+.3f +- %.3f %%  desc pk %.3f  osc RMS %.3f (mean over seeds)\n', ...
            mean(D.metrics.rows(:, 3)), std(D.metrics.rows(:, 3)), mean(D.metrics.rows(:, 1)), mean(D.metrics.rows(:, 2)));
end
fprintf('\nL3 STAGE B ARMS COMPLETE\n');

function v = Wx_rule(file, name)
    W = load(file, name);
    v = W.(name).a_bar_hat_out(:);
end
