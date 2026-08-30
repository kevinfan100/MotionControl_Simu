% STAGE A -- ap_known JOINT arm, paired against a matched 'bmid' baseline.
% Pre-registration: test_script/scratch/stageA_prereg.txt (written BEFORE this run).
%
% Both arms use opts.arm = 'bmid' (lock_b = true, b = 8/9 = the 4-state
% writing of the tex) so slot 5 is provably inert on BOTH arms. That
% neutralises the fourth confound found in the code today: dap_db_i is frozen
% at controller:1070, BEFORE the ap_known override at 1081, so on a b-free arm
% F_e(4,5)/H(2,5) would keep running on the law's slope while predict ran on
% the true one.
%
% NO production file is modified. Driver options only.
cd('/Users/kevin/Code/MotionControl_Simu-motion-test');
addpath(genpath('test_script')); addpath(genpath('model'));

SEEDS = 1:100;
% b_hat_out and gate_out are kept this time -- neither was saved in the 08-21
% / 08-24 budget stacks, and W1 (the lock sanity check) needs b_hat_out.
F = {'a_xm_out','a_true_out','a_bar_hat_out','R2_out','K_a_y1_out','K_a_y2_out', ...
     'innov_y1_out','innov_y2_out','P41_out','a_prime_out','a_prime_true_out', ...
     'dx_r_out','b_hat_out','gate_out'};

arms = { ...
    'stageA_bmid_base_100.mat',    struct('seeds', SEEDS, 'arm', 'bmid'); ...
    'stageA_bmid_apknown_100.mat', struct('seeds', SEEDS, 'arm', 'bmid', 'ap_known', true)};

for a = 1:size(arms, 1)
    fname = arms{a, 1};  opts = arms{a, 2};
    % Persistents live in both files; clearing between arms is mandatory.
    clear run_formC_b motion_control_law_formC_b;
    fprintf('\n=== ARM %d/%d : %s ===\n', a, size(arms, 1), fname);
    tic;  D = run_formC_b(opts);  el = toc;

    ns = numel(D.runs);  N = numel(D.runs{1}.tout);
    S = struct();
    for i = 1:numel(F)
        S.(F{i}) = zeros(N, 3, ns);
        for s = 1:ns
            S.(F{i})(:, :, s) = D.runs{s}.(F{i});
        end
    end
    S.t = D.runs{1}.tout(:);  S.K = D.runs{1}.ctrl_const;  S.a_nom = D.runs{1}.a_nom;
    S.opts_used = opts;
    save(fullfile('test_results/am_r22_deep', fname), '-struct', 'S', '-v7.3');
    fprintf('ARM DONE: %d seeds, %.1f s (%.2f s/seed) -> %s\n', ns, el, el/ns, fname);
end

fprintf('\nSTAGE A PAIR COMPLETE. Judge with judge_stageA_apknown.m\n');
