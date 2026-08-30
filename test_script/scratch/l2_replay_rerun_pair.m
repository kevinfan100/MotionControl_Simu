% L2 predict replay -- Route B rerun of the Stage A pair (2 x 100 seeds) with the
% extra fields needed to reconstruct M_row4 offline. Settings identical to
% test_script/scratch/run_stageA_apknown_pair.m (arm 'bmid', ap_known on/off).
% Registration: test_script/scratch/l2_replay_prereg.txt
cd('/Users/kevin/code/MotionControl_Simu-motion-test');
addpath(genpath('test_script')); addpath(genpath('model'));

SEEDS = 1:100;
F3 = {'a_xm_out','a_true_out','a_bar_hat_out','R2_out','K_a_y1_out','K_a_y2_out', ...
      'innov_y1_out','innov_y2_out','P41_out','a_prime_out','a_prime_true_out', ...
      'dx_r_out','b_hat_out','gate_out', ...
      'delta_x_hat_3_out','a_bar_Q_out','P_a_out','K_b_y1_out','K_b_y2_out'};   % N x 3
F1 = {'h_bar_d_out','h_bar_true_out'};                                            % N x 1

arms = { ...
    'l2_bmid_base_100.mat',    struct('seeds', SEEDS, 'arm', 'bmid'); ...
    'l2_bmid_apknown_100.mat', struct('seeds', SEEDS, 'arm', 'bmid', 'ap_known', true)};

for a = 1:size(arms, 1)
    fname = arms{a, 1};  opts = arms{a, 2};
    clear run_formC_b motion_control_law_formC_b;
    fprintf('\n=== ARM %d/%d : %s ===\n', a, size(arms, 1), fname);
    tic;  D = run_formC_b(opts);  el = toc;
    ns = numel(D.runs);  N = numel(D.runs{1}.tout);
    S = struct();
    for i = 1:numel(F3)
        S.(F3{i}) = zeros(N, 3, ns);
        for s = 1:ns; S.(F3{i})(:, :, s) = D.runs{s}.(F3{i}); end
    end
    for i = 1:numel(F1)
        S.(F1{i}) = zeros(N, ns);
        for s = 1:ns; S.(F1{i})(:, s) = D.runs{s}.(F1{i})(:); end
    end
    S.t = D.runs{1}.tout(:);  S.K = D.runs{1}.ctrl_const;  S.a_nom = D.runs{1}.a_nom;
    S.R_um = physical_constants().R;
    S.opts_used = opts;
    save(fullfile('test_results/am_r22_deep', fname), '-struct', 'S', '-v7.3');
    fprintf('ARM DONE: %d seeds, %.1f s (%.2f s/seed) -> %s\n', ns, el, el/ns, fname);
end
fprintf('\nL2 RERUN PAIR COMPLETE.\n');
