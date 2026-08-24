% Ablation arm: use_fdet=false, deep band, 100 seeds (same set as the
% baseline budget run: seeds 1:100). No code change -- driver option only.
cd('/Users/kevin/Code/MotionControl_Simu-motion-test');
addpath(genpath('test_script')); addpath(genpath('model'));
clear run_formC_b motion_control_law_formC_b;
tic;
D = run_formC_b(struct('seeds', 1:100, 'ctrl_const_override', struct('use_fdet', false)));
el = toc;
ns = numel(D.runs); N = numel(D.runs{1}.tout);
f = {'a_xm_out','a_true_out','a_bar_hat_out','R2_out','K_a_y1_out','K_a_y2_out', ...
     'innov_y1_out','innov_y2_out','P41_out','a_prime_out','dx_r_out'};
S = struct();
for i = 1:numel(f)
    S.(f{i}) = zeros(N,3,ns);
    for s = 1:ns; S.(f{i})(:,:,s) = D.runs{s}.(f{i}); end
end
S.t = D.runs{1}.tout(:); S.K = D.runs{1}.ctrl_const; S.a_nom = D.runs{1}.a_nom;
save('test_results/am_r22_deep/fdet_off_budget_100.mat','-struct','S','-v7.3');
fprintf('\nFDET-OFF ARM DONE: %d seeds, %.1f s\n', ns, el);
