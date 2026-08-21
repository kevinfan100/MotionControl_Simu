% DECISIVE ARM (approved by Kevin 2026-08-21): feed the TRUE gain to the
% control law. The EKF still estimates its own a_hat and the whole y2 chain
% still runs -- only the control law's pole placement stops using a_hat, so
% lambda_eff = lambda_c = 0.700 EXACTLY and the pole hypothesis has nowhere
% to hide.
%
% Seeds 1:100 = the first 100 of the 400-seed baseline, so it is PAIRED.
% Deep band (driver default since 16631ef). No production code is modified.
cd('/Users/kevin/Code/MotionControl_Simu-motion-test');
addpath(genpath('test_script')); addpath(genpath('model'));
clear run_formC_b motion_control_law_formC_b;
tic;
D = run_formC_b(struct('seeds', 1:100, 'a_ctrl_override', 'true'));
el = toc;
ns = numel(D.runs); N = numel(D.runs{1}.tout);
A_xm = zeros(N,3,ns); A_tr = zeros(N,3,ns); A_ht = zeros(N,3,ns);
D_xr = zeros(N,3,ns); R2   = zeros(N,3,ns);
for s = 1:ns
    A_xm(:,:,s) = D.runs{s}.a_xm_out;
    A_tr(:,:,s) = D.runs{s}.a_true_out;
    A_ht(:,:,s) = D.runs{s}.a_bar_hat_out;
    D_xr(:,:,s) = D.runs{s}.dx_r_out;
    R2(:,:,s)   = D.runs{s}.R2_out;
end
t = D.runs{1}.tout(:); K = D.runs{1}.ctrl_const; a_nom = D.runs{1}.a_nom;
P = D.runs{1}.meta.params_value;
save('test_results/am_r22_deep/truea_100.mat', 'A_xm','A_tr','A_ht','D_xr','R2', ...
     't','K','a_nom','P','-v7.3');
fprintf('\nTRUE-A ARM DONE: %d seeds, %.1f s (%.2f s/seed)\n', ns, el, el/ns);
fprintf('saved test_results/am_r22_deep/truea_100.mat\n');
