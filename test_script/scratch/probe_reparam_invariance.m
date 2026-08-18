% Reparameterisation invariance check. b_new = 1/b_old exactly, so a run LOCKED
% at b_old = 1 must be identical to one locked at b_new = 1, and b_old = 9/8
% must be identical to b_new = 8/9. Anything other than machine precision means
% the conversion is wrong. Run BEFORE the code change to save the baseline,
% then AFTER with the reciprocal seeds.
SEEDS = [7 11 23 42 101 777 27 31];
tag = getenv('REPARAM_TAG'); if isempty(tag); tag = 'pre'; end
if strcmp(tag,'pre'); B = [1, 9/8]; else; B = [1, 8/9]; end
R = struct();
for j = 1:2
    o = struct('arm','b98','seeds',SEEDS,'verbose',false,'save_mat',false, ...
               'make_fig',false,'ctrl_const_override',struct('lock_b',true,'b_init',B(j)));
    R.(sprintf('b%d',j)) = run_formC_b(o);
end
save(sprintf('/Users/kevin/Code/MotionControl_Simu-motion-test/test_results/reparam_%s.mat',tag), ...
     'R','SEEDS','B','-v7.3');
fprintf('\n[%s]  b seeds: %.6f  %.6f\n', tag, B(1), B(2));
for j = 1:2
    M = R.(sprintf('b%d',j)).metrics.rows;
    fprintf('  arm %d : desc %.6f  osc %.6f  rms %.6f\n', j, ...
        mean(M(:,1)), mean(M(:,2)), mean(M(:,4)));
end
