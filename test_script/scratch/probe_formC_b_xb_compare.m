% The three-way comparison under the MULTIPLICATIVE law a' = b(1-a)^2.
%   fixed  : b locked at the far-field anchor 8/9
%   est    : b free, seeded at 8/9
%   trueap : a' supplied from the TRUE height -- the ceiling this writing could
%            reach if the integrand carried no state error
% All three use the seed-local P44[0] floor the driver now computes per arm.
SEEDS = [7 11 23 42 101 777 27 31];
R = struct();
R.fixed  = run_formC_b(struct('arm','b98','seeds',SEEDS,'verbose',false, ...
                              'save_mat',false,'make_fig',false, ...
                              'ctrl_const_override',struct('lock_b',true)));
R.est    = run_formC_b(struct('arm','b98','seeds',SEEDS,'verbose',false, ...
                              'save_mat',false,'make_fig',false));
R.trueap = run_formC_b(struct('arm','b98','seeds',SEEDS,'verbose',false, ...
                              'save_mat',false,'make_fig',false,'ap_known',true));
save('/Users/kevin/Code/MotionControl_Simu-motion-test/test_results/formC_b_xb_compare.mat', ...
     'R','SEEDS','-v7.3');
fprintf('\n%-10s %12s %12s %12s %14s\n','arm','desc pk %','osc RMS %','rms all %','b_hat[end]');
for nm = {'fixed','est','trueap'}
    M = R.(nm{1}).metrics.rows;
    bb = arrayfun(@(q) R.(nm{1}).runs{q}.b_hat_out(end,3), (1:8).');
    fprintf('%-10s %5.2f+-%-5.2f %5.2f+-%-5.2f %5.2f+-%-5.2f %7.5f+-%.5f\n', nm{1}, ...
        mean(M(:,1)),std(M(:,1)),mean(M(:,2)),std(M(:,2)),mean(M(:,4)),std(M(:,4)), ...
        mean(bb),std(bb));
end
d = R.est.metrics.rows(:,4) - R.fixed.metrics.rows(:,4);
fprintf('\nest - fixed   %+.3f  sd %.3f  SEM %.3f  t=%+.2f  better %d/8\n', ...
    mean(d),std(d),std(d)/sqrt(8),mean(d)/(std(d)/sqrt(8)),sum(d<0));
e = R.trueap.metrics.rows(:,4) - R.fixed.metrics.rows(:,4);
fprintf('trueap-fixed  %+.3f  sd %.3f  SEM %.3f  t=%+.2f  better %d/8\n', ...
    mean(e),std(e),std(e)/sqrt(8),mean(e)/(std(e)/sqrt(8)),sum(e<0));
