% STATUS: CLOSED 2026-08-18 -- estimating b does not pay: fixed 2.20+-1.33 %, estimated 2.46+-1.58 %, t=+2.61, 1/8; ceiling 1.16 %
%   Verdict recorded in formC_state_b_ref.tex + memory project-formC-state-b-multiplicative-2026-08-18.
% The comparison the user asked for, ALL with the corrected seed-local P44[0]:
%   fixed  : b locked at 9/8, not estimated
%   est    : b free, seeded at 9/8
%   trueap : a_bar' supplied from the TRUE height -- the ceiling this writing
%            could reach if the integrand were perfect
SEEDS = [7 11 23 42 101 777 27 31];
R = struct();
R.fixed  = run_formC_b(struct('arm','b98','seeds',SEEDS,'verbose',false, ...
                              'save_mat',false,'make_fig',false, ...
                              'ctrl_const_override',struct('lock_b',true)));
R.est    = run_formC_b(struct('arm','b98','seeds',SEEDS,'verbose',false, ...
                              'save_mat',false,'make_fig',false));
R.trueap = run_formC_b(struct('arm','b98','seeds',SEEDS,'verbose',false, ...
                              'save_mat',false,'make_fig',false,'ap_known',true));
save('/Users/kevin/Code/MotionControl_Simu-motion-test/test_results/formC_b_final_compare.mat','R','SEEDS','-v7.3');
fprintf('\n%-10s %12s %12s %12s\n','arm','desc pk %','osc RMS %','rms all %');
for nm={'fixed','est','trueap'}
  M=R.(nm{1}).metrics.rows;
  fprintf('%-10s %5.2f+-%-5.2f %5.2f+-%-5.2f %5.2f+-%-5.2f\n', nm{1}, ...
    mean(M(:,1)),std(M(:,1)),mean(M(:,2)),std(M(:,2)),mean(M(:,4)),std(M(:,4)));
end
d=R.est.metrics.rows(:,4)-R.fixed.metrics.rows(:,4);
fprintf('\nest - fixed   %+.3f  sd %.3f  SEM %.3f  t=%+.2f  better %d/8\n', ...
  mean(d),std(d),std(d)/sqrt(8),mean(d)/(std(d)/sqrt(8)),sum(d<0));
e=R.trueap.metrics.rows(:,4)-R.fixed.metrics.rows(:,4);
fprintf('trueap-fixed  %+.3f  sd %.3f  SEM %.3f  t=%+.2f  better %d/8\n', ...
  mean(e),std(e),std(e)/sqrt(8),mean(e)/(std(e)/sqrt(8)),sum(e<0));
