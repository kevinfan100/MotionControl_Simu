% STATUS: CLOSED 2026-08-18 -- found Pf_a_floor sized by the envelope sup (119x too large): 5.12 -> 2.27 %, t=-3.62, 8/8
%   Verdict recorded in formC_state_b_ref.tex + memory project-formC-state-b-multiplicative-2026-08-18.
% Is P44[0] sized by the wrong thing?
%   now  : Pf_a_floor = sup over the PLANNED ENVELOPE, attained near the wall
%          at w_bar 2.41 -- a height the run has not reached at t = 0.
%   fix  : the LOCAL shape error at the seed height, which is what
%          P[0] = E[(truth - initial)^2] actually means.
% At w_bar = 22.2222 with b = 9/8 the law is exact to 4.3e-5, so the two
% differ by 119x in sqrt(P44[0]). Everything else identical.
SEEDS = [7 11 23 42 101 777 27 31];
FLOOR_LOCAL = 4.2999e-05;      % |1 - (9/8)/22.2222 - a_true(22.2222)|
R = struct();
R.env = run_formC_b(struct('arm','b98','seeds',SEEDS,'verbose',false, ...
                           'save_mat',false,'make_fig',false));
R.loc = run_formC_b(struct('arm','b98','seeds',SEEDS,'verbose',false, ...
                           'save_mat',false,'make_fig',false, ...
                           'ctrl_const_override',struct('Pf_a_floor',FLOOR_LOCAL)));
save('/Users/kevin/Code/MotionControl_Simu-motion-test/test_results/formC_b_seedfloor.mat','R','SEEDS','-v7.3');
fprintf('\n%-22s %12s %12s %12s\n','P44[0] 來源','desc pk %','osc RMS %','rms all %');
for nm={'env','loc'}
  M=R.(nm{1}).metrics.rows;
  fprintf('%-22s %5.2f+-%-5.2f %5.2f+-%-5.2f %5.2f+-%-5.2f\n', nm{1}, ...
    mean(M(:,1)),std(M(:,1)),mean(M(:,2)),std(M(:,2)),mean(M(:,4)),std(M(:,4)));
end
for c=[1 2 4]
  d=R.loc.metrics.rows(:,c)-R.env.metrics.rows(:,c);
  nmc={'desc pk','osc RMS','','rms all'};
  fprintf('loc - env  %-9s %+7.3f  sd %6.3f  t=%+6.2f  better %d/8\n', ...
      nmc{c}, mean(d), std(d), mean(d)/(std(d)/sqrt(8)), sum(d<0));
end
b=arrayfun(@(q) R.loc.runs{q}.b_hat_out(end,3),(1:8).');
fprintf('\nb_hat[end] (loc): %.5f +- %.5f    sqrt(P55) end %.6f\n', mean(b), std(b), ...
        mean(arrayfun(@(q) R.loc.runs{q}.P_b_out(end,3),(1:8).')));
