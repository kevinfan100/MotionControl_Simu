% STATUS: CLOSED 2026-08-18 -- superseded by test_script/integration/plot_formC_belief_vs_command.m, whose figure is the one in the tex
%   Verdict recorded in formC_state_b_ref.tex + memory project-formC-state-b-multiplicative-2026-08-18.
% Does the model act as a RESTORING FORCE on the gain state?
%
%   Both arms are seeded DELIBERATELY WRONG, with b_init = 1 instead of the
%   far-field 9/8. At the seed height that puts a_bar_hat[0] at 0.955000
%   against a truth of 0.949418 -- a known +0.0056 offset, identical in both
%   arms and identical across seeds.
%
%   state : a_bar' = (1-a_bar_hat)^2/b, computed FROM the offset belief
%   exo   : a_bar' supplied from the true height, so the model cannot follow
%           the belief
%
%   If the model is a restoring force, the exogenous arm's offset is inert --
%   the increments are right, so the offset simply persists and the
%   measurements erode it. If the model FOLLOWS the belief, the state arm
%   amplifies the offset while it moves, because a_bar_hat too high makes
%   (1-a_bar_hat)^2 too small, so the descent under-shoots and the offset
%   grows. The two are distinguished by the SHAPE of e_a(t) over the descent,
%   not by any RMS.
SEEDS = [7 11 23 42 101 777 27 31];
base = struct('arm','bfree1','seeds',SEEDS,'verbose',false, ...
              'save_mat',false,'make_fig',false, ...
              'ctrl_const_override',struct('lock_b',true));
R = struct();
R.state = run_formC_b(base);
b2 = base; b2.ap_known = true;
R.exo = run_formC_b(b2);
save('/Users/kevin/Code/MotionControl_Simu-motion-test/test_results/formC_restoring.mat','R','SEEDS','-v7.3');

t = R.state.runs{1}.tout(:);
seg = {'hold 0-0.5', 'descent 0.5-1.5', 'osc 1.5-3.5', 'hold 3.5-end'};
tb  = [0 .5; .5 1.5; 1.5 3.5; 3.5 t(end)];
fprintf('\n平均 e_a = a_bar_hat - a_bar_true  (絕對值，非 %%)\n');
fprintf('%-16s %14s %14s\n','segment','state','exogenous');
for s = 1:4
    w = t>=tb(s,1) & t<tb(s,2);
    v = zeros(2,1); k = 0;
    for nm = {'state','exo'}
        k = k+1; E = [];
        for q = 1:8
            r = R.(nm{1}).runs{q};
            ad = r.a_hat_out(1,3)/r.a_bar_hat_out(1,3);
            E(:,q) = r.a_bar_hat_out(:,3) - r.a_true_out(:,3)/ad; %#ok<AGROW>
        end
        v(k) = mean(mean(E(w,:),2));
    end
    fprintf('%-16s %+14.6f %+14.6f\n', seg{s}, v(1), v(2));
end
