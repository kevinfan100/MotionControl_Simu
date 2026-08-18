% STATUS: CLOSED 2026-08-18 -- b_hat moves ~6 % of one prior width over the run; where it lands barely depends on where it starts
%   Verdict recorded in formC_state_b_ref.tex + memory project-formC-state-b-multiplicative-2026-08-18.
% Does b_hat move at all, and does where it lands depend on where it starts?
%   best   free, seeded at the band midpoint 1.1396
%   b98    free, seeded at the far-field anchor 9/8
%   bfree1 free, seeded AT the attractor 1  -> if it still does not move,
%          nothing about the collapse is what stops it; b just gets no
%          usable information across the run.
SEEDS = [7 11 23 42 101 777 27 31];
R = struct();
for nm = {'best','b98','bfree1'}
    o = struct('arm',nm{1},'seeds',SEEDS,'verbose',false,'save_mat',false,'make_fig',false);
    R.(nm{1}) = run_formC_b(o);
    fprintf('=== %s done ===\n', nm{1});
end
save('/Users/kevin/Code/MotionControl_Simu-motion-test/test_results/formC_b_seed_sweep.mat','R','SEEDS','-v7.3');
fprintf('\n%-8s %10s %12s %12s %12s\n','arm','b seed','b[step2]','b[end]','rms all %');
for nm = {'best','b98','bfree1'}
    A = R.(nm{1}); M = A.metrics.rows;
    b0 = arrayfun(@(q) A.runs{q}.b_hat_out(1,3),   (1:8).');
    b2 = arrayfun(@(q) A.runs{q}.b_hat_out(2,3),   (1:8).');
    be = arrayfun(@(q) A.runs{q}.b_hat_out(end,3), (1:8).');
    fprintf('%-8s %10.5f %12.5f %9.5f+-%.5f %5.2f+-%-5.2f\n', nm{1}, ...
        mean(b0), mean(b2), mean(be), std(be), mean(M(:,4)), std(M(:,4)));
    fprintf('         %s 運動期間走的距離 |b[end]-b[step2]| = %.5f\n', ...
        blanks(0), mean(abs(be-b2)));
end
