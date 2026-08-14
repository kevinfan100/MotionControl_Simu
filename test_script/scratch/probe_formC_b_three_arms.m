% Version (a) of formC_state_b: does estimating b help?
%   b1   = b locked at 1      (reproduces the parameter-free baseline)
%   bmid = b locked at the envelope midpoint (what a better FIXED value buys)
%   best = b free from the same seed as bmid  (the derivation's claim)
% best vs bmid is the controlled test: same seed, same centre, only the lock differs.
SEEDS = [7 11 23 42 101 777 27 31];
R = struct();
for nm = {'b1','bmid','best'}
    o = struct('arm',nm{1},'seeds',SEEDS,'verbose',false,'save_mat',false,'make_fig',false);
    R.(nm{1}) = run_formC_b(o);
    fprintf('=== %s done ===\n', nm{1});
end
save('/Users/kevin/Code/MotionControl_Simu-motion-test/test_results/formC_b_three_arms.mat','R','SEEDS','-v7.3');
fprintf('\n%-8s %12s %12s %12s %14s\n','arm','desc pk %','osc RMS %','rms all %','b_hat[end]');
for nm = {'b1','bmid','best'}
    M = R.(nm{1}).metrics.rows;
    bb = arrayfun(@(q) R.(nm{1}).runs{q}.b_hat_out(end,3), (1:8).');
    fprintf('%-8s %5.2f+-%-5.2f %5.2f+-%-5.2f %5.2f+-%-5.2f %7.4f+-%.4f\n', nm{1}, ...
        mean(M(:,1)),std(M(:,1)),mean(M(:,2)),std(M(:,2)),mean(M(:,4)),std(M(:,4)), ...
        mean(bb),std(bb));
end
d = R.best.metrics.rows(:,4) - R.bmid.metrics.rows(:,4);
fprintf('\nbest - bmid  %+.3f +- %.3f  t=%+.2f  better %d/8\n', ...
        mean(d), std(d), mean(d)/(std(d)/sqrt(8)), sum(d<0));
e = R.bmid.metrics.rows(:,4) - R.b1.metrics.rows(:,4);
fprintf('bmid - b1    %+.3f +- %.3f  t=%+.2f  better %d/8\n', ...
        mean(e), std(e), mean(e)/(std(e)/sqrt(8)), sum(e<0));
