% Does the far-field anchor b = 9/8 explain the argument-error dose response?
%   RIVAL to "a constant offset in a_bar is what helps": an offset is
%   height-dependent in its effect, dividing by b is uniform. If b = 9/8 at the
%   TRUE a_bar lands near the true-slope arm, the law's b is the whole story.
SEEDS = [7 11 23 42 101 777 27 31];
B98 = 9/8;
A = struct();
A.exo_b1   = struct('ap_known',true, 'ap_law_bias',0, 'law_b',1);
A.exo_b98  = struct('ap_known',true, 'ap_law_bias',0, 'law_b',B98);
A.state_b1 = struct('ap_known',false,'law_b',1);
A.state_b98= struct('ap_known',false,'law_b',B98);
nm = fieldnames(A); R = struct();
for j = 1:numel(nm)
    o = A.(nm{j});
    o.arm='base'; o.seeds=SEEDS; o.verbose=false; o.save_mat=false; o.make_fig=false;
    R.(nm{j}) = run_formC_dist(o);
    fprintf('=== %s done ===\n', nm{j});
end
save('/Users/kevin/Code/MotionControl_Simu-motion-test/test_results/formC_law_anchor.mat','R','SEEDS','-v7.3');
fprintf('\n%-24s %12s %12s %12s\n','arm','desc pk %','osc RMS %','rms all %');
for j = 1:numel(nm)
    M = R.(nm{j}).metrics.rows;
    fprintf('%-24s %5.2f+-%-5.2f %5.2f+-%-5.2f %5.2f+-%-5.2f\n', nm{j}, ...
        mean(M(:,1)),std(M(:,1)),mean(M(:,2)),std(M(:,2)),mean(M(:,4)),std(M(:,4)));
end
d = R.state_b98.metrics.rows(:,4) - R.state_b1.metrics.rows(:,4);
fprintf('\nstate: b98 - b1 paired  %+.3f +- %.3f  t=%+.2f  better %d/8\n', ...
    mean(d), std(d), mean(d)/(std(d)/sqrt(8)), sum(d<0));
