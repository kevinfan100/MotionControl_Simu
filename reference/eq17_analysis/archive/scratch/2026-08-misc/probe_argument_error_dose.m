% Argument-error dose response: the LAW fed the TRUE a_bar displaced by a
% constant, exogenously. sigma = 0.02731 = arm (a)'s realised absolute RMS.
SEEDS = [7 11 23 42 101 777 27 31];
SG    = 0.02731;
BIAS  = [0, +SG, -SG, +2*SG];
R = cell(numel(BIAS),1);
for j = 1:numel(BIAS)
    o = struct('arm','base','seeds',SEEDS,'verbose',false, ...
               'ap_known',true,'ap_law_bias',BIAS(j),'save_mat',false,'make_fig',false);
    R{j} = run_formC_dist(o);
    fprintf('=== bias %+.5f done ===\n', BIAS(j));
end
save('/Users/kevin/Code/MotionControl_Simu-motion-test/test_results/formC_argerr.mat','R','BIAS','SG','SEEDS','-v7.3');
fprintf('\n%-14s %10s %10s %10s\n','bias/sigma','desc pk %','osc RMS %','overall %');
for j = 1:numel(BIAS)
    M = R{j}.metrics.rows;
    fprintf('%+13.2f %6.2f+-%-5.2f %6.2f+-%-5.2f %6.2f+-%-5.2f\n', BIAS(j)/SG, ...
        mean(M(:,1)),std(M(:,1)),mean(M(:,2)),std(M(:,2)),mean(M(:,3)),std(M(:,3)));
end
