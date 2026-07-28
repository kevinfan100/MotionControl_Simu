%TEMP_DIAG_WHO_MOVES_AHAT  Which update actually drives the a_hat drift?
%   Every step:  d(a_hat) = predict_delta + K1(4)*innov1 + K2(4)*innov2
%   Accumulate each term over a long positioning run and see which one carries
%   the drift. This does not assume a mechanism -- it measures the motion.
clear; clc;
here = fileparts(mfilename('fullpath'));  addpath(genpath(fullfile(here,'..')));
base = user_config();
base.trajectory_type = 'positioning';  base.h_init = 50;  base.T_sim = 12;
pc = physical_constants();  base.h_min = 1.05*pc.R;
base.ctrl_enable=true; base.thermal_enable=true; base.meas_noise_enable=true;
base.lambda_c=0.7; base.a_pd=0.05; base.a_cov=0.05;
base.meas_noise_std=[0.00062;0.00057;0.00331];
base.h_bar_safe=1.5; base.Pf_a_frac=0.03;
az=3; seeds=[7 11 23 42 101 777];
fprintf('PURE POSITIONING h_bar=22.2, T=12 s, 6 seeds, z axis\n');
fprintf('累積貢獻 (nm/pN), 相對 a_true 的百分比\n\n');
fprintf('%12s | %11s %11s %11s | %11s %11s\n','window [s]', ...
        'from y1','from y2','from predict','sum','a_hat 實際變化');
wins=[0 1;1 2;2 4;4 6;6 8;8 10;10 12];
A1=[];A2=[];AP=[];AH=[];T=[];
for q=1:6
    s=temp_run_powerlaw_diag(base,struct('seed',seeds(q),'verbose',false));
    T=s.tout(:);
    A1(:,q)=s.DG.dg_dA_y1(:,az); A2(:,q)=s.DG.dg_dA_y2(:,az);
    AP(:,q)=s.DG.dg_dA_pred(:,az); AH(:,q)=s.a_hat_out(:,az);
end
aT = 0.013962;
for w=1:size(wins,1)
    m = T>wins(w,1) & T<=wins(w,2);
    c1=1e3*mean(sum(A1(m,:),1)); c2=1e3*mean(sum(A2(m,:),1));
    cp=1e3*mean(sum(AP(m,:),1));
    i0=find(m,1,'first'); i1=find(m,1,'last');
    dah=1e3*mean(AH(i1,:)-AH(i0,:));
    fprintf('%5.0f - %-5.0f | %+10.4f %+10.4f %+10.4f | %+10.4f %+10.4f\n', ...
        wins(w,1),wins(w,2),c1,c2,cp,c1+c2+cp,dah);
end
c1=1e3*mean(sum(A1,1)); c2=1e3*mean(sum(A2,1)); cp=1e3*mean(sum(AP,1));
fprintf('\n全程 0-12 s : y1 %+.4f | y2 %+.4f | predict %+.4f  nm/pN\n', c1,c2,cp);
fprintf('           占 a_true(%.3f nm/pN) : y1 %+.1f%% | y2 %+.1f%% | predict %+.1f%%\n', ...
        1e3*aT, 100*c1/(1e3*aT), 100*c2/(1e3*aT), 100*cp/(1e3*aT));
