%TEMP_DIAG_FDET_TEST  Does the fdet fix kill the y1 rectification?
%   fe_use_fdet = 1 builds F_e(3,4)/F_e(4,4) from the DETERMINISTIC control
%   mirror (delta_x_m := 0, own history) instead of the realised f_d.
%   PREDICTIONS if the endogeneity/rectification mechanism is right:
%     (a) the y1 contribution to a_hat collapses from -24.3% to ~0
%     (b) a_hat/aT stops drifting with T_sim
%     (c) in PURE POSITIONING F_dh_det = 0 exactly -> a_hat rests on y2 only,
%         so a_hat/aT should land near a_xm/aT (~0.945)
clear; clc;
here = fileparts(mfilename('fullpath'));  addpath(genpath(fullfile(here,'..')));
pc = physical_constants();
az=3; seeds=[7 11 23 42 101 777];

pos = user_config();
pos.trajectory_type='positioning'; pos.h_init=50;
pos.h_min=1.05*pc.R; pos.ctrl_enable=true; pos.thermal_enable=true;
pos.meas_noise_enable=true; pos.lambda_c=0.7; pos.a_pd=0.05; pos.a_cov=0.05;
pos.meas_noise_std=[0.00062;0.00057;0.00331]; pos.h_bar_safe=1.5; pos.Pf_a_frac=0.03;

fprintf('=== A. 貢獻分解 + 漂移 (純定位, z, 6 seed) ===\n');
fprintf('%-10s %5s | %9s %9s %9s | %9s %9s | %8s\n','arm','T[s]', ...
        'y1 %%','y2 %%','pred %%','a_hat/aT','a_xm/aT','trk nm');
aT = 0.013962;
for fd = [0 1]
  for T = [2 5 12]
    c = pos; c.T_sim = T; c.fe_use_fdet = fd;
    c1=zeros(6,1);c2=c1;cp=c1;ar=c1;ax=c1;tk=c1;
    for q=1:6
        s=temp_run_powerlaw_diag(c,struct('seed',seeds(q),'verbose',false));
        t=s.tout(:); ss=t>1.0;
        c1(q)=1e3*sum(s.DG.dg_dA_y1(:,az)); c2(q)=1e3*sum(s.DG.dg_dA_y2(:,az));
        cp(q)=1e3*sum(s.DG.dg_dA_pred(:,az));
        ar(q)=mean(s.a_hat_out(ss,az)./max(s.a_true_out(ss,az),eps));
        ax(q)=mean(s.a_xm_out(ss,az))/mean(s.a_true_out(ss,az));
        e=s.p_d_out(2:end,:)-s.p_true_out(1:end-1,:); te=t(2:end);
        tk(q)=1e3*std(e(te>1.0,az));
    end
    p=100/(1e3*aT);
    fprintf('%-10s %5.0f | %+8.1f %+8.1f %+8.1f | %9.4f %9.4f | %8.2f\n', ...
        ternary(fd==0,'fdet OFF','fdet ON'), T, mean(c1)*p, mean(c2)*p, mean(cp)*p, ...
        mean(ar), mean(ax), mean(tk));
  end
end

fprintf('\n=== B. 原本的振盪場景 (hold->descend->osc, 4.8 s) ===\n');
osc = pos; osc.trajectory_type='osc'; osc.h_bottom=4.5; osc.amplitude=2.5;
osc.frequency=1; osc.n_cycles=2; osc.t_hold=0.5; osc.t_descend_override=1.0;
osc.T_sim=4.8;
fprintf('%-10s | %9s %9s %9s | %9s %8s | %8s\n','arm','a_hat/aT x','y','z', ...
        'a_xm/aT z','p_osc z','trk z nm');
for fd = [0 1]
    c = osc; c.fe_use_fdet = fd;
    A=zeros(6,3); axz=zeros(6,1); pz=zeros(6,1); tk=zeros(6,1);
    for q=1:6
        s=temp_run_powerlaw_diag(c,struct('seed',seeds(q),'verbose',false));
        t=s.tout(:); mo=t>1.6;
        for k=1:3; A(q,k)=mean(s.a_hat_out(mo,k)./max(s.a_true_out(mo,k),eps)); end
        axz(q)=mean(s.a_xm_out(mo,az))/mean(s.a_true_out(mo,az));
        pz(q)=mean(s.p_hat_out(mo,az));
        e=s.p_d_out(2:end,:)-s.p_true_out(1:end-1,:); te=t(2:end);
        tk(q)=1e3*std(e(te>1.6,az));
    end
    fprintf('%-10s | %9.4f %9.4f %9.4f | %9.4f %8.4f | %8.2f\n', ...
        ternary(fd==0,'fdet OFF','fdet ON'), mean(A(:,1)),mean(A(:,2)),mean(A(:,3)), ...
        mean(axz), mean(pz), mean(tk));
end
function o=ternary(c,a,b); if c; o=a; else; o=b; end; end
