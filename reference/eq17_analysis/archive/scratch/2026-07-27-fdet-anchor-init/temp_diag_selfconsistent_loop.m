%TEMP_DIAG_SELFCONSISTENT_LOOP  Is the a_hat drift a self-consistent feedback loop?
%   Loop:  a_hat low -> f_d = (1/a_hat){..} large -> tighter closed loop
%          -> sigma^2_delta_h smaller -> a_xm (∝ sigma^2_dh_r) reads lower
%          -> confirms the low a_hat -> keeps drifting.
%   PREDICTION: over a long positioning run, a_hat, a_xm and the REALISED
%   tracking std must all fall together. If the realised std stays flat while
%   a_hat falls, the loop hypothesis is dead.
%   Scratch; delete with the rest of this round's temp files.
clear; clc;
here = fileparts(mfilename('fullpath'));  addpath(genpath(fullfile(here,'..')));

base = user_config();
base.trajectory_type = 'positioning';  base.h_init = 50;  base.T_sim = 12;
pc = physical_constants();  base.h_min = 1.05*pc.R;
base.ctrl_enable = true; base.thermal_enable = true; base.meas_noise_enable = true;
base.lambda_c = 0.7; base.a_pd = 0.05; base.a_cov = 0.05;
base.meas_noise_std = [0.00062;0.00057;0.00331];
base.h_bar_safe = 1.5; base.Pf_a_frac = 0.03;
az = 3; seeds = [7 11 23 42 101 777];

N = numel(seeds);
AH = []; AX = []; SD = []; FD = [];
for q = 1:N
    s = temp_run_powerlaw_diag(base, struct('seed',seeds(q),'verbose',false));
    t = s.tout(:);
    AH(:,q) = s.a_hat_out(:,az) ./ s.a_true_out(:,az);            %#ok<AGROW>
    AX(:,q) = s.a_xm_out(:,az)  ./ s.a_true_out(:,az);            %#ok<AGROW>
    e = [0; (s.p_d_out(2:end,:)-s.p_true_out(1:end-1,:))*[0;0;1]];
    SD(:,q) = e;                                                   %#ok<AGROW>
    FD(:,q) = s.f_d_out(:,az);                                     %#ok<AGROW>
end

fprintf('PURE POSITIONING h_bar=22.2, T=12 s, %d seeds, z axis\n', N);
fprintf('window means over 6 seeds; tracking std is the REALISED closed-loop std\n\n');
fprintf('%12s | %10s %10s | %12s %12s | %10s\n','window [s]','a_hat/aT','a_xm/aT', ...
        'trk std nm','rms f_d pN','loop?');
wins = [1 2; 2 4; 4 6; 6 8; 8 10; 10 12];
prev_sd = NaN;
for w = 1:size(wins,1)
    m = t > wins(w,1) & t <= wins(w,2);
    ah = mean(mean(AH(m,:),1));
    ax = mean(mean(AX(m,:),1));
    sd = 1e3*mean(std(SD(m,:),0,1));
    fd = mean(rms(FD(m,:),1));
    if isnan(prev_sd); tag = '-'; else
        tag = ternary(sd < prev_sd, 'std DOWN', 'std flat/up');
    end
    fprintf('%5.0f - %-5.0f | %10.4f %10.4f | %12.3f %12.4f | %10s\n', ...
            wins(w,1), wins(w,2), ah, ax, sd, fd, tag);
    prev_sd = sd;
end

fprintf('\n--- 相關性 (跨窗) ---\n');
ahv = zeros(size(wins,1),1); sdv = ahv; axv = ahv;
for w = 1:size(wins,1)
    m = t > wins(w,1) & t <= wins(w,2);
    ahv(w) = mean(mean(AH(m,:),1));
    axv(w) = mean(mean(AX(m,:),1));
    sdv(w) = 1e3*mean(std(SD(m,:),0,1));
end
fprintf('corr(a_hat, trk std) = %+.3f   corr(a_xm, trk std) = %+.3f\n', ...
        corr(ahv,sdv), corr(axv,sdv));
fprintf('a_hat 從 %.4f 跌到 %.4f (%.1f%%) ; 追蹤 std 從 %.2f 到 %.2f nm (%.1f%%)\n', ...
        ahv(1), ahv(end), 100*(ahv(end)/ahv(1)-1), sdv(1), sdv(end), ...
        100*(sdv(end)/sdv(1)-1));
fprintf('\n迴路成立 -> 兩者同向下降, corr 高且為正\n');
fprintf('迴路死亡 -> 追蹤 std 持平, a_hat 卻獨自往下\n');

function o=ternary(c,a,b); if c; o=a; else; o=b; end; end
