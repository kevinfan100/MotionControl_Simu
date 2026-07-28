%TEMP_DIAG_INIT_ASYM  Honest init that is ALSO accurate.
%   Three ways to seed a_h[0] without reading the full c(h_bar) curve:
%     peek   : a_nom / c(h_bar_0)                 <- reads the true curve (cheat)
%     flat   : a_nom                              <- honest, +5.33% error
%     asym   : a_nom / (1 + (9/8)/h_bar_0)        <- honest, +0.25% error
%              (perp; parallel uses (9/16)/h_bar_0). This is the leading term of
%              the SAME method-of-reflections asymptote that pins p = 1, and it
%              needs only h_bar_0 (wall position, already assumed known).
%   Scratch; delete with the rest of this round's temp files.

clear; clc;
here = fileparts(mfilename('fullpath'));
proj = fullfile(here, '..');
addpath(genpath(proj));

cfg = user_config();
cfg.trajectory_type = 'osc';
cfg.h_init = 50;  cfg.h_bottom = 4.5;  cfg.amplitude = 2.5;
cfg.frequency = 1;  cfg.n_cycles = 2;
cfg.t_hold = 0.5;  cfg.t_descend_override = 1.0;  cfg.T_sim = 4.8;
pc = physical_constants();
cfg.h_min = 1.05 * pc.R;
cfg.ctrl_enable = true;  cfg.thermal_enable = true;  cfg.meas_noise_enable = true;
cfg.lambda_c = 0.7;  cfg.a_pd = 0.05;  cfg.a_cov = 0.05;
cfg.meas_noise_std = [0.00062; 0.00057; 0.00331];
cfg.h_bar_safe = 1.5;  cfg.y2_whiten = 1;  cfg.Pf_p_std = 0.035;
seeds = [7 11 23 42 101 777];  az = 3;

arms = { 'peek (現行, 作弊)     ', struct('Pf_a_frac',0.03); ...
         'flat  a_nom          ', struct('a_init_honest',1,'Pf_a_frac',0.06); ...
         'asym  1+(9/8)/hbar   ', struct('a_init_asym',1,'Pf_a_frac',0.01); ...
         'asym  + Pf_a 0.03    ', struct('a_init_asym',1,'Pf_a_frac',0.03) };

fprintf('%-22s | %8s %9s %9s %9s | %9s %8s | %7s\n','arm','err@0', ...
        'max|e| hold','e@0.5s','e@0.4s','a_hat/aT osc','sc','trk nm');
for r = 1:size(arms,1)
    c = cfg;  f = fieldnames(arms{r,2});
    for j = 1:numel(f); c.(f{j}) = arms{r,2}.(f{j}); end
    e0=zeros(6,1); mx=zeros(6,1); e5=zeros(6,1); e4=zeros(6,1);
    ar=zeros(6,1); tk=zeros(6,1);
    for q = 1:6
        s = temp_run_powerlaw_diag(c, struct('seed', seeds(q), 'verbose', false));
        tt = s.tout(:);
        rel = 100*(s.a_hat_out(:,az)-s.a_true_out(:,az))./s.a_true_out(:,az);
        hold_m = tt <= 0.5;
        e0(q) = rel(1);  mx(q) = max(abs(rel(hold_m)));
        [~,i5] = min(abs(tt-0.5));  e5(q) = rel(i5);
        [~,i4] = min(abs(tt-0.4));  e4(q) = rel(i4);
        mo = tt > 1.6;
        ar(q) = mean(s.a_hat_out(mo,az)./max(s.a_true_out(mo,az),eps));
        e = s.p_d_out(2:end,:) - s.p_true_out(1:end-1,:);  te = tt(2:end);
        tk(q) = 1e3*std(e(te>1.6,az));
    end
    fprintf('%-22s | %7.2f%% %10.2f%% %8.2f%% %8.2f%% | %9.4f %8.4f | %7.2f\n', ...
        arms{r,1}, mean(e0), mean(mx), mean(e5), mean(e4), mean(ar), std(ar), mean(tk));
end
fprintf('\nerr@0 = init 假設誤差本身 ; max|e| hold = 遠場 hold 期最大偏離\n');
fprintf('a_hat/aT osc 若四臂相同 -> init 對穩態沒有影響, 差別只在暫態\n');
