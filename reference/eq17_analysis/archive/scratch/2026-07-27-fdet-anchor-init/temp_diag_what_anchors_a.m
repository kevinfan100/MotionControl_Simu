%TEMP_DIAG_WHAT_ANCHORS_A  Which setting / derivation element gives the good a estimate?
%   Reports PER AXIS, 6 seeds:
%     a_hat/a_true  (the estimate)      and   a_xm/a_true  (its raw measurement)
%   Arms:
%     1 pre-fix   (un-whitened y2, prior 0.3)   <- what the committed figure used
%     2 production (whitened y2, prior 0.035)
%     3 production but y2 DISABLED (R2 -> inf)  <- ablate the level anchor
%     4 production but gain SWEEP disabled (p_fb_off + s_h forced 0 via far field)
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
cfg.h_bar_safe = 1.5;  cfg.Pf_a_frac = 0.03;
seeds = [7 11 23 42 101 777];  an = {'x','y','z'};

arms = { 'pre-fix (as committed)',   struct('y2_whiten',0,'Pf_p_std',0.30); ...
         'production (A1+A2)',       struct('y2_whiten',1,'Pf_p_std',0.035); ...
         'production, y2 DISABLED',  struct('y2_whiten',1,'Pf_p_std',0.035,'r2_scale',1e10) };

for r = 1:size(arms,1)
    c = cfg;  f = fieldnames(arms{r,2});
    for j = 1:numel(f); c.(f{j}) = arms{r,2}.(f{j}); end
    AR = zeros(6,3);  AX = zeros(6,3);  TK = zeros(6,3);
    for q = 1:6
        s = temp_run_powerlaw_diag(c, struct('seed', seeds(q), 'verbose', false));
        tt = s.tout(:);  mo = tt > 1.6;
        e = s.p_d_out(2:end,:) - s.p_true_out(1:end-1,:);  te = tt(2:end);
        for ax = 1:3
            AR(q,ax) = mean(s.a_hat_out(mo,ax)./max(s.a_true_out(mo,ax),eps));
            AX(q,ax) = mean(s.a_xm_out(mo,ax))/mean(s.a_true_out(mo,ax));
            TK(q,ax) = 1e3*std(e(te>1.6,ax));
        end
    end
    fprintf('\n=== %s ===\n', arms{r,1});
    fprintf('%6s | %14s %8s | %14s | %8s\n','axis','a_hat/a_true','sc', ...
            'a_xm/a_true','trk nm');
    for ax = 1:3
        fprintf('%6s | %14.4f %8.4f | %14.4f | %8.2f\n', an{ax}, ...
                mean(AR(:,ax)), std(AR(:,ax)), mean(AX(:,ax)), mean(TK(:,ax)));
    end
end

fprintf('\n--- reading ---\n');
fprintf('a_xm/a_true  = is the RAW measurement itself biased?\n');
fprintf('a_hat/a_true = does the estimator match its own measurement?\n');
fprintf('y2 DISABLED  = what the estimate is worth WITHOUT the direct a_h observation\n');
