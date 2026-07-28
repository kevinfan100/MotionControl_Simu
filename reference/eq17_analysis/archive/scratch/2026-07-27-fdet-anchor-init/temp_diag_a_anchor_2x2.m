%TEMP_DIAG_A_ANCHOR_2X2  What actually holds the LEVEL of a_hat?
%   Two candidate anchors:
%     (i)  the init value a_h[0]          -> peek (exact) vs honest (a_nom, +5.33%)
%     (ii) the y2 = a_xm direct observation -> on vs off (R2 -> inf)
%   If a_hat follows the init when y2 is off, the level is DEAD RECKONING from
%   init + the power-law sweep, and y2 is nearly inert.
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

fprintf('all arms: whitened y2 (production R2), z axis, 6 seeds\n');
fprintf('honest init starts a_h[0] +5.33%% high; peek init starts exact\n\n');
fprintf('%-34s | %10s %10s | %9s %8s | %8s\n', 'arm', 'a_hat@0.4s', 'a_hat/aT osc', ...
        'a_xm/aT', 'sc', 'trk nm');
arms = { 'peek   init , y2 ON  (production)', 0, 0.03, 1; ...
         'peek   init , y2 OFF',              0, 0.03, 1e10; ...
         'honest init , y2 ON',               1, 0.06, 1; ...
         'honest init , y2 OFF',              1, 0.06, 1e10 };
for r = 1:4
    c = cfg;  c.a_init_honest = arms{r,2};  c.Pf_a_frac = arms{r,3};
    c.r2_scale = arms{r,4};
    e04 = zeros(6,1); ar = zeros(6,1); ax = zeros(6,1); tk = zeros(6,1);
    for q = 1:6
        s = temp_run_powerlaw_diag(c, struct('seed', seeds(q), 'verbose', false));
        tt = s.tout(:);  mo = tt > 1.6;
        [~,i4] = min(abs(tt-0.4));
        e04(q) = 100*(s.a_hat_out(i4,az)/s.a_true_out(i4,az) - 1);
        ar(q) = mean(s.a_hat_out(mo,az)./max(s.a_true_out(mo,az),eps));
        ax(q) = mean(s.a_xm_out(mo,az))/mean(s.a_true_out(mo,az));
        e = s.p_d_out(2:end,:) - s.p_true_out(1:end-1,:);  te = tt(2:end);
        tk(q) = 1e3*std(e(te>1.6,az));
    end
    fprintf('%-34s | %9.2f%% %10.4f | %9.4f %8.4f | %8.2f\n', arms{r,1}, ...
            mean(e04), mean(ar), mean(ax), std(ar), mean(tk));
end
fprintf('\ncolumn a_hat@0.4s = end of the far-field hold, before any sweep\n');
fprintf('if OFF ~ ON, y2 is inert and the level is init + power-law dead reckoning\n');
fprintf('if honest/OFF keeps the +5.33%% offset, y2 is the ONLY thing that removes it\n');
