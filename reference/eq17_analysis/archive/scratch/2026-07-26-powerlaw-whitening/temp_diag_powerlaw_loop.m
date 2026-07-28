%TEMP_DIAG_POWERLAW_LOOP  Pin down the descent p_hat swing mechanism.
%   J: break the  p_hat -> s_h -> a_h-sweep  feedback (p_fb_off) and see whether
%      the swing collapses  -> is the loop an amplifier, or is p_hat just an
%      open-loop noise integrator?
%   K: lag cross-correlation between p_hat error and a_hat error in the descent
%      -> is the loop negative (self-correcting but delayed) or positive?
%   L: a_cov sweep -- the IIR memory is the delay inside that loop.
%   Scratch; delete after the diagnosis.

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
az = 3;  seeds = [7 11 23 42 101 777];

fprintf('=========== PART J: break the p_hat -> s_h feedback ===========\n');
fprintf('p_fb_off = 1 : p_hat is still ESTIMATED but no longer drives the sweep\n');
fprintf('%-10s %6s | %9s %9s %9s | %9s %8s %8s\n', 'arm','seed', ...
        'p desc min','p desc max','p desc std','p osc mean','trk z nm','a_hat/aT');
for fb = [0 1]
    dsd = zeros(numel(seeds),1);  osm = zeros(numel(seeds),1);
    trk = zeros(numel(seeds),1);  arr = zeros(numel(seeds),1);
    for q = 1:numel(seeds)
        c = cfg;  c.p_fb_off = fb;
        s = temp_run_powerlaw_diag(c, struct('seed', seeds(q), 'verbose', false));
        tt = s.tout(:);  ph = s.p_hat_out(:,az);
        md = tt > 0.5 & tt < 1.5;  mo = tt > 1.6;
        e = s.p_d_out(2:end,:) - s.p_true_out(1:end-1,:);  te = tt(2:end);
        dsd(q) = std(ph(md));  osm(q) = mean(ph(mo));
        trk(q) = 1e3*std(e(te>1.6,az));
        arr(q) = mean(s.a_hat_out(mo,az) ./ max(s.a_true_out(mo,az), eps));
        if q == 1
            fprintf('%-10s %6d | %9.4f %9.4f %9.4f | %9.4f %8.2f %8.3f\n', ...
                sprintf('fb=%d',fb), seeds(q), min(ph(md)), max(ph(md)), ...
                dsd(q), osm(q), trk(q), arr(q));
        end
    end
    fprintf('%-10s %6s | %9s %9s %9.4f | %9.4f %8.2f %8.3f   <- 6-seed mean\n', ...
        sprintf('fb=%d',fb), 'ALL', '', '', mean(dsd), mean(osm), mean(trk), mean(arr));
    fprintf('%-10s %6s |  p_osc scatter across seeds = %.4f\n\n', '', '', std(osm));
end

fprintf('=========== PART K: loop sign / delay (seed 7, descent window) ===========\n');
c = cfg;  c.p_fb_off = 0;
s = temp_run_powerlaw_diag(c, struct('seed', 7, 'verbose', false));
tt = s.tout(:);  md = tt > 0.5 & tt < 1.5;
ep = s.p_hat_out(md,az) - 1.0;                                   % p_hat error
ea = s.a_hat_out(md,az) - s.a_true_out(md,az);                   % a_hat error
in2 = s.innov_y2_out(md,az);
ep = ep - mean(ep);  ea = ea - mean(ea);  in2 = in2 - mean(in2);
L = 200;
[cc, lg] = xcorr(ep, ea, L, 'coeff');
fprintf('corr( p_hat_err[k+L] , a_hat_err[k] )  -- L>0 means a_hat leads p_hat\n');
for Lq = [-120 -60 -30 -10 0 10 30 60 120]
    fprintf('   L=%+5d (%+7.1f ms) : %+7.3f\n', Lq, Lq*0.625, cc(lg==Lq));
end
[~, im] = max(abs(cc));
fprintf('peak |corr| = %.3f at L = %+d (%.1f ms)\n', abs(cc(im)), lg(im), lg(im)*0.625);
[cc2, lg2] = xcorr(ep, in2, L, 'coeff');
[~, im2] = max(abs(cc2));
fprintf('corr(p_hat_err, innov2) peak = %+.3f at L = %+d (%.1f ms)\n', ...
        cc2(im2), lg2(im2), lg2(im2)*0.625);

fprintf('\n=========== PART L: a_cov (IIR memory = loop delay) sweep ===========\n');
fprintf('%8s %10s | %10s %10s %9s %8s\n','a_cov','1/a_cov','p desc std','p osc mean', ...
        'std(a_xm)','trk z nm');
for acv = [0.02 0.05 0.10 0.20]
    c = cfg;  c.a_cov = acv;  c.p_fb_off = 0;
    s = temp_run_powerlaw_diag(c, struct('seed', 7, 'verbose', false));
    tt = s.tout(:);  ph = s.p_hat_out(:,az);
    md = tt > 0.5 & tt < 1.5;  mo = tt > 1.6;
    e = s.p_d_out(2:end,:) - s.p_true_out(1:end-1,:);  te = tt(2:end);
    fprintf('%8.3f %10.1f | %10.4f %10.4f %9.3f %8.2f\n', acv, 1/acv, ...
        std(ph(md)), mean(ph(mo)), 1e3*std(s.a_xm_out(mo,az)), ...
        1e3*std(e(te>1.6,az)));
end
