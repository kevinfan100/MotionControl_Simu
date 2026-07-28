%TEMP_DIAG_POWERLAW_PHASE3  Systematic-vs-random test for the descent p_hat swing.
%   G: 6-seed repeat -- is the descent excursion reproducible (systematic /
%      derivation) or seed-dependent (noise amplification)?
%   H: a_hat tracking quality per phase (the "unsteady tracking" the eye sees).
%   I: does p_hat lock over-confidently?  Q55_floor arm.
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
az = 3;
seeds = [7 11 23 42 101 777];

fprintf('=========== PART G: seed repeatability of the descent p_hat swing ===========\n');
fprintf('%6s | %9s %9s %9s | %9s %9s %9s | %8s\n', 'seed', ...
        'p desc min','p desc max','p desc std','p osc mean','sig_p end','p_end','trk z nm');
Pgrid = [];  Pend = [];  Posc = [];
for q = 1:numel(seeds)
    s = temp_run_powerlaw_diag(cfg, struct('seed', seeds(q), 'verbose', false));
    tt = s.tout(:);  ph = s.p_hat_out(:,az);
    md = tt > 0.5 & tt < 1.5;  mo = tt > 1.6;
    e = s.p_d_out(2:end,:) - s.p_true_out(1:end-1,:);  te = tt(2:end);
    fprintf('%6d | %9.4f %9.4f %9.4f | %9.4f %9.4f %9.4f | %8.2f\n', seeds(q), ...
        min(ph(md)), max(ph(md)), std(ph(md)), mean(ph(mo)), ...
        s.P_p_out(end,az), ph(end), 1e3*std(e(te>1.6,az)));
    Pgrid = [Pgrid, ph];  Pend(end+1) = ph(end);  Posc(end+1) = mean(ph(mo)); %#ok<AGROW>
end
fprintf('\nacross seeds: p_end   mean %.4f  std %.4f\n', mean(Pend), std(Pend));
fprintf('              p_osc   mean %.4f  std %.4f\n', mean(Posc), std(Posc));
fprintf('true effective exponent (z, over hbar 2..22) ~ 0.976 .. 1.004\n');
tt = s.tout(:);
md = tt > 0.5 & tt < 1.5;
fprintf('\nensemble p_hat(t) during descent: mean-trace vs per-seed spread\n');
fprintf('%7s %9s %9s %9s\n','t[s]','mean','std','spread');
for tq = [0.6 0.8 0.9 1.0 1.1 1.2 1.3 1.4 1.5]
    [~, i] = min(abs(tt - tq));
    fprintf('%7.2f %9.4f %9.4f %9.4f\n', tt(i), mean(Pgrid(i,:)), std(Pgrid(i,:)), ...
            max(Pgrid(i,:)) - min(Pgrid(i,:)));
end

fprintf('\n=========== PART H: a_hat quality per phase (seed 7) ===========\n');
s = temp_run_powerlaw_diag(cfg, struct('seed', 7, 'verbose', false));
tt = s.tout(:);
rel = (s.a_hat_out(:,az) - s.a_true_out(:,az)) ./ s.a_true_out(:,az);
ph_name = {'hold  t<0.5','descend .5-1.5','osc   >1.6'};
msk = {tt<0.5, tt>=0.5 & tt<1.5, tt>1.6};
fprintf('%-16s %10s %10s %10s %10s\n','phase','mean%','std%','min%','max%');
for j = 1:3
    m = msk{j};
    fprintf('%-16s %10.2f %10.2f %10.2f %10.2f\n', ph_name{j}, ...
        100*mean(rel(m)), 100*std(rel(m)), 100*min(rel(m)), 100*max(rel(m)));
end

fprintf('\n=========== PART I: Q55_floor arm (can p_hat recover after locking?) ===========\n');
fprintf('%12s | %9s %9s %9s | %9s %8s\n','Q55_floor','p osc mean','sig_p end','p_end', ...
        'p desc std','trk z nm');
for qf = [0 1e-10 1e-8 1e-6]
    c3 = cfg;  c3.Q55_floor = qf;
    % Q55_floor is read from ctrl_const, injected via the scratch driver hook
    s3 = run_with_q55(c3, qf, 7);
    tt3 = s3.tout(:);  ph3 = s3.p_hat_out(:,az);
    mo3 = tt3 > 1.6;  md3 = tt3 > 0.5 & tt3 < 1.5;
    e3 = s3.p_d_out(2:end,:) - s3.p_true_out(1:end-1,:);  te3 = tt3(2:end);
    fprintf('%12.3g | %9.4f %9.4f %9.4f | %9.4f %8.2f\n', qf, mean(ph3(mo3)), ...
        s3.P_p_out(end,az), ph3(end), std(ph3(md3)), 1e3*std(e3(te3>1.6,az)));
end

function s = run_with_q55(cfg, qf, seed)
    cfg.Q55_floor_hook = qf;
    s = temp_run_powerlaw_diag(cfg, struct('seed', seed, 'verbose', false));
end
