%TEMP_DIAG_POWERLAW_PHASE2  Phase-1/2 evidence: WHY p_hat swings in the descent.
%   Uses an instrumented COPY of the controller (temp_mcl_powerlaw_diag) -- the
%   production file is untouched. Scratch; delete after the diagnosis.
%
%   D: decomposition of the p-update gain  K2(5) = [P45 + P55*H25] / S2
%      -> is p_hat driven by REAL p information (P55*H25) or by a_h-error
%         leakage through the cross-covariance (P45)?
%   E: size of the DERIVATION-DROPPED H(2,4) correction (-dHbar*s_a)*e_ah
%      against the KEPT p term H25*e_p -- same order?
%   F: Pf_p_std sweep (init sensitivity) + p-frozen arm.

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
cfg.h_bar_safe = 1.5;
cfg.Pf_a_frac = 0.03;
seed = 7;

fprintf('running baseline (instrumented copy)...\n');
sim = temp_run_powerlaw_diag(cfg, struct('seed', seed, 'verbose', false));
P = sim.meta.params_value;  t = sim.tout(:);
w = P.wall.w_hat(:);  pz = P.wall.pz;  R = P.common.R;
hb_d = (sim.p_d_out * w - pz) / R;
DG = sim.DG;  az = 3;

fprintf('\n=========== PART D: K2(5) decomposition (z axis) ===========\n');
fprintf('K2(5) = P45/S2 + P55*H25/S2 ; p_hat jump = K2(5)*innov2\n');
fprintf('%7s %8s %11s %11s %11s %11s %9s %9s\n', ...
        't[s]','hbar_d','K25_total','via_P45','via_P55H25','P45','innov2','p_hat');
for tq = [0.30 0.60 0.80 0.95 0.98 1.00 1.05 1.20 1.40 1.50 2.00 3.00 4.00]
    [~, i] = min(abs(t - tq));
    fprintf('%7.3f %8.2f %11.3e %11.3e %11.3e %11.3e %9.3f %9.4f\n', ...
        t(i), hb_d(i), DG.dg_K25(i,az), DG.dg_K25_via_P45(i,az), ...
        DG.dg_K25_via_P55H(i,az), DG.dg_P45(i,az), ...
        1e3*sim.innov_y2_out(i,az), sim.p_hat_out(i,az));
end
mv = t > 0.5 & t < 1.5 & DG.dg_S2(:,az) > 0;
r_share = abs(DG.dg_K25_via_P55H(mv,az)) ./ max(abs(DG.dg_K25(mv,az)), eps);
fprintf('\ndescent window: share of K2(5) coming from the REAL p term (P55*H25):\n');
fprintf('   median %.3e   max %.3e   -> P45 leakage carries %.4f%% of the p update\n', ...
        median(r_share), max(r_share), 100*(1-median(r_share)));

fprintf('\n=========== PART E: dropped H(2,4) correction vs kept H(2,5) term ===========\n');
fprintf('y2 truth: e_y2 = [1 - dHbar*s_a]*e_ah - dHbar*(s_h/p)*e_p + n_a\n');
fprintf('derivation KEEPS the e_p term, DROPS the e_ah correction. Compare them:\n');
e_ah = sim.a_hat_out(:,az) - sim.a_true_out(:,az);              % actual gain error
e_p  = sim.p_hat_out(:,az) - 1.0;                               % actual p error (p_true~1)
term_drop = DG.dg_H24_drop(:,az) .* e_ah;                       % dropped contribution
term_keep = DG.dg_H25(:,az) .* e_p;                             % kept p contribution
fprintf('%7s %8s %13s %13s %9s\n','t[s]','hbar_d','dropped_e_ah','kept_e_p','ratio');
for tq = [0.60 0.80 1.00 1.10 1.20 1.30 1.40 1.45]
    [~, i] = min(abs(t - tq));
    fprintf('%7.3f %8.2f %13.3e %13.3e %9.2f\n', t(i), hb_d(i), ...
        term_drop(i), term_keep(i), abs(term_drop(i))/max(abs(term_keep(i)),eps));
end
md = t > 0.5 & t < 1.5;
fprintf('descent RMS: dropped %.3e   kept %.3e   ratio %.2f\n', ...
        rms(term_drop(md)), rms(term_keep(md)), rms(term_drop(md))/rms(term_keep(md)));

fprintf('\n=========== PART F: init sensitivity (Pf_p_std sweep) ===========\n');
fprintf('%12s | %9s %9s %9s | %9s %9s | %8s %8s\n', 'Pf_p_std', ...
        'p desc min','p desc max','p desc std', 'p osc mean','p osc std', ...
        'trk z nm','a_hat/aT');
arms = [0.3 0.10 0.05 1e-3];
for q = 1:numel(arms)
    c2 = cfg;  c2.Pf_p_std = arms(q);
    s2 = temp_run_powerlaw_diag(c2, struct('seed', seed, 'verbose', false));
    tt = s2.tout(:);  ph = s2.p_hat_out(:,az);
    md2 = tt > 0.5 & tt < 1.5;   mo2 = tt > 1.6;
    e2 = s2.p_d_out(2:end,:) - s2.p_true_out(1:end-1,:);
    te2 = tt(2:end);  mo2e = te2 > 1.6;
    ar = mean(s2.a_hat_out(mo2,az) ./ max(s2.a_true_out(mo2,az), eps));
    fprintf('%12.4g | %9.4f %9.4f %9.4f | %9.4f %9.4f | %8.2f %8.3f\n', ...
        arms(q), min(ph(md2)), max(ph(md2)), std(ph(md2)), ...
        mean(ph(mo2)), std(ph(mo2)), 1e3*std(e2(mo2e,az)), ar);
end

save(fullfile(here, 'temp_powerlaw_diag.mat'), 'sim', 'cfg', '-v7.3');
fprintf('\nsaved temp_powerlaw_diag.mat\n');
