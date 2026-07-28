%SMOKE_5STATE_EXPGAIN  Canonical-scenario smoke test for the 5-state
%   exponential-gain controller (motion_control_law_5state_expgain).
%
%   Scenario (single seed, raw traces, no averaging):
%       hold 0.5 s @ h = 50 um (h_bar = 22.2) -> descend 1.0 s
%       -> 1 Hz oscillation, amplitude 2.5 um, trough h = 4.5 um (h_bar = 2.0)
%       2 cycles, T = 4.8 s, thermal + measurement noise ON, lambda_c = 0.7.
%
%   Reports run health, tracking std, a_hat/a_true, and b_hat against the TRUE
%   b_eff(h_bar) curve -- not against the seed b_0, which is a model
%   assumption, not a truth (the power-law sibling's reading trap, 306c89e).
%
%   b_eff(h_bar) := -ln(1 - a_true/a_o)/ln(h_bar) is the exponent the model
%   would need at that height to be exact. It is an ANALYSIS quantity computed
%   from the exact correction curve; the controller never sees it.
%
%   Focus axis is z (wall-normal). x/y run so the plant is complete but the
%   exponential model is structurally wrong for them (b_eff diverges near the
%   wall for parallel motion), so their numbers are reported, not judged.

clear; clc;
here = fileparts(mfilename('fullpath'));
proj = fullfile(here, '..', '..');
addpath(genpath(proj));

% ---------------- Canonical scenario ----------------
cfg = user_config();
cfg.trajectory_type = 'osc';
cfg.h_init   = 50;                 % h_bar_0 = 22.2
cfg.h_bottom = 4.5;                % trough h_bar = 2.0
cfg.amplitude = 2.5;               % [um]
cfg.frequency = 1;                 % [Hz]
cfg.n_cycles  = 2;
cfg.t_hold    = 0.5;
cfg.t_descend_override = 1.0;
cfg.T_sim     = 4.8;
pc = physical_constants();
cfg.h_min     = 1.05 * pc.R;
cfg.ctrl_enable = true;
cfg.thermal_enable    = true;
cfg.meas_noise_enable = true;
cfg.lambda_c = 0.7;
cfg.a_pd  = 0.05;
cfg.a_cov = 0.05;
cfg.meas_noise_std = [0.00062; 0.00057; 0.00331];   % [um]
cfg.h_bar_safe = 1.5;

seed = 7;
lastwarn('');

sim = run_5state_expgain(cfg, struct('seed', seed, 'verbose', true));

[wmsg, wid] = lastwarn();

% ---------------- Metrics ----------------
t   = sim.tout;
axis_name = {'x', 'y', 'z'};

% tracking error: desired[k] vs true[k-1] (L1 convention) [um -> nm]
e_all = sim.p_d_out(2:end, :) - sim.p_true_out(1:end-1, :);
t_e   = t(2:end);
osc   = t_e > 1.6;                                  % steady oscillation window
track_std_nm  = 1e3 * std(e_all(osc, :), 0, 1);
track_mean_nm = 1e3 * mean(e_all(osc, :), 1);
max_abs_e_um  = max(abs(e_all(:)));

osc_full = t > 1.6;
hold_win = t > 0.1 & t < 0.5;                       % the initial hold

a_ratio_osc  = mean(sim.a_hat_out(osc_full, :) ./ max(sim.a_true_out(osc_full, :), eps), 1);
a_ratio_hold = mean(sim.a_hat_out(hold_win, :) ./ max(sim.a_true_out(hold_win, :), eps), 1);

% b_slope(h_bar) along the realised trajectory = the analysis truth for b.
%   b enters the filter ONLY through a_h' (the level is the a_h state), so the
%   honest reference is the exponent that reproduces the TRUE SLOPE,
%       b_slope = h_bar * a'_true / (a_o - a_true),
%   NOT the one that reproduces the level. The two differ by up to 0.04 here.
a_o_phys = sim.a_nom;
b_eff = sim.h_bar_true_out .* sim.a_prime_true_out ./ max(a_o_phys - sim.a_true_out, eps);
b_err = sim.b_hat_out - b_eff;
b_err_osc_rms = sqrt(mean(b_err(osc_full, :).^2, 1));
b_hon = b_err_osc_rms ./ mean(sim.P_b_out(osc_full, :), 1);   % honesty: |err| / claimed sigma

% health
any_nan = any(~isfinite(sim.p_true_out(:))) || any(~isfinite(sim.a_hat_out(:))) ...
          || any(~isfinite(sim.b_hat_out(:)));
diverged = max_abs_e_um > 0.5;
gate_frac = mean(sim.gate_out(osc_full, :), 1);

% ---------------- Report ----------------
fprintf('\n============== 5-state EXPONENTIAL-GAIN smoke test ==============\n');
fprintf('scenario: hold %.1fs -> descend %.1fs -> %g Hz osc, T=%.1fs, seed=%d\n', ...
        cfg.t_hold, cfg.t_descend_override, cfg.frequency, cfg.T_sim, seed);
fprintf('h_bar range (true): [%.2f, %.2f]\n', min(sim.h_bar_true_out), max(sim.h_bar_true_out));
fprintf('seed: a_h[0]/a_o = %.6f  b_0 = %.4f (z)\n', ...
        sim.a_hat_out(1, 3)/a_o_phys, sim.b_hat_out(1, 3));
fprintf('-----------------------------------------------------------------\n');
fprintf('RUN HEALTH:  NaN=%d  diverged(|e|>0.5um)=%d  max|e|=%.3f um\n', ...
        any_nan, diverged, max_abs_e_um);
if isempty(wmsg)
    fprintf('WARNINGS:    none (no RCOND/singular)\n');
else
    fprintf('WARNINGS:    [%s] %s\n', wid, wmsg);
end
fprintf('-----------------------------------------------------------------\n');
fprintf('%-4s | %10s | %10s | %10s | %10s | %10s | %8s | %5s\n', ...
        'axis', 'trk std nm', 'trk mean', 'a/aT hold', 'a/aT osc', 'b RMSerr', 'honesty', 'gate%');
for ax = 1:3
    fprintf('%-4s | %10.2f | %+10.2f | %10.4f | %10.4f | %10.4f | %8.2f | %5.1f\n', ...
            axis_name{ax}, track_std_nm(ax), track_mean_nm(ax), ...
            a_ratio_hold(ax), a_ratio_osc(ax), b_err_osc_rms(ax), b_hon(ax), ...
            100 * gate_frac(ax));
end
fprintf('-----------------------------------------------------------------\n');
fprintf('z detail: b_slope(true) range [%.4f, %.4f]; b_hat range [%.4f, %.4f]\n', ...
        min(b_eff(2:end, 3)), max(b_eff(2:end, 3)), ...
        min(sim.b_hat_out(:, 3)), max(sim.b_hat_out(:, 3)));
fprintf('          claimed sigma_b(z): %.4f -> %.4f\n', sim.P_b_out(1, 3), sim.P_b_out(end, 3));
fprintf('          a_hat/a_true(z) at t = 1.0 / 2.5 / 4.8 s: %.4f / %.4f / %.4f\n', ...
        interp1(t, sim.a_hat_out(:,3)./sim.a_true_out(:,3), 1.0), ...
        interp1(t, sim.a_hat_out(:,3)./sim.a_true_out(:,3), 2.5), ...
        sim.a_hat_out(end,3)/sim.a_true_out(end,3));
fprintf('=================================================================\n');
fprintf('(honesty = RMS(b_hat - b_eff) / claimed sigma_b; ~1 is honest,\n');
fprintf('  >>1 over-confident, <<1 over-conservative)\n');

% test_results/ is gitignored: simulation output is not a source artefact.
out_dir = fullfile(proj, 'test_results');
if ~exist(out_dir, 'dir'); mkdir(out_dir); end
save(fullfile(out_dir, 'smoke_5state_expgain.mat'), 'sim', 'track_std_nm', ...
     'a_ratio_osc', 'a_ratio_hold', 'b_eff', 'b_err_osc_rms', 'b_hon', 'cfg');
