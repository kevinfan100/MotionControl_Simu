%SMOKE_5STATE_POWERLAW  Smoke test for the NEW 5-state power-law controller.
%   Canonical scenario: hold(0.5s) -> descend(1s) -> 1 Hz near-wall oscillation,
%   thermal + measurement noise ON, single seed, ~4.8 s. Reports run health,
%   tracking std per axis, a_hat/a_true ratio, and p_hat behaviour.
%   Scratch harness (chat 2026-07-24); delete after the power-law verification.

clear; clc;
here = fileparts(mfilename('fullpath'));
proj = fullfile(here, '..', '..');
addpath(genpath(proj));

% ---------------- Canonical scenario config ----------------
cfg = user_config();
cfg.trajectory_type = 'osc';
cfg.h_init   = 50;                 % start far from wall (h_bar ~ 22)
cfg.h_bottom = 4.5;                % osc trough h_bar ~ 2.0 (relaxed, gate-free)
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
% Pf_a_frac deliberately NOT set: the controller now defaults to 5/h_bar_0^2
% (= 0.0101 here), matched to the asymptotic a_h[0] seed's own accuracy. The
% old 0.03 override was a patch for the pre-fdet cold-start spike and would
% now be 3x looser than the honest prior.

seed = 7;
lastwarn('');   % clear warning state to catch RCOND/singular

sim = run_5state_powerlaw(cfg, struct('seed', seed, 'verbose', true));

[wmsg, wid] = lastwarn();

% ---------------- Metrics ----------------
t   = sim.tout;
Pv  = sim.meta.params_value;
axis_name = {'x', 'y', 'z'};

% tracking error: desired[k] vs true[k-1] (matches L1 convention) [um -> nm]
e_all = sim.p_d_out(2:end, :) - sim.p_true_out(1:end-1, :);
t_e   = t(2:end);
osc   = t_e > 1.6;                                  % steady oscillation window
track_std_nm = 1e3 * std(e_all(osc, :), 0, 1);
track_mean_nm = 1e3 * mean(e_all(osc, :), 1);
max_abs_e_um  = max(abs(e_all(:)));

% a_hat vs a_true (ratio over osc window)
osc_full = t > 1.6;
a_ratio = mean(sim.a_hat_out(osc_full, :) ./ max(sim.a_true_out(osc_full, :), eps), 1);

% p_hat behaviour
p_mean = mean(sim.p_hat_out(osc_full, :), 1);
p_std  = std(sim.p_hat_out(osc_full, :), 0, 1);

% health
any_nan = any(~isfinite(sim.p_true_out(:))) || any(~isfinite(sim.a_hat_out(:))) ...
          || any(~isfinite(sim.p_hat_out(:)));
diverged = max_abs_e_um > 0.5;
gate_frac = mean(sim.gate_out(osc_full, :), 1);

% ---------------- Report ----------------
fprintf('\n================ 5-state power-law SMOKE TEST ================\n');
fprintf('scenario: hold %.1fs -> descend %.1fs -> %g Hz osc, T=%.1fs, seed=%d\n', ...
        cfg.t_hold, cfg.t_descend_override, cfg.frequency, cfg.T_sim, seed);
fprintf('h_bar range (true): [%.2f, %.2f]\n', min(sim.h_bar_out), max(sim.h_bar_out));
fprintf('-------------------------------------------------------------\n');
fprintf('RUN HEALTH:  NaN=%d  diverged(|e|>0.5um)=%d  max|e|=%.3f um\n', ...
        any_nan, diverged, max_abs_e_um);
if isempty(wmsg)
    fprintf('WARNINGS:    none (no RCOND/singular)\n');
else
    fprintf('WARNINGS:    [%s] %s\n', wid, wmsg);
end
fprintf('-------------------------------------------------------------\n');
fprintf('%-6s | %-14s | %-14s | %-16s | %-14s | gate%%\n', ...
        'axis', 'track std nm', 'track mean nm', 'a_hat/a_true', 'p_hat mean(std)');
for ax = 1:3
    fprintf('%-6s | %12.2f   | %+12.2f   | %14.3f   | %6.3f (%.3f)  | %5.1f\n', ...
            axis_name{ax}, track_std_nm(ax), track_mean_nm(ax), a_ratio(ax), ...
            p_mean(ax), p_std(ax), 100 * gate_frac(ax));
end
fprintf('=============================================================\n');
fprintf('(sibling eq17 healthy tracking std ~ 24-30 nm; a_hat/a_true ~ 0.6-1.0)\n');

save(fullfile(here, 'temp_smoke_5state_powerlaw.mat'), 'sim', 'track_std_nm', ...
     'a_ratio', 'p_mean', 'p_std', 'cfg');
