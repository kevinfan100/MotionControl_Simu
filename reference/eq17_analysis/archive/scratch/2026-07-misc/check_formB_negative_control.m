% STATUS: ACTIVE (formB_ws pre-Tier-1 test ladder, step 3: negative control) -- contract-first; targets model/controller/motion_control_law_formB_ws.m + test_script/integration/run_formB_ws.m (implementation may not exist yet)
%CHECK_FORMB_NEGATIVE_CONTROL  Ladder step 3: all-parameters-locked negative
%   control for the Form B unknown-wall 7-state pair.
%
%   Design: lock ALL of (b, p, w_s) at plant-truth-matched values (minimax
%   fit of the Form B law to the simulator's own correction curve over the
%   trajectory's height range -- perp for z, para for x/y) and start from a
%   perfect on-manifold init (the default a_bar seed = gain law at the seed
%   parameters, which with truth-matched parameters is exact to the fit's own
%   sup error). Every parameter channel is then removed from the filter; what
%   remains is the (delay-line + a_bar_w) machinery driven by y1/y2.
%
%   WHAT PASS PROVES: given the truth shape, the filter machinery itself
%   introduces NO systematic drift in a_hat -- residuals sit inside the
%   derived budgets (form's fit floor + measured readout excesses + 3 sigma
%   of the claimed thermal floor), the drift slope over the steady window is
%   bounded, and the lock mechanism is bit-exact. A FAIL here isolates an
%   estimation-machinery bug (sign, container, normalization) from any
%   model-shape or prior question, because both were handed the truth.
%
%   Judgement is z-only (perp axis, the mainline scope); x/y are reported
%   unjudged (Form B on parallel axes shares the known structural limits of
%   the family near the wall).
%
%   Budgets are derived or measured, never tuned:
%     - fit sup error: computed in this script (the form's own floor);
%     - stationary readout excess +1.0% and motion excess +5.2% @ 1 Hz:
%       measured 2026-07-28 (defect-2 record, formB_ws_ref.tex S8 OPEN);
%     - thermal floor: the filter's own claimed sqrt(P_aa) (logged).
%
%   INTERFACE CONTRACT (shared with smoke_formB_ws.m): ctrl_const_override
%   knobs lock_b / lock_p / lock_ws (lock = H col zeroed + K entry zeroed +
%   P row/col frozen) and per-axis seeds b_init / p_init / ws_init (3x1
%   accepted, expgain b_init precedent); logs a_hat_out, a_true_out, P_a_out
%   (sqrt, um/pN), b_hat_out, p_hat_out, ws_hat_out, P_b_out, P_p_out,
%   P_ws_out per the run_5state_expgain-analogous schema.

clear; clc;
here = fileparts(mfilename('fullpath'));
proj = fullfile(here, '..', '..');
addpath(genpath(proj));

% ---------------- Named constants (no magic numbers) ----------------
SEED = 42;                        % driver default seed (run_5state_expgain)
% Derived seeds (formB_ws_ref.tex S10) -- fminsearch starting point only:
B0_SEED  = 9/8;                   % length scale [R]
P0_SEED  = 1;                     % exponent
WS0_SEED = 1;                     % contact position [R]
N_TRUTH_GRID   = 400;             % truth-curve fit grid points
WS_CEILING_GAP = 0.5;             % [R] fit barrier: ws must stay this far below the trough
STATIONARY_READOUT_EXCESS = 0.010;  % measured stationary Var(dw_r) excess (+1.0%, 2026-07-28)
DEFECT2_MOTION_ENVELOPE   = 0.052;  % measured motion-time readout excess @ 1 Hz (+5.2%, 2026-07-28)
SIGMA_MULT       = 3;             % 3-sigma allowance on the claimed floor
HONESTY_MAX      = 5;             % order-of-magnitude honesty band (gross P over-confidence catch)
DIVERGE_LIMIT_UM = 0.5;           % house smoke divergence bound [um]
LOCK_TOL         = 1e-12;         % frozen-slot drift tolerance (abs; slots are O(1))
SETTLE_SKIP_S    = 0.15;          % [s] skip after t=0 (> 10x the a_pd/a_cov EWMA time constants)

% ---------------- Scenario: canonical shape, longer steady window ----------
cfg = user_config();
cfg.trajectory_type = 'osc';
cfg.h_init    = 50;               % h_bar_0 = 22.2
cfg.h_bottom  = 4.5;              % trough h_bar = 2.0
cfg.amplitude = 2.5;              % [um]
cfg.frequency = 1;                % [Hz]
cfg.n_cycles  = 4;                % longer osc window than the smoke for the drift fit
cfg.t_hold    = 0.5;              % [s]
cfg.t_descend_override = 1.0;     % [s]
cfg.T_sim     = 5.5;              % [s] = t_hold + descent + n_cycles/frequency
pc = physical_constants();
cfg.h_min     = 1.05 * pc.R;      % [um]
cfg.ctrl_enable       = true;
cfg.thermal_enable    = true;
cfg.meas_noise_enable = true;
cfg.lambda_c = 0.7;
cfg.a_pd  = 0.05;
cfg.a_cov = 0.05;
cfg.meas_noise_std = [0.00062; 0.00057; 0.00331];   % [um]
cfg.h_bar_safe = 1.5;

fprintf('========= formB_ws negative control (ladder step 3) =========\n');

% ---------------- Truth-matched parameters (minimax fit) ----------------
% Fit the Form B law to the simulator's OWN truth (calc_correction_functions)
% over the realised height range. This is an analysis-side fit: the
% controller still never reads c(h_bar); it only receives three locked
% numbers, exactly as a calibration would hand them over.
h_lo = cfg.h_bottom / pc.R;       % [R] trajectory trough
h_hi = cfg.h_init   / pc.R;       % [R] starting height
h_grid = logspace(log10(h_lo), log10(h_hi), N_TRUTH_GRID).';
a_bar_true_para = zeros(N_TRUTH_GRID, 1);
a_bar_true_perp = zeros(N_TRUTH_GRID, 1);
for i = 1:N_TRUTH_GRID
    [c_para_i, c_perp_i] = calc_correction_functions(h_grid(i));
    a_bar_true_para(i) = 1 / c_para_i;    % a_bar_true = a/a_o = 1/c
    a_bar_true_perp(i) = 1 / c_perp_i;
end

ws_ceiling = h_lo - WS_CEILING_GAP;
fmopts = optimset('TolX', 1e-12, 'TolFun', 1e-12, ...
                  'MaxFunEvals', 2e4, 'MaxIter', 2e4, 'Display', 'off');
th0 = [log(B0_SEED); log(P0_SEED); WS0_SEED];

th_perp = fminsearch(@(th) local_fit_obj(th, h_grid, a_bar_true_perp, ws_ceiling), th0, fmopts);
th_perp = fminsearch(@(th) local_fit_obj(th, h_grid, a_bar_true_perp, ws_ceiling), th_perp, fmopts);
sup_perp = local_fit_obj(th_perp, h_grid, a_bar_true_perp, ws_ceiling);

th_para = fminsearch(@(th) local_fit_obj(th, h_grid, a_bar_true_para, ws_ceiling), th0, fmopts);
th_para = fminsearch(@(th) local_fit_obj(th, h_grid, a_bar_true_para, ws_ceiling), th_para, fmopts);
sup_para = local_fit_obj(th_para, h_grid, a_bar_true_para, ws_ceiling);

b_fit_perp = exp(th_perp(1)); p_fit_perp = exp(th_perp(2)); ws_fit_perp = th_perp(3);
b_fit_para = exp(th_para(1)); p_fit_para = exp(th_para(2)); ws_fit_para = th_para(3);

fprintf('truth fit over h_bar in [%.2f, %.2f] (%d pts, minimax rel err):\n', ...
        h_lo, h_hi, N_TRUTH_GRID);
fprintf('  perp (z):   b = %.4f  p = %.4f  ws = %.4f  sup err = %.4f%%\n', ...
        b_fit_perp, p_fit_perp, ws_fit_perp, 100 * sup_perp);
fprintf('  para (x,y): b = %.4f  p = %.4f  ws = %.4f  sup err = %.4f%%\n', ...
        b_fit_para, p_fit_para, ws_fit_para, 100 * sup_para);
fprintf('--------------------------------------------------------------\n');

% ---------------- Lock everything at the truth-matched values --------------
ov = struct();
ov.lock_b  = true;
ov.lock_p  = true;
ov.lock_ws = true;
ov.b_init  = [b_fit_para;  b_fit_para;  b_fit_perp];
ov.p_init  = [p_fit_para;  p_fit_para;  p_fit_perp];
ov.ws_init = [ws_fit_para; ws_fit_para; ws_fit_perp];

% ---------------- Run ----------------
simOut = run_formB_ws(cfg, struct('seed', SEED, 'verbose', false, ...
                                  'ctrl_const_override', ov));

t = simOut.tout;
N = numel(t);
all_ok = true;

% ---------------- N1: lock integrity (bit-exact freeze) ----------------
lock_err_b  = max(abs(simOut.b_hat_out  - ov.b_init.'),  [], 'all');
lock_err_p  = max(abs(simOut.p_hat_out  - ov.p_init.'),  [], 'all');
lock_err_ws = max(abs(simOut.ws_hat_out - ov.ws_init.'), [], 'all');
pfrozen = max([max(abs(simOut.P_b_out  - simOut.P_b_out(1, :)),  [], 'all'), ...
               max(abs(simOut.P_p_out  - simOut.P_p_out(1, :)),  [], 'all'), ...
               max(abs(simOut.P_ws_out - simOut.P_ws_out(1, :)), [], 'all')]);
ok_lock = (lock_err_b <= LOCK_TOL) && (lock_err_p <= LOCK_TOL) && ...
          (lock_err_ws <= LOCK_TOL) && (pfrozen <= LOCK_TOL);
all_ok = all_ok && ok_lock;
local_print_check('N1: locks bit-exact (b, p, ws, and P frozen)', ok_lock, ...
    sprintf('max drift b/p/ws = %.1e/%.1e/%.1e, P = %.1e (tol %.0e)', ...
            lock_err_b, lock_err_p, lock_err_ws, pfrozen, LOCK_TOL));

% ---------------- N2: health ----------------
finite_ok = all(isfinite(simOut.a_hat_out(:))) && all(isfinite(simOut.p_true_out(:)));
e_all = simOut.p_d_out(2:end, :) - simOut.p_true_out(1:end-1, :);
max_abs_e_um = max(abs(e_all(:)));
ok_health = finite_ok && (max_abs_e_um < DIVERGE_LIMIT_UM);
all_ok = all_ok && ok_health;
local_print_check('N2: run healthy (finite, bounded)', ok_health, ...
    sprintf('finite = %d, max|e| = %.4f um (limit %.1f)', ...
            finite_ok, max_abs_e_um, DIVERGE_LIMIT_UM));

% ---------------- Residual metrics (z axis) ----------------
t_osc0   = cfg.t_hold + cfg.t_descend_override;          % oscillation start [s]
hold_win = (t > SETTLE_SKIP_S) & (t < cfg.t_hold);       % initial hold
osc_win  = t > (t_osc0 + 1 / cfg.frequency);             % steady osc (skip 1st cycle)

ratio_z   = simOut.a_hat_out(:, 3) ./ max(simOut.a_true_out(:, 3), eps);
bias_hold = mean(ratio_z(hold_win)) - 1;
bias_osc  = mean(ratio_z(osc_win))  - 1;

tw = t(osc_win); rw = ratio_z(osc_win);
pf_lin      = polyfit(tw - tw(1), rw, 1);
drift_total = pf_lin(1) * (tw(end) - tw(1));             % ratio drift over the window
r_detr      = rw - polyval(pf_lin, tw - tw(1));
fluct_frac  = std(r_detr);                               % detrended fluctuation of the ratio

% claimed thermal floor, as a fraction of the gain (P_a_out is sqrt, um/pN)
floor_hold = mean(simOut.P_a_out(hold_win, 3)) / mean(simOut.a_true_out(hold_win, 3));
floor_osc  = mean(simOut.P_a_out(osc_win, 3))  / mean(simOut.a_true_out(osc_win, 3));
resid_rms  = sqrt(mean((rw - 1).^2));                    % total residual, floor comparison
honesty    = fluct_frac / max(floor_osc, realmin);

% ---------------- N3-N6: derived-budget gates (z) ----------------
tol_hold     = sup_perp + STATIONARY_READOUT_EXCESS + SIGMA_MULT * floor_hold;
tol_drift    = sup_perp + SIGMA_MULT * floor_osc;
tol_bias_osc = sup_perp + DEFECT2_MOTION_ENVELOPE + SIGMA_MULT * floor_osc;

ok_hold = abs(bias_hold) <= tol_hold;
all_ok = all_ok && ok_hold;
local_print_check('N3: hold bias inside stationary budget (z)', ok_hold, ...
    sprintf('bias = %+.3f%% (budget %.3f%% = fit %.3f + stat %.1f + %dsig %.3f)', ...
            100 * bias_hold, 100 * tol_hold, 100 * sup_perp, ...
            100 * STATIONARY_READOUT_EXCESS, SIGMA_MULT, 100 * SIGMA_MULT * floor_hold));

ok_drift = abs(drift_total) <= tol_drift;
all_ok = all_ok && ok_drift;
local_print_check('N4: no systematic drift over steady window (z)', ok_drift, ...
    sprintf('drift = %+.3f%% over %.1f s (budget %.3f%%)', ...
            100 * drift_total, tw(end) - tw(1), 100 * tol_drift));

ok_bias = abs(bias_osc) <= tol_bias_osc;
all_ok = all_ok && ok_bias;
local_print_check('N5: motion bias inside known-defect budget (z)', ok_bias, ...
    sprintf('bias = %+.3f%% (budget %.3f%% incl. defect-2 %.1f%%)', ...
            100 * bias_osc, 100 * tol_bias_osc, 100 * DEFECT2_MOTION_ENVELOPE));

ok_hon = honesty <= HONESTY_MAX;
all_ok = all_ok && ok_hon;
local_print_check('N6: fluctuation vs claimed thermal floor (z)', ok_hon, ...
    sprintf('detrended fluct %.3f%% / floor %.3f%% = %.2f (max %g)', ...
            100 * fluct_frac, 100 * floor_osc, honesty, HONESTY_MAX));

% ---------------- Report (numbers to console, x/y unjudged) ----------------
fprintf('--------------------------------------------------------------\n');
fprintf('z residual detail: RMS(a_hat/a_true - 1) = %.3f%% over steady window\n', 100 * resid_rms);
fprintf('                   claimed floor = %.3f%% -> RMS/floor = %.2f\n', ...
        100 * floor_osc, resid_rms / max(floor_osc, realmin));
for ax = 1:2
    r_ax = simOut.a_hat_out(:, ax) ./ max(simOut.a_true_out(:, ax), eps);
    fprintf('%s (para, report-only): osc bias = %+.3f%%, fit floor = %.3f%%\n', ...
            char('x' + ax - 1), 100 * (mean(r_ax(osc_win)) - 1), 100 * sup_para);
end
fprintf('gate fraction (osc, x/y/z) = %.1f / %.1f / %.1f %%; steps N = %d\n', ...
        100 * mean(simOut.gate_out(osc_win, 1)), 100 * mean(simOut.gate_out(osc_win, 2)), ...
        100 * mean(simOut.gate_out(osc_win, 3)), N);

fprintf('--------------------------------------------------------------\n');
if all_ok
    fprintf('NEGATIVE CONTROL OVERALL: PASS\n');
else
    fprintf('NEGATIVE CONTROL OVERALL: FAIL\n');
end
fprintf('==============================================================\n');


%% =================== Local helpers ===================

function e_sup = local_fit_obj(theta_log, h_grid, a_bar_true, ws_ceiling)
%LOCAL_FIT_OBJ  Minimax relative error of the Form B law against the truth.
%   theta_log = [log(b); log(p); ws] -- log parameterization keeps b, p > 0;
%   a barrier keeps ws below the trajectory trough so w - ws > 0 on the grid.
    b  = exp(theta_log(1));
    p  = exp(theta_log(2));
    ws = theta_log(3);
    if ws >= ws_ceiling
        e_sup = 1e6 + (ws - ws_ceiling);   % barrier; optimum sits far inside
        return;
    end
    a_mod = 1 - (1 + (h_grid - ws) ./ b).^(-p);
    e_sup = max(abs(a_mod - a_bar_true) ./ a_bar_true);
end


function local_print_check(name, ok, detail)
%LOCAL_PRINT_CHECK  One PASS/FAIL line per check.
    if ok
        tag = 'PASS';
    else
        tag = 'FAIL';
    end
    fprintf('[%s] %-46s %s\n', tag, name, detail);
end
