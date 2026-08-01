% STATUS: ACTIVE (formB_ws pre-Tier-1 test ladder, step 2: normalization) -- contract-first; targets model/controller/motion_control_law_formB_ws.m + test_script/integration/run_formB_ws.m (implementation may not exist yet)
%CHECK_FORMB_NORMALIZATION  Ladder step 2: value-preserving checks of the
%   full normalization (lengths /R, a_bar = a/a_o, f_bar = a_o*f).
%
%   Spec: formB_ws.tex S8 (Q entries) + formB_ws_ref.tex Conventions; the
%   coding checklist marks Q34/Q44 as the most dangerous cells ("old formula
%   copied into the new coordinates = silent wrong weight"). This step pins
%   the container identities that make such a copy impossible to miss:
%
%     (a) Q33 = kappa_T * a_bar  must be numerically IDENTICAL to the expgain
%         container (4*kB*T/R) * a_h at the same gain/height,
%         kappa_T := 4*kB*T*a_o/R (dimensionless), a_o = dt/(gamma_N*R);
%     (b) the force boundary f = f_bar/a_o must round-trip at machine
%         precision (a_o's ONLY legal appearances: kappa_T definition, force
%         output conversion, display layer);
%     (c) a_bar*f_bar == a*f identically (the value-preserving statement of
%         the whole normalization).
%
%   WHAT PASS PROVES: the normalized containers carry exactly the same
%   numbers as the physical ones -- machine-precision identity, not
%   approximation -- both at the formula level (Part A, no simulation) and
%   inside the running controller (Part B, logged quantities). It proves
%   nothing about filter accuracy.
%
%   INTERFACE CONTRACT (additions over run_5state_expgain's schema; shared
%   with smoke_formB_ws.m): simOut fields Q33_out, a_bar_Q_out (the a_bar
%   actually used to build Q33 this step), f_bar_out, a_bar_hat_out (all
%   N x 3, internal non-dimensional), a_hat_out [um/pN], a_nom [um/pN].

clear; clc;
here = fileparts(mfilename('fullpath'));
proj = fullfile(here, '..', '..');
addpath(genpath(proj));

% ---------------- Named constants (no magic numbers) ----------------
SEED          = 7;            % canonical smoke seed
B0_SEED       = 9/8;          % Form B length-scale seed [R] (derived anchor, S10)
P0_SEED       = 1;            % Form B exponent seed (derived anchor, S10)
WS0_SEED      = 1;            % nominal contact position [R] (S10)
H_GRID_MIN    = 1.2;          % height grid lower edge [R] (inside validity, > ws)
H_GRID_MAX    = 25;           % height grid upper edge [R] (covers canonical h_bar_0 = 22.2)
N_H_GRID      = 61;           % height grid points
F_DECADE_LO   = -3;           % force spot grid: 10^-3 .. 10^3 pN, both signs
F_DECADE_HI   = 3;
N_F_GRID      = 25;
TOL_IDENT     = 16 * eps;     % product-reordering identity (a few ulps)
TOL_ROUNDTRIP = 4 * eps;      % one multiply + one divide
TOL_LOOP      = 64 * eps;     % in-loop: allows controller-side op-order variants

fprintf('========= formB_ws normalization check (ladder step 2) =========\n');

% ================= Part A: formula-level identities =================
pc      = physical_constants();
Ts      = pc.Ts;                      % [s]
R       = pc.R;                       % [um]
gamma_N = pc.gamma_N;                 % [pN*s/um]
kBT     = pc.k_B * pc.T;              % [pN*um]
a_o_nd  = Ts / (gamma_N * R);         % [1/pN]  far-field non-dimensional gain
kappa_T = 4 * kBT * a_o_nd / R;       % [-]     thermal constant (spec S8)

fprintf('constants: a_o = %.6e 1/pN, kappa_T = %.6e (dimensionless)\n', a_o_nd, kappa_T);
fprintf('----------------------------------------------------------------\n');

h_grid = logspace(log10(H_GRID_MIN), log10(H_GRID_MAX), N_H_GRID).';   % [R]
a_bar  = local_formB_gain(h_grid, B0_SEED, P0_SEED, WS0_SEED);         % [-]
a_h_nd = a_o_nd .* a_bar;                                              % [1/pN]

% (A1) CURRENT-STEP COMPONENT of Q33 vs the expgain sibling's (4kBT/R)*a_h.
%      DELIBERATE DIVERGENCE (S8 revision 2026-08-01, ref S8 note (i)): the
%      Form B container is now the full Var(eps_w) -- current thermal step
%      + (1-lc)^2 thermal history + (1-lc)^2 n_w feedthrough -- so the TOTAL
%      no longer equals the sibling's. Only this first term still does, and
%      that is what A1 pins; B1 pins the full closed form.
Q33_cur = kappa_T .* a_bar;
Q33_old = (4 * kBT / R) .* a_h_nd;
errA1 = max(abs(Q33_cur - Q33_old) ./ max(abs(Q33_old), realmin));
okA1  = errA1 <= TOL_IDENT;
local_print_check('A1: Q33 current-step term == (4kBT/R)*a_h', okA1, errA1, TOL_IDENT);

% (A2) f_bar/a_o round trip across 6 decades, both signs
f_mag  = logspace(F_DECADE_LO, F_DECADE_HI, N_F_GRID).';               % [pN]
f_spot = [f_mag; -f_mag];                                              % [pN]
f_bar  = a_o_nd * f_spot;                                              % [-]
f_back = f_bar / a_o_nd;                                               % [pN]
errA2 = max(abs(f_back - f_spot) ./ abs(f_spot));
okA2  = errA2 <= TOL_ROUNDTRIP;
local_print_check('A2: f -> f_bar = a_o*f -> f_bar/a_o round trip', okA2, errA2, TOL_ROUNDTRIP);

% (A3) a_bar*f_bar == a*f (outer product over heights x forces)
lhs = a_bar * f_bar.';         % [-]  a_bar(i) * f_bar(j)
rhs = a_h_nd * f_spot.';       % [-]  a(i) [1/pN] * f(j) [pN]
errA3 = max(abs(lhs - rhs) ./ max(abs(rhs), realmin), [], 'all');
okA3  = errA3 <= TOL_IDENT;
local_print_check('A3: a_bar*f_bar == a*f (spot grid)', okA3, errA3, TOL_IDENT);

% ================= Part B: in-loop, logged quantities =================
fprintf('----------------------------------------------------------------\n');
fprintf('Part B: short Tier-1 run, identities on LOGGED quantities\n');

cfg = user_config();
cfg.trajectory_type = 'osc';
cfg.h_init    = 50;                 % h_bar_0 = 22.2
cfg.h_bottom  = 4.5;                % trough h_bar = 2.0
cfg.amplitude = 2.5;                % [um]
cfg.frequency = 1;                  % [Hz]
cfg.n_cycles  = 1;
cfg.t_hold    = 0.5;                % [s]
cfg.t_descend_override = 1.0;       % [s]
cfg.T_sim     = 2.5;                % [s] short: hold + descent + half a cycle
cfg.h_min     = 1.05 * pc.R;        % [um]
cfg.ctrl_enable       = true;
cfg.thermal_enable    = true;
cfg.meas_noise_enable = true;
cfg.lambda_c = 0.7;
cfg.a_pd  = 0.05;
cfg.a_cov = 0.05;
cfg.meas_noise_std = [0.00062; 0.00057; 0.00331];   % [um]
cfg.h_bar_safe = 1.5;

simOut = run_formB_ws(cfg, struct('seed', SEED, 'verbose', false, ...
                                  'ctrl_const_override', struct('lock_ws', true)));

% Rebuild the two boundary constants from the run's OWN params, so Part B is
% pinned to what the controller actually saw, not to physical_constants().
Pv = simOut.meta.params_value;
a_o_run  = Pv.ctrl.Ts / (Pv.ctrl.gamma * Pv.common.R);          % [1/pN]
kappa_T_run = 4 * Pv.ctrl.k_B * Pv.ctrl.T * a_o_run / Pv.common.R;   % [-]

% (B1) controller Q33 == the FULL S8 closed form rebuilt from the logged
%      a_bar history and the run's own constants, at every logged step:
%         Q33[k] = kappa_T*( a_bar[k] + (1-lc)^2*sum_{i=1..d} a_bar[k-i] )
%                  + (1-lc)^2*sigma2_nw
%      The controller feeds that sum from a_ctrl_km1/km2, which equal the
%      logged a_bar_Q of the previous steps (a_ctrl and the Q-side a_bar are
%      the same floored state). Row 1 is the init call, where the whole
%      history equals the seed -- hence the index clamp at 1.
lc_run    = simOut.ctrl_const.lambda_c;
d_run     = simOut.ctrl_const.d;
sig2nw_nd = Pv.ctrl.sigma2_noise(:).' / Pv.common.R^2;      % 1x3 [-]
aQ        = simOut.a_bar_Q_out;                              % N x 3
n_log     = size(aQ, 1);
hist_sum  = zeros(size(aQ));
for i_lag = 1:d_run
    idx      = max((1:n_log).' - i_lag, 1);
    hist_sum = hist_sum + aQ(idx, :);
end
q_pred = kappa_T_run * (aQ + (1 - lc_run)^2 * hist_sum) ...
         + (1 - lc_run)^2 * repmat(sig2nw_nd, n_log, 1);
errB1 = max(abs(simOut.Q33_out - q_pred) ./ max(abs(q_pred), realmin), [], 'all');
okB1  = errB1 <= TOL_LOOP;
local_print_check('B1: logged Q33 == full S8 closed form (all steps)', okB1, errB1, TOL_LOOP);

% (B2) output conversion: logged f_d [pN] == logged f_bar / a_o
f_pred  = simOut.f_bar_out / a_o_run;
mask_f  = abs(simOut.f_d_out) > 0;
if any(mask_f(:))
    errB2 = max(abs(f_pred(mask_f) - simOut.f_d_out(mask_f)) ./ abs(simOut.f_d_out(mask_f)));
else
    errB2 = Inf;   % a run with identically zero force cannot certify the boundary
end
zeros_consistent = all(simOut.f_bar_out(~mask_f) == 0);
okB2 = (errB2 <= TOL_ROUNDTRIP) && zeros_consistent;
local_print_check('B2: logged f_d == f_bar/a_o (round trip)', okB2, errB2, TOL_ROUNDTRIP);
if ~zeros_consistent
    fprintf('       (zero-force steps carry nonzero f_bar -- boundary leak)\n');
end

% (B3) value preservation across BOTH boundaries at once:
%      a_phys*f_phys [um] == R * (a_bar*f_bar), using only logged data.
lhsB = simOut.a_hat_out .* simOut.f_d_out;                       % [um]
rhsB = Pv.common.R * (simOut.a_bar_hat_out .* simOut.f_bar_out); % [um]
mask3 = abs(rhsB) > 0;
if any(mask3(:))
    errB3 = max(abs(lhsB(mask3) - rhsB(mask3)) ./ abs(rhsB(mask3)));
else
    errB3 = Inf;
end
okB3 = errB3 <= TOL_LOOP;
local_print_check('B3: a_hat.*f_d == R*(a_bar.*f_bar) (logged)', okB3, errB3, TOL_LOOP);

% (B4) display layer: a_hat_out [um/pN] == a_nom * a_bar_hat_out
errB4 = max(abs(simOut.a_hat_out - simOut.a_nom * simOut.a_bar_hat_out) ...
            ./ max(abs(simOut.a_hat_out), realmin), [], 'all');
okB4  = errB4 <= TOL_LOOP;
local_print_check('B4: a_hat == a_nom*a_bar_hat (display layer)', okB4, errB4, TOL_LOOP);

% ---------------- Verdict ----------------
fprintf('----------------------------------------------------------------\n');
all_ok = okA1 && okA2 && okA3 && okB1 && okB2 && okB3 && okB4;
if all_ok
    fprintf('NORMALIZATION OVERALL: PASS (all identities at machine precision)\n');
else
    fprintf('NORMALIZATION OVERALL: FAIL\n');
end
fprintf('================================================================\n');


%% =================== Local helpers ===================

function a_bar = local_formB_gain(w_bar, b, p, ws)
%LOCAL_FORMB_GAIN  Form B gain law (formB_ws.tex S1), fully normalized:
%   a_bar = 1 - (1 + (w_bar - ws)/b)^(-p),  valid for w_bar > ws, b > 0, p > 0.
    a_bar = 1 - (1 + (w_bar - ws) ./ b).^(-p);
end


function local_print_check(name, ok, err, tol)
%LOCAL_PRINT_CHECK  One PASS/FAIL line with the max relative error in ulps.
    if ok
        tag = 'PASS';
    else
        tag = 'FAIL';
    end
    fprintf('[%s] %-46s max rel err = %.3g (%.1f ulp, tol %.1f ulp)\n', ...
            tag, name, err, err / eps, tol / eps);
end
