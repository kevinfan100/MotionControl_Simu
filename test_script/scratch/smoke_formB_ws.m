% STATUS: ACTIVE (formB_ws pre-Tier-1 test ladder, step 1: smoke) -- contract-first; targets model/controller/motion_control_law_formB_ws.m + test_script/integration/run_formB_ws.m (implementation may not exist yet)
%SMOKE_FORMB_WS  Ladder step 1: run-health smoke test for the Form B
%   unknown-wall-position 7-state pair, Tier-1 configuration (w_s locked at
%   the nominal contact position, (b, p) estimated).
%
%   Spec: reference/eq17_analysis/derivation/formB_ws.tex (S1-S8) +
%   formB_ws_ref.tex (S9-S11 contracts). State order
%   [dw1 dw2 dw3 a_bar_w b_w p_w w_s], fully non-dimensional internals
%   (lengths /R, a_bar = a/a_o, f_bar = a_o*f).
%
%   WHAT PASS PROVES: the pair completes the canonical scenario with no
%   NaN/Inf anywhere, the posterior covariance stays positive definite at
%   EVERY step (chol on the active sub-block; zero-variance frozen rows must
%   be exactly zero), the locked w_s slot is genuinely frozen, and the closed
%   loop does not diverge. It proves nothing about estimation accuracy
%   (ladder steps 2-3 and the S11 acceptance suite own that).
%
%   INTERFACE CONTRACT (drop-in analogous to run_5state_expgain /
%   motion_control_law_5state_expgain; the fields below are this ladder's
%   ADDITIONS to that driver's output schema):
%     simOut = run_formB_ws(cfg, opts)
%       opts: .seed .verbose .ctrl_const_override .a_ctrl_override .log_P_full
%     extra logs (N x 3, internal non-dimensional unless noted):
%       a_bar_hat_out, p_hat_out, ws_hat_out, P_p_out / P_ws_out (sqrt of the
%       posterior variances), Q33_out, a_bar_Q_out (the a_bar actually used to
%       build Q33 this step), f_bar_out; P_full_out (N x n x n x 3, n = 9 under ma2_aug, posterior
%       P, internal non-dimensional) iff opts.log_P_full is true. Init-call
%       rows carry the seeds for the parameter slots (expgain precedent), not
%       unfilled zeros.
%     ctrl_const_override knobs: lock_b / lock_p / lock_ws (Tier-1 default
%       lock_ws = true; lock = H col zeroed + K entry zeroed + P row/col
%       frozen), b_init (default 9/8), p_init (1), ws_init (1),
%       Pf_b_std (1/8), Pf_p_std (1/8).
%
%   PD NOTE: if a lock freezes NONZERO cross-covariances while P(4,4) keeps
%   contracting, P can lose positive definiteness. This chol check is the
%   guard for that implementation choice; zeroing the locked rows/cols at
%   lock time is the PD-safe convention.

clear; clc;
here = fileparts(mfilename('fullpath'));
proj = fullfile(here, '..', '..');
addpath(genpath(proj));

% ---------------- Named constants (no magic numbers) ----------------
SEED             = 7;         % canonical smoke seed (smoke_5state_expgain)
N_STATES         = 9;         % [dw1 dw2 dw3 a_bar_w b_w p_w w_s m1 m2]
                              % (ma2_aug production default, 2026-08-01;
                              %  set 7 when running with ma2_aug = false)
WS_SEED          = 1;         % nominal contact position [units of R] (derived seed, S10)
DIVERGE_LIMIT_UM = 0.5;       % house smoke divergence bound [um] (smoke_5state_expgain)
LOCK_TOL         = 1e-12;     % frozen-slot drift tolerance (abs; slots are O(1))
SYM_TOL_REL      = 1e-9;      % P asymmetry tolerance relative to max |diag(P)|

% ---------------- Canonical scenario (same as smoke_5state_expgain) --------
cfg = user_config();
cfg.trajectory_type = 'osc';
cfg.h_init    = 50;                 % h_bar_0 = 22.2
cfg.h_bottom  = 4.5;                % trough h_bar = 2.0
cfg.amplitude = 2.5;                % [um]
cfg.frequency = 1;                  % [Hz]
cfg.n_cycles  = 2;
cfg.t_hold    = 0.5;                % [s]
cfg.t_descend_override = 1.0;       % [s]
cfg.T_sim     = 4.8;                % [s]
pc = physical_constants();
cfg.h_min     = 1.05 * pc.R;        % [um]
cfg.ctrl_enable       = true;
cfg.thermal_enable    = true;
cfg.meas_noise_enable = true;
cfg.lambda_c = 0.7;
cfg.a_pd  = 0.05;
cfg.a_cov = 0.05;
cfg.meas_noise_std = [0.00062; 0.00057; 0.00331];   % [um]
cfg.h_bar_safe = 1.5;

% Tier-1 configuration: pin w_s at the nominal contact (sim truth is 1, so
% the pin carries zero model error); estimate (b, p). Passed explicitly even
% though it is the contract default, so the script documents itself.
ov = struct('lock_ws', true);

fprintf('=============== formB_ws smoke test (ladder step 1) ===============\n');
fprintf('scenario: hold %.1fs -> descend %.1fs -> %g Hz osc, T=%.1fs, seed=%d\n', ...
        cfg.t_hold, cfg.t_descend_override, cfg.frequency, cfg.T_sim, SEED);
fprintf('config: Tier-1 (lock_ws = true, estimate (b, p))\n');
fprintf('-------------------------------------------------------------------\n');

% ---------------- Run ----------------
lastwarn('');
run_ok  = true;
run_err = '';
simOut  = struct();
try
    simOut = run_formB_ws(cfg, struct('seed', SEED, 'verbose', false, ...
                                      'log_P_full', true, ...
                                      'ctrl_const_override', ov));
catch ME
    run_ok  = false;
    run_err = ME.message;
end
[wmsg, wid] = lastwarn();

all_ok = run_ok;
local_print_check('run completes without error', run_ok, run_err);
if ~run_ok
    fprintf('-------------------------------------------------------------------\n');
    fprintf('SMOKE OVERALL: FAIL (driver errored; nothing else checked)\n');
    return;
end

% ---------------- Check: contract fields present ----------------
req_fields = {'p_true_out', 'f_d_out', 'a_hat_out', 'a_bar_hat_out', ...
              'b_hat_out', 'p_hat_out', 'ws_hat_out', ...
              'P_a_out', 'P_b_out', 'P_p_out', 'P_ws_out', ...
              'Q33_out', 'a_bar_Q_out', 'f_bar_out', 'P_full_out', ...
              'h_bar_true_out', 'gate_out', 'p_d_out'};
present = isfield(simOut, req_fields);
ok_fields = all(present);
all_ok = all_ok && ok_fields;
if ok_fields
    local_print_check('contract log fields present', true, sprintf('%d fields', numel(req_fields)));
else
    local_print_check('contract log fields present', false, ...
                      ['missing: ' strjoin(req_fields(~present), ', ')]);
    fprintf('-------------------------------------------------------------------\n');
    fprintf('SMOKE OVERALL: FAIL (contract fields missing; P/state checks skipped)\n');
    return;
end

% ---------------- Check: all states/logs finite ----------------
log_names = {'p_true_out', 'f_d_out', 'a_hat_out', 'a_bar_hat_out', ...
             'b_hat_out', 'p_hat_out', 'ws_hat_out'};
bad_logs = {};
for i = 1:numel(log_names)
    v = simOut.(log_names{i});
    if ~all(isfinite(v(:)))
        bad_logs{end+1} = log_names{i}; %#ok<SAGROW>
    end
end
ok_finite = isempty(bad_logs);
all_ok = all_ok && ok_finite;
if ok_finite
    local_print_check('all logged states finite (no NaN/Inf)', true, '');
else
    local_print_check('all logged states finite (no NaN/Inf)', false, ...
                      ['NaN/Inf in: ' strjoin(bad_logs, ', ')]);
end

% ---------------- Check: P finite / symmetric / PD every step ----------------
Pfull = simOut.P_full_out;
[N_steps, np1, np2, n_ax] = size(Pfull);
ok_dims = (np1 == N_STATES) && (np2 == N_STATES) && (n_ax == 3);
all_ok = all_ok && ok_dims;
local_print_check('P_full_out dims are N x n_state x n_state x 3', ok_dims, ...
                  sprintf('got %d x %d x %d x %d', N_steps, np1, np2, n_ax));

p_finite = true;
p_pd = true;
frozen_leak = false;
worst_asym_rel = 0;
first_bad_k = 0;
first_bad_ax = 0;
if ok_dims
    for ax = 1:3
        for k = 1:N_steps
            Pk = squeeze(Pfull(k, :, :, ax));
            if ~all(isfinite(Pk(:)))
                p_finite = false; first_bad_k = k; first_bad_ax = ax;
                break;
            end
            dscale = max(abs(diag(Pk)));
            if dscale > 0
                worst_asym_rel = max(worst_asym_rel, ...
                                     max(abs(Pk - Pk.'), [], 'all') / dscale);
            end
            Ps = 0.5 * (Pk + Pk.');
            dP = diag(Ps);
            act = dP > 0;
            if any(dP < 0) || any(any(Ps(~act, :) ~= 0))
                p_pd = false;
                frozen_leak = any(any(Ps(~act, :) ~= 0));
                first_bad_k = k; first_bad_ax = ax;
                break;
            end
            [~, chol_flag] = chol(Ps(act, act));
            if chol_flag ~= 0
                p_pd = false; first_bad_k = k; first_bad_ax = ax;
                break;
            end
        end
        if ~p_finite || ~p_pd
            break;
        end
    end
else
    p_finite = false;
    p_pd = false;
end
ok_sym = ok_dims && (worst_asym_rel <= SYM_TOL_REL);
all_ok = all_ok && p_finite && ok_sym && p_pd;

local_print_check('P finite every step', p_finite, ...
                  local_bad_loc_str(p_finite, first_bad_k, first_bad_ax));
local_print_check('P symmetric every step', ok_sym, ...
                  sprintf('worst rel asym = %.2e (tol %.0e)', worst_asym_rel, SYM_TOL_REL));
if p_pd
    local_print_check('P positive definite every step (chol)', true, ...
                      sprintf('%d steps x 3 axes', N_steps));
else
    local_print_check('P positive definite every step (chol)', false, ...
                      sprintf('first failure k=%d ax=%d (frozen-row leak: %d)', ...
                              first_bad_k, first_bad_ax, frozen_leak));
end

% ---------------- Check: closed loop bounded ----------------
% Tracking error convention: desired[k] vs true[k-1] (L1, smoke_5state_expgain)
e_all = simOut.p_d_out(2:end, :) - simOut.p_true_out(1:end-1, :);
max_abs_e_um = max(abs(e_all(:)));
ok_div = max_abs_e_um < DIVERGE_LIMIT_UM;
all_ok = all_ok && ok_div;
local_print_check('no divergence', ok_div, ...
                  sprintf('max|e| = %.4f um (limit %.1f)', max_abs_e_um, DIVERGE_LIMIT_UM));

% ---------------- Check: lock_ws integrity ----------------
ws_drift  = max(abs(simOut.ws_hat_out(:) - WS_SEED));
Pws_drift = max(abs(simOut.P_ws_out - simOut.P_ws_out(1, :)), [], 'all');
ok_lock = (ws_drift <= LOCK_TOL) && (Pws_drift <= LOCK_TOL);
all_ok = all_ok && ok_lock;
local_print_check('locked w_s frozen (state + P)', ok_lock, ...
                  sprintf('max|ws-1| = %.2e, max P_ws drift = %.2e (tol %.0e)', ...
                          ws_drift, Pws_drift, LOCK_TOL));

% ---------------- Check: no linear-algebra warnings ----------------
ok_warn = isempty(wmsg);
all_ok = all_ok && ok_warn;
if ok_warn
    local_print_check('no warnings (RCOND/singular)', true, '');
else
    local_print_check('no warnings (RCOND/singular)', false, sprintf('[%s] %s', wid, wmsg));
end

% ---------------- Info (report-only) ----------------
fprintf('-------------------------------------------------------------------\n');
fprintf('INFO: h_bar range (true) = [%.2f, %.2f]; gate fraction = %.1f / %.1f / %.1f %%\n', ...
        min(simOut.h_bar_true_out), max(simOut.h_bar_true_out), ...
        100 * mean(simOut.gate_out(:, 1)), 100 * mean(simOut.gate_out(:, 2)), ...
        100 * mean(simOut.gate_out(:, 3)));
if ok_dims
    dP_end = diag(squeeze(Pfull(end, :, :, 3)));
    dP_pos = dP_end(dP_end > 0);
    fprintf('INFO: final P diag (z, active slots) in [%.2e, %.2e] -- normalization\n', ...
            min(dP_pos), max(dP_pos));
    fprintf('      sanity: entries should be broadly comparable (checklist item 8)\n');
end
fprintf('INFO: b_hat(z) %.4f -> %.4f, p_hat(z) %.4f -> %.4f (info only at this step)\n', ...
        simOut.b_hat_out(1, 3), simOut.b_hat_out(end, 3), ...
        simOut.p_hat_out(1, 3), simOut.p_hat_out(end, 3));

fprintf('-------------------------------------------------------------------\n');
if all_ok
    fprintf('SMOKE OVERALL: PASS\n');
else
    fprintf('SMOKE OVERALL: FAIL\n');
end
fprintf('===================================================================\n');


%% =================== Local helpers ===================

function local_print_check(name, ok, detail)
%LOCAL_PRINT_CHECK  One PASS/FAIL line per check.
    if ok
        tag = 'PASS';
    else
        tag = 'FAIL';
    end
    if isempty(detail)
        fprintf('[%s] %s\n', tag, name);
    else
        fprintf('[%s] %-42s %s\n', tag, name, detail);
    end
end


function s = local_bad_loc_str(ok, k, ax)
%LOCAL_BAD_LOC_STR  Location string for the first bad P step.
    if ok
        s = '';
    else
        s = sprintf('first NaN/Inf at k=%d ax=%d', k, ax);
    end
end
