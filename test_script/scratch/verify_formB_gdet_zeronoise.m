% verify_formB_gdet_zeronoise.m -- PURPOSE: Stage 1 + Stage 2 of the proposed
%   "g_det feedforward" fix for defect 3 (long-run w_s drift). Stage 1 writes
%   down, in closed form, the loop transfer H_loop(z) from the commanded height
%   w_d to the realized particle position that motion_control_law_formB_ws
%   ACTUALLY implements, and evaluates the deterministic tracking residual
%   dw_det = (1 - H_loop(q)) w_d at the 1 Hz oscillation. Stage 2 validates
%   that closed form against a ZERO-NOISE run of the production driver, where
%   the deterministic residual is observable directly (no band-limiting).
%   EXPIRES: when the defect-3 fix is implemented or the proposal is retracted.
%   Read-only w.r.t. production files; console output only.
%
% =====================================================================
% STAGE 1 -- CLOSED FORM (all symbols dimensionless: lengths / R)
% =====================================================================
% Notation, matching the controller source:
%   w_d[k]   commanded height sample, = p_d_out(k)/R          (driver pd_k)
%   w[k]     TRUE height at t_k                               (plant state)
%   dw[k]    = w_d[k] - w[k]                                  (tracking error)
%   A[k]     = a_bar_ctrl[k] * f_bar_d[k]                     (COMMANDED step)
%   alpha    = 1 - lambda_c = 0.3 ,  lambda_c = 0.7 ,  d = 2 ,  Ts = 1/1600 s
%
% (1) Control law, lines 747-758 of the controller (traj_term expanded):
%       traj_term[k] = w_d[k+1] - lc*w_d[k] - alpha*w_d[k-2]
%                    = Dw_d[k] + alpha*( w_d[k] - w_d[k-2] )
%       A[k] = traj_term[k] + alpha*( dw_m[k] - A[k-1] - A[k-2] )         (C)
%     with dw_m[k] = the DELAYED measured error = dw[k-2] + n_w[k].
%
% (2) Plant, one control sample (step_dynamics, force held constant):
%       w[k+1] - w[k] = a_bar_true,eff[k] * f_bar_d[k] =: (1 + eps[k]) * A[k]
%     which DEFINES the relative gain defect
%       eps[k] := a_bar_true,eff[k] / a_bar_ctrl[k] - 1 .                 (E)
%     a_bar_true,eff is the gain the step actually realized (it differs from
%     a_bar(w[k]) by the sub-step mobility drift, 0.5*(a'/a)*Dw, which is
%     therefore inside eps and needs no separate bookkeeping).
%
% (3) Substituting (C) into (2), using the exact travel identity
%       dw[k] = dw[k-2] + (w_d[k]-w_d[k-2]) - sum_{i=1,2} (1+eps[k-i])A[k-i],
%     every trajectory term cancels IDENTICALLY and what is left is
%
%       dw[k+1] = lc*dw[k] - eps[k]A[k] - alpha*( eps[k-1]A[k-1]
%                                                 + eps[k-2]A[k-2] )
%                          - alpha*n_w[k] - (thermal terms)               (I)
%
%     (I) is exact -- no truncation, no linearization. It is the same object
%     the filter already carries as F_e row 3 = [0 0 lc -F_dw 0 0 0], with
%     F_dw = f_bar + alpha*(f_bar[k-1]+f_bar[k-2]): the -F_dw*e_a coupling IS
%     the -eps*A sum, written with the gain error e_a as the regressor.
%
% (4) THE STAGE-1 RESULT. Set the noises to zero and eps = 0 (i.e. the loop
%     runs on the believed gain, which is what "derived from known loop
%     quantities" can ever mean): (I) collapses to dw[k+1] = lc*dw[k], a
%     homogeneous recursion with zero forcing for ANY commanded trajectory.
%     Hence
%           H_loop(z) = 1   IDENTICALLY, at every frequency,
%           dw_det = (1 - H_loop(q)) w_d = 0 ,   tau_theory = 0 ms .
%     The delay-compensating feedforward inverts the plant model EXACTLY,
%     d = 2 included; there is no structural lag to feed forward. The
%     measured 0.22 ms is NOT a property of the loop transfer.
%
% (5) What (I) says the measured residual must be instead. At 1 Hz the three
%     eps*A terms are coherent over 3 samples, so with Dw_d = Ts*wdot_d:
%           u[k] ~ -(1 + 2*alpha) * eps(t) * Ts * wdot_d(t)
%     and the chain's DC gain 1/(1-lc) gives
%           dw_det(t) ~ -[(1+2*alpha)/(1-lc)] * Ts * eps(t) * wdot_d(t)
%                     = -5.3333 * Ts * eps(t) * wdot_d(t) .               (L)
%     Consequences, both falsifiable below:
%       (a) only the CYCLE-MEAN part of eps produces a component in phase
%           with wdot_d (the observed "pure lag"); any part of eps that is
%           itself proportional to wdot_d (gain-estimate lag, sub-step drift)
%           lands on wdot_d^2, i.e. 2 Hz + DC, not 1 Hz;
%       (b) the equivalent timing offset is tau_eff = 5.3333*Ts*|eps_DC|,
%           so the reported tau_eff = 0.22 ms requires |eps_DC| = 6.6 %,
%           a mean gain error ~10x the anchored-law representation floor
%           (Pf_a_floor ~ 0.0066) and of the size of a one-prior-width b
%           error (dA/db * 0.1 / a_bar ~ 4.7 % at w_bar = 2).
%
% =====================================================================
% STAGE 2 -- VALIDATION PLAN (what this script measures)
% =====================================================================
%   V1  identity (I) holds to machine precision in the zero-noise run
%       => the algebra above matches the code (two-step audit, step one).
%   V2  the eps = 0 closed form of (4) predicts ZERO residual; compare with
%       the measured 1 Hz amplitude of dw. PASS iff within 15 %.
%   V3  attribution: re-drive the chain (I) with (a) eps_DC only, (b) eps
%       minus its mean, (c) eps = 0, and see which reproduces the 1 Hz line.
%   V4  ablation arms, same scenario:
%         A  ws_inject 0.05 (the specified arm)
%         B  ws_inject 0    (is the residual injection-driven?)
%         C  ws_inject 0.05 + a_ctrl_override 'true' (control law fed the
%            EXACT gain; eps then contains only the sub-step term)
%       If the residual collapses in C, the residual is 100 % gain-error
%       driven and (4) stands: there is nothing structural to feed forward.
%   V5  what reaches the filter: 1 Hz line of nu1 vs of dw (the "~20 %"),
%       and the w_s drive rate mean(dws_y1)/Ts with its kappa = rate/<wdot^2>.

here = fileparts(mfilename('fullpath'));
proj = fullfile(here, '..', '..');
pdirs = {'model', 'model/dual_track', 'model/diag', 'model/thermal_force', ...
         'model/config', 'model/wall_effect', 'model/controller', ...
         'model/trajectory', 'test_script/integration'};
for i = 1:numel(pdirs); addpath(fullfile(proj, pdirs{i})); end
res_dir = fullfile(proj, 'test_results');
if ~exist(res_dir, 'dir'); mkdir(res_dir); end

% ---------------- named constants ----------------
AX        = 3;         % wall-normal axis (z)
SEED      = 7;         % house first seed
INJ       = 0.05;      % [R] TRUE wall offset, plant side (arms A and C)
N_CYC     = 8;         % oscillation cycles
T_SIM     = 10.8;      % [s] 0.5 hold + 1.0 descend + 8 osc + 1.3 hold
F0        = 1;         % [Hz] oscillation fundamental
WIN       = [3.5, 9.5];% [s] analysis window: 6 whole cycles, 2 cycles settle
PF_WS_STD = 0.111;     % calibration-based w_s prior width (C2/C3 working line)
TOL_FRAC  = 0.15;      % Stage-2 pre-registered agreement tolerance
LC        = 0.7;       % closed-loop pole (canonical; asserted against the log)
TS_NOM    = 1 / 1600;  % [s] control period (asserted against the log)
TAU_REPORT = 0.22e-3;  % [s] timing offset reported by the prior investigation

% Arms D/E are the NOISY controls: the reported 0.22 ms / 1.5e-3 R was measured
% on a production (noisy) run, so the zero-noise arms alone cannot tell whether
% that residual is deterministic. D reproduces it under the identical lock-in;
% E repeats D with the control law fed the exact gain (eps -> sub-step only).
arms = struct( ...
    'tag',   {'A_inj',        'B_noinj',       'C_oraclegain',  'D_noisy',      'E_noisy_oracle'}, ...
    'inj',   { INJ,            0,               INJ,             INJ,            INJ}, ...
    'aov',   { [],             [],              'true',          [],             'true'}, ...
    'noise', { false,          false,           false,           true,           true}, ...
    'note',  {'specified arm', 'injection off', 'TRUE gain to the control law', ...
              'NOISY control (production noise)', 'NOISY + TRUE gain to control law'});

fprintf('\n===============================================================\n');
fprintf('STAGE 1 closed form: H_loop(z) == 1 identically  ->  tau_theory = 0.000 ms\n');
fprintf('  (delay-compensating feedforward inverts the model exactly; the\n');
fprintf('   deterministic residual requires a gain defect eps ~= 0, eq (I))\n');
% eq (L) inverted: tau_eff = [(1+2*alpha)/(1-lc)]*Ts*|eps_DC|
eps_dc_implied = TAU_REPORT * (1 - LC) / ((1 + 2 * (1 - LC)) * TS_NOM);
fprintf('  implied by the reported tau_eff = %.2f ms: |eps_DC| = %.1f %%\n', ...
        1e3 * TAU_REPORT, 100 * eps_dc_implied);
fprintf('===============================================================\n');

Rep = cell(1, numel(arms));
for q = 1:numel(arms)
    S = local_load_arm(arms(q), proj, res_dir, SEED, N_CYC, T_SIM, PF_WS_STD, AX);
    assert(abs(S.Ts - TS_NOM) < 1e-15, 'verify_gdet:TsMismatch', ...
           'log Ts = %g s, closed form assumes %g s.', S.Ts, TS_NOM);
    Rep{q} = local_analyze(S, arms(q), WIN, F0, LC, TOL_FRAC);
end

% ---------------- cross-arm verdict ----------------
fprintf('\n=== CROSS-ARM SUMMARY (1 Hz line of the true tracking residual) ===\n');
fprintf('%-15s %11s %11s %10s %10s %11s %11s\n', 'arm', '|dw|_1Hz[R]', ...
        'rms dw[R]', 'tau1Hz[ms]', 'tauLS[ms]', 'eps_DC[%]', 'ws rate[/s]');
for q = 1:numel(arms)
    r = Rep{q};
    fprintf('%-15s %11.3e %11.3e %10.4f %10.4f %11.3f %+11.3e\n', arms(q).tag, ...
            r.amp_dw, r.rms_dw, 1e3 * r.tau_eff, 1e3 * r.tau_ls, ...
            100 * r.eps_dc, r.rate_ws_y1);
end
fprintf('reported by the prior investigation: |dw|_1Hz 1.5e-03 R, tau %.2f ms\n', ...
        1e3 * TAU_REPORT);
% Pre-registered Stage-2 criterion: the Stage-1 structural closed form predicts
% amplitude 0 for arm A; PASS iff the measured line agrees within TOL_FRAC.
amp_pred = 0;
amp_meas = Rep{1}.amp_dw;
ok = abs(amp_meas - amp_pred) <= TOL_FRAC * max(abs(amp_pred), abs(amp_meas));
fprintf('\nStage-2 criterion (structural closed form, eps = 0): predicted %.3e R, measured %.3e R\n', ...
        amp_pred, amp_meas);
if ok
    fprintf('VERDICT: PASS\n');
else
    fprintf('VERDICT: **FAIL** -- structural prediction 0, measured %.3e R (tau_eff %.3f ms).\n', ...
            amp_meas, 1e3 * Rep{1}.tau_eff);
    fprintf('  The residual is not a loop-transfer lag; do NOT build a feedforward on it.\n');
end

% =====================================================================
% STAGE 2b -- is the 1 Hz line DETERMINISTIC at all?
% =====================================================================
% A feedforward built from the PLANNED trajectory can only cancel the part of
% the residual that repeats across seeds. Averaging the COMPLEX 1 Hz phasor
% over seeds separates the two: the deterministic part adds coherently, the
% noise-mediated part averages down as 1/sqrt(n_seeds). Two variants, because
% the w_s Kalman entry K1(7) rides on M_row4 = Dw_d[k-1] + alpha*dw3_hat, whose
% SECOND term is thermal: turning it off (fe_row4_full = false) leaves M purely
% deterministic and tests whether that term is the rectifier behind the drift.
ENS_SEEDS = [7 11 23 42 101 777];
fprintf('\n=== STAGE 2b: seed ensemble, 1 Hz phasor of dw and w_s drive ===\n');
E1 = local_ensemble(ENS_SEEDS, false, proj, res_dir, 'zeronoise', INJ, ...
                    N_CYC, T_SIM, PF_WS_STD, AX, WIN, F0, struct());
E2 = local_ensemble(ENS_SEEDS, true, proj, res_dir, 'noisy', INJ, ...
                    N_CYC, T_SIM, PF_WS_STD, AX, WIN, F0, struct());
E3 = local_ensemble(ENS_SEEDS, true, proj, res_dir, 'noisy_m4det', INJ, ...
                    N_CYC, T_SIM, PF_WS_STD, AX, WIN, F0, ...
                    struct('fe_row4_full', false));
% n = 6 cannot separate a coherent 1 Hz line from phasor scatter (the null
% ratio is already 1/sqrt(6) = 0.41), so the coherence question gets 24 seeds.
E4 = local_ensemble(20000 + (1:24), true, proj, res_dir, 'noisy24', INJ, ...
                    N_CYC, T_SIM, PF_WS_STD, AX, WIN, F0, struct());

% =====================================================================
% STAGE 2c -- what the deterministic residual IS: the parameter-error signature
% =====================================================================
% Zero noise, ALL parameters locked (lock_b/lock_p/lock_ws), true wall at
% 1 + INJ, w_s seeded away from it. The gain defect eps is then fixed by the
% seeded law error alone, so closed form (L) becomes predictive with no free
% quantity: tau_eff = [(1+2*alpha)/(1-lc)]*Ts*|eps_DC|. This is the honest
% validation of (L) -- and it shows the residual is the signal the w_s state is
% identified FROM, which is what a g_det feedforward would cancel.
fprintf('\n=== STAGE 2c: locked-parameter sweep, closed form (L) made predictive ===\n');
fprintf('%9s %11s %11s %12s %12s %9s\n', 'ws_init', 'ws err[R]', 'eps_DC[%]', ...
        'tau meas[ms]', 'tau (L)[ms]', 'ratio');
ws_seeds = [0.95, 1.00, 1.05, 1.10];
Csw = zeros(numel(ws_seeds), 4);
for q = 1:numel(ws_seeds)
    ovq = struct('lock_b', true, 'lock_p', true, 'lock_ws', true, ...
                 'ws_init', ws_seeds(q));
    armq = struct('tag', sprintf('lock_ws%03.0f', 100 * ws_seeds(q)), ...
                  'inj', INJ, 'aov', [], 'noise', false, ...
                  'note', sprintf('locked, ws_init %.2f', ws_seeds(q)));
    Sq = local_load_arm(armq, proj, res_dir, SEED, N_CYC, T_SIM, PF_WS_STD, AX, ovq);
    Rq = local_analyze(Sq, armq, WIN, F0, LC, TOL_FRAC, true);
    Csw(q, :) = [ws_seeds(q) - (1 + INJ), Rq.eps_dc, Rq.tau_eff, Rq.tau_from_epsdc];
    fprintf('%9.2f %11.3f %11.3f %12.4f %12.4f %9.2f\n', ws_seeds(q), Csw(q, 1), ...
            100 * Csw(q, 2), 1e3 * Csw(q, 3), 1e3 * Csw(q, 4), ...
            Csw(q, 4) / max(Csw(q, 3), eps));
end
ok_L = all(abs(Csw(:, 4) - Csw(:, 3)) <= TOL_FRAC * max(Csw(:, 3), eps));
fprintf('closed form (L) vs measured, all four seeds within %.0f %%: %s\n', ...
        100 * TOL_FRAC, local_verdict(ok_L));

fprintf('\nFEEDFORWARD CEILING: a trajectory-derived g_det can only cancel the\n');
fprintf('  seed-independent part, %.3e R, = %.2f %% of the production 1 Hz line\n', ...
        E1.A_coh, 100 * E1.A_coh / max(mean(abs(E4.z_dw)), eps));
fprintf('  (%.2f %% of the scatter-corrected coherent part %.3e R).\n', ...
        100 * E1.A_coh / max(E4.A_coh, eps), E4.A_coh);

save(fullfile(res_dir, 'verify_formB_gdet_zeronoise_report.mat'), ...
     'Rep', 'arms', 'E1', 'E2', 'E3', 'E4');
fprintf('\nsaved: %s\n', fullfile(res_dir, 'verify_formB_gdet_zeronoise_report.mat'));


%% =================== local helpers ===================

function S = local_load_arm(arm, ~, res_dir, seed, n_cyc, T_sim, pf_ws, ax, ov_extra)
%LOCAL_LOAD_ARM  Run (or reload) one zero-noise arm and extract the signals.
%   Zero noise = thermal_enable false AND meas_noise_enable false, so the
%   tracking residual and the y1 innovation are purely deterministic.
    f = fullfile(res_dir, sprintf('verify_formB_gdet_zeronoise_%s.mat', arm.tag));
    if exist(f, 'file')
        load(f, 'S');
        fprintf('loaded cached arm %s: %s\n', arm.tag, f);
        return;
    end
    ov = struct('lock_b', false, 'lock_p', true, 'lock_ws', false, ...
                'Pf_ws_std', pf_ws);
    if nargin >= 9 && ~isempty(ov_extra)
        fn = fieldnames(ov_extra);
        for j = 1:numel(fn); ov.(fn{j}) = ov_extra.(fn{j}); end
    end
    o = struct('seeds', seed, 'ws_inject', arm.inj, 'verbose', false, ...
               'ctrl_const_override', ov, ...
               'a_ctrl_override', arm.aov, ...
               'config_override', struct('n_cycles', n_cyc, 'T_sim', T_sim, ...
                                         'thermal_enable', arm.noise, ...
                                         'meas_noise_enable', arm.noise));  %#ok<NASGU> read by the evalc below
    t0 = tic;
    [~, out] = evalc('run_formB_ws(o)');
    r = out.runs{1};
    fprintf('arm %-14s ran in %.1f s (%s)\n', arm.tag, toc(t0), arm.note);

    S = struct();
    S.t      = r.tout(:);
    S.Ts     = S.t(2) - S.t(1);
    S.R      = r.R;
    S.wd     = r.p_d_out(:, ax) / r.R;            % w_d[k]           [-]
    S.p_true = r.p_true_out(:, ax) / r.R;         % w[k+1]  (END of step k)
    S.p0     = r.meta.params_value.common.p0(ax) / r.R;
    S.fbar   = r.f_bar_out(:, ax);                % f_bar_d[k]       [-]
    S.abar   = r.a_bar_hat_out(:, ax);            % POSTERIOR a_bar[k]
    S.dh_m   = r.dh_m_out(:, ax) / r.R;           % dw_m[k]          [-]
    S.nu1    = r.innov_y1_out(:, ax);             % y1 innovation    [-]
    S.dws1   = r.dws_y1_out(:, ax);               % K1(7)*nu1        [-]
    S.dws2   = r.dws_y2_out(:, ax);
    S.ws     = r.ws_hat_out(:, ax);
    S.b_hat  = r.b_hat_out(:, ax);
    S.a_true = r.a_true_out(:, ax);               % [um/pN] at w[k]
    S.a_nom  = r.a_nom;
    S.gate   = r.gate_out(:, ax);
    S.lc     = r.ctrl_const.lambda_c;
    save(f, 'S');
    fprintf('saved arm %s: %s\n', arm.tag, f);
end


function R = local_analyze(S, arm, win, f0, lc_expect, tol, quiet)
%LOCAL_ANALYZE  Stage-2 audit of one arm: identity (I), attribution, lock-in.
    if nargin < 7; quiet = false; end
    if quiet
        prf = @(varargin) [];                       % sweep rows print their own
    else
        prf = @(varargin) fprintf(varargin{:});
    end
    Ts = S.Ts;
    assert(abs(S.lc - lc_expect) < 1e-12, 'verify_gdet:lambdaMismatch', ...
           'controller lambda_c = %g, closed form assumes %g.', S.lc, lc_expect);
    al = 1 - S.lc;
    N  = numel(S.t);

    % --- reconstruct the true height chain -------------------------------
    % p_true_out(k) is the position at the END of step k, i.e. w[k+1].
    w      = [S.p0; S.p_true];              % w[1..N+1]
    wd     = S.wd;                          % w_d[1..N]
    dw     = wd - w(1:N);                   % dw[k] = w_d[k] - w[k]
    dw_rea = diff(w);                       % w[k+1]-w[k], length N

    % --- commanded step A[k], two independent reconstructions ------------
    % (i) from the logged gain and force  A = a_bar_ctrl * f_bar
    %     a_bar_ctrl[k] is the PRE-update state, i.e. the posterior of k-1.
    A_log = [0; S.abar(1:end-1)] .* S.fbar;
    % (ii) from the control law itself (gain-free; also valid for the arm
    %      whose control law runs on an overridden gain):
    %          A[k] = w_d[k+1] - lc*w_d[k] - al*w_d[k-2]
    %                 + al*( dw_m[k] - A[k-1] - A[k-2] )
    A_rec = zeros(N, 1);
    for k = 3:N-1
        traj = wd(k+1) - S.lc * wd(k) - al * wd(k-2);
        A_rec(k) = traj + al * (S.dh_m(k) - A_rec(k-1) - A_rec(k-2));
    end

    % A_rec is the commanded step implied by the control-law CODE and is
    % gain-free, so it stays valid when a_ctrl_override feeds the control law a
    % gain the log does not carry; A_log is kept as the cross-check.
    A = A_rec;

    % --- relative gain defect eps[k] = dw_real/A - 1 ----------------------
    good = abs(A) > 1e-14;
    eps_k = zeros(N, 1);
    eps_k(good) = dw_rea(good) ./ A(good) - 1;

    % --- window (integer cycles) ----------------------------------------
    idx = find(S.t >= win(1) & S.t < win(2));
    idx = idx(idx > 3 & idx < N - 2);
    ncyc = numel(idx) * Ts * f0;
    assert(abs(ncyc - round(ncyc)) < 1e-6, 'verify_gdet:nonIntegerCycles', ...
           'window holds %.4f cycles -- lock-in would leak.', ncyc);

    % --- V1: exact identity (I) ------------------------------------------
    % dw[k+1] - lc*dw[k] + eps[k]A[k] + al*(eps[k-1]A[k-1] + eps[k-2]A[k-2]) = 0
    % Exact only with the noises off; in the noisy arms the omitted
    % -al*n_w[k] term is expected to dominate, so V1 is asserted there only as
    % a report, not a criterion.
    D  = dw_rea - A;                           % = eps.*A, the step defect
    res_I = dw(min(idx+1, N)) - S.lc * dw(idx) ...
            + D(idx) + al * (D(idx-1) + D(idx-2));
    scale = max(abs(dw(idx)));
    prf('\n=== ARM %s (%s): %d samples = %.0f cycles ===\n', ...
            arm.tag, arm.note, numel(idx), ncyc);
    if arm.noise
        prf('V1 identity (I) residual: max %.3e R (rms %.3e) -- NOISY arm, the\n', ...
                max(abs(res_I)), sqrt(mean(res_I.^2)));
        prf('   measurement-noise term -al*n_w is omitted by construction (not a criterion)\n');
    else
        prf('V1 identity (I) residual: max %.3e R (rms %.3e) vs |dw| scale %.3e -> %s\n', ...
                max(abs(res_I)), sqrt(mean(res_I.^2)), scale, ...
                local_verdict(max(abs(res_I)) < 1e-9 * max(scale, eps)));
    end
    prf('   control-law cross-check max|A_rec - A_log| = %.3e R (|A| rms %.3e)\n', ...
            max(abs(A_rec(idx) - A_log(idx))), sqrt(mean(A(idx).^2)));

    % --- commanded velocity + lock-in at f0 ------------------------------
    wdot = [diff(wd); 0] / Ts;                  % wdot_d[k] = Dw_d[k]/Ts
    [amp_dw, ph_dw] = local_lockin(dw(idx), S.t(idx), f0);
    [amp_v,  ph_v ] = local_lockin(wdot(idx), S.t(idx), f0);
    [amp_nu, ph_nu] = local_lockin(S.nu1(idx), S.t(idx), f0);
    tau_eff = amp_dw / max(amp_v, eps);         % [s] equivalent timing offset
    dphi    = local_wrap(ph_dw - ph_v);

    % --- eps decomposition ------------------------------------------------
    eps_w  = eps_k(idx);
    eps_dc = mean(eps_w);
    [amp_e1, ~] = local_lockin(eps_w - eps_dc, S.t(idx), f0);
    [amp_e2, ~    ] = local_lockin(eps_w - eps_dc, S.t(idx), 2 * f0);
    tau_from_epsdc = (1 + 2 * al) / (1 - S.lc) * Ts * abs(eps_dc);

    % Broadband least-squares lag: the coefficient of dw on wdot_d, which is
    % what "effective timing offset" reads if fitted without band-limiting.
    tau_ls = (dw(idx)' * wdot(idx)) / max(wdot(idx)' * wdot(idx), eps);
    prf('V2 measured 1 Hz line of dw: |dw| = %.4e R, phase vs wdot %+.1f deg\n', ...
            amp_dw, dphi);
    prf('   rms(dw) over window %.4e R; broadband LS lag <dw*wdot>/<wdot^2> = %+.4f ms\n', ...
            sqrt(mean(dw(idx).^2)), 1e3 * tau_ls);
    prf('   |wdot|_1Hz = %.4f R/s -> tau_eff = %.4f ms   (structural prediction 0.0000 ms)\n', ...
            amp_v, 1e3 * tau_eff);
    prf('   eps: DC %+.4f (%.2f %%)  |1Hz| %.4e  |2Hz| %.4e  rms %.4e\n', ...
            eps_dc, 100 * eps_dc, amp_e1, amp_e2, sqrt(mean(eps_w.^2)));
    prf('   closed form (L) from eps_DC alone: tau = %.4f ms (%.0f %% of measured)\n', ...
            1e3 * tau_from_epsdc, 100 * tau_from_epsdc / max(tau_eff, eps));

    % --- V3: re-drive the chain with pieces of eps -------------------------
    lbl = {'full eps', 'eps_DC only', 'eps - mean', 'eps = 0 (structural)'};
    epsv = {eps_k, [], [], zeros(N, 1)};
    e_dc_full = zeros(N, 1); e_dc_full(idx) = eps_dc;
    epsv{2} = e_dc_full;
    e_ac = eps_k; e_ac(idx) = eps_k(idx) - eps_dc;
    epsv{3} = e_ac;
    prf('V3 attribution (chain re-driven with each eps piece):\n');
    for j = 1:numel(epsv)
        dwj = local_redrive(epsv{j}, A, S.lc, dw, idx);
        aj  = local_lockin(dwj, S.t(idx), f0);
        prf('   %-22s |dw|_1Hz = %.4e R  (%6.1f %% of measured)\n', ...
                lbl{j}, aj, 100 * aj / max(amp_dw, eps));
    end

    % --- V5: what reaches the filter --------------------------------------
    rate1 = mean(S.dws1(idx)) / Ts;
    rate2 = mean(S.dws2(idx)) / Ts;
    kappa = -rate1 / max(mean(wdot(idx).^2), eps);
    prf('V5 nu1 1 Hz line %.4e R = %.1f %% of dw line; phase vs wdot %+.1f deg\n', ...
            amp_nu, 100 * amp_nu / max(amp_dw, eps), local_wrap(ph_nu - ph_v));
    prf('   w_s drive: y1 %+.4e /s, y2 %+.4e /s; kappa = -rate/<wdot^2> = %+.4e s\n', ...
            rate1, rate2, kappa);
    prf('   ws_hat %.4f -> %.4f over the window; b_hat %.4f -> %.4f; y2 gated %.0f %% of steps\n', ...
            S.ws(idx(1)), S.ws(idx(end)), S.b_hat(idx(1)), S.b_hat(idx(end)), ...
            100 * mean(S.gate(idx)));

    R = struct('tag', arm.tag, 'amp_dw', amp_dw, 'tau_eff', tau_eff, ...
               'dphi_deg', dphi, 'eps_dc', eps_dc, 'amp_eps1', amp_e1, ...
               'tau_from_epsdc', tau_from_epsdc, 'amp_nu', amp_nu, ...
               'rate_ws_y1', rate1, 'kappa', kappa, 'tau_ls', tau_ls, ...
               'rms_dw', sqrt(mean(dw(idx).^2)), ...
               'identity_max', max(abs(res_I)), 'tol', tol);
end


function dwj = local_redrive(eps_use, A, lc, dw_true, idx)
%LOCAL_REDRIVE  Integrate identity (I) with a chosen eps, from the true dw at
%   the window start, and return the window slice.
    al = 1 - lc;
    D  = eps_use .* A;
    N  = numel(A);
    x  = zeros(N, 1);
    x(idx(1)) = dw_true(idx(1));
    for k = idx(1):min(idx(end), N - 1)
        x(k+1) = lc * x(k) - D(k) - al * (D(max(k-1, 1)) + D(max(k-2, 1)));
    end
    dwj = x(idx);
end


function E = local_ensemble(seeds, noise_on, ~, res_dir, tag, inj, ...
                            n_cyc, T_sim, pf_ws, ax, win, f0, extra_ov)
%LOCAL_ENSEMBLE  Per-seed 1 Hz phasor of the tracking residual and w_s drive.
%   Coherent (deterministic) content = |mean of the complex phasors|; incoherent
%   content = mean of the magnitudes. Their ratio is the fraction of the 1 Hz
%   line a trajectory-derived feedforward could ever cancel.
    f = fullfile(res_dir, sprintf('verify_formB_gdet_ens_%s.mat', tag));
    if exist(f, 'file')
        load(f, 'E');
        fprintf('loaded cached ensemble %s\n', tag);
    else
        ns = numel(seeds);
        E = struct('tag', tag, 'seeds', seeds, 'z_dw', zeros(ns, 1), ...
                   'z_nu', zeros(ns, 1), 'rate_ws', zeros(ns, 1), ...
                   'ws_end', zeros(ns, 1), 'rms_dw', zeros(ns, 1));
        ov = struct('lock_b', false, 'lock_p', true, 'lock_ws', false, ...
                    'Pf_ws_std', pf_ws);
        fn = fieldnames(extra_ov);
        for j = 1:numel(fn); ov.(fn{j}) = extra_ov.(fn{j}); end
        for q = 1:ns
            o = struct('seeds', seeds(q), 'ws_inject', inj, 'verbose', false, ...
                       'ctrl_const_override', ov, ...
                       'config_override', struct('n_cycles', n_cyc, ...
                            'T_sim', T_sim, 'thermal_enable', noise_on, ...
                            'meas_noise_enable', noise_on));  %#ok<NASGU> read by the evalc below
            [~, out] = evalc('run_formB_ws(o)');
            r  = out.runs{1};
            t  = r.tout(:);
            Ts = t(2) - t(1);
            w  = [r.meta.params_value.common.p0(ax); r.p_true_out(1:end-1, ax)] / r.R;
            dw = r.p_d_out(:, ax) / r.R - w;
            idx = find(t >= win(1) & t < win(2));
            E.z_dw(q)   = 2 * mean(dw(idx) .* exp(-1i * 2 * pi * f0 * t(idx)));
            E.z_nu(q)   = 2 * mean(r.innov_y1_out(idx, ax) .* ...
                                   exp(-1i * 2 * pi * f0 * t(idx)));
            E.rate_ws(q) = mean(r.dws_y1_out(idx, ax) + r.dws_y2_out(idx, ax)) / Ts;
            E.ws_end(q)  = r.ws_hat_out(end, ax);
            E.rms_dw(q)  = sqrt(mean(dw(idx).^2));
        end
        save(f, 'E');
    end
    % Unbiased coherent amplitude: |mean z|^2 over-states it by the phasor
    % scatter, E|mean z|^2 = |A_coh|^2 + S^2/n, so subtract the sample scatter.
    % Under the null (no repeatable component) |mean z|/mean|z| -> 1/sqrt(n).
    n     = numel(E.seeds);
    coh   = abs(mean(E.z_dw));            % raw coherent estimate
    incoh = mean(abs(E.z_dw));            % per-seed magnitude
    S2    = sum(abs(E.z_dw - mean(E.z_dw)).^2) / max(n - 1, 1);
    A_coh = sqrt(max(coh^2 - S2 / n, 0));
    se    = sqrt(S2 / n);
    T2    = coh^2 / max(S2 / n, eps);     % Hotelling T^2 on (Re, Im)
    fprintf('\n%-12s  n=%d  1 Hz dw: |mean z| %.3e R, mean|z| %.3e R (ratio %.2f, null 1/sqrt(n) = %.2f)\n', ...
            tag, n, coh, incoh, coh / max(incoh, eps), 1 / sqrt(n));
    fprintf('%-12s  coherent amplitude (scatter-corrected) %.3e R +- %.3e; T^2 %.2f (F(2,n-2) 5%% ~ %.1f)\n', ...
            '', A_coh, se, T2, 2 * (n - 1) / max(n - 2, 1) * local_f95(n));
    E.A_coh = A_coh; E.se_coh = se; E.T2 = T2;
    fprintf('%-12s  1 Hz nu1: coherent %.3e vs mean|z| %.3e R;  rms dw %.3e R\n', ...
            '', abs(mean(E.z_nu)), mean(abs(E.z_nu)), mean(E.rms_dw));
    fprintf('%-12s  w_s drive rate: mean %+.4e /s, sd %.4e, sem %.4e -> t = %+.2f\n', ...
            '', mean(E.rate_ws), std(E.rate_ws), std(E.rate_ws) / sqrt(numel(E.seeds)), ...
            mean(E.rate_ws) / max(std(E.rate_ws) / sqrt(numel(E.seeds)), eps));
    fprintf('%-12s  ws_hat at end of run: %s\n', '', ...
            sprintf('%.4f ', E.ws_end));
end


function f = local_f95(n)
%LOCAL_F95  95th percentile of F(2, n-2), closed form (no Statistics Toolbox).
%   For F(2,d) the survival function is (1 + 2x/d)^(-d/2), so the 95 % point is
%   x = (d/2)*(0.05^(-2/d) - 1).
    d = max(n - 2, 1);
    f = (d / 2) * (0.05^(-2 / d) - 1);
end


function [amp, ph] = local_lockin(x, t, f0)
%LOCAL_LOCKIN  Amplitude and phase of the f0 line (integer-cycle window).
    z   = 2 * mean(x(:) .* exp(-1i * 2 * pi * f0 * t(:)));
    amp = abs(z);
    ph  = rad2deg(angle(z));
end


function d = local_wrap(d)
%LOCAL_WRAP  Wrap a phase difference to (-180, 180] degrees.
    d = mod(d + 180, 360) - 180;
end


function s = local_verdict(ok)
%LOCAL_VERDICT  PASS/FAIL label.
    if ok; s = 'PASS'; else; s = 'FAIL'; end
end
