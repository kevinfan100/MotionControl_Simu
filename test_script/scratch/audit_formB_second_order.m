% audit_formB_second_order.m -- PURPOSE: enumerate and EVALUATE every
%   second-order (curvature) mean correction the first-order EKF
%   motion_control_law_formB_ws omits, and convert each into a w_bar_s push,
%   to test them against the measured ghost of the S11 calibration stall
%   (T1 total ws push +5.4e-3/run at Pf_ws_std 0.044, /3.65 at 0.022;
%    T2 descent mean innov_y2 phantom -2.4e-4; T3 sign fixed UP on both sides).
%
%   The filter rows that are NONLINEAR in the free parameters (b, w_s) are
%     process row 4:  f4 = x4 + s(g,b,p)*M ,  s = da_bar/dw_bar ,
%                     g = w_bar_d - x7 , M = Dw_d[k-1] + alpha*(x3+x8+x9)
%     measurement y2: h2 = H2s*( x4 - e*s(g,b,p)*Grad )
%   so every omitted term is 0.5*d2f/dxi dxj*P_ij with (i,j) drawn from
%   {b, w_s} x {b, w_s, dw3, m1, m2} (process) and {b, w_s}^2 (measurement).
%   NOTE the gain LEVEL is a state here, so the level curvature 0.5*a''*P_ws
%   does NOT appear in h2; it appears only as the ACCUMULATED process term
%   0.5*P_ws*sum a'''(g)*M ~ 0.5*P_ws*[a''(g_end) - a''(g_0)].
%
%   Ledger: a row-4 bias beta (EKF mean minus true conditional mean of a_bar)
%   propagates  beta <- F44*beta - c4 ,  then the y2 update drains it and
%   pushes w_s:   E[innov2] = -H2s*beta + cy2 ,
%                 dws = K2(7)*E[innov2] ,  beta <- beta + K2(4)*E[innov2].
%   K2 is reconstructed EXACTLY from the logged posterior: the Joseph identity
%   gives P_post*H2' = K2*R2 for a scalar update, so K2 = P_post*H2'/R2.
%   The y1 path is not modelled; its measured share is reported from dws_y1_out.
%
%   Measured phantom for comparison: paired stall/mirror cancellation
%   (protocol of check_formB_curvature_phantom.m) -- first-order innovation
%   content flips sign between the arms, a curvature phantom is common.
%
%   Outputs: console tables + test_results/audit_formB_second_order.mat
%
%   VERDICT (2026-08-02 run, 36 runs): the terms are real, the dominant one is
%   the process {ws,ws} term (descent sum -2.22e-4 in a_bar units, matching the
%   closed form 0.5*P_ws*[a''(g_trough) - a''(g_0)]), its sign IS fixed up, and
%   it scales with P_ws as required -- but its w_s push is +7.1e-5/run
%   (no-drain bound +1.05e-4) against a nominal ghost of +5.5e-3, i.e. ~1.3%.
%   T1/T2 are themselves consistent with zero at N = 6 (t = 0.43 / -0.16; one
%   seed carries the whole mean, leave-one-out flips its sign). The apparent
%   match of the old "H1 prediction" -0.5*a''*P_ws = -2.27e-4 to the measured
%   -2.4e-4 is a UNIT SLIP: innov_y2 lives in whitened units (a_cov*a_bar), so
%   the comparable curvature figure is a_cov times smaller, -1.1e-5.
%   EXPIRES: when the second-order correction is accepted (or the phantom is
%   re-attributed) and recorded in the formB derivation.

clear; close all;
here = fileparts(mfilename('fullpath'));
proj = fullfile(here, '..', '..');
addpath(genpath(fullfile(proj, 'model')));
addpath(fullfile(proj, 'test_script', 'integration'));

AX          = 3;                    % z / wall-normal
SEED_BASE   = 60000;                % same band as check_formB_curvature_phantom
N_SEED      = 6;
PRIORS      = [0.044, 0.022];       % Pf_ws_std legs (scaling test)
DELTA       = 0.05;                 % injected TRUE wall offset [R]
HALF_OFF    = 0.025;                % estimate starts half an offset from truth
PHASE_EDGES = [0.5 1.5; 1.6 3.5; 3.8 Inf];   % descent | osc | final hold
PHASE_NAME  = {'desc', 'osc', 'hold'};
B_SEED = 9/8; P_SEED = 1;           % derived anchors (do not re-derive)
TERM_NAME = {'P.wsws', 'P.bb', 'P.b_ws', 'P.ws_x3m', 'P.b_x3m', ...
             'M.wsws', 'M.bb', 'M.b_ws'};
n_term = numel(TERM_NAME);

%% ---------------------------------------------------------------------
%  [0] Canonical config (replica of run_formB_ws.local_canonical_config) and
%      the envelope priors (replica of local_envelope_priors) -- the cfg form
%      of the driver skips the arm-form derivation, so they are passed in.
%      par_law = false: verified bit-identical on the z axis (it moves x/y only).
%% ---------------------------------------------------------------------
pc  = physical_constants();
cfg = user_config();
cfg.trajectory_type = 'osc';
cfg.h_init = 50; cfg.h_bottom = 4.5; cfg.amplitude = 2.5;
cfg.frequency = 1; cfg.n_cycles = 2;
cfg.t_hold = 0.5; cfg.t_descend_override = 1.0; cfg.T_sim = 4.8;
cfg.h_min = 1.1 * pc.R;
cfg.ctrl_enable = true; cfg.thermal_enable = true; cfg.meas_noise_enable = true;
cfg.lambda_c = 0.7; cfg.a_pd = 0.05; cfg.a_cov = 0.05;
cfg.meas_noise_std = [0.00062; 0.00057; 0.00331];
cfg.h_bar_safe = 1.5;

env_lo = cfg.h_bottom / pc.R - 0.1;      % ENV_LO_MARGIN
env_hi = cfg.h_init   / pc.R + 1.0;      % ENV_HI_MARGIN
[sPbb, sPpp, floor_a] = local_envelope_priors(env_lo, env_hi, B_SEED, P_SEED);
fprintf('envelope priors on [%.3f %.3f]: sqrtPbb %.6f  sqrtPpp %.6f  floor_a %.6f\n', ...
        env_lo, env_hi, sPbb, sPpp, floor_a);

base_ov = struct('lock_b', false, 'lock_p', true, 'lock_ws', false, ...
                 'Pf_b_std', sPbb, 'Pf_p_std', sPpp, 'Pf_a_floor', floor_a, ...
                 'par_law', false);

%% ---------------------------------------------------------------------
%  [1] Law derivatives: analytic vs finite difference (sign discipline)
%% ---------------------------------------------------------------------
local_verify_derivatives();

%% ---------------------------------------------------------------------
%  [2] Runs: 2 priors x 2 arms x N_SEED, full covariance logging
%% ---------------------------------------------------------------------
%  The third arm starts the estimate EXACTLY at the true wall: any sign-fixed
%  innovation there is structural, with no first-order w_s error to cancel.
arms = struct('name', {'stall', 'mirror', 'center'}, ...
              'delta', {-DELTA, +DELTA, -DELTA}, ...
              'init',  {1 - DELTA + HALF_OFF, 1 + DELTA - HALF_OFF, 1 - DELTA});
n_arm = numel(arms);

V = struct();                      % per-run scalars: [prior, arm, seed]
z3 = zeros(numel(PRIORS), n_arm, N_SEED);
V.innov_desc = z3; V.innov_osc = z3; V.innov_hold = z3;
V.push_meas = z3; V.push_pred = z3; V.push_nd = z3;
V.aerr_desc = z3; V.c4ws_desc = z3; V.ws_err_end = z3;

res = struct();
for ip = 1:numel(PRIORS)
  for ia = 1:n_arm
    acc = []; n_acc = 0;
    for c = 1:N_SEED
      seed = SEED_BASE + 100*ip + c;
      ov = base_ov;
      ov.Pf_ws_std = PRIORS(ip);
      ov.ws_init   = arms(ia).init;
      topts = struct('seed', seed, 'ctrl_const_override', ov, ...
                     'log_P_full', true, 'ws_inject', arms(ia).delta, ...
                     'verbose', false);
      s = run_formB_ws(cfg, topts);
      a = local_analyze_run(s, AX, PHASE_EDGES, cfg, n_term);
      if isempty(acc); acc = a; else; acc = local_accumulate(acc, a); end
      n_acc = n_acc + 1;
      V.innov_desc(ip, ia, c) = a.innov_y2_phase(1);
      V.innov_osc(ip, ia, c)  = a.innov_y2_phase(2);
      V.innov_hold(ip, ia, c) = a.innov_y2_phase(3);
      V.push_meas(ip, ia, c)  = sum(a.dws_traj_phase(1:3));
      V.push_pred(ip, ia, c)  = sum(a.push_phase(:));
      V.push_nd(ip, ia, c)    = a.push_nd_phase(4);
      V.aerr_desc(ip, ia, c)  = a.aerr_phase(1);
      V.c4ws_desc(ip, ia, c)  = a.corr_phase(1, 1);
      V.ws_err_end(ip, ia, c) = s.ws_hat_out(end, AX) - (1 + arms(ia).delta);
      fprintf('  prior %.3f %-6s seed %d: ws_end %+.5f (true %+.3f) | c4 tot %+.3e | pred push %+.3e | meas push %+.3e\n', ...
              PRIORS(ip), arms(ia).name, seed, s.ws_hat_out(end, AX), ...
              1 + arms(ia).delta, a.c4_tot, sum(a.push_phase(:)), sum(a.dws_traj_phase));
    end
    res.(sprintf('p%d_%s', ip, arms(ia).name)) = local_finalize(acc, n_acc);
  end
end
res.V = V;

%% ---------------------------------------------------------------------
%  [3] Reports
%% ---------------------------------------------------------------------
fprintf('\n================ SELF-CHECKS ================\n');
for ip = 1:numel(PRIORS)
  A = res.(sprintf('p%d_stall', ip));
  fprintf('prior %.3f: max|a''_recon - a''_log| %.3e | max|K2(4)_rec - K_a_y2| %.3e | max|K2(7)*innov - dws_y2| %.3e\n', ...
          PRIORS(ip), A.chk_aprime, A.chk_K24, A.chk_dws);
  fprintf('           max|a''*F_dw| %.4f (F44 = 1 + a''*F_dw) | y2-gated steps %d\n', ...
          A.chk_F44, round(A.n_gated));
end

fprintf('\n================ TERM TABLE: ws push per run [-] ================\n');
fprintf('(phantom-isolated = (stall + mirror)/2, the sign-fixed component)\n');
for ip = 1:numel(PRIORS)
  S = res.(sprintf('p%d_stall', ip)); M = res.(sprintf('p%d_mirror', ip));
  fprintf('\n--- Pf_ws_std = %.3f ---\n', PRIORS(ip));
  fprintf('%-10s %11s %11s %11s %11s\n', 'term', 'desc', 'osc', 'hold', 'TOTAL');
  tot = zeros(1, 4);
  for it = 1:n_term
    row = (S.push_phase(it, :) + M.push_phase(it, :)) / 2;
    row(4) = sum(row(1:3));
    tot = tot + row;
    fprintf('%-10s %+11.3e %+11.3e %+11.3e %+11.3e\n', TERM_NAME{it}, row);
  end
  fprintf('%-10s %+11.3e %+11.3e %+11.3e %+11.3e\n', 'SUM(2nd)', tot);
  meas2 = (S.dws_y2_phase + M.dws_y2_phase) / 2;  meas2(4) = sum(meas2(1:3));
  meas1 = (S.dws_y1_phase + M.dws_y1_phase) / 2;  meas1(4) = sum(meas1(1:3));
  trm   = (S.dws_traj_phase + M.dws_traj_phase)/2; trm(4)  = sum(trm(1:3));
  fprintf('%-10s %+11.3e %+11.3e %+11.3e %+11.3e\n', 'meas y2', meas2);
  fprintf('%-10s %+11.3e %+11.3e %+11.3e %+11.3e\n', 'meas y1', meas1);
  fprintf('%-10s %+11.3e %+11.3e %+11.3e %+11.3e\n', 'meas tot', trm);
end

fprintf('\n================ RAW CORRECTION SUMS (gain units, per run) ================\n');
for ip = 1:numel(PRIORS)
  S = res.(sprintf('p%d_stall', ip)); M = res.(sprintf('p%d_mirror', ip));
  fprintf('\n--- Pf_ws_std = %.3f ---\n', PRIORS(ip));
  fprintf('%-10s %11s %11s %11s %11s\n', 'term', 'desc', 'osc', 'hold', 'TOTAL');
  for it = 1:n_term
    row = (S.corr_phase(it, :) + M.corr_phase(it, :)) / 2;
    row(4) = sum(row(1:3));
    fprintf('%-10s %+11.3e %+11.3e %+11.3e %+11.3e\n', TERM_NAME{it}, row);
  end
end

fprintf('\n================ INNOVATION PHANTOM (y2 units) ================\n');
fprintf('%-8s %-6s %13s %13s %10s\n', 'prior', 'phase', 'pred_mean', 'meas_mean', 'pred/meas');
for ip = 1:numel(PRIORS)
  S = res.(sprintf('p%d_stall', ip)); M = res.(sprintf('p%d_mirror', ip));
  for iw = 1:3
    pr = (S.einnov_phase(iw) + M.einnov_phase(iw)) / 2;
    me = (S.innov_y2_phase(iw) + M.innov_y2_phase(iw)) / 2;
    fprintf('%-8.3f %-6s %+13.3e %+13.3e %10.2f\n', PRIORS(ip), PHASE_NAME{iw}, pr, me, pr/me);
  end
end

fprintf('\n================ WHAT THE MEASURED PHANTOM IS ================\n');
fprintf('innov_y2 = a_cov*(a_bar_true - a_bar_hat), so -mean(innov)/a_cov must equal the\n');
fprintf('mean a_bar error. Phantom = (stall+mirror)/2; per-arm values show if it is sign-fixed.\n');
fprintf('%-8s %-6s %12s %12s %12s %12s %12s\n', 'prior', 'phase', ...
        'aerr_direct', 'aerr_innov', 'aerr_stall', 'aerr_mirror', 'K2_ws');
for ip = 1:numel(PRIORS)
  S = res.(sprintf('p%d_stall', ip)); M = res.(sprintf('p%d_mirror', ip));
  for iw = 1:3
    fprintf('%-8.3f %-6s %+12.3e %+12.3e %+12.3e %+12.3e %+12.3e\n', PRIORS(ip), PHASE_NAME{iw}, ...
            (S.aerr_phase(iw) + M.aerr_phase(iw))/2, ...
            (S.aerr_from_innov(iw) + M.aerr_from_innov(iw))/2, ...
            S.aerr_phase(iw), M.aerr_phase(iw), ...
            (S.K27_phase(iw) + M.K27_phase(iw))/2);
  end
end

fprintf('\n================ LEDGER-FREE BOUND (no drain: |beta| maximal) ================\n');
fprintf('%-8s %13s %13s %13s %13s\n', 'prior', 'innov_nd_desc', 'innov_meas', 'push_nd_tot', 'push_meas');
for ip = 1:numel(PRIORS)
  S = res.(sprintf('p%d_stall', ip)); M = res.(sprintf('p%d_mirror', ip));
  fprintf('%-8.3f %+13.3e %+13.3e %+13.3e %+13.3e\n', PRIORS(ip), ...
          (S.einnov_nd_phase(1) + M.einnov_nd_phase(1))/2, ...
          (S.innov_y2_phase(1) + M.innov_y2_phase(1))/2, ...
          (S.push_nd_phase(4) + M.push_nd_phase(4))/2, ...
          sum((S.dws_traj_phase(1:3) + M.dws_traj_phase(1:3))/2));
end

fprintf('\n================ P-SCALING (prior 0.044 / prior 0.022) ================\n');
S1 = res.p1_stall; M1 = res.p1_mirror; S2 = res.p2_stall; M2 = res.p2_mirror;
for it = 1:n_term
  a1 = sum((S1.push_phase(it, :) + M1.push_phase(it, :)) / 2);
  a2 = sum((S2.push_phase(it, :) + M2.push_phase(it, :)) / 2);
  fprintf('%-10s %+11.3e / %+11.3e = %7.2f\n', TERM_NAME{it}, a1, a2, a1 / a2);
end
p1 = sum(sum((S1.push_phase + M1.push_phase) / 2));
p2 = sum(sum((S2.push_phase + M2.push_phase) / 2));
r1 = sum((S1.dws_traj_phase + M1.dws_traj_phase) / 2);
r2 = sum((S2.dws_traj_phase + M2.dws_traj_phase) / 2);
fprintf('%-10s %+11.3e / %+11.3e = %7.2f   (predicted 2nd-order total)\n', 'SUM(2nd)', p1, p2, p1/p2);
fprintf('%-10s %+11.3e / %+11.3e = %7.2f   (measured total ws push)\n', 'MEASURED', r1, r2, r1/r2);
i1 = (S1.innov_y2_phase(1) + M1.innov_y2_phase(1))/2;
i2 = (S2.innov_y2_phase(1) + M2.innov_y2_phase(1))/2;
k1 = (S1.K27_phase(1) + M1.K27_phase(1))/2;
k2 = (S2.K27_phase(1) + M2.K27_phase(1))/2;
fprintf('%-10s %+11.3e / %+11.3e = %7.2f   (measured descent innov phantom)\n', 'innov', i1, i2, i1/i2);
fprintf('%-10s %+11.3e / %+11.3e = %7.2f   (descent-mean K2_ws: push = K2_ws x innov)\n', 'K2_ws', k1, k2, k1/k2);
fprintf('a curvature phantom is itself ~P_ws, and K2_ws ~P_ws too, so a curvature-driven\n');
fprintf('push must scale ~16x; the measured push scales %.2fx.\n', r1/r2);

fprintf('\n================ T3: sign on both sides of truth ================\n');
for ip = 1:numel(PRIORS)
  for ia = 1:n_arm
    A = res.(sprintf('p%d_%s', ip, arms(ia).name));
    fprintf('prior %.3f %-6s (start %+.3f from truth %.3f): pred 2nd-order %+.3e | measured %+.3e\n', ...
            PRIORS(ip), arms(ia).name, arms(ia).init - (1 + arms(ia).delta), ...
            1 + arms(ia).delta, sum(A.push_phase(:)), sum(A.dws_traj_phase));
  end
end

fprintf('\n================ M-approximation sensitivity ================\n');
for ip = 1:numel(PRIORS)
  S = res.(sprintf('p%d_stall', ip));
  fprintf('prior %.3f: c4 total, M = Dw_d[k-1] %+.3e | M += alpha*dw_m %+.3e (%+.1f%%)\n', ...
          PRIORS(ip), S.c4_tot, S.c4_tot_alt, 100*(S.c4_tot_alt - S.c4_tot)/abs(S.c4_tot));
end

fprintf('\n================ STATISTICS ON THE TARGETS (N = %d seed pairs) ================\n', N_SEED);
fprintf('phantom_i = (stall_i + mirror_i)/2 per seed; center = estimate started AT truth.\n');
fprintf('%-8s %-14s %12s %12s %8s\n', 'prior', 'quantity', 'mean', 'SEM', 't');
for ip = 1:numel(PRIORS)
  q = struct('name', {}, 'val', {});
  q(1).name = 'push (paired)';  q(1).val = (squeeze(V.push_meas(ip,1,:)) + squeeze(V.push_meas(ip,2,:)))/2;
  q(2).name = 'push (center)';  q(2).val = squeeze(V.push_meas(ip,3,:));
  q(3).name = 'innovD(paired)'; q(3).val = (squeeze(V.innov_desc(ip,1,:)) + squeeze(V.innov_desc(ip,2,:)))/2;
  q(4).name = 'innovD(center)'; q(4).val = squeeze(V.innov_desc(ip,3,:));
  q(5).name = 'innovH(center)'; q(5).val = squeeze(V.innov_hold(ip,3,:));
  q(6).name = 'ws_end(center)'; q(6).val = squeeze(V.ws_err_end(ip,3,:));
  q(7).name = 'pred 2nd (all)'; q(7).val = squeeze(mean(V.push_pred(ip,:,:), 2));
  for j = 1:numel(q)
    v = q(j).val; m = mean(v); se = std(v)/sqrt(numel(v));
    fprintf('%-8.3f %-14s %+12.3e %12.3e %8.2f\n', PRIORS(ip), q(j).name, m, se, m/se);
  end
end

out_file = fullfile(proj, 'test_results', 'audit_formB_second_order.mat');
save(out_file, 'res', 'PRIORS', 'TERM_NAME', 'PHASE_EDGES', '-v7.3');
fprintf('\nsaved: %s\n', out_file);


%% =================== Local Helpers ===================

function A = local_analyze_run(s, AX, PH, cfg, n_term)
%LOCAL_ANALYZE_RUN  Second-order term series + ws-push ledger for ONE run.
%   Index convention: the controller call at step k uses the POSTERIOR of
%   step k-1 as its prior, so x_curr(.) = *_out(k-1) and P_curr = P_full(k-1).
    N  = numel(s.tout);
    t  = s.tout(:);
    R  = s.R;
    cc = s.ctrl_const;
    P  = s.meta.params_value;
    lc = cfg.lambda_c;
    al = 1 - lc;                      % alpha (MA(2) weight / contraction slack)
    H2s = cfg.a_cov;                  % dy2_pred/dx4 (whitening scale)
    p_law = 1;                        % locked by lock_p

    a_o     = P.ctrl.Ts / (P.ctrl.gamma * R);
    kappa_T = 4 * (P.ctrl.k_B * P.ctrl.T / R) * a_o;
    s2n     = (cfg.meas_noise_std(AX) / R)^2;
    xi_bar  = (cc.C_n / cc.C_dpmr) * s2n / kappa_T;
    [S_T, S_n] = local_echo_S(lc, cfg.a_pd);

    % --- commanded height and its increments (exact, from the logged p_d) ---
    wd = s.p_d_out(:, AX) / R;                        % w_bar_d[k]
    dW = [0; diff(wd)];                               % = Delta_wbar_d[k-1]
    Gr = [0; 0; wd(3:end) - wd(1:end-2)];             % = Grad_wbar_d[k]

    % --- controller-side state at step k = posterior of step k-1 ---
    shift = @(v) [v(1); v(1:end-1)];
    ws_c = shift(s.ws_hat_out(:, AX));
    b_c  = shift(s.b_hat_out(:, AX));
    a_c  = shift(s.a_bar_hat_out(:, AX));
    gap  = max(wd - ws_c, 1e-3);                      % ws_margin floor

    [sl, s_g, s_gg, s_b, s_bb, s_gb] = local_slope_derivs(gap, b_c, p_law);

    % --- self-check 1: reconstructed a' vs the logged one (a_prime_out is
    %     a_bar' * a_disp and a_disp = a_nom = Ts/gamma_N) -----------------
    ap_log = s.a_prime_out(:, AX) / s.a_nom;
    ok = 3:N;
    A.chk_aprime = max(abs(sl(ok) - ap_log(ok)));

    % --- echo factor and H2 row (p column is zero: lock_p) ---------------
    S_i  = (S_T * a_c + S_n * xi_bar) ./ (a_c + xi_bar);
    ech  = 1 - S_i;
    H2_4 = H2s * ech;                       % dInnov/d(x4 error)
    H2_5 = -H2s * ech .* Gr .* s_b;         % -Grad*J_b
    H2_7 = +H2s * ech .* Gr .* s_g;         % -Grad*J_ws , J_ws = -s_g

    % --- covariance slices ------------------------------------------------
    Pf   = squeeze(s.P_full_out(:, :, :, AX));        % N x n x n
    pr   = @(i, j) [Pf(1, i, j); Pf(1:end-1, i, j)];  % prior  = posterior[k-1]
    po   = @(i, j) Pf(:, i, j);                       % posterior at k
    P55p = pr(5, 5); P77p = pr(7, 7); P57p = pr(5, 7);
    P37p = pr(3, 7); P87p = pr(8, 7); P97p = pr(9, 7);
    P35p = pr(3, 5); P85p = pr(8, 5); P95p = pr(9, 5);
    P55o = po(5, 5); P77o = po(7, 7); P57o = po(5, 7);

    % --- exact K2 from the posterior: Joseph gives P_post*H2' = K2*R2 -----
    R2   = s.R2_out(:, AX);
    gate = s.gate_out(:, AX);
    K2_4 = (po(4,4).*H2_4 + po(4,5).*H2_5 + po(4,7).*H2_7) ./ R2;
    K2_7 = (po(7,4).*H2_4 + po(7,5).*H2_5 + po(7,7).*H2_7) ./ R2;
    K2_4(gate) = 0; K2_7(gate) = 0;
    K2_4(1) = 0; K2_7(1) = 0;
    A.chk_K24 = max(abs(K2_4(ok) - s.K_a_y2_out(ok, AX)));
    A.chk_dws = max(abs(K2_7(ok) .* s.innov_y2_out(ok, AX) - s.dws_y2_out(ok, AX)));
    A.n_gated = nnz(gate);

    % --- row-4 error-dynamics multiplier F44 = 1 + a'*F_dw ---------------
    fb   = s.f_bar_out(:, AX);
    F_dw = fb + al * ([0; fb(1:end-1)] + [0; 0; fb(1:end-2)]);   % realized proxy
    F_dw = [0; F_dw(1:end-1)];                                    % controller uses [k-1]
    F44  = 1 + sl .* F_dw;
    A.chk_F44 = max(abs(sl(ok) .* F_dw(ok)));

    % --- M multiplier (row-4): approximation and its sensitivity leg ------
    M    = dW;
    M_alt = dW + al * (s.dh_m_out(:, AX) / R);

    % --- the eight second-order mean corrections --------------------------
    c4  = zeros(N, n_term);      % process row 4 (gain units, per step)
    cy2 = zeros(N, n_term);      % measurement y2 (y2 units, per step)
    c4(:, 1) = 0.5 * s_gg .* M .* P77p;                      % {ws,ws}
    c4(:, 2) = 0.5 * s_bb .* M .* P55p;                      % {b,b}
    c4(:, 3) = -s_gb .* M .* P57p;                           % {b,ws} (x2 folded)
    c4(:, 4) = -al * s_g .* (P37p + P87p + P97p);            % {ws,x3/m1/m2}
    c4(:, 5) =  al * s_b .* (P35p + P85p + P95p);            % {b,x3/m1/m2}
    cy2(:, 6) = -0.5 * H2s * ech .* Gr .* s_gg .* P77o;      % {ws,ws}
    cy2(:, 7) = -0.5 * H2s * ech .* Gr .* s_bb .* P55o;      % {b,b}
    cy2(:, 8) =  H2s * ech .* Gr .* s_gb .* P57o;            % {b,ws} (x2 folded)

    c4_alt1 = 0.5 * s_gg .* M_alt .* P77p;                   % M-sensitivity leg
    A.c4_tot     = sum(sum(c4(2:end, :)));
    A.c4_tot_alt = A.c4_tot + sum(c4_alt1(2:end) - c4(2:end, 1));

    % --- ws-push ledger, run once per term + once for all terms ----------
    push = zeros(N, n_term);
    for it = 1:n_term
        push(:, it) = local_ledger(c4(:, it), cy2(:, it), F44, K2_4, K2_7, H2s, gate);
    end
    [~, einnov] = local_ledger(sum(c4, 2), sum(cy2, 2), F44, K2_4, K2_7, H2s, gate);

    % --- LEDGER-FREE upper bound: no drain at all, so the row-4 bias keeps
    %     the whole accumulated correction (|beta| <= |beta_nodrain|) -------
    beta_nd   = -cumsum(sum(c4, 2));
    einnov_nd = -H2s * beta_nd + sum(cy2, 2);
    einnov_nd(gate) = 0;
    push_nd   = K2_7 .* einnov_nd;

    % --- what the measured phantom actually is: the sign-fixed a_bar error
    %     (innov_y2 ~ a_cov*(a_bar_true - a_bar_hat), so the two must agree)
    abar_err = s.a_bar_hat_out(:, AX) - s.a_true_out(:, AX) / s.a_nom;

    % --- phase integrals --------------------------------------------------
    A.push_phase   = zeros(n_term, 3);
    A.corr_phase   = zeros(n_term, 3);
    A.einnov_phase = zeros(1, 3);
    A.innov_y2_phase = zeros(1, 3);
    A.dws_y1_phase = zeros(1, 4);
    A.dws_y2_phase = zeros(1, 4);
    A.dws_traj_phase = zeros(1, 4);
    A.einnov_nd_phase = zeros(1, 3);
    A.push_nd_phase   = zeros(1, 4);
    A.aerr_phase      = zeros(1, 3);
    A.aerr_from_innov = zeros(1, 3);
    A.K27_phase       = zeros(1, 3);
    A.K24_phase       = zeros(1, 3);
    A.innov_y1_phase  = zeros(1, 3);
    for iw = 1:3
        w = t >= PH(iw, 1) & t < PH(iw, 2);
        A.push_phase(:, iw) = sum(push(w, :), 1).';
        A.corr_phase(:, iw) = sum(c4(w, :) + cy2(w, :), 1).';
        A.einnov_phase(iw)  = mean(einnov(w));
        A.innov_y2_phase(iw) = mean(s.innov_y2_out(w, AX));
        A.dws_y1_phase(iw)  = sum(s.dws_y1_out(w, AX));
        A.dws_y2_phase(iw)  = sum(s.dws_y2_out(w, AX));
        k0 = find(w, 1, 'first'); k1 = find(w, 1, 'last');
        A.dws_traj_phase(iw) = s.ws_hat_out(k1, AX) - s.ws_hat_out(k0, AX);
        A.einnov_nd_phase(iw) = mean(einnov_nd(w));
        A.push_nd_phase(iw)   = sum(push_nd(w));
        A.aerr_phase(iw)      = mean(abar_err(w));
        A.aerr_from_innov(iw) = -mean(s.innov_y2_out(w, AX)) / H2s;
        A.K27_phase(iw)       = mean(K2_7(w));
        A.K24_phase(iw)       = mean(K2_4(w));
        A.innov_y1_phase(iw)  = mean(s.innov_y1_out(w, AX));
    end
    A.push_nd_phase(4) = sum(A.push_nd_phase(1:3));
end

function [push, einnov] = local_ledger(c4, cy2, F44, K2_4, K2_7, H2s, gate)
%LOCAL_LEDGER  Propagate the row-4 mean bias and read off the w_s push.
%   beta = EKF mean minus true conditional mean of a_bar.
%     predict : beta <- F44*beta - c4          (c4 = the omitted mean shift)
%     y2      : E[innov] = -H2s*beta + cy2 ;  dws = K2(7)*E[innov] ;
%               beta <- beta + K2(4)*E[innov]
    N = numel(c4);
    push = zeros(N, 1); einnov = zeros(N, 1); beta = 0;
    for k = 2:N
        beta = F44(k) * beta - c4(k);
        if ~gate(k)
            ei = -H2s * beta + cy2(k);
            einnov(k) = ei;
            push(k)   = K2_7(k) * ei;
            beta      = beta + K2_4(k) * ei;
        end
    end
end

function [sl, s_g, s_gg, s_b, s_bb, s_gb] = local_slope_derivs(g, b, p)
%LOCAL_SLOPE_DERIVS  s = da_bar/dw_bar and its derivatives w.r.t. (g, b).
%   s      = (p/b)*u^(-p-1) ,  u = 1 + g/b                        (= a_bar')
%   s_g    = -(p+1)/(g+b) * s                                     (= a'')
%   s_gg   = (p+1)*(p+2)/(g+b)^2 * s                              (= a''')
%   s_b    = Lb*s ,  Lb = -1/b + (p+1)*g/(b*(g+b))                (= J_b)
%   s_bb   = (Lb^2 + dLb/db)*s ,
%            dLb/db = 1/b^2 - (p+1)*g*(g+2b)/(b^2*(g+b)^2)
%   s_gb   = -(p+1)*(Lb*(g+b) - 1)/(g+b)^2 * s
    u    = 1 + g ./ b;
    sl   = (p ./ b) .* u.^(-p-1);
    gb   = g + b;
    s_g  = -(p+1) ./ gb .* sl;
    s_gg = (p+1) * (p+2) ./ gb.^2 .* sl;
    Lb   = -1 ./ b + (p+1) * g ./ (b .* gb);
    dLb  = 1 ./ b.^2 - (p+1) * g .* (g + 2*b) ./ (b.^2 .* gb.^2);
    s_b  = Lb .* sl;
    s_bb = (Lb.^2 + dLb) .* sl;
    s_gb = -(p+1) * (Lb .* gb - 1) ./ gb.^2 .* sl;
end

function local_verify_derivatives()
%LOCAL_VERIFY_DERIVATIVES  Analytic vs central difference at three gaps.
    fprintf('\n--- derivative verification (analytic / finite difference) ---\n');
    for g = [0.5, 1.0, 5.0]
        b = 9/8; p = 1; h = 1e-5;
        S = @(gg, bb) (p ./ bb) .* (1 + gg ./ bb).^(-p-1);
        [sl, s_g, s_gg, s_b, s_bb, s_gb] = local_slope_derivs(g, b, p);
        fd_g  = (S(g+h, b) - S(g-h, b)) / (2*h);
        fd_gg = (S(g+h, b) - 2*S(g, b) + S(g-h, b)) / h^2;
        fd_b  = (S(g, b+h) - S(g, b-h)) / (2*h);
        fd_bb = (S(g, b+h) - 2*S(g, b) + S(g, b-h)) / h^2;
        fd_gb = (S(g+h, b+h) - S(g+h, b-h) - S(g-h, b+h) + S(g-h, b-h)) / (4*h^2);
        fprintf(['g=%.2f: s %.6f | s_g %+.6f/%+.6f | s_gg %+.6f/%+.6f | ' ...
                 's_b %+.6f/%+.6f | s_bb %+.6f/%+.6f | s_gb %+.6f/%+.6f\n'], ...
                g, sl, s_g, fd_g, s_gg, fd_gg, s_b, fd_b, s_bb, fd_bb, s_gb, fd_gb);
    end
end

function [S_T, S_n] = local_echo_S(lambda_c, a_pd)
%LOCAL_ECHO_S  Replica of the controller's y2 self-echo sensitivities (exact
%   6-state Lyapunov of the mismatched hold loop; thermal and noise shares).
    alE = 1 - lambda_c; epE = 1e-4;
    vE = zeros(2, 3); gE_list = [1, 1/(1+epE), 1/(1-epE)];
    for iN = 1:2
        for iG = 1:3
            gE = gE_list(iG);
            AE = zeros(6); BqE = zeros(6,1); BnE = zeros(6,1);
            AE(1,1)=1; AE(1,3)=-gE*alE; AE(1,4)=-gE*alE; AE(1,5)=-gE*alE;
            BnE(1)=-gE*alE; BqE(1)=1;
            AE(2,1)=1; AE(3,2)=1;
            AE(4,3)=-alE; AE(4,4)=-alE; AE(4,5)=-alE; BnE(4)=-alE;
            AE(5,4)=1;
            AE(6,3)=a_pd; AE(6,6)=1-a_pd; BnE(6)=a_pd;
            if iN == 1; QE = BqE*BqE.'; extraE = 0;
            else;       QE = BnE*BnE.'; extraE = (1-a_pd)^2; end
            XE = reshape((eye(36) - kron(AE,AE)) \ QE(:), 6, 6);
            cE = zeros(1,6); cE(3) = 1-a_pd; cE(6) = -(1-a_pd);
            vE(iN,iG) = cE*XE*cE.' + extraE;
        end
    end
    S_T = (log(vE(1,2)) - log(vE(1,3))) / (2*epE);
    S_n = (log(vE(2,2)) - log(vE(2,3))) / (2*epE);
end

function A = local_accumulate(A, B)
%LOCAL_ACCUMULATE  Field-wise sum (chk_* fields take the max instead).
    f = fieldnames(A);
    for i = 1:numel(f)
        if startsWith(f{i}, 'chk_')
            A.(f{i}) = max(A.(f{i}), B.(f{i}));
        else
            A.(f{i}) = A.(f{i}) + B.(f{i});
        end
    end
end

function A = local_finalize(A, n)
%LOCAL_FINALIZE  Turn the accumulated sums into per-run means.
    f = fieldnames(A);
    for i = 1:numel(f)
        if ~startsWith(f{i}, 'chk_')
            A.(f{i}) = A.(f{i}) / n;
        end
    end
end

function [sPbb, sPpp, floor_a] = local_envelope_priors(h_lo, h_hi, B_SEED, P_SEED)
%LOCAL_ENVELOPE_PRIORS  Replica of run_formB_ws.local_envelope_priors.
    N_SWEEP = 20001;
    h = linspace(h_lo, h_hi, N_SWEEP).';
    c = zeros(N_SWEEP, 1); dc = zeros(N_SWEEP, 1);
    for i = 1:N_SWEEP
        [~, c_i, dv_i] = calc_correction_functions(h(i), true);
        c(i) = c_i; dc(i) = dv_i.dc_perp_dh;
    end
    b_eff = (c - 1) .* (h - 1);
    p_eff = -((h - 1) + B_SEED) .* dc ./ (c .* (c - 1));
    sPbb  = max(abs(b_eff - B_SEED));
    sPpp  = max(abs(p_eff - P_SEED));
    a_anch = 1 - (1 + (h - 1) / B_SEED).^(-P_SEED);
    floor_a = max(abs(a_anch - 1 ./ c));
end
