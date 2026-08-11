% STATUS: ACTIVE | B8 follow-ups 1 & 2 (pre-registered, WITH magnitude floors)
%   EXPIRES: when the production-width deficit is on record and the y2-echo hypothesis is
%   adjudicated.  Zero production changes: production is called only through
%   run_formB_ws(opts) / run_formB_ws(cfg, test_opts).
% FORK OF test_script/scratch/check_formB_b8_selfconfirm.m @ untracked-2026-08-10 |
%   the local statistics helpers are copied verbatim and generalised to per-arm time
%   grids (that script's arms all share T = 20.8 s; these do not).  The frozen B8
%   pre-registration in the parent file is NOT edited. | 產線改動不會自動跟上
%
% =======================================================================================
% WHY THE STATISTIC CHANGED FROM THE PARENT SCRIPT: beta, not the information ratio
% =======================================================================================
% The parent reported the over-claim in the INFORMATION currency (dI_claim/dI_real =
% 1.108).  Reading the controller for task 2 shows that is the wrong currency for a
% mis-scaled measurement row, and the right one is derivable in three lines.
%
% Scalar static parameter, filter believes sensitivity h_u, truth is h_t:
%     1/P_k = 1/P_{k-1} + h_u^2/R                       (claimed)
%     m_k   = m_{k-1}(1 - K_k h_u) + K_k h_t ,  g_k = K_k h_u = 1 - P_k/P_{k-1}
%           = m_{k-1}(1 - g_k) + beta g_k ,     beta = h_t/h_u
%  telescoping with m_0 = 0:
%     m_k = beta * (1 - P_k/P_0)      i.e.     G_meas(t) = beta * G_claim(t) ,   beta CONST.
%
% So a mis-scaled H row shows up as a CONSTANT multiplicative deficit in the G currency.
% The information-currency ratio it induces is a derived quantity that grows with G:
%     I_claim/I_real = (1 - beta G) / (beta (1 - G))
% which at the parent's numbers gives 1.084 at G = 0.856 from beta = 0.988 -- i.e. the
% parent's "10.8 % excess" and a 1.2 % response deficit are the SAME observation in two
% currencies, and the growth of the excess from 1.068 (4.8 s) to 1.108 (20.8 s) is what
% a constant beta predicts.  beta is therefore the statistic here.
%
% =======================================================================================
% CODE READING FOR TASK 2 (done before the design; team-lead premise checked)
% =======================================================================================
%  * echo_fac = 1 - S_i multiplies the ENTIRE H2 row uniformly
%    (motion_control_law_formB_ws.m:1072), all state columns.                    CONFIRMED
%  * R2 is echo-FREE: compute_R2_formB builds R2_int = amlpf*K_var*IF*(a_bar+xi_bar)^2
%    (line 1293) with no echo term.                                              CONFIRMED
%    => claimed per-step information H2'/R2*H2 scales exactly as (1-S_u)^2.
%  * echo_fac ALSO enters the nonlinear prediction y2_pred (line 1082, the a_bar' back-off
%    term).  That is an innovation OFFSET, not a row scale, so it produces common-mode
%    drift rather than a response scaling -- reported separately, not folded into beta.
%  * S is NOT a knob.  S_echo_T / S_echo_n are persistent, computed in the init block from
%    a 6-state Lyapunov solve that depends only on lambda_c and a_pd; ctrl_const exposes
%    only the BINARY y2_echo_corr.  A continuous S sweep would require editing production,
%    so the two-point flag arm is the only legal lever.  H2_scale = a_cov is the y2
%    whitening normalisation (it also builds y2 itself), so scaling it is not an
%    S-surrogate.
%  * CORRECTION to the assumed S_used = 0.32: the run-time S is a variance-share BLEND of
%    S_echo_T = +0.32812 and S_echo_n = -0.24373 (negative), weights (a_bar, xi_bar).
%    With xi_bar(z) = 0.01527 and a_bar in [0.458, 0.950] over this envelope,
%        S_used = 0.3097 (trough) .. 0.3191 (far field),  1 - S_used = 0.6809 .. 0.6903.
%    So the premise holds to 2 %, but the SCALE to use is 1 - S_used ~ 0.686, not 0.68,
%    and S_used is mildly height-dependent -- which the free test 2a exploits.
%
% =======================================================================================
% PRE-REGISTRATION -- every criterion is "effect > X AND CI3 excludes X", X sourced
% =======================================================================================
% TASK 1 (production width 0.0157, d = 0.0157, r = 1, T = 4.8 s, 30 seeds/sign)
%   Statistic beta_prod(4.8) = G_meas/G_claim at t = 4.8 s.
%   X  = the main arm's beta at the SAME time, beta_main(4.8) = 0.99015, read from the
%        frozen parent .mat together with its own 3-sigma bootstrap half-width h_main.
%        Provenance: measurement, same statistic, same time, same estimator.
%   (i)   COMPATIBLE      : |beta_prod - beta_main| <= h_main AND CI3 of the difference
%                           contains 0.
%   (ii)  SMALLER DEFICIT : (beta_prod - beta_main) > h_main AND CI3 of the difference
%                           excludes h_main.   [floor = the precision of the number it is
%                           being compared with; a smaller difference is not a claim]
%   (iii) LARGER DEFICIT  : mirror of (ii).
%   (iv)  UNRESOLVED      : 3-sigma half-width of beta_prod > (1 - beta_main) = 0.00985,
%                           i.e. the arm cannot see the whole effect under test.  Then
%                           report the required N = 30 * (halfwidth/0.00985)^2.
%
% TASK 2 (y2_echo_corr = false, width 0.10, d = 0.10, T = 20.8 s, 15 seeds/sign, the SAME
%         seeds 81001-81015 as the main arm -> the beta_off - beta_on comparison is
%         paired at the seed level, not just at the arm level)
%   Statistic beta_off(20.8), and the paired difference against beta_on on those seeds.
%   S-HYPOTHESIS prediction: with the correction OFF, S_used = 0 EXACTLY, so
%        beta_off = 1 - S_true      (no derivation uncertainty enters).
%      X_band = 1 - [0.323 +- 3*0.043] = [0.548, 0.806].   Provenance: the independent
%      paired-forcing measurement of S recorded in memory (0.323 +- 0.043).
%      Point prediction from the ON arm itself: beta_off = beta_on * (1 - S_used)
%        = 0.98797 * 0.686 = 0.678.
%   S-INNOCENT prediction: beta_off = beta_on = 0.988 (turning the row scale off changes
%      nothing about the deficit).
%   (i)   S GUILTY   : beta_off inside X_band AND CI3(beta_off) excludes beta_on.
%   (ii)  S INNOCENT : beta_off > 0.806 AND CI3(beta_off) entirely above X_band AND
%                      CI3(beta_off - beta_on) contains 0.
%   (iii) PARTIAL    : CI3 excludes BOTH X_band and beta_on -> the row scale explains part
%                      of it; report the fraction and the next candidate.
%   (iv)  UNRESOLVED : CI3 half-width > (beta_on - 0.806)/2 = 0.091, i.e. the arm cannot
%                      separate the two predictions.
%   NOTE the ON arm alone CANNOT test this: any beta_on in [0.81, 1.19] maps to an S_true
%   inside the +-3*0.043 band, which is 12x wider than the 0.011 that beta_on moves over
%   the whole run.  The discriminating power is entirely in the OFF arm.
%
% TASK 2a (FREE, stored data only, no runs): the blend makes S_used height-dependent, so
%   a constant-S_true H-scaling predicts a SHAPE for beta(t):
%        beta(t) = (1 - S_true) / (1 - S_used(t)) ,  S_used from the reconstructed a_bar.
%   One free constant against a whole curve.  Pre-registered: the fitted S_true must land
%   inside X_band AND the shape residual must be smaller than the drift it explains
%   (|beta(20.8) - beta(1.5)| = 0.0108), otherwise the shape test is uninformative.
%
%   RESUMABLE: MAX_RUNS_PER_CALL jobs per invocation, progress saved after every run.
%   Output: test_results/temp_formB_b8_followups.mat (gitignored).

clear cd    % MATLAB cd-shadowing hang, recorded in memory

here = fileparts(mfilename('fullpath'));
proj = fullfile(here, '..', '..');
addpath(genpath(fullfile(proj, 'model')));
addpath(fullfile(proj, 'test_script', 'integration'));

% ---------------- design constants ----------------------------------------------------
AX_Z      = 3;
B_ANCHOR  = 9/8;
P_TRUE    = 1;
WS_TRUE   = 1;
DEC       = 16;                     % 1600 Hz -> 100 Hz log decimation
W_PROD    = 0.0157;                 % production envelope prior width
W_MAIN    = 0.10;                   % main-arm width (echo arm must match it)
T_PROD    = 4.8;                    % [s] canonical scenario length
T_LONG    = 20.8;                   % [s] main-arm length (echo arm must match it)
T_REF     = 1.5;                    % [s] end of descent
N_PROD    = 30;                     % seeds per sign, task 1
N_ECHO    = 15;                     % seeds per sign, task 2
SEED0_PROD = 91000;                 % fresh block
SEED_ECHO  = 81001:81015;           % REUSED from the main arm on purpose (paired)
S_MEAS     = 0.323;                 % independent paired-forcing measurement of the echo
S_MEAS_SD  = 0.043;                 %   share, memory project_formB_y2_echo_derivation
MAX_RUNS_PER_CALL = 24;
N_BOOT    = 10000;
BOOT_SEED = 20260810;
SIG3      = 3;

db_file  = fullfile(proj, 'test_results', 'temp_formB_b8_followups.mat');
par_file = fullfile(proj, 'test_results', 'temp_formB_b8_selfconfirm.mat');
assert(exist(par_file, 'file') == 2, 'b8fu:noParent', ...
       'the frozen B8 result is required as the comparator (missing %s).', par_file);

% ---------------- job list ------------------------------------------------------------
jobs = struct('key', {}, 'arm', {}, 'sgn', {}, 'seed', {}, 'width', {}, 'd', {}, ...
              'b_true', {}, 'T', {}, 'echo', {});
for s = [1 -1]
    for i = 1:N_PROD
        jobs(end+1) = local_job('prod', s, SEED0_PROD + i, W_PROD, W_PROD, ...
                                B_ANCHOR + s * W_PROD, T_PROD, true); %#ok<SAGROW>
    end
end
for s = [1 -1]
    for i = 1:N_ECHO
        jobs(end+1) = local_job('echooff', s, SEED_ECHO(i), W_MAIN, W_MAIN, ...
                                B_ANCHOR + s * W_MAIN, T_LONG, false); %#ok<SAGROW>
    end
end

% ---------------- resume --------------------------------------------------------------
if exist(db_file, 'file')
    S = load(db_file);  DB = S.DB;
    if isfield(S, 'HS'); HS = S.HS; else; HS = struct(); end
else
    DB = struct('key', {}, 'arm', {}, 'sgn', {}, 'seed', {}, 'width', {}, 'd', {}, ...
                'b_true', {}, 'T', {}, 'echo', {}, 't', {}, 'b_hat', {}, 'sP_b', {}, ...
                'a_hat', {}, 'a_true', {}, 'hb_lo', {}, 'hb_hi', {}, 'nan_flag', {});
    HS = struct();
end
todo = find(~ismember({jobs.key}, {DB.key}));
fprintf('B8 follow-ups: %d/%d jobs done, %d remaining\n', numel(DB), numel(jobs), numel(todo));

n_this_call = 0;
for j = todo
    if n_this_call >= MAX_RUNS_PER_CALL; break; end
    job = jobs(j);
    ck = matlab.lang.makeValidName(sprintf('%s_%g', job.arm, job.width));
    if ~isfield(HS, ck)
        HS.(ck) = local_handshake(job, P_TRUE, WS_TRUE, AX_Z);
    end
    H = HS.(ck);
    pc_handle = @(hb) calc_formB_cperp(hb, ...
                    struct('b', job.b_true, 'p', P_TRUE, 'ws', WS_TRUE));
    topts = struct('seed', job.seed, 'ctrl_const_override', H.ctrl_const, ...
                   'verbose', false, 'a_ctrl_override', [], 'log_P_full', false, ...
                   'ws_inject', 0, 'plant_cperp', pc_handle);
    r = run_formB_ws(H.cfg, topts);

    idx = 1:DEC:numel(r.tout);
    rec = job;
    rec.t = r.tout(idx);            rec.b_hat = r.b_hat_out(idx, AX_Z);
    rec.sP_b = r.P_b_out(idx, AX_Z); rec.a_hat = r.a_hat_out(idx, AX_Z);
    rec.a_true = r.a_true_out(idx, AX_Z);
    rec.hb_lo = min(r.h_bar_true_out); rec.hb_hi = max(r.h_bar_true_out);
    rec.nan_flag = any(~isfinite(r.b_hat_out(:, AX_Z)));
    DB(end+1) = rec; %#ok<SAGROW>
    save(db_file, 'DB', 'HS', '-v7.3');
    n_this_call = n_this_call + 1;
    fprintf('  [%3d/%3d] %-22s b_true %7.4f  b_hat_end %7.4f  sqrtP_end %8.5f\n', ...
            numel(DB), numel(jobs), job.key, job.b_true, rec.b_hat(end), rec.sP_b(end));
end

if numel(DB) < numel(jobs)
    fprintf('PARTIAL: %d/%d done. Re-run to continue.\n', numel(DB), numel(jobs));
    return;
end
fprintf('ALL JOBS DONE (%d runs).\n\n', numel(DB));

% =======================================================================================
%  ANALYSIS
% =======================================================================================
rng(BOOT_SEED);
PAR = load(par_file, 'res');  Pm = PAR.res.main;
kA_par = PAR.res.kA;  kB_par = PAR.res.kB;  kR_par = PAR.res.kR;
beta_main   = Pm.G_meas ./ Pm.G_claim;
beta_main_A = beta_main(kA_par);   % t = 4.8 s, the task-1 comparator
beta_main_B = beta_main(kB_par);   % t = 20.8 s, the task-2 comparator

A_prod = local_analyse(DB, 'prod',    B_ANCHOR, N_BOOT, SIG3);
A_echo = local_analyse(DB, 'echooff', B_ANCHOR, N_BOOT, SIG3);

% --- main-arm comparator uncertainty, recomputed from the frozen per-run data ----------
[bm_A, hm_A] = local_beta_from_parent(par_file, 'main', kA_par, [], N_BOOT, SIG3);
[bm_B, hm_B] = local_beta_from_parent(par_file, 'main', kB_par, SEED_ECHO, N_BOOT, SIG3);
fprintf('COMPARATORS from the frozen parent .mat:\n');
fprintf('  beta_main(4.8 s)  = %.5f  +-%.5f (3sd, 30 pairs)   [task-1 X]\n', bm_A, hm_A);
fprintf('  beta_main(20.8 s) = %.5f  +-%.5f (3sd, the 15 paired seeds)  [task-2 X]\n\n', ...
        bm_B, hm_B);

% =============================== TASK 1 ================================================
kP = find(A_prod.t >= T_PROD - 1e-9, 1);
kPr = find(A_prod.t >= T_REF - 1e-9, 1);
fprintf('===== TASK 1: production width %.4f (%d runs, T = %.1f s) =====\n', ...
        W_PROD, A_prod.n_runs, T_PROD);
fprintf('%8s | %7s %7s %7s | %8s %8s | %8s %8s %8s\n', 't[s]', 'rho', 'rho_lo', ...
        'rho_hi', 'G_claim', 'G_meas', 'beta', 'beta_lo', 'beta_hi');
for k = [kPr kP]
    fprintf('%8.2f | %7.3f %7.3f %7.3f | %8.4f %8.4f | %8.5f %8.5f %8.5f\n', ...
            A_prod.t(k), A_prod.rho(k), A_prod.rho_lo(k), A_prod.rho_hi(k), ...
            A_prod.G_claim(k), A_prod.G_meas(k), A_prod.beta(k), ...
            A_prod.beta_lo(k), A_prod.beta_hi(k));
end
hw_prod = (A_prod.beta_hi(kP) - A_prod.beta_lo(kP)) / 2;
d_beta  = A_prod.beta(kP) - bm_A;
sd_diff = sqrt(((A_prod.beta_hi(kP) - A_prod.beta_lo(kP)) / (2*SIG3))^2 + (hm_A/SIG3)^2);
fprintf('  beta_prod(4.8) - beta_main(4.8) = %+.5f  (3sd of the difference %.5f)\n', ...
        d_beta, SIG3 * sd_diff);
fprintf('  information-currency excess (1-beta*G)/(beta*(1-G)) at 4.8 s: prod %.4f | main %.4f\n', ...
        local_iratio(A_prod.beta(kP), A_prod.G_claim(kP)), ...
        local_iratio(bm_A, Pm.G_claim(kA_par)));
fprintf('  traversal budget %.3f [%.3f %.3f] vs null %.3f\n', A_prod.bud(kP), ...
        A_prod.bud_lo(kP), A_prod.bud_hi(kP), A_prod.bud_hon(kP));
if hw_prod > (1 - bm_A)
    v1 = sprintf('UNRESOLVED (3sd half-width %.5f > effect %.5f; need N = %.0f pairs)', ...
                 hw_prod, 1 - bm_A, N_PROD * (hw_prod / (1 - bm_A))^2);
elseif abs(d_beta) <= hm_A && abs(d_beta) <= SIG3 * sd_diff
    v1 = 'COMPATIBLE with the main arm';
elseif d_beta > hm_A && (d_beta - SIG3 * sd_diff) > hm_A
    v1 = 'SMALLER deficit than the main arm';
elseif d_beta < -hm_A && (d_beta + SIG3 * sd_diff) < -hm_A
    v1 = 'LARGER deficit than the main arm';
else
    v1 = 'INDETERMINATE (between the pre-registered bands)';
end
fprintf('  >>> TASK 1 VERDICT: %s\n\n', v1);

% =============================== TASK 2 ================================================
kE = numel(A_echo.t);
kEr = find(A_echo.t >= T_REF - 1e-9, 1);
band = [1 - (S_MEAS + SIG3*S_MEAS_SD), 1 - (S_MEAS - SIG3*S_MEAS_SD)];
fprintf('===== TASK 2: y2_echo_corr = false (%d runs, T = %.1f s, seeds paired with main) =====\n', ...
        A_echo.n_runs, T_LONG);
fprintf('%8s | %7s %7s %7s | %8s %8s | %8s %8s %8s\n', 't[s]', 'rho', 'rho_lo', ...
        'rho_hi', 'G_claim', 'G_meas', 'beta', 'beta_lo', 'beta_hi');
for k = [kEr kE]
    fprintf('%8.2f | %7.3f %7.3f %7.3f | %8.4f %8.4f | %8.5f %8.5f %8.5f\n', ...
            A_echo.t(k), A_echo.rho(k), A_echo.rho_lo(k), A_echo.rho_hi(k), ...
            A_echo.G_claim(k), A_echo.G_meas(k), A_echo.beta(k), ...
            A_echo.beta_lo(k), A_echo.beta_hi(k));
end
b_off = A_echo.beta(kE);
hw_echo = (A_echo.beta_hi(kE) - A_echo.beta_lo(kE)) / 2;
pt_pred = bm_B * 0.686;                       % beta_on * (1 - S_used), S_used from code
fprintf('  S-hypothesis band (1 - S_meas +- 3sd): [%.4f %.4f]; point prediction %.4f\n', ...
        band(1), band(2), pt_pred);
fprintf('  S-innocent prediction: beta_off = beta_on = %.5f\n', bm_B);
fprintf('  implied S_true = 1 - beta_off = %.4f  (S_used = 0 exactly in this arm)\n', 1 - b_off);
fprintf('  common-mode drift at t_end %+.5f [%+.5f %+.5f]  (echo also enters y2_pred)\n', ...
        A_echo.cm(kE), A_echo.cm_lo(kE), A_echo.cm_hi(kE));
fprintf('  b_hat range over runs [%.4f %.4f] (clamp check)\n', A_echo.b_lo, A_echo.b_hi);
if hw_echo > (bm_B - band(2)) / 2
    v2 = sprintf('UNRESOLVED (3sd half-width %.4f > %.4f)', hw_echo, (bm_B - band(2))/2);
elseif b_off >= band(1) && b_off <= band(2) && A_echo.beta_hi(kE) < bm_B
    v2 = 'S GUILTY -- the deficit is the y2 echo row scale';
elseif A_echo.beta_lo(kE) > band(2) && abs(b_off - bm_B) <= hw_echo
    v2 = 'S INNOCENT -- the deficit does not follow the echo row scale';
elseif A_echo.beta_lo(kE) > band(2) && A_echo.beta_hi(kE) < bm_B
    v2 = 'PARTIAL -- the echo row scale explains part of the deficit';
else
    v2 = 'INDETERMINATE (between the pre-registered bands)';
end
fprintf('  >>> TASK 2 VERDICT: %s\n\n', v2);

% =============================== TASK 2a (free) ========================================
% Reconstruct S_used(t) from the true a_bar and test the SHAPE beta(t) with one constant.
pc = physical_constants();
prm = calc_simulation_params(HS.(matlab.lang.makeValidName(sprintf('echooff_%g', W_MAIN))).cfg);
Pv = prm.Value;
a_nom = Pv.common.Ts / Pv.common.gamma_N;
S_T = 0.32812; S_n = -0.24373; xi_z = 0.015265;    % from the offline Lyapunov replica
Dp = load(par_file, 'DB');
abar_t = mean([Dp.DB(strcmp({Dp.DB.arm}, 'main')).a_true], 2) / a_nom;
S_used_t = (S_T * abar_t + S_n * xi_z) ./ (abar_t + xi_z);
msk = PAR.res.main.G_claim > 0.2;               % below that beta is 0/0 noise
obj = @(st) sum((beta_main(msk) - (1 - st) ./ (1 - S_used_t(msk))).^2);
S_fit = fminbnd(obj, -0.5, 0.9, optimset('TolX', 1e-8));
resid = beta_main(msk) - (1 - S_fit) ./ (1 - S_used_t(msk));
drift = abs(beta_main(kB_par) - beta_main(kR_par));
fprintf('===== TASK 2a (free, stored data): shape of beta(t) vs the reconstructed S_used(t) =====\n');
fprintf('  S_used(t) reconstructed range over the run: [%.4f %.4f]\n', ...
        min(S_used_t), max(S_used_t));
fprintf('  one-constant fit  S_true = %.4f   (band [%.4f %.4f] -> %s)\n', S_fit, ...
        band(1), band(2), local_inband(1 - S_fit, band));
fprintf('  shape residual RMS %.5f vs the drift it must explain %.5f -> %s\n', ...
        sqrt(mean(resid.^2)), drift, ...
        local_verdict_shape(sqrt(mean(resid.^2)), drift));
fprintf('  beta_main(1.5) %.5f -> beta_main(20.8) %.5f ; H-scaling predicts ratio %.5f\n', ...
        beta_main(kR_par), beta_main(kB_par), ...
        (1 - S_used_t(kR_par)) / (1 - S_used_t(kB_par)));

res = struct('prod', A_prod, 'echo', A_echo, 'beta_main', beta_main, ...
             'bm_A', bm_A, 'hm_A', hm_A, 'bm_B', bm_B, 'hm_B', hm_B, ...
             'band', band, 'S_fit', S_fit, 'verdict1', v1, 'verdict2', v2, ...
             'S_used_t', S_used_t);
save(db_file, 'DB', 'HS', 'res', '-v7.3');
fprintf('\nsaved: %s\n', db_file);


%% =================== local helpers ===================

function j = local_job(arm, sgn, seed, width, d, b_true, T, echo)
    j = struct('key', sprintf('%s|%+d|%d', arm, sgn, seed), 'arm', arm, 'sgn', sgn, ...
               'seed', seed, 'width', width, 'd', d, 'b_true', b_true, 'T', T, 'echo', echo);
end


function H = local_handshake(job, p_true, ws_true, ax)
%LOCAL_HANDSHAKE  Production arm-form call -> cfg + ctrl_const, with a bit-identity
%   assertion that the cheap test-ladder path reproduces it exactly.
    o = struct();
    o.seeds = job.seed;
    o.config_override     = struct('T_sim', job.T);
    o.ctrl_const_override = struct('lock_b', false, 'lock_p', true, 'lock_ws', true, ...
                                   'Pf_b_std', job.width);
    if ~job.echo
        o.ctrl_const_override.y2_echo_corr = false;
    end
    o.plant_gain_law = struct('b', job.b_true, 'p', p_true, 'ws', ws_true);
    [~, out] = evalc('run_formB_ws(o)');

    H = struct('cfg', out.cfg, 'ctrl_const', out.runs{1}.ctrl_const);
    topts = struct('seed', job.seed, 'ctrl_const_override', H.ctrl_const, ...
                   'plant_cperp', @(hb) calc_formB_cperp(hb, ...
                       struct('b', job.b_true, 'p', p_true, 'ws', ws_true)));
    chk = run_formB_ws(H.cfg, topts);
    dd = max(abs(chk.b_hat_out(:, ax) - out.runs{1}.b_hat_out(:, ax)));
    assert(dd == 0, 'b8fu:pathDrift', 'cheap path differs by %.3e.', dd);
    assert(abs(out.runs{1}.P_b_out(1, ax) - job.width) < 1e-12, 'b8fu:priorWidth', ...
           'prior %.6f != %.6f.', out.runs{1}.P_b_out(1, ax), job.width);
    fprintf('  handshake %s (echo=%d, T=%.1f): prior sqrt(P_bb)[0] = %.4f, paths bit-identical\n', ...
            job.arm, job.echo, job.T, out.runs{1}.P_b_out(1, ax));
end


function A = local_analyse(DB, arm, b0, n_boot, sig)
%LOCAL_ANALYSE  Paired +/- arm on its own time grid: beta, rho, common mode, budget.
    ip = find(strcmp({DB.arm}, arm) & [DB.sgn] > 0);
    im = find(strcmp({DB.arm}, arm) & [DB.sgn] < 0);
    [~, o1] = sort([DB(ip).seed]); ip = ip(o1);
    [~, o2] = sort([DB(im).seed]); im = im(o2);
    n = numel(ip);
    d = DB(ip(1)).d;  width = DB(ip(1)).width;

    Bp = [DB(ip).b_hat]; Bm = [DB(im).b_hat];
    Ep = Bp - (b0 + d);  Em = Bm - (b0 - d);
    sP = mean([[DB(ip).sP_b] [DB(im).sP_b]], 2);
    dP = width^2 - sP.^2;

    A = struct();
    A.arm = arm; A.width = width; A.d = d; A.n_runs = numel(ip) + numel(im);
    A.t = DB(ip(1)).t;
    A.r = d^2 / width^2;
    A.G_claim = 1 - (sP.^2) / width^2;
    A.rho_hon = sqrt(max(1 + (A.r - 1) * (1 - A.G_claim), 0));
    A.bud_hon = A.G_claim * A.r + (1 - A.G_claim);
    A.sP = sP;
    A.b_lo = min([Bp(:); Bm(:)]);  A.b_hi = max([Bp(:); Bm(:)]);
    A.n_nan = sum([DB([ip im]).nan_flag]);

    st = @(sel) local_stats(Ep(:, sel), Em(:, sel), Bp(:, sel), Bm(:, sel), sP, dP, ...
                            b0, d, A.G_claim);
    s0 = st(1:n);
    A.rho = s0.rho; A.rms = s0.rms; A.G_meas = s0.G; A.cm = s0.cm;
    A.bud = s0.bud; A.beta = s0.beta;

    gi = unique([1:20:numel(sP), numel(sP)]);
    stg = @(sel) local_stats(Ep(gi, sel), Em(gi, sel), Bp(gi, sel), Bm(gi, sel), ...
                             sP(gi), dP(gi), b0, d, A.G_claim(gi));
    boot = zeros(n_boot, numel(gi), 4);
    for r = 1:n_boot
        sel = randi(n, 1, n);  sr = stg(sel);
        boot(r,:,1) = sr.rho; boot(r,:,2) = sr.cm; boot(r,:,3) = sr.bud; boot(r,:,4) = sr.beta;
    end
    pl = normcdf(-sig); ph = normcdf(sig);
    ex = @(v) interp1(gi(:), v(:), (1:numel(sP)).', 'linear', 'extrap');
    qq = @(x, p) quantile(x, p, 1).';
    A.rho_lo  = ex(qq(boot(:,:,1), pl)); A.rho_hi  = ex(qq(boot(:,:,1), ph));
    A.cm_lo   = ex(qq(boot(:,:,2), pl)); A.cm_hi   = ex(qq(boot(:,:,2), ph));
    A.bud_lo  = ex(qq(boot(:,:,3), pl)); A.bud_hi  = ex(qq(boot(:,:,3), ph));
    A.beta_lo = ex(qq(boot(:,:,4), pl)); A.beta_hi = ex(qq(boot(:,:,4), ph));
end


function s = local_stats(Ep, Em, Bp, Bm, sP, dP, b0, d, Gc)
    s.rms  = sqrt(mean([Ep Em].^2, 2));
    s.rho  = s.rms ./ sP;
    s.G    = (mean(Bp, 2) - mean(Bm, 2)) / (2 * d);
    s.cm   = (mean(Bp, 2) + mean(Bm, 2)) / 2 - b0;
    Tr     = [Bp Bm] - [Bp(1, :) Bm(1, :)];
    s.bud  = mean(Tr.^2, 2) ./ max(dP, eps);
    s.beta = s.G ./ max(Gc, eps);
end


function [b, hw] = local_beta_from_parent(par_file, arm, k, seed_subset, n_boot, sig)
%LOCAL_BETA_FROM_PARENT  beta and its 3-sigma half-width at one time index, recomputed
%   from the frozen parent per-run data (optionally restricted to the paired seeds).
    S = load(par_file, 'DB', 'res');
    ip = find(strcmp({S.DB.arm}, arm) & [S.DB.sgn] > 0);
    im = find(strcmp({S.DB.arm}, arm) & [S.DB.sgn] < 0);
    [~, o1] = sort([S.DB(ip).seed]); ip = ip(o1);
    [~, o2] = sort([S.DB(im).seed]); im = im(o2);
    if ~isempty(seed_subset)
        keep = ismember([S.DB(ip).seed], seed_subset);
        ip = ip(keep); im = im(keep);
    end
    n = numel(ip);
    Bp = [S.DB(ip).b_hat]; Bm = [S.DB(im).b_hat];
    d  = S.res.main.d;  Gc = S.res.main.G_claim(k);
    f  = @(sel) ((mean(Bp(k, sel), 2) - mean(Bm(k, sel), 2)) / (2 * d)) / Gc;
    b  = f(1:n);
    bs = zeros(n_boot, 1);
    for r = 1:n_boot; bs(r) = f(randi(n, 1, n)); end
    hw = (quantile(bs, normcdf(sig)) - quantile(bs, normcdf(-sig))) / 2;
end


function r = local_iratio(beta, G)
    r = (1 - beta * G) / (beta * (1 - G));
end


function s = local_inband(v, band)
    if v >= band(1) && v <= band(2); s = 'INSIDE the S band'; else; s = 'OUTSIDE the S band'; end
end


function s = local_verdict_shape(resid, drift)
    if resid < drift; s = 'shape EXPLAINED (residual below the drift)';
    else;             s = 'shape NOT explained (residual >= the drift)'; end
end
