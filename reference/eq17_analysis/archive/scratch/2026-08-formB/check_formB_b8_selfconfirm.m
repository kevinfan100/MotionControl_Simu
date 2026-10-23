% STATUS: ACTIVE | B8 adjudication (pre-registered) -- does the post-descent growth of
%   1/P_bb correspond to REAL information, or is the filter confirming itself?
%   EXPIRES: when B8 is adjudicated (a) / (b) and the verdict is recorded in memory.
%   Zero production changes: production code is called only through
%   run_formB_ws(opts) / run_formB_ws(cfg, test_opts), never edited.
%
% ---------------------------------------------------------------------------------------
% THE PROPOSITION (memory project_formB_bonly_prior_reproducibility_2026-08-06, item B8)
% ---------------------------------------------------------------------------------------
%   B5 established: Q_bb = 0 and F_e row 5 is a unit row, so P_bb is exactly monotone
%   non-increasing, and 98 % of b_hat's travel happens in the 1 s descent.  B8 observed
%   that AFTER the descent the declared confidence 1/P_bb keeps doubling while the
%   accuracy does not improve.  Two mutually exclusive readings:
%     (a) luck      -- the late phase really does collect information; N = 2 cannot see it.
%     (b) self-confirmation -- the 1/P growth has no real information behind it.  If (b)
%         holds, every "collect more information" route fails and a forgetting factor /
%         Q_bb > 0 becomes structural, not a knob.
%
% ---------------------------------------------------------------------------------------
% WHY THIS DESIGN MAKES THE NULL EXACT (r = 1 construction)
% ---------------------------------------------------------------------------------------
%   Scalar linear-Gaussian estimation of a static b with prior N(b_0, P0) and cumulative
%   data information I(t).  Write the learning fraction G(t) = 1 - P(t)/P0.  For a FIXED
%   truth b_true with offset d = b_true - b_0,
%       b_hat - b_true = -d(1-G) + G*eps ,   Var(G*eps) = G(1-G)P0
%       MSE(t) = d^2 (1-G)^2 + G P(t)
%   so with r = d^2/P0 the honesty ratio
%       rho(t) = RMS_seeds(b_hat - b_true) / sqrt(P(t))      obeys      rho^2 = 1 + (r-1)(1-G).
%   Setting r = 1 EXACTLY (truth placed one prior width from the seed) gives
%
%       rho(t) == 1 for all t, for ANY amount of learning G.                       (NULL)
%
%   That is the whole point of the construction: the null is G-free, so it cannot be
%   satisfied or broken by "how much" the filter learned -- only by whether the confidence
%   it declares is matched by the error it actually has.  Two consequences used below:
%     * P0-budget form: E[(b_hat - b_hat_0)^2] = P0 - P(t) exactly (the house criterion,
%       saturated -- here budget ratio 1 is the PREDICTION, not an upper bound).
%     * The plant must be IN-FAMILY, otherwise b_true is not defined and rho > 1 cannot be
%       separated from model-form error.  Arms A/B use opts.plant_gain_law (exact Form B
%       truth curve); arm C deliberately does not, to price that very difference.
%
%   Conservativeness: the controller keeps its production shape floor Pf_a_floor, which
%   prices a representation error that is exactly ZERO in arms A/B.  That makes the filter
%   over-cautious, i.e. it pushes rho BELOW 1.  Any rho > 1 found here is therefore an
%   under-statement of the over-confidence.  One-sided in the safe direction.
%
% ---------------------------------------------------------------------------------------
% STEP 1: POWER, WRITTEN BEFORE ANY MODEL WAS RUN (project rule: 先功效後獵捕)
% ---------------------------------------------------------------------------------------
%   Nuisance inputs come from the NULL itself (Var = G(1-G)P0), with G(t) read off a
%   single deterministic P-trajectory pilot (P is seed-independent to <1e-3), so no
%   effect-size choice is informed by the data:
%       G(1.5 s) = 0.550    G(4.8 s) = 0.727    G(20.8 s) = 0.840     (width 0.10 arm)
%   Sampling s.d. of rho with M runs, from Var(RMS) of e ~ N(mu, s^2),
%       mu^2 = (1-G)^2 P0 , s^2 = G(1-G) P0 :
%       sd(rho)/rho = 0.5*sqrt(2 s^4 + 4 mu^2 s^2) / (sqrt(M) (mu^2+s^2))
%                   = 0.680/sqrt(M) at t = 4.8 s ,  0.698/sqrt(M) at t = 20.8 s.
%   EFFECT SIZES PRE-REGISTERED (both are hypothesis-(b) predictions, not data-derived):
%       E1 "post-descent information entirely fake": MSE frozen at its t = 1.5 s value
%          while P keeps contracting  =>  rho(4.8) = sqrt(0.4504/0.2730) = 1.285
%                                         rho(20.8) = sqrt(0.4504/0.1599) = 1.678
%       E2 "the 1.0 -> 1.3 effect named in the brief": rho = 1.30.
%   REQUIRED N (3 sigma), worst case M = N (assume the +/- arms are perfectly correlated
%   through the common random numbers, i.e. the paired design buys nothing):
%       E1 at 20.8 s: 3*0.698/sqrt(N) <= 0.678  ->  N >=  9.5
%       E1 at  4.8 s: 3*0.680/sqrt(N) <= 0.285  ->  N >= 51.2
%       E2 at 20.8 s: 3*0.698/sqrt(N) <= 0.30   ->  N >= 48.7
%   If instead the +/- arms are independent (M = 2N) those become N >= 5 / 26 / 25.
%   CHOSEN N = 30 seeds per sign (M = 30..60).  This is powered at >= 3 sigma for the
%   LONG-arm test under the worst case (5.3 sigma at M = 30), and for the short arm and
%   for E2 only if the paired correlation is weak (2.3 sigma at M = 30, 3.2 sigma at
%   M = 60).  Declared consequence: the RUN-LENGTH test is the confirmatory one; the
%   4.8 s read-out and the E2 threshold are secondary and may return "unresolved".
%   Response statistic S2 is better powered: sd(dG) <= 0.382/sqrt(2N) = 0.049 against a
%   claimed post-descent increment G(20.8)-G(1.5) = 0.290, i.e. 5.9 sigma.
%   Actual CIs below are BOOTSTRAP over the seed (pair) as resampling unit, which measures
%   the paired correlation instead of assuming it.
%
% ---------------------------------------------------------------------------------------
% STEP 3: VERDICT RULES, FIXED BEFORE THE RUN
% ---------------------------------------------------------------------------------------
%   Let CI3 denote a 3-sigma bootstrap interval (seed as unit, 10000 resamples).
%   (a) HONEST      : rho(20.8) <= 1 + CI3 AND rho(20.8) - rho(4.8) <= CI3 (paired)
%                     AND G_meas(20.8) within CI3 of G_claim(20.8)
%                     AND |common-mode drift| within CI3 of 0.
%                     The first two are ONE-SIDED on purpose: rho below 1 is the
%                     conservative direction the retained shape floor predicts.
%   (b) SELF-CONFIRM: rho(20.8) > 1 + CI3 AND rho(20.8) > rho(4.8) + CI3 (paired,
%                     i.e. it gets worse with run length)
%                     AND/OR G_meas(20.8) - G_meas(1.5) < G_claim(20.8) - G_claim(1.5)
%                     by more than CI3.
%   UNRESOLVED      : anything else -- in particular rho > 1 with NO run-length growth
%                     (that is a fixed model-form/bias offset, not self-confirmation),
%                     or all effects inside CI3.  A third verdict is allowed and will be
%                     reported as such rather than forced into (a) or (b).
%   Reported separately in every case: the NUMERATOR (RMS error) and DENOMINATOR
%   (sqrt P) of rho as functions of time, so "P shrank" and "error grew" are never
%   conflated.
%
% ---------------------------------------------------------------------------------------
% ARMS
% ---------------------------------------------------------------------------------------
%   All arms: tier t1 (w_s locked at the TRUE wall), p locked at the TRUE exponent 1,
%   b the single free parameter, par_law default.  Run length T_sim = 20.8 s = the
%   canonical 4.8 s scenario plus 16 s of extra trough hold; phases 1-3 are untouched and
%   the extended run's first 4.8 s is BIT-IDENTICAL to the canonical run (verified,
%   max|db_hat| = 0.000e+00).  The brief's arm A (short) and arm B (long) are therefore
%   read off the SAME runs at t = 4.8 s and t = 20.8 s -- exact pairing, half the cost.
%     A/B main      width 0.10, plant Form B b_true = 9/8 +- 0.10  (r = 1), 30 seeds/sign
%     A/B invariance width 0.25, plant Form B b_true = 9/8 +- 0.25 (r = 1), 15 seeds/sign
%     C  plane      width 0.10, plant = published plane curve, b_ref = minimax-in-a_bar
%                   fit on the same envelope; the b_eff spread over the envelope is
%                   reported as the irreducible ambiguity band.
%   DEVIATION FROM THE BRIEF, with reason: the brief specifies the PRODUCTION prior
%   (sqrt P0 = 0.0157) for arm A.  At that width the pilot P-trajectory contracts by only
%   6.2 % over the whole run (P/P0: 0.971 -> 0.938), i.e. the B8 phenomenon -- 1/P
%   doubling after the descent -- DOES NOT EXIST at production width; there is nothing to
%   adjudicate.  The phenomenon B8 recorded (1/P 146 -> 273) lives in the WIDENED-prior
%   arm.  Width 0.10 is the smallest width that reproduces it (1/P x1.65 post-descent,
%   x1.71 more over the extra hold) while keeping b in a linear, physical range; width
%   0.25 repeats the test at x1.96 for the invariance check demanded by
%   .claude/rules/derivation-workflow.md rule 2.  The production width is reported as
%   context from its deterministic P-trajectory, no seeds needed.
%
%   ARM D, ADDED AFTER THE CONFIRMATORY RUN (declared as post-hoc, with its own
%   discriminator written before it was run).  The confirmatory arms tie the truth offset
%   d to the prior width (that is what enforces r = 1), so a response deficit measured by
%   the finite difference [E b_hat(+d) - E b_hat(-d)]/(2d) has TWO possible sources that
%   the design cannot separate: (i) a genuine over-claim, size independent of d, and
%   (ii) curvature of the mean response m(b_true), which contaminates the finite
%   difference at O(d^2).  Arm D breaks the tie: SAME prior width 0.10, d = 0.25 instead
%   of 0.10 (so r = 6.25, and rho's null moves to sqrt(1 + 5.25(1-G))).  Pre-registered
%   discriminator: a curvature artefact must grow by (0.25/0.10)^2 = 6.25x, a structural
%   over-claim must stay the same size.  The two predictions differ by 6.25x, so 12
%   seeds/sign resolves them; no effect-size hunting is involved.
%   Third statistic added at the same time, on the ALREADY-COLLECTED data, with no
%   finite difference in it at all: the P[0]-traversal budget
%       budget(t) = mean_runs (b_hat - b_hat_0)^2 / (P0 - P(t)) ,  null = G r + (1 - G)
%   (= 1 when r = 1).  It is the house criterion, here in its saturated form, and it is
%   immune to the curvature confound because it never differentiates anything.
%
%   RESUMABLE: MAX_RUNS_PER_CALL jobs per invocation, progress saved after every run.
%   Re-run until it prints ALL JOBS DONE, then it prints the statistics.
%   Outputs: test_results/temp_formB_b8_selfconfirm.mat (gitignored).

clear cd    % MATLAB cd-shadowing hang, recorded in memory

here = fileparts(mfilename('fullpath'));
proj = fullfile(here, '..', '..');
addpath(genpath(fullfile(proj, 'model')));
addpath(fullfile(proj, 'test_script', 'integration'));

% ---------------- design constants (no magic numbers: each is derived or declared) -----
AX_Z      = 3;                      % wall-normal axis
B_ANCHOR  = 9/8;                    % far-field reflection coefficient = the b seed
P_TRUE    = 1;                      % far-field power = the locked exponent
WS_TRUE   = 1;                      % true wall position in law coordinates (tier t1)
T_LONG    = 20.8;                   % [s] canonical 4.8 s + 16 s extra trough hold
T_CANON   = 4.8;                    % [s] canonical end = the brief's arm A read-out
T_REF     = 1.5;                    % [s] end of descent = where B5 says learning is done
DEC       = 16;                     % log decimation 1600 Hz -> 100 Hz
W_MAIN    = 0.10;                   % main arm prior width (see ARMS note above)
W_INV     = 0.25;                   % invariance arm prior width
D_FAR     = 0.25;                   % arm D truth offset at prior width W_MAIN (r = 6.25)
N_MAIN    = 30;                     % seeds per sign, main arm (power calc above)
N_INV     = 15;                     % seeds per sign, invariance arm (secondary)
N_PLANE   = 30;                     % seeds, plane arm
N_FAR     = 12;                     % seeds per sign, arm D (6.25x separation, see header)
SEED0_MAIN  = 81000;                % fresh seed blocks (house seeds 1:20 are burned:
SEED0_INV   = 82000;                % the "y2 frozen bias" ghost lived there)
SEED0_PLANE = 83000;
SEED0_FAR   = 84000;
MAX_RUNS_PER_CALL = 24;             % ~7.7 min/call at 19.2 s per 20.8 s run; progress is
                                    % saved after EVERY run, so a timeout costs one run
N_BOOT    = 10000;                  % bootstrap resamples
BOOT_SEED = 20260810;               % fixed so the CIs are reproducible
SIG3      = 3;                      % sigma multiple for every pre-registered threshold

db_file = fullfile(proj, 'test_results', 'temp_formB_b8_selfconfirm.mat');

% ---------------- job list (deterministic; key = arm|sign|seed) ------------------------
jobs = struct('key', {}, 'arm', {}, 'sgn', {}, 'seed', {}, 'width', {}, 'b_true', {});
for s = [1 -1]
    for i = 1:N_MAIN
        jobs(end+1) = local_job('main', s, SEED0_MAIN + i, W_MAIN, B_ANCHOR + s * W_MAIN); %#ok<SAGROW>
    end
end
for s = [1 -1]
    for i = 1:N_INV
        jobs(end+1) = local_job('inv', s, SEED0_INV + i, W_INV, B_ANCHOR + s * W_INV); %#ok<SAGROW>
    end
end
for i = 1:N_PLANE
    jobs(end+1) = local_job('plane', 0, SEED0_PLANE + i, W_MAIN, NaN); %#ok<SAGROW>
end
for s = [1 -1]
    for i = 1:N_FAR
        jobs(end+1) = local_job('dfar', s, SEED0_FAR + i, W_MAIN, B_ANCHOR + s * D_FAR); %#ok<SAGROW>
    end
end

% ---------------- resume ---------------------------------------------------------------
if exist(db_file, 'file')
    S = load(db_file);  DB = S.DB;
    if isfield(S, 'HS'); HS = S.HS; else; HS = struct(); end
else
    DB = struct('key', {}, 'arm', {}, 'sgn', {}, 'seed', {}, 'width', {}, 'b_true', {}, ...
                't', {}, 'b_hat', {}, 'sP_b', {}, 'a_hat', {}, 'a_true', {}, ...
                'hb_lo', {}, 'hb_hi', {}, 'nan_flag', {});
    HS = struct();
end
done_keys = {DB.key};
todo = find(~ismember({jobs.key}, done_keys));
fprintf('B8 self-confirmation adjudication: %d/%d jobs done, %d remaining\n', ...
        numel(DB), numel(jobs), numel(todo));

% ---------------- per-arm production handshake ----------------------------------------
% One arm-form call per (width, plant) configuration gives the PRODUCTION cfg and
% ctrl_const; the remaining seeds go through the test-ladder form, which is the same
% local_run_once with no .mat write and no console spam.  Bit-identity of the two paths
% is asserted on the handshake seed itself, so this cannot silently drift.  The handshake
% is cached in the .mat so a resumed call does not repeat it.
n_this_call = 0;
for j = todo
    if n_this_call >= MAX_RUNS_PER_CALL; break; end
    job = jobs(j);
    ck  = matlab.lang.makeValidName(sprintf('%s_%g', job.arm, job.width));
    if ~isfield(HS, ck)
        HS.(ck) = local_handshake(job, P_TRUE, WS_TRUE, T_LONG, AX_Z);
    end
    H = HS.(ck);
    if strcmp(job.arm, 'plane')
        pc_handle = [];
    else
        pc_handle = @(hb) calc_formB_cperp(hb, ...
                        struct('b', job.b_true, 'p', P_TRUE, 'ws', WS_TRUE));
    end

    topts = struct('seed', job.seed, 'ctrl_const_override', H.ctrl_const, ...
                   'verbose', false, 'a_ctrl_override', [], 'log_P_full', false, ...
                   'ws_inject', 0, 'plant_cperp', pc_handle);
    r = run_formB_ws(H.cfg, topts);

    idx = 1:DEC:numel(r.tout);
    rec = job;
    rec.t        = r.tout(idx);
    rec.b_hat    = r.b_hat_out(idx, AX_Z);
    rec.sP_b     = r.P_b_out(idx, AX_Z);          % driver stores SQRT of the variance
    rec.a_hat    = r.a_hat_out(idx, AX_Z);
    rec.a_true   = r.a_true_out(idx, AX_Z);
    rec.hb_lo    = min(r.h_bar_true_out);
    rec.hb_hi    = max(r.h_bar_true_out);
    rec.nan_flag = any(~isfinite(r.b_hat_out(:, AX_Z)));
    DB(end+1) = rec; %#ok<SAGROW>
    save(db_file, 'DB', 'HS', '-v7.3');
    n_this_call = n_this_call + 1;
    fprintf('  [%3d/%3d] %-28s b_true %7.4f  b_hat_end %7.4f  sqrtP_end %7.4f\n', ...
            numel(DB), numel(jobs), job.key, job.b_true, rec.b_hat(end), rec.sP_b(end));
end

if numel(DB) < numel(jobs)
    fprintf('PARTIAL: %d/%d done. Re-run this script to continue.\n', numel(DB), numel(jobs));
    return;
end
fprintf('ALL JOBS DONE (%d runs).\n\n', numel(DB));

% =======================================================================================
%  ANALYSIS
% =======================================================================================
t   = DB(1).t;
kR  = find(t >= T_REF   - 1e-9, 1);
kA  = find(t >= T_CANON - 1e-9, 1);
kB  = numel(t);
rng(BOOT_SEED);

% --- plane-arm reference: minimax-in-a_bar b on the SAME envelope the priors use -------
pcst  = physical_constants();
cfg0  = local_canon_cfg_probe(proj);
env_lo = cfg0.h_bottom / pcst.R - 0.1;      % ENV_LO_MARGIN, driver value
env_hi = cfg0.h_init   / pcst.R + 1.0;      % ENV_HI_MARGIN, driver value
[b_ref, sup_ref, b_eff_lo, b_eff_hi] = local_plane_reference(env_lo, env_hi, B_ANCHOR, P_TRUE);
fprintf(['PLANE REFERENCE on w_bar in [%.3f %.3f]: minimax b = %.5f (sup|da| = %.5f); ', ...
         'b_eff band [%.4f %.4f] = the irreducible ambiguity (%.2f prior widths at %.2f)\n\n'], ...
        env_lo, env_hi, b_ref, sup_ref, b_eff_lo, b_eff_hi, ...
        (b_eff_hi - b_eff_lo) / W_MAIN, W_MAIN);

res = struct();
res.main  = local_analyse_pm(DB, 'main',  B_ANCHOR, W_MAIN, W_MAIN, N_BOOT, SIG3);
res.inv   = local_analyse_pm(DB, 'inv',   B_ANCHOR, W_INV,  W_INV,  N_BOOT, SIG3);
res.dfar  = local_analyse_pm(DB, 'dfar',  B_ANCHOR, W_MAIN, D_FAR,  N_BOOT, SIG3);
res.plane = local_analyse_plane(DB, 'plane', B_ANCHOR, W_MAIN, b_ref, N_BOOT, SIG3);
res.t = t; res.kR = kR; res.kA = kA; res.kB = kB;
res.b_ref = b_ref; res.b_eff_band = [b_eff_lo b_eff_hi]; res.sup_ref = sup_ref;
res.T = [T_REF T_CANON T_LONG];

for nm = {'main', 'inv', 'dfar', 'plane'}
    A = res.(nm{1});
    fprintf('=== ARM %s (prior width %.4f, %d runs) ===\n', upper(nm{1}), A.width, A.n_runs);
    fprintf('%8s | %8s %8s %8s | %9s %9s | %9s %9s %8s %8s | %9s %9s %9s\n', 't[s]', ...
            'rho', 'rho_lo', 'rho_hi', 'RMSerr', 'sqrtP', 'G_claim', 'G_meas', ...
            'G_lo', 'G_hi', 'commonmd', 'cm_lo', 'cm_hi');
    for k = [kR kA kB]
        if isfield(A, 'G_lo'); glo = A.G_lo(k); ghi = A.G_hi(k); else; glo = NaN; ghi = NaN; end
        fprintf('%8.2f | %8.3f %8.3f %8.3f | %9.5f %9.5f | %9.4f %9.4f %8.4f %8.4f | %+9.5f %+9.5f %+9.5f\n', ...
                t(k), A.rho(k), A.rho_lo(k), A.rho_hi(k), A.rms(k), A.sP(k), ...
                A.G_claim(k), A.G_meas(k), glo, ghi, A.cm(k), A.cm_lo(k), A.cm_hi(k));
    end
    if isfield(A, 'bud')
        fprintf('  traversal budget (no finite difference) at the three times: %.3f / %.3f / %.3f  vs null %.3f / %.3f / %.3f\n', ...
                A.bud(kR), A.bud(kA), A.bud(kB), A.bud_hon(kR), A.bud_hon(kA), A.bud_hon(kB));
    end
    fprintf('  honest prediction rho_hon: %.3f / %.3f / %.3f   (r = %.4f)\n', ...
            A.rho_hon(kR), A.rho_hon(kA), A.rho_hon(kB), A.r);
    fprintf('  strong-(b) prediction (MSE frozen at t_ref): rho = %.3f (t=%.1f) / %.3f (t=%.1f)\n', ...
            A.rho_frozen(kA), t(kA), A.rho_frozen(kB), t(kB));
    fprintf('  paired growth rho(%.1f)-rho(%.1f) = %+.4f  [%+.4f %+.4f] (3-sigma boot)\n', ...
            t(kB), t(kA), A.drho, A.drho_lo, A.drho_hi);
    if isfield(A, 'dG_meas')
        fprintf('  truth offset d = %.4f  (r = d^2/P0 = %.3f)\n', A.d, A.r);
        fprintf('  response LEVEL at t_end: G_meas %.4f [%.4f %.4f] vs G_claim %.4f -> deficit %+.5f (%.2f %% of 1-G)\n', ...
                A.G_meas(kB), A.G_lo(kB), A.G_hi(kB), A.G_claim(kB), ...
                A.G_meas(kB) - A.G_claim(kB), ...
                100 * (A.G_meas(kB) - A.G_claim(kB)) / (1 - A.G_claim(kB)));
        fprintf('  post-descent response: dG_meas %+.4f [%+.4f %+.4f] vs dG_claim %+.4f  -> deficit %+.4f\n', ...
                A.dG_meas, A.dG_meas_lo, A.dG_meas_hi, A.dG_claim, A.dG_meas - A.dG_claim);
        fprintf('  information ratio dI_claim/dI_real over [%.1f, %.1f] s: %.3f [%.3f %.3f]\n', ...
                t(kR), t(kB), A.Iratio, A.Iratio_lo, A.Iratio_hi);
        fprintf('  traversal budget (no finite difference): %.3f [%.3f %.3f] vs null %.3f  (t = %.1f s)\n', ...
                A.bud(kB), A.bud_lo(kB), A.bud_hi(kB), A.bud_hon(kB), t(kB));
    end
    fprintf('  gain error e_a [%%] at t = %.1f / %.1f s: %+.3f / %+.3f\n', ...
            t(kA), t(kB), A.ea(kA), A.ea(kB));
    fprintf('  h_bar range over runs: [%.3f %.3f]   NaN runs: %d\n\n', ...
            A.hb_lo, A.hb_hi, A.n_nan);
end

% --- context: the production width has no phenomenon to adjudicate --------------------
fprintf('CONTEXT (deterministic P-trajectory, no seeds): production width 0.0157 ->\n');
fprintf('  see header ARMS note; P/P0 0.971 (1.5 s) -> 0.938 (4.8 s), i.e. 1/P +3.5 %%.\n\n');

% --- pre-registered verdict ------------------------------------------------------------
% One-sided where the pre-registration is one-sided ("rho(20.8) <= 1 + CI3", "rho(20.8)
% - rho(4.8) <= CI3"): rho BELOW the null is the conservative direction the header
% predicts from the retained production shape floor, and is not evidence against (a).
A = res.main;
c_rho_hi   = A.rho_lo(kB) > A.rho_hon(kB);        % whole 3-sigma CI above the null (=1)
c_growth   = A.drho_lo > 0;                       % grows with run length
c_resp_def = A.dG_meas_hi < A.dG_claim;           % post-descent response deficit, 3 sigma
c_rho_ok   = ~c_rho_hi;
c_nogrowth = ~c_growth;
c_resp_ok  = (A.G_lo(kB) <= A.G_claim(kB)) && (A.G_hi(kB) >= A.G_claim(kB));
c_cm_ok    = (A.cm_lo(kB) <= 0) && (A.cm_hi(kB) >= 0);

fprintf('=== PRE-REGISTERED VERDICT (main arm) ===\n');
fprintf('  response level at t_end: G_meas %.4f [%.4f %.4f] vs G_claim %.4f\n', ...
        A.G_meas(kB), A.G_lo(kB), A.G_hi(kB), A.G_claim(kB));
fprintf('  common mode at t_end:  %+.5f [%+.5f %+.5f]\n', A.cm(kB), A.cm_lo(kB), A.cm_hi(kB));
fprintf('  (b) rho(long) > null + 3sd ... %d\n', c_rho_hi);
fprintf('  (b) rho grows with run length  %d\n', c_growth);
fprintf('  (b) response deficit > 3sd ... %d\n', c_resp_def);
fprintf('  (a) rho(long) not above null   %d\n', c_rho_ok);
fprintf('  (a) no run-length growth ..... %d\n', c_nogrowth);
fprintf('  (a) response matches claim ... %d\n', c_resp_ok);
fprintf('  (a) common mode consistent 0   %d\n', c_cm_ok);
if c_rho_ok && c_nogrowth && c_resp_ok && c_cm_ok
    verdict = '(a) HONEST';
elseif (c_rho_hi && c_growth) || c_resp_def
    verdict = '(b) SELF-CONFIRMATION';
else
    verdict = 'UNRESOLVED';
end
fprintf('  VERDICT: %s\n', verdict);
res.verdict = verdict;

% --- arm D: is the response deficit curvature of the finite difference, or structural? --
def_main = res.main.G_meas(kB) - res.main.G_claim(kB);
def_far  = res.dfar.G_meas(kB) - res.dfar.G_claim(kB);
sd_far   = (res.dfar.G_hi(kB) - res.dfar.G_lo(kB)) / (2 * SIG3);
pred_curv = def_main * (D_FAR / W_MAIN)^2;
fprintf('\n=== ARM D DISCRIMINATOR (post-hoc arm, discriminator fixed before it ran) ===\n');
fprintf('  response deficit at t_end:  d = %.2f -> %+.5f   |   d = %.2f -> %+.5f (1 sd %.5f)\n', ...
        W_MAIN, def_main, D_FAR, def_far, sd_far);
fprintf('  curvature prediction (x%.2f): %+.5f   structural prediction: %+.5f\n', ...
        (D_FAR / W_MAIN)^2, pred_curv, def_main);
fprintf('  observed is %.1f sd from curvature, %.1f sd from structural\n', ...
        abs(def_far - pred_curv) / sd_far, abs(def_far - def_main) / sd_far);
if abs(def_far - def_main) < abs(def_far - pred_curv)
    res.deficit_source = 'STRUCTURAL (d-independent)';
else
    res.deficit_source = 'CURVATURE of the finite difference (d^2)';
end
fprintf('  -> %s\n', res.deficit_source);
% Two-point decomposition deficit(d) = A_struct + B*d^2 (exact fit to the two arms;
% reported as a decomposition, not as a tested model).
Bq = (def_far - def_main) / (D_FAR^2 - W_MAIN^2);
A_struct = def_main - Bq * W_MAIN^2;
fprintf('  two-point split at d = %.2f: structural %+.5f (%.0f %%) + curvature %+.5f (%.0f %%)\n', ...
        W_MAIN, A_struct, 100 * A_struct / def_main, Bq * W_MAIN^2, ...
        100 * Bq * W_MAIN^2 / def_main);

% --- forgetting-factor anchor implied by the measured excess (deliverable 4) -----------
Ts_ctrl = 1 / 1600;                               % [s] control period, house value
Ic_R = res.main.I_claim(kR); Ic_B = res.main.I_claim(kB);
Ir_R = res.main.I_real(kR);  Ir_B = res.main.I_real(kB);
f_excess = 1 - (Ir_B - Ir_R) / (Ic_B - Ic_R);     % fraction of claimed info not supported
I_bar    = (Ic_R + Ic_B) / 2;
n_steps  = (t(kB) - t(kR)) / Ts_ctrl;
lam = 1 - f_excess * (Ic_B - Ic_R) / (I_bar * n_steps);
fprintf('\n=== FORGETTING-FACTOR ANCHOR (main arm, window [%.1f %.1f] s) ===\n', t(kR), t(kB));
fprintf('  claimed info increment %.3f / P0, response-supported %.3f / P0 -> excess %.1f %%\n', ...
        (Ic_B - Ic_R) * W_MAIN^2, (Ir_B - Ir_R) * W_MAIN^2, 100 * f_excess);
fprintf('  per-step lambda that would discard exactly that excess: %.8f (tau = %.1f s)\n', ...
        lam, Ts_ctrl / (1 - lam));
fprintf('  NOTE the measured real information is still RISING at t_end (G_meas %.4f -> %.4f),\n', ...
        res.main.G_meas(kA), res.main.G_meas(kB));
fprintf('  so a saturating forgetting factor is the wrong structure for this residual.\n');
res.lambda_anchor = lam; res.f_excess = f_excess;

save(db_file, 'DB', 'HS', 'res', '-v7.3');
fprintf('\nsaved: %s\n', db_file);


%% =================== local helpers ===================

function j = local_job(arm, sgn, seed, width, b_true)
    j = struct('key', sprintf('%s|%+d|%d', arm, sgn, seed), 'arm', arm, 'sgn', sgn, ...
               'seed', seed, 'width', width, 'b_true', b_true);
end


function H = local_handshake(job, p_true, ws_true, T_long, ax)
%LOCAL_HANDSHAKE  One production arm-form call -> cfg + ctrl_const, plus a bit-identity
%   assertion that the cheap test-ladder path reproduces it exactly.
    o = struct();
    o.seeds = job.seed;
    o.config_override     = struct('T_sim', T_long);
    o.ctrl_const_override = struct('lock_b', false, 'lock_p', true, 'lock_ws', true, ...
                                   'Pf_b_std', job.width);
    if ~strcmp(job.arm, 'plane')
        o.plant_gain_law = struct('b', job.b_true, 'p', p_true, 'ws', ws_true);
    end
    [~, out] = evalc('run_formB_ws(o)');

    H = struct();
    H.cfg        = out.cfg;
    H.ctrl_const = out.runs{1}.ctrl_const;
    if strcmp(job.arm, 'plane')
        pc_handle = [];
    else
        pc_handle = @(hb) calc_formB_cperp(hb, ...
                        struct('b', job.b_true, 'p', p_true, 'ws', ws_true));
    end
    % production path vs cheap path, same seed: must be bit-identical
    topts = struct('seed', job.seed, 'ctrl_const_override', H.ctrl_const, ...
                   'plant_cperp', pc_handle);
    chk = run_formB_ws(H.cfg, topts);
    d = max(abs(chk.b_hat_out(:, ax) - out.runs{1}.b_hat_out(:, ax)));
    assert(d == 0, 'check_formB_b8:pathDrift', ...
           'test-ladder path differs from the arm path by %.3e -- plumbing drifted.', d);
    assert(abs(out.runs{1}.P_b_out(1, ax) - job.width) < 1e-12, 'check_formB_b8:priorWidth', ...
           'controller reported sqrt(P_bb)[0] = %.6f, expected %.6f.', ...
           out.runs{1}.P_b_out(1, ax), job.width);
    fprintf('  handshake %s: prior sqrt(P_bb)[0] = %.4f, paths bit-identical\n', ...
            job.arm, out.runs{1}.P_b_out(1, ax));
end


function cfg = local_canon_cfg_probe(~)
%LOCAL_CANON_CFG_PROBE  The canonical scenario heights, taken from the driver defaults so
%   the envelope used for the plane reference is the one the priors are derived on.
    cfg = struct('h_init', 50, 'h_bottom', 4.5);   % run_formB_ws.local_canonical_config
end


function [b_mm, sup_mm, b_eff_lo, b_eff_hi] = local_plane_reference(w_lo, w_hi, b0, p0)
%LOCAL_PLANE_REFERENCE  Minimax-in-a_bar single-parameter fit of the anchored Form B law
%   to the published plane c_perp, plus the level read-off band b_eff = (c-1)(w-1) which
%   is the set of locally-true b values -- the irreducible ambiguity of arm C.
    n = 20001;
    w = linspace(w_lo, w_hi, n).';
    c = zeros(n, 1);
    for i = 1:n
        [~, c(i)] = calc_correction_functions(w(i), true);
    end
    a_true = 1 ./ c;
    supf = @(b) max(abs((1 - (1 + (w - 1) / b).^(-p0)) - a_true));
    b_mm  = fminbnd(supf, 0.5 * b0, 2 * b0, optimset('TolX', 1e-10));
    sup_mm = supf(b_mm);
    b_eff = (c - 1) .* (w - 1);
    b_eff_lo = min(b_eff);
    b_eff_hi = max(b_eff);
end


function A = local_analyse_pm(DB, arm, b0, width, d, n_boot, sig)
%LOCAL_ANALYSE_PM  Paired +/- arm: honesty ratio, response fraction, common mode, budget.
%   d is the TRUTH OFFSET (b_true = b0 +- d) and width is sqrt(P0); r = d^2/P0.  The
%   confirmatory arms have d = width (r = 1); arm D unties them.
    ip = find(strcmp({DB.arm}, arm) & [DB.sgn] > 0);
    im = find(strcmp({DB.arm}, arm) & [DB.sgn] < 0);
    [~, op] = sort([DB(ip).seed]);  ip = ip(op);
    [~, om] = sort([DB(im).seed]);  im = im(om);
    assert(numel(ip) == numel(im), 'check_formB_b8:unpaired', 'arm %s is unpaired.', arm);
    n = numel(ip);

    Bp = [DB(ip).b_hat];  Bm = [DB(im).b_hat];        % [T x n]
    Pp = [DB(ip).sP_b];   Pm = [DB(im).sP_b];
    Ep = Bp - (b0 + d);   Em = Bm - (b0 - d);
    A = local_common(DB, [ip im], arm, width, n);
    A.d = d;
    A.r = d^2 / width^2;

    sP = mean([Pp Pm], 2);
    A.sP      = sP;
    A.G_claim = 1 - (sP.^2) / width^2;
    A.rho_hon = sqrt(max(1 + (A.r - 1) * (1 - A.G_claim), 0));
    A.bud_hon = A.G_claim * A.r + (1 - A.G_claim);     % null of the traversal budget

    dP = width^2 - sP.^2;                              % P0 - P(t)
    stat = @(sel) local_stats_pm(Ep(:, sel), Em(:, sel), Bp(:, sel), Bm(:, sel), sP, dP, b0, d);
    s0 = stat(1:n);
    A.rho = s0.rho;  A.rms = s0.rms;  A.G_meas = s0.G;  A.cm = s0.cm;  A.bud = s0.bud;

    % Bootstrap on a coarse time grid (the full 100 Hz grid x 10^4 resamples would be
    % 0.5 GB); the three pre-registered landmarks are forced into the grid, so every
    % number quoted in the verdict is an exact bootstrap value, never an interpolant.
    kR = A.kR; kA = A.kA; kB = A.kB;
    gi = unique([1:local_boot_stride():numel(sP), kR, kA, kB]);
    gR = find(gi == kR); gA = find(gi == kA); gB = find(gi == kB);
    statg = @(sel) local_stats_pm(Ep(gi, sel), Em(gi, sel), Bp(gi, sel), Bm(gi, sel), ...
                                  sP(gi), dP(gi), b0, d);
    boot = zeros(n_boot, numel(gi), 4);   % rho | G | cm | budget
    for r = 1:n_boot
        sel = randi(n, 1, n);
        sr  = statg(sel);
        boot(r, :, 1) = sr.rho;  boot(r, :, 2) = sr.G;
        boot(r, :, 3) = sr.cm;   boot(r, :, 4) = sr.bud;
    end
    pl = normcdf(-sig);  ph = normcdf(sig);
    ex = @(v) interp1(gi(:), v(:), (1:numel(sP)).', 'linear', 'extrap');
    qq = @(x, p) quantile(x, p, 1).';
    A.rho_lo = ex(qq(boot(:, :, 1), pl));  A.rho_hi = ex(qq(boot(:, :, 1), ph));
    A.G_lo   = ex(qq(boot(:, :, 2), pl));  A.G_hi   = ex(qq(boot(:, :, 2), ph));
    A.cm_lo  = ex(qq(boot(:, :, 3), pl));  A.cm_hi  = ex(qq(boot(:, :, 3), ph));
    A.bud_lo = ex(qq(boot(:, :, 4), pl));  A.bud_hi = ex(qq(boot(:, :, 4), ph));
    A.boot_grid = gi;

    % pre-registered scalars: run-length growth and post-descent response
    A.rho_frozen = A.rms(kR) ./ sP;                 % strong-(b): MSE frozen at t_ref
    A.drho    = A.rho(kB) - A.rho(kA);
    A.dG_meas = A.G_meas(kB) - A.G_meas(kR);
    A.dG_claim = A.G_claim(kB) - A.G_claim(kR);
    dr = boot(:, gB, 1) - boot(:, gA, 1);
    dg = boot(:, gB, 2) - boot(:, gR, 2);
    A.drho_lo = quantile(dr, pl);  A.drho_hi = quantile(dr, ph);
    A.dG_meas_lo = quantile(dg, pl);  A.dG_meas_hi = quantile(dg, ph);

    % information accounting: claimed 1/P growth vs the growth the response supports
    Ic = 1 ./ sP.^2 - 1 / width^2;
    Ir = @(G) G ./ max(1 - G, eps) / width^2;
    A.Iratio = (Ic(kB) - Ic(kR)) / (Ir(A.G_meas(kB)) - Ir(A.G_meas(kR)));
    bir = (Ic(kB) - Ic(kR)) ./ (Ir(boot(:, gB, 2)) - Ir(boot(:, gR, 2)));
    A.Iratio_lo = quantile(bir, pl);  A.Iratio_hi = quantile(bir, ph);
    A.I_claim = Ic;  A.I_real = Ir(A.G_meas);
end


function s = local_boot_stride()
%LOCAL_BOOT_STRIDE  Bootstrap time-grid stride on the 100 Hz decimated log (5 Hz bands).
    s = 20;
end


function s = local_stats_pm(Ep, Em, Bp, Bm, sP, dP, b0, d)
    s.rms = sqrt(mean([Ep Em].^2, 2));
    s.rho = s.rms ./ sP;
    s.G   = (mean(Bp, 2) - mean(Bm, 2)) / (2 * d);
    s.cm  = (mean(Bp, 2) + mean(Bm, 2)) / 2 - b0;
    Tr    = [Bp Bm] - [Bp(1, :) Bm(1, :)];             % traversal from the seed
    s.bud = mean(Tr.^2, 2) ./ max(dP, eps);
end


function A = local_analyse_plane(DB, arm, b0, width, b_ref, n_boot, sig)
%LOCAL_ANALYSE_PLANE  Plane-wall arm: the truth sits essentially AT the prior mean, so
%   the honest prediction is rho_hon = sqrt(1 + (r-1)(1-G)) with r = (b0-b_ref)^2/P0 << 1.
    ii = find(strcmp({DB.arm}, arm));
    [~, oo] = sort([DB(ii).seed]);  ii = ii(oo);
    n = numel(ii);
    B = [DB(ii).b_hat];  P = [DB(ii).sP_b];
    E = B - b_ref;
    A = local_common(DB, ii, arm, width, n);
    sP = mean(P, 2);
    A.sP = sP;
    A.G_claim = 1 - (sP.^2) / width^2;
    A.r = (b0 - b_ref)^2 / width^2;
    A.rho_hon = sqrt(max(1 + (A.r - 1) * (1 - A.G_claim), 0));

    A.rms = sqrt(mean(E.^2, 2));
    A.rho = A.rms ./ sP;
    A.cm  = mean(B, 2) - b_ref;          % drift away from the best in-family value
    A.G_meas = nan(size(sP));            % no +/- pair: response fraction not identified

    kA = A.kA; kB = A.kB; kR = A.kR;
    gi = unique([1:local_boot_stride():numel(sP), kR, kA, kB]);
    gA = find(gi == kA); gB = find(gi == kB);
    boot = zeros(n_boot, numel(gi), 2);
    for r = 1:n_boot
        sel = randi(n, 1, n);
        boot(r, :, 1) = (sqrt(mean(E(gi, sel).^2, 2)) ./ sP(gi)).';
        boot(r, :, 2) = (mean(B(gi, sel), 2) - b_ref).';
    end
    pl = normcdf(-sig);  ph = normcdf(sig);
    ex = @(v) interp1(gi(:), v(:), (1:numel(sP)).', 'linear', 'extrap');
    qq = @(x, p) quantile(x, p, 1).';
    A.rho_lo = ex(qq(boot(:, :, 1), pl));  A.rho_hi = ex(qq(boot(:, :, 1), ph));
    A.cm_lo  = ex(qq(boot(:, :, 2), pl));  A.cm_hi  = ex(qq(boot(:, :, 2), ph));
    A.boot_grid = gi;
    A.rho_frozen = A.rms(kR) ./ sP;
    A.drho = A.rho(kB) - A.rho(kA);
    dr = boot(:, gB, 1) - boot(:, gA, 1);
    A.drho_lo = quantile(dr, pl);  A.drho_hi = quantile(dr, ph);
end


function A = local_common(DB, idx, arm, width, n_pairs)
%LOCAL_COMMON  Shared bookkeeping: time index landmarks, gain error, health flags.
    T_REF_L = 1.5; T_CANON_L = 4.8;
    t = DB(idx(1)).t;
    A = struct();
    A.arm = arm; A.width = width; A.n_runs = numel(idx); A.n_pairs = n_pairs;
    A.t = t;
    A.kR = find(t >= T_REF_L - 1e-9, 1);
    A.kA = find(t >= T_CANON_L - 1e-9, 1);
    A.kB = numel(t);
    AH = [DB(idx).a_hat];  AT = [DB(idx).a_true];
    A.ea = 100 * mean((AH - AT) ./ AT, 2);
    A.hb_lo = min([DB(idx).hb_lo]);
    A.hb_hi = max([DB(idx).hb_hi]);
    A.n_nan = sum([DB(idx).nan_flag]);
end
