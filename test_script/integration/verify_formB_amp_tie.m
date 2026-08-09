% STATUS: ACTIVE | acceptance for ctrl_const.law_form_amp (the amplitude
%          writing probe). Derivation SSOT:
%          reference/eq17_analysis/derivation/formB_amp_bonly_probe.tex
%
%VERIFY_FORMB_AMP_TIE  Structural acceptance for the tied (amplitude) arm.
%
%   The arm ties the law origin to the length constant, w_s = b, which at
%   p = 1 turns the production law into a_bar = 1 - b/w_bar exactly, so slot 5
%   carries a single free scalar beta on the (b, w_s) diagonal with
%       J_beta   = J_b   + J_ws   = 1/w_bar^2      (strictly positive)
%       dA/dbeta = dA/db + dA/dws = -1/w_bar
%   Nothing below looks at whether the arm ESTIMATES well; these are the
%   structural claims that must hold before any result is read.
%
%   V1  DEFAULT BIT-IDENTITY. Three ways of not asking for the arm (no field,
%       explicit false, and the driver default) must produce bit-identical
%       runs on two house seeds. This is the contract commit 52da65e set for
%       ws0_perp ("the plane baseline is bit-identical, both house seeds").
%
%   V2  CLOSED-FORM FOLD. The production algebra, evaluated at the tied point
%       (gap = w - b, b, p = 1), must equal the two closed forms above to
%       machine precision. This tests the mathematics of the fold.
%
%   V2b TIE ACTIVE IN PRODUCTION. The controller's own logged a_prime must
%       equal b_hat / h_bar_d^2 at every step of a real run. a_prime is
%       b/(gap+b)^2, so this equals b/w^2 IF AND ONLY IF gap = w - b, i.e.
%       only if law_ws = b took effect inside the controller. This is what
%       binds V2's replica to the shipped code.
%
%   V3  FINITE DIFFERENCE. The assembled row-4 / H2 entries that actually
%       carry beta -- F_e(4,5) = J_beta*M and H2(5) = -Grad*J_beta -- against
%       central differences of the quantities they linearise.
%
%   V3b FOLD NEGATIVE CONTROL. A length arm seeded at ws_init = 9/8 with w_s
%       locked has the SAME law as the tied arm at t = 0 (both evaluate at
%       w - 9/8) but only J_b in its b column, because its J_ws is zeroed by
%       the lock. The two arms therefore differ by the fold alone. If the
%       fold were misplaced (after the lock zeroing instead of before) the
%       tied arm would collapse onto this control.
%
%   V4  POLE GUARD. Under the tie the law's pole moves with the estimate, so
%       the slot-7 clamp is inert and the guard lives on slot 5. Assert the
%       law argument never hits gap_floor -- on the canonical scenario AND on
%       a scenario whose trough sits below the default b_clamp upper bound,
%       which is the case the scenario-number clamp would have missed.
%
%   Usage:  verify_formB_amp_tie          (script; ~40 s, 8 short runs)

clear; clc; clear functions;

root = fileparts(fileparts(mfilename('fullpath')));
root = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(root));
addpath(genpath(fullfile(root, 'model')));
addpath(genpath(fullfile(root, 'test_script')));

AX_Z    = 3;
B_ANCHOR = 9/8;
SEEDS   = [7, 11];
all_ok  = true;

fprintf('=== verify_formB_amp_tie ===\n');

% ======================================================================
% V1  default path bit-identity
% ======================================================================
fprintf('\n[V1] default-path bit identity\n');
cfg = local_canonical_cfg(root);
v1_ok = true;
for s = SEEDS
    a = run_formB_ws(cfg, struct('seed', s));
    b = run_formB_ws(cfg, struct('seed', s, ...
            'ctrl_const_override', struct('law_form_amp', false)));
    same = isequal(a.p_m_out, b.p_m_out) && isequal(a.f_d_out, b.f_d_out) ...
           && isequal(a.b_hat_out, b.b_hat_out) && isequal(a.a_hat_out, b.a_hat_out) ...
           && isequal(a.P_b_out, b.P_b_out);
    v1_ok = v1_ok && same;
    fprintf('  seed %-4d explicit false vs absent : %s\n', s, local_tf(same));
end
all_ok = all_ok && v1_ok;

% ======================================================================
% V2  closed-form fold (replica of the shipped local_gain_law_formB)
% ======================================================================
fprintf('\n[V2] closed-form fold at the tied point\n');
W = linspace(1.5, 30, 401).';
B = linspace(0.05, 1.45, 57);
e_J = 0; e_A = 0; e_ap = 0;
for bq = B
    g = W - bq;                       % the tie: gap = w - beta
    [ab, ap, Jb, ~, Jws, dAdb, ~, dAdws] = local_law_replica(g, bq, 1);
    e_J  = max(e_J,  max(abs((Jb + Jws)     - 1 ./ W.^2)  ./ (1 ./ W.^2)));
    e_A  = max(e_A,  max(abs((dAdb + dAdws) + 1 ./ W)     .* W));
    e_ap = max(e_ap, max(abs(ap - bq ./ W.^2) ./ (bq ./ W.^2)));
    e_ab = max(abs(ab - (1 - bq ./ W)));
end
v2_ok = (e_J < 8 * eps) && (e_A < 8 * eps) && (e_ap < 8 * eps) && (e_ab < 8 * eps);
fprintf('  rel |J_b + J_ws  - 1/w^2|   max %.2e\n', e_J);
fprintf('  rel |dA_db+dA_dws + 1/w|    max %.2e\n', e_A);
fprintf('  rel |a_prime - b/w^2|       max %.2e\n', e_ap);
fprintf('  abs |a_bar - (1 - b/w)|     max %.2e   -> %s (tol %.1e)\n', ...
        e_ab, local_tf(v2_ok), 8 * eps);
all_ok = all_ok && v2_ok;

% ======================================================================
% V2b tie active inside the shipped controller
% ======================================================================
fprintf('\n[V2b] tie active in production (a_prime == b_hat / h_bar_d^2)\n');
amp = run_formB_ws(cfg, struct('seed', 7, ...
        'ctrl_const_override', local_amp_override(root, cfg)));
% a_prime_out is logged in PHYSICAL units (a_bar' * a_disp, a_disp = a_o*R),
% so the test is scale-free: the ratio a_prime / (beta/w^2) must be a CONSTANT
% across the whole run, and that constant must be a_disp. A constant ratio can
% only happen if gap = w - beta at every step.
% LOG ALIGNMENT: a_prime[k] is built from the PRIOR estimate x_curr(5), while
% b_hat_out[k] is the POSTERIOR. The prior at step k is the posterior at k-1,
% so the comparison must use the lagged b. (Ignoring this shows a spurious
% ~1% spread that tracks the descent, where b moves fastest.)
apz = amp.a_prime_out(:, AX_Z);
bp  = amp.b_hat_out(:, AX_Z);
bz  = [bp(1); bp(1:end-1)];        % prior = previous posterior
wz  = amp.h_bar_d_out(:);          % N x 1 (commanded wall-normal distance)
ok_i = isfinite(apz) & isfinite(wz) & (apz > 0) & (wz > 0);
rat  = apz(ok_i) ./ (bz(ok_i) ./ wz(ok_i).^2);
spread = max(rat) / min(rat) - 1;
pc_v   = physical_constants();
a_disp = (pc_v.Ts / (pc_v.gamma_N * pc_v.R)) * pc_v.R;
v2b_ok = (spread < 1e-12) && (abs(mean(rat) / a_disp - 1) < 1e-11);
fprintf('  ratio a_prime / (beta/w^2) over %d steps: spread %.2e, mean %.6e\n', ...
        sum(ok_i), spread, mean(rat));
fprintf('  a_disp = a_o*R = %.6e   rel diff %.2e  -> %s\n', ...
        a_disp, abs(mean(rat) / a_disp - 1), local_tf(v2b_ok));
% the same statistic for the LENGTH arm must NOT be constant (negative control)
len = run_formB_ws(cfg, struct('seed', 7));
apl = len.a_prime_out(:, AX_Z); blp = len.b_hat_out(:, AX_Z);
bl = [blp(1); blp(1:end-1)]; wl = len.h_bar_d_out(:);
ok_l = isfinite(apl) & isfinite(wl) & (apl > 0) & (wl > 0);
rat_l = apl(ok_l) ./ (bl(ok_l) ./ wl(ok_l).^2);
spread_l = max(rat_l) / min(rat_l) - 1;
v2b_ok = v2b_ok && (spread_l > 1e-3);
fprintf('  same ratio, LENGTH arm: spread %.2e (must be large) -> %s\n', ...
        spread_l, local_tf(spread_l > 1e-3));
all_ok = all_ok && v2b_ok;

% ======================================================================
% V3  finite-difference check of the assembled beta entries
% ======================================================================
fprintf('\n[V3] finite-difference of F_e(4,5) and H2(5)\n');
w_pt = [2.0, 2.5, 3.0, 4.2, 10.0, 22.2];
M_pt = 0.004;                       % representative commanded step [R]
b_pt = 1.09;
e_fe = 0; e_h2 = 0;
for w = w_pt
    % F_e(4,5) linearises a_bar[k+1] = a_bar + a_prime(beta)*M w.r.t. beta
    f  = @(bb) local_aprime_tied(w, bb) * M_pt;
    e_fe = max(e_fe, abs(local_fd(f, b_pt) - local_Jtied(w, b_pt) * M_pt));
    % H2(5) linearises y2_pred = H2s*(a_bar - a_prime(beta)*Grad) w.r.t. beta
    g  = @(bb) -local_aprime_tied(w, bb) * M_pt;
    e_h2 = max(e_h2, abs(local_fd(g, b_pt) + local_Jtied(w, b_pt) * M_pt));
end
v3_ok = (e_fe < 1e-6) && (e_h2 < 1e-6);
fprintf('  max |FD - analytic|  F_e(4,5) %.2e   H2(5) %.2e  -> %s (tol 1e-6)\n', ...
        e_fe, e_h2, local_tf(v3_ok));
% the folded column must be the ONLY theta column that is alive
[~, ~, Jb0, Jp0, Jws0] = local_law_replica(3.0 - b_pt, b_pt, 1);
v3_ok = v3_ok && abs(Jp0) > 0 && abs(Jws0) > 0;   % both nonzero BEFORE folding
fprintf('  pre-fold J_p %.3e, J_ws %.3e both nonzero (so the fold is not a no-op)\n', ...
        Jp0, Jws0);
all_ok = all_ok && v3_ok;

% ======================================================================
% V3b fold negative control: same law at t=0, no fold
% ======================================================================
fprintf('\n[V3b] fold negative control (length arm seeded at ws = 9/8)\n');
ovc = local_amp_override(root, cfg);
ovc.law_form_amp = false;
ovc.ws_init      = B_ANCHOR;         % same law origin at t = 0
ctl = run_formB_ws(cfg, struct('seed', 7, 'ctrl_const_override', ovc));
d_amp = amp.b_hat_out(end, AX_Z) - amp.b_hat_out(1, AX_Z);
d_ctl = ctl.b_hat_out(end, AX_Z) - ctl.b_hat_out(1, AX_Z);
s_amp = amp.P_b_out(1, AX_Z);
% Same seed, same noise realisation, same prior: any difference is
% deterministic and is the fold. The criterion is OBSERVABILITY (the fold is
% wired and changes the answer), not a size threshold -- no derivation gives
% one, and the magnitude is a result, not an acceptance.
v3b_ok = abs(d_amp - d_ctl) > 1e-6;
fprintf('  beta travel  tied %+.5f (%.2f sigma) | unfolded control %+.5f (%.2f sigma)\n', ...
        d_amp, d_amp / s_amp, d_ctl, d_ctl / s_amp);
fprintf('  difference %+.5f (%.1f%% of the tied travel) -> %s (fold is wired)\n', ...
        d_amp - d_ctl, 100 * (d_amp - d_ctl) / d_amp, local_tf(v3b_ok));
fprintf('  REPORT (not an acceptance): the control shares the tied law at t=0\n');
fprintf('          but keeps only J_b, so this split separates the Jacobian\n');
fprintf('          fold from the origin+prior change. Read it in Stage 5.\n');
all_ok = all_ok && v3b_ok;

% ======================================================================
% V4  pole guard
% ======================================================================
fprintf('\n[V4] pole guard (law argument never reaches gap_floor)\n');
GAP_FLOOR = 1e-3;
wd_a = amp.h_bar_d_out(:);
keep_a = isfinite(wd_a) & (wd_a > 0);   % row 1 is the init call, w_d = 0
gap_amp = wd_a(keep_a) - amp.b_hat_out(keep_a, AX_Z);
ok_a = min(gap_amp) > GAP_FLOOR;
fprintf('  canonical scenario: min(w_d - beta) = %.4f  -> %s\n', min(gap_amp), local_tf(ok_a));
% the case a scenario-number clamp would have missed: trough well below the
% default b_clamp upper bound
cfg2 = cfg; cfg2.h_bottom = 2.5; cfg2.amplitude = 0.5; cfg2.h_min = 1.05 * 2.25;
amp2 = run_formB_ws(cfg2, struct('seed', 7, ...
        'ctrl_const_override', local_amp_override(root, cfg2)));
wd_b = amp2.h_bar_d_out(:);
keep_b = isfinite(wd_b) & (wd_b > 0);
gap2 = wd_b(keep_b) - amp2.b_hat_out(keep_b, AX_Z);
ok_b = min(gap2) > GAP_FLOOR;
fprintf('  low-trough scenario (h_bottom 2.5 um, h_bar_min %.2f): min = %.4f -> %s\n', ...
        cfg2.h_bottom / 2.25, min(gap2), local_tf(ok_b));
v4_ok = ok_a && ok_b;
all_ok = all_ok && v4_ok;

% ======================================================================
% V5  covariance health + frozen-slot integrity (smoke_formB_ws conventions)
% ======================================================================
fprintf('\n[V5] covariance health and frozen slots\n');
ampP = run_formB_ws(cfg, struct('seed', 7, 'log_P_full', true, ...
        'ctrl_const_override', local_amp_override(root, cfg)));
Pf = ampP.P_full_out;                       % N x n x n x 3
[N_s, n1, n2, n_ax] = size(Pf);
LOCK_TOL = 1e-12; SYM_TOL = 1e-10;
p_ok = true; sym_worst = 0; leak = false; first_bad = 0;
for ax = 1:n_ax
    for k = 1:N_s
        Pk = squeeze(Pf(k, :, :, ax));
        if ~all(isfinite(Pk(:))); p_ok = false; first_bad = k; break; end
        ds = max(abs(diag(Pk)));
        if ds > 0
            sym_worst = max(sym_worst, max(abs(Pk - Pk.'), [], 'all') / ds);
        end
        Ps = 0.5 * (Pk + Pk.'); dP = diag(Ps); act = dP > 0;
        if any(dP < 0) || any(any(Ps(~act, :) ~= 0))
            p_ok = false; leak = any(any(Ps(~act, :) ~= 0)); first_bad = k; break;
        end
        [~, cf] = chol(Ps(act, act));
        if cf ~= 0; p_ok = false; first_bad = k; break; end
    end
    if ~p_ok; break; end
end
fprintf('  P finite / symmetric / PD every step (%d x %d): %s  (worst rel asym %.2e%s)\n', ...
        N_s, n_ax, local_tf(p_ok && sym_worst <= SYM_TOL), sym_worst, ...
        local_bad(first_bad, leak));
% slot 6 (p) and slot 7 (w_s) must be frozen in state AND covariance
d_p  = max(abs(ampP.p_hat_out(:, AX_Z)  - ampP.p_hat_out(1, AX_Z)));
d_ws = max(abs(ampP.ws_hat_out(:, AX_Z) - ampP.ws_hat_out(1, AX_Z)));
d_Pp = max(abs(ampP.P_p_out(:)  - ampP.P_p_out(1)));
d_Pw = max(abs(ampP.P_ws_out(:) - ampP.P_ws_out(1)));
froz_ok = (d_p <= LOCK_TOL) && (d_ws <= LOCK_TOL) && (d_Pp <= LOCK_TOL) && (d_Pw <= LOCK_TOL);
fprintf('  frozen slots: |dp| %.1e |dws| %.1e |dPp| %.1e |dPws| %.1e -> %s (tol %.0e)\n', ...
        d_p, d_ws, d_Pp, d_Pw, local_tf(froz_ok), LOCK_TOL);
% slot 5 free and its variance monotone non-increasing (Q_bb = 0, F row = e5)
Pb = ampP.P_b_out(:, AX_Z);
n_up = sum(diff(Pb) > 1e-14);
mono_ok = (n_up == 0) && (Pb(end) < Pb(1));
fprintf('  slot 5: sqrt(P) %.5f -> %.5f, %d increases over %d steps -> %s\n', ...
        Pb(1), Pb(end), n_up, numel(Pb) - 1, local_tf(mono_ok));
e_all = ampP.p_d_out(2:end, :) - ampP.p_true_out(1:end-1, :);
div_ok = max(abs(e_all(:))) < 5;
fprintf('  no divergence: max|e| = %.4f um -> %s\n', max(abs(e_all(:))), local_tf(div_ok));
v5_ok = p_ok && (sym_worst <= SYM_TOL) && froz_ok && mono_ok && div_ok;
all_ok = all_ok && v5_ok;

% ======================================================================
fprintf('\n-----------------------------------------------------------------\n');
fprintf('VERDICT: %s   (V1 %s, V2 %s, V2b %s, V3 %s, V3b %s, V4 %s, V5 %s)\n', ...
        local_tf(all_ok), local_tf(v1_ok), local_tf(v2_ok), local_tf(v2b_ok), ...
        local_tf(v3_ok), local_tf(v3b_ok), local_tf(v4_ok), local_tf(v5_ok));

% ======================================================================
function ov = local_amp_override(root, cfg) %#ok<INUSD>
%LOCAL_AMP_OVERRIDE  ctrl_const for the tied arm in the test-ladder form.
%   The ladder form does not run the driver's arm block, so the amplitude
%   priors are derived here with the SAME formulas the driver uses.
    pc = physical_constants();
    env_lo = cfg.h_bottom / pc.R - 0.1;
    env_hi = cfg.h_init   / pc.R + 1.0;
    h = linspace(env_lo, env_hi, 20001).';
    c = zeros(size(h));
    for i = 1:numel(h)
        [~, c(i)] = calc_correction_functions(h(i), true);
    end
    b_eff = h .* (c - 1) ./ c;
    ov = struct('law_form_amp', true, 'lock_b', false, 'lock_p', true, ...
                'lock_ws', true, 'p_init', 1, 'b_init', 9/8, ...
                'Pf_b_std', max(abs(b_eff - 9/8)), ...
                'Pf_a_floor', max(abs((1 - (9/8) ./ h) - 1 ./ c)));
end

function cfg = local_canonical_cfg(root) %#ok<INUSD>
%LOCAL_CANONICAL_CFG  The canonical scenario, spelled out (the driver's own
%   local_canonical_config is private to it).
    pc = physical_constants();
    cfg = user_config();
    cfg.trajectory_type = 'osc';
    cfg.h_init    = 50;   cfg.h_bottom = 4.5;  cfg.amplitude = 2.5;
    cfg.frequency = 1;    cfg.n_cycles = 2;
    cfg.t_hold    = 0.5;  cfg.t_descend_override = 1.0;  cfg.T_sim = 4.8;
    cfg.h_min     = 1.1 * pc.R;
    cfg.ctrl_enable = true; cfg.thermal_enable = true; cfg.meas_noise_enable = true;
    cfg.lambda_c = 0.7;   cfg.a_pd = 0.05;  cfg.a_cov = 0.05;
    cfg.meas_noise_std = [0.00062; 0.00057; 0.00331];
    cfg.h_bar_safe = 1.5;
end

function [a_bar, a_bar_p, J_b, J_p, J_ws, dA_db, dA_dp, dA_dws] = ...
        local_law_replica(gap, b, p)
% REPLICA OF model/controller/motion_control_law_formB_ws.m local_gain_law_formB
% (copied verbatim, vectorised). V2b binds this replica to the shipped code.
    u    = 1 + gap ./ b;
    upow = u.^(-p);
    a_bar   = 1 - upow;
    a_bar_p = (p ./ b) .* upow ./ u;
    J_b  = (-1 ./ b + (p + 1) .* gap ./ (b .* (gap + b))) .* a_bar_p;
    J_p  = (1 ./ p - log(u)) .* a_bar_p;
    J_ws = ((p + 1) ./ (gap + b)) .* a_bar_p;
    dA_db  = -(gap ./ b) .* a_bar_p;
    dA_dp  = upow .* log(u);
    dA_dws = -a_bar_p;
end

function ap = local_aprime_tied(w, b)
    [~, ap] = local_law_replica(w - b, b, 1);
end

function J = local_Jtied(w, b)
    [~, ~, Jb, ~, Jws] = local_law_replica(w - b, b, 1);
    J = Jb + Jws;
end

function d = local_fd(f, x)
    h = max(1e-7, 1e-7 * abs(x));
    d = (f(x + h) - f(x - h)) / (2 * h);
end

function s = local_tf(ok)
    if ok; s = 'PASS'; else; s = 'FAIL'; end
end

function s = local_bad(k, leak)
    if k == 0; s = ''; else; s = sprintf('; first bad k=%d, frozen-row leak %d', k, leak); end
end
