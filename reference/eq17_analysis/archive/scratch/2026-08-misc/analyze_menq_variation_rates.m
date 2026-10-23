% analyze_menq_variation_rates.m
%
% STATUS: ACTIVE -- tests Menq's stated assumption (b slow, w_s near zero) against the run
%   log; it comes out backwards: ws_hat moves 4-6x more than b_hat, corr +0.999.
%   See memory project-formb-7a-falsified-quadrature-ratchet-2026-08-06.
%
% PURPOSE: measure the ACTUAL time-domain variation of b_hat(t) and ws_hat(t)
%   inside the production Form B filter, to test Prof. Menq's stated modelling
%   assumption: "While we assume that the variation of b is slow, the variation
%   of w_s should be near zero except in the initial time period in which the
%   error caused by the initial guess being corrected."
%   Production runs Q55 = Q66 = Q77 = 0, so BOTH parameters are modelled as
%   constants; the question is what the posteriors actually do.
%
% SCOPE: read-only w.r.t. production code. Only run_formB_ws(opts) is called;
%   no file under model/ or test_script/integration/ is touched. Canonical
%   4.8 s scenario, wall-normal axis (z = 3), 6 seeds.
%     Arm A : opts.tier = 't2'  (w_s FREE, Pf_ws_std = 0.028 calibration-grade)
%     Arm B : opts.tier = 't1'  (w_s PINNED) -- control arm: how b_hat moves
%             when it has no w_s to share the burden with
%
% CRITERIA (pre-registered):
%   M1  Menq's assumption holds  <=>  in the steady window
%         sd(ws_hat)/w_s_anchor  <<  sd(b_hat)/b_anchor      and
%         |drift_ws| [%/s]       <<  |drift_b| [%/s].
%       It is FALSIFIED if the normalized ratio ws/b is >= 1, i.e. the state he
%       expects to be frozen moves as much as, or more than, the slow one.
%   M2  |corr(b_hat, ws_hat)| -> 1 over the steady window = the two states ride
%       one common mode = time-domain fingerprint of b <-> w_s compensation.
%   M3  sqrt(P_ws) must be non-increasing for the whole run (structural
%       consequence of Q77 = 0); record the total shrink factor.
%
% CAVEAT recorded up front: in this scenario the TRUE wall sits at w_bar_s = 1
%   and the controller seed is exactly 1 (no opts.ws_inject), so the "initial
%   guess error" Menq refers to is ZERO here. Any excursion measured in item 3
%   is therefore NOT initial-guess correction -- it is noise-driven wander of a
%   state whose initial guess was already right.
%
% OUTPUT: test_results/temp_menq_variation_rates.{mat,png}  (gitignored)

clear; clc;

here = fileparts(mfilename('fullpath'));
proj = fullfile(here, '..', '..');
pdirs = {'model', 'model/dual_track', 'model/diag', 'model/thermal_force', ...
         'model/config', 'model/wall_effect', 'model/controller', ...
         'model/trajectory', 'test_script/integration'};
for i = 1:numel(pdirs); addpath(fullfile(proj, pdirs{i})); end
res_dir = fullfile(proj, 'test_results');
if ~exist(res_dir, 'dir'); mkdir(res_dir); end

% ---------------- named constants (no magic numbers) ----------------
AX_Z        = 3;            % wall-normal axis index
SEEDS       = [7 11 3001 3002 3003 3004];
PF_WS_STD   = 0.028;        % [-] calibration-grade w_s prior (project value)
B_ANCHOR    = 9/8;          % [-] derived far-field reflection coefficient
WS_ANCHOR   = 1;            % [-] nominal contact position = TRUE wall here
% Window boundaries [s]. Scenario: hold 0.5 -> descend 1.0 -> 2 x 1 Hz osc
% -> final hold; T = 4.8 s. The filter starts at h_bar = 22 (far field, wall
% position essentially unobservable) and only meets the wall during descent.
T_TRANS_END = 1.6;          % [s] end of "initial period": hold + descent
                            %     + 0.1 s osc settle (driver's OSC_SETTLE_S)
T_HOLD_BEG  = 3.8;          % [s] final-hold window start (driver's
                            %     t3 + HOLD_SETTLE_S = 3.5 + 0.3)
T_END       = 4.8;          % [s] end of run
H_GRID_LO   = 1.05;         % [-] truth-curve lookup grid, below the h_min clamp
H_GRID_HI   = 30;           % [-] above the start height 22.2
H_GRID_N    = 4001;
MONO_TOL    = 1e-12;        % [-] float tolerance for the P monotonicity test

% ---------------- run the two arms ----------------
optA = struct('tier', 't2', 'seeds', SEEDS, ...
              'ctrl_const_override', struct('Pf_ws_std', PF_WS_STD));
optB = struct('tier', 't1', 'seeds', SEEDS);

fprintf('\n########## ARM A (t2, w_s free, Pf_ws_std = %.3f) ##########\n', PF_WS_STD);
outA = run_formB_ws(optA);
fprintf('\n########## ARM B (t1, w_s pinned) ##########\n');
outB = run_formB_ws(optB);

% ---------------- extract per-seed trajectories (z axis) ----------------
A = local_stack(outA, AX_Z, B_ANCHOR, WS_ANCHOR);
B = local_stack(outB, AX_Z, B_ANCHOR, WS_ANCHOR);
t = A.t;
ns = numel(SEEDS);

w_trans  = t <= T_TRANS_END;
w_steady = t >  T_TRANS_END;
w_hold   = t >  T_HOLD_BEG;
w_st1    = w_steady & t <= (T_TRANS_END + T_END) / 2;   % steady 1st half
w_st2    = w_steady & t >  (T_TRANS_END + T_END) / 2;   % steady 2nd half

% ---------------- item 1: variation magnitude and drift rate ----------------
fprintf('\n================ ITEM 1: VARIATION MAGNITUDE (z axis) ================\n');
fprintf('windows: transient [0, %.1f] s | steady (%.1f, %.1f] s | final hold (%.1f, %.1f] s\n', ...
        T_TRANS_END, T_TRANS_END, T_END, T_HOLD_BEG, T_END);
fprintf('normalization: b/%.4f, w_s/%.0f -> both in %% of their anchor\n', B_ANCHOR, WS_ANCHOR);

statA_st = local_window_stats(t, A.db, A.dws, w_steady);
statB_st = local_window_stats(t, B.db, B.dws, w_steady);
statA_hd = local_window_stats(t, A.db, A.dws, w_hold);
statB_hd = local_window_stats(t, B.db, B.dws, w_hold);

local_print_stats('ARM A steady', statA_st, ns);
local_print_stats('ARM A hold  ', statA_hd, ns);
local_print_stats('ARM B steady', statB_st, ns);
local_print_stats('ARM B hold  ', statB_hd, ns);

rat_sd  = statA_st.sd_ws  ./ max(statA_st.sd_b, eps);
rat_dr  = abs(statA_st.dr_ws) ./ max(abs(statA_st.dr_b), eps);
fprintf(['\nMENQ RATIO (arm A steady, per seed): sd(w_s)/sd(b) = %s\n' ...
         '                                     mean %.2f  (Menq expects << 1)\n'], ...
        mat2str(round(rat_sd, 2)), mean(rat_sd));
fprintf(['                                     |drift w_s|/|drift b| = %s\n' ...
         '                                     mean %.2f\n'], ...
        mat2str(round(rat_dr, 2)), mean(rat_dr));

% Exchange rate: is "% of anchor" doing the work? Push both deviations through
% the SAME law Jacobians the controller uses (run_formB_ws.local_parallel_law
% pattern): |dA/dws| = (p/b)(1+g/b)^(-p-1) and |dA/db| = (g/b)|dA/dws|, so the
% gain-equivalent ratio needs neither p nor the common factor:
%     sigma_gain(w_s) / sigma_gain(b) = [sd_ws% * ws_anchor * b] / [sd_b% * b_anchor * g]
g_hold  = mean(mean(A.gap(w_hold, :)));            % [-] law argument at the hold
b_hold  = mean(mean(A.b_abs(w_hold, :)));          % [-] posterior b there
rat_gain = (statA_st.sd_ws * WS_ANCHOR * b_hold) ./ ...
           max(statA_st.sd_b * B_ANCHOR * g_hold, eps);
fprintf(['GAIN-EQUIVALENT RATIO (same law Jacobians, g = %.3f, b = %.4f at the hold):\n' ...
         '                                     %s  mean %.2f\n'], ...
        g_hold, b_hold, mat2str(round(rat_gain, 2)), mean(rat_gain));

% Mechanism check (indicative, single-channel): if the pair absorbs ONE common
% gain innovation, a scalar update splits it as delta_theta_i ~ P_ii * H_i, so
%     sd_ws% / sd_b%  =  (P_ws/P_bb) * (H_ws/H_b) * (b_anchor/ws_anchor),
% with H_ws/H_b = b/g from the law Jacobians above. This is only exact for a
% single scalar channel; the filter runs y1 and y2 with different Jacobians, so
% treat the number as an order check on the MECHANISM, not an identity.
for wsel = 1:2
    if wsel == 1; msk = w_steady; lab = 'steady'; else; msk = w_hold; lab = 'hold  '; end
    Pr   = mean(mean(A.sqP_ws(msk, :))).^2 / mean(mean(A.sqP_b(msk, :))).^2;
    gm   = mean(mean(A.gap(msk, :)));
    bm   = mean(mean(A.b_abs(msk, :)));
    pred = Pr * (bm / gm) * (B_ANCHOR / WS_ANCHOR);
    if wsel == 1; meas = mean(rat_sd); else; meas = mean(statA_hd.sd_ws ./ max(statA_hd.sd_b, eps)); end
    fprintf('PRIOR-SPLIT PREDICTION (%s): P ratio %.2f -> predicted sd_ws/sd_b %.2f vs measured %.2f\n', ...
            lab, Pr, pred, meas);
end

d_ab = max(abs(mean(B.db, 2) - mean(A.db, 2)));
fprintf(['ARM A vs ARM B, b_hat ensemble mean: max gap %.4f %% of anchor ' ...
         '(sd %.3f%% vs %.3f%%) -- releasing w_s barely moves b_hat\n'], ...
        d_ab, mean(statA_st.sd_b), mean(statB_st.sd_b));

% ---------------- item 2: correlation ----------------
fprintf('\n================ ITEM 2: corr(b_hat, w_s_hat), steady window ================\n');
rho_st = zeros(ns, 1); rho_hd = zeros(ns, 1);
for s = 1:ns
    rho_st(s) = local_corr(A.db(w_steady, s), A.dws(w_steady, s));
    rho_hd(s) = local_corr(A.db(w_hold, s),   A.dws(w_hold, s));
end
fprintf('per seed (steady): %s\n', mat2str(round(rho_st.', 3)));
fprintf('per seed (hold)  : %s\n', mat2str(round(rho_hd.', 3)));
fprintf('steady mean %+.3f +- %.3f (sd) | hold mean %+.3f +- %.3f\n', ...
        mean(rho_st), std(rho_st), mean(rho_hd), std(rho_hd));

% ---------------- item 3: initial transient of w_s ----------------
fprintf('\n================ ITEM 3: w_s INITIAL EXCURSION (arm A) ================\n');
fprintf('NOTE: ws seed = %g = TRUE wall, so the "initial guess error" is zero here.\n', WS_ANCHOR);
exc = zeros(ns, 1); t_exc = zeros(ns, 1); t90 = nan(ns, 1); dws_inf = zeros(ns, 1);
for s = 1:ns
    y = A.dws(:, s);
    [exc(s), ie] = max(abs(y(w_trans)));
    tt = t(w_trans); t_exc(s) = tt(ie);
    dws_inf(s) = mean(y(w_hold));
    thr = 0.9 * abs(dws_inf(s));
    k90 = find(abs(y) >= thr, 1, 'first');
    if ~isempty(k90); t90(s) = t(k90); end
end
fprintf('%6s | %10s %8s | %10s %9s\n', 'seed', 'peak|dws|%', 't_peak s', 'dws_inf %', 't90 s');
for s = 1:ns
    fprintf('%6d | %10.3f %8.3f | %+10.3f %9.3f\n', SEEDS(s), exc(s), t_exc(s), dws_inf(s), t90(s));
end
fprintf('mean: peak %.3f%%  final %+.3f%%  t90 %.3f s\n', mean(exc), mean(dws_inf), mean(t90, 'omitnan'));
fprintf('does it stop after the initial period? drift rate 1st vs 2nd half of steady:\n');
s1 = local_window_stats(t, A.db, A.dws, w_st1);
s2 = local_window_stats(t, A.db, A.dws, w_st2);
fprintf('  w_s drift  1st half %+.3f +- %.3f %%/s | 2nd half %+.3f +- %.3f %%/s\n', ...
        mean(s1.dr_ws), std(s1.dr_ws), mean(s2.dr_ws), std(s2.dr_ws));
fprintf('  b   drift  1st half %+.3f +- %.3f %%/s | 2nd half %+.3f +- %.3f %%/s\n', ...
        mean(s1.dr_b), std(s1.dr_b), mean(s2.dr_b), std(s2.dr_b));

% ---------------- item 4: sqrt(P) monotonicity and shrink ----------------
fprintf('\n================ ITEM 4: sqrt(P) MONOTONICITY (Q = 0 consequence) ================\n');
fprintf('%6s | %11s %11s %8s | %11s %11s %8s | %10s\n', 'seed', ...
        'sqPws[0]', 'sqPws[end]', 'shrink', 'sqPb[0]', 'sqPb[end]', 'shrink', 'max +step');
for s = 1:ns
    pw = A.sqP_ws(:, s); pb = A.sqP_b(:, s);
    up = max(max(diff(pw)), max(diff(pb)));
    fprintf('%6d | %11.5f %11.5f %8.2fx | %11.5f %11.5f %8.2fx | %10.2e\n', ...
            SEEDS(s), pw(1), pw(end), pw(1)/max(pw(end), eps), ...
            pb(1), pb(end), pb(1)/max(pb(end), eps), up);
end
mono_ok = all(all(diff(A.sqP_ws, 1, 1) <= MONO_TOL)) && all(all(diff(A.sqP_b, 1, 1) <= MONO_TOL));
fprintf('M3 monotone non-increasing (tol %.0e): %s\n', MONO_TOL, local_pf(mono_ok));
fprintf('arm B sqrt(P_ws) constant at %.5f (pinned): %s\n', B.sqP_ws(1, 1), ...
        local_pf(all(all(abs(diff(B.sqP_ws, 1, 1)) < MONO_TOL))));

% ---------------- truth demand for b (offline reference line) ----------------
hg = linspace(H_GRID_LO, H_GRID_HI, H_GRID_N).';
cg = zeros(H_GRID_N, 1);
for i = 1:H_GRID_N
    [~, cg(i)] = calc_correction_functions(hg(i), true);
end
b_eff_g  = (cg - 1) .* (hg - 1);                 % level reading of the truth
h_true   = outA.runs{1}.h_bar_true_out(:);
b_eff_t  = interp1(hg, b_eff_g, min(max(h_true, H_GRID_LO), H_GRID_HI), 'linear');
db_truth = 100 * (b_eff_t / B_ANCHOR - 1);       % [%] of anchor
fprintf('\nb_eff truth demand over the visited envelope: [%.2f, %.2f] %% of anchor\n', ...
        min(db_truth), max(db_truth));

% ---------------- figure ----------------
COL_TRUE = [0.8 0 0]; COL_HAT = [0 0.2 0.9]; BANDC = [0.45 0.55 0.95];
COL_ALT  = [0.25 0.25 0.25];
FS = 18; LFS = 13; AXLW = 2.0; LW = 2.0;

mA_b  = mean(A.db,  2);  sA_b  = std(A.db,  0, 2);
mA_w  = mean(A.dws, 2);  sA_w  = std(A.dws, 0, 2);
mB_b  = mean(B.db,  2);

% Shared y-limits so the two panels can be compared BY EYE (that is the point
% of this figure). They are set from the ESTIMATE traces + bands of both
% panels; the truth-demand line may leave the axes and is reported to console.
lim_hi = max([max(mA_b + sA_b), max(mA_w + sA_w), max(mB_b)]);
lim_lo = min([min(mA_b - sA_b), min(mA_w - sA_w), min(mB_b)]);
pad    = 0.12 * (lim_hi - lim_lo);
ylims  = [lim_lo - pad, lim_hi + pad];
clip_note = min(db_truth) < ylims(1) || max(db_truth) > ylims(2);

fig = figure('Position', [80 80 1100 760], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile; hold on;
fill([t; flipud(t)], [mA_b - sA_b; flipud(mA_b + sA_b)], BANDC, 'FaceAlpha', 0.30, ...
     'EdgeColor', 'none', 'DisplayName', '\pm\sigma_{seed}');
plot(t, db_truth, '-', 'Color', COL_TRUE, 'LineWidth', LW, 'DisplayName', 'b_{eff} truth demand');
plot(t, mB_b, '--', 'Color', COL_ALT, 'LineWidth', LW, 'DisplayName', '\^b (w_s pinned)');
plot(t, mA_b, '-', 'Color', COL_HAT, 'LineWidth', LW, 'DisplayName', '\^b (w_s free)');
xlim([0 T_END]); ylim(ylims);
ylabel('\deltab / b_0  (%)', 'FontSize', FS, 'FontWeight', 'bold');
legend('Location', 'northoutside', 'Orientation', 'horizontal', ...
       'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on', 'Interpreter', 'tex');
set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;

nexttile; hold on;
fill([t; flipud(t)], [mA_w - sA_w; flipud(mA_w + sA_w)], BANDC, 'FaceAlpha', 0.30, ...
     'EdgeColor', 'none', 'DisplayName', '\pm\sigma_{seed}');
plot(t, zeros(size(t)), '-', 'Color', COL_TRUE, 'LineWidth', LW, 'DisplayName', 'w_s true');
plot(t, mA_w, '-', 'Color', COL_HAT, 'LineWidth', LW, 'DisplayName', 'w_s estimate');
xlim([0 T_END]); ylim(ylims);
ylabel('\deltaw_s / w_{s,0}  (%)', 'FontSize', FS, 'FontWeight', 'bold');
xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
legend('Location', 'northoutside', 'Orientation', 'horizontal', ...
       'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on', 'Interpreter', 'tex');
set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;

png_path = fullfile(res_dir, 'temp_menq_variation_rates.png');
exportgraphics(fig, png_path, 'Resolution', 150);
close(fig);
fprintf('\nfigure: %s   (shared ylim [%.2f, %.2f] %%; truth line clipped: %d)\n', ...
        png_path, ylims(1), ylims(2), clip_note);

M = struct('seeds', SEEDS, 't', t, 'A', A, 'B', B, 'db_truth', db_truth, ...
           'statA_st', statA_st, 'statB_st', statB_st, 'statA_hd', statA_hd, ...
           'statB_hd', statB_hd, 'rho_steady', rho_st, 'rho_hold', rho_hd, ...
           'exc', exc, 'dws_inf', dws_inf, 't90', t90, 'mono_ok', mono_ok, ...
           'windows', [0 T_TRANS_END; T_TRANS_END T_END; T_HOLD_BEG T_END]);
mat_path = fullfile(res_dir, 'temp_menq_variation_rates.mat');
save(mat_path, 'M');
fprintf('saved: %s\n', mat_path);


%% =================== local helpers ===================

function S = local_stack(out, ax, b_anchor, ws_anchor)
%LOCAL_STACK  Per-seed z-axis columns, normalized to % of the anchors.
    ns = numel(out.runs);
    S.t = out.runs{1}.tout(:);
    n  = numel(S.t);
    S.db     = zeros(n, ns);   % 100*(b_hat/b_anchor - 1)   [%]
    S.dws    = zeros(n, ns);   % 100*(ws_hat/ws_anchor - 1) [%]
    S.sqP_b  = zeros(n, ns);   % sqrt(P_bb)  (driver already stores sqrt)
    S.sqP_ws = zeros(n, ns);
    S.b_abs  = zeros(n, ns);   % raw b_hat [-]
    S.gap    = zeros(n, ns);   % law argument g = w_bar_d - ws_hat [-]
    for s = 1:ns
        r = out.runs{s};
        S.db(:, s)     = 100 * (r.b_hat_out(:, ax)  / b_anchor  - 1);
        S.dws(:, s)    = 100 * (r.ws_hat_out(:, ax) / ws_anchor - 1);
        S.sqP_b(:, s)  = r.P_b_out(:, ax);
        S.sqP_ws(:, s) = r.P_ws_out(:, ax);
        S.b_abs(:, s)  = r.b_hat_out(:, ax);
        S.gap(:, s)    = r.h_bar_d_out(:) - r.ws_hat_out(:, ax);
    end
end


function st = local_window_stats(t, db, dws, mask)
%LOCAL_WINDOW_STATS  Per-seed sd [%] and linear drift rate [%/s] in one window.
    ns = size(db, 2);
    st = struct('sd_b', zeros(ns, 1), 'sd_ws', zeros(ns, 1), ...
                'dr_b', zeros(ns, 1), 'dr_ws', zeros(ns, 1), ...
                'mean_b', zeros(ns, 1), 'mean_ws', zeros(ns, 1));
    tw = t(mask);
    for s = 1:ns
        yb = db(mask, s); yw = dws(mask, s);
        st.sd_b(s)  = std(yb);   st.sd_ws(s)  = std(yw);
        st.mean_b(s) = mean(yb); st.mean_ws(s) = mean(yw);
        pb = polyfit(tw, yb, 1); pw = polyfit(tw, yw, 1);
        st.dr_b(s)  = pb(1);     st.dr_ws(s)  = pw(1);
    end
end


function local_print_stats(label, st, ns)
%LOCAL_PRINT_STATS  Ensemble line for one arm/window.
    sem = @(v) std(v) / sqrt(ns);
    fprintf(['%s : b  sd %6.3f%%  drift %+7.3f +-%5.3f %%/s (SEM) | ' ...
             'w_s sd %6.3f%%  drift %+7.3f +-%5.3f %%/s\n'], label, ...
            mean(st.sd_b), mean(st.dr_b), sem(st.dr_b), ...
            mean(st.sd_ws), mean(st.dr_ws), sem(st.dr_ws));
end


function r = local_corr(x, y)
%LOCAL_CORR  Pearson correlation (base MATLAB; 0 if either side is constant).
    if std(x) < eps || std(y) < eps
        r = 0;
        return;
    end
    C = corrcoef(x, y);
    r = C(1, 2);
end


function s = local_pf(tf)
%LOCAL_PF  PASS/FAIL label.
    if tf; s = 'PASS'; else; s = 'FAIL'; end
end
