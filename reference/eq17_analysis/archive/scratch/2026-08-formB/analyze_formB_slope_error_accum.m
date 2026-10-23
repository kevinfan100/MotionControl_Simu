% analyze_formB_slope_error_accum.m
%
% STATUS: ACTIVE -- found the real defect behind the 7a debate: forward-Euler quadrature
%   ratchet -0.55 %, fixed by one midpoint line (-> 0.060 %).
%   See memory project-formb-7a-falsified-quadrature-ratchet-2026-08-06.
%
% PURPOSE: quantify the "defect-1 amplification" for Form B. The controller
%   carries the normalized gain a_bar as a STATE and pushes it forward with the
%   law's own slope,
%       a_bar[k+1] = a_bar[k] + a_bar'(b, p, w_s) * (increment) ,
%   never re-anchoring the level from (b, p, w_s). The sibling expgain filter
%   did the same and was convicted of "defect 1": a 1.65 % RMS slope-model error
%   integrated into a 5.2 % level error over one 10x height sweep (~3x
%   amplification); switching to the ALGEBRAIC form (level recomputed from the
%   constants every step) cut the descent error 11.03 % -> 5.05 %.
%   This script measures the same two numbers for Form B, offline:
%     (1) e_direct[k]     = a_bar_model(w[k]) - a_bar_true(w[k])
%                           (what an algebraic Form B would carry)
%     (2) e_integrated[k] = A[k] - a_bar_true(w[k]),  A[0] = a_bar_true(w[0]),
%         A propagated with the MODEL slope only
%   and their ratio, per trajectory phase, for three quadrature conventions and
%   an expgain reference arm.
%
% SCOPE: PURE OFFLINE GEOMETRY. No filter is run, no production file is touched
%   or modified. The only production code called is
%       calc_correction_functions   (truth SSOT: a_bar_true = 1/c_perp)
%       user_config / calc_simulation_params / trajectory_generator
%                                   (canonical commanded-height sequence)
%   The canonical scenario overrides are copied verbatim from the local function
%   local_canonical_config inside test_script/integration/run_formB_ws.m (hold
%   0.5 s @ h = 50 um -> 1.0 s cosine descent -> 2 x 1 Hz osc, trough
%   h = 4.5 um -> final hold; T = 4.8 s, Ts = 1/1600 s).
%
% DECLARED LIMITATION (must accompany every number reported from this script):
%   the integrated arm has NO measurement update. The real filter is pulled back
%   toward the truth by y1 / y2 every step, so e_integrated here is the
%   OPEN-LOOP UPPER BOUND on what the slope model alone can build up, not a
%   prediction of the deployed filter's error. The increment used is the
%   commanded difference only; the production predict adds
%   (1 - lambda_c)*dw3_hat, a noise-driven term deliberately excluded here.
%
% QUADRATURE CONVENTIONS (the propagation is a quadrature of a_bar' dw, so the
%   rule matters and is NOT the same in the two sibling controllers):
%   'left'  A[k+1] = A[k] + a'(w[k]) * (w[k+1] - w[k])   -- the assignment's
%           formula; also what expgain does (Delta_hbar_d, current call)
%   'prod'  A[k+1] = A[k] + a'(w[k]) * (w[k] - w[k-1])   -- what Form B
%           production does: motion_control_law_formB_ws.m evaluates the law at
%           w_bar_d (current pd) but multiplies by Delta_wbar_d_km1, the
%           PREVIOUS call's increment (the "one-step lead" fix) => right-endpoint
%   'mid'   A[k+1] = A[k] + a'((w[k]+w[k+1])/2) * (w[k+1] - w[k])  -- second-order
%           control arm; used only to prove that the residual left over after the
%           telescoping identity is quadrature, not representation
%
% ARMS (Form B, each under all three conventions):
%   A. anchor    (b, p, w_s) = (9/8, 1, 1)        -- the production z-axis seed
%   B. fit2      (b, p) minimax on the visited envelope, w_s = 1 pinned
%                                                 -- the production oracle_bp line
%   C. fit3      (b, p, w_s) minimax, origin free -- the parallel-axis recipe
%   REF. expgain 7b: a_bar = 1 - w^(-b), slope a_bar' = (b/w)*(1 - a_bar) taken
%        from local_gain_slope in motion_control_law_5state_expgain.m. Its slope
%        depends on the STATE, so its propagation is an ODE, not a quadrature --
%        included to reproduce the defect-1 amplification for scale.
%
% OUTPUT: test_results/temp_formB_slope_accum.{png,mat}  (gitignored)

clear; clc;

here = fileparts(mfilename('fullpath'));
proj = fullfile(here, '..', '..');
pdirs = {'model', 'model/config', 'model/wall_effect', 'model/trajectory'};
for i = 1:numel(pdirs); addpath(fullfile(proj, pdirs{i})); end
res_dir = fullfile(proj, 'test_results');
if ~exist(res_dir, 'dir'); mkdir(res_dir); end

% ---------------- named constants (no magic numbers) ----------------
B_ANCHOR    = 9/8;      % [-] far-field reflection coefficient (derived anchor)
P_ANCHOR    = 1;        % [-] far-field power (derived anchor)
WS_ANCHOR   = 1;        % [-] contact at w_bar = 1 (the TRUE wall in sim)
B_EXPGAIN   = 1;        % [-] expgain far-field anchor (same two limits)
GAP_FLOOR   = 0.01;     % [-] law-argument floor, matches the controller guard
FIT_N       = 8000;     % grid for the minimax fits (sup is grid-independent)
FIT_START2  = [1.03, 0.95];         % near the production oracle pair
FIT_START3  = [1.03, 0.95, 1.00];   % origin-free fit, seeded at the anchor
% canonical scenario (verbatim from run_formB_ws/local_canonical_config)
H_INIT      = 50;       % [um]
H_BOTTOM    = 4.5;      % [um]
AMPLITUDE   = 2.5;      % [um]
FREQUENCY   = 1;        % [Hz]
N_CYCLES    = 2;
T_HOLD      = 0.5;      % [s]
T_DESCEND   = 1.0;      % [s]
T_SIM       = 4.8;      % [s]
H_BAR_MIN   = 1.1;      % [-] truth-curve validity floor -> h_min = 1.1*R
LAMBDA_C    = 0.7;      % [-] canonical closed-loop pole (logged, unused here)

% ================================================================
% 1. Commanded height sequence w_bar_d[k], from the production generator
% ================================================================
pc  = physical_constants();
cfg = user_config();
cfg.trajectory_type = 'osc';
cfg.h_init    = H_INIT;
cfg.h_bottom  = H_BOTTOM;
cfg.amplitude = AMPLITUDE;
cfg.frequency = FREQUENCY;
cfg.n_cycles  = N_CYCLES;
cfg.t_hold    = T_HOLD;
cfg.t_descend_override = T_DESCEND;
cfg.T_sim     = T_SIM;
cfg.h_min     = H_BAR_MIN * pc.R;
cfg.lambda_c  = LAMBDA_C;

params = calc_simulation_params(cfg);
P  = params.Value;
Ts = P.common.Ts;
N  = round(cfg.T_sim / Ts) + 1;
t  = (0:N-1)' * Ts;

clear trajectory_generator;              % fresh persistent state
w_bar = zeros(N, 1);                     % commanded w_bar_d[k] seen by the filter
pd_for_ctrl = P.common.p0;               % driver convention: pd_k = previous call
for k = 1:N
    w_bar(k) = (dot(pd_for_ctrl, P.wall.w_hat) - P.wall.pz) / P.common.R;
    pd_for_ctrl = trajectory_generator(t(k), P);
end
clear trajectory_generator;

w_lo = min(w_bar); w_hi = max(w_bar);
assert(w_lo > 1, 'analyze_formB_slope_error_accum:belowContact', ...
       'commanded trajectory reaches w_bar = %.4f <= 1.', w_lo);

% ================================================================
% 2. Truth curve on the commanded heights (SSOT, analytic derivative)
% ================================================================
a_true  = zeros(N, 1);      % a_bar_true = 1/c_perp                        [-]
ap_true = zeros(N, 1);      % d a_bar_true / d w_bar                       [-]
for k = 1:N
    [~, c_perp_k, dv_k] = calc_correction_functions(w_bar(k), true);
    a_true(k)  = 1 / c_perp_k;
    ap_true(k) = -dv_k.dc_perp_dh / c_perp_k^2;
end

% ================================================================
% 3. Best-fit constants on the visited envelope (minimax, like production)
% ================================================================
wf = linspace(w_lo, w_hi, FIT_N)';
cf = zeros(FIT_N, 1);
for i = 1:FIT_N
    [~, cf(i)] = calc_correction_functions(wf(i));
end
a_target = 1 ./ cf;
sopt = optimset('TolX', 1e-10, 'TolFun', 1e-12, ...
                'MaxFunEvals', 2e4, 'MaxIter', 2e4, 'Display', 'off');

obj2 = @(th) max(abs(law_a(wf, th(1), th(2), WS_ANCHOR, GAP_FLOOR) - a_target));
th2  = fminsearch(obj2, FIT_START2, sopt);
th2  = fminsearch(obj2, th2,        sopt);

obj3 = @(th) max(abs(law_a(wf, th(1), th(2), th(3), GAP_FLOOR) - a_target));
th3  = fminsearch(obj3, FIT_START3, sopt);
th3  = fminsearch(obj3, th3,        sopt);

% expgain reference: best single b for a_bar = 1 - w^(-b) on the same envelope
objE = @(bb) max(abs((1 - wf.^(-bb)) - a_target));
bE   = fminsearch(objE, B_EXPGAIN, sopt);

arms = struct( ...
    'name', {'anchor (9/8, 1, 1)', 'fit2 (b,p | w_s=1)', 'fit3 (b,p,w_s free)'}, ...
    'tag',  {'anchor', 'fit2', 'fit3'}, ...
    'b',    {B_ANCHOR, th2(1), th3(1)}, ...
    'p',    {P_ANCHOR, th2(2), th3(2)}, ...
    'ws',   {WS_ANCHOR, WS_ANCHOR, th3(3)});

CONV = {'left', 'prod', 'mid'};

% Phase windows [s] (trajectory_generator boundaries)
t1 = T_HOLD; t2 = t1 + T_DESCEND; t3 = t2 + N_CYCLES / FREQUENCY;
ph = struct('name', {'hold-far', 'descent', 'oscillation', 'hold-trough', 'ALL'}, ...
            'mask', {t <= t1, t > t1 & t <= t2, t > t2 & t <= t3, t > t3, true(N, 1)});

fprintf('=== Form B slope-error accumulation (offline geometry, no measurement) ===\n');
fprintf('scenario: hold %.1fs @ w_bar %.3f -> descend %.1fs -> %g Hz osc x%d -> hold, T = %.1f s, Ts = 1/%g s\n', ...
        T_HOLD, w_bar(1), T_DESCEND, FREQUENCY, N_CYCLES, T_SIM, 1/Ts);
fprintf('commanded w_bar range [%.4f, %.4f]  (%.1fx sweep), N = %d steps\n', ...
        w_lo, w_hi, w_hi / w_lo, N);
fprintf('a_bar_true range [%.4f, %.4f];  reported %% are of a_bar_true at that instant\n\n', ...
        min(a_true), max(a_true));

% ================================================================
% 4. Direct vs integrated level error, per arm x convention
% ================================================================
store = struct();
for m = 1:numel(arms)
    b = arms(m).b; p = arms(m).p; ws = arms(m).ws;
    a_mod  = law_a(w_bar, b, p, ws, GAP_FLOOR);
    ap_mod = law_ap(w_bar, b, p, ws, GAP_FLOOR);
    e_dir  = a_mod - a_true;                        % algebraic-form error  [-]
    e_dir_pct = 100 * e_dir ./ a_true;
    slope_pct = 100 * (ap_mod - ap_true) ./ max(abs(ap_true), eps);

    fprintf('---- Form B arm %s:  b = %.4f  p = %.4f  w_s = %.4f ----\n', ...
            arms(m).name, b, p, ws);
    fprintf('  sup|a_model - a_true| on the visited envelope = %.5f (abs) = %.3f %% of a_true there\n', ...
            max(abs(e_dir)), max(abs(e_dir_pct)));
    fprintf('  slope-model error RMS over the whole run = %.3f %%   [expgain defect-1 reference: 1.65 %%]\n', ...
            sqrt(mean(slope_pct.^2)));

    for cj = 1:numel(CONV)
        A     = integrate_slope(w_bar, ap_mod, a_true(1), CONV{cj}, b, p, ws, GAP_FLOOR);
        e_int = A - a_true;
        e_int_pct = 100 * e_int ./ a_true;
        % Telescoping identity: a'(w)dw is an EXACT differential in w, so the
        % Riemann sum obeys  e_int = e_dir - e_dir[0] + disc, with disc = pure
        % quadrature residual. Anything beyond +-|e_dir[0]| is therefore NOT
        % representation error.
        disc = e_int - (e_dir - e_dir(1));

        fprintf('  [%s] %-12s | %9s %9s | %9s %9s | %7s %7s\n', CONV{cj}, 'phase', ...
                'dir pk %', 'dir RMS %', 'int pk %', 'int RMS %', 'amp pk', 'amp RMS');
        row = zeros(numel(ph), 6);
        for q = 1:numel(ph)
            msk = ph(q).mask;
            d_pk = max(abs(e_dir_pct(msk)));  d_rms = sqrt(mean(e_dir_pct(msk).^2));
            i_pk = max(abs(e_int_pct(msk)));  i_rms = sqrt(mean(e_int_pct(msk).^2));
            row(q, :) = [d_pk, d_rms, i_pk, i_rms, i_pk / max(d_pk, eps), i_rms / max(d_rms, eps)];
            fprintf('  %6s %-12s | %9.4f %9.4f | %9.4f %9.4f | %7.2f %7.2f\n', ...
                    '', ph(q).name, row(q, 1), row(q, 2), row(q, 3), row(q, 4), row(q, 5), row(q, 6));
        end
        i_osc = find(ph(3).mask); i_hold = find(ph(4).mask); i_desc = find(ph(2).mask);
        fprintf(['  %6s quadrature residual (abs a_bar): descent end %+.3e | osc end %+.3e | ' ...
                 'ratchet over osc %+.3e = %+.4f %%/cycle\n'], '', ...
                disc(i_desc(end)), disc(i_osc(end)), ...
                disc(i_osc(end)) - disc(i_osc(1)), ...
                100 * (disc(i_osc(end)) - disc(i_osc(1))) / a_true(i_osc(end)) / N_CYCLES);
        fprintf('  %6s e_dir[0] inherited offset %+.5f (%+.3f %%) | final-hold e_dir %+.4f %% e_int %+.4f %%\n', ...
                '', e_dir(1), e_dir_pct(1), mean(e_dir_pct(i_hold)), mean(e_int_pct(i_hold)));

        store.(arms(m).tag).(CONV{cj}) = struct('e_int_pct', e_int_pct, 'disc', disc, 'rows', row);
    end
    store.(arms(m).tag).e_dir_pct = e_dir_pct;
    store.(arms(m).tag).theta     = [b p ws];
    fprintf('\n');
end

% ================================================================
% 5. expgain reference arm (state-dependent slope -> ODE, not quadrature)
% ================================================================
fprintf('---- REFERENCE: expgain 7b, a_bar = 1 - w^(-b), a_bar'' = (b/w)*(1 - A) ----\n');
for bb = [B_EXPGAIN, bE]
    a_mod_e = 1 - w_bar.^(-bb);
    e_dir_e = 100 * (a_mod_e - a_true) ./ a_true;
    A = zeros(N, 1); A(1) = a_true(1);
    for k = 1:N-1
        A(k+1) = A(k) + (bb / w_bar(k)) * (1 - A(k)) * (w_bar(k+1) - w_bar(k));
    end
    e_int_e = 100 * (A - a_true) ./ a_true;
    fprintf('  b = %.4f | %-12s |  dir pk %%  dir RMS %% |  int pk %%  int RMS %% |  amp pk  amp RMS\n', ...
            bb, 'phase');
    for q = 1:numel(ph)
        msk = ph(q).mask;
        d_pk = max(abs(e_dir_e(msk))); d_rms = sqrt(mean(e_dir_e(msk).^2));
        i_pk = max(abs(e_int_e(msk))); i_rms = sqrt(mean(e_int_e(msk).^2));
        fprintf('  %10s %-12s | %9.4f %9.4f | %9.4f %9.4f | %7.2f %7.2f\n', '', ph(q).name, ...
                d_pk, d_rms, i_pk, i_rms, i_pk / max(d_pk, eps), i_rms / max(d_rms, eps));
    end
    store.(sprintf('expgain_b%d', round(1000 * bb))) = ...
        struct('e_dir_pct', e_dir_e, 'e_int_pct', e_int_e, 'b', bb);
end
fprintf('\n');

% ================================================================
% 6. Is the leftover residual really quadrature? Two independent checks
%    (anchor arm, production rule). If it is, it must (a) equal the analytic
%    forward-Euler term sum(0.5*a''*dw^2) and (b) scale linearly with Ts.
% ================================================================
b0 = B_ANCHOR; p0 = P_ANCHOR; ws0 = WS_ANCHOR;
g0 = max(w_bar - ws0, GAP_FLOOR); u0 = 1 + g0 / b0;
app_mod = -(p0 * (p0 + 1) / b0^2) * u0.^(-p0 - 2);     % d2 a_bar / d w_bar^2
dw_step = [0; diff(w_bar)];                             % production interval
resid_analytic = cumsum(0.5 * app_mod .* dw_step.^2);   % right-endpoint sign
fprintf('---- residual identification (anchor arm, production rule) ----\n');
fprintf('  analytic  0.5*sum(a''''*dw^2) at run end = %+.4e\n', resid_analytic(end));
fprintf('  measured  quadrature residual at run end = %+.4e\n', ...
        store.anchor.prod.disc(end));
for sub = [2 4]
    w_f = interp1((1:N)', w_bar, linspace(1, N, sub * (N - 1) + 1)', 'pchip');
    ap_f = law_ap(w_f, b0, p0, ws0, GAP_FLOOR);
    Nf = numel(w_f);
    Af = zeros(Nf, 1); Af(1) = a_true(1); Af(2) = Af(1);
    for k = 2:Nf-1
        Af(k+1) = Af(k) + ap_f(k) * (w_f(k) - w_f(k-1));
    end
    a_true_end = a_true(end);
    fprintf('  Ts / %d : end-of-run residual %+.4e  (ratio to Ts case %.3f; %.3f expected)\n', ...
            sub, (Af(end) - a_true_end) - (store.anchor.e_dir_pct(end) / 100 * a_true_end ...
            - store.anchor.e_dir_pct(1) / 100 * a_true(1)), ...
            ((Af(end) - a_true_end) - (store.anchor.e_dir_pct(end) / 100 * a_true_end ...
            - store.anchor.e_dir_pct(1) / 100 * a_true(1))) / store.anchor.prod.disc(end), ...
            1 / sub);
end
fprintf('\n');

% ================================================================
% 7. Figure (project style: no grid, no title, box on, legend northoutside)
% ================================================================
COL_TRUE = [0.8 0 0];         % direct / algebraic form
COL_HAT  = [0 0.2 0.9];       % integrated / differential form
COL_REF  = [0.25 0.25 0.25];  % height reference
FS = 18; LFS = 12; AXLW = 2.0;

f = figure('Position', [80 80 1100 900], 'Color', 'w', ...
           'NumberTitle', 'off', 'Visible', 'off');
tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
plot_tags = {'anchor', 'fit2'};
plot_lbl  = {'anchor', 'best fit'};
for r = 1:2
    s = store.(plot_tags{r});
    nexttile; hold on;
    hd = plot(t, s.e_dir_pct, '-', 'Color', COL_TRUE, 'LineWidth', 2.0, ...
              'DisplayName', 'direct (algebraic)');
    hp = plot(t, s.prod.e_int_pct, '-', 'Color', COL_HAT, 'LineWidth', 2.0, ...
              'DisplayName', 'integrated (production rule)');
    hl = plot(t, s.left.e_int_pct, '--', 'Color', COL_HAT, 'LineWidth', 2.0, ...
              'DisplayName', 'integrated (left-endpoint)');
    yline(0, '-', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, 'HandleVisibility', 'off');
    ymax = max(abs([s.e_dir_pct; s.prod.e_int_pct; s.left.e_int_pct]));
    xlim([0 T_SIM]); ylim(1.15 * ymax * [-1 1]);
    ylabel(sprintf('%s:  \\Deltaa/a  (%%)', plot_lbl{r}), 'FontSize', FS, 'FontWeight', 'bold');
    if r == 1
        legend([hd hp hl], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
               'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    end
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
end
nexttile; hold on;
plot(t, w_bar, '-', 'Color', COL_REF, 'LineWidth', 2.0);
xlim([0 T_SIM]);
ylabel('w_d  (R)', 'FontSize', FS, 'FontWeight', 'bold');
xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;

png_path = fullfile(res_dir, 'temp_formB_slope_accum.png');
exportgraphics(f, png_path, 'Resolution', 150);
close(f);

save(fullfile(res_dir, 'temp_formB_slope_accum.mat'), ...
     't', 'w_bar', 'a_true', 'ap_true', 'store', 'arms', 'cfg', 'th2', 'th3', 'bE');
fprintf('saved: %s\n', png_path);


%% =================== Local Helpers ===================

function a = law_a(w, b, p, ws, gap_floor)
%LAW_A  Form B gain law level, a_bar = 1 - (1 + (w - w_s)/b)^(-p)   [-]
    g = max(w - ws, gap_floor);
    a = 1 - (1 + g / b).^(-p);
end

function ap = law_ap(w, b, p, ws, gap_floor)
%LAW_AP  Form B gain law slope, d a_bar / d w_bar = (p/b)(1 + g/b)^(-p-1)  [-]
    g  = max(w - ws, gap_floor);
    u  = 1 + g / b;
    ap = (p / b) * u.^(-p - 1);
end

function A = integrate_slope(w, ap_mod, a0, conv_rule, b, p, ws, gap_floor)
%INTEGRATE_SLOPE  Propagate the gain level with the model slope alone.
%   conv_rule: 'left' | 'prod' | 'mid' (see the header block).
    N = numel(w);
    A = zeros(N, 1);
    A(1) = a0;
    switch conv_rule
        case 'left'      % slope at w[k], increment w[k+1]-w[k]
            for k = 1:N-1
                A(k+1) = A(k) + ap_mod(k) * (w(k+1) - w(k));
            end
        case 'prod'      % slope at w[k], increment w[k]-w[k-1]  (Form B production)
            A(2) = A(1);          % first call has no stored previous increment
            for k = 2:N-1
                A(k+1) = A(k) + ap_mod(k) * (w(k) - w(k-1));
            end
        case 'mid'       % second-order control arm
            wm = 0.5 * (w(1:end-1) + w(2:end));
            apm = law_ap(wm, b, p, ws, gap_floor);
            for k = 1:N-1
                A(k+1) = A(k) + apm(k) * (w(k+1) - w(k));
            end
        otherwise
            error('integrate_slope:badRule', 'unknown convention %s', conv_rule);
    end
end
