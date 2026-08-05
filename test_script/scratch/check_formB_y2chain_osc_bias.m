%CHECK_FORMB_Y2CHAIN_OSC_BIAS  Does the a_xm readout chain gain a MEAN BIAS
%   when the true gain oscillates?  (defect-3 sub-derivation A)
%
%   PURPOSE. The ledger attributes a persistent POSITIVE mean y2 innovation
%   (readout reads high vs the filter prediction) to the readout chain being
%   derived for a STATIC gain. This script tests that hypothesis OFFLINE, with
%   the filter taken out of the loop, by replicating the chain exactly and
%   driving it with a synthetic ground truth under two conditions that share
%   noise seeds:
%       (a) static gain at the osc-window mean height  w_bar = 3.111
%       (b) the canonical 1 Hz osc  w_bar(t) = 3.111 - 1.111*cos(2*pi*t)
%   Bias is always measured against the SAME smoothing applied to the true
%   gain, so no smoothing/lag artefact can masquerade as bias. Both windows
%   are an integer number of cycles, which makes the comparison insensitive
%   to the choice of reference kernel (any unity-DC-gain kernel preserves the
%   cycle mean).
%
%   PRE-REGISTERED VERDICT (from the task charter)
%       chain-side       bias(osc) - bias(static) = +1e-3 .. +3e-3  [-]
%       prediction-side  |bias(osc) - bias(static)| < 0.5e-3        [-]
%
%   WHAT IS REPLICATED (production code, cited)
%       readout chain      model/controller/motion_control_law_formB_ws.m:729-732
%       C_dpmr / C_n       model/controller/build_eq17_6state_constants.m:107-114
%                          (FULL a_pd form: 3.1610 / 1.1093 at lc=.7, a_pd=.05)
%       kappa_T, sigma2_n  motion_control_law_formB_ws.m:291-296
%       eps_w MA(2) law    motion_control_law_formB_ws.m:96-97
%       control law        motion_control_law_formB_ws.m:747-758
%       canonical config   test_script/integration/run_formB_ws.m:438-462
%       osc trajectory     model/trajectory/trajectory_generator.m:113-116
%
%   STAGES (env Y2CHAIN_STAGES, default '1,2,3,6'; results merge into
%   test_results/formB_y2chain_osc_bias.mat so stages can be run separately)
%       1  cross-validate the closed-form MA(2) replica against an explicit
%          simulation of the control law itself (same seed, must agree)
%       2  high-N chain bias: static vs osc, analytic replica
%       3  gain-mismatch (self-echo) sensitivity: static vs osc, explicit loop
%       4  real plant + real controller, a_ctrl_override = 'true' (filter out
%          of the loop, RK4 plant): static vs osc
%       5  real production run (a_ctrl = a_hat): a_xm vs smoothed truth in the
%          two late windows + mean innovation audit
%       6  figure
%
%   Usage:  Y2CHAIN_STAGES=1,2,3,6 matlab -batch "run('.../check_formB_y2chain_osc_bias.m')"

clear; clc;
here = fileparts(mfilename('fullpath'));
proj = fileparts(fileparts(here));
addpath(genpath(fullfile(proj, 'model')));
addpath(fullfile(proj, 'test_script', 'integration'));

stage_env = getenv('Y2CHAIN_STAGES');
if isempty(stage_env); stage_env = '1,2,3,6'; end
stages = str2double(strsplit(stage_env, ','));
res_dir  = fullfile(proj, 'test_results');
if ~exist(res_dir, 'dir'); mkdir(res_dir); end
suffix   = getenv('Y2CHAIN_SUFFIX');           % lets stages run in parallel
res_file = fullfile(res_dir, ['formB_y2chain_osc_bias' suffix '.mat']);
if exist(res_file, 'file'); S = load(res_file); RES = S.RES; else; RES = struct(); end

% ---------------------------------------------------------------------
% Canonical constants (no magic numbers: every value from the production
% builders / configs cited in the header)
% ---------------------------------------------------------------------
pc   = physical_constants();
R    = pc.R;                                   % [um]
Ts   = pc.Ts;                                  % [s]
fs   = 1 / Ts;                                 % [Hz]
LAMBDA_C = 0.7;                                % run_formB_ws.m:457
A_PD     = 0.05;                               % run_formB_ws.m:458
A_COV    = 0.05;                               % run_formB_ws.m:459
D_DELAY  = 2;                                  % run_formB_ws.m:791 (eq17_opts.d)
MEAS_STD = [0.00062; 0.00057; 0.00331];        % [um] run_formB_ws.m:460
AX_Z     = 3;

cc = build_eq17_6state_constants(struct( ...
        'lambda_c', LAMBDA_C, 'option', 'A_MA2_full', ...
        'sigma2_n_s', MEAS_STD.^2, 'kBT', pc.k_B * pc.T, 'd', D_DELAY, ...
        'a_cov', A_COV, 'a_pd', A_PD, 't_warmup_kf', 0, 'h_bar_safe', 1.5, ...
        'iir_warmup_mode', 'prefill'));
C_DPMR = cc.C_dpmr;                            % [-]
C_N    = cc.C_n;                               % [-]

a_o     = Ts / (pc.gamma_N * R);               % [1/pN]  controller:293
KAPPA_T = 4 * (pc.k_B * pc.T / R) * a_o;       % [-]     controller:295
A_DISP  = a_o * R;                             % [um/pN] controller:296
S2_N    = MEAS_STD(AX_Z)^2 / R^2;              % [-]     controller:291
ALPHA   = 1 - LAMBDA_C;                        % MA(2) coefficient [-]
S_ECHO  = 0.32;                                % y2 self-echo share, controller:355-365
                                               % (STAGE 3 measures 0.324 independently)

% canonical osc scenario, wall-normal axis
W_CENTER = (4.5 + 2.5) / R;                    % [-] h_bottom + amplitude
W_AMP    = 2.5 / R;                            % [-] half-amplitude
F_OSC    = 1;                                  % [Hz]
A_BAR_STATIC = 1 / local_cperp(W_CENTER);      % [-] gain at the mean height
N_CYC_SAMP   = round(fs / F_OSC);              % 1600 samples per osc cycle
t_cyc        = (0:N_CYC_SAMP-1)' * Ts;
w_cyc        = W_CENTER - W_AMP * cos(2*pi*F_OSC*t_cyc);   % trajectory_generator:116
a_cyc        = 1 ./ local_cperp(w_cyc);        % [-] one period of the true gain

fprintf('=== canonical constants (all from production builders) ===\n');
fprintf('C_dpmr %.4f  C_n %.4f  kappa_T %.6e  sigma2_n_nd %.6e\n', ...
        C_DPMR, C_N, KAPPA_T, S2_N);
fprintf('a_disp %.6f um/pN   lambda_c %.2f  a_pd %.2f  a_cov %.2f  d %d\n', ...
        A_DISP, LAMBDA_C, A_PD, A_COV, D_DELAY);
fprintf('w_bar: center %.4f  amp %.4f  ->  a_bar(center) %.6f  ', ...
        W_CENTER, W_AMP, A_BAR_STATIC);
fprintf('a_bar range [%.4f %.4f]\n', 1/local_cperp(W_CENTER + W_AMP), ...
        1/local_cperp(W_CENTER - W_AMP));

% ---------------------------------------------------------------------
% STAGE 1 -- closed-form MA(2) replica vs explicit control-law simulation
% ---------------------------------------------------------------------
if any(stages == 1)
    fprintf('\n=== STAGE 1: MA(2) closed form vs explicit control-law loop ===\n');
    N_CYC1 = 60;  N1 = N_CYC1 * N_CYC_SAMP;
    for cond = 1:2
        if cond == 1
            a_bar = A_BAR_STATIC * ones(N1, 1);  w_bar_d = W_CENTER * ones(N1, 1);
            nm = 'static';
        else
            w_bar_d = repmat(w_cyc, N_CYC1, 1);
            a_bar   = repmat(a_cyc, N_CYC1, 1);
            nm = 'osc   ';
        end
        rng(101, 'twister');
        zT = randn(N1, 1);  zn = randn(N1, 1);
        dwm_a = local_loop_analytic(a_bar, zT, zn, KAPPA_T, S2_N, LAMBDA_C, D_DELAY);
        dwm_e = local_loop_explicit(a_bar, w_bar_d, zT, zn, KAPPA_T, S2_N, ...
                                    LAMBDA_C, D_DELAY, ones(N1, 1));
        k0 = round(5 * fs);
        d1 = dwm_a(k0:end);  d2 = dwm_e(k0:end);
        cm = corrcoef(d1, d2);
        fprintf('%s : std(analytic) %.6e  std(explicit) %.6e  ratio %.5f  corr %.6f\n', ...
                nm, std(d1), std(d2), std(d2)/std(d1), cm(1,2));
        b_a = local_chain_from_dwm(dwm_a, a_bar, k0, D_DELAY, A_PD, A_COV, ...
                                   C_DPMR, C_N, KAPPA_T, S2_N);
        b_e = local_chain_from_dwm(dwm_e, a_bar, k0, D_DELAY, A_PD, A_COV, ...
                                   C_DPMR, C_N, KAPPA_T, S2_N);
        fprintf('         chain bias: analytic %+.4e   explicit %+.4e  [-]\n', b_a, b_e);
        % Deterministic leak: the only way the a_pd mean-subtraction can inject
        % trajectory power into the variance estimator is a non-zero
        % deterministic part of dw_m. The eq17 feedforward drives the particle
        % onto the DELAYED reference w_d[k-d], so this should vanish.
        dwm_0 = local_loop_explicit(a_bar, w_bar_d, zeros(N1,1), zeros(N1,1), ...
                                    KAPPA_T, S2_N, LAMBDA_C, D_DELAY, ones(N1,1));
        det_rms = sqrt(mean(local_win(dwm_0, k0).^2));
        noise_rms = std(local_win(dwm_e, k0));
        fprintf('         deterministic dw_m RMS %.3e  (noise RMS %.3e, ratio %.2e)\n', ...
                det_rms, noise_rms, det_rms/noise_rms);
        fprintf('         -> variance it could add: %.3e of %.3e  (%.4f%% of readout)\n', ...
                det_rms^2, C_DPMR*KAPPA_T*mean(a_bar) + C_N*S2_N, ...
                100*det_rms^2/(C_DPMR*KAPPA_T*mean(a_bar) + C_N*S2_N));
    end
end

% ---------------------------------------------------------------------
% STAGE 2 -- high-N chain bias, static vs osc (analytic replica)
% ---------------------------------------------------------------------
if any(stages == 2)
    fprintf('\n=== STAGE 2: chain bias, static vs osc (analytic, ideal loop) ===\n');
    N_SEED = 20;  N_CYC2 = 400;                 % whole cycles -> aligned window
    N2 = N_CYC2 * N_CYC_SAMP;  T_SEED = N2 / fs;  BURN = round(5 * fs);
    a_osc = repmat(a_cyc, N_CYC2, 1);
    a_sta = A_BAR_STATIC * ones(N2, 1);
    w_osc = repmat(w_cyc, N_CYC2, 1);
    w_sta = W_CENTER * ones(N2, 1);
    ref_sta = mean(local_win(local_ref(a_sta, D_DELAY, A_COV), BURN));   % [-]
    ref_osc = mean(local_win(local_ref(a_osc, D_DELAY, A_COV), BURN));
    b_sta = zeros(N_SEED, 2);  b_osc = zeros(N_SEED, 2);   % col 1 analytic, 2 explicit
    tic;
    for q = 1:N_SEED
        rng(1000 + q, 'twister');
        zT = randn(N2, 1);  zn = randn(N2, 1);   % common random numbers
        da = local_loop_analytic(a_sta, zT, zn, KAPPA_T, S2_N, LAMBDA_C, D_DELAY);
        de = local_loop_explicit(a_sta, w_sta, zT, zn, KAPPA_T, S2_N, LAMBDA_C, ...
                                 D_DELAY, ones(N2,1));
        b_sta(q,1) = local_chain_from_dwm(da, a_sta, BURN, D_DELAY, A_PD, A_COV, C_DPMR, C_N, KAPPA_T, S2_N);
        b_sta(q,2) = local_chain_from_dwm(de, a_sta, BURN, D_DELAY, A_PD, A_COV, C_DPMR, C_N, KAPPA_T, S2_N);
        da = local_loop_analytic(a_osc, zT, zn, KAPPA_T, S2_N, LAMBDA_C, D_DELAY);
        de = local_loop_explicit(a_osc, w_osc, zT, zn, KAPPA_T, S2_N, LAMBDA_C, ...
                                 D_DELAY, ones(N2,1));
        b_osc(q,1) = local_chain_from_dwm(da, a_osc, BURN, D_DELAY, A_PD, A_COV, C_DPMR, C_N, KAPPA_T, S2_N);
        b_osc(q,2) = local_chain_from_dwm(de, a_osc, BURN, D_DELAY, A_PD, A_COV, C_DPMR, C_N, KAPPA_T, S2_N);
        fprintf('  seed %2d: [analytic] sta %+.3e osc %+.3e | [explicit] sta %+.3e osc %+.3e  (%.0fs)\n', ...
                q, b_sta(q,1), b_osc(q,1), b_sta(q,2), b_osc(q,2), toc);
    end
    st2 = struct();
    for m = 1:2
        nm = {'STAGE 2 analytic MA(2) loop', 'STAGE 2 explicit control-law loop'};
        r = local_report(nm{m}, b_sta(:,m), b_osc(:,m), ref_sta, ref_osc);
        if m == 1; st2.analytic = r; else; st2.explicit = r; end
    end
    st2.b_sta = b_sta; st2.b_osc = b_osc; st2.T_seed = T_SEED; st2.n_seed = N_SEED;
    st2.ref_sta = ref_sta; st2.ref_osc = ref_osc;
    RES.stage2 = st2;  save(res_file, 'RES');
end

% ---------------------------------------------------------------------
% STAGE 3 -- self-echo sensitivity: is dReadout/d(gain error) the same
%            static and oscillating?  (explicit control-law loop)
% ---------------------------------------------------------------------
if any(stages == 3)
    fprintf('\n=== STAGE 3: gain-mismatch (self-echo) sensitivity ===\n');
    N_SEED = 10;  N_CYC3 = 200;  E_LIST = [-0.02, 0, 0.02];   % a_ctrl = (1+e)*a_true
    N3 = N_CYC3 * N_CYC_SAMP;  T_SEED = N3 / fs;  BURN = round(5 * fs);
    w_osc = repmat(w_cyc, N_CYC3, 1);
    a_osc = repmat(a_cyc, N_CYC3, 1);
    a_sta = A_BAR_STATIC * ones(N3, 1);
    w_sta = W_CENTER * ones(N3, 1);
    B = zeros(N_SEED, numel(E_LIST), 2);        % seed x e x {static, osc}
    tic;
    for q = 1:N_SEED
        rng(2000 + q, 'twister');
        zT = randn(N3, 1);  zn = randn(N3, 1);
        for ie = 1:numel(E_LIST)
            e = E_LIST(ie);
            for cond = 1:2
                if cond == 1; a_bar = a_sta; w_d = w_sta; else; a_bar = a_osc; w_d = w_osc; end
                dwm = local_loop_explicit(a_bar, w_d, zT, zn, KAPPA_T, S2_N, ...
                                          LAMBDA_C, D_DELAY, (1+e)*ones(N3,1));
                B(q, ie, cond) = local_chain_from_dwm(dwm, a_bar, BURN, ...
                      D_DELAY, A_PD, A_COV, C_DPMR, C_N, KAPPA_T, S2_N);
            end
        end
        fprintf('  seed %2d done (%.0fs)\n', q, toc);
    end
    st3 = struct('E_LIST', E_LIST, 'B', B, 'n_seed', N_SEED, 'T_seed', T_SEED);
    i0 = find(E_LIST == 0);
    fprintf('\n%-10s %14s %14s %14s\n', 'e (a_ctrl)', 'static bias', 'osc bias', 'osc-static');
    for ie = 1:numel(E_LIST)
        ds = B(:, ie, 1);  do_ = B(:, ie, 2);
        fprintf('%+9.3f %8.3e+-%.0e %8.3e+-%.0e %8.3e+-%.0e\n', E_LIST(ie), ...
                mean(ds), local_sem(ds), mean(do_), local_sem(do_), ...
                mean(do_-ds), local_sem(do_-ds));
    end
    for cond = 1:2
        sl = (mean(B(:, end, cond)) - mean(B(:, 1, cond))) / (E_LIST(end) - E_LIST(1));
        nm = {'static', 'osc'};
        fprintf('d(readout bias)/d(e)  %-7s = %+.4f  [-] per unit gain error\n', nm{cond}, sl);
    end
    fprintf('  (reference: a_bar = %.4f, so a pure echo of the applied gain gives -%.4f)\n', ...
            A_BAR_STATIC, A_BAR_STATIC);
    st3.i0 = i0;
    RES.stage3 = st3;  save(res_file, 'RES');
end

% ---------------------------------------------------------------------
% STAGE 4 -- real plant (RK4) + real controller, filter OUT of the loop
% ---------------------------------------------------------------------
if any(stages == 4)
    fprintf('\n=== STAGE 4: real plant, a_ctrl_override = ''true'' ===\n');
    SEEDS = 1:16;  N_CYC = 32;  T_SIM = 34.8;
    WIN = [13.5, 33.5];                         % [s] 20 cycles inside the osc phase
    b4 = zeros(numel(SEEDS), 2);  ref4 = zeros(1, 2);
    for cond = 1:2
        cfgo = struct('n_cycles', N_CYC, 'T_sim', T_SIM);
        if cond == 1                            % static hold at w_bar = 3.111
            cfgo.h_bottom = 7.0; cfgo.amplitude = 0;
        end
        o = struct('seeds', SEEDS, 'verbose', false, 'a_ctrl_override', 'true', ...
                   'config_override', cfgo, ...
                   'ctrl_const_override', struct('lock_b', true, 'lock_p', true, ...
                                                 'lock_ws', true));
        out = run_formB_ws(o);
        for q = 1:numel(SEEDS)
            r = out.runs{q};
            [b4(q, cond), ref4(cond)] = local_bias_from_run(r, WIN, AX_Z, A_DISP, ...
                                                            D_DELAY, A_COV, Ts);
        end
    end
    st4 = local_report('STAGE 4 (real RK4 plant, a_ctrl = a_true)', b4(:,1), b4(:,2), ...
                       ref4(1), ref4(2));
    st4.b = b4;  st4.seeds = SEEDS;  st4.win = WIN;  st4.ref = ref4;
    RES.stage4 = st4;  save(res_file, 'RES');
end

% ---------------------------------------------------------------------
% STAGE 5 -- production run: a_xm vs smoothed truth + innovation audit
% ---------------------------------------------------------------------
if any(stages == 5)
    fprintf('\n=== STAGE 5: production run (a_ctrl = a_hat), late windows ===\n');
    SEEDS = 1:8;
    o = struct('seeds', SEEDS, 'ws_inject', 0.05, 'verbose', false, ...
               'ctrl_const_override', struct('lock_b', false, 'lock_p', true, ...
                                             'lock_ws', false, 'Pf_ws_std', 0.111), ...
               'config_override', struct('n_cycles', 32, 'T_sim', 34.8));
    out = run_formB_ws(o);
    WINS = {[13.5, 23.5], [23.5, 33.5]};
    n = numel(SEEDS);  nw = numel(WINS);
    B = zeros(n, nw, 8);   % see lbl below
    for q = 1:n
        r = out.runs{q};
        for iw = 1:nw
            B(q, iw, :) = local_y2_budget(r, WINS{iw}, AX_Z, A_DISP, D_DELAY, ...
                                          A_COV, Ts, S_ECHO);
        end
    end
    lbl = {'readout - truth', 'estimate - truth', 'innov_y2 / a_cov', ...
           'echo back-off term', 'closure residual', 'gated fraction', ...
           'y2 total push on a_hat', 'a_hat drift over window'};
    for iw = 1:nw
        fprintf('\n--- window [%.1f, %.1f) s, %d seeds ---\n', WINS{iw}(1), WINS{iw}(2), n);
        for j = 1:numel(lbl)
            v = B(:, iw, j);
            fprintf('  %-20s %+.4e +- %.2e  [-]\n', lbl{j}, mean(v), local_sem(v));
        end
        fprintf('  identity: (readout-truth) - (estimate-truth) + backoff = innov/a_cov\n');
    end
    RES.stage5 = struct('B', B, 'labels', {lbl}, 'seeds', SEEDS, 'wins', {WINS}, ...
                        'a_cov', A_COV, 's_echo', S_ECHO);
    save(res_file, 'RES');
end

% ---------------------------------------------------------------------
% STAGE 6 -- figure
% ---------------------------------------------------------------------
if any(stages == 6)
    fprintf('\n=== STAGE 6: figure ===\n');
    N6 = 3 * N_CYC_SAMP;  PRE = 20 * N_CYC_SAMP;   % 20 cycles of EWMA warm-up
    a6 = repmat(a_cyc, 23, 1);
    rng(7, 'twister');
    zT = randn(N6+PRE, 1);  zn = randn(N6+PRE, 1);
    dwm = local_loop_analytic(a6, zT, zn, KAPPA_T, S2_N, LAMBDA_C, D_DELAY);
    awm = local_chain(dwm, A_PD, A_COV, C_DPMR, C_N, KAPPA_T, S2_N, a6(1));
    ref = local_ref(a6, D_DELAY, A_COV);
    idx = PRE + (1:N6);
    tt  = (0:N6-1)' * Ts;

    fig = figure('Position', [100 100 1000 900], 'Color', 'w');
    tl = tiledlayout(fig, 2, 1, 'Padding', 'compact', 'TileSpacing', 'compact');

    ax1 = nexttile(tl);
    plot(ax1, tt, awm(idx), 'Color', [0.55 0.78 0.93], 'LineWidth', 0.5); hold(ax1, 'on');
    % centered moving average (zero phase) so the comparison is lag-free
    plot(ax1, tt, movmean(awm(idx), round(0.1*fs)), 'b-', 'LineWidth', 2.5);
    plot(ax1, tt, ref(idx), 'r-', 'LineWidth', 2.5);
    box(ax1, 'on'); grid(ax1, 'off');
    xlabel(ax1, 'Time [s]'); ylabel(ax1, 'Normalized gain a_{bar} [-]');
    ylim(ax1, [0 1.6]);
    set(ax1, 'FontSize', 18, 'FontWeight', 'bold', 'LineWidth', 1.2);
    lg = legend(ax1, {'Readout a_{xm} (replica)', 'Readout, 0.1 s centered mean', ...
                      'True gain (chain smoothing)'}, ...
                'Orientation', 'horizontal', 'Location', 'northoutside');
    set(lg, 'FontSize', 13, 'FontWeight', 'bold');

    ax2 = nexttile(tl);
    if isfield(RES, 'stage2')
        n2 = RES.stage2.n_seed;  q = (1:n2)';
        d_an = RES.stage2.b_osc(:,1) - RES.stage2.b_sta(:,1);
        d_ex = RES.stage2.b_osc(:,2) - RES.stage2.b_sta(:,2);
        rm_an = cumsum(d_an) ./ q;  rm_ex = cumsum(d_ex) ./ q;
        sem_r = arrayfun(@(k) std(d_ex(1:k))/sqrt(k), q);  sem_r(1) = NaN;
        hold(ax2, 'on');
        hb = fill(ax2, [q; flipud(q)], [rm_ex-sem_r; flipud(rm_ex+sem_r)], ...
                  [0.55 0.78 0.93], 'EdgeColor', 'none', 'FaceAlpha', 0.5);
        h4 = plot(ax2, [1 n2], [1e-3 1e-3], 'k--', 'LineWidth', 2);
        plot(ax2, [1 n2], [3e-3 3e-3], 'k--', 'LineWidth', 2);
        plot(ax2, [1 n2], [0 0], 'k:', 'LineWidth', 1.5);
        h1 = plot(ax2, q, rm_ex, 'b-', 'LineWidth', 3);
        h2 = plot(ax2, q, rm_an, 'r--', 'LineWidth', 2.5);
        xlim(ax2, [1 n2]);  ylim(ax2, [-1e-3 3.5e-3]);
        xlabel(ax2, sprintf('Seeds averaged (%g s each)', RES.stage2.T_seed));
        ylabel(ax2, 'Motion-induced bias [-]');
        lg2 = legend(ax2, [h1 h2 hb h4], {'explicit control-law loop', ...
                     'closed-form MA(2) loop', '\pm 1 SEM', ...
                     'pre-registered chain-side band'}, ...
                     'Orientation', 'horizontal', 'Location', 'northoutside');
        set(lg2, 'FontSize', 12, 'FontWeight', 'bold');
    end
    box(ax2, 'on'); grid(ax2, 'off');
    set(ax2, 'FontSize', 18, 'FontWeight', 'bold', 'LineWidth', 1.2);

    set(findall(fig, 'Type', 'axes'), 'Toolbar', []);
    fig_path = fullfile(res_dir, 'formB_y2chain_osc_bias.png');
    exportgraphics(fig, fig_path, 'Resolution', 150);
    fprintf('figure: %s\n', fig_path);
end

fprintf('\nresults: %s\n', res_file);


%% =================== Local helpers ===================

function c = local_cperp(w_bar)
%LOCAL_CPERP  Plant-truth perpendicular correction c_perp(w_bar), vectorized.
    c = zeros(size(w_bar));
    for i = 1:numel(w_bar)
        [~, c(i)] = calc_correction_functions(w_bar(i));
    end
end

function dwm = local_loop_analytic(a_bar, zT, zn, kT, s2n, lc, d)
%LOCAL_LOOP_ANALYTIC  Ideal closed loop in closed form (controller:96-97).
%   dw_1[k+1] = lc*dw_1[k] + eps_w[k],
%   eps_w[k]  = w_T[k] + alpha*(w_T[k-1]+w_T[k-2]) - alpha*n_w[k-d]
%   dw_m[k]   = dw_1[k] + n_w[k]                                        [-]
    alpha = 1 - lc;
    N  = numel(zT);
    wT = sqrt(kT * a_bar) .* zT;                 % [-] thermal step
    nw = sqrt(s2n) * zn;                         % [-] measurement noise
    eps_w = wT + alpha * ([0; wT(1:end-1)] + [0; 0; wT(1:end-2)]) ...
            - alpha * [zeros(d,1); nw(1:end-d)];
    dw1 = filter(1, [1 -lc], [0; eps_w(1:end-1)]);
    dwm = dw1 + nw;
    dwm = dwm(1:N);
end

function dwm = local_loop_explicit(a_bar, w_bar_d, zT, zn, kT, s2n, lc, d, g_ctrl)
%LOCAL_LOOP_EXPLICIT  Explicit eq17 control law on a linear plant, wall-normal
%   axis only, everything normalized by R (controller:747-758). g_ctrl is the
%   applied-gain factor: a_ctrl = g_ctrl * a_true (1 = ideal loop).
    N   = numel(zT);
    alpha = 1 - lc;
    wT  = sqrt(kT * a_bar) .* zT;
    nw  = sqrt(s2n) * zn;
    w   = w_bar_d(1);                 % true normalized position
    wm_buf = repmat(w, d+1, 1);       % measured position, d-step delay line
    fb1 = 0; fb2 = 0; ac1 = a_bar(1); ac2 = a_bar(1);
    dwm = zeros(N, 1);
    for k = 1:N
        wd_k   = w_bar_d(k);
        wd_kp1 = w_bar_d(min(k+1, N));
        wd_kmd = w_bar_d(max(k-d, 1));
        wm_d   = wm_buf(1);
        dwm(k) = wd_kmd - wm_d;
        a_ctrl = g_ctrl(k) * a_bar(k);
        traj   = wd_kp1 - lc*wd_k - alpha*wd_kmd;
        fb     = (traj + alpha*dwm(k) - alpha*(ac1*fb1 + ac2*fb2)) / a_ctrl;
        w      = w + a_bar(k)*fb + wT(k);         % linear plant
        wm_buf = [wm_buf(2:end); w + nw(k)];
        fb2 = fb1; fb1 = fb;  ac2 = ac1; ac1 = a_ctrl;
    end
end

function y = local_ewma(x, a)
%LOCAL_EWMA  y[k] = (1-a)*y[k-1] + a*x[k], started at x(1).
    y = filter(a, [1 -(1-a)], x, (1-a)*x(1));
end

function awm = local_chain(dwm, a_pd, a_cov, C_dpmr, C_n, kT, s2n, a_seed)
%LOCAL_CHAIN  The production readout chain, verbatim (controller:729-732).
    m   = local_ewma(dwm, a_pd);
    dwr = dwm - m;
    s2_seed = C_dpmr*kT*a_seed + C_n*s2n;         % 'prefill' init (controller:611)
    s2  = filter(a_cov, [1 -(1-a_cov)], dwr.^2, (1-a_cov)*s2_seed);
    awm = (s2 - C_n*s2n) / (C_dpmr*kT);
end

function ref = local_ref(a_bar, d, a_cov)
%LOCAL_REF  Same delay + EWMA smoothing applied to the TRUE gain.
    ad  = [repmat(a_bar(1), d, 1); a_bar(1:end-d)];
    ref = local_ewma(ad, a_cov);
end

function y = local_win(x, burn)
%LOCAL_WIN  Analysis window: drop the burn-in, keep a whole number of cycles.
    y = x(burn+1:end);
end

function bias = local_chain_from_dwm(dwm, a_bar, burn, d, a_pd, a_cov, ...
                                     C_dpmr, C_n, kT, s2n)
    awm = local_chain(dwm, a_pd, a_cov, C_dpmr, C_n, kT, s2n, a_bar(1));
    ref = local_ref(a_bar, d, a_cov);
    bias = mean(local_win(awm, burn) - local_win(ref, burn));
end

function [bias, ref_mean] = local_bias_from_run(r, win, ax, a_disp, d, a_cov, Ts)
%LOCAL_BIAS_FROM_RUN  Readout minus identically smoothed truth, from a real run.
    a_bar_true = r.a_true_out(:, ax) / a_disp;
    a_bar_wm   = r.a_xm_out(:, ax) / a_disp;
    ref = local_ref(a_bar_true, d, a_cov);
    t   = (0:numel(ref)-1)' * Ts;
    m   = t >= win(1) & t < win(2);
    bias = mean(a_bar_wm(m) - ref(m));
    ref_mean = mean(ref(m));
end

function v = local_y2_budget(r, win, ax, a_disp, d, a_cov, Ts, s_echo)
%LOCAL_Y2_BUDGET  Reconstruct the y2 innovation from the log (two-step audit).
%   With y2 = a_wm[k] - (1-a_cov)*a_wm[k-1] and
%   y2_pred = a_cov*(a_hat_preY2 - (1-S)*a_bar'*Grad_wbar_d), a cycle-aligned
%   window gives   innov/a_cov = (a_wm - truth) - (a_hat - truth) + backoff.
    a_true = r.a_true_out(:, ax) / a_disp;          % [-] plant truth
    a_wm   = r.a_xm_out(:, ax)   / a_disp;          % [-] raw readout
    a_hat  = r.a_hat_out(:, ax)  / a_disp;          % [-] posterior estimate
    a_pr   = r.a_prime_out(:, ax) / a_disp;         % [-] d a_bar / d w_bar
    ref    = local_ref(a_true, d, a_cov);
    % estimate BEFORE the y2 update (the innovation's prediction point)
    a_pre  = a_hat - r.K_a_y2_out(:, ax) .* r.innov_y2_out(:, ax);
    wd     = r.h_bar_d_out(:);                       % [-] desired height
    grad   = wd - [repmat(wd(1), d, 1); wd(1:end-d)];
    backoff = (1 - s_echo) * a_pr .* grad;
    t = (0:numel(ref)-1)' * Ts;
    m = t >= win(1) & t < win(2);
    % y2's total contribution to a_hat over the window, and the drift that
    % survives it (first cycle mean vs last cycle mean of a_hat - truth)
    push = sum(r.K_a_y2_out(m, ax) .* r.innov_y2_out(m, ax));
    e_a  = a_hat - ref;  idx = find(m);
    ncyc = round(1 / Ts);                                  % 1 s = one cycle
    drift = mean(e_a(idx(end-ncyc+1:end))) - mean(e_a(idx(1:ncyc)));
    v = [mean(a_wm(m) - ref(m)); ...
         mean(a_pre(m) - ref(m)); ...
         mean(r.innov_y2_out(m, ax)) / a_cov; ...
         mean(backoff(m)); ...
         mean(a_wm(m) - a_pre(m) + backoff(m)) - mean(r.innov_y2_out(m, ax))/a_cov; ...
         mean(r.gate_out(m, ax)); ...
         push; ...
         drift];
end

function s = local_sem(x)
    s = std(x) / sqrt(numel(x));
end

function st = local_report(tag, b_sta, b_osc, ref_sta, ref_osc)
%LOCAL_REPORT  Absolute bias (the pre-registered metric) plus the multiplicative
%   readout gain b/ref, which separates a pure scale miscalibration of the chain
%   (same factor both conditions) from a genuinely motion-induced bias.
    d  = b_osc - b_sta;                       % [-] absolute, pre-registered
    ms = b_sta / ref_sta;  mo = b_osc / ref_osc;    % relative
    dm = mo - ms;
    dm_abs = dm * ref_osc;                    % motion-induced part, absolute
    fprintf('\n--- %s ---\n', tag);
    fprintf('reference gain   static %.6f   osc %.6f  [-]  (Jensen gap %+.4e)\n', ...
            ref_sta, ref_osc, ref_osc - ref_sta);
    fprintf('static bias      %+.4e +- %.2e  [-]   (%+.3f%% of reference)\n', ...
            mean(b_sta), local_sem(b_sta), 100*mean(ms));
    fprintf('osc bias         %+.4e +- %.2e  [-]   (%+.3f%%)\n', ...
            mean(b_osc), local_sem(b_osc), 100*mean(mo));
    fprintf('osc - static     %+.4e +- %.2e  [-]   <- pre-registered metric\n', ...
            mean(d), local_sem(d));
    fprintf('  of which scale-times-Jensen %+.4e ; MOTION-INDUCED %+.4e +- %.2e\n', ...
            mean(ms)*(ref_osc-ref_sta), mean(dm_abs), local_sem(dm_abs));
    if mean(d) - 2*local_sem(d) > 0.5e-3
        v = 'CHAIN-SIDE (motion-induced positive bias confirmed)';
    elseif abs(mean(d)) + 2*local_sem(d) < 0.5e-3
        v = 'PREDICTION-SIDE (chain carries no motion-induced bias)';
    else
        v = 'INCONCLUSIVE (need more samples)';
    end
    fprintf('VERDICT: %s\n', v);
    st = struct('tag', tag, 'mean_static', mean(b_sta), 'sem_static', local_sem(b_sta), ...
                'mean_osc', mean(b_osc), 'sem_osc', local_sem(b_osc), ...
                'mean_diff', mean(d), 'sem_diff', local_sem(d), ...
                'rel_static', mean(ms), 'rel_osc', mean(mo), ...
                'mean_motion', mean(dm_abs), 'sem_motion', local_sem(dm_abs), ...
                'ref_static', ref_sta, 'ref_osc', ref_osc, 'verdict', v);
end
