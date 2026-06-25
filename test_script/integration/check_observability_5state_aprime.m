function out = check_observability_5state_aprime(opts)
%CHECK_OBSERVABILITY_5STATE_APRIME  Formal observability of the a'-slope state.
%   Answers "is a'_x observable?" structurally (rank of the observability matrix),
%   complementing the empirical L1/L2 estimability. The per-axis EKF is
%       x[k+1] = F_e[k] x[k] + ...,   y[k] = H[k] x[k] + ...   (5 states; slot 5 = a'_x)
%   Over an N-step window the observability matrix
%       O_N(k) = [ H[k]; H[k+1]Phi(k+1,k); ...; H[k+N-1]Phi(k+N-1,k) ]
%   has rank = number of observable states. rank=5 -> a' observable; the 5th
%   singular value sigma_5 measures HOW strongly a' is observable (small = weak).
%
%   (A) Structural: at Delta_h_d=0 (hold) column 5 of O_N is zero -> rank 4
%       (a' unobservable, the null direction is slot 5); at Delta_h_d!=0 (motion)
%       a' couples to the outputs via F_e(4,5)=Delta_h_d and H(2,5)=-Delta_H_d
%       -> rank 5 (observable).
%   (B/C) Numerical: reconstruct F_e[k],H[k] along a real 1 Hz near-wall run and
%       report rank / sigma_5 / ||O(:,5)|| at hold / descent / osc / end-hold, and
%       plot the a'-observability vs |Delta_h_d| (it tracks the motion / speed).
%
%   out = check_observability_5state_aprime()
%   out = check_observability_5state_aprime(struct('axis',3,'N_win',12))
%
%   See also: build_F_e_5state_aprime, verify_eq17_5state_aprime_L1

    if nargin < 1; opts = struct(); end
    if ~isfield(opts,'frequency'); opts.frequency = 1.0; end
    if ~isfield(opts,'T_sim');     opts.T_sim = 4.0;     end
    if ~isfield(opts,'N_win');     opts.N_win = 12;      end
    if ~isfield(opts,'seed');      opts.seed = 1;        end
    if ~isfield(opts,'axis');      opts.axis = 3;        end   % z = wall-normal
    if ~isfield(opts,'make_figs'); opts.make_figs = true; end

    here = fileparts(mfilename('fullpath'));
    proj = fullfile(here, '..', '..');
    addpath(genpath(proj));
    lambda_c = 0.7; N_win = opts.N_win; ax = opts.axis;
    out = struct();

    % ============================================================
    % (A) Structural check: hold (Delta_h_d=0) vs motion (Delta_h_d!=0)
    % ============================================================
    fd_s = 1.0; F1_s = 1.7;
    Fe_hold = build_F_e_5state_aprime(lambda_c, fd_s, F1_s, 0, 0);
    H_hold  = [1 0 0 0 0; 0 0 0 1 0];
    O_hold  = obs_ti(Fe_hold, H_hold, N_win);
    Dh = 0.01; DH = 0.018; dFh = (1 - lambda_c) * 0.012;
    Fe_mov  = build_F_e_5state_aprime(lambda_c, fd_s, F1_s, dFh, Dh);
    H_mov   = [1 0 0 0 0; 0 0 0 1 -DH];
    O_mov   = obs_ti(Fe_mov, H_mov, N_win);
    [r_hold, nd_hold] = rank_and_null(O_hold);
    [r_mov,  ~]       = rank_and_null(O_mov);
    out.structural = struct('rank_hold', r_hold, 'rank_motion', r_mov, 'null_hold', nd_hold);

    fprintf('\n[A | structural]  hold (Delta_h_d=0): rank = %d/5    motion (Delta_h_d!=0): rank = %d/5\n', r_hold, r_mov);
    fprintf('   hold unobservable direction |components| on states [dx1 dx2 dx3 a_x a''_x]:\n      [%s]\n', ...
            sprintf('%.3f ', abs(nd_hold)));
    fprintf('   -> the null direction is concentrated on slot 5 (a''_x): a''_x is THE unobservable state during a hold.\n');

    % ============================================================
    % (B) Reconstruct F_e[k], H[k] along a real run (axis ax)
    % ============================================================
    cfg = build_cfg(opts.frequency, opts.T_sim);
    ro = struct('seed', opts.seed, 'verbose', false, 'collect_diag', false, 'use_true_gain', false);
    s = run_pure_simulation(cfg, ro);
    P = s.meta.params_value; w = P.wall.w_hat(:); R = P.common.R; Ts = P.common.Ts;
    fd = s.f_d_out(:, ax);                 % per-axis control force [N x1]
    hd = s.p_d_out * w;                    % wall-normal desired height [N x1]
    N  = numel(hd); t = (0:N-1)' * Ts;
    hbar = (hd - P.wall.pz) / R;

    % ============================================================
    % (C) Windowed observability along the trajectory
    % ============================================================
    kmin = 3; kmax = N - 1 - N_win;
    sig1 = nan(N,1); sig5 = nan(N,1); coln = nan(N,1); rnk = nan(N,1); Dhk = nan(N,1);
    for k0 = kmin:kmax
        Fc = cell(N_win-1, 1); Hc = cell(N_win, 1); ok = true;
        for j = 0:N_win-1
            k = k0 + j;
            if k + 1 > N || k - 2 < 1; ok = false; break; end
            Dh_k = hd(k+1) - hd(k);
            dh1  = hd(k) - hd(k-1); dh2 = hd(k) - hd(k-2); DH_k = dh2;
            F1_k = fd(k-1) + fd(k-2);
            dFh_k = (1 - lambda_c) * (dh1 * fd(k-1) + dh2 * fd(k-2));
            Hc{j+1} = [1 0 0 0 0; 0 0 0 1 -DH_k];
            if j < N_win-1
                Fc{j+1} = build_F_e_5state_aprime(lambda_c, fd(k), F1_k, dFh_k, Dh_k);
            end
        end
        if ~ok; continue; end
        O = obs_tv(Fc, Hc);
        sv = svd(O);
        sig1(k0) = sv(1); sig5(k0) = sv(5); coln(k0) = norm(O(:,5));
        rnk(k0)  = sum(sv > 1e-10 * sv(1));
        Dhk(k0)  = hd(k0+1) - hd(k0);
    end

    % ============================================================
    % (D) Report at key phases
    % ============================================================
    pnames = {'hold-far','descent','osc-vmax','osc-turn(near)','end-hold'};
    ptimes = [0.3, 0.9, 1.75, 1.5, 3.8];
    fprintf('\n[B/C | numerical, axis=%d (%s), window=%d steps]\n', ax, axis_name(ax), N_win);
    fprintf('  %-16s %5s %9s %11s %11s\n', 'phase', 'rank', '|Dh_d|', 'sig5/sig1', '||O(:,5)||');
    rep = struct();
    for i = 1:numel(pnames)
        [~, k0] = min(abs(t - ptimes(i))); k0 = max(min(k0, kmax), kmin);
        fprintf('  %-16s %5d %9.1e %11.2e %11.2e\n', pnames{i}, rnk(k0), abs(Dhk(k0)), sig5(k0)/sig1(k0), coln(k0));
        rep.(matlab.lang.makeValidName(pnames{i})) = struct('rank',rnk(k0),'Dh',abs(Dhk(k0)),'sig5_sig1',sig5(k0)/sig1(k0),'colnorm',coln(k0));
    end
    fprintf('  -> a'' observable (rank 5) whenever |Delta_h_d|>0; rank drops to 4 at holds/turning points.\n');
    fprintf('     ||O(:,5)|| (a'' observability strength) scales with |Delta_h_d| = speed: weak motion -> weak observability.\n');

    out.t = t; out.sig5 = sig5; out.sig1 = sig1; out.coln = coln; out.rnk = rnk;
    out.Dh = Dhk; out.hbar = hbar; out.phases = rep;

    if opts.make_figs
        out.fig = plot_obs(t, coln, Dhk, rnk, hbar, proj, opts.frequency, ax);
        fprintf('\n[figure] %s\n', out.fig);
    end
end


% ====================== helpers ======================

function O = obs_ti(F, H, N)
%OBS_TI  Time-invariant observability matrix [H; HF; HF^2; ...; HF^(N-1)].
    n = size(F, 1); O = []; Phi = eye(n);
    for j = 1:N
        O = [O; H * Phi]; %#ok<AGROW>
        Phi = F * Phi;
    end
end

function O = obs_tv(Fc, Hc)
%OBS_TV  Time-varying observability matrix [H1; H2 F1; H3 F2 F1; ...].
    N = numel(Hc); n = size(Hc{1}, 2); O = []; Phi = eye(n);
    for j = 1:N
        O = [O; Hc{j} * Phi]; %#ok<AGROW>
        if j < N; Phi = Fc{j} * Phi; end
    end
end

function [r, nulldir] = rank_and_null(O)
%RANK_AND_NULL  Rank (relative tol) and the weakest right singular vector.
    [~, S, V] = svd(O);
    sv = diag(S);
    r = sum(sv > 1e-10 * sv(1));
    nulldir = V(:, end);
end

function nm = axis_name(ax)
    names = {'x parallel','y parallel','z wall-normal'};
    nm = names{ax};
end

function cfg = build_cfg(f, T_sim)
%BUILD_CFG  1 Hz near-wall osc (mirrors verify_eq17_5state_aprime_L1).
    cfg = user_config();
    cfg.eq17_variant   = '5state_aprime';
    cfg.trajectory_type = 'osc';
    cfg.h_init   = 50; cfg.h_bottom = 2.7; cfg.amplitude = 2.5;
    cfg.frequency = f; cfg.n_cycles = 2 * f; cfg.t_hold = 0.5;
    cfg.t_descend_override = 1.0; cfg.T_sim = T_sim;
    pc = physical_constants(); cfg.h_min = 1.05 * pc.R;
    cfg.ctrl_enable = true; cfg.thermal_enable = true; cfg.meas_noise_enable = true;
    cfg.lambda_c = 0.7; cfg.a_pd = 0.05; cfg.a_cov = 0.05;
    cfg.meas_noise_std = [0.00062; 0.00057; 0.00331];
    cfg.suppress_xD = true; cfg.h_bar_safe = 1; cfg.use_am_lpf = false; cfg.a_det = 0.005;
    cfg.Q_aprime_factor = 10;
end

function fig_path = plot_obs(t, coln, Dh, rnk, hbar, proj, freq, ax)
%PLOT_OBS  a' observability strength + |Delta_h_d| + rank-loss + h_bar.
    out_dir = fullfile(proj, 'test_results', 'verify_5state_aprime_L1', sprintf('f%gHz', freq));
    if ~exist(out_dir, 'dir'); mkdir(out_dir); end
    FS = 16; AXLW = 1.9;
    fg = figure('Position', [60 60 1100 900], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');

    subplot(3,1,1); hold on;
    semilogy(t, max(coln, 1e-12), '-', 'Color', [0.49 0.18 0.56], 'LineWidth', 1.8);
    unobs = rnk < 5;
    plot(t(unobs), max(coln(unobs),1e-12), '.', 'Color', [0.85 0.1 0.1], 'MarkerSize', 6);
    ylabel('||O(:,5)||  (a'' obs.)', 'FontSize', FS, 'FontWeight', 'bold');
    set(gca, 'YScale', 'log', 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
    title('a'' observability (purple) — red dots = rank<5 (a'' unobservable)', 'FontSize', FS-2);

    subplot(3,1,2); hold on;
    plot(t, abs(Dh), '-', 'Color', [0.2 0.5 0.2], 'LineWidth', 1.8);
    ylabel('|\Delta h_d|', 'FontSize', FS, 'FontWeight', 'bold');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;

    subplot(3,1,3); hold on;
    plot(t, hbar, '-', 'Color', [0.25 0.25 0.25], 'LineWidth', 1.8, 'HandleVisibility', 'off');
    yline(1.8, 'r--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
    ylabel('h / R', 'FontSize', FS, 'FontWeight', 'bold'); xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;

    fig_path = fullfile(out_dir, sprintf('fig_observability_aprime_ax%d_f%gHz.png', ax, freq));
    exportgraphics(fg, fig_path, 'Resolution', 150); close(fg);
end
