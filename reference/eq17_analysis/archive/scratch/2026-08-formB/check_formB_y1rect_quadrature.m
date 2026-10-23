% check_formB_y1rect_quadrature.m -- PURPOSE: sub-derivation B of the
%   "defect 3" case. In a LONG run (n_cycles 32, T 34.8 s) the w_s update fed
%   by the y1 (position) channel is RECTIFIED negative: mean(K)*mean(nu) ~ 0
%   yet cov(K_ws_y1, nu1) ~ -3.3e-3 /s. K_ws_y1 is phase-locked to the 1 Hz
%   oscillation, and its IN-PHASE projection on the commanded force gives a
%   POSITIVE product -- so the negative drive must sit in the QUADRATURE part
%   or in a harmonic. This script decomposes E[K1.*nu1] exactly (Parseval, so
%   the bands sum to the measured mean) into DC / LF / 1 Hz / 2 Hz / 3-5 Hz /
%   HF, splits the 1 Hz and 2 Hz bands into in-phase and quadrature halves
%   against two phase references (desired-height velocity and commanded
%   force), and cross-checks with a lagged cross-covariance sweep.
%   PRE-REGISTERED: the identified band+phase term must reproduce -3.3e-3 /s
%   within +-40 %.
%   EXPIRES: verdict recorded (report / memory) for the defect-3 case.
%   Style: .claude/rules/figure-style.md. Read-only w.r.t. production files.

here = fileparts(mfilename('fullpath'));
proj = fullfile(here, '..', '..');
pdirs = {'model', 'model/dual_track', 'model/diag', 'model/thermal_force', ...
         'model/config', 'model/wall_effect', 'model/controller', ...
         'model/trajectory', 'test_script/integration'};
for i = 1:numel(pdirs); addpath(fullfile(proj, pdirs{i})); end
res_dir  = fullfile(proj, 'test_results');
if ~exist(res_dir, 'dir'); mkdir(res_dir); end
res_file = fullfile(res_dir, 'check_formB_y1rect_quadrature.mat');

% ---------------- named constants (no magic numbers) ----------------
AX        = 3;            % wall-normal axis (z)
SEED      = 7;
INJ       = 0.05;         % [R] TRUE wall offset injected on the plant side
N_CYC     = 32;           % oscillation cycles (long identification run)
T_SIM     = 34.8;         % [s] total sim time (0.5 hold + 1.0 descend + 32 + 1.3)
F0        = 1;            % [Hz] oscillation fundamental
WIN_A     = [13.5, 23.5]; % [s] primary window (10 cycles), y1 drive negative
WIN_B     = [23.5, 33.5]; % [s] contrast window (10 cycles), y1 total flips +
NU_FLOOR  = 1e-12;        % [-] innovation magnitude below which K1 is unusable
N_HARM    = 5;            % harmonics resolved individually before the HF bin
LAG_NEAR  = 40;           % [samples] pre-registered near-lag sweep
LAG_WIDE  = 800;          % [samples] half period at 1600 Hz: full phase sweep
TARGET    = -3.3e-3;      % [1/s] pre-registered rectified drive
TOL_FRAC  = 0.40;         % pre-registered reconstruction tolerance
FS_FONT   = 18;

% ---------------- run (incremental: cached signals) ----------------
if exist(res_file, 'file')
    load(res_file, 'S');
    fprintf('loaded cached signals: %s\n', res_file);
else
    ov = struct('lock_b', false, 'lock_p', true, 'lock_ws', false, ...
                'Pf_ws_std', 0.111);
    o  = struct('seeds', SEED, 'ws_inject', INJ, 'verbose', false, ...
                'ctrl_const_override', ov, ...
                'config_override', struct('n_cycles', N_CYC, 'T_sim', T_SIM));
    out = run_formB_ws(o);
    r   = out.runs{1};
    S = struct();
    S.t        = r.tout(:);
    S.nu1      = r.innov_y1_out(:, AX);
    S.dws1     = r.dws_y1_out(:, AX);
    S.dws2     = r.dws_y2_out(:, AX);
    S.f_d      = r.f_d_out(:, AX);
    S.h_bar_d  = r.h_bar_d_out(:, 1);
    S.h_bar_tr = r.h_bar_true_out(:, 1);
    S.dh_m     = r.dh_m_out(:, AX);
    S.ws_hat   = r.ws_hat_out(:, AX);
    S.sqP_ws   = r.P_ws_out(:, AX);
    S.Ts       = S.t(2) - S.t(1);
    save(res_file, 'S');
    fprintf('saved signals: %s\n', res_file);
end

t   = S.t;
Ts  = S.Ts;
nu1 = S.nu1;
K1  = zeros(size(nu1));
ok  = abs(nu1) > NU_FLOOR;
K1(ok) = S.dws1(ok) ./ nu1(ok);

% Auxiliary phase references / physical partners, full length.
wd    = S.h_bar_d;                                  % desired height [-]
wdot  = [0; diff(wd)] / Ts;                         % desired-height velocity [1/s]
fd    = S.f_d;                                      % commanded force [pN]
trk   = S.h_bar_tr - S.h_bar_d;                     % true tracking residual [-]
dhm   = S.dh_m;                                     % delayed measured error [um]

fprintf('\n=== y1-channel rectification: band / phase decomposition ===\n');
fprintf('seed %d  inject %+.3f R  n_cycles %d  T %.1f s  Ts %.6f s\n', ...
        SEED, INJ, N_CYC, t(end), Ts);

SIG = struct('t', t, 'K', K1, 'nu', nu1, 'wdot', wdot, 'fd', fd, 'trk', trk, ...
             'dhm', dhm, 'dws1', S.dws1, 'dws2', S.dws2, 'ws', S.ws_hat, ...
             'sqP', S.sqP_ws);

wins = {WIN_A, WIN_B};
A    = cell(1, 2);
for q = 1:2
    idx = find(t >= wins{q}(1) & t < wins{q}(2));
    A{q} = local_window_report(SIG, idx, Ts, F0, N_HARM, ...
                               LAG_NEAR, LAG_WIDE, wins{q});
end

% ---------------- pre-registered verdict ----------------
a = A{1};
fprintf('\n--- PRE-REGISTERED CRITERION (window [%.1f, %.1f) s) ---\n', ...
        WIN_A(1), WIN_A(2));
fprintf('measured mean(K1.*nu1)/Ts = %+.4e /s  (cov version %+.4e /s); target %+.4e /s\n', ...
        a.rate_total, a.rate_cov, TARGET);
cands = {'1 Hz band alone',        a.rate_band(1); ...
         '1 Hz + 2 Hz bands',      a.rate_band(1) + a.rate_band(2); ...
         'DC + 1 Hz + 2 Hz',       a.rate_dc + a.rate_band(1) + a.rate_band(2); ...
         '1 Hz in-phase (v ref)',  a.term_rate(3); ...
         '1 Hz quadrature (v ref)', a.term_rate(4)};
for j = 1:size(cands, 1)
    rec = cands{j, 2};
    fprintf('  %-24s %+.4e /s  = %6.1f %% of measured  |  %+7.1f %% vs target -> %s\n', ...
            cands{j, 1}, rec, 100 * rec / a.rate_total, ...
            100 * (rec - TARGET) / abs(TARGET), ...
            local_verdict(abs(rec - TARGET) <= TOL_FRAC * abs(TARGET)));
end
fprintf('phase relation at 1 Hz: phase(K1) - phase(nu1) = %+.1f deg (cos %+.3f) -> ANTI-PHASE, not quadrature\n', ...
        a.dphi_deg, cosd(a.dphi_deg));

% ---------------- figures ----------------
% The components tile the spectrum, so appending the total lets the figure
% show its own closure against the pre-registered line.
lbl = [a.term_name, {'TOTAL'}];
Mbar = 100 * [A{1}.term_rate(:), A{2}.term_rate(:); ...
              A{1}.rate_total,   A{2}.rate_total];      % [%/s]

fig = figure('Color', 'w', 'Position', [80 60 1080 600]); drawnow;
ax = axes(fig); hold(ax, 'on');
ax.Toolbar.Visible = 'off';
hb = bar(ax, Mbar, 'grouped');
hb(1).FaceColor = [0.20 0.35 0.75];
hb(2).FaceColor = [0.85 0.85 0.90];
ht = plot(ax, [0.4, numel(lbl) + 0.6], 100 * TARGET * [1 1], 'r--', 'LineWidth', 2.0);
plot(ax, [0.4, numel(lbl) + 0.6], [0 0], 'k:', 'LineWidth', 1.2, ...
     'HandleVisibility', 'off');
set(ax, 'XTick', 1:numel(lbl), 'XTickLabel', lbl, 'FontSize', FS_FONT, ...
        'FontWeight', 'bold');
xlim(ax, [0.4, numel(lbl) + 0.6]);
ylabel(ax, 'w_s drive  [% of R per s]', 'FontSize', FS_FONT, 'FontWeight', 'bold');
xlabel(ax, 'band / phase component', 'FontSize', FS_FONT, 'FontWeight', 'bold');
box(ax, 'on');
legend(ax, [hb(1) hb(2) ht], ...
       {sprintf('t = %.1f-%.1f s', WIN_A(1), WIN_A(2)), ...
        sprintf('t = %.1f-%.1f s', WIN_B(1), WIN_B(2)), ...
        'pre-registered -0.33 %/s'}, ...
       'Location', 'northoutside', 'Orientation', 'horizontal', ...
       'FontSize', FS_FONT - 4);
exportgraphics(fig, fullfile(res_dir, 'formB_y1rect_bands.png'), 'Resolution', 150);

fig = figure('Color', 'w', 'Position', [80 60 1080 600]); drawnow;
ax = axes(fig); hold(ax, 'on');
ax.Toolbar.Visible = 'off';
tw = a.t_win;
h1 = plot(ax, tw, 100 * cumsum(a.prod_full), 'b', 'LineWidth', 1.8);
h2 = plot(ax, tw, 100 * cumsum(a.prod_rec), 'Color', [0.90 0.45 0.00], ...
          'LineWidth', 1.8);
h3 = plot(ax, tw, 100 * cumsum(a.prod_q1), 'Color', [0.30 0.65 0.30], ...
          'LineWidth', 1.4);
h4 = plot(ax, tw, 100 * TARGET * (tw - tw(1)), 'r--', 'LineWidth', 2.0);
xlabel(ax, 'time  [s]', 'FontSize', FS_FONT, 'FontWeight', 'bold');
ylabel(ax, 'cumulative w_s drive  [% of R]', 'FontSize', FS_FONT, ...
       'FontWeight', 'bold');
set(ax, 'FontSize', FS_FONT, 'FontWeight', 'bold'); box(ax, 'on');
legend(ax, [h1 h2 h3 h4], {'full K_1\nu_1', 'DC + 1 Hz + 2 Hz', ...
       '1 Hz quadrature only', 'pre-registered -0.33 %/s'}, ...
       'Location', 'northoutside', 'Orientation', 'horizontal', ...
       'FontSize', FS_FONT - 5);
exportgraphics(fig, fullfile(res_dir, 'formB_y1rect_cumsum.png'), 'Resolution', 150);

fprintf('\nfigures: %s/formB_y1rect_bands.png\n', res_dir);
fprintf('         %s/formB_y1rect_cumsum.png\n', res_dir);


%% =================== local helpers ===================

function R = local_window_report(SIG, idx, Ts, f0, n_harm, ...
                                 lag_near, lag_wide, win)
%LOCAL_WINDOW_REPORT  Exact band/phase decomposition of mean(K.*nu) on one window.
    t = SIG.t;  K = SIG.K;  nu = SIG.nu;  wdot = SIG.wdot;  fd = SIG.fd;
    trk = SIG.trk;  dhm = SIG.dhm;  dws2 = SIG.dws2;
    Kw  = K(idx);   nuw = nu(idx);
    N   = numel(idx);
    fs  = 1 / Ts;
    cyc = N * Ts * f0;
    assert(abs(cyc - round(cyc)) < 1e-6, ...
           'check_formB_y1rect:nonIntegerCycles', ...
           'window holds %.4f cycles -- spectral leakage would corrupt the split.', cyc);

    prod_full = Kw .* nuw;
    mean_prod = mean(prod_full);
    cov_prod  = mean_prod - mean(Kw) * mean(nuw);

    % --- exact Parseval per-bin decomposition (bins sum to mean_prod) ---
    FK = fft(Kw);  Fn = fft(nuw);
    cbin = real(FK .* conj(Fn)) / N^2;
    fbin = (0:N-1)' * (fs / N);
    fsig = fbin;  fsig(fbin > fs/2) = fbin(fbin > fs/2) - fs;
    fa   = abs(fsig);

    % Nearest-harmonic binning: every bin lands in exactly one band, so the
    % contributions tile the spectrum and sum to mean_prod with no residual.
    n_near = round(fa / f0);
    m_dc = fa == 0;
    m_lf = ~m_dc & n_near == 0;
    m_h  = false(N, n_harm);
    for n = 1:n_harm
        m_h(:, n) = n_near == n;
    end
    m_hf = n_near > n_harm;
    assert(all(m_dc | m_lf | any(m_h, 2) | m_hf), ...
           'check_formB_y1rect:bandGap', 'band masks do not tile the spectrum.');

    rate_dc = sum(cbin(m_dc)) / Ts;
    rate_lf = sum(cbin(m_lf)) / Ts;
    rate_hf = sum(cbin(m_hf)) / Ts;
    rate_band = zeros(1, n_harm);
    for n = 1:n_harm
        rate_band(n) = sum(cbin(m_h(:, n))) / Ts;
    end

    % --- analytic band signals + phase references ---
    pos = fsig > 0;
    zK1 = local_analytic(FK, m_h(:, 1) & pos, N);
    zn1 = local_analytic(Fn, m_h(:, 1) & pos, N);
    zK2 = local_analytic(FK, m_h(:, 2) & pos, N);
    zn2 = local_analytic(Fn, m_h(:, 2) & pos, N);

    Fv  = fft(wdot(idx));   zv1 = local_analytic(Fv, m_h(:, 1) & pos, N);
    Ff  = fft(fd(idx));     zf1 = local_analytic(Ff, m_h(:, 1) & pos, N);
    Ftr = fft(trk(idx));    ztr = local_analytic(Ftr, m_h(:, 1) & pos, N);
    Fdh = fft(dhm(idx));    zdh = local_analytic(Fdh, m_h(:, 1) & pos, N);

    th_v = angle(zv1);            % reference #1: desired-height velocity
    th_f = angle(zf1);            % reference #2: commanded force
    [Iv1, Qv1] = local_iq(zK1, zn1, th_v);
    [If1, Qf1] = local_iq(zK1, zn1, th_f);
    [Iv2, Qv2] = local_iq(zK2, zn2, 2 * th_v);
    [If2, Qf2] = local_iq(zK2, zn2, 2 * th_f);
    assert(abs((Iv1 + Qv1) / Ts - rate_band(1)) < 1e-3 * max(abs(rate_band(1)), eps), ...
           'check_formB_y1rect:iqMismatch', ...
           'I+Q (%.4e) does not close on the 1 Hz band total (%.4e).', ...
           (Iv1 + Qv1) / Ts, rate_band(1));

    % --- reconstruction signals for the cumulative figure ---
    K1b = real(zK1);  n1b = real(zn1);
    K2b = real(zK2);  n2b = real(zn2);
    prod_rec = (mean(Kw) + K1b + K2b) .* (mean(nuw) + n1b + n2b);
    % quadrature-only 1 Hz product: both phasors stripped of their in-phase part
    zKq = 1i * imag(zK1 .* exp(-1i * th_v)) .* exp(1i * th_v);
    znq = 1i * imag(zn1 .* exp(-1i * th_v)) .* exp(1i * th_v);
    prod_q1 = real(zKq) .* real(znq);

    % --- lagged cross-covariance: E[K[k]*nu[k+tau]] ---
    lags_n = (-lag_near:lag_near)';
    lags_w = (-lag_wide:round(lag_wide/40):lag_wide)';
    cn = local_lagcov(K, nu, idx, lags_n) / Ts;
    cw = local_lagcov(K, nu, idx, lags_w) / Ts;
    [~, jw] = min(cw);
    slope_near = (cn(end) - cn(1)) / (lags_n(end) - lags_n(1));

    % --- console report ---
    fprintf('\n--- window [%.1f, %.1f) s : %d samples = %.0f cycles ---\n', ...
            win(1), win(2), N, cyc);
    fprintf('mean(K1)   %+.4e   mean(nu1) %+.4e   product-of-means rate %+.4e /s\n', ...
            mean(Kw), mean(nuw), mean(Kw) * mean(nuw) / Ts);
    fprintf('rms(K1)    %+.4e   rms(nu1)  %+.4e   corr(K1,nu1) %+.3f\n', ...
            sqrt(mean(Kw.^2)), sqrt(mean(nuw.^2)), local_corr(Kw, nuw));
    fprintf('TOTAL      mean(K1.*nu1)/Ts = %+.4e /s   (cov/Ts = %+.4e /s)\n', ...
            mean_prod / Ts, cov_prod / Ts);
    fprintf('y1 ws drive from log        = %+.4e /s   y2 ws drive = %+.4e /s\n', ...
            mean(prod_full) / Ts, mean(dws2(idx)) / Ts);
    fprintf('\n  band            rate [/s]     share of total\n');
    fprintf('  %-14s %+.4e   %6.1f %%\n', 'DC x DC', rate_dc, 100 * rate_dc / mean_prod * Ts);
    fprintf('  %-14s %+.4e   %6.1f %%\n', 'LF (<0.5 Hz)', rate_lf, 100 * rate_lf / mean_prod * Ts);
    for n = 1:n_harm
        fprintf('  %-14s %+.4e   %6.1f %%\n', sprintf('%d Hz', n * f0), ...
                rate_band(n), 100 * rate_band(n) / mean_prod * Ts);
    end
    fprintf('  %-14s %+.4e   %6.1f %%\n', 'HF (>5.5 Hz)', rate_hf, 100 * rate_hf / mean_prod * Ts);
    fprintf('  %-14s %+.4e   (closure check vs total)\n', 'SUM', ...
            rate_dc + rate_lf + sum(rate_band) + rate_hf);

    % --- exact carrier line vs modulation sidebands, inside each band ---
    m_line1 = abs(fa - f0)     < 1e-9;
    m_line2 = abs(fa - 2 * f0) < 1e-9;
    rate_line1 = sum(cbin(m_line1)) / Ts;
    rate_line2 = sum(cbin(m_line2)) / Ts;
    fprintf('\n  carrier line vs sidebands: 1 Hz line %+.4e /s (%.0f %% of band), sidebands %+.4e /s\n', ...
            rate_line1, 100 * rate_line1 / rate_band(1), rate_band(1) - rate_line1);
    fprintf('                             2 Hz line %+.4e /s (%.0f %% of band), sidebands %+.4e /s\n', ...
            rate_line2, 100 * rate_line2 / rate_band(2), rate_band(2) - rate_line2);

    fprintf('\n  in-phase / quadrature split (I+Q = band total)\n');
    fprintf('  ref = desired-height velocity:  1 Hz  I %+.4e  Q %+.4e   /s\n', Iv1 / Ts, Qv1 / Ts);
    fprintf('                                  2 Hz  I %+.4e  Q %+.4e   /s\n', Iv2 / Ts, Qv2 / Ts);
    fprintf('  ref = commanded force f_d:      1 Hz  I %+.4e  Q %+.4e   /s\n', If1 / Ts, Qf1 / Ts);
    fprintf('                                  2 Hz  I %+.4e  Q %+.4e   /s\n', If2 / Ts, Qf2 / Ts);

    fprintf('\n  1 Hz phasors (amplitude, phase relative to desired-height velocity)\n');
    ph = @(z) [mean(abs(z)), rad2deg(angle(mean(z .* exp(-1i * th_v))))];
    for pr = {{'K_ws_y1', zK1}, {'nu1', zn1}, {'f_d [pN]', zf1}, ...
              {'trk resid', ztr}, {'delayed err [um]', zdh}, {'wdot [1/s]', zv1}}
        v = ph(pr{1}{2});
        fprintf('  %-18s  |A| %.4e   phase %+7.1f deg\n', pr{1}{1}, v(1), v(2));
    end
    dphi = rad2deg(angle(mean(zK1 .* exp(-1i * th_v))) - angle(mean(zn1 .* exp(-1i * th_v))));
    dphi = mod(dphi + 180, 360) - 180;
    fprintf('  phase(K1) - phase(nu1) = %+.1f deg  -> cos = %+.3f  (band total sign)\n', ...
            dphi, cosd(dphi));
    fprintf('  implied timing: nu1 lags K1 by %+.1f ms (%+.0f samples) at %g Hz\n', ...
            1000 * dphi / 360 / f0, dphi / 360 / f0 / Ts, f0);

    % --- premise check: naive broadband projections on the two references,
    %     which is what a correlation-based reading of "in-phase" would use ---
    fdw = fd(idx);  vdw = wdot(idx);
    fprintf('\n  broadband projections (what a raw correlation reads)\n');
    fprintf('  corr(K1, f_d)  %+.3f   corr(nu1, f_d)  %+.3f   product of projections %+.4e /s\n', ...
            local_corr(Kw, fdw), local_corr(nuw, fdw), ...
            local_proj(Kw, fdw) * local_proj(nuw, fdw) * mean(fdw.^2) / Ts);
    fprintf('  corr(K1, wdot) %+.3f   corr(nu1, wdot) %+.3f   product of projections %+.4e /s\n', ...
            local_corr(Kw, vdw), local_corr(nuw, vdw), ...
            local_proj(Kw, vdw) * local_proj(nuw, vdw) * mean(vdw.^2) / Ts);
    % Where does corr(nu1, f_d) live? Same Parseval split applied to <nu1*f_d>.
    cnf = real(Fn .* conj(fft(fdw))) / N^2;
    fprintf('  <nu1*f_d> by band [pN]: DC %+.3e  1Hz %+.3e  2Hz %+.3e  3-5Hz %+.3e  HF(>5.5Hz) %+.3e\n', ...
            sum(cnf(m_dc)), sum(cnf(m_h(:, 1))), sum(cnf(m_h(:, 2))), ...
            sum(cnf(any(m_h(:, 3:end), 2))), sum(cnf(m_hf)));
    pf = real(fft(fdw) .* conj(fft(fdw))) / N^2;
    fprintf('  f_d power by band [pN^2]: DC %.3e  1Hz %.3e  2Hz %.3e  HF(>5.5Hz) %.3e (rms %.3f pN)\n', ...
            sum(pf(m_dc)), sum(pf(m_h(:, 1))), sum(pf(m_h(:, 2))), sum(pf(m_hf)), ...
            sqrt(mean(fdw.^2)));

    % --- w_s bookkeeping closure: the logged slot-7 motion must equal the sum
    %     of the two channel increments over the window ---
    dws_sum = sum(SIG.dws1(idx)) + sum(dws2(idx));
    dws_obs = SIG.ws(idx(end)) - SIG.ws(idx(1)) + SIG.dws1(idx(1)) + dws2(idx(1));
    fprintf('\n  w_s bookkeeping: mean ws_hat %.4f  mean sqrt(P_ws) %.4f  ws_hat at window end %.4f\n', ...
            mean(SIG.ws(idx)), mean(SIG.sqP(idx)), SIG.ws(idx(end)));
    fprintf('  delta ws_hat over window %+.4f R   channel sum (y1+y2) %+.4f R   y1 alone %+.4f R\n', ...
            dws_obs, dws_sum, sum(SIG.dws1(idx)));

    % --- rectification test: is the 1 Hz product one-signed, and does it
    %     follow the SQUARE of the commanded velocity? ---
    Fvd  = fft(vdw);
    v1b  = real(local_analytic(Fvd, m_h(:, 1) & pos, N));
    p1b  = K1b .* n1b;
    kappa = mean(p1b) / mean(v1b.^2);
    fprintf('\n  rectification test on the 1 Hz band product K_1Hz * nu_1Hz\n');
    fprintf('  fraction of window with product < 0: %.3f   corr(product, wdot_1Hz^2) %+.3f\n', ...
            mean(p1b < 0), local_corr(p1b, v1b.^2));
    fprintf('  implied law  K*nu = -kappa * wdot^2  with kappa = %+.4e s^2  -> rate %+.4e /s\n', ...
            -kappa, mean(p1b) / Ts);

    fprintf('\n  lagged cross-cov E[K1[k]*nu1[k+tau]]/Ts:\n');
    for tau = [-lag_near, -20, -5, 0, 5, 20, lag_near]
        fprintf('   tau %+5d (%+6.2f ms): %+.4e /s\n', tau, 1000 * tau * Ts, ...
                cn(lags_n == tau));
    end
    fprintf('   near-lag slope %+.4e /s per sample; wide sweep minimum at tau %+d (%+.1f ms)\n', ...
            slope_near, lags_w(jw), 1000 * lags_w(jw) * Ts);

    R = struct();
    R.t_win     = t(idx);
    R.rate_total = mean_prod / Ts;
    R.rate_cov   = cov_prod / Ts;
    R.rate_dc    = rate_dc;
    R.rate_lf    = rate_lf;
    R.rate_band  = rate_band;
    R.rate_hf    = rate_hf;
    R.term_rate  = [rate_dc, rate_lf, Iv1 / Ts, Qv1 / Ts, Iv2 / Ts, Qv2 / Ts, ...
                    sum(rate_band(3:end)), rate_hf];
    R.term_name  = {'DC', 'LF', '1Hz I', '1Hz Q', '2Hz I', '2Hz Q', '3-5Hz', 'HF'};
    R.iq_force   = [If1, Qf1, If2, Qf2] / Ts;
    R.dphi_deg   = dphi;
    R.prod_full  = prod_full;
    R.prod_rec   = prod_rec;
    R.prod_q1    = prod_q1;
    R.lags_near  = lags_n;
    R.cov_near   = cn;
    R.lags_wide  = lags_w;
    R.cov_wide   = cw;
end


function z = local_analytic(X, mask_pos, N)
%LOCAL_ANALYTIC  Band-limited analytic signal: real(z) is the band-passed signal.
    Z = zeros(N, 1);
    Z(mask_pos) = 2 * X(mask_pos);
    z = ifft(Z);
end


function [I, Q] = local_iq(zx, zy, theta)
%LOCAL_IQ  In-phase / quadrature halves of mean(real(zx).*real(zy)).
%   Rotating both phasors by the reference phase leaves the product invariant,
%   so I + Q is the full band contribution and the I x Q cross terms vanish.
    ex = zx .* exp(-1i * theta);
    ey = zy .* exp(-1i * theta);
    I = 0.5 * mean(real(ex) .* real(ey));
    Q = 0.5 * mean(imag(ex) .* imag(ey));
end


function c = local_lagcov(K, nu, idx, lags)
%LOCAL_LAGCOV  E[K[k]*nu[k+tau]] over the window, nu drawn from the full record.
    n = numel(K);
    c = zeros(numel(lags), 1);
    for j = 1:numel(lags)
        jj = idx + lags(j);
        keep = jj >= 1 & jj <= n;
        c(j) = mean(K(idx(keep)) .* nu(jj(keep)));
    end
end


function a = local_proj(x, y)
%LOCAL_PROJ  Least-squares coefficient of x on y (the "in-phase" amplitude).
    a = (x' * y) / max(y' * y, eps);
end


function r = local_corr(x, y)
%LOCAL_CORR  Pearson correlation without the Statistics Toolbox.
    xc = x - mean(x);  yc = y - mean(y);
    r = (xc' * yc) / max(sqrt((xc' * xc) * (yc' * yc)), eps);
end


function s = local_verdict(ok)
%LOCAL_VERDICT  PASS/FAIL label.
    if ok; s = 'PASS'; else; s = 'FAIL'; end
end
