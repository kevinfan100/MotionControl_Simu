% STATUS: ACTIVE (scratch) | PURPOSE: audit the f_det split's DROPPED term.
%   4state_del_hd.tex "F_e endogeneity" chapter splits F_dx = F_det + F_ram,
%   puts F_det in the Jacobian and declares "F_ram[k] e_a[k] -> noise",
%   bounding only its VARIANCE (Var(F_ram e_a)/Var(eps) < 1e-3). It never
%   bounds its MEAN -- yet a one-signed mean is exactly what the same chapter
%   calls fatal (rectification) when it argues against the realised force.
%   F_ram is propagated from its OWN recursion (tex eq. "Remainder"), driven
%   by the logged delta_w_m and the applied gain, so no trajectory term and no
%   offset traps enter; the final hold doubles as the alignment check, since
%   there traj_term == 0 exactly and the mirror pole |z| = sqrt(1-lc) = 0.548
%   has driven f_det to machine zero, so F_ram must equal the realised F_dw.
%   EXPIRES: when the endogeneity chapter either bounds E[F_ram e_a] or gives
%   the term a container (Q row / bias state).
% Production files untouched; read-only post-processing of run_formB_ws logs.

AX_Z   = 3;
LC     = 0.7;      % closed-loop pole (user_config)
NBLK   = 20;       % blocks for an autocorrelation-robust SEM
WIN    = {'descent', [0.5 1.5]; 'osc', [1.6 3.5]; 'hold', [3.8 4.8]};

here = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(here));
addpath(genpath(fullfile(root, 'model')));
addpath(fullfile(root, 'test_script', 'integration'));

out  = run_formB_ws();
nrun = numel(out.runs);
nwin = size(WIN, 1);

MU = zeros(nrun, nwin);  TT = zeros(nrun, nwin);  SG = zeros(nrun, nwin);
VR = zeros(nrun, nwin);  DET = zeros(nrun, nwin);
U = cell(nrun, 1);  TVEC = cell(nrun, 1);  CHK = zeros(nrun, 1);
FDET_ALL = [];  FRAW_ALL = [];   % cross-seed exogeneity test (same trajectory)

for q = 1:nrun
    s  = out.runs{q};
    t  = s.tout(:);
    N  = numel(t);

    % ---- the two factors, in normalized (w_bar per step) units -------
    fbar   = s.f_bar_out(:, AX_Z);                     % realised f_bar_d [-]
    dwm    = s.dh_m_out(:, AX_Z) / s.R;                % delta_w_bar_m [-]
    abar_h = s.a_bar_hat_out(:, AX_Z);                 % POSTERIOR at k
    a_ctrl = [abar_h(1); abar_h(1:end-1)];             % control law uses [k-1]
    a_disp = s.a_hat_out(:, AX_Z) ./ abar_h;
    e_abar = s.a_true_out(:, AX_Z) ./ a_disp - abar_h; % e = truth - estimate (S1)

    % ---- F_ram by its own recursion (tex "Remainder") ----------------
    fram = zeros(N, 1);
    for k = 3:N
        fram(k) = (1 - LC) / a_ctrl(k) * (dwm(k) ...
                  - a_ctrl(k-1) * fram(k-1) - a_ctrl(k-2) * fram(k-2));
    end
    F_ram = fram;
    F_ram(3:end) = fram(3:end) + (1 - LC) * (fram(2:end-1) + fram(1:end-2));
    F_raw = fbar;
    F_raw(3:end) = fbar(3:end) + (1 - LC) * (fbar(2:end-1) + fbar(1:end-2));
    F_det = F_raw - F_ram;

    % ---- alignment check: F_det must vanish in the final hold --------
    kh = t >= 4.2;
    CHK(q) = max(abs(F_det(kh))) / max(abs(F_raw(kh)));

    u = F_ram .* e_abar;
    U{q} = u;  TVEC{q} = t;
    FDET_ALL(:, q) = F_det;  FRAW_ALL(:, q) = F_raw;   %#ok<SAGROW>

    for w = 1:nwin
        kk  = t >= WIN{w, 2}(1) & t <= WIN{w, 2}(2);
        uu  = u(kk);
        q33 = mean(s.Q33_out(kk, AX_Z));
        len = floor(numel(uu) / NBLK) * NBLK;
        bm  = mean(reshape(uu(1:len), len / NBLK, NBLK), 1);
        MU(q, w)  = mean(uu);
        TT(q, w)  = mean(uu) / (std(bm) / sqrt(NBLK));
        SG(q, w)  = sqrt(q33);
        VR(q, w)  = var(uu) / q33;
        DET(q, w) = mean(abs(F_det(kk) .* e_abar(kk)));
    end
end

fprintf('\n============ f_det split audit: the DROPPED term F_ram*e_abar ============\n');
fprintf('units = normalized w_bar per step, i.e. the same units as eps_w\n');
fprintf('alignment check max|F_det|/max|F_raw| in the final hold (must be ~0): %.2e\n\n', max(CHK));
for w = 1:nwin
    fprintf('--- window %s [%.1f %.1f] s ---\n', WIN{w, 1}, WIN{w, 2}(1), WIN{w, 2}(2));
    fprintf('%6s | %11s %8s | %11s | %9s | %11s\n', ...
            'seed', 'mean(u)', 'mean/SE', 'sqrt(Q33)', 'var ratio', 'E|F_det e|');
    for q = 1:nrun
        fprintf('%6d | %+11.3e %+8.1f | %11.3e | %9.2e | %11.3e\n', ...
                out.seeds(q), MU(q, w), TT(q, w), SG(q, w), VR(q, w), DET(q, w));
    end
    fprintf('%6s | %+11.3e  %d/%d>0 | ratio to sqrt(Q33) %.2e | kept/dropped %.1f\n\n', ...
            'MEAN', mean(MU(:, w)), sum(MU(:, w) > 0), nrun, ...
            abs(mean(MU(:, w))) / mean(SG(:, w)), ...
            mean(DET(:, w)) / mean(abs(MU(:, w))));
end

% ---- exogeneity test: is F_det really trajectory-only? -------------------
% Same commanded trajectory for every seed, so a trajectory-only quantity
% would be seed-INVARIANT. Reported over the moving part of the run.
tv  = TVEC{1};
kmv = tv >= 0.5 & tv <= 3.5;
sd_det = std(FDET_ALL(kmv, :), 0, 2);
mu_det = mean(FDET_ALL(kmv, :), 2);
sd_raw = std(FRAW_ALL(kmv, :), 0, 2);
mu_raw = mean(FRAW_ALL(kmv, :), 2);
fprintf('--- exogeneity of F_det (cross-seed, identical trajectory, t in [0.5 3.5]) ---\n');
fprintf('F_det : rms cross-seed spread / rms level = %.4f  (0 = exogenous)\n', ...
        rms(sd_det) / rms(mu_det));
fprintf('F_raw : rms cross-seed spread / rms level = %.4f  (reference)\n', ...
        rms(sd_raw) / rms(mu_raw));

% ---------------- figure ----------------
fig = figure('Position', [100 100 1500 430]);
subplot(1, 3, 1); hold on; box on;
for q = 1:nrun
    plot(TVEC{q}, cumsum(U{q}) ./ (1:numel(U{q})).', 'LineWidth', 1.1);
end
yline(0, 'k--');
xlabel('t [s]'); ylabel('running mean of dropped term  [R/step]');
legend(arrayfun(@(s) sprintf('seed %d', s), out.seeds, 'UniformOutput', false), ...
       'Location', 'northoutside', 'Orientation', 'horizontal');

subplot(1, 3, 2); hold on; box on;
bar(MU);
yline(0, 'k--');
set(gca, 'XTick', 1:nrun, 'XTickLabel', arrayfun(@(s) sprintf('%d', s), out.seeds, ...
    'UniformOutput', false));
xlabel('seed'); ylabel('window mean of dropped term  [R/step]');
legend(WIN(:, 1), 'Location', 'northoutside', 'Orientation', 'horizontal');

subplot(1, 3, 3); hold on; box on;
kz = tv >= 2.0 & tv <= 2.3;
plot(tv(kz), FDET_ALL(kz, :), 'LineWidth', 1.0);
xlabel('t [s]'); ylabel('F_{det}  [-]');
legend({'6 seeds, identical command'}, 'Location', 'northoutside', ...
       'Orientation', 'horizontal');

png = fullfile(here, 'temp_fdet_remainder_hold.png');
exportgraphics(fig, png, 'Resolution', 150);
fprintf('figure: %s\n', png);
