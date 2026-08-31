% STATUS: ACTIVE (scratch, derivation probe series 2026-08-31)
% T6: G4 = T_A (coherent) + T_B (slope evaluation-point) confirmed; T_B is the anti-coherent term.
% Part of the closed-loop mean-bias derivation chain; see memory
% project_formC_inject_response_y1_amplifies_2026-08-26 (08-31 entries).
WT = '/Users/kevin/Code/MotionControl_Simu-law-error-budget';
addpath(genpath(fullfile(WT, 'model')));
od = fullfile(WT, 'test_results', 'loop_mean_bias');
A4 = load(fullfile(od, 't4_adjoint.mat'));
S = load(fullfile(od, 'replay_v2_profile.mat'));
ax = 3;  lam = 0.7;
TA = [];  TB = [];  G4 = S.out.G4;
for sd = 1:8
    C = load(fullfile(od, sprintf('loop_capture_seed%d.mat', sd)));
    R = load(fullfile(od, sprintf('run_log_seed%d.mat', sd)));  r = R.run_log;
    kk = C.KK;  n = numel(kk);  a_nom = r.a_nom;
    ht = r.h_bar_true_out(:);  hd = r.h_bar_d_out(:);
    dtrs = [0; diff(ht)];  dtrs = dtrs(kk);
    dwd  = [0; diff(hd)];  dwd  = dwd(kk);
    ah = r.a_bar_hat_out(kk, ax);  bh = r.b_hat_out(kk, ax);
    x3 = C.XU(3, :).';  m89 = (C.XU(8, :) + C.XU(9, :)).';
    Mhat = dwd + [0; (1-lam)*x3(1:end-1)] + [0; (1-lam)*m89(1:end-1)];
    mis = dtrs - Mhat;
    apH = [0; bh(1:end-1) .* (1 - ah(1:end-1)).^2];              % a'(a_hat[k-1])
    % true slope at the true position, previous step, normalized:
    apT_raw = r.a_prime_true_out(kk, ax);
    % calibrate units: normalized a' should be ~ b*(1-a_true)^2; fit scale once
    at = r.a_true_out(kk, ax) / a_nom;
    apT_ref = bh .* (1 - at).^2;
    if sd == 1
        sc = apT_raw \ (apT_ref * a_nom);  %#ok<NASGU> % probe scale (printed below)
        sc1 = (apT_raw .* apT_ref > 0);
        fprintf('unit probe: median apT_raw/apT_ref = %.4f (if ~a_nom=%.4f, raw is dimensional)\n', ...
                median(apT_raw(sc1) ./ apT_ref(sc1)), a_nom);
    end
    apT = apT_raw / a_nom * 1;                                    % assume raw = dimensional slope per w_bar
    apTm = [0; apT(1:end-1)];
    TA(:, sd) = apH .* mis; %#ok<SAGROW>
    TB(:, sd) = (apTm - apH) .* dtrs; %#ok<SAGROW>
end
w4 = A4.w4;  t = A4.t(:);
TAm = mean(TA, 2);  TBm = mean(TB, 2);  G4m = mean(G4, 2);
fprintf('adjoint dots (row4):  T_A %+8.4f   T_B %+8.4f   T_A+T_B %+8.4f   measured G4 %+8.4f\n', ...
    nansum(w4 .* TAm), nansum(w4 .* TBm), nansum(w4 .* (TAm + TBm)), nansum(w4 .* G4m));
% per-step identity: how much of G4 do T_A + T_B explain?
m = t > 0.5;
X = [TAm(m), TBm(m)];
bfit = X \ G4m(m);
res = G4m(m) - X * bfit;
fprintf('mean-profile fit G4 ~ [T_A, T_B]: coeffs %+0.3f %+0.3f   R2 %.3f\n', bfit, 1 - var(res)/var(G4m(m)));
% per-seed per-step (fluctuation) fit
q = 7;
Xs = [TA(m, q), TB(m, q)];
bs = Xs \ G4(m, q);
fprintf('seed-7 per-step fit: coeffs %+0.3f %+0.3f   R2 %.3f\n', bs, 1 - var(G4(m,q) - Xs*bs)/var(G4(m,q)));
SEG = {'desc', t > 0.55 & t < 1.45; 'osc', t >= 1.5 & t < 3.5; 'hold2', t > 3.7};
fprintf('segment means (x1e-5):  T_A      T_B      sum      G4\n');
for g = 1:3
    mm = SEG{g,2};
    fprintf('  %-6s %+8.2f %+8.2f %+8.2f %+8.2f\n', SEG{g,1}, 1e5*mean(TAm(mm)), 1e5*mean(TBm(mm)), ...
            1e5*mean(TAm(mm)+TBm(mm)), 1e5*mean(G4m(mm)));
end
fprintf('T6 DONE\n');
