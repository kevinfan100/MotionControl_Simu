% STATUS: ACTIVE (scratch, derivation probe series 2026-08-31)
% T1c: 0.72 SOLVED = lag split [0.70, 0.00, 0.30] over lags [0,1,2], sum 1.008 (d=2 delay).
% Part of the closed-loop mean-bias derivation chain; see memory
% project_formC_inject_response_y1_amplifies_2026-08-26 (08-31 entries).
WT = '/Users/kevin/Code/MotionControl_Simu-law-error-budget';
od = fullfile(WT, 'test_results', 'loop_mean_bias');
C = load(fullfile(od, 'loop_capture_seed7.mat'));
R = load(fullfile(od, 'run_log_seed7.mat'));  r = R.run_log;
S = load(fullfile(od, 'replay_v2_profile.mat'));
ax = 3;  lam = 0.7;  kk = C.KK;  n = numel(kk);  t = C.t(:);
ht = r.h_bar_true_out(:);  hd = r.h_bar_d_out(:);
dtrs = [0; diff(ht)];  dtrs = dtrs(kk);
dwd  = [0; diff(hd)];  dwd  = dwd(kk);
x3 = C.XU(3, :).';  m89 = (C.XU(8, :) + C.XU(9, :)).';
Mhat = dwd + [0; (1-lam)*x3(1:end-1)] + [0; (1-lam)*m89(1:end-1)];
mis = dtrs - Mhat;
g3 = S.out.G3(:, 7);  g4 = S.out.G4(:, 7);
ah = r.a_bar_hat_out(kk, ax);  bh = r.b_hat_out(kk, ax);
apA = [0; (bh(1:end-1) .* (1 - ah(1:end-1)).^2)];
m = find(t > 0.5);
% two-lag regression: does the missing 0.28 sit in the adjacent step?
X3 = [-mis(m), -[0; mis(m(1:end-1))], -[0; 0; mis(m(1:end-2))]];
b3 = X3 \ g3(m);
res3 = g3(m) - X3 * b3;
fprintf('G3 on -mis lags [0 1 2]: coeffs %+0.3f %+0.3f %+0.3f  sum %+0.3f   R2 %.3f\n', b3, sum(b3), 1 - var(res3)/var(g3(m)));
v4 = apA .* mis;
X4 = [v4(m), [0; v4(m(1:end-1))], [0; 0; v4(m(1:end-2))]];
b4 = X4 \ g4(m);
res4 = g4(m) - X4 * b4;
fprintf('G4 on a''*mis lags [0 1 2]: coeffs %+0.3f %+0.3f %+0.3f  sum %+0.3f   R2 %.3f\n', b4, sum(b4), 1 - var(res4)/var(g4(m)));
fprintf('T1C DONE\n');
