% STATUS: ACTIVE (scratch, derivation probe series 2026-08-31)
% T1b: is 0.72 the update-level (I-KH) mixing? NO (slope unchanged).
% Part of the closed-loop mean-bias derivation chain; see memory
% project_formC_inject_response_y1_amplifies_2026-08-26 (08-31 entries).
WT = '/Users/kevin/Code/MotionControl_Simu-law-error-budget';
od = fullfile(WT, 'test_results', 'loop_mean_bias');
C = load(fullfile(od, 'loop_capture_seed7.mat'));
R = load(fullfile(od, 'run_log_seed7.mat'));  r = R.run_log;
S = load(fullfile(od, 'replay_v2_profile.mat'));
ax = 3;  lam = 0.7;  a_nom = r.a_nom;  kk = C.KK;  n = numel(kk);  t = C.t(:);
ht = r.h_bar_true_out(:);  hd = r.h_bar_d_out(:);
dtrs = [0; diff(ht)];  dtrs = dtrs(kk);
dwd  = [0; diff(hd)];  dwd  = dwd(kk);
ah = r.a_bar_hat_out(kk, ax);  bh = r.b_hat_out(kk, ax);
x3 = C.XU(3, :).';  m89 = (C.XU(8, :) + C.XU(9, :)).';
Mhat = dwd + [0; (1-lam)*x3(1:end-1)] + [0; (1-lam)*m89(1:end-1)];
mis = dtrs - Mhat;                                   % displacement mismatch (process level)
apA = [0; (bh(1:end-1) .* (1 - ah(1:end-1)).^2)];
g4 = S.out.G4(:, 7);  g3 = S.out.G3(:, 7);
ns = size(C.F, 1);  H1 = zeros(1, ns);  H1(1) = 1;
% predicted UPD-level rows 3,4 from process forcing [row3 = -mis? sign; row4 = a'*mis]
% try both row-3 signs; row 3 of e is w_d - w: extra true displacement +mis lowers it -> -mis
v4 = zeros(n, 2);  v3 = zeros(n, 2);
for i = 2:n
    U = (eye(ns) - C.K2(:,i) * C.H2S(:,i).') * (eye(ns) - C.K1(:,i) * H1);
    for sgn = 1:2
        s3 = (-1)^sgn;                                % sgn=1 -> -mis on row3; sgn=2 -> +mis
        gp = zeros(ns, 1);  gp(3) = s3 * mis(i);  gp(4) = apA(i) * mis(i);
        gu = U * gp;
        v3(i, sgn) = gu(3);  v4(i, sgn) = gu(4);
    end
end
m = t > 0.5;
for sgn = 1:2
    fprintf('[row3 sign %+d] G4 ~ upd-level pred: slope %.3f corr %.3f | G3 ~ pred row3: slope %.3f corr %.3f\n', ...
        (-1)^sgn, v4(m,sgn) \ g4(m), corr(v4(m,sgn), g4(m)), v3(m,sgn) \ g3(m), corr(v3(m,sgn), g3(m)));
end
% also: row-4 pred WITHOUT row-3 coupling (only row4 through U)
v4o = zeros(n, 1);
for i = 2:n
    U = (eye(ns) - C.K2(:,i) * C.H2S(:,i).') * (eye(ns) - C.K1(:,i) * H1);
    gp = zeros(ns, 1);  gp(4) = apA(i) * mis(i);
    gu = U * gp;  v4o(i) = gu(4);
end
fprintf('[row4-only mix]  slope %.3f corr %.3f\n', v4o(m) \ g4(m), corr(v4o(m), g4(m)));
fprintf('T1B DONE\n');
