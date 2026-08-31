% STATUS: ACTIVE (scratch, derivation probe series 2026-08-31)
% T3: closed-form forcing replays to ~0 (segment means match but bias lives in time structure).
% Part of the closed-loop mean-bias derivation chain; see memory
% project_formC_inject_response_y1_amplifies_2026-08-26 (08-31 entries).
% T3: THE milestone test. Closed-form forcing, no data-fitted numbers:
%   C_ws(t) = b*(2q - V),  q = kappa_T*a_bar_true(t),  b = 1-lc
%   V(t)    = [q(1+2b^2) + 4 lc b q + b^2 r] / (1-lc^2)
%   g3(t)   = (a'/a_hat)(t) * C_ws(t)                     (update-level row 3)
%   E_mis   = -g3 ;  g4(t) = a'(t) * (0.7*E_mis(t) + 0.3*E_mis(t-2))
% Replay [g3; g4] through the canonical seed-7 loop -> predicted mean bias,
% against: actual seed-mean e4 -0.0121 +- 0.0022, and the measured-forcing
% replay -0.0115. Also compare g3(t) with the measured smoothed mean-G3 curve.
WT = '/Users/kevin/Code/MotionControl_Simu-law-error-budget';
addpath(genpath(fullfile(WT, 'model')));
od = fullfile(WT, 'test_results', 'loop_mean_bias');
C = load(fullfile(od, 'loop_capture_seed7.mat'));
R = load(fullfile(od, 'run_log_seed7.mat'));  r = R.run_log;
S = load(fullfile(od, 'replay_v2_profile.mat'));
ax = 3;  lc = 0.7;  b = 1 - lc;  kk = C.KK;  n = numel(kk);  t = C.t(:);
pc = physical_constants();
a_o = (r.a_nom / pc.R);
kappa = 4 * pc.k_B * pc.T / pc.R * a_o;
rn = (3.31e-3 / pc.R)^2;                      % z meas noise variance [R^2]
abar = r.a_true_out(kk, ax) / r.a_nom;        % use true a_bar profile (run-time version would use a_hat; diff is 2nd order)
ah = r.a_bar_hat_out(kk, ax);  bh = r.b_hat_out(kk, ax);
ap = bh .* (1 - ah).^2;
q = kappa * abar;
V = (q * (1 + 2*b^2) + 4*lc*b*q + b^2 * rn) / (1 - lc^2);
Cws = b * (2*q - V);
g3 = (ap ./ max(ah, 0.02)) .* Cws;
Emis = -g3;
g4 = ap .* (0.7 * Emis + 0.3 * [0; 0; Emis(1:end-2)]);
% ---- compare g3(t) with the measured mean-G3 profile --------------------
G3m = movmean(mean(S.out.G3, 2), 401);
SEG = {'hold1', t > 0.05 & t < 0.5; 'desc', t > 0.55 & t < 1.45; 'osc', t >= 1.5 & t < 3.5; 'hold2', t > 3.7};
fprintf('g3 closed form vs measured mean G3 (x1e-5):\n');
for gI = 1:4
    m = SEG{gI, 2};
    fprintf('  %-6s pred %+6.2f   meas %+6.2f +- %4.2f\n', SEG{gI,1}, 1e5*mean(g3(m)), ...
        1e5*mean(mean(S.out.G3(m,:),1)), 1e5*std(mean(S.out.G3(m,:),1))/sqrt(8));
end
fprintf('  curve corr(g3_pred, G3_meas smoothed), t>0.5: %+.3f\n', corr(g3(t>0.5), G3m(t>0.5)));
fprintf('g4 closed form vs measured mean G4 (x1e-5):\n');
for gI = 1:4
    m = SEG{gI, 2};
    fprintf('  %-6s pred %+6.2f   meas %+6.2f +- %4.2f\n', SEG{gI,1}, 1e5*mean(g4(m)), ...
        1e5*mean(mean(S.out.G4(m,:),1)), 1e5*std(mean(S.out.G4(m,:),1))/sqrt(8));
end
% ---- replay ----------------------------------------------------------------
nsl = size(C.F, 1);  e = zeros(nsl, 1);  E4 = zeros(n, 1);
for i = 3:n
    g = zeros(nsl, 1);  g(3) = g3(i);  g(4) = g4(i);
    e = C.A(:,:,i) * e + g;  E4(i) = e(4);
end
mh = t > 3.7;
fprintf('\n[T3 milestone] end-hold e4: CLOSED FORM %+.4f | measured-forcing replay -0.0115 | actual -0.0121 +- 0.0022\n', mean(E4(mh)));
fprintf('T3 DONE\n');
