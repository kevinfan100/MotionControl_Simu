% STATUS: ACTIVE (scratch, derivation probe series 2026-08-31)
% T5: coherent row3 + slope*row4 injection self-cancels (-0.0236 vs +0.0237); bias exists because rows are NOT coherent.
% Part of the closed-loop mean-bias derivation chain; see memory
% project_formC_inject_response_y1_amplifies_2026-08-26 (08-31 entries).
WT = '/Users/kevin/Code/MotionControl_Simu-law-error-budget';
addpath(genpath(fullfile(WT, 'model')));
od = fullfile(WT, 'test_results', 'loop_mean_bias');
A4 = load(fullfile(od, 't4_adjoint.mat'));
C = load(fullfile(od, 'loop_capture_seed7.mat'));
R = load(fullfile(od, 'run_log_seed7.mat'));  r = R.run_log;
ax = 3;  lc = 0.7;  b = 1 - lc;  kk = C.KK;  n = numel(kk);  t = C.t(:);
pc = physical_constants();
a_o = r.a_nom / pc.R;  kappa = 4 * pc.k_B * pc.T / pc.R * a_o;  rn = (3.31e-3 / pc.R)^2;
abar = r.a_true_out(kk, ax) / r.a_nom;
ah = r.a_bar_hat_out(kk, ax);  bh = r.b_hat_out(kk, ax);  ap = bh .* (1 - ah).^2;
q = kappa * abar;
V = (q * (1 + 2*b^2) + 4*lc*b*q + b^2 * rn) / (1 - lc^2);
Cws = b * (2*q - V);
g3 = (ap ./ max(ah, 0.02)) .* Cws;
Emis = -g3;
g4 = ap .* (0.7 * Emis + 0.3 * [0; 0; Emis(1:end-2)]);
w3 = A4.w3;  w4 = A4.w4;
fprintf('adjoint dot closed-form: row3 %+8.4f  row4 %+8.4f  total %+8.4f   (T3 forward said +0.0001)\n', ...
    nansum(w3 .* g3), nansum(w4 .* g4), nansum(w3 .* g3) + nansum(w4 .* g4));
SEG = {'desc', t > 0.55 & t < 1.45; 'osc', t >= 1.5 & t < 3.5; 'gap2', t >= 3.5 & t <= 3.7; 'hold2', t > 3.7};
for gI = 1:4
    m = SEG{gI,2};
    fprintf('  %-6s row3 %+8.4f  row4 %+8.4f\n', SEG{gI,1}, nansum(w3(m).*g3(m)), nansum(w4(m).*g4(m)));
end
% forward replay cross-check (same code as T3)
nsl = size(C.F, 1);  e = zeros(nsl, 1);  E4 = zeros(n, 1);
for i = 3:n
    g = zeros(nsl, 1);  g(3) = g3(i);  g(4) = g4(i);
    e = C.A(:,:,i) * e + g;  E4(i) = e(4);
end
mh = t > 3.7;
fprintf('forward replay (recheck): end-hold e4 %+8.4f\n', mean(E4(mh)));
% row3-only and row4-only forward
for mode = 1:2
    e = zeros(nsl, 1);  E4m = zeros(n, 1);
    for i = 3:n
        g = zeros(nsl, 1);
        if mode == 1; g(3) = g3(i); else; g(4) = g4(i); end
        e = C.A(:,:,i) * e + g;  E4m(i) = e(4);
    end
    fprintf('forward row%d-only: %+8.4f\n', 2 + mode, mean(E4m(mh)));
end
fprintf('T5 DONE\n');
