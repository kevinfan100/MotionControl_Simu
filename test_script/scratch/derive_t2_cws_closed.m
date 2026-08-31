% STATUS: ACTIVE (scratch, derivation probe series 2026-08-31)
% T2: C_ws closed form b(2q-V) from moment equations; hold ratio 1.22, Var 0.91, g3(hold) 1.13.
% Part of the closed-loop mean-bias derivation chain; see memory
% project_formC_inject_response_y1_amplifies_2026-08-26 (08-31 entries).
% T2 numeric check: hand-derived stationary covariance of the position loop
% (delayed reconstruction, pole lc) against the measured centred covariance
% <dw_c * step_c> in the canonical END HOLD (motion off => no command terms).
%
% Scalar fluctuation model derived on paper (2026-08-31):
%   y[k+1] = lc*y[k] + u[k] + (1-lc)*(u[k-1] + u[k-2] + n[k])
%   step_c = (1-lc)*(-y[k] + u[k-1] + u[k-2] + n[k])
%   C_ws = E[y*step_c] = (1-lc)*(2q - V) - (1-lc)^2? ... coded from the moment
%   equations below (no shortcuts); q = Var(u) = kappa_T*a_bar, r = Var(n).
WT = '/Users/kevin/Code/MotionControl_Simu-law-error-budget';
addpath(genpath(fullfile(WT, 'model')));
od = fullfile(WT, 'test_results', 'loop_mean_bias');
lc = 0.7;  b = 1 - lc;
% ---- moment equations (solve numerically, no hand algebra trusted) ----------
% state s = [y; u1; u2] with u1 = u[k-1], u2 = u[k-2]:
%   y+  = lc*y + b*u1 + b*u2 + 1*u_new + b*n_new   (u_new, n_new fresh)
%   u1+ = u_new ;  u2+ = u1
momfun = @(q, r) local_moments(lc, q, r);
% ---- measured side: canonical 8 seeds, end hold ----------------------------
pcR = physical_constants();  Rr = pcR.R;
kB = pcR.k_B;  T = pcR.T;
DWc_ST = [];  QQ = [];  AB = [];  APH = [];
for sd = 1:8
    L = load(fullfile(od, sprintf('run_log_seed%d.mat', sd)));  r0 = L.run_log;
    t = r0.tout(:);  ax = 3;  a_nom = r0.a_nom;
    mh = t > 3.7;
    dw = -(r0.h_bar_d_out(:) - r0.h_bar_true_out(:));       % y = w - w_d
    % believed feedback step fluct: use (1-lc)*x3_hat as before (needs XU -> use delta_x_hat_3)
    st = (1 - lc) * r0.delta_x_hat_3_out(:, ax) / Rr;
    DWc_ST(:, sd) = dw(mh);  QQ(:, sd) = st(mh); %#ok<SAGROW>
    AB(:, sd) = r0.a_true_out(mh, ax) / a_nom; %#ok<SAGROW>
    APH(:, sd) = (r0.b_hat_out(mh, ax) .* (1 - r0.a_bar_hat_out(mh, ax)).^2) ./ max(r0.a_bar_hat_out(mh, ax), 0.02); %#ok<SAGROW>
end
dc = DWc_ST - mean(DWc_ST, 2);  sc = QQ - mean(QQ, 2);
C_meas = mean(mean(dc .* sc, 2));
abar = mean(AB(:));
a_o = 1 / 2.25 * 0.014706;                                   % a_o = a_nom/R  [1/pN um] -> dimensionless via kappa
kappa_T = 4 * kB * T / Rr * a_o;                             % [-] per spec kappa_T = 4 kB T a_o / R
q = kappa_T * abar;                                           % Var(u) per step
rn = (r0.tout(2) > 0) * (3.31e-3 / 2.25)^2;                  % z meas noise 0.00331 um -> R units, squared
M = momfun(q, rn);
fprintf('hold: kappa_T = %.3e  a_bar = %.4f  q = %.3e  r = %.3e\n', kappa_T, abar, q, rn);
fprintf('measured  <y*step> (hold, 8 seeds)  = %+.3e\n', C_meas);
fprintf('predicted C_ws (moment equations)    = %+.3e    ratio pred/meas = %.2f\n', M.Cws, M.Cws / C_meas);
fprintf('measured Var(y) = %.3e   predicted V = %.3e   ratio %.2f\n', mean(var(DWc_ST, 0, 2)), M.V, M.V / mean(var(DWc_ST, 0, 2)));
% implied row-3 mean forcing in the hold: (a'/a)*C_ws vs measured G3 hold mean (-1.8e-5 +- 2.4)
fprintf('implied g3(hold) = (a''/a)*C_ws = %+.3e   (measured G3 hold mean: -1.8e-5 +- 2.4e-5 [canonical was -2.61e-5 Meng]; canonical comp hold: -1.8e-5)\n', mean(APH(:)) * M.Cws);
fprintf('T2 DONE\n');

function M = local_moments(lc, q, r)
    b = 1 - lc;
    % unknown second moments: V=E[y^2], c1=E[y u1], c2=E[y u2]  (stationary)
    % propagation: y+ = lc y + b u1 + b u2 + u0 + b n0 ; u1+ = u0 ; u2+ = u1
    % E[y+ u1+] = E[(...) u0] = q            -> c1 = q
    % E[y+ u2+] = E[(...) u1] = lc c1 + b q  -> c2 = lc*q + b*q = q
    c1 = q;  c2 = lc * q + b * q;
    % V: E[y+^2] = lc^2 V + b^2 q + b^2 q + q + b^2 r + 2 lc b c1 + 2 lc b c2
    V = (2 * b^2 * q + q + b^2 * r + 2 * lc * b * (c1 + c2)) / (1 - lc^2);
    % step_c = b*(-y + u1 + u2 + n)  (n fresh w.r.t. y)
    Cws = b * (-V + c1 + c2);
    M = struct('V', V, 'c1', c1, 'c2', c2, 'Cws', Cws);
end
