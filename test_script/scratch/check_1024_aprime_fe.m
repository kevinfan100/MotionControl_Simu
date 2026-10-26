% STATUS: SCRATCH -- 2026-10-24: one-step check of 1024_estimator_xyz_aprime.tex Section 5 (e_a row of F_e, incl. Section 4).
%   True and estimated states with small errors; propagate the nonlinear gain update (Section 2 true, Section 3 estimator without L);
%   compare the actual change of e_a with the F_e prediction. Pass = residual falls ~4x when the errors halve (second order only).
%   EXPIRES: when the controller with A-hat' is coded and checked
Rx = @(a) [1 0 0; 0 cos(a) -sin(a); 0 sin(a) cos(a)];  Ry = @(a) [cos(a) 0 sin(a); 0 1 0; -sin(a) 0 cos(a)];
sym6 = @(v) [v(1) v(4) v(5); v(4) v(2) v(6); v(5) v(6) v(3)];  vec6 = @(A) [A(1,1); A(2,2); A(3,3); A(1,2); A(1,3); A(2,3)];
G = @(v) [v(1) 0 0 v(2) v(3) 0; 0 v(2) 0 v(1) 0 v(3); 0 0 v(3) 0 v(1) v(2)];
lc = 0.4;  V = Ry(deg2rad(25)) * Rx(deg2rad(-7));
Atrue = V * diag([0.6 0.6 0.3]) * V';  a = vec6(Atrue);  ap = vec6(V * diag([0.3 0.3 0.5]) * V');
dpd = [0.01; -0.02; 0.015];  F = [0.05; 0.03; -0.04];  dp3 = [0.002; -0.001; 0.003];
rng(5);  E0 = randn(6, 1);  Ep0 = randn(6, 1);  e30 = randn(3, 1);
for sc = [1e-3 5e-4 2.5e-4]
    ea = sc * E0;  eap = sc * Ep0;  e3 = 0.1 * sc * e30;
    ah = a - ea;  aph = ap - eap;  Ah = sym6(ah);
    [w, ~, ~] = plw(Atrue, Rx, Ry);  [wh, thh, Vh] = plw(Ah, Rx, Ry);
    M = dpd + (1 - lc) * dp3;  Mh = dpd + (1 - lc) * (dp3 - e3);
    ea_next = (a + ap * (w' * (M + (Atrue - Ah) * F))) - (ah + aph * (wh' * Mh));
    hx = Ah(1,1) - Ah(3,3);  vx = -2 * Ah(1,3);  hy = Ah(2,2) - Ah(3,3);  vy = -2 * Ah(2,3);
    dthy = (hx * (-2 * ea(5)) + 2 * Ah(1,3) * (ea(1) - ea(3))) / (2 * (hx^2 + vx^2));
    dphi = (hy * (-2 * ea(6)) + 2 * Ah(2,3) * (ea(2) - ea(3))) / (2 * (hy^2 + vy^2));
    T = tan(atan2(vy, hy) / 2);  c = cos(thh(2));  s = sin(thh(2));  den = 1 + T^2 * c^2;
    dthx = -(c * (1 + T^2) / den) * dphi + (T * s / den) * dthy;
    pred = (1 - lc) * aph * (wh' * e3) + ea + aph * (wh' * G(F) * ea) ...
           + aph * (-Vh(:,2)' * Mh * dthx + cos(thh(1)) * Vh(:,1)' * Mh * dthy) + (wh' * Mh) * eap;
    fprintf('scale %.1e: |actual - linear| = %.3e   (|change of e_a| %.3e)\n', sc, norm(ea_next - pred), norm(ea_next - ea));
end

function [w, th, V] = plw(A, Rx, Ry)
    thy = atan2(-2 * A(1,3), A(1,1) - A(3,3)) / 2;  phi = atan2(-2 * A(2,3), A(2,2) - A(3,3)) / 2;  thx = -atan(tan(phi) * cos(thy));
    V = Ry(thy) * Rx(thx);  w = V(:, 3);  th = [thx; thy];
end
