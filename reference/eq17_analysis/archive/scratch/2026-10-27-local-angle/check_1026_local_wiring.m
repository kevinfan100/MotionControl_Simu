% STATUS: SCRATCH -- 2026-10-26 (user): wiring check of the local version (run_t3_xyz_fixed form 'local', 1026_local_angle_estimator.tex).
%   Criteria written before the run:
%   (a) F_e in the code vs one step of the true process (Section 3) minus the estimator f (Section 4), written here independently,
%       feedback and eps_p left out: residual / linear part shrinks ~10x when the error shrinks 10x (second order). The true off-diagonal
%       elements are set from the angle error, a_vw = (a_par - a_perp)(theta_x - theta-hat_x) (Section 2), so the angle rows are checked too.
%   (b) H: process outputs vs estimates of the same quantities -> 0 / 1 (by construction, printed).
%   (c) health over 2 s (20 deg wall, 15 um, theta-hat[0] = (0, 15) deg, gains at the seeds), without and with the gate (test option):
%       no NaN, no contact, theta-hat_y within 60 deg of the wall.
here = fileparts(mfilename('fullpath'));  addpath(here);
o = struct('theta_y', 20, 'h0', 15 / 2.25, 'seeds', 1, 'T', 2, 'aprime', true, 'form', 'local', 'th0_deg', [0 15], 'gain0', 'seed', ...
           'dump_k', 30, 'tag', 'w26');   % early step, before any divergence
R = run_t3_xyz_fixed(o);  r = R.runs{1};  D = r.dump;
fprintf('dump at step 30: angle step taken (go) = %d, theta-hat there (%.2f, %.2f)\n', D.go, D.xk(16:17) * 180 / pi);
rng(3);  E0 = randn(17, 1);  E0(12:13) = 0;  sc = [1e-3 1e-4];  rat = zeros(1, 2);  rw = zeros(2, 17);
for is = 1:2
    e = sc(is) * E0;  xh = D.xk;  xt = xh + e;
    dg = xt(10) - xt(11);                                       % true off-diagonal elements from the angle error (Section 2)
    xt(13) = dg * e(16);  xt(12) = -dg * cos(xh(16)) * e(17);  e = xt - xh;
    act = local_true(xt, xh, D) - D.fx;  pred = D.Fe * e;
    rat(is) = max(abs(act - pred)) / max(abs(pred));  rw(is, :) = abs(act - pred)' ./ max(abs(pred));
end
fprintf('(a) F_e: residual / linear part %.2e (error 1e-3), %.2e (error 1e-4), ratio %.1f -> %s\n', rat, rat(1) / rat(2), ...
        local_pf(rat(1) / rat(2) > 5));
[~, iw] = max(rw(2, :));  fprintf('    largest residual row at error 1e-4: %d (%.1e)\n', iw, rw(2, iw));
fprintf('(b) H = [I3 0 0 0 0 0; 0 0 0 I4 0 0] (process outputs (uu+vv)/2, ww, uw, vw vs a_par, a_perp, a_uw, a_vw)\n');
for gg = [0 2]
    og = o;  og = rmfield(og, 'dump_k');  og.tag = sprintf('w26_g%d', gg);  if gg > 0; og.gap_gate = gg; end
    Rg = run_t3_xyz_fixed(og);  rg = Rg.runs{1};  k = 1:rg.kend;  bad = any(isnan(rg.thy(k))) || any(abs(rg.thy(k) - 20) > 60);
    fprintf('(c) gate %d: NaN %d, contact %d, |theta-hat_y - 20| > 60 deg: %d -> %s | theta-hat at 2 s (%.2f, %.2f)\n', gg, ...
            sum(isnan(rg.thy(k))), rg.contact, any(abs(rg.thy(k) - 20) > 60), local_pf(~bad && ~rg.contact), rg.thx(rg.kend), rg.thy(rg.kend));
end

function xn = local_true(x, xh, D)                         % Section 3 true state (eps_p = 0), local coordinate of V-hat[k]
    lc = D.lc;  F = D.vF;
    At = [x(10) 0 x(12); 0 x(10) x(13); x(12) x(13) x(11)];  Ah = [xh(10) 0 xh(12); 0 xh(10) xh(13); xh(12) xh(13) xh(11)];
    w = [cos(xh(16)) * (x(17) - xh(17)); -(x(16) - xh(16)); 1];
    M = D.dpd + (1 - lc) * x(7:9);  mv = (At - Ah) * F;  dh = w' * (M + mv);
    xn = x;  xn(1:3) = x(4:6);  xn(4:6) = x(7:9);  xn(7:9) = lc * x(7:9) - mv;
    xn(10) = x(10) + x(14) * dh;  xn(11) = x(11) + x(15) * dh;
    rs = (x(14) - x(15)) / (x(10) - x(11));  xn(12) = x(12) * (1 + rs * dh);  xn(13) = x(13) * (1 + rs * dh);
    % angle: true theta is constant; the error after the step uses theta-hat[k+1] = theta-hat[k] + delta-theta-hat[k] (in D.fx), so the
    % true state entry is theta itself: x(16:17)
end

function s = local_pf(ok)
    if ok; s = 'PASS'; else; s = 'FAIL'; end
end
