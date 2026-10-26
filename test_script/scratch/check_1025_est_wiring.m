% STATUS: SCRATCH -- 2026-10-25 (user, test 1): wiring check of the estimated-angle arm (run_t3_xyz_fixed opts.form = 'estimated',
%   1025_computed_estimated_angle.tex Section 6). Criteria written before the run:
%   (a) H gain rows in the code vs finite difference of A-hat = V(theta) diag(a_par, a_par, a_perp) V': max rel. error < 1e-6;
%   (b) F_e in the code vs one step of the true process and the estimator written here independently (eps_p = 0, feedback left out):
%       residual / linear part shrinks 10x when the error shrinks 10x (residual second order);
%   (c) health over 2 s (20 deg wall, 15 um, theta-hat[0] 5 deg off): no NaN, no contact, eigenvalue floor count not above the
%       computed-angle arm on the same seed. | EXPIRES: when test 2 (comparison with 1015) is done
here = fileparts(mfilename('fullpath'));  addpath(here);
o = struct('theta_y', 20, 'h0', 15 / 2.25, 'seeds', 1, 'T', 2, 'aprime', true, 'tag', 'w1');
oe = o;  oe.form = 'estimated';  oe.th0_deg = [0 15];  oe.dump_k = 1000;
Re = run_t3_xyz_fixed(oe);  re = Re.runs{1};  Rc = run_t3_xyz_fixed(o);  rc = Rc.runs{1};
D = re.dump;
% ---- (a) H ----
eH = max(abs(D.Ja(:) - D.Jfd(:))) / max(abs(D.Jfd(:)));
fprintf('(a) H vs finite difference: max rel. error %.2e  -> %s\n', eH, local_pf(eH < 1e-6));
% ---- (b) F_e ----
rng(11);  E0 = randn(15, 1);  rat = zeros(1, 2);  sc = [1e-4 1e-5];
for is = 1:2
    e = sc(is) * E0;  xh = D.x;  xt = xh + e;
    act = local_step(xt, xh, D, 'true') - local_step(xh, xh, D, 'est');
    pred = D.Fe * e;
    rat(is) = max(abs(act - pred)) / max(abs(pred));
end
fprintf('(b) F_e one step: residual / linear part %.2e (error 1e-4), %.2e (error 1e-5), ratio %.1f  -> %s\n', rat, rat(1) / rat(2), ...
        local_pf(abs(rat(1) / rat(2) - 10) < 2));
% ---- (c) health ----
nn = sum(isnan(re.thy(1:re.kend)));
fprintf('(c) estimated: NaN %d, contact %d, floor %d, min eig %.3f | computed (same seed): floor %d, min eig %.3f  -> %s\n', ...
        nn, re.contact, re.n_floor, min(re.mineig), rc.n_floor, min(rc.mineig), local_pf(nn == 0 && ~re.contact && re.n_floor <= rc.n_floor));
fprintf('    end of 2 s: theta-hat (%.2f, %.2f) deg vs (0, 20); a_par %.3f vs %.3f, a_perp %.3f vs %.3f\n', ...
        re.thx(re.kend), re.thy(re.kend), re.aT(re.kend), re.aT_true(1), re.aN(re.kend), re.aN_true(1));

function xn = local_step(x, xh, D, mode)                     % one step of Section 6 written independently (eps_p = 0, no feedback)
    lc = D.lc;  F = [D.GF(1,1); D.GF(2,2); D.GF(3,3)];
    Wf = @(th) [sin(th(2)) * cos(th(1)); -sin(th(1)); cos(th(2)) * cos(th(1))];
    Af = @(z) z(10) * eye(3) + (z(11) - z(10)) * (Wf(z(14:15)) * Wf(z(14:15))');
    xn = x;  xn(1:3) = x(4:6);  xn(4:6) = x(7:9);
    if strcmp(mode, 'true')
        dA = Af(x) - Af(xh);
        xn(7:9) = lc * x(7:9) - dA * F;
        s = Wf(x(14:15))' * (D.dpd + (1 - lc) * x(7:9) + dA * F);
    else
        xn(7:9) = lc * x(7:9);
        s = Wf(x(14:15))' * (D.dpd + (1 - lc) * x(7:9));
    end
    xn(10) = x(10) + x(12) * s;  xn(11) = x(11) + x(13) * s;
end

function s = local_pf(ok)
    if ok; s = 'PASS'; else; s = 'FAIL'; end
end
