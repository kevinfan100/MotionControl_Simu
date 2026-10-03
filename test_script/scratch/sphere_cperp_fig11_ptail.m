function cp = sphere_cperp_fig11_ptail(h_bar)
%SPHERE_CPERP_FIG11_PTAIL  Sphere plant variant: the Fig. 11 sphere below 4.5 R, relaxing to the PLANE far tail above (opts.plant_cperp).
%   PURPOSE (2026-09-22, user: "handle the sphere far-field extrapolation"): sphere_cperp_fig11.m has data only on 1.03-4.5 R
%   and continues above 4.5 R with the [1/1] band fit (far level 1 - a ~ 0.857/h); the plane's far level is 1.125/h. The two
%   tails differ by 0.268/h (0.040 at 6.67 R, 0.012 at 22.2 R), which is exactly the start-level error the plane-init estimator
%   shows on the sphere. Nothing in the data decides the tail, so this variant BRACKETS it from the other side:
%     h <= 4.5 R     identical to sphere_cperp_fig11 (data + near-contact cubic)
%     h >  4.5 R     a = a_plane(h) + D0 exp(-(h - 4.5)/tau),  D0 = a_sphere(4.5) - a_plane(4.5)  (value-continuous)
%                    tau = D0 / (a'_plane(4.5) - a'_sphere(4.5))                                   (slope-continuous, C^1)
%   The offset to the plane decays exponentially at the rate the joint itself dictates (no free number): the plane's slope at
%   4.5 R is steeper than the sphere's, so the sphere must lose its excess level going outward, and tau ~ 7 R is how fast it
%   does so if it does it smoothly. Monotone and concave on [1, 25] (checked on first call). The first version blended a
%   smoothstep to the plane over 4.5-8 R and forced the offset out over 3.5 R: the slope dipped to 0.009 there and a_true(t)
%   showed a shoulder on the canon descent (user 09-22) -- replaced. The truth for a sphere of comparable radius is expected
%   between the two tails. Vectorised, same clamps as the other plants.
%   FORK OF sphere_cperp_fig11.m | EXPIRES: with the wall-test protocol line | 產線改動不會自動跟上
    persistent D0 TAU
    H1 = 4.5;
    a_pl = @(h) 1 - (9/8) ./ h + (1/2) ./ h.^3 - (57/100) ./ h.^4 + (1/5) ./ h.^5 + (7/200) ./ h.^11 - (1/25) ./ h.^12;
    if isempty(D0)
        d = 1e-4;
        a_s = @(h) 1 ./ sphere_cperp_fig11(h);
        D0  = a_s(H1) - a_pl(H1);
        aps = (a_s(H1 + d) - a_s(H1 - d)) / (2 * d);  app = (a_pl(H1 + d) - a_pl(H1 - d)) / (2 * d);
        assert(app > aps && D0 > 0, 'sphere_cperp_fig11_ptail:joint', 'the C^1 exponential tail needs a_plane steeper than the sphere at the joint and a positive offset');
        TAU = D0 / (app - aps);
        hm = linspace(1.0, 25, 4000)';  am = local_eval(hm, H1, D0, TAU, a_pl);
        apm = diff(am) ./ diff(hm);
        fprintf('[sphere_cperp_fig11_ptail] plane tail above %.1f R: offset D0 %.4f, slopes at the joint sphere %.4f plane %.4f -> tau %.2f R | a_bar(6.67) %.4f (plane %.4f) a_bar(22.2) %.4f (plane %.4f) | monotone %d concave above joint %d\n', ...
                H1, D0, aps, app, TAU, interp1(hm, am, 6.67), a_pl(6.67), interp1(hm, am, 22.2), a_pl(22.2), all(apm > 0), all(diff(apm(hm(2:end-1) > H1 + 0.05)) < 1e-9));
    end
    h = max(h_bar, 1.0);
    a = local_eval(h, H1, D0, TAU, a_pl);
    cp = 1 ./ max(a, 1e-3);
end

function a = local_eval(h, H1, D0, TAU, a_pl)
    a = 1 ./ sphere_cperp_fig11(h);
    k = h > H1;
    a(k) = a_pl(h(k)) + D0 * exp(-(h(k) - H1) / TAU);
end
