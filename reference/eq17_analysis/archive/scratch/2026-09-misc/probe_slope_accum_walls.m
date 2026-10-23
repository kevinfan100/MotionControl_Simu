% PURPOSE (2026-09-22): the offline metric that matches what a slope-propagating a-slot estimator suffers from a wrong slope law:
%   the gain error it would accumulate along a descent from h0 if the measurements never corrected it,
%       E(h) = int_h^{h0} [a'_true(h') - a'_law(h')] dh',     reported as max_h |E(h)| on the band,
%   next to sup|e_a'| (which hides far-field relative errors because a' is small there). Per wall (plane / cell / sphere,
%   sphere also with the plane-tail variant) and per family, constants fitted per wall by least squares on a' unless stated:
%     (i)   (1-c)/(h+delta)^2, delta and c free           -- F2 (0922_estb_f2_slope_dc.tex)
%     (ii)  kappa_plane/(h+delta)^2, delta free           -- one constant, level pinned at the plane's
%     (iii) a'_plane(h+delta), delta free                 -- the delta law in slope form (C65-C70)
%     (iv)  (1-c)/h^2.042, c free                         -- version 2 (0921_estb_v2_slope_c.tex)
%     (v)   (1-c)/h^2.042, c = +0.037 (plane) not learned
%     (vi)  (1-c)/(h+delta)^2 at the plane pair, not learned
%   Numbers quoted in 0922_estb_f2_slope_dc.tex (Seed and prior) and the 09-22 memory. Console only.
%   EXPIRES: with the F2 line
function probe_slope_accum_walls()
    here = fileparts(mfilename('fullpath'));  addpath(here);
    D_CELL = 1.05;  H0 = 22.2;
    Dp    = @(u) -9/8 + 3/2*u.^2 - 228/100*u.^3 + u.^4 + 77/200*u.^10 - 12/25*u.^11;   % D'(u): the plane slope is a' = -D'(1/h)/h^2
    ap_pl = @(h) -Dp(1 ./ h) ./ h.^2;
    ap_ce = @(h) ap_pl(h + D_CELL);
    ap_sp = local_num_slope(@(h) 1 ./ sphere_cperp_fig11(h));
    ap_st = local_num_slope(@(h) 1 ./ sphere_cperp_fig11_ptail(h));
    W = {'plane', ap_pl; 'cell', ap_ce; 'sphere', ap_sp; 'sphere_pt', ap_st};
    hB = linspace(1.10, H0, 8000)';  dh = hB(2) - hB(1);
    kfit  = @(hh, dl, y) (1 ./ (hh + dl).^2) \ y;
    cost2 = @(dl, hh, y) mean((kfit(hh, dl, y) ./ (hh + dl).^2 - y).^2);
    accum = @(e) max(abs(flipud(cumsum(flipud(e))) * dh));
    yp = ap_pl(hB);  dl_p = fminbnd(@(d) cost2(d, hB, yp), -0.9, 3);  kap_p = kfit(hB, dl_p, yp);
    fprintf('\nplane F2 band pair: delta %+.3f  kappa = 1 - c = %.3f (c %+.3f)\n', dl_p, kap_p, 1 - kap_p);
    fprintf('%-6s %-44s %-9s %10s %14s\n', '', 'family (constants per wall, LS on a'')', 'wall', 'sup|e_a''|', 'max|int e dh|');
    for w = 1:size(W, 1)
        apf = W{w, 2};  y = apf(hB);
        d = fminbnd(@(x) cost2(x, hB, y), -0.9, 3);  k = kfit(hB, d, y);  e = y - k ./ (hB + d).^2;
        local_row('(i)', sprintf('(1-c)/(h+delta)^2 both free  (%+.3f, c %+.3f)', d, 1 - k), W{w, 1}, e, accum(e));
        d = fminbnd(@(x) mean((kap_p ./ (hB + x).^2 - y).^2), -0.9, 3);  e = y - kap_p ./ (hB + d).^2;
        local_row('(ii)', sprintf('kappa_plane/(h+delta)^2, delta free (%+.3f)', d), W{w, 1}, e, accum(e));
        d = fminbnd(@(x) mean((ap_pl(hB + x) - y).^2), -0.9, 3);  e = y - ap_pl(hB + d);
        local_row('(iii)', sprintf('a''_plane(h+delta), delta free (%+.3f)', d), W{w, 1}, e, accum(e));
        k = (1 ./ hB.^2.042) \ y;  e = y - k ./ hB.^2.042;
        local_row('(iv)', sprintf('(1-c)/h^2.042, c free (c %+.3f)', 1 - k), W{w, 1}, e, accum(e));
        e = y - (1 - 0.037) ./ hB.^2.042;
        local_row('(v)', '(1-c)/h^2.042, c = +0.037 fixed', W{w, 1}, e, accum(e));
        e = y - kap_p ./ (hB + dl_p).^2;
        local_row('(vi)', 'kappa_plane/(h+delta_plane)^2 fixed', W{w, 1}, e, accum(e));
        fprintf('\n');
    end
end

function local_row(id, fam, wall, e, acc)
    fprintf('%-6s %-44s %-9s %10.3f %14.3f\n', id, fam, wall, max(abs(e)), acc);
end

function apf = local_num_slope(af)
    apf = @(h) (af(max(h, 1 + 2e-3) + 1e-4) - af(max(h, 1 + 2e-3) - 1e-4)) / 2e-4;
end
