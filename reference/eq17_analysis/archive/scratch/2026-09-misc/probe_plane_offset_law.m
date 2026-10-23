% PURPOSE (2026-09-18, user re-reading fig11_s_from_fit under the wall-surface coordinate): in that figure the cell's s_eff
%   is flat (~1) only when the contact is placed ~1 R below the visible surface -- i.e. the cell looks like the PLANE with
%   an offset, not a reshaped plane. Test that directly on the three Fig. 11 curves (band 1.1 <= x <= 5.5, x = w - w_s,
%   w_s = visible surface):
%     (a) plane polynomial + ONE position constant:   a(x) = D_perp(1 / (x + delta))        (delta = 0 is the plane)
%     (b) plane polynomial + offset + gap scale:      a(x) = D_perp(1 / (1 + s (x + delta - 1)))
%     (c) [1/1] free contact (c0, d1), for comparison
%   Prints delta, s, rms and max relative error on the band for each surface. If (a) matches the cell to the [1/1] level
%   (2%), the cell needs zero shape parameters: one position-type constant on the exact plane law.
%   EXPIRES: with the [1/1] two-wall line
function out = probe_plane_offset_law()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));
    T = readtable(fullfile(root, 'reference', 'eq17_analysis', 'data', 'meng_11072733_fig11_three_surfaces.csv'));
    Dp = @(u) 1 - 9/8*u + 1/2*u.^3 - 57/100*u.^4 + 1/5*u.^5 + 7/200*u.^11 - 1/25*u.^12;
    fa = @(x, delta) Dp(1 ./ max(x + delta, 1.0));                         % (a) h_eff = x + delta, floored at contact
    fb = @(x, p)     Dp(1 ./ max(1 + p(2) .* (x + p(1) - 1), 1.0));        % (b) p = [delta, s]
    fc = @(x, p)     (x + p(1)) ./ (x + p(2));                              % (c) [1/1] (c0, d1)
    opt = optimset('TolX', 1e-10, 'TolFun', 1e-12, 'MaxFunEvals', 2e4, 'MaxIter', 2e4);
    out = struct();
    fprintf('%-7s | (a) plane+offset: delta   rms    max|rel%%| | (b) +scale: delta   s      rms    max|rel%%| | (c) [1/1]: c0     d1     rms    max|rel%%|\n', 'surface');
    for S = {'plane', 'sphere', 'cell'}
        m = strcmp(T.surface, S{1});  x = T.hd_R(m);  y = T.lambda_norm(m);
        k = x >= 1.1 & x <= 5.5;  x = x(k);  y = y(k);
        rel = @(f) 100 * max(abs((f - y) ./ y));
        da = fminsearch(@(d) rms(y - fa(x, d)), 0, opt);
        pb = fminsearch(@(p) rms(y - fb(x, p)), [da, 1], opt);
        A1 = [y, -ones(size(x))];  p0 = A1 \ (x - y .* x);  pc = fminsearch(@(p) rms(y - fc(x, p)), [p0(2), p0(1)], opt);
        fprintf('%-7s | %+7.3f  %.4f  %6.1f          | %+7.3f  %.3f  %.4f  %6.1f          | %+7.3f  %.3f  %.4f  %6.1f\n', S{1}, ...
                da, rms(y - fa(x, da)), rel(fa(x, da)), pb(1), pb(2), rms(y - fb(x, pb)), rel(fb(x, pb)), ...
                pc(1), pc(2), rms(y - fc(x, pc)), rel(fc(x, pc)));
        out.(S{1}) = struct('delta', da, 'delta_s', pb, 'c0d1', pc);
    end
    fprintf('reading: (a) delta = how far the effective (no-slip) wall sits BELOW the visible surface, in R; plane should give ~0.\n');
end
