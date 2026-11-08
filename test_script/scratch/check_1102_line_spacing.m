% STATUS: SCRATCH -- 2026-11-02 (user): recovered scan lines on the floor at x > 0 have an uneven spacing and cross near y -5..3. Per line of temp_rho4_report.mat: y of the probe and of the wall point at x = +10, -10, where the line turns, and y travel on the line. Sanity: 27 lines, y at the start near -13.
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');  Rr = 2.25;  ad = 0.07;  hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
load(fullfile(fo, 'temp_rho4_report.mat'), 'r');  ok = find(isfinite(r.aN(1:r.kend)));  ks = ok(r.t(ok) >= r.tsc & r.t(ok) <= r.rs_tend);  p = r.p(ks, :);  tx = r.thx(ks) * pi / 180;  ty = r.thy(ks) * pi / 180;  tx = tx(:);  ty = ty(:);
Wh = [sin(ty) .* cos(tx), -sin(tx), cos(ty) .* cos(tx)];  Vh = [sin(ty) .* sin(tx), cos(tx), cos(ty) .* sin(tx)];  W = movmean(p - Rr * hs * Wh, 41, 1);  dv = [0; sum(diff(p, 1, 1) .* Vh(2:end, :), 2)];  isv = movmean(dv, 21) > 0.012;  stepv = conv(double(isv), ones(61, 1), 'same') > 0;  id = cumsum(diff([0; stepv]) == 1) + 1;
fprintf('line dir |  y start   y end | y at x=+10 (probe, wall) | y at x=-10 (probe, wall) | x at end, z at end, gap at end\n');
for L = 1:max(id)
    k = find(id == L & ~stepv);  x = p(k, 1);  d = sign(x(end) - x(1));  ip = k(find(abs(x - 10) < 0.3, 1));  im = k(find(abs(x + 10) < 0.3, 1));  ye = k(end);
    fprintf('%3d %+d  | %6.2f %6.2f | %6.2f %6.2f | %6.2f %6.2f | %6.1f %5.2f %5.2f\n', L, d, p(k(1), 2), p(ye, 2), local_v(p, ip, 2), local_v(W, ip, 2), local_v(p, im, 2), local_v(W, im, 2), p(ye, 1), p(ye, 3), r.h(ks(ye)) * Rr);
end

function v = local_v(M, i, c)
    if isempty(i); v = NaN; else; v = M(i, c); end
end
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
