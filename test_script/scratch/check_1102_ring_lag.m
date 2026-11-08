% STATUS: SCRATCH -- 2026-11-02 (user): is the w-hat error in the ring a lag or noise. Fixed 36 um, 5 seeds of temp_square_why.mat. [hypothesis] lag, w-hat follows the true w late while w turns about 1 deg per step; rival: noise. Split: (1) angle to the true w shifted by s samples, a lag gives a clear minimum at s > 0; (2) error along and across the turning direction, a lag gives a one-signed along part, by rate of turn. Sanity: s = 0 repeats the ring table. No figure.
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');  load(fullfile(fo, 'temp_square_why.mat'), 'rs', 'A');  id = find([A.a] == 3);  SS = [0 5 10 20 40 80];  Rg = cell(numel(id), 1);
M = NaN(numel(SS), 2);  allA = [];  allC = [];  allR = [];
for j = 1:numel(SS)
    s = SS(j);  E = [];
    for i = 1:numel(id)
        r = rs{id(i)};  ok = find(isfinite(r.aN(1:r.kend)));  k = ok(r.t(ok) >= r.tsc + 0.5 & r.t(ok) <= r.te);  k = k(k > s + 1);  rr = hypot(r.p(k, 1), r.p(k, 2));  m = rr > 7.2 & rr < 9;  kr = k(m);  a = local_w(r.thx(kr), r.thy(kr));  b = local_w(r.th_tr(kr - s, 1), r.th_tr(kr - s, 2));  E = [E; acosd(min(sum(a .* b, 2), 1))];
        if s == 0
            b0 = local_w(r.th_tr(kr - 1, 1), r.th_tr(kr - 1, 2));  db = movmean(b - b0, 11, 1);  n = vecnorm(db, 2, 2);  rate = asind(min(n, 1));  d = db ./ max(n, 1e-9);  e = a - b;  al = asind(max(min(sum(e .* d, 2), 1), -1));  cr = cross(b, d, 2);  ac = asind(max(min(sum(e .* cr, 2), 1), -1));  allA = [allA; al];  allC = [allC; ac];  allR = [allR; rate];
        end
    end
    M(j, :) = [median(E), prctile(E, 95)];  fprintf('shift %2d samples (%.1f ms): angle median %.2f, 95%% %.2f deg\n', s, 1e3 * s / 1612, M(j, 1), M(j, 2));
end
ed = [0 0.05 0.2 0.5 1 2];
for j = 1:numel(ed) - 1
    m = allR >= ed(j) & allR < ed(j + 1);  fprintf('turn rate %.2f-%.2f deg/step (%d samples): along mean %+.2f sd %.2f deg | across mean %+.2f sd %.2f deg\n', ed(j), ed(j + 1), sum(m), mean(allA(m)), std(allA(m)), mean(allC(m)), std(allC(m)));
end
function w = local_w(thx, thy)
    tx = thx(:) * pi / 180;  ty = thy(:) * pi / 180;  w = [sin(ty) .* cos(tx), -sin(tx), cos(ty) .* cos(tx)];
end
