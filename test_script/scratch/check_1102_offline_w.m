% STATUS: SCRATCH -- 2026-11-02 (user): how to reduce the move of the recovered wall. The 2.43 um is the geometry; the part to reduce is the error of w-hat. Offline two-sided moving mean of w-hat (window N samples, renormalised), fixed 36 um, 5 seeds of temp_square_why.mat. Wall point error against the wall point with the true w, by region. Sanity: N = 1 gives the raw angle table of run_1102_square_why.m. No figure.
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');  Rr = 2.25;  ad = 0.07;  hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  L = Rr * hs;  load(fullfile(fo, 'temp_square_why.mat'), 'rs', 'A');  NW = [1 21 41 81 161 321];  id = find([A.a] == 3);  sn = {'floor r > 10', 'ring r 7.2-9', 'bead r < 7.2'};
for n = NW
    E = [];  Ang = [];  RR = [];
    for q = id
        r = rs{q};  ok = find(isfinite(r.aN(1:r.kend)));  k = ok(r.t(ok) >= r.tsc + 0.5 & r.t(ok) <= r.te);  We = local_w(r.thx(k), r.thy(k));  Wt = local_w(r.th_tr(k, 1), r.th_tr(k, 2));  Ws = movmean(We, n, 1);  Ws = Ws ./ vecnorm(Ws, 2, 2);  E = [E; L * vecnorm(Ws - Wt, 2, 2)];  Ang = [Ang; acosd(min(sum(Ws .* Wt, 2), 1))];  RR = [RR; hypot(r.p(k, 1), r.p(k, 2))];
    end
    seg = {RR > 10, RR > 7.2 & RR < 9, RR < 7.2};  fprintf('N %3d (%.0f ms, %.1f um):', n, 1e3 * n / 1612, 40 * n / 1612);
    for j = 1:3; m = seg{j}; fprintf(' | %s angle med %.2f 95%% %.1f, wall med %.3f 95%% %.3f', sn{j}, median(Ang(m)), prctile(Ang(m), 95), median(E(m)), prctile(E(m), 95)); end
    fprintf('\n');
end
function w = local_w(thx, thy)
    tx = thx(:) * pi / 180;  ty = thy(:) * pi / 180;  w = [sin(ty) .* cos(tx), -sin(tx), cos(ty) .* cos(tx)];
end
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
