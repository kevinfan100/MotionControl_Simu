% PURPOSE (2026-09-07): fit the three measured surfaces of Meng's Fig. 11 (digitised CSV in reference/eq17_analysis/data) with
%   our law family, 1/(1-a) = b (w - w_s)  <=>  a = 1 - 1/(b (w - w_s)), over 1.1-5.5 R, and with the Brenner plane curve
%   shifted / shifted+scaled. Result 09-07: plane (b 0.870, w_s -0.168; Brenner itself rms 0.0016), sphere (b 1.156, w_s +0.162),
%   cell (b 0.877, w_s -1.197): the cell differs from the plane only by the wall position (-1 R below the apparent top), the
%   sphere only by b (weaker wall effect). Form B writing: b_B = 1/b, w_s,B = w_s + b_B => plane (1.15, 0.98), sphere (0.87, 1.03),
%   cell (1.14, -0.06). These (b, w_s) feed run_three_walls.m. | EXPIRES: with the unknown-wall line
function fit_meng_fig11_walls()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(genpath(fullfile(root, 'model')));
    T = readtable(fullfile(root, 'reference', 'eq17_analysis', 'data', 'meng_11072733_fig11_three_surfaces.csv'));
    wg = linspace(1.001, 12, 6000); ag = zeros(size(wg)); for i = 1:numel(wg); [~, c] = calc_correction_functions(wg(i)); ag(i) = 1/c; end
    plane = @(w) interp1(wg, ag, min(max(w, wg(1)), wg(end)));  lawC = @(p, w) 1 - 1 ./ max(p(1) * (w - p(2)), 1e-6);
    S = {'plane','sphere','cell'};
    fprintf('%-7s | law (b, w_s) rms | Form B (b_B, w_s,B) | Brenner rms | shifted plane rms (s) | shift+scale rms (s, k)\n', 'surface');
    for i = 1:3
        m = strcmp(T.surface, S{i}); x = T.hd_R(m); y = T.lambda_norm(m); k = x >= 1.1 & x <= 5.5; x = x(k); y = y(k);
        fC = @(p) rms(y - lawC(p, x)); pC = fminsearch(fC, [0.9 0.1]);
        f1 = @(p) rms(y - plane(x - p(1))); p1 = fminsearch(f1, 0);  f3 = @(p) rms(y - p(2) * plane(x - p(1))); p3 = fminsearch(f3, [0 1]);
        fprintf('%-7s | %.4f (%.3f, %+.3f) | (%.3f, %+.3f) | %.4f | %.4f (%+.3f) | %.4f (%+.3f, %.3f)\n', S{i}, fC(pC), pC, 1/pC(1), pC(2) + 1/pC(1), rms(y - plane(x)), f1(p1), p1, f3(p3), p3);
    end
end
