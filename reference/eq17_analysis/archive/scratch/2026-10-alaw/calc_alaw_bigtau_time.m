% STATUS: SCRATCH -- 2026-10-19: the a law with tau chosen so the start speed equals the validated 21.1 R/s, true a and a' in the law
%   (no estimator): dh/dt = -(a - a_d)/(a' tau), plane a_d 0.07. Prints tau and the time to reach given heights.
function calc_alaw_bigtau_time()
    ad = 0.07;  hs = 1.078;  d = 1e-4;  A = @(h) 1 / local_c(h);
    v0 = (A(22.2) - ad) / ((A(22.2 + d) - A(22.2 - d)) / (2 * d));  tau = v0 / 21.1;
    fprintf('tau = %.1f s (start speed = (a - a_d)/a'' / tau = 21.1 R/s)\n', tau);
    h = 22.2;  t = 0;  dt = 2e-3;  marks = [10 5 3 2 1.5 1.2 1.1];  im = 1;
    while h > hs + 0.01 && t < 600
        a = A(h);  ap = (A(h + d) - A(h - d)) / (2 * d);  v = (a - ad) / (ap * tau);  h = h - v * dt;  t = t + dt;
        if im <= numel(marks) && h <= marks(im); fprintf('  h %4.1f R at t = %6.1f s (speed %.3f R/s)\n', marks(im), t, v);  im = im + 1; end
    end
    fprintf('  h* + 0.01 R at t = %.1f s\n', t);
end

function c = local_c(h)
    [~, c] = calc_correction_functions(h);
end
