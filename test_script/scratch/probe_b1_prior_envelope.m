% FORK OF nothing (new) | PURPOSE: instrument check of the b1 prior (sup |db_true/dw|) the driver derives for
%   ctrl_const.b1_state -- where the supremum sits, whether the gradient is smooth, and what it is on sub-bands
%   | EXPIRES: when b1_state is closed | 產線改動不會自動跟上
addpath(genpath(fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), 'model')));
for band = {[1.00 1.10], [1.05 3.5], [1.10 3.32], [1.10 1.20], [1.20 2.0], [2.0 3.32], [1.10 6.67], [1.10 22.3]}
    lo = band{1}(1); hi = band{1}(2); N = 20001; w = linspace(max(lo, 1.0005), hi, N).'; b = zeros(N, 1);
    for i = 1:N
        [~, cp, d] = calc_correction_functions(w(i), true); a = 1/cp; ap = -d.dc_perp_dh / cp^2; b(i) = ap / (1 - a)^2;
    end
    g = gradient(b, w); [gm, im] = max(abs(g));
    wc = linspace(max(lo, 1.0005), hi, 201).'; bc = interp1(w, b, wc); gc = max(abs(diff(bc) ./ diff(wc)));   % coarse secant check
    fprintf('band [%.2f %.2f]: b in [%.4f %.4f] | sup|db/dw| fine %.4f at w %.4f | coarse secant %.4f | b(lo) %.4f b(hi) %.4f\n', ...
        lo, hi, min(b), max(b), gm, w(im), gc, b(1), b(end));
end
