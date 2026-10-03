function cp = sphere_cperp_fig11(h_bar)
%SPHERE_CPERP_FIG11  True c_perp(h_bar) of Meng's Fig. 11 sphere, for the plant (opts.plant_cperp of run_formC_b).
%   PURPOSE (2026-09-21): the shape-type test wall of the wall-test protocol (.claude/rules/wall-test-protocol.md).
%   Same recipe as cell_cperp_fig11.m (C60): ONE smooth function a_bar = 1 + sum_{k=1}^{12} c_k h_bar^-k, least squares on
%     1.00 <= h_bar <= 1.5    ONE cubic through contact (a = 0 at h = 1, rigid sphere) least-squares fitted to the digitised points
%                             1.03-1.5 R (their local slopes ripple 0.3-1.0; rms 0.0027 = digitisation noise; concave, no junction)
%     1.5  <= h_bar <= 4.5    the digitised curve (reference/eq17_analysis/data/meng_11072733_fig11_three_surfaces.csv)
%     4.5  <= h_bar <= 25     the [1/1] band fit a = (h + c0)/(h + d1) of the sphere data on 1.1-4.5 R, as pseudo-data
%   fitted with lsqlin under d a_bar/d h >= 0 on a grid of [1, 25] (the unconstrained fit had a hook on 1.00-1.03 R).
%   h_bar = commanded height above the visible sphere surface (Fig. 11 x-axis), contact at 1; c_perp = 1 / a_bar. Vectorised.
%   Console on first call: the [1/1] band fit, contact slope, data rms, plant-law handover, monotonicity over [1, 25].
%   FORK OF cell_cperp_fig11.m | EXPIRES: with the wall-test protocol line | 產線改動不會自動跟上
    persistent cfit nfit
    if isempty(cfit)
        here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
        T = readtable(fullfile(root, 'reference', 'eq17_analysis', 'data', 'meng_11072733_fig11_three_surfaces.csv'));
        m = strcmp(T.surface, 'sphere');  x = T.hd_R(m);  y = T.lambda_norm(m);
        kb = x >= 1.1 & x <= 4.5;                                     % the [1/1] band fit: c0 - a d1 = h (a - 1), linear LS
        cd = [ones(nnz(kb), 1), -y(kb)] \ (x(kb) .* (y(kb) - 1));  C0_11 = cd(1);  D1_11 = cd(2);
        % near wall (user 09-21, fourth version): the digitised points on 1.03-1.5 R carry a slope ripple (local slopes 0.3-1.0) that
        % the 12th-order fit followed as a shoulder in a'; a straight-line patch and a two-piece arc + cubic both left a kink at the
        % junction. Now ONE cubic through contact, a = A1 (h-1) + A2 (h-1)^2 + A3 (h-1)^3 (a = 0 at h = 1, rigid sphere), least squares
        % on the digitised points 1.03-1.5 R: rms 0.0027 (the digitisation noise), concave, a' decreasing 1.02 -> 0.97 -> 0.86 -> 0.47
        % at 1.0 / 1.03 / 1.1 / 1.5 R, no junction below 1.5 R. Raw data take over from 1.5 R.
        H_SM = 1.5;
        kn = x >= 1.03 & x <= H_SM;  dk = x(kn) - 1;  cq = [dk, dk.^2, dk.^3] \ y(kn);
        xp = linspace(1.0, H_SM, 150)';  dp = xp - 1;  yp = [dp, dp.^2, dp.^3] * cq;
        Aq = cq(1);                                                       % contact slope, reported
        k = x > H_SM & x <= 4.5;  x = x(k);  y = y(k);  nfit = 12;
        xt = linspace(4.5, 25, 600)';  yt = (xt + C0_11) ./ (xt + D1_11);
        xa = [xp; x; xt];  ya = [yp; y; yt];
        Aall = (1 ./ xa) .^ (1:nfit);  yall = ya - 1;
        hg = [linspace(1.0, 1.3, 200), linspace(1.3, 25, 400)]';  ug = 1 ./ hg;   % constraint grid
        Aineq1 = (ug .^ (2:nfit + 1)) .* (1:nfit);                    % monotone: a' = -sum k c_k u^(k+1) >= 0  <=>  sum k c_k u^(k+1) <= 0
        Aineq2 = (ug .^ (3:nfit + 2)) .* ((1:nfit) .* (2:nfit + 1));  % concave: a'' = sum k(k+1) c_k u^(k+2) <= 0 (the slope must not rise
        %   towards the wall -- the unconstrained fit followed the digitisation ripple of the first points and had a'
        %   increasing from 0.78 at 1.03 R to 0.81 at 1.10 R; user 09-21)
        opt = optimoptions('lsqlin', 'Display', 'off');
        Aeq = ones(1, nfit);                                            % a(1) = 0 exactly (contact): 1 + sum c_k = 0 -- keeps the clamp a >= 1e-3 from
        cfit = lsqlin(Aall, yall, [Aineq1; Aineq2], zeros(2 * numel(hg), 1), Aeq, -1, [], [], [], opt);   % flattening a on [1, 1.001] (plotted slope fell to 0 there)
        af = @(h) 1 + ((1 ./ h(:)) .^ (1:nfit)) * cfit;
        law = @(h) (h(:) + C0_11) ./ (h(:) + D1_11);
        hchk = linspace(4.5, 25, 300)';  hm = linspace(1.0, 25, 4000)';  am = af(hm);
        fprintf(['[sphere_cperp_fig11] [1/1] band fit 1.1-4.5 R (c0, d1) = (%.4f, %.4f) | near wall: one cubic through contact on [1, %.1f] R (contact slope %.3f) | data rms (1.5-4.5 R) %.4f | ' ...
                 'plant-law above 4.5 R max %.4f | a_bar(1.0) %.4f a_bar(1.1) %.4f a_bar(2.0) %.4f a_bar(4.5) %.4f (data %.4f) a_bar(22.2) %.4f | monotone on [1, 25]: %d | min %.4f\n'], ...
                C0_11, D1_11, H_SM, Aq, rms(y - af(x)), max(abs(af(hchk) - law(hchk))), af(1.0), af(1.1), af(2.0), af(4.5), y(end), af(22.2), all(diff(am) > 0), min(am));
    end
    h = max(h_bar, 1.0);
    u = 1 ./ h(:);
    a = reshape(1 + (u .^ (1:nfit)) * cfit, size(h));
    a = max(a, 1e-3);
    cp = 1 ./ a;
end
