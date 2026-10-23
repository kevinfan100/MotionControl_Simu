function cp = cell_cperp_fig11(h_bar)
%CELL_CPERP_FIG11  True c_perp(h_bar) of Meng's Fig. 11 cell, for the plant (opts.plant_cperp of run_formC_b).
%   PURPOSE (2026-09-17, tail revised 2026-09-18): the data-faithful cell plant of the [1/1] two-wall test
%   (0916_estb_11_two_versions.tex). ONE smooth function a_bar = 1 + sum_{k=1}^{12} c_k h_bar^-k, least squares on
%     1.03 <= h_bar <= 4.5   the digitised curve (reference/eq17_analysis/data/meng_11072733_fig11_three_surfaces.csv)
%     4.5 <= h_bar <= 25     the [1/1] band fit (g + 1.056)/(g + 2.197), g = h_bar - 1, as pseudo-data
%   The data above 4.5 R are NOT used: their slope there (kap 0.35) is the digitisation artefact seen in the s_eff
%   blow-up above 4.5 R (ledger R64), and at 4.5 R the data and the band fit agree to 0.003, so the handover is seamless.
%   09-17 -> 09-18 revision: the first tail was 1 - 1.141/(g + del) with del set by value continuity to the data at
%   5.5 R (del 3.23); it differed from the band fit by +0.020 at 7 R and showed up as a +0.018 bump at t = 1.1 s of the
%   canon descent in BOTH estimator versions of the T arm (ledger C60, probe_cell_tail_arm.m). Now the plant IS the
%   [1/1] band fit wherever there are no data, so "seed = truth" holds over the whole trajectory.
%   The x-axis of Fig. 11 is the commanded height above the visible cell surface, so h_bar here is the same coordinate
%   the plant uses for the plane (actual wall at h_bar = 1). c_perp = 1 / a_bar. Vectorised.
%   FORK OF nothing | EXPIRES: with the [1/1] two-wall line | 產線改動不會自動跟上
    persistent cfit nfit
    if isempty(cfit)
        here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
        T = readtable(fullfile(root, 'reference', 'eq17_analysis', 'data', 'meng_11072733_fig11_three_surfaces.csv'));
        m = strcmp(T.surface, 'cell');  x = T.hd_R(m);  y = T.lambda_norm(m);
        % the PLANT uses all the digitised points (1.03-5.5 R): the particle dips below the 1.10 R command by its
        % tracking error (min true height 1.06 R), and a polynomial fitted from 1.1 R only curled to a = 0.32 at 1.06 R
        % against 0.50 in the data. (The ESTIMATOR's band fit stays 1.1-5.5 R -- different object.)
        k = x >= 1.03 & x <= 4.5;  x = x(k);  y = y(k);  nfit = 12;
        % ONE smooth function for the whole height range: the data (1.03-4.5 R) plus the [1/1] band fit as pseudo-data
        % from 4.5 R to 25 R, one 12th-order polynomial in 1/h fitted to both (a two-piece plant had a slope kink that
        % the estimator's smooth law showed as an e_a step; a tail with another origin showed as a +0.018 bump, C60).
        C0_11 = 0.056;  D1_11 = 1.197;                               % the [1/1] band fit in x = w - w_s = h_bar, w_s = visible surface (0916_abar_gap_polynomial.tex S6)
        xt = linspace(4.5, 25, 600)';  yt = (xt + C0_11) ./ (xt + D1_11);
        wt = 1.0;                                                     % the law is exact where there are no data
        Aall = [(1 ./ x) .^ (1:nfit); wt * (1 ./ xt) .^ (1:nfit)];  yall = [y - 1; wt * (yt - 1)];
        cfit = Aall \ yall;
        af = @(h) 1 + ((1 ./ h(:)) .^ (1:nfit)) * cfit;
        law = @(h) (h(:) + C0_11) ./ (h(:) + D1_11);
        hchk = linspace(4.5, 25, 300)';
        fprintf(['[cell_cperp_fig11] one smooth fit, data 1.03-4.5 R + [1/1] law 4.5-25 R: data rms %.4f | plant-law above 4.5 R ' ...
                 'max %.4f | a_bar(1.1) %.4f | a_bar(2.0) %.4f | a_bar(4.5) %.4f (data %.4f, law %.4f) | a_bar(22.2) %.4f\n'], ...
                rms(y - af(x)), max(abs(af(hchk) - law(hchk))), af(1.1), af(2.0), af(4.5), y(end), law(4.5), af(22.2));
    end
    h = max(h_bar, 1.0);
    u = 1 ./ h(:);
    a = reshape(1 + (u .^ (1:nfit)) * cfit, size(h));
    a = max(a, 1e-3);
    cp = 1 ./ a;
end
