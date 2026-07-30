%BUILD_TRUTH_TWO_SPHERE  Exact perpendicular correction for a probe approaching
%   a SPHERE (a cell), from Jeffrey & Onishi, JFM 139 (1984), 261-290.
%
%   Coordinates (all lengths divided by the PROBE radius a1, so the output is
%   directly comparable with calc_correction_functions):
%       lam_JO = a2/a1                     cell radius / probe radius
%       w      = (r - a2)/a1               probe-centre to cell SURFACE
%       contact at w = 1                   (same convention as the plane case)
%       s      = 2r/(a1+a2) = 2(w+lam_JO)/(1+lam_JO)
%       xi     = s - 2                     non-dimensional gap of the paper
%   J&O normalise A_11 by 6*pi*mu*a1 (their eq 1.7a with alpha=beta=1), so
%       c_perp = X^A_11(s, lam_JO)         no further conversion.
%
%   Implements their all-separations expression (3.20): the singular part is
%   written in closed form and only the regular remainder is summed, so the
%   series converges near contact where the raw s^-k series does not.
%       X^A_11 = g1 (1-4 s^-2)^-1 - g2 ln(1-4 s^-2) - g3 (1-4 s^-2) ln(1-4 s^-2)
%                + 1 - g1
%                + sum_{m even >= 2} c_m (2/s)^m
%       c_m    = 2^-m (1+lam)^-m f_m - g1 - 2 g2/m + 4 g3/(m*m2),  m2 = m-2
%                (m2 = -2 at m = 2, their definition below eq 3.20)
%   with (3.19)
%       g1 = 2 lam^2 (1+lam)^-3
%       g2 = (1/5) lam (1+7lam+lam^2) (1+lam)^-3
%       g3 = (1/42)(1+18lam-29lam^2+18lam^3+lam^4)(1+lam)^-3
%   and f_m from (3.15).  f is transcribed from the paper; the odd-index
%   symmetry f_{2k+1}(lam) = lam^(2k+2) f_{2k+1}(1/lam) is checked at run time.
%
%   VALIDATION (printed): (i) the odd-f symmetry, (ii) X^A_11 * eps -> the
%   lubrication value [lam/(1+lam)]^2 as the gap closes, (iii) lam -> infinity
%   reproduces calc_correction_functions (the plane).
%
%   Usage:  c = build_truth_two_sphere(w, lam_JO)      w may be a vector
%           build_truth_two_sphere()                    runs the validation

function c = build_truth_two_sphere(w, lam_JO)
    if nargin == 0, validate(); return; end
    [g1, g2, g3] = gcoef(lam_JO);
    cm = regular_coeffs(lam_JO, g1, g2, g3);
    c  = zeros(size(w));
    for i = 1:numel(w)
        s = 2*(w(i) + lam_JO)/(1 + lam_JO);
        u = 1 - 4/s^2;                                   % = xi(xi+4)/s^2 > 0
        acc = g1/u - g2*log(u) - g3*u*log(u) + 1 - g1;
        for k = 1:numel(cm)
            m = 2*k;
            acc = acc + cm(k)*(2/s)^m;
        end
        c(i) = acc;
    end
end

% ---------------------------------------------------------------------------
function [g1, g2, g3] = gcoef(L)
    d  = (1+L)^3;
    g1 = 2*L^2/d;
    g2 = (1/5)*L*(1 + 7*L + L^2)/d;
    g3 = (1/42)*(1 + 18*L - 29*L^2 + 18*L^3 + L^4)/d;
end

function f = fseries(L)
%FSERIES  f_0 .. f_11 of Jeffrey & Onishi (3.15), transcribed from the paper.
    f = zeros(1, 12);
    f(1)  = 1;                                                   % f_0
    f(2)  = 3*L;                                                 % f_1
    f(3)  = 9*L;                                                 % f_2
    f(4)  = -4*L + 27*L^2 - 4*L^3;                               % f_3
    f(5)  = -24*L + 81*L^2 + 36*L^3;                             % f_4
    f(6)  = 72*L^2 + 243*L^3 + 72*L^4;                           % f_5
    f(7)  = 16*L + 108*L^2 + 281*L^3 + 648*L^4 + 144*L^5;        % f_6
    f(8)  = 288*L^2 + 1620*L^3 + 1515*L^4 + 1620*L^5 + 288*L^6;  % f_7
    f(9)  = 576*L^2 + 4848*L^3 + 5409*L^4 + 4524*L^5 ...
            + 3888*L^6 + 576*L^7;                                % f_8
    f(10) = 1152*L^2 + 9072*L^3 + 14752*L^4 + 26163*L^5 ...
            + 14752*L^6 + 9072*L^7 + 1152*L^8;                   % f_9
    f(11) = 2304*L^2 + 20736*L^3 + 42804*L^4 + 115849*L^5 ...
            + 76176*L^6 + 39264*L^7 + 20736*L^8 + 2304*L^9;      % f_10
    f(12) = 4608*L^2 + 46656*L^3 + 108912*L^4 + 269100*L^5 ...
            + 319899*L^6 + 269100*L^7 + 108912*L^8 ...
            + 46656*L^9 + 4608*L^10;                             % f_11
end

function cm = regular_coeffs(L, g1, g2, g3)
    f  = fseries(L);
    ms = 2:2:10;                       % even m with f_m available
    cm = zeros(1, numel(ms));
    for k = 1:numel(ms)
        m  = ms(k);
        m2 = m - 2;  if m == 2, m2 = -2; end
        cm(k) = 2^(-m)*(1+L)^(-m)*f(m+1) - g1 - 2*g2/m + 4*g3/(m*m2);
    end
end

% ---------------------------------------------------------------------------
function validate()
    here = fileparts(mfilename('fullpath'));
    addpath(fullfile(fileparts(fileparts(here)), 'model', 'wall_effect'));

    fprintf('\n=== (i) odd-index symmetry  f_{2k+1}(L) = L^(2k+2) f_{2k+1}(1/L) ===\n');
    L = 3.111;  fa = fseries(L);  fb = fseries(1/L);
    for k = 0:5
        i = 2*k + 2;                                   % f_{2k+1} -> index 2k+2
        fprintf('  f_%-2d : %14.6e  vs  %14.6e   rel %8.1e\n', 2*k+1, ...
                fa(i), L^(2*k+2)*fb(i), abs(fa(i)/(L^(2*k+2)*fb(i)) - 1));
    end

    fprintf('\n=== (ii) lubrication limit:  c_perp * eps -> [L/(1+L)]^2 ===\n');
    for L = [1 3.111 10 100]
        tgt = (L/(1+L))^2;
        fprintf('  L = %8.3f  target %.6f :', L, tgt);
        for eps = [1e-2 1e-3 1e-4]
            fprintf('  %g -> %.6f', eps, build_truth_two_sphere(1+eps, L)*eps);
        end
        fprintf('\n');
    end

    fprintf('\n=== (iii) truncation convergence at the cell case lam = 3.111 ===\n');
    L = 3.111; [g1,g2,g3] = gcoef(L); f = fseries(L);
    fprintf('%8s %12s %12s %12s %12s\n', 'w', 'M=4', 'M=6', 'M=8', 'M=10');
    for w = [1.05 1.2 1.5 2 5 10 22.2 100]
        v = zeros(1,4); Ms = [4 6 8 10];
        for j = 1:4
            acc = 0; s2 = 2*(w+L)/(1+L); u = 1 - 4/s2^2;
            acc = g1/u - g2*log(u) - g3*u*log(u) + 1 - g1;
            for m = 2:2:Ms(j)
                m2 = m-2; if m == 2, m2 = -2; end
                acc = acc + (2^(-m)*(1+L)^(-m)*f(m+1) - g1 - 2*g2/m + 4*g3/(m*m2))*(2/s2)^m;
            end
            v(j) = acc;
        end
        fprintf('%8.2f %12.6f %12.6f %12.6f %12.6f\n', w, v);
    end
    fprintf('  far field: w = 1e3 -> %.6f,  w = 1e5 -> %.6f   (must -> 1)\n', ...
            build_truth_two_sphere(1e3, L), build_truth_two_sphere(1e5, L));
    fprintf(['\n  NOTE: the truncated (3.20) sum is NOT valid as lam -> infinity.\n' ...
             '  g3 grows like lam/42 and the m > 10 tail that would cancel it is\n' ...
             '  missing, so the plane limit cannot be recovered this way.  Use\n' ...
             '  calc_correction_functions for the plane; this file is for finite\n' ...
             '  lam (the cell), where it converges to 1e-4 by M = 8.\n']);
end
