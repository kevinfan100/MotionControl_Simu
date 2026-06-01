function [c_para, c_perp, derivs] = calc_correction_functions(h_bar, want_derivs)
%CALC_CORRECTION_FUNCTIONS Calculate wall effect correction coefficients
%
%   [c_para, c_perp] = calc_correction_functions(h_bar)
%   [c_para, c_perp, derivs] = calc_correction_functions(h_bar, true)
%   [c_para, c_perp, derivs] = calc_correction_functions(h_bar)  % nargout >= 3
%
%   Calculates the parallel (c_∥) and perpendicular (c_⊥) correction
%   coefficients for the Stokes drag near a planar wall.
%
%   Optionally also returns analytical derivatives of c_para, c_perp with
%   respect to h_bar (needed for Q77 / R(2,2) computation in the
%   eq17_7state controller).
%
%   Inputs:
%       h_bar       - Normalized distance from wall: h_bar = h/R
%                     where h is the distance from wall to particle center
%                     and R is the particle radius
%                     MUST be > 1 (particle not touching wall)
%       want_derivs - (optional, default false) Set true to force
%                     computation of `derivs` even when nargout < 3.
%                     If nargout >= 3, derivs is always computed.
%
%   Outputs:
%       c_para - Parallel correction coefficient (c_∥)
%       c_perp - Perpendicular correction coefficient (c_⊥)
%       derivs - (optional) Struct with fields:
%           dc_para_dh     : dc_para / dh_bar
%           dc_perp_dh     : dc_perp / dh_bar
%           K_h_para       : (1/c_para) * dc_para/dh_bar
%           K_h_perp       : (1/c_perp) * dc_perp/dh_bar
%           K_h_prime_para : dK_h_para / dh_bar
%           K_h_prime_perp : dK_h_perp / dh_bar
%
%   Physical interpretation:
%       - Effective parallel drag: gamma_parallel = gamma_N * c_para
%       - Effective perpendicular drag: gamma_perp = gamma_N * c_perp
%       - Both c_para and c_perp approach 1 as h_bar -> infinity
%       - Both increase as h_bar -> 1 (particle approaches wall)
%       - c_perp > c_para (harder to move perpendicular to wall)
%
%   Reference:
%       Drag_MATLAB.pdf - Wall Effect mathematical model

    if h_bar < 1
        error('calc_correction_functions:invalidInput', ...
            'h_bar must be > 1 (particle must not touch wall). Got h_bar = %.4f', h_bar);
    end

    if nargin < 2 || isempty(want_derivs)
        want_derivs = false;
    end

    % Determine whether the caller actually needs `derivs`
    need_derivs = want_derivs || (nargout >= 3);

    % Compute reusable powers of u = 1/h_bar once (used by c and derivatives)
    u    = 1 / h_bar;
    u2   = u  * u;
    u3   = u2 * u;
    u4   = u3 * u;
    u5   = u4 * u;
    u6   = u5 * u;
    u7   = u6 * u;
    % Higher powers are only needed for c_perp (u^11, u^12) and
    % its derivatives (u^13, u^14)
    u8   = u4 * u4;
    u11  = u8 * u3;
    u12  = u11 * u;
    u13  = u12 * u;
    u14  = u13 * u;

    % Parallel correction coefficient (c_∥)
    % c_para = 1 / D_para,  D_para = 1 - 9/16 u + 1/8 u^3 - 45/256 u^4 - 1/16 u^5
    denom_para = 1 - (9/16)*u + (1/8)*u3 - (45/256)*u4 - (1/16)*u5;
    c_para = 1 / denom_para;

    % Perpendicular correction coefficient (c_⊥)
    % c_perp = 1 / D_perp,  D_perp = 1 - 9/8 u + 1/2 u^3 - 57/100 u^4
    %                                + 1/5 u^5 + 7/200 u^11 - 1/25 u^12
    denom_perp = 1 - (9/8)*u + (1/2)*u3 - (57/100)*u4 + (1/5)*u5 ...
                 + (7/200)*u11 - (1/25)*u12;
    c_perp = 1 / denom_perp;

    if ~need_derivs
        return;
    end

    % --- Analytical derivatives -------------------------------------------
    % Recall d(u^n)/dh_bar = -n * u^(n+1)  (since u = 1/h_bar)
    %
    % D_para'  = (9/16) u^2 - (3/8) u^4 + (45/64) u^5 + (5/16) u^6
    % D_para'' = -(9/8) u^3 + (3/2) u^5 - (225/64) u^6 - (15/8) u^7
    %
    % D_perp'  = (9/8) u^2 - (3/2) u^4 + (57/25) u^5 - u^6
    %            - (77/200) u^12 + (12/25) u^13
    % D_perp'' = -(9/4) u^3 + 6 u^5 - (57/5) u^6 + 6 u^7
    %            + (231/50) u^13 - (156/25) u^14

    D_para_p  = (9/16)*u2 - (3/8)*u4 + (45/64)*u5 + (5/16)*u6;
    D_para_pp = -(9/8)*u3 + (3/2)*u5 - (225/64)*u6 - (15/8)*u7;

    D_perp_p  = (9/8)*u2 - (3/2)*u4 + (57/25)*u5 - u6 ...
                - (77/200)*u12 + (12/25)*u13;
    D_perp_pp = -(9/4)*u3 + 6*u5 - (57/5)*u6 + 6*u7 ...
                + (231/50)*u13 - (156/25)*u14;

    % For c = 1/D:
    %   dc/dh_bar = -D' / D^2
    %   K_h      = (1/c) dc/dh_bar = -D' / D
    %   K_h'     = -D'' / D + (D'/D)^2 = -D''/D + K_h^2
    dc_para_dh     = -D_para_p / (denom_para^2);
    dc_perp_dh     = -D_perp_p / (denom_perp^2);

    K_h_para       = -D_para_p / denom_para;
    K_h_perp       = -D_perp_p / denom_perp;

    K_h_prime_para = -D_para_pp / denom_para + K_h_para^2;
    K_h_prime_perp = -D_perp_pp / denom_perp + K_h_perp^2;

    derivs = struct( ...
        'dc_para_dh',     dc_para_dh, ...
        'dc_perp_dh',     dc_perp_dh, ...
        'K_h_para',       K_h_para, ...
        'K_h_perp',       K_h_perp, ...
        'K_h_prime_para', K_h_prime_para, ...
        'K_h_prime_perp', K_h_prime_perp);

end
