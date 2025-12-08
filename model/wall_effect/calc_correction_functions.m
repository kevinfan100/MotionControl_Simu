function [c_para, c_perp] = calc_correction_functions(h_bar)
%CALC_CORRECTION_FUNCTIONS Calculate wall effect correction coefficients
%
%   [c_para, c_perp] = calc_correction_functions(h_bar)
%
%   Calculates the parallel (c_∥) and perpendicular (c_⊥) correction
%   coefficients for the Stokes drag near a planar wall.
%
%   Inputs:
%       h_bar - Normalized distance from wall: h_bar = h/R
%               where h is the distance from wall to particle center
%               and R is the particle radius
%               MUST be > 1 (particle not touching wall)
%
%   Outputs:
%       c_para - Parallel correction coefficient (c_∥)
%       c_perp - Perpendicular correction coefficient (c_⊥)
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

    % Input validation
    if h_bar <= 1
        error('calc_correction_functions:invalidInput', ...
            'h_bar must be > 1 (particle must not touch wall). Got h_bar = %.4f', h_bar);
    end

    % Compute inverse of h_bar for efficiency
    inv_h = 1 / h_bar;

    % Parallel correction coefficient (c_∥)
    % c_para = 1 / (1 - 9/16*(1/h̄) + 1/8*(1/h̄)^3 - 45/256*(1/h̄)^4 - 1/16*(1/h̄)^5)
    denom_para = 1 - 9/16*inv_h + 1/8*inv_h^3 - 45/256*inv_h^4 - 1/16*inv_h^5;
    c_para = 1 / denom_para;

    % Perpendicular correction coefficient (c_⊥)
    % c_perp = 1 / (1 - 9/8*(1/h̄) + 1/2*(1/h̄)^3 - 57/100*(1/h̄)^4 + 1/5*(1/h̄)^5
    %              + 7/200*(1/h̄)^11 - 1/25*(1/h̄)^12)
    denom_perp = 1 - 9/8*inv_h + 1/2*inv_h^3 - 57/100*inv_h^4 + 1/5*inv_h^5 ...
                 + 7/200*inv_h^11 - 1/25*inv_h^12;
    c_perp = 1 / denom_perp;

end
