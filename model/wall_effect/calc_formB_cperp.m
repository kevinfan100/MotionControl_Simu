function c_perp = calc_formB_cperp(h_bar, law)
%CALC_FORMB_CPERP  Perpendicular correction c_perp from a Form B gain law.
%
%   c_perp = calc_formB_cperp(h_bar, law)
%
%   Plant-side override used by the curve-mismatch robustness arm
%   (run_formB_ws opts.plant_gain_law): the TRUE perpendicular mobility is
%   generated from
%       a_bar = 1 - ( b / ((h_bar - ws) + b) )^p ,   c_perp = 1 / a_bar
%   instead of the published plane curve.  NEVER read by any controller --
%   it is dispatched only through params.wall.plant_gain_law, which exists
%   solely on the P_plant copy inside the driver.
%
%   law - struct with fields b, p, ws (Form B parameters of the true curve)
%
%   Guards: gap and a_bar floored at 1e-6 so contact behaves like the plane
%   curve's divergence instead of going negative.

    gap = max(h_bar - law.ws, 1e-6);
    a_bar = 1 - (law.b ./ (gap + law.b)).^law.p;
    a_bar = max(a_bar, 1e-6);
    c_perp = 1 ./ a_bar;
end
