function cp = cell_cperp_delta(h_bar)
%CELL_CPERP_DELTA  Test cell plant = the plane polynomial slid down by DEL = 1.05 R (ledger C62). Zero fit.
%   a_bar(h) = D_perp(1 / (h + DEL)),   h = w - w_s = centre height above the VISIBLE cell surface, in R.
%   D_perp is the production plane polynomial (calc_correction_functions.m:81-82, c_perp = 1/D_perp). The slide keeps
%   every plane boundary property (simple zero with slope 1, now at h = 1 - DEL; far tail 9/8 / h; monotone, concave)
%   and matches the Meng Fig 11 cell data to rms 0.006 on 1.03-4.5 R (probe_plane_offset_law.m).
%   2026-09-20 (user: the cell is a test object only): replaces cell_cperp_fig11 as the DEFAULT cell of the two-wall,
%   fixed-height, speed-scan and [2/2] runners. cell_cperp_fig11 (12th-order data fit; non-monotone hook below 1.04 R,
%   plot_cell_plant_delta.m) stays as the data-faithful cell for a model-error arm, selected with opts.plant_cperp_cell.
%   Same clamp convention as the other plants: h_bar below the visible-surface contact (1.0) is held at 1.0. Vectorised.
%   FORK OF nothing | EXPIRES: with the delta-law line | 產線改動不會自動跟上
    DEL = 1.05;
    u = 1 ./ (max(h_bar, 1.0) + DEL);
    a = 1 - (9/8) * u + (1/2) * u.^3 - (57/100) * u.^4 + (1/5) * u.^5 + (7/200) * u.^11 - (1/25) * u.^12;
    cp = 1 ./ a;
end
