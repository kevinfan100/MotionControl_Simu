% STATUS: SCRATCH -- 2026-10-11, 1009_conly_wall_position.tex, descent only (user 10-11). Two questions, no new run:
%   (1) which constant pairs (c, w_s) reproduce the wall's slope a'(h) = (1-c)/(h - w_s)^2 over the walked band: slope ratio
%       pair / true at h = 1.10 ... 10 R for the estimate at the end of the descent, the red pair at the bottom (c(1.10), 0), the best
%       constant c with w_s = 0 (run_plaw_walls f1p2 band fit), and the F2 band fit (run_plaw_walls f2 CFIT / DFIT, w_s = -delta);
%   (2) when the walk happens: fraction of the c column (1/h - 1/h0) and of the w_s column (1/h0^2 - 1/h^2) walked by each height
%       (leading order, 1009 v1 Section 3), next to the run's sqrt(P_w) and estimates. Data: run_1009_ws('desc') arm 'ws'.
function probe_1009_ws_slope_valley()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));  addpath(here);
    id = fullfile(root, 'test_results', 'temp_1009_ws');
    H = [1.10 1.5 2 3 5 10];  d = 1e-4;
    apC = arrayfun(@(h) (1 / cell_cperp_delta(h + d) - 1 / cell_cperp_delta(h - d)) / (2 * d), H);
    apP = zeros(size(H));
    for i = 1:numel(H); [~, a] = calc_correction_functions(H(i) + d); [~, b] = calc_correction_functions(H(i) - d); apP(i) = (1 / a - 1 / b) / (2 * d); end
    cC = 1 - H.^2 .* apC;  cP = 1 - H.^2 .* apP;
    fprintf('h                 %s\n', mat2str(H, 3));
    fprintf('cell  true a''     %s | c(h) at w_s = 0 %s\n', mat2str(apC, 3), mat2str(cC, 3));
    fprintf('plane true a''     %s | c(h) at w_s = 0 %s\n', mat2str(apP, 3), mat2str(cP, 3));
    r = @(c, w, ap) (1 - c) ./ (H - w).^2 ./ ap;
    E = load(fullfile(id, 'desc_ws_cell.mat'));  Pp = load(fullfile(id, 'desc_ws_plane.mat'));
    fprintf('\nslope ratio pair / true, cell (h = %s):\n', mat2str(H, 3));
    fprintf('  estimate end   (%+.3f, %+.3f) %s\n', E.c_hat(end), E.w_hat(end), mat2str(r(E.c_hat(end), E.w_hat(end), apC), 3));
    fprintf('  red at 1.10 R  (%+.3f, %+.3f) %s\n', cC(1), 0, mat2str(r(cC(1), 0, apC), 3));
    fprintf('  best c, w_s=0  (%+.3f, %+.3f) %s\n', 0.6405, 0, mat2str(r(0.6405, 0, apC), 3));
    fprintf('  F2 band fit    (%+.3f, %+.3f) %s\n', -0.163, -1.238, mat2str(r(-0.163, -1.238, apC), 3));
    fprintf('slope ratio pair / true, plane:\n');
    fprintf('  estimate end   (%+.3f, %+.3f) %s\n', Pp.c_hat(end), Pp.w_hat(end), mat2str(r(Pp.c_hat(end), Pp.w_hat(end), apP), 3));
    fprintf('  red at 1.10 R  (%+.3f, %+.3f) %s\n', cP(1), 0, mat2str(r(cP(1), 0, apP), 3));
    fprintf('  best c, w_s=0  (%+.3f, %+.3f) %s\n', 0.0533, 0, mat2str(r(0.0533, 0, apP), 3));
    fprintf('  F2 band fit    (%+.3f, %+.3f) %s\n', -0.085, -0.104, mat2str(r(-0.085, -0.104, apP), 3));
    fprintf('\nwalked fraction along the descent (leading order) and the run:\n');
    fprintf('  h_d    t      1/h col  1/h^2 col | sqrtPw cell  plane | w_hat cell  plane | c_hat cell  plane\n');
    h0 = 50 / 2.25;  hb = 1.10;  t = E.t;  wd = E.wd;
    for hv = [15 10 6 4 3 2 1.5 1.2 1.10]
        i = find(wd <= hv + 1e-9 & t > 0.001, 1);
        f1 = (1 / hv - 1 / h0) / (1 / hb - 1 / h0);  f2 = (1 / h0^2 - 1 / hv^2) / (1 / h0^2 - 1 / hb^2);
        fprintf('  %5.2f  %.3f  %.3f    %.3f     | %.3f        %.3f | %+.3f     %+.3f | %+.3f     %+.3f\n', hv, t(i), f1, f2, ...
                E.sP_w(i), Pp.sP_w(i), E.w_hat(i), Pp.w_hat(i), E.c_hat(i), Pp.c_hat(i));
    end
end
