% STATUS: SCRATCH -- 2026-10-22 (user: compare 5 and 50 um only; Kalman gain in the paper notation (estimator feedback rows L_8, L_9 of
%   1015_estimator_3d_wsonly Sec. 4), all entries; top panel = the difference of the ellipses). Wait-only fixed points tmg<gap>_s1.
%   Row 1: gain matrix in the x-z plane as an ellipse (semi-axes = gains along the principal directions) / a_T: true A-bar (red), time
%   average of the measured A_m = V-hat vA_m V-hat' (blue), circle = isotropic (grey dotted).
%   Rows 2-4: |L_r,j| of the angle row (r = 9 for theta_y or 8 for theta_x) for the measurement columns: position dp_m x y z [deg/nm],
%   gains a_Tm a_Nm [deg per unit a-bar], angles theta_xm theta_ym [deg/deg]. Log axes, shared between the two gaps.
%   -> test_results/temp_3d_wsonly/fig_t2_L<r>.png | run: plot_t2_L([50 5], 9)
function plot_t2_L(GAP, row)
    if nargin < 2; row = 9; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_3d_wsonly');  addpath(genpath(fullfile(root, 'model')));  pc = physical_constants();  Rnm = pc.R * 1e3;
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  cG = [0 0.55 0.2];  cO = [0.9 0.5 0];  cW = [0.5 0.5 0.5];  FS = 17;  LFS = 13;
    MU = char(181);  DG = char(176);  TH = char(952);  AB = char(257);  sub = {'x', 'y', 'z'};
    nc = numel(GAP);
    f = figure('Position', [40 40 900 * nc 1700], 'Color', 'w', 'Visible', 'off');
    tl = tiledlayout(4, nc, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ig = 1:nc
        S = load(fullfile(od, sprintf('temp_t2_tmg%d_s1.mat', GAP(ig))));  r = S.R.runs{1};  k = 1:r.kend;  t = r.t(k);  ty = S.R.opts.theta_y;
        Lr = r.(sprintf('L%d', row))(k, :);
        % ---- row 1: ellipses ----
        aT = mean(r.aT_true(k));  aN = mean(r.aN_true(k));
        V = [cosd(ty) 0 sind(ty); 0 1 0; -sind(ty) 0 cosd(ty)];  At = V * diag([aT aT aN]) * V';
        Am = zeros(3);  m = t >= 1;  km = k(m);
        for i = km(:)'
            Vh = [cosd(r.thr(i, 2)) 0 sind(r.thr(i, 2)); 0 1 0; -sind(r.thr(i, 2)) 0 cosd(r.thr(i, 2))] * [1 0 0; 0 cosd(r.thr(i, 1)) -sind(r.thr(i, 1)); 0 sind(r.thr(i, 1)) cosd(r.thr(i, 1))];
            Am = Am + Vh * reshape(r.vAm(i, :), 3, 3) * Vh';
        end
        Am = Am / numel(km);
        ax = nexttile(tl, ig);  hold on;  box on;  ph = linspace(0, 2 * pi, 300);
        plot(cos(ph), sin(ph), ':', 'Color', cW, 'LineWidth', 2, 'DisplayName', 'isotropic');
        for q = 1:2
            if q == 1; A2 = At([1 3], [1 3]); c = cT; lw = 3; nm = ['true  ' char(256)]; else; A2 = Am([1 3], [1 3]); c = cE; lw = 2; nm = ['measured  ' char(256) '_m  (time average)']; end
            [Ev, Dv] = eig(A2 / aT);  el = Ev * Dv * [cos(ph); sin(ph)];
            plot(el(1, :), el(2, :), '-', 'Color', c, 'LineWidth', lw, 'DisplayName', nm);
        end
        nw = [sind(ty) cosd(ty)];  plot([-1.15 1.15] * nw(1), [-1.15 1.15] * nw(2), '--', 'Color', cT, 'LineWidth', 1.2, 'DisplayName', 'wall normal');
        axis equal;  xlim([-1.2 1.2]);  ylim([-1.2 1.2]);
        text(0.03, 0.93, sprintf('%d %sm   %s_N/%s_T = %.2f', GAP(ig), MU, AB, AB, aN / aT), 'Units', 'normalized', 'FontSize', FS - 1, 'FontWeight', 'bold');
        xlabel(['x  (gain / ' AB '_T)'], 'FontSize', FS - 2, 'FontWeight', 'bold');  ylabel(['z  (gain / ' AB '_T)'], 'FontSize', FS - 2, 'FontWeight', 'bold');
        legend('Location', 'northoutside', 'NumColumns', 2, 'FontSize', LFS, 'FontWeight', 'bold');
        set(ax, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 2);
        % ---- rows 2-4: L_r entries ----
        conv = {Lr(:, 1:3) * 180 / pi / Rnm, Lr(:, 4:5) * 180 / pi, Lr(:, 6:7)};
        nms = {arrayfun(@(i) ['\delta p_{m' sub{i} '}'], 1:3, 'UniformOutput', false), {[AB '_{Tm}'], [AB '_{Nm}']}, {[TH '_{xm}'], [TH '_{ym}']}};
        yl = {['|L_{' num2str(row) '}|  [' DG '/nm]'], ['|L_{' num2str(row) '}|  [' DG ' per unit ' AB ']'], ['|L_{' num2str(row) '}|  [' DG '/' DG ']']};
        cols = {[cE; cG; cO], [cE; cO], [cG; cE]};  YL = {[1e-6 1e0], [1e-5 1e1], [1e-8 1e-2]};
        for q = 1:3
            ax = nexttile(tl, q * nc + ig);  hold on;  box on;  hh = gobjects(0);
            for j = 1:size(conv{q}, 2)
                hh(end + 1) = plot(t, movmedian(abs(conv{q}(:, j)), 81), '-', 'Color', cols{q}(j, :), 'LineWidth', 1.6, 'DisplayName', nms{q}{j});
            end
            set(ax, 'YScale', 'log');  ylim(YL{q});  xlim([0 t(end)]);
            if ig == 1; ylabel(yl{q}, 'FontSize', FS - 2, 'FontWeight', 'bold'); end
            if q == 3; xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold'); end
            legend(hh, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
            set(ax, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 2);
        end
        mm = t >= 5;
        fprintf('%2d um: a_N/a_T %.3f | measured A_m (avg) xz: [%.3f %.3f; %.3f %.3f] vs true [%.3f %.3f; %.3f %.3f] | median |L_%d| (t>5 s): dp x/y/z %.1e %.1e %.1e deg/nm, aT %.1e aN %.1e deg, thx %.1e thy %.1e\n', ...
                GAP(ig), aN / aT, Am(1, 1), Am(1, 3), Am(3, 1), Am(3, 3), At(1, 1), At(1, 3), At(3, 1), At(3, 3), row, ...
                median(abs(conv{1}(mm, :))), median(abs(conv{2}(mm, :))), median(abs(conv{3}(mm, :))));
    end
    out = fullfile(od, sprintf('fig_t2_L%d.png', row));
    exportgraphics(f, out, 'Resolution', 100);  close(f);  fprintf('figure: %s\n', out);
end
