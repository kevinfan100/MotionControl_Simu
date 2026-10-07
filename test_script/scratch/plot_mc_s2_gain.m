% STATUS: SCRATCH -- 2026-10-07, motion_ctrl Section 2 figure (style under review with the user).
%   Columns: control law with the nominal motion gain | with the true motion gain. Plane wall, lambda_c 0.4, seed 1.
%   Rows: (1) trajectory z and z_d [um], wall at 0 (as the paper's Fig. 4)
%         (2) [optional] a-bar_z true and the motion gain used in the control law
%         (3) measured tracking error delta z_m [um]: raw (light) and its deterministic component from the
%             single-pole IIR filter (9) of the paper (coefficient 0.05), as the paper's Fig. 5 / Fig. 6 right.
%   Style history (user 10-07): v1 fewer and thicker lines; v2 no legend title, true gain thicker than the gain in the law,
%   z labels, larger fonts; v3 trajectory row and IIR deterministic component; v4 no a-bar row (the arm goes into the
%   trajectory legend entry), ramp only; v5 legend z / z_d only, delta z_m raw only, pure ramp (no holds) T_d 5 s.
%   plot_mc_s2_gain(ver, trj): ver 'v3' (with a-bar row) | 'v4' (without); trj '_ramp2' | '_rtri_f1'.
function plot_mc_s2_gain(ver, trj)
    if nargin < 1; ver = 'v4'; end
    if nargin < 2; trj = '_ramp2'; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    id = fullfile(root, 'test_results', 'temp_plaw_walls');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    TAG = {'_s1_ctrlnom', '_s1_ctrltrue'};  NM = {'nominal motion gain', 'true motion gain'};
    az = [char(257) '_z'];  CTL = {[az '_{,ctrl} = 1'], [az '_{,ctrl} = ' az]};
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  cRaw = [0.62 0.75 0.95];
    FS = 22;  LFS = 17;  AXLW = 2.2;  LWt = 4.0;  LWa = 2.4;  LWm = 2.8;  LWr = 0.9;
    Rum = 2.25;  A_VAR = 0.05;                     % probe radius [um]; IIR coefficient of the paper's (9)
    withA = ~any(strcmp(ver, {'v4', 'v5'}));  nr = 2 + withA;
    v5 = strcmp(ver, 'v5');   % v5 (user 10-07): legend z / z_d only (the figure caption names the arm), delta z_m raw only (no mean)
    f = figure('Position', [40 40 1600 450 * nr], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(nr, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ic = 1:2
        S = load(fullfile(id, sprintf('f1p2_plane_PI%s%s_lc040.mat', TAG{ic}, trj)));  t = S.t(:);
        AX = gobjects(nr, 1);  for r = 1:nr; AX(r) = nexttile(tl, (r - 1) * 2 + ic); hold(AX(r), 'on'); box(AX(r), 'on'); end
        % row 1: trajectory
        zd = Rum * S.wd(:);  zd(S.wd(:) < 1) = NaN;  z = Rum * S.h_true(:, 1);
        zl = 'z';  if ~withA && ~v5; zl = ['z,  ' CTL{ic}]; end
        g1 = plot(AX(1), t, z, '-', 'Color', cT, 'LineWidth', LWt, 'DisplayName', zl);
        g2 = plot(AX(1), t, zd, '--', 'Color', cE, 'LineWidth', LWa, 'DisplayName', 'z_d');
        ylim(AX(1), [0, 1.1 * max(z)]);
        legend(AX(1), [g1 g2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        % optional row: motion gain
        if withA
            if ic == 1; ag = ones(size(t)); else; ag = S.a_true(:, 1); end
            h1 = plot(AX(2), t, S.a_true(:, 1), '-', 'Color', cT, 'LineWidth', LWt, 'DisplayName', az);
            h2 = plot(AX(2), t, ag, '--', 'Color', cE, 'LineWidth', LWa, 'DisplayName', CTL{ic});
            ylim(AX(2), [-0.2 1.6]);
            legend(AX(2), [h1 h2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        end
        % last row: measured tracking error
        x = S.dxm(:, 1);  xm = filter(A_VAR, [1, -(1 - A_VAR)], x);
        ae = AX(end);
        if v5; plot(ae, t, x, '-', 'Color', cE, 'LineWidth', 1.0); else; plot(ae, t, x, '-', 'Color', cRaw, 'LineWidth', LWr); end
        if ~v5; plot(ae, t, xm, '-', 'Color', cE, 'LineWidth', LWm); end
        plot(ae, [t(1) t(end)], [0 0], ':', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.2);
        ylim(ae, [-0.25 0.25]);
        for r = 1:nr; xlim(AX(r), [0 t(end)]); set(AX(r), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW); grid(AX(r), 'off'); end
        xlabel(ae, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        if ic == 1
            ylabel(AX(1), ['z  [' char(181) 'm]'], 'FontSize', FS, 'FontWeight', 'bold');
            if withA; ylabel(AX(2), az, 'FontSize', FS + 2, 'FontWeight', 'bold'); end
            ylabel(ae, [char(948) 'z_m  [' char(181) 'm]'], 'FontSize', FS, 'FontWeight', 'bold');
        end
        fprintf('%-20s | deterministic component: min %+.4f um, max %+.4f um | raw rms %.4f um\n', NM{ic}, min(xm), max(xm), sqrt(mean(x.^2)));
    end
    out = fullfile(fd, sprintf('fig_mc_s2_gain%s_%s.png', trj, ver));  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end
