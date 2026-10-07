% STATUS: SCRATCH -- 2026-10-07, motion_ctrl Section 4 figures. Estimator = Section 4 form (5state_noap first order, y2 = a_m,
%   all refinements off; run_mc_s4_first_order.m form 'plain'), on the Section 2 trajectory (z 22.2 -> 1.10 R in 5 s, x/y 1 Hz
%   9 um sines), plane wall, lambda_c 0.4.
%   which = 'eps'     : z axis, columns epsilon = Q44/R2 = 0 | 1e-5; rows (1) a-bar_z true (red, thick) and estimate (blue),
%                       (2) delta z_m [um]. No a_m and no e_a row (user 10-07).
%   which = 'nominal' : layout of Fig. 2.2 (plot_mc_s2_xyz.m): columns a-hat = 1 | estimated (epsilon 1e-5); rows delta x_m,
%                       delta y_m, delta z_m [um], y range +-0.2, ticks -0.1 0 0.1.
function plot_mc_s4(seed, which)
    if nargin < 1; seed = 1; end
    if nargin < 2; which = 'eps'; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);
    fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  FS = 22;  LFS = 17;  AXLW = 2.2;
    AB = char(257);  HAT = char(770);  az = [AB '_z'];  AXN = {'x', 'y', 'z'};
    switch which
        case 'eps';     FORMS = {'prod_eps0_xyz', 'prod_xyz'};  NM = {'eps 0', 'eps 1e-5'};
        case 'nominal'; FORMS = {'prod_nominal_xyz', 'prod_xyz'};  NM = {'a-hat = 1', 'estimated'};
        case 'tri';     FORMS = {'prod_tri10'};  NM = {'eps 1e-5'};
    end
    R = run_mc_s4_first_order(FORMS, seed, {'plane'});
    nC = numel(FORMS);  D = cell(1, nC);  for ic = 1:nC; D{ic} = R.(matlab.lang.makeValidName(FORMS{ic})).plane(seed); end
    if ~strcmp(which, 'nominal')
        is_tri = true;  nR = 3;  R_UM = 2.25;                         % user 10-07: tracking row (z_d, z) on top of every gain figure
        f = figure('Position', [40 40 max(800 * nC, 1600) 500 * nR], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
        tl = tiledlayout(nR, nC, 'TileSpacing', 'compact', 'Padding', 'compact');
        for ic = 1:nC
            r = D{ic};  t = r.t;
            if is_tri
                ax0 = nexttile(tl, ic);  hold on; box on;          % tracking: desired (solid) and actual (dashed, thinner) [um]
                hz = plot(ax0, t, r.h * R_UM, '-', 'Color', cT, 'LineWidth', 5.0, 'DisplayName', 'z');      % red first (user 10-07)
                hd = plot(ax0, t, r.h_d * R_UM, '--', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', 'z_d');  % only tracking rows dashed
                xlim(ax0, [0 t(end)]);  ylim(ax0, [0 55]);
                legend(ax0, [hz hd], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
                set(ax0, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(ax0, 'off');
                if ic == 1; ylabel(ax0, ['z  [' char(181) 'm]'], 'FontSize', FS, 'FontWeight', 'bold'); end
            end
            ax1 = nexttile(tl, is_tri * nC + ic);  hold on; box on;
            ht = plot(ax1, t, r.a_true, '-', 'Color', cT, 'LineWidth', 5.0, 'DisplayName', az);
            hh = plot(ax1, t, r.a_hat, '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', [AB HAT '_z']);
            ylim(ax1, [-0.2 1.6]);
            legend(ax1, [ht hh], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
            ax2 = nexttile(tl, (1 + is_tri) * nC + ic);  hold on; box on;
            plot(ax2, t, r.dxm, '-', 'Color', cE, 'LineWidth', 1.0);
            ylim(ax2, [-0.2 0.2]);  yticks(ax2, [-0.1 0 0.1]);
            xlabel(ax2, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
            for a = [ax1 ax2]; xlim(a, [0 t(end)]);  set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(a, 'off'); end
            if ic == 1
                ylabel(ax1, az, 'FontSize', FS + 2, 'FontWeight', 'bold');
                ylabel(ax2, [char(948) 'z_m  [' char(181) 'm]'], 'FontSize', FS, 'FontWeight', 'bold');
            end
            e = r.a_true - r.a_hat;
            fprintf('%-9s seed %d | e_a z mean 1-3 s %+.3f | 4.5-5 s %+.3f | last 0.1 s %+.3f | dz_m rms 0-4 s %.4f, 4.5-5 s %.4f um\n', ...
                    NM{ic}, seed, mean(e(t > 1 & t < 3)), mean(e(t > 4.5)), mean(e(t > 4.9)), rms(r.dxm(t < 4)), rms(r.dxm(t > 4.5)));
        end
    else
        f = figure('Position', [40 40 1600 1200], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
        tl = tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
        for ia = 1:3
            for ic = 1:nC
                r = D{ic};  t = r.t;  x = r.dxm_xyz(:, ia);
                ax = nexttile(tl, (ia - 1) * 2 + ic);  hold on; box on;
                plot(ax, t, x, '-', 'Color', cE, 'LineWidth', 1.0);
                xlim(ax, [0 t(end)]);  ylim(ax, [-0.2 0.2]);  yticks(ax, [-0.1 0 0.1]);
                set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(ax, 'off');
                if ia == 3; xlabel(ax, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold'); end
                if ic == 1; ylabel(ax, [char(948) AXN{ia} '_m  [' char(181) 'm]'], 'FontSize', FS, 'FontWeight', 'bold'); end
                fprintf('%-9s %s: rms all %.4f | rms last 1 s %.4f | max |.| %.3f um\n', NM{ic}, AXN{ia}, rms(x), rms(x(t > 4)), max(abs(x)));
            end
        end
    end
    out = fullfile(fd, sprintf('fig_mc_s4_%s_s%d.png', which, seed));  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end
