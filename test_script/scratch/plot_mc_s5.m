% STATUS: SCRATCH -- 2026-10-07, motion_ctrl Sections 5-6 figures, two columns on the same trajectory and seed, lambda_c 0.4.
%   which = 'desc'      : Section 5, first order (Section 4, 5state_noap production) | c-only q55 = 0, plane, Section 2 trajectory
%   which = 'tri'       : Section 5, same on 1 s descent + triangle 1.10 <-> 10 R at 2 Hz
%   which = 's6_<wall>' : Section 6, c-only q55 = 0 | q55 = P_fix^2 I (q55_adapt 'ratio', sqrt(P_fix) 0.15), wall = plane | cell,
%                         triangle trajectory; the estimator starts from the plane values on every wall.
%   Rows: (1) tracking z_d (solid) and z (dashed) [um]; (2) a-bar_z true (red, thick) and estimate (blue); (3) delta z_m [um].
%   Section 6 adds (4) c-hat (blue) and the wall's c(h_d) = 1 - h_d^2 a'_true(h_d) (red, run_plaw_walls c_eff_d) (user 10-07); Section 5 has no c-hat row. only the tracking row is dashed (z_d); red drawn first, red 5.0 / blue 1.8.
%   Data: run_mc_s4_first_order.m ('prod_xyz' / 'prod_tri10') and run_mc_s5_conly.m (temp_plaw_walls, tags _mcs5_*).
function plot_mc_s5(seed, which)
    if nargin < 1; seed = 1; end
    if nargin < 2; which = 'tri'; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);
    fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    id = fullfile(root, 'test_results', 'temp_plaw_walls');
    TRI = 'rtri_f2_top1000_lc040.mat';
    switch which
        case 'desc'; C = {local_fo('prod_xyz', seed), local_co(id, 'f1p2_plane_PI_mcs5_desc_ramp5_lc040.mat', seed)};  NM = {'first order', 'c-only'};
        case 'tri';  C = {local_fo('prod_tri10', seed), local_co(id, ['f1p2_plane_PI_mcs5_tri_' TRI], seed)};          NM = {'first order', 'c-only'};
        otherwise
            wall = which(4:end);
            C = {local_co(id, sprintf('f1p2_%s_PI_mcs5_tri_%s', wall, TRI), seed), local_co(id, sprintf('f1p2_%s_PI_mcs5_tri_ratio_%s', wall, TRI), seed)};
            NM = {[wall ' q55 = 0'], [wall ' P_fix']};
    end
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  FS = 22;  LFS = 17;  AXLW = 2.2;  R_UM = 2.25;
    AB = char(257);  HAT = char(770);  az = [AB '_z'];
    f = figure('Position', [40 40 1600 500 * (3 + startsWith(which, 's6_'))], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    is6 = startsWith(which, 's6_');  nR = 3 + is6;                      % Section 6: c-hat row (user 10-07)
    tl = tiledlayout(nR, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ic = 1:2
        r = C{ic};  t = r.t;  ax = gobjects(nR, 1);
        ax(1) = nexttile(tl, ic);  hold on; box on;
        hz = plot(ax(1), t, r.h * R_UM, '-', 'Color', cT, 'LineWidth', 5.0, 'DisplayName', 'z');      % red first (user 10-07)
        hd = plot(ax(1), t, r.h_d * R_UM, '--', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', 'z_d');  % only tracking rows dashed
        ylim(ax(1), [0 55]);
        legend(ax(1), [hz hd], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        ax(2) = nexttile(tl, 2 + ic);  hold on; box on;
        ht = plot(ax(2), t, r.a_true, '-', 'Color', cT, 'LineWidth', 5.0, 'DisplayName', az);
        ls = '-';                                                       % comparisons solid; only tracking rows dashed (user 10-07)
        hh = plot(ax(2), t, r.a_hat, ls, 'Color', cE, 'LineWidth', 1.8, 'DisplayName', [AB HAT '_z']);
        ylim(ax(2), [-0.2 1.6]);
        legend(ax(2), [ht hh], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        if is6
            ax(4) = nexttile(tl, 4 + ic);  hold on; box on;
            ht4 = plot(ax(4), t, r.c_true, '-', 'Color', cT, 'LineWidth', 5.0, 'DisplayName', 'c');   % c(h_d) = 1 - h_d^2 a'(h_d) of the wall
            hc = plot(ax(4), t, r.c_hat, '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', ['c' HAT]);
            ylim(ax(4), [-0.4 1.2]);
            legend(ax(4), [ht4 hc], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
            if ic == 1; ylabel(ax(4), 'c', 'FontSize', FS + 2, 'FontWeight', 'bold'); end
            fprintf('%-14s c_hat at 1 s %+.3f, end %+.3f\n', NM{ic}, interp1(t, r.c_hat, 1), r.c_hat(end));
        end
        ax(3) = nexttile(tl, (nR - 1) * 2 + ic);  hold on; box on;
        plot(ax(3), t, r.dxm, '-', 'Color', cE, 'LineWidth', 1.0);
        ylim(ax(3), [-0.2 0.2]);  yticks(ax(3), [-0.1 0 0.1]);
        xlabel(ax(3), 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        for q = 1:nR; xlim(ax(q), [0 t(end)]);  set(ax(q), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(ax(q), 'off'); end
        if ic == 1
            ylabel(ax(1), ['z  [' char(181) 'm]'], 'FontSize', FS, 'FontWeight', 'bold');
            ylabel(ax(2), az, 'FontSize', FS + 2, 'FontWeight', 'bold');
            ylabel(ax(3), [char(948) 'z_m  [' char(181) 'm]'], 'FontSize', FS, 'FontWeight', 'bold');
        end
        e = r.a_true - r.a_hat;
        if strcmp(which, 'desc')
            fprintf('%-14s seed %d | e_a 1-3 s %+.3f | 4.5-5 s %+.3f | last 0.1 s %+.3f | dz_m rms last 1 s %.4f um\n', NM{ic}, seed, ...
                    mean(e(t > 1 & t < 3)), mean(e(t > 4.5)), mean(e(t > 4.9)), rms(r.dxm(t > 4)));
        else
            tb = [1 1.5 2 2.5];  tt = [1.25 1.75 2.25 2.75];
            eb = arrayfun(@(x) mean(e(abs(t - x) < 0.01)), tb);  et = arrayfun(@(x) mean(e(abs(t - x) < 0.01)), tt);
            fprintf('%-14s seed %d | troughs %s | tops %s | triangle mean|e_a| %.3f | dz_m rms triangle %.4f um\n', NM{ic}, seed, ...
                    mat2str(eb, 2), mat2str(et, 2), mean(abs(e(t > 1))), rms(r.dxm(t > 1)));
        end
    end
    out = fullfile(fd, sprintf('fig_mc_s5_%s_s%d.png', which, seed));
    if startsWith(which, 's6_'); out = fullfile(fd, sprintf('fig_mc_s6_%s_s%d.png', which(4:end), seed)); end
    exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end

function r = local_fo(form, seed)
    R = run_mc_s4_first_order({form}, seed, {'plane'});  r = R.(form).plane(seed);
end

function r = local_co(id, fn, seed)
    S = load(fullfile(id, fn));  wd = S.wd(:);  if wd(1) == 0; wd(1) = wd(2); end
    r = struct('t', S.t(:), 'h_d', wd, 'h', S.h_true(:, seed), 'a_true', S.a_true(:, seed), 'a_hat', S.a_hat(:, seed), 'dxm', S.dxm(:, seed), ...
               'c_hat', S.c_hat(:, seed), 'c_true', S.c_eff_d(:));
    if r.c_true(1) > r.c_true(2) + 0.1; r.c_true(1) = r.c_true(2); end   % first sample uses h_d = 0 (wd(1) = 0 in the log)
end
