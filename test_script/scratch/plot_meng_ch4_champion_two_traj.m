% FORK OF test/motion-test test_script/scratch/plot_btrue_two_traj_final.m @ 8aef954 | PURPOSE: the mainline four-row
%   two-trajectory page (seed 7 absolute / 10-seed mean +- sigma / gain error band + seed 7 / same-instant tracking error)
%   for the Meng Ch4 champion runs of run_meng_ch4_champion_two_traj.m; left Meng, right canon; thin gray lines = phase
%   boundaries (hold | descent | osc | hold) | EXPIRES: with run_meng_ch4_champion_two_traj.m | 產線改動不會自動跟上
function plot_meng_ch4_champion_two_traj()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'meng_ch4_s0');
    D = {load(fullfile(od, 'champ_two_traj_meng.mat')), load(fullfile(od, 'champ_two_traj_canon.mat'))};
    SEED = 7;  COL_TRUE = [0.8 0 0];  COL_HAT = [0 0.2 0.9];  BANDC = [0.45 0.55 0.95];  COL_SEED = [0.55 0.74 0.96];
    FS = 18;  LFS = 12;  AXLW = 1.8;  NR = 4;
    f = figure('Units', 'inches', 'Position', [0 0 15 4.2*NR], 'Color', 'w', 'Visible', 'off');
    tiledlayout(NR, 2, 'TileSpacing', 'compact', 'Padding', 'compact');  ax = gobjects(NR, 2);
    for a = 1:2
        d = D{a};  t = d.t(:);  T_END = ceil(t(end));  iS = find(d.seeds == SEED, 1);  if isempty(iS); iS = 1; end
        nS = size(d.E, 2);  TR = (d.hd(:) - d.HB) * d.R;  ph = d.phases(1:3);
        ax(1, a) = nexttile(a); hold on;
        ht = plot(t, d.AT(:, iS), '-', 'Color', COL_TRUE, 'LineWidth', 2.0);
        hh = plot(t, d.AH(:, iS), '-', 'Color', COL_HAT, 'LineWidth', 1.2);
        legend([hh ht], {sprintf('\\^a_z / a_{nom}   seed %d', d.seeds(iS)), 'a_z / a_{nom}  true'}, ...
               'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        local_style(a == 1, 'a_z / a_{nom}', false, T_END, ph, FS, AXLW);
        ax(2, a) = nexttile(2 + a); hold on;
        m = mean(d.AH, 2);  s = std(d.AH, 0, 2);
        hb = fill([t; flipud(t)], [m + s; flipud(m - s)], BANDC, 'FaceAlpha', 0.30, 'EdgeColor', 'none');
        ht = plot(t, mean(d.AT, 2), '-', 'Color', COL_TRUE, 'LineWidth', 2.0);
        hh = plot(t, m, '-', 'Color', COL_HAT, 'LineWidth', 1.5);
        legend([hh ht hb], {sprintf('mean \\^a_z / a_{nom}  (%d seeds)', nS), 'a_z / a_{nom}  true', '\pm \sigma_{seed}'}, ...
               'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        local_style(a == 1, 'a_z / a_{nom}', false, T_END, ph, FS, AXLW);
        ax(3, a) = nexttile(4 + a); hold on;
        mE = mean(d.E, 2);  sE = std(d.E, 0, 2);
        hb = fill([t; flipud(t)], [mE + sE; flipud(mE - sE)], BANDC, 'FaceAlpha', 0.30, 'EdgeColor', 'none');
        yline(0, '-', 'Color', [0.5 0.5 0.5], 'LineWidth', 0.8, 'HandleVisibility', 'off');
        h1 = plot(t, d.E(:, iS), '-', 'Color', COL_SEED, 'LineWidth', 0.6);
        hm = plot(t, mE, '-', 'Color', COL_HAT, 'LineWidth', 2.0);
        legend([hm hb h1], {'seed mean (\^a_z - a_z)/a_{nom}', '\pm \sigma_{seed}', sprintf('seed %d', d.seeds(iS))}, ...
               'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        local_style(a == 1, '(\^a_z - a_z) / a_{nom}', false, T_END, ph, FS, AXLW);
        ax(4, a) = nexttile(6 + a); hold on;
        mT = mean(TR, 2);  sT = std(TR, 0, 2);
        hb = fill([t; flipud(t)], [mT + sT; flipud(mT - sT)], BANDC, 'FaceAlpha', 0.30, 'EdgeColor', 'none');
        yline(0, '-', 'Color', [0.5 0.5 0.5], 'LineWidth', 0.8, 'HandleVisibility', 'off');
        h1 = plot(t, TR(:, iS), '-', 'Color', COL_SEED, 'LineWidth', 0.6);
        hm = plot(t, mT, '-', 'Color', COL_HAT, 'LineWidth', 2.0);
        legend([hm hb h1], {'seed mean  R\deltaw_3', '\pm \sigma_{seed}', sprintf('seed %d', d.seeds(iS))}, ...
               'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        local_style(a == 1, 'R\deltaw_3  [\mum]', true, T_END, ph, FS, AXLW);
    end
    for r = 1:NR; linkaxes(ax(r, :), 'y'); end
    png = fullfile(od, 'champ_two_traj.png');
    exportgraphics(f, png, 'Resolution', 150);  close(f);  fprintf('saved %s\n', png);
end

function local_style(is_left, ylab, is_bottom, T_END, ph, FS, AXLW)
    for x = ph; xline(x, ':', 'Color', [0.6 0.6 0.6], 'LineWidth', 1.0, 'HandleVisibility', 'off'); end
    if is_left; ylabel(ylab, 'FontSize', FS, 'FontWeight', 'bold'); end
    if is_bottom; xlabel('time  [s]', 'FontSize', FS, 'FontWeight', 'bold'); end
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');  grid off;  xlim([0 T_END]);
end
