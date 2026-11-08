function plot_1106_meng_vs_acc(seed, ropts, tag)
% PURPOSE: figures of run_1106_meng_law_1d (constant-velocity descent 22.2 -> 1.10 R in 1 s + 0.2 s hold, plane wall, seed 1), style copied
%   from plot_mc_apdiff.m (1019 report): (1) columns current law | Meng law, rows z_d / z [um], a-bar_z, a-bar'_z (log, negatives not
%   drawn), delta z_m [um]; (2) one figure, both laws overlaid: e_a = a-bar_z - a-hat_z and e_a' = a-bar'_z - a-hat'_z.
% EXPIRES: with run_1106_meng_law_1d.m.
    if nargin < 1; seed = 1; end
    if nargin < 2; ropts = struct(); end                  % extra run_1106_meng_law_1d options (e.g. q55_hold0)
    if nargin < 3; tag = ''; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_1106_meng');  if ~exist(od, 'dir'); mkdir(od); end
    o = struct('seeds', seed);  fn = fieldnames(ropts);  for i = 1:numel(fn); o.(fn{i}) = ropts.(fn{i}); end
    R = run_1106_meng_law_1d(o);
    pc = physical_constants();  R_UM = pc.R;
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  cM = [0.55 0.75 1.0];  cA = [0.35 0.35 0.35];  FS = 22;  LFS = 17;  AXLW = 2.2;
    AB = char(257);  HAT = char(770);  az = [AB '_z'];
    LAW = {'acc', 'meng'};  NM = {'current law', 'Meng law'};
    % ---- figure 1: two columns ----
    f = figure('Position', [40 40 1600 2000], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(4, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ic = 1:2
        r = R.(LAW{ic})(1);  m = r.a > 0;  t = r.t(m);  ax = gobjects(4, 1);
        ax(1) = nexttile(tl, ic);  hold on; box on;
        hd = plot(ax(1), t, r.xd(m) * R_UM, '-', 'Color', cT, 'LineWidth', 5.0, 'DisplayName', 'z_d');
        hz = plot(ax(1), t, r.x(m) * R_UM, '--', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', 'z');
        ylim(ax(1), [0 55]);
        lg = legend(ax(1), [hd hz], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        lg.Title.String = sprintf('plane wall, %s, seed %d', NM{ic}, seed);
        ax(2) = nexttile(tl, 2 + ic);  hold on; box on;
        ht = plot(ax(2), t, r.a(m), '-', 'Color', cT, 'LineWidth', 5.0, 'DisplayName', az);
        hh = plot(ax(2), t, r.ah(m), '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', [AB HAT '_z']);
        ylim(ax(2), [-0.2 1.6]);
        legend(ax(2), [ht hh], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        ax(3) = nexttile(tl, 4 + ic);  hold on; box on;
        aph = r.aph(m);  aph(aph <= 0) = NaN;
        ht3 = plot(ax(3), t, r.ap(m), '-', 'Color', cT, 'LineWidth', 5.0, 'DisplayName', [AB '''_z']);
        hh3 = plot(ax(3), t, aph, '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', [AB HAT '''_z']);
        set(ax(3), 'YScale', 'log');  ylim(ax(3), [1e-4 3]);  yticks(ax(3), [1e-4 1e-2 1]);
        legend(ax(3), [ht3 hh3], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        ax(4) = nexttile(tl, 6 + ic);  hold on; box on;
        plot(ax(4), t, r.dxm(m) * R_UM, '-', 'Color', cM, 'LineWidth', 1.0);
        ylim(ax(4), [-0.2 0.2]);  yticks(ax(4), [-0.1 0 0.1]);
        xlabel(ax(4), 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        for q = 1:4; xlim(ax(q), [0 t(end)]);  set(ax(q), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(ax(q), 'off'); end
        if ic == 1
            ylabel(ax(1), ['z  [' char(181) 'm]'], 'FontSize', FS, 'FontWeight', 'bold');
            ylabel(ax(2), az, 'FontSize', FS + 2, 'FontWeight', 'bold');
            ylabel(ax(3), [AB '''_z'], 'FontSize', FS + 2, 'FontWeight', 'bold');
            ylabel(ax(4), [char(948) 'z_m  [' char(181) 'm]'], 'FontSize', FS, 'FontWeight', 'bold');
        end
        fprintf('%-12s seed %d | negative a''-hat samples %d of %d\n', NM{ic}, seed, sum(r.aph(m) <= 0), sum(m));
    end
    out = fullfile(od, sprintf('fig_1106_meng_vs_acc%s_s%d.png', tag, seed));  exportgraphics(f, out, 'Resolution', 150);  close(f);  fprintf('figure: %s\n', out);
    % ---- figure 2: errors, both laws overlaid ----
    g = figure('Position', [40 40 1600 1100], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tg = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');  bx = gobjects(2, 1);  C = {cA, cE};  W = [2.4 1.8];
    bx(1) = nexttile(tg);  hold on; box on;  bx(2) = nexttile(tg);  hold on; box on;  hl = gobjects(2, 1);
    for ic = 1:2
        r = R.(LAW{ic})(1);  m = r.a > 0;  t = r.t(m);
        hl(ic) = plot(bx(1), t, r.a(m) - r.ah(m), '-', 'Color', C{ic}, 'LineWidth', W(ic), 'DisplayName', NM{ic});
        plot(bx(2), t, r.ap(m) - r.aph(m), '-', 'Color', C{ic}, 'LineWidth', W(ic));
    end
    for q = 1:2; yline(bx(q), 0, '-', 'Color', [0.6 0.6 0.6], 'HandleVisibility', 'off'); end
    lg = legend(bx(1), hl, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
    lg.Title.String = sprintf('plane wall, seed %d', seed);
    ylabel(bx(1), ['e_{' AB '_z}'], 'FontSize', FS + 2, 'FontWeight', 'bold');
    ylabel(bx(2), ['e_{' AB '''_z}'], 'FontSize', FS + 2, 'FontWeight', 'bold');
    xlabel(bx(2), 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    for q = 1:2; xlim(bx(q), [0 t(end)]);  set(bx(q), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(bx(q), 'off'); end
    out = fullfile(od, sprintf('fig_1106_err_a_ap%s_s%d.png', tag, seed));  exportgraphics(g, out, 'Resolution', 150);  close(g);  fprintf('figure: %s\n', out);
end
