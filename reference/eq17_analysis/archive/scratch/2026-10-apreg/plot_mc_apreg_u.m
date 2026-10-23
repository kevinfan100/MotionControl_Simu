% STATUS: SCRATCH -- 2026-10-18 (user): a' state regulation, a law + speed cap (tau 0.1) | u law tau 0.5 | u law tau 1.0 (no cap).
%   One figure per case: plane a_d 0.07 (seed 1), cell a_d 0.60 (seed 2: seed 1 touches the wall in the u law tau 1.0 arm).
%   Rows: z_d / z / z* [um] (log); a-bar_z true / estimate / a_d; a-bar' estimate (blue) and true (red), log; delta z_m [um].
%   Output test_results/temp_1009_ws/fig_mc_apreg_u_<wall>_s<seed>.png.
function plot_mc_apreg_u()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(genpath(fullfile(root, 'model')));
    od = fullfile(root, 'test_results', 'temp_1009_ws');  pc = physical_constants();  R_UM = pc.R;
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  FS = 20;  LFS = 15;  AXLW = 2.2;  AB = char(257);  HAT = char(770);  az = [AB '_z'];
    ARMS = {'state', [AB ' law + speed cap, \tau = 0.1 s']; 'stateu050', 'u law, \tau = 0.5 s'; 'stateu100', 'u law, \tau = 1 s'};
    CASES = {'plane', '', 1; 'cell', '_ad060', 2};
    for ic0 = 1:2
        w = CASES{ic0, 1};  seed = CASES{ic0, 3};
        f = figure('Position', [40 40 2300 2000], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
        tl = tiledlayout(4, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
        for ic = 1:3
            S = load(fullfile(od, sprintf('apreg_%s_%s%s.mat', ARMS{ic, 1}, w, CASES{ic0, 2})));  q = find(S.seeds == seed);
            hs = fzero(@(x) local_a(x, w) - S.a_d, [1.001 30]);
            t = S.t;  hd = S.wd(:, q);  hd(1) = hd(2);  h = S.h_true(:, q);  ax = gobjects(4, 1);
            ax(1) = nexttile(tl, ic);  hold on; box on;
            h1 = plot(ax(1), t, hd * R_UM, '-', 'Color', cT, 'LineWidth', 5.0, 'DisplayName', 'z_d');
            h2 = plot(ax(1), t, h * R_UM, '--', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', 'z');
            h3 = yline(ax(1), hs * R_UM, '--', 'Color', 'k', 'LineWidth', 1.5, 'DisplayName', 'z^*');
            set(ax(1), 'YScale', 'log');  ylim(ax(1), [2 60]);  yticks(ax(1), [2 3 5 10 20 50]);
            lg = legend(ax(1), [h1 h2 h3], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
            lg.Title.String = sprintf('%s wall, %s_d = %.2f, seed %d: %s', w, AB, S.a_d, seed, ARMS{ic, 2});
            ax(2) = nexttile(tl, 3 + ic);  hold on; box on;
            ht = plot(ax(2), t, S.a_true(:, q), '-', 'Color', cT, 'LineWidth', 5.0, 'DisplayName', az);
            hh = plot(ax(2), t, S.a_hat(:, q), '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', [AB HAT '_z']);
            hdd = yline(ax(2), S.a_d, '--', 'Color', 'k', 'LineWidth', 1.5, 'DisplayName', [AB '_d']);
            ylim(ax(2), [-0.2 1.6]);
            legend(ax(2), [ht hh hdd], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
            ax(3) = nexttile(tl, 6 + ic);  hold on; box on;
            apt = local_ap(max(h, 1.001), w);  aph = S.ap_hat(:, q);  aph(aph <= 0) = NaN;
            ht3 = plot(ax(3), t, apt, '-', 'Color', cT, 'LineWidth', 5.0, 'DisplayName', [AB '''_z']);
            hh3 = plot(ax(3), t, aph, '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', [AB HAT '''_z']);
            set(ax(3), 'YScale', 'log');  ylim(ax(3), [1e-3 3]);  yticks(ax(3), [1e-3 1e-2 1e-1 1]);
            legend(ax(3), [ht3 hh3], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
            ax(4) = nexttile(tl, 9 + ic);  hold on; box on;
            plot(ax(4), t, S.dxm(:, q), '-', 'Color', cE, 'LineWidth', 1.0);
            ylim(ax(4), [-0.2 0.2]);  yticks(ax(4), [-0.1 0 0.1]);  xlabel(ax(4), 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
            for k = 1:4; xlim(ax(k), [0 t(end)]);  set(ax(k), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(ax(k), 'off'); end
            if ic == 1
                ylabel(ax(1), ['z  [' char(181) 'm]'], 'FontSize', FS, 'FontWeight', 'bold');  ylabel(ax(2), az, 'FontSize', FS + 2, 'FontWeight', 'bold');
                ylabel(ax(3), [AB '''_z'], 'FontSize', FS + 2, 'FontWeight', 'bold');  ylabel(ax(4), [char(948) 'z_m  [' char(181) 'm]'], 'FontSize', FS, 'FontWeight', 'bold');
            end
        end
        out = fullfile(od, sprintf('fig_mc_apreg_u_%s_s%d.png', w, seed));
        exportgraphics(f, out, 'Resolution', 150);  close(f);  fprintf('figure: %s\n', out);
    end
end

function ap = local_ap(h, wall)
    d = 1e-4;  ap = (local_a(h + d, wall) - local_a(h - d, wall)) / (2 * d);
end

function a = local_a(h, wall)
    if strcmp(wall, 'cell'); a = 1 ./ cell_cperp_delta(h); return; end
    a = zeros(size(h));  for i = 1:numel(h); [~, c] = calc_correction_functions(h(i)); a(i) = 1 / c; end
end
