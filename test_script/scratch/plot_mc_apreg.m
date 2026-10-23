% STATUS: SCRATCH -- 2026-10-16 (user): motion_ctrl style, gain regulation with a' difference vs a' state (run_mc_apreg), seed 1.
%   One figure per case: plane a_d 0.07 (h* 1.078 R), cell a_d 0.60 (h* 1.654 R). Columns difference | state.
%   Rows: z_d (red) / z (blue dashed) / z* = R h*, the height where a_true = a_d (black dashed) [um]; a-bar_z true (red) / estimate (blue) / a_d (black dashed); a-bar' true at the particle
%   (red) / slope the law uses (blue), log scale; delta z_m [um]. Output test_results/temp_1009_ws/fig_mc_apreg_<wall>_s<seed>.png.
%   10-19 (user): mode 'ws' = columns a' state | w_s state (run_mc_apreg arm 'ws': Section 8 + fe_cmd_eval, same law, gate, step cap);
%   output fig_mc_apws_reg_<wall>_s<seed>.png.
function plot_mc_apreg(seed, mode)
    if nargin < 1; seed = 1; end
    if nargin < 2 || isempty(mode); mode = 'diff'; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(genpath(fullfile(root, 'model')));
    od = fullfile(root, 'test_results', 'temp_1009_ws');  pc = physical_constants();  R_UM = pc.R;
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  FS = 22;  LFS = 17;  AXLW = 2.2;
    AB = char(257);  HAT = char(770);  az = [AB '_z'];  NM = {'difference', 'state'};  ARM = {'diff', 'state'};
    if startsWith(mode, 'ws'); NM = {[AB char(8242) ' state'], ['w' char(772) '_s state']};  ARM = {'state', mode}; end   % 'ws9' = w_s arm with the Section 9 law (no gate, no cap)
    CASES = {'plane', ''; 'cell', '_ad060'};
    for ic0 = 1:2
        w = CASES{ic0, 1};
        f = figure('Position', [40 40 1600 2000], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
        tl = tiledlayout(4, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
        for ic = 1:2
            S = load(fullfile(od, sprintf('apreg_%s_%s%s.mat', ARM{ic}, w, CASES{ic0, 2})));  q = find(S.seeds == seed);
            S.h_star = fzero(@(x) local_a(x, w) - S.a_d, [1.001 30]);
            t = S.t;  hd = S.wd(:, q);  hd(1) = hd(2);  h = S.h_true(:, q);  ax = gobjects(4, 1);
            ax(1) = nexttile(tl, ic);  hold on; box on;
            h1 = plot(ax(1), t, hd * R_UM, '-', 'Color', cT, 'LineWidth', 5.0, 'DisplayName', 'z_d');
            h2 = plot(ax(1), t, h * R_UM, '--', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', 'z');
            hs = yline(ax(1), S.h_star * R_UM, '--', 'Color', 'k', 'LineWidth', 1.5, 'DisplayName', 'z^*');
            set(ax(1), 'YScale', 'log');  ylim(ax(1), [2 60]);  yticks(ax(1), [2 3 5 10 20 50]);
            lg = legend(ax(1), [h1 h2 hs], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
            lg.Title.String = sprintf('%s wall, %s, %s_d = %.2f', w, NM{ic}, AB, S.a_d);
            ax(2) = nexttile(tl, 2 + ic);  hold on; box on;
            ht = plot(ax(2), t, S.a_true(:, q), '-', 'Color', cT, 'LineWidth', 5.0, 'DisplayName', az);
            hh = plot(ax(2), t, S.a_hat(:, q), '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', [AB HAT '_z']);
            hdd = yline(ax(2), S.a_d, '--', 'Color', 'k', 'LineWidth', 1.5, 'DisplayName', [AB '_d']);
            ylim(ax(2), [-0.2 1.6]);
            legend(ax(2), [ht hh hdd], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
            ax(3) = nexttile(tl, 4 + ic);  hold on; box on;
            apt = local_ap(max(h, 1.001), w);  aph = S.ap_hat(:, q);  aph(aph <= 0) = NaN;
            ht3 = plot(ax(3), t, apt, '-', 'Color', cT, 'LineWidth', 5.0, 'DisplayName', [AB '''_z']);
            hh3 = plot(ax(3), t, aph, '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', [AB HAT '''_z']);
            set(ax(3), 'YScale', 'log');  ylim(ax(3), [1e-3 3]);  yticks(ax(3), [1e-3 1e-2 1e-1 1]);
            legend(ax(3), [ht3 hh3], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
            ax(4) = nexttile(tl, 6 + ic);  hold on; box on;
            plot(ax(4), t, S.dxm(:, q), '-', 'Color', cE, 'LineWidth', 1.0);
            ylim(ax(4), [-0.2 0.2]);  yticks(ax(4), [-0.1 0 0.1]);
            xlabel(ax(4), 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
            for k = 1:4; xlim(ax(k), [0 t(end)]);  set(ax(k), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(ax(k), 'off'); end
            if ic == 1
                ylabel(ax(1), ['z  [' char(181) 'm]'], 'FontSize', FS, 'FontWeight', 'bold');
                ylabel(ax(2), az, 'FontSize', FS + 2, 'FontWeight', 'bold');
                ylabel(ax(3), [AB '''_z'], 'FontSize', FS + 2, 'FontWeight', 'bold');
                ylabel(ax(4), [char(948) 'z_m  [' char(181) 'm]'], 'FontSize', FS, 'FontWeight', 'bold');
            end
            e = S.a_true - S.a_hat;  lt = t > 5;  ea = mean(e(lt, :), 1);  gap = mean(S.h_true(lt, :), 1) - S.h_star;
            k1 = find(t >= 1.0, 1);
            fprintf('%-5s %-8s a_d %.2f | contact %d/5 | min h %.3f | e_a @1 s seed %d %+.3f | last 1 s e_a %+.4f +- %.4f | h - h* %+.3f +- %.3f R\n', ...
                    w, ARM{ic}, S.a_d, nnz(S.contact), min(S.h_true(:)), seed, e(k1, q), mean(ea), std(ea), mean(gap), std(gap));
        end
        out = fullfile(od, sprintf('fig_mc_apreg_%s_s%d.png', w, seed));
        if startsWith(mode, 'ws'); out = fullfile(od, sprintf('fig_mc_ap%s_reg_%s_s%d.png', mode, w, seed)); end
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
