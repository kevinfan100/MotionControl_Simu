% PURPOSE (2026-09-20): figures of run_delta_fixed_speed.m (offset law, version A, seed = plant delta +- 0.5 R).
%   Row 1  fixed height, one panel per height (2.0 R, 1.10 R): delta_hat - target against time, seed mean (solid) with the
%          seed-mean +- sqrt(P) (dashed), plane blue / cell orange, + seed solid / - seed dotted. Converging from the level
%          alone is the offset law's advantage over [1/1] (C57: one height = one line).
%   Row 2  speed scan, one panel per plant: delta_hat - target against time for T_d = 0.2 / 2 / 20 s (+ seed), descent end
%          marked; the y1 path (~ velocity) does the work at 0.2 s, the y2 level at 20 s.
%   Output reference/eq17_analysis/derivation/figures/delta_fixed_speed.png | EXPIRES: with the delta-law line
function plot_delta_fixed_speed(sign, res_dir)
    if nargin < 1 || isempty(sign); sign = 'T'; end                          % 'T' seed = plant; 'PI' plane init
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    if nargin < 2 || isempty(res_dir); res_dir = fullfile(root, 'test_results', 'temp_delta_two_walls'); end
    od = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    CP = [0 0.2 0.9]; CC = [0.85 0.45 0]; FS = 18; LFS = 13; AXLW = 2.0;   % house style
    f = figure('Position', [40 40 1500 950], 'Color', 'w', 'Visible', 'off');
    tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    H = {'2p00', '1p10'};  HL = {'2.0 R', '1.10 R'};
    for ih = 1:2
        nexttile(ih); hold on; box on;  hs = gobjects(0);
        for pl = {'plane', 'cell'}
            col = CP; if strcmp(pl{1}, 'cell'); col = CC; end
            for sg = {sign}                                            % only the requested sign (user 09-20: simpler legends)
                fn = fullfile(res_dir, sprintf('fixed_%s_%s_h%s.mat', pl{1}, sg{1}, H{ih}));
                if ~exist(fn, 'file'); continue; end
                r = load(fn);  e = r.dl_hat - r.dl_target;  m = mean(e, 2);  sp = mean(r.sP_dl, 2);
                ls = '-'; if strcmp(sg{1}, 'Tm'); ls = ':'; elseif strcmp(sg{1}, 'Tp'); ls = '-.'; end
                hs(end+1) = plot(r.t, m, ls, 'Color', col, 'LineWidth', 2.0, 'DisplayName', sprintf('%s, seed %+.1f R', pl{1}, r.dl_seed - r.dl_target));
                plot(r.t, m + sp, '--', 'Color', col, 'LineWidth', 0.8, 'HandleVisibility', 'off');
                plot(r.t, m - sp, '--', 'Color', col, 'LineWidth', 0.8, 'HandleVisibility', 'off');
            end
        end
        yline(0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off');
        ylim([-0.8 0.8]);  if startsWith(sign, 'PI'); ylim([-1.2 0.4]); end;  if ~isempty(hs); xlim([0 r.t(end)]); end
        h0 = plot(NaN, NaN, 'w.', 'DisplayName', sprintf('hold %s; dashed = \\pm sqrt(P)', HL{ih}));
        xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');  if ih == 1; ylabel('\delta_{hat} - \delta  [R]', 'FontSize', FS, 'FontWeight', 'bold'); end
        legend([h0 hs], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
    end
    TD = {'0p2', '2', '20'};  TDL = {'0.2', '2', '20'};  shade = [1.0, 0.65, 0.35];
    pls = {'plane', 'cell'};
    for ip = 1:2
        nexttile(2 + ip); hold on; box on;  hs = gobjects(0);  base = CP; if ip == 2; base = CC; end
        for it = 1:3
            fn = fullfile(res_dir, sprintf('speed_%s_%s_Td%s.mat', pls{ip}, sign, TD{it}));     % requested sign; fall back to the +0.5 R arm
            if ~exist(fn, 'file'); fn = fullfile(res_dir, sprintf('speed_%s_Tp_Td%s.mat', pls{ip}, TD{it})); end
            if ~exist(fn, 'file'); continue; end
            r = load(fn);  e = r.dl_hat - r.dl_target;  m = mean(e, 2);  sp = mean(r.sP_dl, 2);
            col = 1 - (1 - base) * shade(it);
            hs(end+1) = plot(r.t, m, '-', 'Color', col, 'LineWidth', 2.0, 'DisplayName', sprintf('T_d = %s s', TDL{it}));
            plot(r.t, m + sp, '--', 'Color', col, 'LineWidth', 0.8, 'HandleVisibility', 'off');
            plot(r.t, m - sp, '--', 'Color', col, 'LineWidth', 0.8, 'HandleVisibility', 'off');
            xline(r.seg(2), ':', 'Color', col, 'LineWidth', 1.2, 'HandleVisibility', 'off');
        end
        yline(0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off');
        set(gca, 'XScale', 'log'); xlim([0.05 25]); ylim([-0.3 0.8]);  if startsWith(sign, 'PI'); ylim([-1.2 0.4]); end
        h0 = plot(NaN, NaN, 'w.', 'DisplayName', sprintf('%s wall, 5.5 -> 1.10 R, \\delta seed %.2f', upper(pls{ip}), r.dl_seed));
        xlabel('Time (sec, log)', 'FontSize', FS, 'FontWeight', 'bold');  if ip == 1; ylabel('\delta_{hat} - \delta  [R]', 'FontSize', FS, 'FontWeight', 'bold'); end
        legend([h0 hs], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
    end
    fn = fullfile(od, sprintf('delta_fixed_speed_%s.png', sign));
    exportgraphics(f, fn, 'Resolution', 150); close(f); fprintf('figure: %s\n', fn);
end
