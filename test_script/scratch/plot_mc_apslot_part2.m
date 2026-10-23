% STATUS: SCRATCH -- 2026-10-19 (user): 1019 report Part II, the slope a-bar' as a state variable; one figure per section, seed 1,
%   constant-velocity trajectory 22.2 -> 1.10 R in 1 s + 0.2 s held, plane wall (run_mc_apslot_r2 arms raw_q0, white_q0, white_fin).
%   'R': columns y2 = a_m | y2 = a_cov u, both q55 = 0. Rows: a-bar_z true (red) / estimate (blue); accumulated contributions to a-hat
%        (true change red, prediction a-hat' M-hat dark blue, y1 correction l41 e_x1 grey, y2 correction l42 e_am green; the three sums
%        add up to a-hat - a-hat[0] exactly); |l41| grey, |l42| green (50 ms mean, log).
%   'Q': columns q55 = 0 | q55 = P_fix^2 I (both y2 = a_cov u). Rows: a-bar_z; accumulated contributions; a-bar'_z true / estimate (log,
%        negative values not drawn); |l51| grey, |l52| green (50 ms mean, log), from the logged slot-7 updates / estimation errors.
%   Output test_results/temp_1009_ws/fig_mc_apslot_part2_{R,Q}_s<seed>.png.
function plot_mc_apslot_part2(which, seed)
    if nargin < 1 || isempty(which); which = {'R', 'Q'}; end
    if ischar(which); which = {which}; end
    if nargin < 2 || isempty(seed); seed = 1; end
    N_WIN = 80;                       % 50 ms at Ts = 1/1600 s
    od = fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), 'test_results', 'temp_1009_ws');
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  cG = [0.55 0.55 0.55];  cY2 = [0.10 0.60 0.20];  FS = 22;  LFS = 16;  AXLW = 2.2;
    AB = char(257);  HAT = char(770);  PR = char(8242);  EL = char(8467);  az = [AB '_z'];
    for iw = 1:numel(which)
        if strcmp(which{iw}, 'R')
            arms = {'raw_q0', 'white_q0'};  names = {['y_2 = ' AB '_m'], 'y_2 = a_{cov} u'};  nr = 3;
        else
            arms = {'white_q0', 'white_fin'};  names = {'q_{55} = 0', 'q_{55} = P_{fix}^2 I'};  nr = 4;
        end
        f = figure('Position', [40 40 1600 560 * nr], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
        tl = tiledlayout(nr, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
        for ic = 1:2
            S = load(fullfile(od, sprintf('apr2_%s_plane_desc.mat', arms{ic})));  t = S.t;  q = seed;
            c1 = S.da_y1(:, q);  c2 = S.da_y2(:, q);  c1(~isfinite(c1)) = 0;  c2(~isfinite(c2)) = 0;
            pred = [0; diff(S.a_hat(:, q))] - c1 - c2;
            if strcmp(which{iw}, 'R'); K1 = abs(c1 ./ S.nu1(:, q));  K2 = abs(c2 ./ S.nu2(:, q));  kl = {'41', '42'};
            else; K1 = abs(S.dap_y1(:, q) ./ S.nu1(:, q));  K2 = abs(S.dap_y2(:, q) ./ S.nu2(:, q));  kl = {'51', '52'}; end
            K1(~isfinite(K1)) = NaN;  K2(~isfinite(K2)) = NaN;
            ax = gobjects(nr, 1);
            % row 1: motion gain
            ax(1) = nexttile(tl, ic);  hold on; box on;
            h1 = plot(ax(1), t, S.a_true(:, q), '-', 'Color', cT, 'LineWidth', 5.0, 'DisplayName', az);
            h2 = plot(ax(1), t, S.a_hat(:, q), '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', [AB HAT '_z']);
            ylim(ax(1), [-0.2 1.2]);
            lg = legend(ax(1), [h1 h2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
            lg.Title.String = names{ic};
            % row 2: accumulated contributions
            ax(2) = nexttile(tl, 2 + ic);  hold on; box on;
            ht = plot(ax(2), t, S.a_true(:, q) - S.a_true(1, q), '-', 'Color', cT, 'LineWidth', 5.0, 'DisplayName', 'true');
            hp = plot(ax(2), t, cumsum(pred), '-', 'Color', cE, 'LineWidth', 2.4, 'DisplayName', ['\Sigma ' AB HAT PR ' M' HAT]);
            hy1 = plot(ax(2), t, cumsum(c1), '-', 'Color', cG, 'LineWidth', 2.0, 'DisplayName', ['\Sigma ' EL '_{41} e_{x_1}']);
            hy2 = plot(ax(2), t, cumsum(c2), '-', 'Color', cY2, 'LineWidth', 2.4, 'DisplayName', ['\Sigma ' EL '_{42} e_{a_m}']);
            yline(ax(2), 0, ':', 'Color', [0.3 0.3 0.3], 'HandleVisibility', 'off');  ylim(ax(2), [-1 0.4]);
            legend(ax(2), [ht hp hy1 hy2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
            r = 3;
            if strcmp(which{iw}, 'Q')   % row 3: slope
                ax(3) = nexttile(tl, 4 + ic);  hold on; box on;
                apt = S.ap_true(:, q);  aph = S.ap_hat(:, q);  aph(aph <= 0) = NaN;
                h3 = plot(ax(3), t, apt, '-', 'Color', cT, 'LineWidth', 5.0, 'DisplayName', [AB PR '_z']);
                h4 = plot(ax(3), t, aph, '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', [AB HAT PR '_z']);
                set(ax(3), 'YScale', 'log');  ylim(ax(3), [1e-4 3]);  yticks(ax(3), [1e-4 1e-2 1]);
                legend(ax(3), [h3 h4], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
                r = 4;
            end
            % last row: feedback entries
            ax(r) = nexttile(tl, 2 * (r - 1) + ic);  hold on; box on;
            hk1 = plot(ax(r), t, movmean(K1, N_WIN, 'omitnan'), '-', 'Color', cG, 'LineWidth', 2.0, 'DisplayName', ['|' EL '_{' kl{1} '}|']);
            hk2 = plot(ax(r), t, movmean(K2, N_WIN, 'omitnan'), '-', 'Color', cY2, 'LineWidth', 2.4, 'DisplayName', ['|' EL '_{' kl{2} '}|']);
            set(ax(r), 'YScale', 'log');
            if strcmp(which{iw}, 'R'); ylim(ax(r), [1e-5 1]);  yticks(ax(r), 10.^(-5:0)); else; ylim(ax(r), [1e-6 10]);  yticks(ax(r), 10.^(-6:2:0)); end
            legend(ax(r), [hk1 hk2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
            xlabel(ax(r), 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
            for k = 1:nr; xlim(ax(k), [0 t(end)]);  set(ax(k), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(ax(k), 'off'); end
            if ic == 1
                ylabel(ax(1), az, 'FontSize', FS + 2, 'FontWeight', 'bold');
                ylabel(ax(2), ['change of ' AB HAT '_z'], 'FontSize', FS - 2, 'FontWeight', 'bold');
                if r == 4; ylabel(ax(3), [AB PR '_z'], 'FontSize', FS + 2, 'FontWeight', 'bold'); end
                ylabel(ax(r), ['|' EL '_{' kl{1} '}|,  |' EL '_{' kl{2} '}|'], 'FontSize', FS - 2, 'FontWeight', 'bold');
            end
        end
        out = fullfile(od, sprintf('fig_mc_apslot_part2_%s_s%d.png', which{iw}, seed));
        exportgraphics(f, out, 'Resolution', 150);  close(f);  fprintf('figure: %s\n', out);
    end
end
