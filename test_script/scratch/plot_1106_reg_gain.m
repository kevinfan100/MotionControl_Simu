function plot_1106_reg_gain(seeds)
% PURPOSE: which output moves a-hat in the 1-D Meng-law PI regulation (plot_1106_reg setting, 10 s): per step
%   a-hat[k+1] - a-hat[k] = (update by y1: l41 e_x1) + (update by y2: l42 e_am) + (prediction a'-hat M-hat), update form of the code
%   (K(4,1) nu_1, K(4,2) nu_2). Figure seed 1 (style of the 1019 Fig. 4.1: prediction dark blue, y1 grey, y2 green); console all seeds.
% EXPIRES: with run_1106_meng_law_1d.m.
    if nargin < 1; seeds = 1:10; end
    addpath(genpath(fullfile(fileparts(mfilename('fullpath')), '..', '..', 'model')));
    pc = physical_constants();  T = 10;  h0 = 15 / pc.R;  reg = struct('a_d', 0.05, 'tau_p', 0.5, 'tau_i', 0.5);
    R = run_1106_meng_law_1d(struct('seeds', seeds, 'laws', {{'meng'}}, 'h0', h0, 'h1', h0, 'T_move', 1, 'T_hold', T - 1, 'reg', reg));
    W = [0 1; 1 2; 2 T];
    fprintf('\nupdate of a-hat per step by channel (rms over the window, mean over seeds) and the net sum over the window\n');
    fprintf('%-8s %12s %12s %12s %10s   %12s %12s %12s\n', 'window', 'rms y1', 'rms y2', 'rms pred', 'y2/y1', 'sum y1', 'sum y2', 'sum pred');
    for iw = 1:size(W, 1)
        A = zeros(numel(seeds), 6);
        for is = 1:numel(seeds)
            r = R.meng(is);  w = r.a > 0 & r.t >= W(iw, 1) & r.t < W(iw, 2);
            A(is, :) = [rms(r.u41(w)), rms(r.u42(w)), rms(r.pr4(w)), sum(r.u41(w)), sum(r.u42(w)), sum(r.pr4(w))];
        end
        m = mean(A, 1);
        fprintf('%g-%g s  %12.2e %12.2e %12.2e %10.3f   %+12.4f %+12.4f %+12.4f\n', W(iw, :), m(1), m(2), m(3), m(2) / m(1), m(4), m(5), m(6));
    end
    % ---- figure, seed 1 ----
    r = R.meng(1);  m = r.a > 0;  t = r.t(m);  nw = round(0.05 / pc.Ts);
    cP = [0 0.15 0.55];  cY1 = [0.5 0.5 0.5];  cY2 = [0 0.6 0.2];  cE = [0 0.25 0.85];  FS = 22;  LFS = 17;  AXLW = 2.2;  AB = char(257);  HAT = char(770);  EL = char(8467);
    f = figure('Position', [40 40 1500 1700], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');  ax = gobjects(3, 1);
    ax(1) = nexttile(tl);  hold on; box on;
    h0l = plot(ax(1), t, r.ah(m) - r.ah(find(m, 1)), '-', 'Color', cE, 'LineWidth', 2.5, 'DisplayName', [AB HAT '_z[k] - ' AB HAT '_z[0]']);
    h1 = plot(ax(1), t, cumsum(r.pr4(m)), '-', 'Color', cP, 'LineWidth', 1.8, 'DisplayName', ['\Sigma ' AB HAT ''' M' HAT]);
    h2 = plot(ax(1), t, cumsum(r.u41(m)), '-', 'Color', cY1, 'LineWidth', 1.8, 'DisplayName', ['\Sigma ' EL '_{41} e_{x_1}']);
    h3 = plot(ax(1), t, cumsum(r.u42(m)), '-', 'Color', cY2, 'LineWidth', 1.8, 'DisplayName', ['\Sigma ' EL '_{42} e_{a_m}']);
    yline(ax(1), 0, '-', 'Color', [0.7 0.7 0.7], 'HandleVisibility', 'off');
    lg = legend(ax(1), [h0l h1 h2 h3], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
    lg.Title.String = sprintf('plane wall, Meng law, %s_d = %.2f, seed %d', AB, reg.a_d, seeds(1));
    ylabel(ax(1), ['change of ' AB HAT '_z'], 'FontSize', FS, 'FontWeight', 'bold');
    ax(2) = nexttile(tl);  hold on; box on;
    g1 = plot(ax(2), t, movmean(abs(r.u41(m)), nw), '-', 'Color', cY1, 'LineWidth', 2, 'DisplayName', ['|' EL '_{41} e_{x_1}|']);
    g2 = plot(ax(2), t, movmean(abs(r.u42(m)), nw), '-', 'Color', cY2, 'LineWidth', 2, 'DisplayName', ['|' EL '_{42} e_{a_m}|']);
    set(ax(2), 'YScale', 'log');  ylabel(ax(2), 'update per step', 'FontSize', FS, 'FontWeight', 'bold');
    legend(ax(2), [g1 g2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
    ax(3) = nexttile(tl);  hold on; box on;
    k1 = plot(ax(3), t, movmean(abs(r.k41(m)), nw), '-', 'Color', cY1, 'LineWidth', 2, 'DisplayName', ['|' EL '_{41}|  [1/R]']);
    k2 = plot(ax(3), t, movmean(abs(r.k42(m)), nw), '-', 'Color', cY2, 'LineWidth', 2, 'DisplayName', ['|' EL '_{42}|']);
    set(ax(3), 'YScale', 'log');  ylabel(ax(3), ['|' EL '|'], 'FontSize', FS, 'FontWeight', 'bold');
    legend(ax(3), [k1 k2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
    xlabel(ax(3), 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    for q = 1:3; xlim(ax(q), [0 T]);  set(ax(q), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(ax(q), 'off'); end
    od = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'test_results', 'temp_1106_meng');
    out = fullfile(od, sprintf('fig_1106_reg_gain_s%d.png', seeds(1)));  exportgraphics(f, out, 'Resolution', 150);  close(f);  fprintf('figure: %s\n', out);
end
