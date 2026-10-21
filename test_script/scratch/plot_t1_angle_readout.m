% STATUS: SCRATCH -- 2026-10-16, figure for run_t1_angle_readout (house style: bold, true = red, measured = blue, no grid / title).
%   (a) mean measured angle theta_ym over the hold vs the fixed estimated angle theta-hat_y, seed mean +- SEM; red = wall 20 deg,
%       10-19 user: red lines = expectations of Section 6 (same as Fig. 2): left sin e cos e, right a_Nm solid, a_Tm dashed
%   (b) mean measured input gains / true gains vs e_theta.
%   -> reference/eq17_analysis/derivation/figures/fig_wa_t1_readout.png (1015_wall_angle_readout.tex); a_Tm here = the u direction
%   only, as that document (x-z plane) | run: plot_t1_angle_readout
function plot_t1_angle_readout()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_3d_wsonly');
    S = load(fullfile(od, 'temp_t1_angle_readout.mat'));  runs = [S.R.runs{:}];  th0 = S.R.opts.theta_y;
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  cGr = [0 0.55 0.2];  cG = [0.45 0.45 0.45];  FS = 20;  LFS = 15;
    AB = char(257);  HAT = char(770);  DG = char(176);  TH = char(952);
    ths = unique([runs.th_hat]);  n = numel(ths);
    my = zeros(n, 1);  sy = my;  mT = my;  sT = my;  mN = my;  sN = my;  pT = my;  pN = my;
    % 10-19 (user): PAIRED by seed -- every point shares the same seeds, so each seed's own reading at e_theta = 0 (theta-hat = theta_y)
    %   is subtracted; this removes the common fluctuation of the seed set that otherwise shifts all points together.
    B0 = runs([runs.th_hat] == th0);  [~, ib] = sort([B0.seed]);  B0 = B0(ib);
    b_th = [B0.m_thy] - th0;  b_T = [B0.m_aTu] ./ [B0.aT];  b_N = [B0.m_aNm] ./ [B0.aN];
    for i = 1:n
        R = runs([runs.th_hat] == ths(i));  [~, ir] = sort([R.seed]);  R = R(ir);  k = numel(R);
        v = ([R.m_thy] - ths(i)) - b_th;  my(i) = ths(i) + mean(v);  sy(i) = std(v) / sqrt(k);
        v = [R.m_aTu] ./ [R.aT] - b_T + 1;  mT(i) = mean(v);  sT(i) = std(v) / sqrt(k);   % u direction only = a_Tm of 1015_wall_angle_readout
        v = [R.m_aNm] ./ [R.aN] - b_N + 1;  mN(i) = mean(v);  sN(i) = std(v) / sqrt(k);
        e = (th0 - ths(i)) * pi / 180;  aT = mean([R.aT]);  aN = mean([R.aN]);
        pT(i) = (cos(e)^2 * aT + sin(e)^2 * aN) / aT;  pN(i) = (sin(e)^2 * aT + cos(e)^2 * aN) / aN;
    end
    xf = linspace(min(ths) - 2, max(ths) + 2, 200);  ef = (th0 - xf) * pi / 180;
    f = figure('Position', [40 40 1800 700], 'Color', 'w', 'Visible', 'off');
    tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax = nexttile(tl);  hold on;  box on;                     % (a) as estimation errors: e_theta = theta_y - theta-hat_y, e_theta_m = theta_ym - theta-hat_y
    ex = th0 - ths;  eg = linspace(min(ex) - 2, max(ex) + 2, 200);
    h1 = plot(ax, eg, sin(2 * eg * pi / 180) / 2 * 180 / pi, '-', 'Color', cT, 'LineWidth', 4, 'DisplayName', 'Section 6');   % 10-19 user: red = expectation of Section 6
    h3 = errorbar(ax, ex, my(:)' - ths, sy, 'o', 'Color', cE, 'MarkerFaceColor', cE, 'MarkerSize', 9, 'LineWidth', 2, 'DisplayName', ['e_{' TH 'm}  (mean)']);
    xlim(ax, [min(eg), max(eg)]);  ylim(ax, [min(eg) max(eg)]);   % 10-19: small angle errors only (user)
    xlabel(ax, ['e_' TH ' = ' TH '_y - ' TH HAT '_y  [' DG ']'], 'FontSize', FS, 'FontWeight', 'bold');
    ylabel(ax, ['e_{' TH 'm} = ' TH '_{ym} - ' TH HAT '_y  [' DG ']'], 'FontSize', FS, 'FontWeight', 'bold');
    legend(ax, [h1 h3], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
    set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2);  grid(ax, 'off');
    ax = nexttile(tl);  hold on;  box on;
    aT = mean([runs.aT]);  aN = mean([runs.aN]);  er = eg * pi / 180;      % red = expectations of Section 6 at the hold gains
    g1 = plot(ax, eg, cos(er).^2 + sin(er).^2 * aT / aN, '-', 'Color', cT, 'LineWidth', 4, 'DisplayName', 'Section 6');
    plot(ax, eg, cos(er).^2 + sin(er).^2 * aN / aT, '--', 'Color', cT, 'LineWidth', 3, 'HandleVisibility', 'off');
    g3 = errorbar(ax, ex, mN, sN, 'o', 'Color', cE, 'MarkerFaceColor', cE, 'MarkerSize', 9, 'LineWidth', 2, 'DisplayName', [AB '_{Nm} / ' AB '_N']);
    g4 = errorbar(ax, ex, mT, sT, 's', 'Color', cGr, 'MarkerFaceColor', cGr, 'MarkerSize', 9, 'LineWidth', 2, 'DisplayName', [AB '_{Tm} / ' AB '_T']);
    xlim(ax, [min(eg), max(eg)]);  ylim(ax, [0.96 1.06]);
    xlabel(ax, ['e_' TH ' = ' TH '_y - ' TH HAT '_y  [' DG ']'], 'FontSize', FS, 'FontWeight', 'bold');  ylabel(ax, 'measured / true', 'FontSize', FS, 'FontWeight', 'bold');
    legend(ax, [g1 g3 g4], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
    set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2);  grid(ax, 'off');
    out = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures', 'fig_wa_t1_readout.png');   % referenced by 1015_wall_angle_readout.tex
    exportgraphics(f, out, 'Resolution', 140);  close(f);  fprintf('figure: %s\n', out);
    for i = 1:n
        fprintf('theta-hat %3g: theta_ym %.2f +- %.2f (no echo %.2f) | a_Nm/a_N %.3f +- %.3f (no echo %.3f) | a_Tm/a_T %.3f +- %.3f (no echo %.3f)\n', ...
                ths(i), my(i), sy(i), ths(i) + sin(2 * (th0 - ths(i)) * pi / 180) / 2 * 180 / pi, mN(i), sN(i), pN(i), mT(i), sT(i), pT(i));
    end
end
