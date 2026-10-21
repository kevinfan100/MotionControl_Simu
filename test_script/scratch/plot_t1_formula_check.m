% STATUS: SCRATCH -- 2026-10-16, verification figure of 1015_wall_angle_readout Section 5 (user "這個的驗證圖先產出").
%   x = e_theta = theta_y - theta-hat_y (wall 20 deg); red line = Section 5 exact expressions (with the true gains at the hold);
%   blue = readout only rotated by theta-hat (control law with the true angle) -> tests Section 5 itself;
%   black squares = control law also uses theta-hat (closed loop, T1). Seed mean +- SEM, 15 seeds.
%   (a) a_Nm / a_N   (b) a_Tm / a_T (u direction)   (c) e_theta_m [deg]; row 2 (user 10-16): measured - Section 5
%   -> test_results/temp_3d_wsonly/fig_wa_t1_formula.png | run: plot_t1_formula_check
%   plot_t1_formula_check(true) -> document version: top row only, Section 5 line + readout only, saved to
%   reference/eq17_analysis/derivation/figures/fig_wa_t1_formula.png (1015_wall_angle_readout Fig. 2)
function plot_t1_formula_check(doc)
    if nargin < 1; doc = false; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_3d_wsonly');
    F = load(fullfile(od, 'temp_t1_formula_check.mat'));  T = load(fullfile(od, 'temp_t1_angle_readout.mat'));
    th0 = F.R.opts.theta_y;  A = [F.R.A{:}];  thr = F.R.opts.th_read;
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  cK = [0 0 0];  FS = 20;  LFS = 15;
    AB = char(257);  DG = char(176);  TH = char(952);  HAT = char(770);
    aT = mean([A.aT]);  aN = mean([A.aN]);
    % arm A: one run per seed, all read angles
    MA = cat(3, [A.m_aNm] ./ [A.aN], [A.m_aTu] ./ [A.aT], [A.m_eth]);           % nr x seeds x 3
    k0 = find(thr == th0);                                                      % 10-19 (user): PAIRED by seed -- subtract each seed's own
    MA = MA - MA(k0, :, :) + reshape([1 1 0], 1, 1, 3);                         %   reading at e_theta = 0 (removes the shared fluctuation)
    eA = th0 - thr;  mA = squeeze(mean(MA, 2));  sA = squeeze(std(MA, 0, 2)) / sqrt(size(MA, 2));
    % arm B: closed loop, new runs + T1
    B = [F.R.B{:}];  T1 = [T.R.runs{:}];
    if isempty(B); B = struct('th_ctrl', {}, 'm_aNm', {}, 'aN', {}, 'm_aTu', {}, 'aT', {}, 'm_eth', {}); end   % 10-19 small-angle run has no B arm
    thB = unique([[B.th_ctrl], [T1.th_hat]]);
    mB = zeros(numel(thB), 3);  sB = mB;
    for i = 1:numel(thB)
        Bi = B([B.th_ctrl] == thB(i));
        if ~isempty(Bi)
            v = [[Bi.m_aNm] ./ [Bi.aN]; [Bi.m_aTu] ./ [Bi.aT]; [Bi.m_eth]];
        else
            Ti = T1([T1.th_hat] == thB(i));
            v = [[Ti.m_aNm] ./ [Ti.aN]; [Ti.m_aTu] ./ [Ti.aT]; [Ti.m_thy] - thB(i)];
        end
        mB(i, :) = mean(v, 2)';  sB(i, :) = std(v, 0, 2)' / sqrt(size(v, 2));
    end
    eB = th0 - thB;
    eg = linspace(-7, 7, 300);  er = eg * pi / 180;   % 10-19: small angle errors only (user)
    fN = (sin(er).^2 * aT + cos(er).^2 * aN) / aN;  fT = (cos(er).^2 * aT + sin(er).^2 * aN) / aT;  fE = sin(er) .* cos(er) * 180 / pi;
    YL = {[0.96 1.06], [0.96 1.06], [-7 7]};
    YLAB = {[AB '_{Nm} / ' AB '_N'], [AB '_{Tm} / ' AB '_T'], ['e_{' TH 'm}  [' DG ']']};
    FF = {fN, fT, fE};
    if doc                                                     % document version (user 10-16): top row only, readout only
        f = figure('Position', [40 40 2400 760], 'Color', 'w', 'Visible', 'off');
        tl = tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    else
        f = figure('Position', [40 40 2400 1400], 'Color', 'w', 'Visible', 'off');
        tl = tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    end
    sec5 = {@(e) (sin(e).^2 * aT + cos(e).^2 * aN) / aN, @(e) (cos(e).^2 * aT + sin(e).^2 * aN) / aT, @(e) sin(e) .* cos(e) * 180 / pi};
    YD = {[-0.05 0.2], [-0.05 0.15], [-1.5 3]};
    for q = (1:3) * ~doc                                       % row 2: measured - Section 5 (not in the document version)
        if q == 0; break; end
        ax = nexttile(tl, 3 + q);  hold on;  box on;
        h1 = plot(ax, eg, 0 * eg, '-', 'Color', cT, 'LineWidth', 4, 'DisplayName', 'Section 6');
        h2 = errorbar(ax, eA, mA(:, q) - sec5{q}(eA(:) * pi / 180), sA(:, q), 'o', 'Color', cE, 'MarkerFaceColor', cE, 'MarkerSize', 9, 'LineWidth', 2, 'DisplayName', 'readout only');
        h3 = errorbar(ax, eB, mB(:, q) - sec5{q}(eB(:) * pi / 180), sB(:, q), 's', 'Color', cK, 'MarkerSize', 11, 'LineWidth', 2, 'DisplayName', 'closed loop');
        xlim(ax, [-7 7]);  ylim(ax, YD{q});
        xlabel(ax, ['e_' TH ' = ' TH '_y - ' TH HAT '_y  [' DG ']'], 'FontSize', FS, 'FontWeight', 'bold');
        ylabel(ax, [YLAB{q} '  -  Section 5'], 'FontSize', FS - 2, 'FontWeight', 'bold');
        legend(ax, [h1 h2 h3], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2);  grid(ax, 'off');
    end
    for q = 1:3
        ax = nexttile(tl, q);  hold on;  box on;
        h1 = plot(ax, eg, FF{q}, '-', 'Color', cT, 'LineWidth', 4, 'DisplayName', 'Section 6');
        h2 = errorbar(ax, eA, mA(:, q), sA(:, q), 'o', 'Color', cE, 'MarkerFaceColor', cE, 'MarkerSize', 9, 'LineWidth', 2, 'DisplayName', 'readout only');
        hs = [h1 h2];
        if ~doc
            h3 = errorbar(ax, eB, mB(:, q), sB(:, q), 's', 'Color', cK, 'MarkerSize', 11, 'LineWidth', 2, 'DisplayName', 'closed loop');
            hs = [h1 h2 h3];
        end
        xlim(ax, [-7 7]);  ylim(ax, YL{q});
        xlabel(ax, ['e_' TH ' = ' TH '_y - ' TH HAT '_y  [' DG ']'], 'FontSize', FS, 'FontWeight', 'bold');
        ylabel(ax, YLAB{q}, 'FontSize', FS, 'FontWeight', 'bold');
        legend(ax, hs, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2);  grid(ax, 'off');
    end
    out = fullfile(od, 'fig_wa_t1_formula.png');
    if doc; out = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures', 'fig_wa_t1_formula.png'); end   % 1015_wall_angle_readout Fig. 2
    exportgraphics(f, out, 'Resolution', 140);  close(f);  fprintf('figure: %s\n', out);
    fprintf('e_theta | a_Nm/a_N: Sec5  A  B         | a_Tm/a_T: Sec5  A  B        | e_theta_m: Sec5  A  B\n');
    for i = 1:numel(eA)
        j = find(eB == eA(i), 1);  b = NaN(1, 3);  if ~isempty(j); b = mB(j, :); end
        e = eA(i) * pi / 180;
        fprintf('%6.0f  | %.3f %.3f %.3f | %.3f %.3f %.3f | %+6.2f %+6.2f %+6.2f\n', eA(i), ...
                (sin(e)^2 * aT + cos(e)^2 * aN) / aN, mA(i, 1), b(1), (cos(e)^2 * aT + sin(e)^2 * aN) / aT, mA(i, 2), b(2), ...
                sin(e) * cos(e) * 180 / pi, mA(i, 3), b(3));
    end
end
