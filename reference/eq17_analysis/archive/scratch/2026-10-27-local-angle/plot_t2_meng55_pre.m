% STATUS: SCRATCH -- 2026-10-25 (user: test hypothesis (a) with our estimator -- does holding at the start before Meng's ramp put the angle at
%   45 deg by t = 0?). Runs meng55iir1_s2 (no hold), meng55pre5_s2, meng55pre20_s2 (hold 5 / 20 s at 15 um, then the 2.3 s approach), seed 2,
%   theta-hat_y[0] = 0, prior 20 deg. Time axis shifted so the approach starts at 0 (hold = negative time). Rows: theta-hat_y, theta-hat_x.
%   -> test_results/temp_3d_wsonly/fig_t2_meng55_pre.png | run: plot_t2_meng55_pre()
function plot_t2_meng55_pre()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_3d_wsonly');
    cT = [0.85 0 0];  CO = {[0.6 0.6 0.6], [0.9 0.5 0], [0 0.25 0.85]};  FS = 17;  LFS = 13;  TH = char(952);  HT = char(770);  DG = char(176);
    TG = {'meng55iir1_s2', 'meng55pre5_s2', 'meng55pre20_s2'};  T0 = [0 5 20];  LB = {'no hold', 'hold 5 s', 'hold 20 s'};
    f = figure('Position', [40 40 1500 800], 'Color', 'w', 'Visible', 'off');
    tl = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    for q = 1:2
        ax = nexttile(tl, q);  hold on;  box on;  hh = gobjects(0);
        hh(end + 1) = plot([-20 2.3], [45 45] * (q == 1), '-', 'Color', cT, 'LineWidth', 2.5, 'DisplayName', [TH '_' char('y' - (q - 1))]);
        for i = 1:3
            S = load(fullfile(od, ['temp_t2_' TG{i} '.mat']));  r = S.R.runs{1};  k = 1:r.kend;  t = r.t(k) - T0(i);
            v = r.thy(k);  if q == 2; v = r.thx(k); end
            hh(end + 1) = plot(t, v, '-', 'Color', CO{i}, 'LineWidth', 1.6, 'DisplayName', [TH HT '_' char('y' - (q - 1)) ', ' LB{i}]);   %#ok<AGROW>
            if q == 1
                at = @(x) v(find(t >= x, 1));
                fprintf('%-10s theta-hat_y at ramp start %.1f deg, at 0.5 / 1 / 2.3 s of the ramp %.1f / %.1f / %.1f deg | sqrt P at ramp start %.1f deg\n', ...
                        LB{i}, at(0), at(0.5), at(1), v(end), r.sP(k(find(t >= 0, 1)), 6));
            end
        end
        xlim([-20 2.3]);  ylim([-30 60]);  if q == 2; ylim([-45 45]); end
        ylabel([TH '_' char('y' - (q - 1)) '  [' DG ']'], 'FontSize', FS, 'FontWeight', 'bold');
        if q == 2; xlabel('Time from the start of the approach (sec)', 'FontSize', FS, 'FontWeight', 'bold'); end
        legend(hh, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        set(ax, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 2);
    end
    out = fullfile(od, 'fig_t2_meng55_pre.png');  exportgraphics(f, out, 'Resolution', 150);  close(f);  fprintf('figure: %s\n', out);
end
