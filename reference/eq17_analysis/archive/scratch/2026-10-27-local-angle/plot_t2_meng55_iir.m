% STATUS: SCRATCH -- 2026-10-25 (user: Meng's IIR angle method literally, a_cov = 1 and 0.05, vs ours, single seed, sqrt P_theta[0] = 20 deg).
%   Runs meng55iir1_s2 / meng55iir005_s2 (run_meng55_iir.sh): Meng Sec. 5.5 scenario; our estimator controls, the Meng observer
%   (diss. 5.4-5.14) is passive. Columns: ours (estimated angle, sqrt P 20 deg) | Meng IIR a_cov = 1 | Meng IIR a_cov = 0.05.
%   Rows: theta_y (true 45 deg), theta_x (true 0). -> test_results/temp_3d_wsonly/fig_t2_meng55_iir.png | run: plot_t2_meng55_iir()
function plot_t2_meng55_iir()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_3d_wsonly');
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  cG = [0 0.55 0.2];  FS = 17;  LFS = 13;  TH = char(952);  HT = char(770);  DG = char(176);
    A = load(fullfile(od, 'temp_t2_meng55iir1_s2.mat'));  B = load(fullfile(od, 'temp_t2_meng55iir005_s2.mat'));
    ra = A.R.runs{1};  rb = B.R.runs{1};  k = 1:ra.kend;  t = ra.t(k);
    C = {{ra.thy(k), ra.thx(k), ['ours (estimated angle),  \surdP_' TH '[0] = 20' DG]}, ...
         {ra.th_meng(k, 2), ra.th_meng(k, 1), 'Meng IIR,  a_{cov} = 1'}, {rb.th_meng(k, 2), rb.th_meng(k, 1), 'Meng IIR,  a_{cov} = 0.05'}};
    f = figure('Position', [40 40 1900 800], 'Color', 'w', 'Visible', 'off');
    tl = tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ic = 1:3
        for q = 1:2
            ax = nexttile(tl, (q - 1) * 3 + ic);  hold on;  box on;
            tr = 45 * (q == 1);  col = cE;  if q == 2; col = cG; end
            h1 = plot([0 t(end)], [tr tr], '-', 'Color', cT, 'LineWidth', 2.5, 'DisplayName', [TH '_' char('y' - (q - 1)) ]);
            h2 = plot(t, C{ic}{q}, '-', 'Color', col, 'LineWidth', 1.5, 'DisplayName', [TH HT '_' char('y' - (q - 1))]);
            ylim([-90 90] * (q == 1) + [-45 45] * (q == 2) + [0 0]);  if q == 1; ylim([-30 90]); end
            xlim([0 t(end)]);
            if q == 1; title(C{ic}{3}, 'FontSize', FS - 2, 'FontWeight', 'bold'); end   % column label (three methods side by side)
            if ic == 1; ylabel([TH '_' char('y' - (q - 1)) '  [' DG ']'], 'FontSize', FS, 'FontWeight', 'bold'); end
            if q == 2; xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold'); end
            legend([h1 h2], 'Location', 'northwest', 'FontSize', LFS, 'FontWeight', 'bold');
            set(ax, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 2);
        end
        v = C{ic}{1};  pick = arrayfun(@(x) v(find(t >= x, 1)), [0.05 0.5 1 1.5 2]);
        e = abs(v - 45);  tail = flipud(cummax(flipud(e)));  i10 = find(tail < 10, 1);  t10 = NaN;  if ~isempty(i10); t10 = t(i10); end
        fprintf('%-40s theta_y at 0.05 / 0.5 / 1 / 1.5 / 2 / end: %s / %.1f deg | stays within 10 deg from %.2f s | theta_x end %.1f\n', ...
                C{ic}{3}, mat2str(round(pick, 1)), v(end), t10, C{ic}{2}(end));
    end
    out = fullfile(od, 'fig_t2_meng55_iir.png');  exportgraphics(f, out, 'Resolution', 150);  close(f);  fprintf('figure: %s\n', out);
end
