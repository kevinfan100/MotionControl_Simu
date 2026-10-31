% STATUS: SCRATCH -- 2026-10-25 (user: replicate Meng diss. Sec. 5.5, Figs. 38-40, look at the angle convergence). Runs meng55_s<seed>
%   (run_meng55.sh): plane tilted 45 deg, start 15 um, approach 11.5 um along the true normal at 5 um/s, default init.
%   Left column (Fig. 40): theta-hat_y (wall tilt, true 45 deg) and theta-hat_x (true 0); right column (Fig. 39): a-hat_T, a-hat_N vs the
%   true gains; bottom left: distance h [um]. Seed 2 bold blue, other seeds light; true red.
%   -> test_results/temp_3d_wsonly/fig_t2_meng55.png | run: plot_t2_meng55(1:3, 2)
function plot_t2_meng55(SD, show, tag)
    if nargin < 3; tag = 'meng55'; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_3d_wsonly');  addpath(genpath(fullfile(root, 'model')));  pc = physical_constants();
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  cL = [0.65 0.75 0.95];  FS = 17;  LFS = 13;
    TH = char(952);  HT = char(770);  DG = char(176);  MU = char(181);  AB = char(257);
    f = figure('Position', [40 40 1500 1050], 'Color', 'w', 'Visible', 'off');
    tl = tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    rows = {{'thy', 'thy_true', [TH HT '_y'], [TH '_y'], [TH '_y  [' DG ']'], [-5 55]}, {'aT', 'aT_true', [AB HT '_T'], [AB '_T'], [AB '_T'], [0.5 1]}, ...
            {'thx', 'thx_true', [TH HT '_x'], [TH '_x'], [TH '_x  [' DG ']'], [-15 15]}, {'aN', 'aN_true', [AB HT '_N'], [AB '_N'], [AB '_N'], [0.2 0.9]}, ...
            {'h', '', 'h', '', ['h  [' MU 'm]'], [0 16]}};
    for q = 1:5
        ax = nexttile(tl, q);  hold on;  box on;  hh = gobjects(0);
        for s = [setdiff(SD, show), show]
            S = load(fullfile(od, sprintf('temp_t2_%s_s%d.mat', tag, s)));  r = S.R.runs{1};  k = 1:r.kend;  t = r.t(k);
            v = r.(rows{q}{1})(k);  if q == 5; v = v * pc.R; end
            if s == show
                if ~isempty(rows{q}{2}); hh(end + 1) = plot(t, r.(rows{q}{2})(k), '-', 'Color', cT, 'LineWidth', 2.5, 'DisplayName', rows{q}{4}); end   %#ok<AGROW>
                hh(end + 1) = plot(t, v, '-', 'Color', cE, 'LineWidth', 1.5, 'DisplayName', sprintf('%s,  seed %d', rows{q}{3}, s));   %#ok<AGROW>
            else
                plot(t, v, '-', 'Color', cL, 'LineWidth', 1, 'HandleVisibility', 'off');
            end
        end
        if q == 5; plot(t([1 end]), pc.R * [1 1], '-', 'Color', cT, 'LineWidth', 2.5, 'HandleVisibility', 'off'); end
        uistack(hh, 'top');  ylim(rows{q}{6});  xlim([0 t(end)]);
        ylabel(rows{q}{5}, 'FontSize', FS - 1, 'FontWeight', 'bold');
        if q >= 4; xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold'); end
        legend(hh, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        set(ax, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 2);
    end
    for s = SD
        S = load(fullfile(od, sprintf('temp_t2_%s_s%d.mat', tag, s)));  r = S.R.runs{1};  k = 1:r.kend;  t = r.t(k);
        e = abs(r.thy(k) - r.thy_true(k));  tail = flipud(cummax(flipud(e)));
        tt = @(lim) t(find(tail < lim, 1));  t10 = tt(10);  t5 = tt(5);  if isempty(t10); t10 = NaN; end;  if isempty(t5); t5 = NaN; end
        fprintf('seed %d: contact %d | theta-hat_y at 0.5 / 1 / 1.5 / 2 / %.2f s: %s deg | stays within 10 deg from %.2f s, 5 deg from %.2f s | sqrt P end %.1f deg | h end %.2f um\n', ...
                s, r.contact, t(end), mat2str(round(r.thy(k(arrayfun(@(x) find(t >= x, 1), [0.5 1 1.5 2]))), 1)'), t10, t5, r.sP(k(end), 6), r.h(k(end)) * pc.R);
    end
    out = fullfile(od, ['fig_t2_' tag '.png']);  exportgraphics(f, out, 'Resolution', 150);  close(f);  fprintf('figure: %s\n', out);
end
