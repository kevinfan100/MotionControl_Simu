% PURPOSE (2026-09-24, user: "last page of the report: the three cell comparisons side by side"): one figure, cell wall,
%   plane init, T_d = 2 s, the c-only slope law (p = 2, runner law 'f1p2', prior 0.10), three columns = three forgetting
%   choices on the c slot, rows and style of plot_plaw_walls (a_true / a_hat, e_a, c_hat +- sqrt(P) with the wall's band-fit c):
%     information-driven lambda_k = max(exp(-3 s_k), 0.99)  |  uniform lambda_f = 0.999  |  lambda = 1   (order = the user's collage)
%   Runs: test_results/temp_plaw_walls/f1p2_cell_PI_sig010{_ib3,_lf999,}_ramp2.mat (run_lf_battery / run_plaw_walls, seeds 1:10)
%   Output reference/eq17_analysis/derivation/figures/f1p2_cell_lf_compare.png | EXPIRES: with the c-only forgetting line
function plot_f1p2_cell_lf_compare(arms, ftag, plant)
    if nargin < 3 || isempty(plant); plant = 'cell'; end            % 10-02 (user): the same three columns on another wall (file f1p2_<plant>_PI<arm>.mat, figure suffix _<plant> unless cell)
    if nargin < 2; ftag = ''; end                                   % 09-30: output suffix (e.g. '_rho' for the base | ratio | ib3 collage)
    if nargin < 1 || isempty(arms)
        arms = {'_sig010_ib3_ramp2',   '\lambda_k = max(exp(-3 s_k), 0.99)'; ...
                '_sig010_lf999_ramp2', '\lambda_f = 0.999'; ...
                '_sig010_ramp2',       '\lambda = 1'};
    end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    rd = fullfile(root, 'test_results', 'temp_plaw_walls');  od = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    COL_TRUE = [0.8 0 0]; COL_HAT = [0 0.2 0.9]; BANDC = [0.45 0.55 0.95]; FS = 18; LFS = 13; AXLW = 2.0;
    nA = size(arms, 1);  nR = 3;
    f = figure('Position', [60 60 560 * nA + 120, 400 * nR + 50], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(nR, nA, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax = gobjects(nR, nA);
    for ia = 1:nA
        fn_in = fullfile(rd, sprintf('f1p2_%s_PI%s.mat', plant, arms{ia, 1}));
        if ~exist(fn_in, 'file'); error('missing run %s', fn_in); end
        r = load(fn_in);  t = r.t;  seg = r.seg;  ea = r.a_true - r.a_hat;
        % ---- row 1: gain
        ax(1, ia) = nexttile(ia); hold on; box on;
        am = mean(r.a_hat, 2); as = std(r.a_hat, 0, 2);
        fill([t; flipud(t)], [am - as; flipud(am + as)], BANDC, 'FaceAlpha', 0.25, 'EdgeColor', 'none');
        h1 = plot(t, mean(r.a_true, 2), '-', 'Color', COL_TRUE, 'LineWidth', 2.0, 'DisplayName', 'a_{true} / a_{nom}');
        h2 = plot(t, am, '-', 'Color', COL_HAT, 'LineWidth', 1.6, 'DisplayName', 'a_{hat} / a_{nom}  (mean \pm \sigma_{seed})');
        h0 = plot(NaN, NaN, 'w.', 'DisplayName', sprintf('%s wall, plane init', upper(plant)));
        hL = plot(NaN, NaN, 'w.', 'DisplayName', arms{ia, 2});
        local_seg(seg, [0 1]);  xlim([0 t(end)]); ylim([0 1]);
        if ia == 1; ylabel('a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold'); end
        legend([h0 hL h1 h2], 'Location', 'northoutside', 'NumColumns', 2, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        local_ax(FS, AXLW);
        % ---- row 2: gain error, absolute
        ax(2, ia) = nexttile(nA + ia); hold on; box on;
        em = mean(ea, 2); es = std(ea, 0, 2);
        fill([t; flipud(t)], [em - es; flipud(em + es)], BANDC, 'FaceAlpha', 0.25, 'EdgeColor', 'none');
        h3 = plot(t, em, '-', 'Color', COL_HAT, 'LineWidth', 1.6, 'DisplayName', 'e_a = a_{true} - a_{hat}  (mean \pm \sigma_{seed})');
        yline(0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off');
        YE = [-0.1 0.1];  local_seg(seg, YE);  xlim([0 t(end)]); ylim(YE);  set(gca, 'YTick', -0.1:0.05:0.1);
        if ia == 1; ylabel('e_a  [a / a_{nom}]', 'FontSize', FS, 'FontWeight', 'bold'); end
        legend(h3, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        local_ax(FS, AXLW);
        % ---- row 3: the constant
        ax(3, ia) = nexttile(2 * nA + ia); hold on; box on;
        cm = mean(r.c_hat, 2); cs = std(r.c_hat, 0, 2); cp = mean(r.sP_c, 2);  yl = [-0.4 1.0];
        fill([t; flipud(t)], [cm - cs; flipud(cm + cs)], BANDC, 'FaceAlpha', 0.25, 'EdgeColor', 'none');
        h4 = plot(t, cm, '-', 'Color', COL_HAT, 'LineWidth', 1.6, 'DisplayName', 'c_{hat}  (mean \pm \sigma_{seed})');
        h5 = plot(t, cm + cp, '--', 'Color', COL_HAT, 'LineWidth', 1.0, 'DisplayName', '\pm sqrt(P)');
        plot(t, cm - cp, '--', 'Color', COL_HAT, 'LineWidth', 1.0);
        if isfield(r, 'c_eff_d') && isfield(r, 'wd')                      % 09-30: reference = c_eff(h_d(t)) (red, what the law needs here) and c* (descent average, weight Delta s^c)
            xtrue = r.c_eff_d(:);  xtrue(1) = NaN;  hd = max(r.wd(:), 1);  kd = t >= seg(1) & t <= seg(2);
            w = abs([0; diff(1 ./ hd)]);  w(~kd) = 0;  cstar = sum(w .* r.c_eff_d(:)) / max(sum(w), eps);
            h7 = plot(t, xtrue, '-', 'Color', COL_TRUE, 'LineWidth', 2.0, 'DisplayName', 'c_{eff}(h_d)');
            h6 = gobjects(0);  fprintf('  %s: c* (descent average of c_eff) = %+.3f\n', arms{ia, 1}, cstar);   % user 09-30: no c* line on the figure
            uistack(h4, 'top');
        else
            h7 = gobjects(0);
            h6 = yline(r.c_fit, '--', 'Color', [0.1 0.1 0.1], 'LineWidth', 2.2, 'DisplayName', sprintf('c band fit  %+.3f', r.c_fit));   % shorter than plot_plaw_walls' label: three columns, the legend must fit the column
        end
        local_seg(seg, yl);  xlim([0 t(end)]); ylim(yl);
        if ia == 1; ylabel('c', 'FontSize', FS, 'FontWeight', 'bold'); end
        xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        legend([h7 h4 h5 h6], 'Location', 'northoutside', 'NumColumns', 2, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        local_ax(FS, AXLW);
    end
    for rr = 1:nR; linkaxes(ax(rr, :), 'y'); end
    if ~strcmp(plant, 'cell'); ftag = [ftag '_' plant]; end
    fn = fullfile(od, sprintf('f1p2_cell_lf_compare%s.png', ftag));  exportgraphics(f, fn, 'Resolution', 150); close(f); fprintf('figure: %s\n', fn);
end

function local_seg(seg, yl)
    for s = seg; plot([s s], yl, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, 'HandleVisibility', 'off'); end
end

function local_ax(FS, AXLW)
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
end
