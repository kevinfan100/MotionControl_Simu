% PURPOSE (2026-09-21, house style after plot_delta_compact.m): figures of the no-a' 5-state battery (run_5state_noap_battery.m).
%   fig_noap_fixed.png    3 columns (22.2 / 2.0 / 1.10 R, far-field init) x 3 rows:
%                         a_true / a_hat (mean +- sigma_seed) | e_a = a_true - a_hat (mean +- sigma_seed, +- sqrt(P44) dashed)
%                         | rate: delta a_hat (mean +- sigma_seed, +- sqrt(P55) dashed) with the true per-step increment red
%   fig_noap_canon.png    same 3 rows for the canonical deep trajectory (cheat init), segment lines hold | descent | osc | hold
%   fig_noap_meng.png     same for the slow ramp
%   e_a sign is the house sign (a_true - a_hat). Copies go to reference/eq17_analysis/derivation/figures/ for the tex.
%  STATUS: ACTIVE (moved from scratch 2026-09-22; 0920_5state_noaprime.tex depends on it)
function plot_5state_noap_battery(tag_suffix, plant_label)
    if nargin < 1; tag_suffix = ''; end
    if nargin < 2; plant_label = ''; end      % e.g. 'cell wall (plane slid 1.05 R), ' prepended to every description
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    id = fullfile(root, 'test_results', 'temp_5state_noap');
    fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    % ---- fixed heights: 3 columns ----
    cells = {'fixed_h22p2', 'fixed_h2p00', 'fixed_h1p10'};  labels = {'22.2 R', '2.0 R', '1.10 R'};
    f = figure('Position', [80 80 2400 1000], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(3, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    for c = 1:3
        r = load(fullfile(id, [cells{c} tag_suffix '.mat']));
        if isfield(r, 'ctrl_const') && isfield(r.ctrl_const, 'seed_mode') && strcmp(r.ctrl_const.seed_mode, 'anom')
            desc = sprintf('%shold %s, init a_{nom} (far field), prior 0.5 a_{nom}', plant_label, labels{c});
        else
            desc = sprintf('%shold %s, init a(h_0), prior 0.05 a_{nom}', plant_label, labels{c});
        end
        local_three_rows(r, c, 3, desc, c == 1, [0 1.1], [-0.3 0.3], [-1.5 1.5]);
    end
    local_save(f, id, fd, ['fig_noap_fixed' tag_suffix '.png']);
    % ---- trajectories: one column each ----
    specs = {'canon', 'canonical deep (hold | descent | 2 x 1 Hz | hold), init a(h_0)', [0 1.1], [-0.45 0.45], [-3 3]; ...
             'meng',  'slow ramp 15 um -> 2.5 um over 10 s, init a(h_0)',                [0 1.1], [-0.25 0.25], [-0.6 0.6]};
    for k = 1:size(specs, 1)
        r = load(fullfile(id, [specs{k, 1} tag_suffix '.mat']));
        f = figure('Position', [80 80 1300 1000], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
        tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
        local_three_rows(r, 1, 1, [plant_label specs{k, 2}], true, specs{k, 3}, specs{k, 4}, specs{k, 5});
        local_save(f, id, fd, ['fig_noap_' specs{k, 1} tag_suffix '.png']);
    end
end

function local_three_rows(r, col, nC, desc, first_col, yl_a, yl_e, yl_r)
    COL_TRUE = [0.8 0 0]; COL_HAT = [0 0.2 0.9]; BANDC = [0.45 0.55 0.95]; FS = 18; LFS = 13; AXLW = 2.0;   % house style
    t = r.t;  seg = r.seg(r.seg > 0);
    % row 1: gain
    nexttile(col); hold on; box on;
    am = mean(r.a_hat, 2); as = std(r.a_hat, 0, 2);
    fill([t; flipud(t)], [am - as; flipud(am + as)], BANDC, 'FaceAlpha', 0.25, 'EdgeColor', 'none');
    h1 = plot(t, mean(r.a_true, 2), '-', 'Color', COL_TRUE, 'LineWidth', 2.0, 'DisplayName', 'a_{true} / a_{nom}');
    h2 = plot(t, am, '-', 'Color', COL_HAT, 'LineWidth', 1.6, 'DisplayName', 'a_{hat} / a_{nom}  (mean \pm \sigma_{seed})');
    for s = seg; xline(s, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, 'HandleVisibility', 'off'); end
    h0 = plot(NaN, NaN, 'w.', 'DisplayName', desc);
    xlim([0 t(end)]); ylim(yl_a);
    if first_col; ylabel('a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold'); end
    legend([h0 h1 h2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
    % row 2: error with sqrt(P44)
    nexttile(nC + col); hold on; box on;
    ea = r.a_true - r.a_hat;  em = mean(ea, 2); es = std(ea, 0, 2); sp = mean(sqrt(r.P_a), 2);
    fill([t; flipud(t)], [em - es; flipud(em + es)], BANDC, 'FaceAlpha', 0.25, 'EdgeColor', 'none');
    h3 = plot(t, em, '-', 'Color', COL_HAT, 'LineWidth', 1.6, 'DisplayName', 'e_a = a_{true} - a_{hat}  (mean \pm \sigma_{seed})');
    h4 = plot(t, sp, '--', 'Color', COL_HAT, 'LineWidth', 1.0, 'DisplayName', '\pm sqrt(P_{44})');
    plot(t, -sp, '--', 'Color', COL_HAT, 'LineWidth', 1.0);
    yline(0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off');
    for s = seg; xline(s, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, 'HandleVisibility', 'off'); end
    xlim([0 t(end)]); ylim(yl_e);
    if first_col; ylabel('e_a  [a / a_{nom}]', 'FontSize', FS, 'FontWeight', 'bold'); end
    legend([h3 h4], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
    % row 3: rate with sqrt(P55), true increment (0.25 s mean) red
    nexttile(2 * nC + col); hold on; box on;
    dm = mean(r.da_hat, 2) * 1e3; ds = std(r.da_hat, 0, 2) * 1e3; dp = mean(sqrt(r.P_da), 2) * 1e3;
    fill([t; flipud(t)], [dm - ds; flipud(dm + ds)], BANDC, 'FaceAlpha', 0.25, 'EdgeColor', 'none');
    h6 = plot(t, movmean(mean(r.da_true, 2), 401) * 1e3, '-', 'Color', COL_TRUE, 'LineWidth', 2.0, 'DisplayName', 'true increment (0.25 s mean)');
    h5 = plot(t, dm, '-', 'Color', COL_HAT, 'LineWidth', 1.6, 'DisplayName', '\delta a_{hat}  (mean \pm \sigma_{seed})');
    h7 = plot(t, dm + dp, '--', 'Color', COL_HAT, 'LineWidth', 1.0, 'DisplayName', '\pm sqrt(P_{55})');
    plot(t, dm - dp, '--', 'Color', COL_HAT, 'LineWidth', 1.0);
    yline(0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off');
    for s = seg; xline(s, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, 'HandleVisibility', 'off'); end
    tr_s = movmean(mean(r.da_true, 2), 401) * 1e3;  m_r = t > 0.3;    % skip the init transient when sizing the axis
    yr = 1.3 * max([max(abs(tr_s(m_r))), max(abs(dm(m_r)) + ds(m_r)), 0.05]);
    xlim([0 t(end)]); ylim([-yr yr]);  %#ok<NASGU> yl_r kept in the signature for callers
    if first_col; ylabel('\delta a / a_{nom}  [10^{-3} / step]', 'FontSize', FS, 'FontWeight', 'bold'); end
    xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    legend([h5 h7 h6], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
end

function local_save(f, id, fd, name)
    exportgraphics(f, fullfile(id, name), 'Resolution', 150);
    if exist(fd, 'dir'); copyfile(fullfile(id, name), fullfile(fd, name)); end
    close(f);  fprintf('figure: %s\n', fullfile(id, name));
end
