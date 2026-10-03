% PURPOSE (2026-09-21): figure of run_plaw_walls.m (advisor power law, version A, plane init), house style
%   (plot_var_ahat_6state.m: True red, Estimate blue, +-sigma_seed band, legend northoutside, no grid / title):
%     columns  cell | sphere by default (plants argument; plane dropped, user 09-21), same init (plane), wall only in the plant;
%              the f2 law defaults to plane | cell | sphere (wall-test protocol rule 5: the plane's cost is reported alongside)
%     row 1    a_true (red, seed mean) and a_hat (blue, mean +- sigma_seed)
%     row 2    e_a = a_true - a_hat, absolute, mean +- sigma_seed
%     row 3    c_hat (mean +- sigma_seed, +- sqrt(P) dashed) and the band-fit c (grey dotted; c_eff line dropped, user 09-21)
%     row 4    (f2 only) delta_hat, same style, band-fit delta dotted   (2026-09-22)
%   Output reference/eq17_analysis/derivation/figures/<law>_walls_<arm><suffix>.png | EXPIRES: with the advisor-law test
function plot_plaw_walls(arm, suffix, plants, law, ftag)
    if nargin < 5; ftag = ''; end                                % appended to the figure file name (e.g. '_spheretails' for a plant-variant comparison)
    if nargin < 4 || isempty(law); law = 'plaw'; end            % 'plaw' | 'p11d1' (file prefix p11d1_, c label c_0) | 'v2' | 'v2p' | 'f2' | 'f2p'
    if nargin < 1 || isempty(arm); arm = 'PI'; end
    if nargin < 2; suffix = ''; end
    isf1 = startsWith(law, 'f1');                                % f1 (09-23 evening): the advisor's (1-c)/h^p with c AND p states; row 3 = c, row 4 = p
    isf2 = startsWith(law, 'f2') || startsWith(law, 'f4') || isf1;   % f4 (0923): same layout, slot 5 = p instead of c
    if nargin < 3 || isempty(plants)
        plants = {'cell', 'sphere'};                                     % user 09-21: cell and sphere only (plane column dropped)
        if isf2; plants = {'plane', 'cell', 'sphere'}; end
        if isf1; plants = {'plane', 'cell'}; end                        % the (c, p) line runs plane and cell (user 09-23)
    end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    rd = fullfile(root, 'test_results', 'temp_plaw_walls');  od = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    COL_TRUE = [0.8 0 0]; COL_HAT = [0 0.2 0.9]; BANDC = [0.45 0.55 0.95]; FS = 18; LFS = 13; AXLW = 2.0; CG = [0.5 0.5 0.5];
    PL = plants;
    PFX = '';  CL = 'c';  if strcmp(law, 'p11d1'); PFX = 'p11d1_';  CL = 'c_0'; end
    if startsWith(law, 'v2') || isf2; PFX = [law '_'];  CL = 'c'; end
    if startsWith(law, 'f4'); CL = 'p'; end
    files = cellfun(@(p) fullfile(rd, sprintf('%s%s_%s%s.mat', PFX, p, arm, suffix)), PL, 'UniformOutput', false);
    keep = cellfun(@(f) exist(f, 'file') == 2, files);  PL = PL(keep);  files = files(keep);  nP = numel(PL);
    if nP == 0; error('no files for arm %s%s', arm, suffix); end
    lab_init = 'plane init';  if strcmp(arm, 'T'); lab_init = 'seed = band fit'; end
    has4 = isf2 && ~startsWith(law, 'f1p2');                     % row 4 (delta / p) only when that slot is a free constant; 'f1p2' pins p -> no p row (user 09-23)
    nR = 3 + double(has4);
    f = figure('Position', [60 60 560 * nP + 120, 400 * nR + 50], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(nR, nP, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax = gobjects(nR, nP);
    for ip = 1:nP
        r = load(files{ip});  t = r.t;  seg = r.seg;  ea = r.a_true - r.a_hat;
        ax(1, ip) = nexttile(ip); hold on; box on;
        am = mean(r.a_hat, 2); as = std(r.a_hat, 0, 2);
        fill([t; flipud(t)], [am - as; flipud(am + as)], BANDC, 'FaceAlpha', 0.25, 'EdgeColor', 'none');
        hm = gobjects(0);
        if isfield(r, 'a_m')                                              % 09-29 user: the measured gain a_m (y2 reading) on the gain row only
            hm = plot(t, mean(r.a_m, 2), '-', 'Color', [0.35 0.7 0.95], 'LineWidth', 0.9, 'DisplayName', 'a_m / a_{nom}  (measured, mean)');
        end
        h1 = plot(t, mean(r.a_true, 2), '-', 'Color', COL_TRUE, 'LineWidth', 2.0, 'DisplayName', 'a_{true} / a_{nom}');
        h2 = plot(t, am, '-', 'Color', COL_HAT, 'LineWidth', 1.6, 'DisplayName', 'a_{hat} / a_{nom}  (mean \pm \sigma_{seed})');
        wall_lab = upper(PL{ip});  if strcmp(PL{ip}, 'sphere_pt'); wall_lab = 'SPHERE, plane tail > 4.5 R'; elseif strcmp(PL{ip}, 'sphere'); wall_lab = 'SPHERE, [1/1] tail > 4.5 R'; end
        h0 = plot(NaN, NaN, 'w.', 'DisplayName', sprintf('%s wall, %s', wall_lab, lab_init));
        local_seg(seg, [0 1]);  xlim([0 t(end)]); ylim([0 1]);
        if ip == 1; ylabel('a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold'); end
        if isempty(hm)
            legend([h0 h1 h2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        else
            legend([h0 h1 h2 hm], 'Location', 'northoutside', 'NumColumns', 2, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        end
        local_ax(FS, AXLW);
        ax(2, ip) = nexttile(nP + ip); hold on; box on;
        em = mean(ea, 2); es = std(ea, 0, 2);
        fill([t; flipud(t)], [em - es; flipud(em + es)], BANDC, 'FaceAlpha', 0.25, 'EdgeColor', 'none');
        h3 = plot(t, em, '-', 'Color', COL_HAT, 'LineWidth', 1.6, 'DisplayName', 'e_a = a_{true} - a_{hat}  (mean \pm \sigma_{seed})');
        yline(0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off');
        YE = [-0.25 0.25];  if isf2; YE = [-0.1 0.1]; end                % f2 (user 09-22): finer e_a axis
        local_seg(seg, YE);  xlim([0 t(end)]); ylim(YE);
        if isf2; set(gca, 'YTick', -0.1:0.05:0.1); end                   % 5 ticks (user 09-22: fewer ticks)
        if ip == 1; ylabel('e_a  [a / a_{nom}]', 'FontSize', FS, 'FontWeight', 'bold'); end
        legend(h3, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        local_ax(FS, AXLW);
        ax(3, ip) = nexttile(2 * nP + ip); hold on; box on;
        yl = [-0.6 1.2];  if isf2; yl = [-0.4 0.3]; end;  if startsWith(law, 'f4'); yl = [1.5 2.5]; end;  if isf1; yl = [-0.4 1.0]; end
        xref = r.c_fit;  lab_ref = sprintf('%s band fit of this wall  %+.3f', CL, r.c_fit);  xtrue = [];
        if isf1 && isfield(r, 'c_eff_d') && isfield(r, 'wd')             % 09-30 user: row 3 compares c_hat with c_eff(h_d(t)) (what the law needs at this height) and c* (the one constant that zeroes e_a at the wall)
            xtrue = r.c_eff_d(:);  hd = max(r.wd(:), 1);  kd = t >= seg(1) & t <= seg(2);
            w = abs([0; diff(1 ./ hd)]);  w(~kd) = 0;  xref = sum(w .* xtrue) / max(sum(w), eps);   % c* = sum c_eff * Delta s^c / sum Delta s^c over the descent
            lab_ref = sprintf('c*  (descent average of c_{eff})  %+.3f', xref);  fprintf('  %s: c* (descent average of c_eff, weight Delta s^c) = %+.3f\n', PL{ip}, xref);
            xtrue(1) = NaN;                                                 % first sample sits at w_d = 0 before the trajectory starts
        end
        local_const_row(t, r.c_hat, r.sP_c, xref, seg, yl, CL, ip == 1, ~has4, COL_HAT, BANDC, CG, FS, LFS, AXLW, xtrue, lab_ref, COL_TRUE);
        if has4
            ax(4, ip) = nexttile(3 * nP + ip); hold on; box on;
            lab7 = '\delta';  yl7 = [-1 1.8];  if isf1; lab7 = 'p';  yl7 = [1.2 2.6]; end
            local_const_row(t, r.d_hat, r.sP_d, r.d_fit, seg, yl7, lab7, ip == 1, true, COL_HAT, BANDC, CG, FS, LFS, AXLW);
        end
    end
    for rr = 1:nR; linkaxes(ax(rr, :), 'y'); end
    fn = fullfile(od, sprintf('%swalls_%s%s%s.png', PFX, arm, suffix, ftag));  if isempty(PFX); fn = fullfile(od, sprintf('plaw_walls_%s%s%s.png', arm, suffix, ftag)); end
    exportgraphics(f, fn, 'Resolution', 150); close(f); fprintf('figure: %s\n', fn);
end

function local_const_row(t, x, sp, xfit, seg, yl, lab, first_col, with_xlabel, COL_HAT, BANDC, CG, FS, LFS, AXLW, xtrue, lab_ref, COL_TRUE)
    if nargin < 16; xtrue = []; end
    if nargin < 17 || isempty(lab_ref); lab_ref = sprintf('%s band fit of this wall  %+.3f', lab, xfit); end
    if nargin < 18; COL_TRUE = [0.8 0 0]; end
    cm = mean(x, 2); cs = std(x, 0, 2); cp = mean(sp, 2);
    fill([t; flipud(t)], [cm - cs; flipud(cm + cs)], BANDC, 'FaceAlpha', 0.25, 'EdgeColor', 'none');
    hh = gobjects(0);
    if ~isempty(xtrue)                                                     % 09-30: the local truth c_eff(h_d(t)), red like a_true in row 1
        hh = plot(t, xtrue, '-', 'Color', COL_TRUE, 'LineWidth', 2.0, 'DisplayName', 'c_{eff}(h_d) = 1 - h_d^2 a''_{true}(h_d)');
    end
    h4 = plot(t, cm, '-', 'Color', COL_HAT, 'LineWidth', 1.6, 'DisplayName', [lab '_{hat}  (mean \pm \sigma_{seed})']);
    h5 = plot(t, cm + cp, '--', 'Color', COL_HAT, 'LineWidth', 1.0, 'DisplayName', '\pm sqrt(P)');
    plot(t, cm - cp, '--', 'Color', COL_HAT, 'LineWidth', 1.0);
    if isempty(xtrue)
        h6 = yline(xfit, '--', 'Color', [0.1 0.1 0.1], 'LineWidth', 2.2, 'DisplayName', lab_ref);   % user 09-22: the reference value must stand out (CG kept in the signature for the callers)
    end
    local_seg(seg, yl);  xlim([0 t(end)]); ylim(yl);
    if first_col; ylabel(lab, 'FontSize', FS, 'FontWeight', 'bold'); end
    if with_xlabel; xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold'); end
    if isempty(hh)
        legend([h4 h5 h6], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    else                                                                   % user 09-30: with the c_eff curve the c* line is dropped (c* printed to console only)
        legend([hh h4 h5], 'Location', 'northoutside', 'NumColumns', 2, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    end
    local_ax(FS, AXLW);
end

function local_seg(seg, yl)
    for s = seg; plot([s s], yl, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, 'HandleVisibility', 'off'); end
end

function local_ax(FS, AXLW)
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
end
