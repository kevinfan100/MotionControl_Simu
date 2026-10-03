% PURPOSE (2026-09-21, user: "the three-height positioning test, cell and sphere only"): the advisor power law (version A,
%   p fixed 1.9666, c estimated from the plane init 0) held at 22.2 / 2.0 / 1.10 R for 3 s (run_plaw_walls.m, kind 'fixed'),
%   one figure per plant, columns = heights, rows: a_bar true/hat (mean +- sigma_seed), e_a = a_true - a_hat (absolute),
%   c_hat (mean +- sigma_seed, +- sqrt(P) dashed, band-fit c grey dotted). House style (plot_var_ahat_6state.m).
%   Output reference/eq17_analysis/derivation/figures/fixed_plaw_<plant>_<arm>.png | EXPIRES: with the advisor-law test
function plot_plaw_fixed3(arm, plants, law)
    if nargin < 3 || isempty(law); law = 'plaw'; end            % 'plaw' | 'p11d1'
    if nargin < 1 || isempty(arm); arm = 'PI'; end
    if nargin < 2 || isempty(plants); plants = {'cell', 'sphere'}; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    rd = fullfile(root, 'test_results', 'temp_plaw_walls');  od = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    COL_TRUE = [0.8 0 0]; COL_HAT = [0 0.2 0.9]; BANDC = [0.45 0.55 0.95]; FS = 18; LFS = 13; AXLW = 2.0; CG = [0.5 0.5 0.5];
    H = {'22p20', '2p00', '1p10'};  HL = {'22.2 R', '2.0 R', '1.10 R'};
    lab_init = 'plane init';  if strcmp(arm, 'T'); lab_init = 'seed = band fit'; end
    for pl = plants
        PFX = '';  CL = 'c';  if strcmp(law, 'p11d1'); PFX = 'p11d1_';  CL = 'c_0'; end
    if startsWith(law, 'v2'); PFX = [law '_'];  CL = 'c'; end
        files = cellfun(@(h) fullfile(rd, sprintf('%sfixed_%s_%s_h%s.mat', PFX, pl{1}, arm, h)), H, 'UniformOutput', false);
        keep = cellfun(@(f) exist(f, 'file') == 2, files);
        if ~any(keep); continue; end
        R = cellfun(@load, files(keep), 'UniformOutput', false);  names = HL(keep);  nH = numel(R);
        f = figure('Position', [40 40 480 * nH + 100, 950], 'Color', 'w', 'Visible', 'off');
        tiledlayout(3, nH, 'TileSpacing', 'compact', 'Padding', 'compact');
        for v = 1:nH
            r = R{v};  t = r.t;  ea = r.a_true - r.a_hat;  T_END = t(end);
            nexttile(v); hold on; box on;
            am = mean(r.a_hat, 2); as = std(r.a_hat, 0, 2);  at = mean(r.a_true, 2);
            fill([t; flipud(t)], [am - as; flipud(am + as)], BANDC, 'FaceAlpha', 0.25, 'EdgeColor', 'none');
            h1 = plot(t, at, '-', 'Color', COL_TRUE, 'LineWidth', 2.0, 'DisplayName', 'a_{true} / a_{nom}');
            h2 = plot(t, am, '-', 'Color', COL_HAT, 'LineWidth', 1.6, 'DisplayName', 'a_{hat} / a_{nom}');
            h0 = plot(NaN, NaN, 'w.', 'DisplayName', sprintf('%s, hold %s, %s', upper(pl{1}), names{v}, lab_init));
            xlim([0 T_END]); ylim([0 1]);
            if v == 1; ylabel('a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold'); end
            legend([h0 h1 h2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
            set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
            nexttile(nH + v); hold on; box on;
            em = mean(ea, 2); es = std(ea, 0, 2);
            fill([t; flipud(t)], [em - es; flipud(em + es)], BANDC, 'FaceAlpha', 0.25, 'EdgeColor', 'none');
            h3 = plot(t, em, '-', 'Color', COL_HAT, 'LineWidth', 1.6, 'DisplayName', 'e_a = a_{true} - a_{hat}  (mean \pm \sigma_{seed})');
            yline(0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off');
            xlim([0 T_END]); ylim([-0.25 0.25]);
            if v == 1; ylabel('e_a  [a / a_{nom}]', 'FontSize', FS, 'FontWeight', 'bold'); end
            legend(h3, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
            set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
            nexttile(2 * nH + v); hold on; box on;
            cm = mean(r.c_hat, 2); cs = std(r.c_hat, 0, 2); cp = mean(r.sP_c, 2);
            fill([t; flipud(t)], [cm - cs; flipud(cm + cs)], BANDC, 'FaceAlpha', 0.25, 'EdgeColor', 'none');
            h4 = plot(t, cm, '-', 'Color', COL_HAT, 'LineWidth', 1.6, 'DisplayName', [CL '_{hat}  (mean \pm \sigma_{seed})']);
            h5 = plot(t, cm + cp, '--', 'Color', COL_HAT, 'LineWidth', 1.0, 'DisplayName', '\pm sqrt(P)');
            plot(t, cm - cp, '--', 'Color', COL_HAT, 'LineWidth', 1.0);
            h6 = yline(r.c_fit, ':', 'Color', CG, 'LineWidth', 1.4, 'DisplayName', [CL ' band fit']);
            xlim([0 T_END]); ylim([-0.6 1.2]);
            if v == 1; ylabel(CL, 'FontSize', FS, 'FontWeight', 'bold'); end
            xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
            legend([h4 h5 h6], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
            set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
        end
        lawtag = 'plaw';  if ~isempty(PFX); lawtag = PFX(1:end-1); end       % plaw | p11d1 | v2 | v2p
        fn = fullfile(od, sprintf('fixed_%s_%s_%s.png', lawtag, pl{1}, arm));
        exportgraphics(f, fn, 'Resolution', 150); close(f); fprintf('figure: %s\n', fn);
    end
end
