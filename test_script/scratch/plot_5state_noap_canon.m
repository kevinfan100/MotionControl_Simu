% PURPOSE (2026-10-02, user: "test the oscillating trajectory"): canonical deep scenario (hold 0.5 s -> descend 1 s ->
%   2 cycles at 1 Hz in the band 1.10-3.32 R -> hold to 4.8 s), plane, 10 seeds, first order, epsilon = Q44/R2 = 0 (grey)
%   vs 1e-5 (blue). House style: a_true / a_hat (mean +- sd) | e_a (mean +- sd); one file with a_m of seed 1 (_am).
%   Runs cached in test_results/temp_5state_noap/canon_<wall>_<arm>_s<seed>.mat.   STATUS: SCRATCH
function plot_5state_noap_canon(wall, seeds)
    if nargin < 1; wall = 'plane'; end
    if nargin < 2; seeds = 1:10; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));  addpath(fullfile(root, 'test_script', 'integration'));  addpath(here);
    id = fullfile(root, 'test_results', 'temp_5state_noap');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    pc = physical_constants();  a_nom = pc.Ts / pc.gamma_N;
    pcp = [];  if strcmp(wall, 'cell'); pcp = @cell_cperp_delta; end
    ARMS = {'eps', '\epsilon = Q_{44}/R_2 = 10^{-5}', [0 0.2 0.9], 1e-5};   % 2026-10-02: epsilon = 0 dropped from the figure (user)
    nA = size(ARMS, 1);  D = cell(1, nA);
    for m = 1:nA
        AH = [];  AT = [];  am1 = [];
        for q = 1:numel(seeds)
            fn = fullfile(id, sprintf('canon_%s_%s_s%d.mat', wall, ARMS{m, 1}, seeds(q)));
            if exist(fn, 'file'); r = load(fn);
            else
                cfg = canonical_scenario(0.05, 1.10, 'deep');  cfg.eq17_variant = '5state_noap';
                cc = struct('lf_mode', 'const', 'lambda_f', 1, 'gain_order', 1, 'seed_mode', 'true_h0', 'q44_r2_frac', ARMS{m, 4});
                o = struct('seed', seeds(q), 'collect_diag', true, 'ctrl_const_override', cc);
                if ~isempty(pcp); o.plant_cperp = pcp; end
                s = run_pure_simulation(cfg, o);
                r = struct('t', s.tout(:), 'a_true', s.a_true_out(:, 3), 'a_hat', s.diag.a_hat(:, 3), 'a_m', s.diag.a_xm(:, 3), 'h_d', s.p_d_out(:, 3) / pc.R);
                save(fn, '-struct', 'r');
            end
            AH(:, q) = r.a_hat / a_nom;  AT(:, q) = r.a_true / a_nom; %#ok<AGROW>
            if q == 1; am1 = r.a_m / a_nom; hd = r.h_d; end
        end
        D{m} = struct('t', r.t, 'AH', AH, 'AT', AT, 'am1', am1, 'hd', hd);
        t = r.t;  E = AT - AH;
        for w = {[0.5 1.5], [1.5 3.5], [3.5 4.8]}
            k = t >= w{1}(1) & t < w{1}(2);
            fprintf('%-6s %-4s t %.1f-%.1f | e_a mean %+.3f  rms %.3f  worst %+.3f | sd over seeds %.3f\n', wall, ARMS{m, 1}, w{1}(1), w{1}(2), ...
                mean(E(k, :), 'all'), sqrt(mean(E(k, :).^2, 'all')), min(mean(E(k, :), 2)), mean(std(E(k, :), 0, 2)));
        end
    end
    COL_TRUE = [0.8 0 0];  C_MEAS = [0.55 0.75 1.0];  FS = 18;  LFS = 12;  AXLW = 2.0;
    for with_am = [false true]
        f = figure('Position', [40 40 1300 1000], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
        tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
        t = D{1}.t;  segs = [0.5 1.5 3.5];
        ax1 = nexttile(1); hold on; box on;
        h0 = plot(NaN, NaN, 'w.', 'DisplayName', sprintf('%s wall, plane init, canonical deep (band 1.10-3.32 R), %d seeds', wall, numel(seeds)));  hl = h0;
        if with_am; hm = plot(t, D{1}.am1, '-', 'Color', C_MEAS, 'LineWidth', 0.6, 'DisplayName', 'a_m (seed 1)'); end
        for m = 1:nA
            am = mean(D{m}.AH, 2);  as = std(D{m}.AH, 0, 2);
            fill([t; flipud(t)], [am + as; flipud(am - as)], ARMS{m, 3}, 'FaceAlpha', 0.18, 'EdgeColor', 'none', 'HandleVisibility', 'off');
            hl(end + 1) = plot(t, am, '-', 'Color', ARMS{m, 3}, 'LineWidth', 1.8, 'DisplayName', ['a_{hat}, ' ARMS{m, 2}]); %#ok<AGROW>
        end
        ht = plot(t, mean(D{1}.AT, 2), '-', 'Color', COL_TRUE, 'LineWidth', 2.2, 'DisplayName', 'a_{true}');
        hl = [hl(1) ht hl(2:end)];  if with_am; hl(end + 1) = hm; ylim([-0.2 1.6]); else; ylim([0 1.1]); end
        ylabel('a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold');
        legend(hl, 'Location', 'northoutside', 'Orientation', 'horizontal', 'NumColumns', 2, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        ax2 = nexttile(2); hold on; box on;
        for m = 1:nA
            E = D{m}.AT - D{m}.AH;  em = mean(E, 2);  es = std(E, 0, 2);
            fill([t; flipud(t)], [em + es; flipud(em - es)], ARMS{m, 3}, 'FaceAlpha', 0.18, 'EdgeColor', 'none', 'HandleVisibility', 'off');
            plot(t, em, '-', 'Color', ARMS{m, 3}, 'LineWidth', 1.8);
        end
        yline(0, '-', 'Color', [0.6 0.6 0.6], 'HandleVisibility', 'off');  ylim([-0.6 0.3]);
        ylabel('e_a = a_{true} - a_{hat}', 'FontSize', FS, 'FontWeight', 'bold');  xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        for ax = [ax1 ax2]
            xlim(ax, [0 t(end)]);  yl = ylim(ax);
            for x = segs; plot(ax, [x x], yl, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2, 'HandleVisibility', 'off'); end
            set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');  grid(ax, 'off');
        end
        out = fullfile(fd, sprintf('fig_noap_canon_%s%s.png', wall, repmat('_am', 1, with_am)));
        exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);  close(f);
    end
end
