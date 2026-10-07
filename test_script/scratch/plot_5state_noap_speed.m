% PURPOSE (2026-10-02, user: "speed the oscillation up for a clear difference, also the cell; contrast with T_d = 5 s:
%   at high speed first order is not accurate enough, which is why the slope law (c-only) comes next"):
%   columns = slow ramp (22.2 -> 1.10 R in 5 s) | canonical oscillation 1 Hz | same band at 4 Hz (2 s of oscillation kept:
%   n_cycles = 2 f), rows = a_true / a_hat (mean +- sd) | e_a (mean +- sd); arms epsilon = 0 (grey) vs 1e-5 (blue); plane init.
%   Usage: plot_5state_noap_speed('plane') | plot_5state_noap_speed('cell').
%   Runs cached in test_results/temp_5state_noap/speed_<wall>_<scen>_<arm>_s<seed>.mat.   STATUS: SCRATCH
function plot_5state_noap_speed(wall, seeds, freqs)
    if nargin < 1; wall = 'plane'; end
    if nargin < 2 || isempty(seeds); seeds = 1:10; end
    if nargin < 3 || isempty(freqs); freqs = [1 4]; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));  addpath(fullfile(root, 'test_script', 'integration'));  addpath(here);
    id = fullfile(root, 'test_results', 'temp_5state_noap');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    pc = physical_constants();  a_nom = pc.Ts / pc.gamma_N;  R = pc.R;
    pcp = [];  if strcmp(wall, 'cell'); pcp = @cell_cperp_delta; end
    ARMS = {'eps', '\epsilon = Q_{44}/R_2 = 10^{-5}', [0 0.2 0.9], 1e-5};   % 2026-10-02: epsilon = 0 dropped from the figures (user)
    SC = [{{'ramp', 5}}, arrayfun(@(f) {{'osc', f}}, freqs)];  nC = numel(SC);
    nA = size(ARMS, 1);  D = cell(nC, nA);
    fprintf('\n%s wall\n', wall);
    for c = 1:nC
        sc = SC{c};  stag = strrep(sprintf('%s%g', sc{1}, sc{2}), '.', 'p');
        for m = 1:nA
            AH = [];  AT = [];
            for q = 1:numel(seeds)
                fn = fullfile(id, sprintf('speed_%s_%s_%s_s%d.mat', wall, stag, ARMS{m, 1}, seeds(q)));
                if exist(fn, 'file'); r = load(fn);
                else
                    cfg = canonical_scenario(0.05, 1.10, 'deep');  cfg.eq17_variant = '5state_noap';
                    if strcmp(sc{1}, 'ramp')
                        cfg.trajectory_type = 'ramp_descent';  cfg.h_init = 50;  cfg.h_bottom = 1.10 * R;  cfg.h_min = 1.10 * R;  cfg.amplitude = 0;
                        cfg.t_hold = 0.5;  cfg.t_descend_override = sc{2};  cfg.T_sim = 0.5 + sc{2} + 2.0;
                    else
                        cfg.frequency = sc{2};  cfg.n_cycles = 2 * sc{2};      % 2 s of oscillation at any frequency
                    end
                    cc = struct('lf_mode', 'const', 'lambda_f', 1, 'gain_order', 1, 'seed_mode', 'true_h0', 'q44_r2_frac', ARMS{m, 4});
                    o = struct('seed', seeds(q), 'collect_diag', true, 'ctrl_const_override', cc);
                    if ~isempty(pcp); o.plant_cperp = pcp; end
                    s = run_pure_simulation(cfg, o);
                    r = struct('t', s.tout(:), 'a_true', s.a_true_out(:, 3), 'a_hat', s.diag.a_hat(:, 3));
                    save(fn, '-struct', 'r');
                end
                AH(:, q) = r.a_hat / a_nom;  AT(:, q) = r.a_true / a_nom; %#ok<AGROW>
            end
            t = r.t;  if strcmp(sc{1}, 'ramp'); t = t - 0.5; mov = t >= 0 & t <= sc{2}; else; mov = t >= 0.5 & t <= 3.5; end
            D{c, m} = struct('t', t, 'AH', AH, 'AT', AT, 'sc', {sc});
            E = AT - AH;
            fprintf('  %-6s %-4s | motion: e_a mean %+.3f  rms %.3f  worst %+.3f  sd %.3f | end %+.3f\n', stag, ARMS{m, 1}, ...
                mean(E(mov, :), 'all'), sqrt(mean(E(mov, :).^2, 'all')), min(mean(E(mov, :), 2)), mean(std(E(mov, :), 0, 2)), mean(E(end, :)));
        end
    end
    COL_TRUE = [0.8 0 0];  FS = 16;  LFS = 11;  AXLW = 2.0;
    f = figure('Position', [40 40 max(1300, 700 * nC) 1000], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(2, nC, 'TileSpacing', 'compact', 'Padding', 'compact');
    for c = 1:nC
        sc = SC{c};  t = D{c, 1}.t;
        if strcmp(sc{1}, 'ramp'); desc = sprintf('%s, ramp 22.2 -> 1.10 R in %g s', wall, sc{2}); segs = [0 sc{2}];
        else; desc = sprintf('%s, oscillation %g Hz, band 1.10-3.32 R', wall, sc{2}); segs = [0.5 1.5 3.5]; end
        nexttile(c); hold on; box on;  hl = plot(NaN, NaN, 'w.', 'DisplayName', [desc sprintf(', %d seeds', numel(seeds))]);
        for m = 1:nA
            am = mean(D{c, m}.AH, 2);  as = std(D{c, m}.AH, 0, 2);
            fill([t; flipud(t)], [am + as; flipud(am - as)], ARMS{m, 3}, 'FaceAlpha', 0.18, 'EdgeColor', 'none', 'HandleVisibility', 'off');
            hl(end + 1) = plot(t, am, '-', 'Color', ARMS{m, 3}, 'LineWidth', 1.6, 'DisplayName', ['a_{hat}, ' ARMS{m, 2}]); %#ok<AGROW>
        end
        ht = plot(t, mean(D{c, 1}.AT, 2), '-', 'Color', COL_TRUE, 'LineWidth', 2.0, 'DisplayName', 'a_{true}');
        ylim([0 1.1]);  local_seg(segs);  xlim([t(1) t(end)]);
        if c == 1; ylabel('a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold'); end
        legend([hl(1) ht hl(2:end)], 'Location', 'northoutside', 'Orientation', 'horizontal', 'NumColumns', 2, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');
        nexttile(nC + c); hold on; box on;
        for m = 1:nA
            E = D{c, m}.AT - D{c, m}.AH;  em = mean(E, 2);  es = std(E, 0, 2);
            fill([t; flipud(t)], [em + es; flipud(em - es)], ARMS{m, 3}, 'FaceAlpha', 0.18, 'EdgeColor', 'none', 'HandleVisibility', 'off');
            plot(t, em, '-', 'Color', ARMS{m, 3}, 'LineWidth', 1.6);
        end
        yline(0, '-', 'Color', [0.6 0.6 0.6], 'HandleVisibility', 'off');  ylim([-0.6 0.4]);  local_seg(segs);  xlim([t(1) t(end)]);
        if c == 1; ylabel('e_a = a_{true} - a_{hat}', 'FontSize', FS, 'FontWeight', 'bold'); end
        xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');
    end
    out = fullfile(fd, sprintf('fig_noap_speed_%s.png', wall));  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end

function local_seg(segs)
    yl = ylim;  for x = segs; plot([x x], yl, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2, 'HandleVisibility', 'off'); end;  ylim(yl);
end
