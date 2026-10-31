% PURPOSE (2026-10-07, branch test/conly-meng-law): stage-4 paired comparison of the journal control law (control_law 'j6')
%   against the current one ('eq17') on ramp_tri 1 Hz, plane init, three walls, 10 paired seeds (run_plaw_walls tags
%   _eq17 / _j6). Pre-registered readings (canonical-trajectory.md): e_a at the valleys 1 / 2 / 3 s, the tops 1.5 / 2.5 s,
%   mean |e_a| over the triangle 1-3 s; plus std of delta_x_m over the triangle. Paired difference j6 - eq17, mean +- SEM
%   and t = mean/SEM (|t| < 2 = no difference beyond seed scatter). Figures: one per wall, rows a-bar | e_a | delta_x_m | c,
%   columns eq17 | j6, seed 1, 10-05 single-seed style (feedback_single_seed_figure_style_2026-10-05).
function out = plot_conly_meng_walls(opts)
    if nargin < 1; opts = struct(); end
    if ~isfield(opts, 'walls'); opts.walls = {'plane', 'cell', 'sphere'}; end
    if ~isfield(opts, 'laws');  opts.laws  = {'eq17', 'j6'}; end
    if ~isfield(opts, 'seed_fig'); opts.seed_fig = 1; end
    if ~isfield(opts, 'tag'); opts.tag = ''; end                 % run tag between PI_ and the law, e.g. 'pfix015_' (10-07: the P_fix arm)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    id = fullfile(root, 'test_results', 'temp_plaw_walls');  od = fullfile(root, 'test_results', 'temp_conly_meng');
    abar = char(257);  hat = char(770);
    T_VAL = [1, 2, 3];  T_TOP = [1.5, 2.5];  T_TRI = [1, 3];
    nW = numel(opts.walls);
    out = struct();
    fprintf('\nTHREE WALLS  ramp_tri 1 Hz, plane init, 10 paired seeds, tag ''%s''; e_a = a_true - a_hat (absolute, a/a_nom units)\n', opts.tag);
    fprintf('%-8s %-14s %12s %12s %16s %7s\n', 'wall', 'reading', opts.laws{1}, opts.laws{2}, 'j6 - eq17 (+-SEM)', 't');
    for iw = 1:nW
        w = opts.walls{iw};  R = cell(1, 2);
        for il = 1:2
            fn = fullfile(id, sprintf('f1p2_%s_PI_%s%s_rtri_f1.mat', w, opts.tag, opts.laws{il}));
            assert(exist(fn, 'file') == 2, 'missing %s', fn);
            R{il} = load(fn);
        end
        t = R{1}.t;  ns = size(R{1}.a_hat, 2);
        iv = arrayfun(@(tt) find(t >= tt, 1), T_VAL);  it = arrayfun(@(tt) find(t >= tt, 1), T_TOP);
        tri = t >= T_TRI(1) & t <= T_TRI(2);
        vals = zeros(2, ns, 7);                                   % law x seed x [v1 v2 v3 top1 top2 tri|e| dxm_std_nm]
        for il = 1:2
            ea = R{il}.a_true - R{il}.a_hat;
            vals(il, :, 1:3) = ea(iv, :)';
            vals(il, :, 4:5) = ea(it, :)';
            vals(il, :, 6)   = mean(abs(ea(tri, :)), 1)';
            vals(il, :, 7)   = std(R{il}.dxm(tri, :), 0, 1)' * 1e3;
        end
        lab = {'valley 1 s', 'valley 2 s', 'valley 3 s', 'top 1.5 s', 'top 2.5 s', 'tri mean|e_a|', 'dx_m std [nm]'};
        for j = 1:7
            a = squeeze(vals(1, :, j));  b = squeeze(vals(2, :, j));  d = b - a;
            sem = std(d) / sqrt(ns);  tt = mean(d) / max(sem, eps);
            fprintf('%-8s %-14s %+12.4f %+12.4f %+8.4f+-%.4f %+7.2f\n', w, lab{j}, mean(a), mean(b), mean(d), sem, tt);
            out.(w).(matlab.lang.makeValidName(lab{j})) = struct('eq17', mean(a), 'j6', mean(b), 'diff', mean(d), 'sem', sem, 't', tt);
        end
        % --- figure: rows a-bar | e_a | dx_m | c, columns eq17 | j6, one seed ---
        q = opts.seed_fig;
        fig = figure('Visible', 'off', 'Position', [100 100 1150 1000]);
        rows = 4;  ylim_row = zeros(rows, 2);
        for il = 1:2
            S = R{il};  ea = S.a_true(:, q) - S.a_hat(:, q);
            ylim_row(1, :) = [min(ylim_row(1, 1), min([S.a_true(:, q); S.a_hat(:, q); S.a_m(:, q)])), max(ylim_row(1, 2), max([S.a_true(:, q); S.a_hat(:, q); S.a_m(:, q)]))];
            ylim_row(2, :) = [min(ylim_row(2, 1), min(ea)), max(ylim_row(2, 2), max(ea))];
            ylim_row(3, :) = [min(ylim_row(3, 1), min(S.dxm(:, q))), max(ylim_row(3, 2), max(S.dxm(:, q)))];
            ylim_row(4, :) = [min(ylim_row(4, 1), min([S.c_eff_d; S.c_hat(:, q)])), max(ylim_row(4, 2), max([S.c_eff_d; S.c_hat(:, q)]))];
        end
        for il = 1:2
            S = R{il};  ea = S.a_true(:, q) - S.a_hat(:, q);  law = opts.laws{il};
            ax = subplot(rows, 2, il);
            plot(t, S.a_m(:, q), 'Color', [0.4 0.7 1.0]);  hold on;
            plot(t, S.a_true(:, q), 'r', 'LineWidth', 1.2);  plot(t, S.a_hat(:, q), 'b', 'LineWidth', 1.2);  hold off;  box on;
            ylabel([abar '_z']);  ylim([max(ylim_row(1, 1), -0.2), min(ylim_row(1, 2), 1.5)]);      % a_m spikes clipped so a_true / a_hat stay readable
            lg = legend(ax, {[abar '_m'], [abar '_{true}'], [abar hat ' (' law ')']}, 'Location', 'northoutside', 'Orientation', 'horizontal');
            lg.Title.String = sprintf('%s wall, seed %d', w, q);
            ax = subplot(rows, 2, 2 + il);
            plot(t, ea, 'b');  box on;  ylabel(['e_{' abar '}']);  ylim(local_pad(ylim_row(2, :)));
            ax = subplot(rows, 2, 4 + il);
            plot(t, S.dxm(:, q), 'Color', [0.4 0.7 1.0]);  box on;  ylabel('\deltax_m [\mum]');  ylim(local_pad(ylim_row(3, :)));
            ax = subplot(rows, 2, 6 + il);
            plot(t, S.c_eff_d, 'r', t, S.c_hat(:, q), 'b');  box on;  ylabel('c');  xlabel('t [s]');  ylim(local_pad(ylim_row(4, :)));
            legend(ax, {'c_{eff}(h_d)', ['c' hat]}, 'Location', 'northoutside', 'Orientation', 'horizontal');
        end
        fn = fullfile(od, sprintf('walls_%s%s_s%d.png', opts.tag, w, q));  exportgraphics(fig, fn, 'Resolution', 150);  close(fig);
        fprintf('figure -> %s\n', fn);
    end
end

function yl = local_pad(yl)
    d = max(yl(2) - yl(1), 1e-6);  yl = [yl(1) - 0.05 * d, yl(2) + 0.05 * d];
end
