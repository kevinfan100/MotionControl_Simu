% PURPOSE (2026-10-01, user: first-order story line, layers 1-3): three arms overlaid -- Q44 = 0 (grey), fixed
%   rho = 1e-5 (blue), the c-only Q/R-ratio rule ported to the gain slot with relative P0 (q44_ratio_rel = 0.1, green) --
%   on one wall, two scenario families, one figure each:
%     'ramp'  : 22.2 -> 1.10 R at constant velocity, T_d = 5 s | 15 s (0.5 s hold before, 2 s after), init = plane at h0
%     'fixed' : positioning held at 22.2 R | 1.10 R for 3 s, init a_nom (prior 0.5 a_nom)
%   House style: two rows (a_true, a_hat mean +- sd | e_a mean +- sd), shared Time axis, legend northoutside;
%   each figure also written with a_m of seed 1 overlaid (_am).
%   Usage: plot_5state_noap_story('plane')  |  plot_5state_noap_story('cell')
%   Runs cached in test_results/temp_5state_noap/story_<wall>_<scen>_<arm>_s<seed>.mat.   STATUS: SCRATCH
function plot_5state_noap_story(wall, seeds)
    if nargin < 1; wall = 'plane'; end
    if nargin < 2; seeds = 1:10; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));  addpath(fullfile(root, 'test_script', 'integration'));  addpath(here);
    id = fullfile(root, 'test_results', 'temp_5state_noap');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    ARMS = {'q0',   'Q_{44} = 0',                          [0.4 0.4 0.4],  struct();
            'rho',  'fixed \rho = 10^{-5}',                 [0 0.2 0.9],    struct('q44_r2_frac', 1e-5);
            'ratr', 'Q/R ratio (c-only rule), \surdP_0 = 0.1 a_{hat}', [0.1 0.6 0.1], struct('q44_ratio_rel', 0.1)};
    FAM = {'ramp',  {{'ramp', 5}, {'ramp', 15}};
           'fixed', {{'fixed', 22.2}, {'fixed', 1.10}}};
    for fi = 1:size(FAM, 1)
        scen = FAM{fi, 2};  D = cell(2, size(ARMS, 1));
        for c = 1:2
            for m = 1:size(ARMS, 1)
                D{c, m} = local_collect(wall, scen{c}, ARMS(m, :), seeds, id);
                local_print(wall, scen{c}, ARMS{m, 1}, D{c, m});
            end
        end
        for with_am = [false true]
            out = fullfile(fd, sprintf('fig_noap_story_%s_%s%s.png', FAM{fi, 1}, wall, repmat('_am', 1, with_am)));
            local_figure(D, scen, ARMS, wall, with_am, out);
        end
    end
end

function d = local_collect(wall, sc, arm, seeds, id)
    R = physical_constants().R;  pc = physical_constants();  a_nom = pc.Ts / pc.gamma_N;
    pcp = [];  if strcmp(wall, 'cell'); pcp = @cell_cperp_delta; end
    stag = sprintf('%s%g', sc{1}, sc{2});  stag = strrep(stag, '.', 'p');
    runs = {};  ncrash = 0;
    for q = 1:numel(seeds)
        fn = fullfile(id, sprintf('story_%s_%s_%s_s%d.mat', wall, stag, arm{1}, seeds(q)));
        if exist(fn, 'file')
            r = load(fn);  if r.crashed; ncrash = ncrash + 1; else; runs{end + 1} = r; end; continue; %#ok<AGROW>
        end
        cfg = canonical_scenario(0.05, 1.10, 'deep');  cfg.eq17_variant = '5state_noap';  cfg.amplitude = 0;
        cc = struct('lf_mode', 'const', 'lambda_f', 1, 'gain_order', 1);
        if strcmp(sc{1}, 'ramp')
            cfg.trajectory_type = 'ramp_descent';  cfg.h_init = 50;  cfg.h_bottom = 1.10 * R;  cfg.h_min = 1.10 * R;
            cfg.t_hold = 0.5;  cfg.t_descend_override = sc{2};  cfg.T_sim = 0.5 + sc{2} + 2.0;  cc.seed_mode = 'true_h0';
        else
            cfg.trajectory_type = 'positioning';  cfg.h_init = sc{2} * R;  cfg.h_bottom = sc{2} * R;  cfg.h_min = min(1.10, sc{2}) * R;
            cfg.t_hold = 0;  cfg.t_descend_override = 0;  cfg.T_sim = 3.0;  cc.seed_mode = 'anom';
        end
        for fn2 = fieldnames(arm{4}).'; cc.(fn2{1}) = arm{4}.(fn2{1}); end
        o = struct('seed', seeds(q), 'collect_diag', true, 'ctrl_const_override', cc);
        if ~isempty(pcp); o.plant_cperp = pcp; end
        try
            s = run_pure_simulation(cfg, o);
        catch err
            if contains(err.message, 'must not touch wall')
                crashed = true;  save(fn, 'crashed');  ncrash = ncrash + 1;  fprintf('  %s %s %s seed %d: particle touched the wall\n', wall, stag, arm{1}, seeds(q));  continue;
            end
            rethrow(err);
        end
        r = struct('t', s.tout(:), 'a_true', s.a_true_out(:, 3), 'a_hat', s.diag.a_hat(:, 3), 'P_a', s.diag.P_a(:, 3), ...
                   'a_m', s.diag.a_xm(:, 3), 'crashed', false);
        save(fn, '-struct', 'r');  runs{end + 1} = r; %#ok<AGROW>
    end
    t0 = 0;  if strcmp(sc{1}, 'ramp'); t0 = 0.5; end
    n = numel(runs{1}.t);  nS = numel(runs);
    d = struct('t', runs{1}.t - t0, 'AT', zeros(n, nS), 'AH', zeros(n, nS), 'SP', zeros(n, nS), 'am1', runs{1}.a_m / a_nom, 'ncrash', ncrash, 'sc', {sc});
    for q = 1:nS
        d.AT(:, q) = runs{q}.a_true / a_nom;  d.AH(:, q) = runs{q}.a_hat / a_nom;  d.SP(:, q) = sqrt(runs{q}.P_a) / a_nom;
    end
end

function local_print(wall, sc, arm, d)
    t = d.t;  E = d.AT - d.AH;
    if strcmp(sc{1}, 'ramp')
        T_d = sc{2};  i1 = find(t <= T_d, 1, 'last');  i2 = find(t <= T_d + 0.5, 1, 'last');
        far = t > -0.4 & t <= 0;  desc = t >= 0 & t <= T_d;  near = t > T_d - 1 & t <= T_d + 2;
        fprintf('%-6s ramp  T_d %-4g %-5s | crashed %d | stop %+.3f +- %.3f | +0.5 s %+.3f | far sd %.4f | rms desc %.4f | honesty %.2f\n', ...
            wall, T_d, arm, d.ncrash, mean(E(i1, :)), std(E(i1, :)), mean(E(i2, :)), mean(std(d.AH(far, :), 0, 1)), ...
            sqrt(mean(E(desc, :).^2, 'all')), sqrt(mean(E(near, :).^2, 'all')) / mean(d.SP(near, :), 'all'));
    else
        i02 = find(t >= 0.2, 1);  i1 = find(t >= 1.0, 1);  last = t > t(end) - 1;
        fprintf('%-6s fixed h %-5g %-5s | crashed %d | a_true %.3f | e_a 0.2 s %+.3f  1 s %+.3f  end %+.3f | sd last 1 s %.4f | honesty %.2f\n', ...
            wall, sc{2}, arm, d.ncrash, mean(d.AT(end, :)), mean(E(i02, :)), mean(E(i1, :)), mean(E(end, :)), ...
            mean(std(d.AH(last, :), 0, 1)), sqrt(mean(E(last, :).^2, 'all')) / mean(d.SP(last, :), 'all'));
    end
end

function local_figure(D, scen, ARMS, wall, with_am, out)
    COL_TRUE = [0.8 0 0];  C_MEAS = [0.55 0.75 1.0];  FS = 18;  LFS = 12;  AXLW = 2.0;  nA = size(ARMS, 1);
    f = figure('Position', [40 40 1700 1000], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for c = 1:2
        sc = scen{c};  t = D{c, 1}.t;
        if strcmp(sc{1}, 'ramp'); desc = sprintf('%s wall, plane init, 22.2 -> 1.10 R in T_d = %g s, %d seeds', wall, sc{2}, size(D{c, 1}.AH, 2));
        else; desc = sprintf('%s wall, held at %g R, init a_{nom}, %d seeds', wall, sc{2}, size(D{c, 1}.AH, 2)); end
        % row 1
        nexttile(c); hold on; box on;
        h0 = plot(NaN, NaN, 'w.', 'DisplayName', desc);  hl = h0;
        if with_am; hm = plot(t, D{c, 1}.am1, '-', 'Color', C_MEAS, 'LineWidth', 0.6, 'DisplayName', 'a_m (seed 1, Q_{44} = 0)'); end
        for m = 1:nA
            d = D{c, m};  am = mean(d.AH, 2);  as = std(d.AH, 0, 2);
            if m > 1; fill([t; flipud(t)], [am + as; flipud(am - as)], ARMS{m, 3}, 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'HandleVisibility', 'off'); end
            lab = ARMS{m, 2};  if d.ncrash > 0; lab = sprintf('%s (%d crashed)', lab, d.ncrash); end
            hl(end + 1) = plot(t, am, '-', 'Color', ARMS{m, 3}, 'LineWidth', 1.8, 'DisplayName', lab); %#ok<AGROW>
        end
        ht = plot(t, mean(D{c, 1}.AT, 2), '-', 'Color', COL_TRUE, 'LineWidth', 2.2, 'DisplayName', 'a_{true}');
        hl = [hl(1) ht hl(2:end)];  if with_am; hl(end + 1) = hm; end %#ok<AGROW>
        if with_am; ylim([-0.2 1.6]); else; ylim([0 1.1]); end
        local_seg(sc);  xlim([t(1) t(end)]);
        if c == 1; ylabel('a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold'); end
        legend(hl, 'Location', 'northoutside', 'Orientation', 'horizontal', 'NumColumns', 2, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        local_ax(FS, AXLW);
        % row 2
        nexttile(2 + c); hold on; box on;
        for m = 1:nA
            d = D{c, m};  E = d.AT - d.AH;  em = mean(E, 2);  es = std(E, 0, 2);
            if m > 1; fill([t; flipud(t)], [em + es; flipud(em - es)], ARMS{m, 3}, 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'HandleVisibility', 'off'); end
            plot(t, em, '-', 'Color', ARMS{m, 3}, 'LineWidth', 1.8);
        end
        yline(0, '-', 'Color', [0.6 0.6 0.6], 'HandleVisibility', 'off');
        if strcmp(sc{1}, 'ramp'); ylim([-0.5 0.2]); else; ylim([-1.0 0.3]); end
        local_seg(sc);  xlim([t(1) t(end)]);
        if c == 1; ylabel('e_a = a_{true} - a_{hat}', 'FontSize', FS, 'FontWeight', 'bold'); end
        xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        local_ax(FS, AXLW);
    end
    exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);  close(f);
end

function local_seg(sc)
    if ~strcmp(sc{1}, 'ramp'); return; end
    yl = ylim;  for x = [0 sc{2}]; plot([x x], yl, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2, 'HandleVisibility', 'off'); end;  ylim(yl);
end

function local_ax(FS, AXLW)
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');  grid off;
end
