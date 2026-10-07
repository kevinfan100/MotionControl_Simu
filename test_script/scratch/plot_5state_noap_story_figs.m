% PURPOSE (2026-10-01, user: first-order story, main line = Q44 = 0 (grey) vs fixed rho = 1e-5 (blue) only):
%   Fig 1  ramp T_d = 5 | 15 s, Q44 = 0 + the 1/R2-weighted average of a_true (black dashed)  -> lag, T_d does not help
%   Fig 3  ramp T_d = 5 | 15 s, Q44 = 0 vs rho = 1e-5 (same layout as Fig 1)                  -> with memory, T_d helps
%   Fig 4  held at 22.2 | 1.10 R, init a_nom, Q44 = 0 vs rho = 1e-5                             -> fixed-point contrast
%   (Fig 2 = fig_noap_q44_fixed_rho_plane.png, plot_5state_noap_q44_walls.m.)
%   Reads the story_* caches of plot_5state_noap_story.m; the Q44 = 0 ramp runs are redone once with the y2 weight
%   w_y2 saved (tag q0w) for the dashed average. Two versions per figure (with / without a_m of seed 1).
%   Usage: plot_5state_noap_story_figs('plane')   STATUS: SCRATCH
function plot_5state_noap_story_figs(wall, seeds, TDs)
    if nargin < 1; wall = 'plane'; end
    if nargin < 2 || isempty(seeds); seeds = 1:10; end
    if nargin < 3 || isempty(TDs); TDs = [5 15]; end   % the two descent times compared in Fig 1 / Fig 3
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));  addpath(fullfile(root, 'test_script', 'integration'));  addpath(here);
    id = fullfile(root, 'test_results', 'temp_5state_noap');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    GREY = [0.4 0.4 0.4];  BLUE = [0 0.2 0.9];
    ramp = {{'ramp', TDs(1)}, {'ramp', TDs(2)}};  tdsfx = '';  if ~isequal(TDs, [5 15]); tdsfx = strrep(sprintf('_td%g_%g', TDs), '.', 'p'); end;  fixed = {{'fixed', 22.2}, {'fixed', 1.10}};
    Q0W = cell(1, 2);  RH = cell(1, 2);  F0 = cell(1, 2);  FR = cell(1, 2);
    for c = 1:2
        Q0W{c} = local_collect(wall, ramp{c}, 'q0w', struct(), seeds, id);
        RH{c}  = local_collect(wall, ramp{c}, 'rho', struct('q44_r2_frac', 1e-5), seeds, id);
        F0{c}  = local_collect(wall, fixed{c}, 'q0', struct(), seeds, id);
        FR{c}  = local_collect(wall, fixed{c}, 'rho', struct('q44_r2_frac', 1e-5), seeds, id);
        for d = {Q0W{c}, RH{c}}; local_print_ramp(wall, d{1}); end
        for d = {F0{c}, FR{c}}; local_print_fixed(wall, d{1}); end
    end
    for with_am = [false true]
        sfx = [wall, tdsfx, repmat('_am', 1, with_am)];
        local_figure({Q0W}, {'Q_{44} = 0'}, {GREY}, true,  ramp,  wall, with_am, fullfile(fd, ['fig_noap_story1_q0_' sfx '.png']));
        local_figure({Q0W, RH}, {'Q_{44} = 0', '\epsilon = Q_{44}/R_2 = 10^{-5}'}, {GREY, BLUE}, false, ramp, wall, with_am, fullfile(fd, ['fig_noap_story3_rho_' sfx '.png']));
        local_figure({F0, FR}, {'Q_{44} = 0', '\epsilon = Q_{44}/R_2 = 10^{-5}'}, {GREY, BLUE}, false, fixed, wall, with_am, fullfile(fd, ['fig_noap_story4_fixed_' sfx '.png']));
    end
end

function d = local_collect(wall, sc, tag, ov, seeds, id)
    pc = physical_constants();  R = pc.R;  a_nom = pc.Ts / pc.gamma_N;
    pcp = [];  if strcmp(wall, 'cell'); pcp = @cell_cperp_delta; end
    stag = strrep(sprintf('%s%g', sc{1}, sc{2}), '.', 'p');  runs = {};  ncrash = 0;
    for q = 1:numel(seeds)
        fn = fullfile(id, sprintf('story_%s_%s_%s_s%d.mat', wall, stag, tag, seeds(q)));
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
        for fn2 = fieldnames(ov).'; cc.(fn2{1}) = ov.(fn2{1}); end
        o = struct('seed', seeds(q), 'collect_diag', true, 'ctrl_const_override', cc);
        if ~isempty(pcp); o.plant_cperp = pcp; end
        try
            s = run_pure_simulation(cfg, o);
        catch err
            if contains(err.message, 'must not touch wall'); crashed = true; save(fn, 'crashed'); ncrash = ncrash + 1; continue; end
            rethrow(err);
        end
        r = struct('t', s.tout(:), 'a_true', s.a_true_out(:, 3), 'a_hat', s.diag.a_hat(:, 3), 'P_a', s.diag.P_a(:, 3), ...
                   'a_m', s.diag.a_xm(:, 3), 'w_y2', s.diag.w_y2(:, 3), 'crashed', false);
        save(fn, '-struct', 'r');  runs{end + 1} = r; %#ok<AGROW>
    end
    t0 = 0;  if strcmp(sc{1}, 'ramp'); t0 = 0.5; end
    n = numel(runs{1}.t);  nS = numel(runs);
    d = struct('t', runs{1}.t - t0, 'AT', zeros(n, nS), 'AH', zeros(n, nS), 'SP', zeros(n, nS), 'AV', [], ...
               'am1', runs{1}.a_m / a_nom, 'ncrash', ncrash, 'sc', {sc}, 'tag', tag);
    for q = 1:nS
        r = runs{q};  d.AT(:, q) = r.a_true / a_nom;  d.AH(:, q) = r.a_hat / a_nom;  d.SP(:, q) = sqrt(r.P_a) / a_nom;
        if isfield(r, 'w_y2')   % 1/R2-weighted average of a_true, initial estimate counted with weight 1/P44[0] (0929 doc 4.7)
            w = r.w_y2;  w(1) = 0;  W0 = 1 / r.P_a(1);
            d.AV(:, q) = (W0 * r.a_hat(1) + cumsum(w .* r.a_true)) ./ (W0 + cumsum(w)) / a_nom;
        end
    end
end

function local_print_ramp(wall, d)
    t = d.t;  E = d.AT - d.AH;  T_d = d.sc{2};  i1 = find(t <= T_d, 1, 'last');  i2 = find(t <= T_d + 0.5, 1, 'last');
    far = t > -0.4 & t <= 0;  desc = t >= 0 & t <= T_d;
    fprintf('%-6s ramp  T_d %-3g %-4s | stop %+.3f +- %.3f | +0.5 s %+.3f | end %+.3f | far sd %.4f | rms desc %.4f\n', wall, T_d, d.tag, ...
        mean(E(i1, :)), std(E(i1, :)), mean(E(i2, :)), mean(E(end, :)), mean(std(d.AH(far, :), 0, 1)), sqrt(mean(E(desc, :).^2, 'all')));
end

function local_print_fixed(wall, d)
    t = d.t;  E = d.AT - d.AH;  i02 = find(t >= 0.2, 1);  i1 = find(t >= 1.0, 1);  last = t > t(end) - 1;
    fprintf('%-6s fixed h %-5g %-4s | a_true %.3f | e_a 0.2 s %+.3f  1 s %+.3f  end %+.3f | sd last 1 s %.4f\n', wall, d.sc{2}, d.tag, ...
        mean(d.AT(end, :)), mean(E(i02, :)), mean(E(i1, :)), mean(E(end, :)), mean(std(d.AH(last, :), 0, 1)));
end

function local_figure(DS, labs, cols, with_avg, scen, wall, with_am, out)
    COL_TRUE = [0.8 0 0];  C_MEAS = [0.55 0.75 1.0];  FS = 18;  LFS = 12;  AXLW = 2.0;  nA = numel(DS);
    f = figure('Position', [40 40 1700 1000], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for c = 1:2
        sc = scen{c};  d1 = DS{1}{c};  t = d1.t;  nS = size(d1.AH, 2);
        if strcmp(sc{1}, 'ramp'); desc = sprintf('%s wall, plane init, 22.2 -> 1.10 R in T_d = %g s, %d seeds', wall, sc{2}, nS);
        else; desc = sprintf('%s wall, held at %g R, init a_{nom}, %d seeds', wall, sc{2}, nS); end
        % row 1
        nexttile(c); hold on; box on;
        h0 = plot(NaN, NaN, 'w.', 'DisplayName', desc);  hl = h0;
        if with_am; hm = plot(t, d1.am1, '-', 'Color', C_MEAS, 'LineWidth', 0.6, 'DisplayName', 'a_m (seed 1)'); end
        for m = 1:nA
            d = DS{m}{c};  am = mean(d.AH, 2);  as = std(d.AH, 0, 2);
            fill([t; flipud(t)], [am + as; flipud(am - as)], cols{m}, 'FaceAlpha', 0.18, 'EdgeColor', 'none', 'HandleVisibility', 'off');
            hl(end + 1) = plot(t, am, '-', 'Color', cols{m}, 'LineWidth', 1.8, 'DisplayName', ['a_{hat}, ' labs{m}]); %#ok<AGROW>
        end
        ht = plot(t, mean(d1.AT, 2), '-', 'Color', COL_TRUE, 'LineWidth', 2.2, 'DisplayName', 'a_{true}');
        hl = [hl(1) ht hl(2:end)];
        if with_avg && ~isempty(d1.AV)
            hl(end + 1) = plot(t, mean(d1.AV, 2), '--', 'Color', 'k', 'LineWidth', 2.0, 'DisplayName', 'weighted avg. of a_{true}');
        end
        if with_am; hl(end + 1) = hm; ylim([-0.2 1.6]); else; ylim([0 1.1]); end
        local_seg(sc);  xlim([t(1) t(end)]);
        if c == 1; ylabel('a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold'); end
        legend(hl, 'Location', 'northoutside', 'Orientation', 'horizontal', 'NumColumns', 2, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        local_ax(FS, AXLW);
        % row 2
        nexttile(2 + c); hold on; box on;
        for m = 1:nA
            d = DS{m}{c};  E = d.AT - d.AH;  em = mean(E, 2);  es = std(E, 0, 2);
            fill([t; flipud(t)], [em + es; flipud(em - es)], cols{m}, 'FaceAlpha', 0.18, 'EdgeColor', 'none', 'HandleVisibility', 'off');
            plot(t, em, '-', 'Color', cols{m}, 'LineWidth', 1.8);
        end
        if with_avg && ~isempty(d1.AV); plot(t, mean(d1.AT - d1.AV, 2), '--', 'Color', 'k', 'LineWidth', 2.0); end
        yline(0, '-', 'Color', [0.6 0.6 0.6], 'HandleVisibility', 'off');
        if strcmp(sc{1}, 'ramp'); ylim([-0.9 0.2]); else; ylim([-1.0 0.3]); end
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
