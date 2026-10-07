% PURPOSE (2026-09-30): first-order gain slot, Q44 designs side by side, T_d = 5 s, 22.2 -> 1.10 R constant velocity,
%   estimator initialised on the plane (wall-test protocol), 10 seeds. One column per arm; Q44 = 0 drawn grey in each.
%   House style (plot_5state_noap_ladder.m): shared time axis, segment lines, True red / estimate blue + sd band,
%   Rows: a_true, a_hat (mean +- sd) | e_a = a_true - a_hat (mean +- sd) with +- sqrt(P44) dashed.
%   Two files per call: <out_name>.png and <out_name>_am.png (the second adds a_m of seed 1, light blue).
%   Arm tags: 'r<rho>' Q44 = rho R2 | 'opt<kappa>' best memory (reads a'_true) | 'rat<sqrtP0>' Q/R-ratio design (ap model,
%   c-only 09-30) | 'lfi<beta>' information-driven forgetting (c-only 09-24).
%   A seed whose particle touches the wall (plant error h_bar <= 1) is recorded as crashed and left out of the statistics.
%   Runs cached in test_results/temp_5state_noap/q44w_<wall>_<arm>_s<seed>.mat.   STATUS: SCRATCH
function plot_5state_noap_q44_walls(cols, out_name, seeds)
    if nargin < 1 || isempty(cols); cols = {'plane', 'opt1'; 'cell', 'opt1'}; end
    if nargin < 2 || isempty(out_name); out_name = 'fig_noap_q44_walls_td5'; end
    if nargin < 3; seeds = 1:10; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));  addpath(fullfile(root, 'test_script', 'integration'));  addpath(here);
    id = fullfile(root, 'test_results', 'temp_5state_noap');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    pc = physical_constants();  a_nom = pc.Ts / pc.gamma_N;  T_d = 5;  T_HOLD0 = 0.5;  T_HOLD1 = 2.0;
    nC = size(cols, 1);
    fprintf('\n%-6s %-8s | %-7s | %-8s %-8s | %-8s %-8s | %-8s | %-8s %-8s | %-7s\n', 'wall', 'arm', 'crashed', 'stop mu', 'stop sd', ...
        '+0.5 mu', '+0.5 sd', 'far sd', 'rms desc', 'rms 1s', 'honesty');
    D = cell(nC, 2);
    for c = 1:nC
        for m = 1:2
            tag = 'r0';  if m == 2; tag = cols{c, 2}; end
            D{c, m} = local_collect(cols{c, 1}, tag, seeds, id, T_d, T_HOLD0, T_HOLD1, a_nom);
            d = D{c, m};  t = d.t;
            i1 = find(t <= T_d, 1, 'last');  i2 = find(t <= T_d + 0.5, 1, 'last');
            far = t > -0.4 & t <= 0;  desc = t >= 0 & t <= T_d;  last1 = t > T_d - 1 & t <= T_d;  near = t > T_d - 1 & t <= T_d + T_HOLD1;
            E = d.AT - d.AH;
            fprintf('%-6s %-8s | %2d/%-4d | %+-8.3f %-8.3f | %+-8.3f %-8.3f | %-8.4f | %-8.4f %-8.4f | %-7.2f\n', cols{c, 1}, tag, ...
                d.ncrash, numel(seeds), mean(E(i1, :)), std(E(i1, :)), mean(E(i2, :)), std(E(i2, :)), mean(std(d.AH(far, :), 0, 1)), ...
                sqrt(mean(E(desc, :).^2, 'all')), sqrt(mean(E(last1, :).^2, 'all')), ...
                sqrt(mean(E(near, :).^2, 'all')) / mean(d.SP(near, :), 'all'));
        end
    end
    for with_am = [false true]
        local_figure(D, cols, with_am, T_d, fullfile(fd, [out_name, repmat('_am', 1, with_am), '.png']));
    end
end

function d = local_collect(wall, tag, seeds, id, T_d, T_HOLD0, T_HOLD1, a_nom)
    R = physical_constants().R;  pcp = [];
    if strcmp(wall, 'cell'); pcp = @cell_cperp_delta; elseif strcmp(wall, 'sphere'); pcp = @sphere_cperp_fig11; end
    acov = 0.05;  base = tag;  k_ac = strfind(tag, '_ac');
    if ~isempty(k_ac); acov = str2double(tag(k_ac + 3:end)); base = tag(1:k_ac - 1); end   % tag suffix _ac<a_cov>
    nw = contains(base, '_nw');  base = strrep(base, '_nw', '');   % tag suffix _nw = y2 whitening off
    [~, ov] = local_arm(base);  if nw; ov.y2_whiten = false; end;  runs = {};  ncrash = 0;
    for q = 1:numel(seeds)
        fn = fullfile(id, sprintf('q44w_%s_%s_s%d.mat', wall, strrep(tag, '.', 'p'), seeds(q)));
        need_am = isempty(runs);
        if exist(fn, 'file')
            r = load(fn);
            if isfield(r, 'crashed') && r.crashed; ncrash = ncrash + 1; continue; end
            if ~need_am || isfield(r, 'a_m'); runs{end + 1} = r; continue; end %#ok<AGROW>
        end
        cfg = canonical_scenario(acov, 1.10, 'deep');  cfg.eq17_variant = '5state_noap';
        cfg.trajectory_type = 'ramp_descent';  cfg.h_init = 50;  cfg.h_bottom = 1.10 * R;  cfg.amplitude = 0;
        cfg.t_hold = T_HOLD0;  cfg.t_descend_override = T_d;  cfg.T_sim = T_HOLD0 + T_d + T_HOLD1;  cfg.h_min = 1.10 * R;
        cc = struct('lf_mode', 'const', 'lambda_f', 1, 'gain_order', 1, 'seed_mode', 'true_h0');
        for fn2 = fieldnames(ov).'; cc.(fn2{1}) = ov.(fn2{1}); end
        o = struct('seed', seeds(q), 'collect_diag', true, 'ctrl_const_override', cc);
        if ~isempty(pcp); o.plant_cperp = pcp; end
        try
            s = run_pure_simulation(cfg, o);
        catch err
            if contains(err.message, 'must not touch wall')
                crashed = true;  save(fn, 'crashed');  ncrash = ncrash + 1;  fprintf('  %s %s seed %d: particle touched the wall\n', wall, tag, seeds(q));  continue;
            end
            rethrow(err);
        end
        r = struct('t', s.tout(:), 'a_true', s.a_true_out(:, 3), 'a_hat', s.diag.a_hat(:, 3), 'P_a', s.diag.P_a(:, 3), ...
                   'w_y2', s.diag.w_y2(:, 3), 'l42', s.diag.K_kf_a_y2(:, 3), 'h_true', s.p_true_out(:, 3) / R, 'p_d', s.p_d_out(:, 3), ...
                   'a_m', s.diag.a_xm(:, 3), 'crashed', false);
        save(fn, '-struct', 'r');  runs{end + 1} = r; %#ok<AGROW>
    end
    n = numel(runs{1}.t);  nS = numel(runs);
    d = struct('t', runs{1}.t - T_HOLD0, 'AT', zeros(n, nS), 'AH', zeros(n, nS), 'SP', zeros(n, nS), 'am1', [], 'ncrash', ncrash, 'tag', tag, 'wall', wall);
    for q = 1:nS
        d.AT(:, q) = runs{q}.a_true / a_nom;  d.AH(:, q) = runs{q}.a_hat / a_nom;  d.SP(:, q) = sqrt(runs{q}.P_a) / a_nom;
    end
    if isfield(runs{1}, 'a_m'); d.am1 = runs{1}.a_m / a_nom; end
end

function local_figure(D, cols, with_am, T_d, out)
    COL_TRUE = [0.8 0 0];  C_EST = [0 0.2 0.9];  B_EST = [0.45 0.55 0.95];  C_MEAS = [0.55 0.75 1.0];  C_ZERO = [0.4 0.4 0.4];
    FS = 18;  LFS = 12;  AXLW = 2.0;  nC = size(cols, 1);
    f = figure('Position', [40 40 max(1100, 800 * nC), 1000], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(2, nC, 'TileSpacing', 'compact', 'Padding', 'compact');
    for c = 1:nC
        d0 = D{c, 1};  d = D{c, 2};  t = d.t;  k_ac = strfind(d.tag, '_ac');
        bt = strrep(d.tag, '_nw', '');  k_ac = strfind(bt, '_ac');
        if isempty(k_ac); [lab, ~] = local_arm(bt); lab = [lab ', a_{cov} = 0.05']; else; [lab, ~] = local_arm(bt(1:k_ac - 1)); lab = sprintf('%s, a_{cov} = %s', lab, bt(k_ac + 3:end)); end
        if contains(d.tag, '_nw'); lab = [lab ', no whitening']; else; lab = [lab ', whitened']; end
        desc = sprintf('%s wall, plane init, 22.2 -> 1.10 R in %g s, %d seeds', d.wall, T_d, size(d.AH, 2));
        if d.ncrash > 0; desc = sprintf('%s (%d crashed)', desc, d.ncrash); end
        am = mean(d.AH, 2);  as = std(d.AH, 0, 2);  E = d.AT - d.AH;  em = mean(E, 2);  es = std(E, 0, 2);
        % row 1
        nexttile(c); hold on; box on;
        h0 = plot(NaN, NaN, 'w.', 'DisplayName', desc);
        if with_am && ~isempty(d.am1)
            hm = plot(t, d.am1, '-', 'Color', C_MEAS, 'LineWidth', 0.6, 'DisplayName', 'a_m (seed 1)');
        end
        fill([t; flipud(t)], [am + as; flipud(am - as)], B_EST, 'FaceAlpha', 0.30, 'EdgeColor', 'none', 'HandleVisibility', 'off');
        hz = plot(t, mean(d0.AH, 2), '-', 'Color', C_ZERO, 'LineWidth', 1.4, 'DisplayName', 'a_{hat}, Q_{44} = 0');
        ht = plot(t, mean(d.AT, 2), '-', 'Color', COL_TRUE, 'LineWidth', 2.2, 'DisplayName', 'a_{true}');
        he = plot(t, am, '-', 'Color', C_EST, 'LineWidth', 1.8, 'DisplayName', ['a_{hat}, ' lab]);
        if with_am && ~isempty(d.am1); ylim([-0.2 1.6]); hl = [h0 ht he hz hm]; else; ylim([0 1.1]); hl = [h0 ht he hz]; end
        local_seg(T_d);  xlim([t(1) t(end)]);
        if c == 1; ylabel('a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold'); end
        legend(hl, 'Location', 'northoutside', 'Orientation', 'horizontal', 'NumColumns', 2, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        local_ax(FS, AXLW);
        % row 2
        nexttile(nC + c); hold on; box on;
        fill([t; flipud(t)], [em + es; flipud(em - es)], B_EST, 'FaceAlpha', 0.30, 'EdgeColor', 'none', 'HandleVisibility', 'off');
        plot(t, mean(d0.AT - d0.AH, 2), '-', 'Color', C_ZERO, 'LineWidth', 1.4);
        plot(t, em, '-', 'Color', C_EST, 'LineWidth', 1.8);
        hp = plot(t, mean(d.SP, 2), '--', 'Color', C_EST, 'LineWidth', 1.2, 'DisplayName', '\pm \surd P_{44}');
        plot(t, -mean(d.SP, 2), '--', 'Color', C_EST, 'LineWidth', 1.2);
        yline(0, '-', 'Color', [0.6 0.6 0.6], 'HandleVisibility', 'off');
        ylim([-0.4 0.2]);  local_seg(T_d);  xlim([t(1) t(end)]);
        if c == 1; ylabel('e_a = a_{true} - a_{hat}', 'FontSize', FS, 'FontWeight', 'bold'); end
        xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        legend(hp, 'Location', 'northoutside', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        local_ax(FS, AXLW);
    end
    exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);  close(f);
end

function local_seg(T_d)
    yl = ylim;  for x = [0 T_d]; plot([x x], yl, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2, 'HandleVisibility', 'off'); end;  ylim(yl);
end

function local_ax(FS, AXLW)
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');  grid off;
end

function [lab, ov] = local_arm(tag)
    if startsWith(tag, 'opt')
        lab = 'best memory (reads a''_{true})';  ov = struct('q44_mse_opt', str2double(tag(4:end)));
    elseif startsWith(tag, 'ratr')
        sg = str2double(tag(5:end));  lab = sprintf('Q/R ratio, \\surdP_0 = %g a_{hat}', sg);  ov = struct('q44_ratio_rel', sg);
    elseif startsWith(tag, 'ratc')
        sp = str2double(tag(5:end));  lab = sprintf('Q/R ratio capped, \\surdP_0 = %g a_{nom}', sp);  ov = struct('q44_ratio_std', sp, 'q44_ratio_cap', true);
    elseif startsWith(tag, 'rat')
        sp = str2double(tag(4:end));  lab = sprintf('Q/R ratio, \\surdP_0 = %g a_{nom}', sp);  ov = struct('q44_ratio_std', sp);
    elseif startsWith(tag, 'lfi')
        b = str2double(tag(4:end));  lab = sprintf('info forgetting \\beta = %g', b);  ov = struct('lf_info_beta', b);
    elseif startsWith(tag, 'r')
        rho = str2double(tag(2:end));
        if rho == 0; lab = 'Q_{44} = 0'; else; lab = sprintf('fixed \\rho = %g (%.2f s)', rho, 0.000625 / sqrt(rho)); end
        ov = struct('q44_r2_frac', rho);
    else
        error('unknown arm %s', tag);
    end
end
