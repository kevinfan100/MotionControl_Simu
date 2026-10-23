% PURPOSE (2026-09-30, user: "run the real simulation, T_d = 5 s first"; then "find what works best first"):
%   closed-loop Q44 arms. first order, lambda_f = 1, plane, seeds 1:3, hold 0.5 s -> 22.2 -> 1.10 R at constant
%   velocity in T_d -> hold 2 s.  Each arm = {tag, legend label, ctrl_const overrides}; runs cached per tag in
%   test_results/temp_5state_noap/q44sim_Td<T_d>_<tag>_s<seed>.mat.  Tag 'r0' must reproduce y2avg_Td<T_d>_s* bit for bit.
%   Rows: a_hat vs a_true | e_a = a_true - a_hat around the stop | a_m / a_true (reading bias) around the stop.
%   Usage: plot_5state_noap_q44_sim(5, {'r0','r1e-05','opt1'})  -- tags from local_arm below.   STATUS: SCRATCH
function plot_5state_noap_q44_sim(T_d, tags, out_tag, rerun, wall)
    if nargin < 1; T_d = 5; end
    if nargin < 2; tags = {'r0', 'r1e-05', 'opt1'}; end
    if nargin < 3; out_tag = ''; end
    if nargin < 4; rerun = false; end
    if nargin < 5; wall = 'plane'; end     % plant only; the estimator always starts from the plane (wall-test protocol)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));  addpath(fullfile(root, 'test_script', 'integration'));
    id = fullfile(root, 'test_results', 'temp_5state_noap');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    addpath(here);  plant_cperp = [];  wsuf = '';
    if strcmp(wall, 'cell'); plant_cperp = @cell_cperp_delta; wsuf = '_cell'; elseif strcmp(wall, 'sphere'); plant_cperp = @sphere_cperp_fig11; wsuf = '_sphere'; end
    pc = physical_constants();  R = pc.R;  a_nom = pc.Ts / pc.gamma_N;
    seeds = 1:3;  T_HOLD0 = 0.5;  T_HOLD1 = 2.0;
    COL = [0 0 0; 0 0.45 0.85; 0.85 0.45 0; 0.1 0.6 0.1; 0.6 0 0.6; 0.55 0.1 0.1; 0 0.6 0.6];  COL_TRUE = [0.8 0 0];
    FS = 18;  LFS = 12;  AXLW = 2.0;
    f = figure('Position', [40 40 1500 1350], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(f, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax = gobjects(1, 3);  for q = 1:3; ax(q) = nexttile(tl, q); hold(ax(q), 'on'); end
    fprintf('\n%s wall, T_d = %g s\n%-8s | %-9s %-9s %-9s | %-8s | %-8s %-8s | %-6s | %-7s\n', wall, T_d, 'arm', 'e_a stop', '+0.5 s', 'end', ...
        'far sd', 'rms desc', 'rms 1s', 'am/at', 'honesty');
    hh = gobjects(1, numel(tags));
    for m = 1:numel(tags)
        [lab, ov] = local_arm(tags{m});  nS = numel(seeds);  run = cell(1, nS);
        for q = 1:nS
            fn = fullfile(id, sprintf('q44sim_Td%g_%s%s_s%d.mat', T_d, tags{m}, wsuf, seeds(q)));
            if ~rerun && exist(fn, 'file'); run{q} = load(fn); continue; end
            cfg = canonical_scenario(0.05, 1.10, 'deep');  cfg.eq17_variant = '5state_noap';
            cfg.trajectory_type = 'ramp_descent';  cfg.h_init = 50;  cfg.h_bottom = 1.10 * R;  cfg.amplitude = 0;
            cfg.t_hold = T_HOLD0;  cfg.t_descend_override = T_d;  cfg.T_sim = T_HOLD0 + T_d + T_HOLD1;  cfg.h_min = 1.10 * R;
            cc = struct('lf_mode', 'const', 'lambda_f', 1, 'gain_order', 1, 'seed_mode', 'true_h0');
            for fn2 = fieldnames(ov).'; cc.(fn2{1}) = ov.(fn2{1}); end
            o = struct('seed', seeds(q), 'collect_diag', true, 'ctrl_const_override', cc);
            if ~isempty(plant_cperp); o.plant_cperp = plant_cperp; end
            s = run_pure_simulation(cfg, o);
            r = struct('t', s.tout(:), 'a_true', s.a_true_out(:, 3), 'a_hat', s.diag.a_hat(:, 3), 'a_m', s.diag.a_xm(:, 3), ...
                       'P_a', s.diag.P_a(:, 3), 'w_y2', s.diag.w_y2(:, 3));
            save(fn, '-struct', 'r');  run{q} = r;
        end
        if strcmp(tags{m}, 'r0') && isempty(wsuf)     % bit check against the runs of plot_5state_noap_y2_average.m
            ref = fullfile(id, sprintf('y2avg_Td%g_s%d.mat', T_d, seeds(1)));
            if exist(ref, 'file'); rr = load(ref); fprintf('  bit check r0 vs y2avg: max |d a_hat| = %g\n', max(abs(rr.a_hat - run{1}.a_hat))); end
        end
        t = run{1}.t - T_HOLD0;  n = numel(t);
        at = zeros(n, nS);  ah = at;  am = at;  Pa = at;
        for q = 1:nS; at(:, q) = run{q}.a_true; ah(:, q) = run{q}.a_hat; am(:, q) = run{q}.a_m; Pa(:, q) = run{q}.P_a; end
        e = (at - ah) / a_nom;  em = mean(e, 2);
        i1 = find(t <= T_d, 1, 'last');  i2 = find(t <= T_d + 0.5, 1, 'last');
        far = t > -0.4 & t <= 0;  wall = t > T_d + 0.5;  near = t > T_d - 1 & t <= T_d + T_HOLD1;
        desc = t >= 0 & t <= T_d;  last1 = t > T_d - 1 & t <= T_d;
        sdf = mean(std(ah(far, :) / a_nom, 0, 1));
        rmsd = sqrt(mean(e(desc, :).^2, 'all'));  rms1 = sqrt(mean(e(last1, :).^2, 'all'));
        amr = mean(mean(am(wall, :), 1) ./ mean(at(wall, :), 1));
        hon = sqrt(mean(e(near, :).^2, 'all')) / mean(sqrt(Pa(near, :)) / a_nom, 'all');
        fprintf('%-8s | %+-9.3f %+-9.3f %+-9.3f | %-8.4f | %-8.4f %-8.4f | %-6.2f | %-7.2f\n', tags{m}, em(i1), em(i2), em(end), sdf, rmsd, rms1, amr, hon);
        if m == 1
            h0 = plot(ax(1), nan, nan, 'w-', 'DisplayName', sprintf('%s wall, plane init, T_d = %g s, %d seeds', wall, T_d, numel(seeds)));
            ht = plot(ax(1), t, mean(at, 2) / a_nom, '-', 'Color', COL_TRUE, 'LineWidth', 2.4, 'DisplayName', 'a_{true}');
            plot(ax(3), [t(1) t(end)] - T_d, [1 1], '-', 'Color', COL_TRUE, 'LineWidth', 1.6);
        end
        hh(m) = plot(ax(1), t, mean(ah, 2) / a_nom, '-', 'Color', COL(m, :), 'LineWidth', 1.8, 'DisplayName', lab);
        plot(ax(2), t - T_d, em, '-', 'Color', COL(m, :), 'LineWidth', 1.8);
        plot(ax(3), t - T_d, movmean(mean(am, 2), 81) ./ mean(at, 2), '-', 'Color', COL(m, :), 'LineWidth', 1.6);
    end
    xlim(ax(1), [-T_HOLD0 T_d + T_HOLD1]);  ylim(ax(1), [0 1.2]);
    xlim(ax(2), [-min(T_d, 5) T_HOLD1]);  ylim(ax(2), [-0.9 0.3]);
    xlim(ax(3), [-min(T_d, 5) T_HOLD1]);  ylim(ax(3), [0.5 3]);
    xlabel(ax(1), 'time since the descent start (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel(ax(3), 'time since the stop (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    ylabel(ax(1), 'a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold');
    ylabel(ax(2), 'e_a = a_{true} - a_{hat}', 'FontSize', FS, 'FontWeight', 'bold');
    ylabel(ax(3), 'a_m / a_{true} (81-step mean)', 'FontSize', FS, 'FontWeight', 'bold');
    for q = 1:3
        if q > 1; yl = ylim(ax(q)); plot(ax(q), [0 0], yl, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2); end
        if q == 2; plot(ax(q), xlim(ax(q)), [0 0], '-', 'Color', [0.6 0.6 0.6]); end
        set(ax(q), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');
    end
    legend(ax(1), [h0 ht hh], 'Location', 'northoutside', 'Orientation', 'horizontal', 'NumColumns', 2, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on', 'AutoUpdate', 'off');
    out = fullfile(fd, sprintf('fig_noap_q44_sim_td%g%s%s.png', T_d, wsuf, out_tag));  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end

function [lab, ov] = local_arm(tag)
%LOCAL_ARM  tag -> legend label and ctrl_const overrides.
    if strcmp(tag, 'apcmd')
        lab = 'Q_{44} = (a''_{true} \Delta x_d)^2';  ov = struct('q44_aprime_cmd', true);
    elseif startsWith(tag, 'lb')
        e = str2double(tag(3:end));  lab = sprintf('lag budget \\epsilon = %g', e);  ov = struct('q44_lag_budget', e);
    elseif startsWith(tag, 'opt')
        k = str2double(tag(4:end));
        if k == 1; lab = 'best memory: l^* = (4v^2/R_2)^{1/3}'; else; lab = sprintf('%g \\times l^*', k); end
        ov = struct('q44_mse_opt', k);
    elseif startsWith(tag, 'r')
        rho = str2double(tag(2:end));
        if rho == 0; lab = '\rho = 0 (Q_{44} = 0)'; else; lab = sprintf('fixed \\rho = %g', rho); end
        ov = struct('q44_r2_frac', rho);
    else
        error('unknown arm tag %s', tag);
    end
end
