% PURPOSE (2026-09-22, user: "slow trajectory is the focus; show a+da vs da-only forgetting"): single-seed pictures, two
%   scenarios (hold 1.10 R far-field init; slow ramp cheat init), two arms on the SAME seed, Ljung schedule in both:
%     both     forgetting on the gain block a + da (lf_slots [4 5 8], default)
%     daonly   forgetting on the rate slot only (lf_slots 5)
%   Rows: a_true / a_hat | e_a = a_true - a_hat with +- sqrt(P44) | zoom of e_a on the steady-state window.
%   House style (plot_delta_compact.m). Output test_results/temp_5state_noap/fig_noap_oracle_<scn>_s<seed>.png
%   EXPIRES: with the layer-1 line | 產線改動不會自動跟上
function plot_5state_noap_lfslots_ss(seed)
    if nargin < 1 || isempty(seed); seed = 1; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));  addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'temp_5state_noap');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    pc = physical_constants();  R = pc.R;  a_nom = pc.Ts / pc.gamma_N;
    COL_TRUE = [0.8 0 0]; COL_REAL = [0 0.2 0.9]; COL_OR = [0.85 0.45 0]; FS = 18; LFS = 13; AXLW = 2.0;   % blue = ljung schedule, orange = lambda_f = 1
    scn = {'hold1p10', 'meng'};
    for is = 1:2
        cfg = canonical_scenario(0.05, 1.10, 'deep');  cfg.eq17_variant = '5state_noap';
        switch scn{is}
            case 'hold1p10'
                cfg.trajectory_type = 'positioning';  cfg.h_init = 1.10 * R;  cfg.h_bottom = 1.10 * R;
                cfg.amplitude = 0;  cfg.t_hold = 0;  cfg.t_descend_override = 0;  cfg.T_sim = 3.0;
                seed_mode = 'anom';  zoom_w = [1.0 3.0];  seg = [];  desc = sprintf('hold 1.10 R, init a_{nom}, seed %d', seed);
            case 'meng'
                cfg.h_init = 15;  cfg.h_bottom = 2.5;  cfg.amplitude = 0;  cfg.frequency = 1;  cfg.n_cycles = 1;
                cfg.t_hold = 0.5;  cfg.t_descend_override = 10;  cfg.T_sim = 12.5;  cfg.h_min = 2.475;
                seed_mode = 'true_h0';  zoom_w = [10.5 12.5];  seg = [0.5 10.5];  desc = sprintf('slow ramp 15 -> 2.5 um over 10 s, init a(h_0), seed %d', seed);
        end
        S = cell(2, 1);
        for ia = 1:2
            cc = struct('seed_mode', seed_mode, 'lf_mode', 'ljung');
            if ia == 2; cc.lf_slots = 5; end
            S{ia} = run_pure_simulation(cfg, struct('seed', seed, 'collect_diag', true, 'ctrl_const_override', cc));
        end
        t = S{1}.tout(:);  at = S{1}.a_true_out(:, 3) / a_nom;
        ah = cellfun(@(s) s.diag.a_hat(:, 3) / a_nom, S, 'UniformOutput', false);
        sp = cellfun(@(s) sqrt(s.diag.P_a(:, 3)) / a_nom, S, 'UniformOutput', false);
        m = t >= zoom_w(1) & t <= zoom_w(2);
        fprintf('%s seed %d | steady window %.1f-%.1f s: a+da  mean e %+.4f  sd(t) %.4f  sqrtP44 %.4f | da-only  mean e %+.4f  sd(t) %.4f  sqrtP44 %.4f\n', ...
                scn{is}, seed, zoom_w, mean(at(m) - ah{1}(m)), std(at(m) - ah{1}(m)), mean(sp{1}(m)), mean(at(m) - ah{2}(m)), std(at(m) - ah{2}(m)), mean(sp{2}(m)));

        f = figure('Position', [80 80 1300 1050], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
        tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
        nexttile; hold on; box on;
        h0 = plot(NaN, NaN, 'w.', 'DisplayName', desc);
        h1 = plot(t, at, '-', 'Color', COL_TRUE, 'LineWidth', 2.0, 'DisplayName', 'a_{true} / a_{nom}');
        h2 = plot(t, ah{1}, '-', 'Color', COL_REAL, 'LineWidth', 1.4, 'DisplayName', 'a_{hat}, \lambda_f on a + \delta a');
        h3 = plot(t, ah{2}, '-', 'Color', COL_OR, 'LineWidth', 1.4, 'DisplayName', 'a_{hat}, \lambda_f on \delta a only');
        local_seg(seg); xlim([0 t(end)]); ylim([0 1.1]);
        ylabel('a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold');
        legend([h0 h1 h2 h3], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on'); local_ax(FS, AXLW);
        nexttile; hold on; box on;
        h4 = plot(t, at - ah{1}, '-', 'Color', COL_REAL, 'LineWidth', 1.4, 'DisplayName', 'e_a = a_{true} - a_{hat}, \lambda_f on a + \delta a');
        h5 = plot(t, at - ah{2}, '-', 'Color', COL_OR, 'LineWidth', 1.4, 'DisplayName', 'e_a, \lambda_f on \delta a only');
        h6 = plot(t, sp{1}, '--', 'Color', COL_REAL, 'LineWidth', 1.0, 'DisplayName', '\pm sqrt(P_{44})');
        plot(t, -sp{1}, '--', 'Color', COL_REAL, 'LineWidth', 1.0);
        yline(0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off'); local_seg(seg);
        xlim([0 t(end)]); ylim([-0.25 0.25]);
        ylabel('e_a  [a / a_{nom}]', 'FontSize', FS, 'FontWeight', 'bold');
        legend([h4 h5 h6], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on'); local_ax(FS, AXLW);
        nexttile; hold on; box on;
        h7 = plot(t(m), at(m) - ah{1}(m), '-', 'Color', COL_REAL, 'LineWidth', 1.4, 'DisplayName', 'e_a, \lambda_f on a + \delta a  (steady-state zoom)');
        h8 = plot(t(m), at(m) - ah{2}(m), '-', 'Color', COL_OR, 'LineWidth', 1.4, 'DisplayName', 'e_a, \lambda_f on \delta a only');
        h9 = plot(t(m), sp{1}(m), '--', 'Color', COL_REAL, 'LineWidth', 1.0, 'DisplayName', '\pm sqrt(P_{44}), a + \delta a');
        plot(t(m), -sp{1}(m), '--', 'Color', COL_REAL, 'LineWidth', 1.0);
        h10 = plot(t(m), sp{2}(m), '--', 'Color', COL_OR, 'LineWidth', 1.0, 'DisplayName', '\pm sqrt(P_{44}), \lambda_f on \delta a only');
        plot(t(m), -sp{2}(m), '--', 'Color', COL_OR, 'LineWidth', 1.0);
        yline(0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off');
        yz = 1.3 * max([abs(at(m) - ah{1}(m)); abs(at(m) - ah{2}(m)); sp{1}(m)]);
        xlim(zoom_w); ylim([-yz yz]);
        ylabel('e_a  [a / a_{nom}]', 'FontSize', FS, 'FontWeight', 'bold'); xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        legend([h7 h8 h9 h10], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on'); local_ax(FS, AXLW);
        fn = fullfile(od, sprintf('fig_noap_lfslots_%s_s%d.png', scn{is}, seed));
        exportgraphics(f, fn, 'Resolution', 150);  copyfile(fn, fullfile(fd, sprintf('fig_noap_lfslots_%s_s%d.png', scn{is}, seed)));  close(f);
        fprintf('figure: %s\n', fn);
    end
end
function local_seg(seg)
    for s = seg; xline(s, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, 'HandleVisibility', 'off'); end
end
function local_ax(FS, AXLW)
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
end
