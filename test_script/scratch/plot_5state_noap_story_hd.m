% PURPOSE (2026-10-01, user: "compare against a(h_d), not a_true"): same runs and layout as fig_noap_story3 (Q44 = 0 vs
%   epsilon = Q44/R2 = 1e-5, plane, T_d = 0.5 | 5 s, 10 seeds), but the reference is the gain at the COMMANDED height,
%   a(h_d) = a_nom / c_perp(h_d) (deterministic, seed-free), and e = a(h_d) - a_hat. a_true (gain at the true height) is
%   drawn thin for comparison. h_d is read from one short run per T_d (the command does not depend on the seed).
%   Reads story_plane_ramp<T_d>_{q0w,rho}_s*.mat.   STATUS: SCRATCH
function plot_5state_noap_story_hd(TDs)
    if nargin < 1; TDs = [0.5 5]; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));  addpath(fullfile(root, 'test_script', 'integration'));
    id = fullfile(root, 'test_results', 'temp_5state_noap');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    pc = physical_constants();  R = pc.R;  a_nom = pc.Ts / pc.gamma_N;
    ARMS = {'q0w', 'Q_{44} = 0', [0.4 0.4 0.4]; 'rho', '\epsilon = Q_{44}/R_2 = 10^{-5}', [0 0.2 0.9]};
    COL_REF = [0.8 0 0];  FS = 18;  LFS = 12;  AXLW = 2.0;
    f = figure('Position', [40 40 1700 1000], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for c = 1:2
        T_d = TDs(c);  stag = strrep(sprintf('ramp%g', T_d), '.', 'p');
        % commanded height -> a(h_d)
        cfg = canonical_scenario(0.05, 1.10, 'deep');  cfg.eq17_variant = '5state_noap';  cfg.amplitude = 0;
        cfg.trajectory_type = 'ramp_descent';  cfg.h_init = 50;  cfg.h_bottom = 1.10 * R;  cfg.h_min = 1.10 * R;
        cfg.t_hold = 0.5;  cfg.t_descend_override = T_d;  cfg.T_sim = 0.5 + T_d + 2.0;
        s = run_pure_simulation(cfg, struct('seed', 1, 'ctrl_const_override', struct('lf_mode', 'const', 'lambda_f', 1, 'gain_order', 1, 'seed_mode', 'true_h0')));
        hd = s.p_d_out(:, 3) / R;  ahd = zeros(size(hd));
        for k = 1:numel(hd); [~, cp] = calc_correction_functions(max(hd(k), 1.0001)); ahd(k) = 1 / cp; end
        nexttile(c); hold on; box on;  h0 = plot(NaN, NaN, 'w.', 'DisplayName', sprintf('plane wall, plane init, 22.2 -> 1.10 R in T_d = %g s, 10 seeds', T_d));
        hl = h0;
        for m = 1:2
            AH = [];  AT = [];
            for q = 1:10
                r = load(fullfile(id, sprintf('story_plane_%s_%s_s%d.mat', stag, ARMS{m, 1}, q)));
                AH(:, q) = r.a_hat / a_nom;  AT(:, q) = r.a_true / a_nom; %#ok<AGROW>
            end
            t = r.t - 0.5;  n = min(numel(t), numel(ahd));  t = t(1:n);  AH = AH(1:n, :);  AT = AT(1:n, :);  ah_d = ahd(1:n);
            E = ah_d - AH;  em = mean(E, 2);  es = std(E, 0, 2);  am = mean(AH, 2);  as = std(AH, 0, 2);
            i1 = find(t <= T_d, 1, 'last');  i2 = find(t <= T_d + 0.5, 1, 'last');  desc = t >= 0 & t <= T_d;
            fprintf('T_d %-4g %-4s | vs a(h_d): stop %+.3f +- %.3f  +0.5 s %+.3f  rms desc %.4f | vs a_true: stop %+.3f  rms desc %.4f | a(h_d) - a_true at stop %+.3f\n', ...
                T_d, ARMS{m, 1}, em(i1), es(i1), em(i2), sqrt(mean(E(desc, :).^2, 'all')), mean(AT(i1, :) - AH(i1, :)), ...
                sqrt(mean((AT(desc, :) - AH(desc, :)).^2, 'all')), ah_d(i1) - mean(AT(i1, :)));
            nexttile(c);
            fill([t; flipud(t)], [am + as; flipud(am - as)], ARMS{m, 3}, 'FaceAlpha', 0.18, 'EdgeColor', 'none', 'HandleVisibility', 'off');
            hl(end + 1) = plot(t, am, '-', 'Color', ARMS{m, 3}, 'LineWidth', 1.8, 'DisplayName', ['a_{hat}, ' ARMS{m, 2}]); %#ok<AGROW>
            if m == 1
                hr = plot(t, ah_d, '-', 'Color', COL_REF, 'LineWidth', 2.2, 'DisplayName', 'a(h_d), commanded height');
                htr = plot(t, mean(AT, 2), ':', 'Color', COL_REF, 'LineWidth', 1.6, 'DisplayName', 'a_{true}, true height');
            end
            nexttile(2 + c); hold on; box on;
            fill([t; flipud(t)], [em + es; flipud(em - es)], ARMS{m, 3}, 'FaceAlpha', 0.18, 'EdgeColor', 'none', 'HandleVisibility', 'off');
            plot(t, em, '-', 'Color', ARMS{m, 3}, 'LineWidth', 1.8);
        end
        nexttile(c);  ylim([0 1.1]);  local_seg(T_d);  xlim([t(1) t(end)]);
        if c == 1; ylabel('a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold'); end
        legend([hl(1) hr htr hl(2:end)], 'Location', 'northoutside', 'Orientation', 'horizontal', 'NumColumns', 2, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');
        nexttile(2 + c);  yline(0, '-', 'Color', [0.6 0.6 0.6], 'HandleVisibility', 'off');  ylim([-0.9 0.2]);  local_seg(T_d);  xlim([t(1) t(end)]);
        if c == 1; ylabel('a(h_d) - a_{hat}', 'FontSize', FS, 'FontWeight', 'bold'); end
        xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');
    end
    out = fullfile(fd, sprintf('fig_noap_story3_hd_plane%s.png', strrep(sprintf('_td%g_%g', TDs), '.', 'p')));
    exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end

function local_seg(T_d)
    yl = ylim;  for x = [0 T_d]; plot([x x], yl, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2, 'HandleVisibility', 'off'); end;  ylim(yl);
end
