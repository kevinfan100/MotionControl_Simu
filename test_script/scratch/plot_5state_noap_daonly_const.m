% PURPOSE (2026-09-22, user: "tentatively lf on delta a only; compare with a constant lf, dynamic trajectory only"):
%   slow ramp (15 -> 2.5 um over 10 s, cheat init), three arms with forgetting on the RATE slot only (lf_slots = 5):
%     schedule       Ljung T* from command speed x height (clamp 0.98)
%     const 0.9995   lambda_f fixed (memory 1.25 s)
%     const 0.998    lambda_f fixed (memory 0.31 s)
%   Figure: single seed, rows a_true/a_hat | e_a with +-sqrt(P44) of the schedule arm | steady-state zoom. Console: 5-seed table.
%   Output test_results/temp_5state_noap/fig_noap_daonly_const_meng_s<seed>.png | EXPIRES: with the layer-1 line
function plot_5state_noap_daonly_const(seed, seeds_tab)
    if nargin < 1 || isempty(seed); seed = 1; end
    if nargin < 2 || isempty(seeds_tab); seeds_tab = 1:5; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));  addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'temp_5state_noap');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    pc = physical_constants();  a_nom = pc.Ts / pc.gamma_N;
    COL_TRUE = [0.8 0 0]; ARM = {[0 0.2 0.9], [0.85 0.45 0], [0.1 0.6 0.2]}; FS = 18; LFS = 13; AXLW = 2.0;
    arms = {'schedule', struct('lf_mode', 'ljung'); 'const 0.9995', struct('lf_mode', 'const', 'lambda_f', 0.9995); 'const 0.998', struct('lf_mode', 'const', 'lambda_f', 0.998)};
    labs = {'\lambda_f schedule (\delta a only)', '\lambda_f = 0.9995 (\delta a only)', '\lambda_f = 0.998 (\delta a only)'};
    cfg = canonical_scenario(0.05, 1.10, 'deep');  cfg.eq17_variant = '5state_noap';
    cfg.h_init = 15;  cfg.h_bottom = 2.5;  cfg.amplitude = 0;  cfg.frequency = 1;  cfg.n_cycles = 1;
    cfg.t_hold = 0.5;  cfg.t_descend_override = 10;  cfg.T_sim = 12.5;  cfg.h_min = 2.475;
    % ---- 5-seed table ----
    fprintf('%-14s | ramp: |e| rms  max | rel rms | hold-end mean  sd  sqrtP44 (hon)\n', 'arm (da only)');
    for ia = 1:3
        E = []; A = []; P = [];
        for sd = seeds_tab
            s = local_run(cfg, sd, arms{ia, 2});  t = s.tout(:);
            E(:, end + 1) = (s.a_true_out(:, 3) - s.diag.a_hat(:, 3)) / a_nom;  A(:, end + 1) = s.a_true_out(:, 3) / a_nom;  P(:, end + 1) = s.diag.P_a(:, 3) / a_nom^2; %#ok<AGROW>
        end
        em = mean(E, 2); a = mean(A, 2); mo = t >= 1.5 & t < 10.5; he = t >= 11.5;
        fprintf('%-14s | %.3f  %.3f | %.3f | %+.4f  %.4f  %.4f (%.2f)\n', arms{ia, 1}, rms(em(mo)), max(abs(em(mo))), rms(em(mo) ./ a(mo)), ...
                mean(em(he)), mean(std(E(he, :), 0, 2)), mean(sqrt(P(he, :)), 'all'), mean(std(E(he, :), 0, 2)) / mean(sqrt(P(he, :)), 'all'));
    end
    % ---- single-seed figure ----
    S = cell(3, 1);  for ia = 1:3; S{ia} = local_run(cfg, seed, arms{ia, 2}); end
    t = S{1}.tout(:);  at = S{1}.a_true_out(:, 3) / a_nom;  seg = [0.5 10.5];  zoom_w = [10.5 12.5];  m = t >= zoom_w(1) & t <= zoom_w(2);
    f = figure('Position', [80 80 1300 1050], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    nexttile; hold on; box on;  hs = gobjects(0);
    h0 = plot(NaN, NaN, 'w.', 'DisplayName', sprintf('slow ramp 15 -> 2.5 um over 10 s, init a(h_0), seed %d', seed));
    hs(end + 1) = plot(t, at, '-', 'Color', COL_TRUE, 'LineWidth', 2.0, 'DisplayName', 'a_{true} / a_{nom}'); %#ok<AGROW>
    for i = 1:3; hs(end + 1) = plot(t, S{i}.diag.a_hat(:, 3) / a_nom, '-', 'Color', ARM{i}, 'LineWidth', 1.3, 'DisplayName', labs{i}); end %#ok<AGROW>
    local_seg(seg); xlim([0 t(end)]); ylim([0 1.1]); ylabel('a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold');
    legend([h0 hs], 'Location', 'northoutside', 'Orientation', 'horizontal', 'NumColumns', 3, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on'); local_ax(FS, AXLW);
    nexttile; hold on; box on;  hs = gobjects(0);
    for i = 1:3; hs(end + 1) = plot(t, at - S{i}.diag.a_hat(:, 3) / a_nom, '-', 'Color', ARM{i}, 'LineWidth', 1.3, 'DisplayName', ['e_a, ' labs{i}]); end %#ok<AGROW>
    sp = sqrt(S{1}.diag.P_a(:, 3)) / a_nom;
    hs(end + 1) = plot(t, sp, '--', 'Color', ARM{1}, 'LineWidth', 1.0, 'DisplayName', '\pm sqrt(P_{44}), schedule'); plot(t, -sp, '--', 'Color', ARM{1}, 'LineWidth', 1.0); %#ok<AGROW>
    yline(0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off'); local_seg(seg);
    xlim([0 t(end)]); ylim([-0.25 0.25]); ylabel('e_a  [a / a_{nom}]', 'FontSize', FS, 'FontWeight', 'bold');
    legend(hs, 'Location', 'northoutside', 'Orientation', 'horizontal', 'NumColumns', 2, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on'); local_ax(FS, AXLW);
    nexttile; hold on; box on;  hs = gobjects(0);  yz = 0;
    for i = 1:3
        e = at(m) - S{i}.diag.a_hat(m, 3) / a_nom;  yz = max(yz, max(abs(e)));
        hs(end + 1) = plot(t(m), e, '-', 'Color', ARM{i}, 'LineWidth', 1.3, 'DisplayName', ['e_a, ' labs{i}]); %#ok<AGROW>
        spi = sqrt(S{i}.diag.P_a(m, 3)) / a_nom;  plot(t(m), spi, '--', 'Color', ARM{i}, 'LineWidth', 1.0, 'HandleVisibility', 'off'); plot(t(m), -spi, '--', 'Color', ARM{i}, 'LineWidth', 1.0, 'HandleVisibility', 'off');
    end
    hs(end + 1) = plot(NaN, NaN, '--', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.0, 'DisplayName', 'dashed: \pm sqrt(P_{44}) of each arm'); %#ok<AGROW>
    yline(0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off');
    xlim(zoom_w); ylim(1.3 * [-yz yz]); ylabel('e_a  [a / a_{nom}]', 'FontSize', FS, 'FontWeight', 'bold'); xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    legend(hs, 'Location', 'northoutside', 'Orientation', 'horizontal', 'NumColumns', 2, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on'); local_ax(FS, AXLW);
    fn = fullfile(od, sprintf('fig_noap_daonly_const_meng_s%d.png', seed));
    exportgraphics(f, fn, 'Resolution', 150);  copyfile(fn, fullfile(fd, sprintf('fig_noap_daonly_const_meng_s%d.png', seed)));  close(f);  fprintf('figure: %s\n', fn);
end
function s = local_run(cfg, sd, cc)
    cc.seed_mode = 'true_h0';  cc.lf_slots = 5;
    s = run_pure_simulation(cfg, struct('seed', sd, 'collect_diag', true, 'ctrl_const_override', cc));
end
function local_seg(seg)
    for s = seg; xline(s, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, 'HandleVisibility', 'off'); end
end
function local_ax(FS, AXLW)
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
end
