% PURPOSE (2026-09-20, user question): on the slow ramp (meng) the layer-1 estimator shows (a) a start transient
%   (e_a wiggle +-0.15 in the first second) and (b) a lag error growing to +0.17 before the low point. Measure, do not argue:
%     rows  1  e_a and the two cumulative channel contributions sum K41*innov1 (y1) and sum K42*innov2 (y2) to a_hat
%           2  sqrt(P44), sqrt(P55) against the per-step |true increment| (can the rate slot still learn the rate?)
%           3  rate estimate vs true increment (zoom on the ramp)
%           4  e_a of three arms: baseline prior | tight level prior (0.05 a_nom) | tight rate prior (a_nom/4000)
%   Cost ~1 min (3 runs x 12.5 s). EXPIRES: with the layer-1 line | 產線改動不會自動跟上
function probe_5state_noap_meng_lag(seed)
    if nargin < 1; seed = 1; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));  addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'temp_5state_noap');
    pc = physical_constants();  a_nom = pc.Ts / pc.gamma_N;
    FS = 16;  LFS = 13;  C_TRUE = [0.85 0.1 0.1];  C_HAT = [0.1 0.3 0.85];  C_Y1 = [0.1 0.6 0.2];  C_Y2 = [0.9 0.5 0.1];
    arms = {'baseline (0.5, 1/400)', struct(); 'tight level prior 0.05', struct('Pf_a_std_rel', 0.05); 'tight rate prior 1/4000', struct('Pf_da_std_rel', 1/4000)};
    S = cell(3, 1);
    for ia = 1:3
        cfg = canonical_scenario(0.05, 1.10, 'deep');  cfg.eq17_variant = '5state_noap';
        cfg.h_init = 15;  cfg.h_bottom = 2.5;  cfg.amplitude = 0;  cfg.frequency = 1;  cfg.n_cycles = 1;
        cfg.t_hold = 0.5;  cfg.t_descend_override = 10;  cfg.T_sim = 12.5;  cfg.h_min = 2.475;
        cc = arms{ia, 2};  cc.seed_mode = 'true_h0';
        S{ia} = run_pure_simulation(cfg, struct('seed', seed, 'collect_diag', true, 'ctrl_const_override', cc));
    end
    s = S{1};  t = s.tout(:);  d = s.diag;
    e = (d.a_hat(:, 3) - s.a_true_out(:, 3)) / a_nom;
    c1 = cumsum(d.K_kf_a_y1(:, 3) .* d.innovation_y1(:, 3)) / a_nom;
    c2 = cumsum(d.K_kf_a_y2(:, 3) .* d.innovation_y2(:, 3)) / a_nom;
    rate_true = [diff(s.a_true_out(:, 3)); 0] / a_nom;  rate_sm = movmean(rate_true, 401);
    % where does the lag peak sit relative to the true rate?
    [emax, imax] = max(e(t > 2));  imax = imax + find(t > 2, 1) - 1;
    [~, irate] = min(rate_sm);
    fprintf('seed %d | lag peak e_a %+.3f at t = %.2f s | fastest true descent of a at t = %.2f s (%.2e/step) | sqrtP55 there %.2e | sqrtP44 there %.4f\n', ...
            seed, emax, t(imax), t(irate), rate_sm(irate), sqrt(d.P_da(irate, 3)) / a_nom, sqrt(d.P_a(irate, 3)) / a_nom);
    % effective lag: time shift that best aligns a_hat with a_true on the ramp
    m = t >= 4 & t <= 10.5;  at = s.a_true_out(:, 3) / a_nom;  ah = d.a_hat(:, 3) / a_nom;  best = inf;  tau_best = 0;
    for tau = 0:0.02:2
        k = round(tau / pc.Ts);  idx = find(m);  idx = idx(idx - k >= 1);
        r = mean((ah(idx) - at(idx - k)).^2);  if r < best; best = r; tau_best = tau; end
    end
    fprintf('best time shift a_hat(t) ~ a_true(t - tau): tau = %.2f s (rms %.4f)\n', tau_best, sqrt(best));
    % start transient: contribution split in the first second, and the arms
    m1 = t <= 1.0;
    fprintf('first 1 s: |e_a| max %.3f | y1 cum contribution range %.3f | y2 cum range %.3f | rate excursion max |da_hat| %.2e/step\n', ...
            max(abs(e(m1))), range(c1(m1)), range(c2(m1)), max(abs(d.delta_a_hat(m1, 3))) / a_nom);
    for ia = 1:3
        ea = (S{ia}.diag.a_hat(:, 3) - S{ia}.a_true_out(:, 3)) / a_nom;
        fprintf('  %-28s | first 1 s max|e_a| %.3f | lag peak %+.3f | hold-end mean %+.4f\n', arms{ia, 1}, max(abs(ea(m1))), max(ea(t > 2)), mean(ea(t > 11.5)));
    end

    f = figure('Position', [60 60 1300 1150], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    nexttile; hold on;
    plot(t, e, '-', 'Color', C_HAT, 'LineWidth', 2);  plot(t, c1, '-', 'Color', C_Y1, 'LineWidth', 1.5);  plot(t, c2, '-', 'Color', C_Y2, 'LineWidth', 1.5);
    yline(0, '-', 'Color', [.6 .6 .6], 'HandleVisibility', 'off');
    ylabel('/ a_{nom}', 'FontSize', FS, 'FontWeight', 'bold');
    legend({'e_a = \^a - a', '\Sigma K_{41} innov_1 (y_1 path)', '\Sigma K_{42} innov_2 (y_2 path)'}, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
    local_ax(FS);
    nexttile; hold on;
    semilogy(t, sqrt(d.P_a(:, 3)) / a_nom, '-', 'Color', C_HAT, 'LineWidth', 2);
    semilogy(t, sqrt(d.P_da(:, 3)) / a_nom, '--', 'Color', C_HAT, 'LineWidth', 2);
    semilogy(t, abs(rate_sm), '-', 'Color', C_TRUE, 'LineWidth', 2);
    set(gca, 'YScale', 'log');  ylabel('/ a_{nom}', 'FontSize', FS, 'FontWeight', 'bold');
    legend({'\surdP_{44}', '\surdP_{55} (per step)', '|true increment| per step (0.25 s mean)'}, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
    local_ax(FS);
    nexttile; hold on;
    plot(t, rate_sm * 1e3, '-', 'Color', C_TRUE, 'LineWidth', 2);  plot(t, d.delta_a_hat(:, 3) / a_nom * 1e3, '-', 'Color', C_HAT, 'LineWidth', 2);
    ylabel('10^{-3} / step', 'FontSize', FS, 'FontWeight', 'bold');
    legend({'True increment (0.25 s mean)', 'Rate estimate \delta\^a'}, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
    local_ax(FS);
    nexttile; hold on;
    cols = {C_HAT, C_Y1, C_Y2};
    for ia = 1:3
        ea = (S{ia}.diag.a_hat(:, 3) - S{ia}.a_true_out(:, 3)) / a_nom;
        plot(t, ea, '-', 'Color', cols{ia}, 'LineWidth', 1.8);
    end
    yline(0, '-', 'Color', [.6 .6 .6], 'HandleVisibility', 'off');
    ylabel('e_a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold');  xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    legend(arms(:, 1)', 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
    local_ax(FS);
    fn = fullfile(od, sprintf('fig_noap_meng_lag_s%d.png', seed));
    exportgraphics(f, fn, 'Resolution', 150);  close(f);  fprintf('saved %s\n', fn);
end
function local_ax(FS)
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2, 'Box', 'on'); grid off;
end
