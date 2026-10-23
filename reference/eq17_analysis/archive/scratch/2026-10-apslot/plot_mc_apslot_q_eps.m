% STATUS: SCRATCH -- 2026-10-19 (user): a' as a state variable with the process noise of a' set like the first-order estimator,
%   q55 = eps R2 (formC_b q77_r2_frac), next to q55 = 0 and q55 = P_fix^2 I (q77_adapt_P0 0.4, q77_hscale 1.1).
%   Constant-velocity trajectory 22.2 -> 1.10 R in 1 s, held 0.2 s (run_mc_apslot_compare 'desc'), plane and cell, 5 seeds.
%   Console: mean|e_a| over the trajectory, e_a at 0.9 / 1.0 s, hold mean, a'-hat / a' true median by band. Figure per wall, seed 1:
%   columns q55 = 0 | eps 1e-5 | eps 1e-3 | P_fix^2 I; rows a-bar_z, a-bar'_z (log). Output test_results/temp_1009_ws/fig_mc_apslot_q_eps_<wall>_s1.png.
function plot_mc_apslot_q_eps(which)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(genpath(fullfile(root, 'model')));
    od = fullfile(root, 'test_results', 'temp_1009_ws');  AB = char(257);  HAT = char(770);
    if nargin < 1 || isempty(which); which = 'r2'; end
    ARMS = {'apP002F000', 'q_{55} = 0'; 'apP002F000E5', 'q_{55} = 10^{-5} R_2'; 'apP002F000E3', 'q_{55} = 10^{-3} R_2'; 'apP002F400H110', 'q_{55} = P_{fix}^2 I'};
    if strcmp(which, 'rel'); ARMS = {'apP002F000', 'q_{55} = 0'; 'apP002F000V5', ['q_{55} = 10^{-5} ' char(257) char(770) '''^2']; 'apP002F000V4', ['q_{55} = 10^{-4} ' char(257) char(770) '''^2']; ...
                                   'apP002F000V3', ['q_{55} = 10^{-3} ' char(257) char(770) '''^2']; 'apP002F400H110', 'q_{55} = P_{fix}^2 I'}; end
    if strcmp(which, 'relbig'); E2 = [char(257) char(770) '''^2']; ARMS = {'apP002F000V2', ['q_{55} = 10^{-2} ' E2]; 'apP002F000V1', ['q_{55} = 10^{-1} ' E2]; ...
            'apP002F000R3', ['q_{55} = (0.09 ' E2 ')^2 I']; 'apP002F000R5', ['q_{55} = (0.25 ' E2 ')^2 I']; 'apP002F000R10', ['q_{55} = (' E2 ')^2 I']; 'apP002F400H110', 'q_{55} = P_{fix}^2 I'}; end
    if strcmp(which, 'relfine'); E2 = [char(257) char(770) '''^2']; ARMS = {'apP002F000V3', ['q_{55} = 10^{-3} ' E2]; 'apP002F000W3', ['q_{55} = 3 10^{-3} ' E2]; ...
            'apP002F000V2', ['q_{55} = 10^{-2} ' E2]; 'apP002F000W30', ['q_{55} = 3 10^{-2} ' E2]; 'apP002F400H110', 'q_{55} = P_{fix}^2 I'}; end
    nA = size(ARMS, 1);
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  FS = 18;  LFS = 13;  AXLW = 2.0;  B = [5 23; 2 5; 1 2];
    for wall = {'plane', 'cell'}
        w = wall{1};  f = figure('Position', [40 40 600 * nA 1000], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
        tl = tiledlayout(2, nA, 'TileSpacing', 'compact', 'Padding', 'compact');
        fprintf('\n[%s] arm            | mean|e_a| 0-1 s | e_a 0.9 s      | e_a 1.0 s      | hold 1.05-1.2 s | a''-hat/a'' h>5, 2-5, <2\n', w);
        for ia = 1:nA
            S = load(fullfile(od, sprintf('apcmp_%s_%s_desc.mat', ARMS{ia, 1}, w)));  t = S.t;  e = S.a_true - S.a_hat;  ap = -S.w_hat;
            apt = local_ap(S.h_true, w);  pk = @(tc) e(find(t >= tc - 1e-9, 1), :);  r = '';
            for ib = 1:3; L = t <= 1 & S.h_true > B(ib, 1) & S.h_true <= B(ib, 2);  r = [r sprintf('%.2f  ', median(ap(L) ./ apt(L)))]; end %#ok<AGROW>
            fprintf('%-15s | %.3f           | %+.3f (%.3f) | %+.3f (%.3f) | %+.3f (%.3f)  | %s\n', ARMS{ia, 1}, mean(mean(abs(e(t <= 1, :)))), mean(pk(0.9)), std(pk(0.9)), ...
                    mean(pk(1.0)), std(pk(1.0)), mean(mean(e(t >= 1.05, :))), std(mean(e(t >= 1.05, :))), r);
            ax = nexttile(tl, ia);  hold on; box on;
            plot(ax, t, S.a_true(:, 1), '-', 'Color', cT, 'LineWidth', 3.5, 'DisplayName', [AB '_z']);  plot(ax, t, S.a_hat(:, 1), '-', 'Color', cE, 'LineWidth', 1.6, 'DisplayName', [AB HAT '_z']);
            ylim(ax, [-0.2 1.2]);  xlim(ax, [0 1.2]);  lg = legend(ax, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
            lg.Title.String = sprintf('%s wall, %s', w, ARMS{ia, 2});  if ia == 1; ylabel(ax, [AB '_z'], 'FontSize', FS + 2, 'FontWeight', 'bold'); end
            ax2 = nexttile(tl, nA + ia);  hold on; box on;  y = ap(:, 1);  y(y <= 0) = NaN;
            plot(ax2, t, apt(:, 1), '-', 'Color', cT, 'LineWidth', 3.5, 'DisplayName', [AB '''_z']);  plot(ax2, t, y, '-', 'Color', cE, 'LineWidth', 1.6, 'DisplayName', [AB HAT '''_z']);
            set(ax2, 'YScale', 'log');  ylim(ax2, [1e-3 3]);  xlim(ax2, [0 1.2]);  xlabel(ax2, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
            legend(ax2, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');  if ia == 1; ylabel(ax2, [AB ''''], 'FontSize', FS + 2, 'FontWeight', 'bold'); end
            for a_ = [ax ax2]; set(a_, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(a_, 'off'); end
        end
        sfx = '';  if ~strcmp(which, 'r2'); sfx = ['_' which]; end
        out = fullfile(od, sprintf('fig_mc_apslot_q_eps%s_%s_s1.png', sfx, w));  exportgraphics(f, out, 'Resolution', 150);  close(f);  fprintf('figure: %s\n', out);
    end
end

function ap = local_ap(h, wall)
    d = 1e-4;  h = max(h, 1.001);  ap = (local_a(h + d, wall) - local_a(h - d, wall)) / (2 * d);
end

function a = local_a(h, wall)
    if strcmp(wall, 'cell'); a = 1 ./ cell_cperp_delta(h); return; end
    a = zeros(size(h));  for i = 1:numel(h); [~, c] = calc_correction_functions(h(i)); a(i) = 1 / c; end
end
