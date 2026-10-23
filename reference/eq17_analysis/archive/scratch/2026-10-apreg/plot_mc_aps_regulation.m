% STATUS: SCRATCH -- 2026-10-15 (user): what the a'-state estimator does inside the gain regulation. Seed 1, a' as a state with
%   P_fix ~ 1/h_d^2 (ap_slot, q77_adapt_P0 0.4, q77_hscale 1.1, fe_cmd_eval), lambda_a = 1 - Ts/0.1 s. Columns: plane a_d 0.5 (runs the
%   full 6 s with this seed), cell a_d 0.6 (run kept up to 2 s; the particle touches the wall between 2.0 and 2.5 s).
%   Rows: height z_d (red) / z (blue dashed) [R]; a-bar_z true (red), estimate (blue), a_d (black dashed); slope: true a'(h_true) (red),
%   estimate a'-hat = slot 7 (blue), black line at 0.
function plot_mc_aps_regulation()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(genpath(fullfile(root, 'model')));
    od = fullfile(root, 'test_results', 'temp_1009_ws');  WL = {'plane', 'cell'};  AD = [0.5 0.6];
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  FS = 18;  LFS = 14;  AB = char(257);  HAT = char(770);
    f = figure('Position', [40 40 2000 1450], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for iw = 1:2
        S = load(fullfile(od, sprintf('gainreg_%s_aps_beforecontact.mat', WL{iw})));  t = S.t;  wd = S.wd;  wd(1) = wd(2);
        a1 = nexttile(tl, iw);  hold on; box on;
        plot(a1, t, wd, '-', 'Color', cT, 'LineWidth', 3.5, 'DisplayName', 'h_d');  plot(a1, t, S.h_true, '--', 'Color', cE, 'LineWidth', 1.6, 'DisplayName', 'h');
        set(a1, 'YScale', 'log');  ylim(a1, [1 30]);  yticks(a1, [1 1.5 2 3 5 10 20]);
        text(a1, 0.97, 0.88, sprintf('%s, %s_d = %.1f', WL{iw}, AB, AD(iw)), 'Units', 'normalized', 'HorizontalAlignment', 'right', 'FontSize', LFS + 3, 'FontWeight', 'bold');
        a2 = nexttile(tl, 2 + iw);  hold on; box on;
        plot(a2, t, S.a_true, '-', 'Color', cT, 'LineWidth', 3.5, 'DisplayName', [AB '_z']);  plot(a2, t, S.a_hat, '-', 'Color', cE, 'LineWidth', 1.6, 'DisplayName', [AB HAT '_z']);
        yline(a2, AD(iw), '--', 'Color', 'k', 'LineWidth', 2, 'DisplayName', [AB '_d']);  ylim(a2, [0.4 1]);
        a3 = nexttile(tl, 4 + iw);  hold on; box on;
        apt = local_ap(max(S.h_true, 1.001), WL{iw});
        plot(a3, t, apt, '-', 'Color', cT, 'LineWidth', 3.5, 'DisplayName', [AB ''' true']);  plot(a3, t, -S.w_hat, '-', 'Color', cE, 'LineWidth', 1.6, 'DisplayName', [AB '''' HAT]);
        yline(a3, 0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.2, 'HandleVisibility', 'off');  ylim(a3, [-0.1 0.35]);
        xlabel(a3, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        if iw == 1; ylabel(a1, 'h  [R]', 'FontSize', FS, 'FontWeight', 'bold');  ylabel(a2, [AB '_z'], 'FontSize', FS, 'FontWeight', 'bold');  ylabel(a3, [AB ''''], 'FontSize', FS, 'FontWeight', 'bold'); end
        for a = [a1 a2 a3]; xlim(a, [0 t(end)]);  legend(a, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
            set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2);  grid(a, 'off'); end
    end
    out = fullfile(od, 'fig_mc_aps_regulation.png');  exportgraphics(f, out, 'Resolution', 120);  fprintf('figure: %s\n', out);
end

function ap = local_ap(h, plant)
    d = 1e-4;  ap = (local_a(h + d, plant) - local_a(h - d, plant)) / (2 * d);
end

function a = local_a(h, plant)
    if strcmp(plant, 'cell'); a = 1 ./ cell_cperp_delta(h); return; end
    a = zeros(size(h));  for i = 1:numel(h); [~, c] = calc_correction_functions(h(i)); a(i) = 1 / c; end
end
