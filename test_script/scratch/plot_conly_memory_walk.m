% PURPOSE (2026-10-05, user: "why does a fixed P_fix mean old vs new? the weight explanation is too abstract"): scalar walk of the
%   c update through the last 0.1 s of the ramp_tri descent on the cell (160 steps, I = 1 per step, i.e. each step's reading has the
%   variance 1/I in c units), readings noise-free at the cell's c = 0.70, start c_hat = 0.05 (plane seed).
%   Q55 = 0: P starts at 1/(100 + 67) (prior 100 units + the far-field 67 units of fig_conly_I_rtri), then 1/P grows by I each step.
%   Q55 = P_fix^2 I, sqrt(P_fix) = 0.15: P^f stays at P_fix, gain K = P_fix I / (1 + P_fix I) every step.
%   Left: c_hat over the 160 steps.  Right: how much each of the 160 readings counts in the final c_hat.   STATUS: SCRATCH
function plot_conly_memory_walk()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    N = 160;  I = 1;  c_true = 0.70;  c0 = 0.05;  Pfix = 0.15^2;
    % Q55 = 0
    c_a = zeros(1, N + 1);  c_a(1) = c0;  infoA = 167;  KA = zeros(1, N);
    for k = 1:N; infoA = infoA + I; KA(k) = I / infoA; c_a(k + 1) = c_a(k) + KA(k) * (c_true - c_a(k)); end
    % Q55 = P_fix^2 I
    c_b = zeros(1, N + 1);  c_b(1) = c0;  KB = Pfix * I / (1 + Pfix * I);
    for k = 1:N; c_b(k + 1) = c_b(k) + KB * (c_true - c_b(k)); end
    % weight of reading j in the final c_hat: K_j * prod_{i > j} (1 - K_i)
    wA = zeros(1, N);  wB = zeros(1, N);
    for j = 1:N; wA(j) = KA(j) * prod(1 - KA(j + 1:end)); wB(j) = KB * (1 - KB)^(N - j); end
    fprintf('Q55 = 0:      c_hat after 160 steps %.3f, gain step 1 %.4f -> step 160 %.4f, all readings weigh %.4f each\n', c_a(end), KA(1), KA(end), wA(end));
    fprintf('P_fix 0.15:   c_hat after 160 steps %.3f, gain every step %.4f, newest reading weighs %.4f, reading 44 steps old %.4f (x%.2f)\n', ...
            c_b(end), KB, wB(end), wB(end - 44), wB(end - 44) / wB(end));
    FS = 18;  LFS = 13;  AXLW = 2.0;
    f = figure('Position', [40 40 1600 650], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax1 = nexttile(tl);  hold on; box on;
    plot(0:N, c_true * ones(1, N + 1), '-', 'Color', [0.8 0 0], 'LineWidth', 2.4, 'DisplayName', 'c the cell needs (0.70)');
    plot(0:N, c_a, '-', 'Color', [0.6 0.6 0.6], 'LineWidth', 2.2, 'DisplayName', 'c_{hat}, Q_{55} = 0');
    plot(0:N, c_b, '-', 'Color', [0 0.2 0.9], 'LineWidth', 2.2, 'DisplayName', 'c_{hat}, \surdP_{fix} = 0.15');
    xlabel('step in the last 0.1 s of the descent', 'FontSize', FS, 'FontWeight', 'bold');  ylabel('c', 'FontSize', FS, 'FontWeight', 'bold');
    legend('Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
    xlim([0 N]);  ylim([0 0.8]);  set(ax1, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(ax1, 'off');
    ax2 = nexttile(tl);  hold on; box on;
    plot(1:N, wA, '-', 'Color', [0.6 0.6 0.6], 'LineWidth', 2.2, 'DisplayName', 'Q_{55} = 0: every reading the same');
    plot(1:N, wB, '-', 'Color', [0 0.2 0.9], 'LineWidth', 2.2, 'DisplayName', '\surdP_{fix} = 0.15: older readings discounted');
    xlabel('reading number (160 = newest)', 'FontSize', FS, 'FontWeight', 'bold');  ylabel('share in the final c_{hat}', 'FontSize', FS, 'FontWeight', 'bold');
    legend('Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
    set(ax2, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(ax2, 'off');
    out = fullfile(fd, 'fig_conly_memory_walk.png');  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end
