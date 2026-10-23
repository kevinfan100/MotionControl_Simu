function out = compare_fe_row4_full(opts)
%COMPARE_FE_ROW4_FULL  Single-flag A/B: fe_row4_full ON (production) vs OFF.
%   Everything else stays at production defaults (arm best, canonical deep,
%   y2_whiten / y2_echo_corr / ma2_aug / use_fdet all on). Same seeds on both
%   sides so the thermal draws cancel. z axis.
%
%   fe_row4_full = true : the row-4 increment the law slope multiplies is
%       M = Delta_wbar_d[k-1] + (1-lambda_c)*dw3_hat   (command step + tracking-error correction)
%   fe_row4_full = false: M = Delta_wbar_d[k-1] only. The parameter columns
%       F_e(4,4), F_e(4,5) are proportional to M, so with OFF they vanish at every
%       trajectory turning point (Delta_wbar_d = 0).
%
%   Output: test_results/flag_ab/fe_row4_full_ab.png + console table.

    if nargin < 1; opts = struct(); end
    if ~isfield(opts, 'seeds'); opts.seeds = 1:8; end
    root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    cd(root); local_addpaths(root);
    od = fullfile(root, 'test_results', 'flag_ab'); if ~exist(od, 'dir'); mkdir(od); end
    AX = 3; seeds = opts.seeds; n = numel(seeds);

    evalc('ON  = run_formC_b(struct(''seeds'', seeds, ''verbose'', false));');
    evalc('OFF = run_formC_b(struct(''seeds'', seeds, ''verbose'', false, ''ctrl_const_override'', struct(''fe_row4_full'', false)));');

    t = ON.runs{1}.tout; a_nom = ON.runs{1}.a_nom; N = numel(t);
    A = zeros(N, n, 2); B = zeros(N, n, 2); S = zeros(N, n, 2); AT = zeros(N, n); G = zeros(N, n, 2);
    for i = 1:n
        for arm = 1:2
            if arm == 1; r = ON.runs{i}; else; r = OFF.runs{i}; end
            AT(:, i) = r.a_true_out(:, AX) / a_nom;
            A(:, i, arm) = r.a_bar_hat_out(:, AX);
            B(:, i, arm) = r.b_hat_out(:, AX);
            S(:, i, arm) = r.P_a_out(:, AX) / a_nom;
            G(:, i, arm) = r.gate_out(:, AX);
        end
    end
    E = A - AT;                                   % estimate error, a_bar units
    w = t > t(end) - 0.9;                         % end hold
    seg = {t >= 0 & t < 0.5, 'hold0'; t >= 0.5 & t < 1.5, 'descent'; t >= 1.5 & t < 3.5, 'osc'; w, 'hold-end'};

    fprintf('\n=== fe_row4_full A/B, canonical deep, arm best, z, seeds %s ===\n', mat2str(seeds));
    fprintf('%-9s | %10s %10s | %9s %9s | %8s %8s | %8s %8s\n', 'segment', 'bias ON', 'bias OFF', 'pct ON', 'pct OFF', 'sd ON', 'sd OFF', 'hon ON', 'hon OFF');
    res = struct();
    for s = 1:size(seg, 1)
        m = seg{s, 1};
        bias = squeeze(mean(E(m, :, :), 1));                      % n x 2
        pct  = squeeze(mean(E(m, :, :) ./ AT(m, :), 1));
        hon  = squeeze(mean(std(E(m, :, :), 0, 2) ./ mean(S(m, :, :), 2), 1));
        fprintf('%-9s | %+10.5f %+10.5f | %+8.2f%% %+8.2f%% | %8.5f %8.5f | %8.3f %8.3f\n', seg{s, 2}, ...
                mean(bias(:, 1)), mean(bias(:, 2)), 100*mean(pct(:, 1)), 100*mean(pct(:, 2)), ...
                std(bias(:, 1)), std(bias(:, 2)), hon(1), hon(2));
        res.(strrep(seg{s, 2}, '-', '_')) = struct('bias', bias, 'pct', pct, 'hon', hon);
    end
    d = squeeze(mean(E(w, :, 2), 1) - mean(E(w, :, 1), 1));
    fprintf('paired end-hold bias (OFF - ON): %+.5f +- %.5f (SEM), t = %.2f\n', mean(d), std(d)/sqrt(n), mean(d)/(std(d)/sqrt(n)));
    fprintf('sqrt(P44) end-hold: ON %.5f  OFF %.5f\n', mean(mean(S(w, :, 1))), mean(mean(S(w, :, 2))));
    fprintf('b_hat end: ON %.4f +- %.4f  OFF %.4f +- %.4f\n', mean(B(end, :, 1)), std(B(end, :, 1)), mean(B(end, :, 2)), std(B(end, :, 2)));
    fprintf('gate hits z: ON %d  OFF %d ; max|a_hat ON-OFF| over run = %.3e\n', sum(G(:, :, 1), 'all'), sum(G(:, :, 2), 'all'), max(abs(A(:, :, 1) - A(:, :, 2)), [], 'all'));
    fprintf('a_hat rms error whole run: ON %.5f  OFF %.5f\n', sqrt(mean(E(:, :, 1).^2, 'all')), sqrt(mean(E(:, :, 2).^2, 'all')));

    % ---- figure: 3 rows, shared time axis ----
    cON = [0 0.2 0.9]; cOFF = [0.5 0.5 0.5]; cT = [0.85 0.1 0.1];
    f = figure('Position', [60 60 1400 1000], 'Color', 'w', 'Visible', 'off');
    tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    nexttile; hold on;
    H = [plot(t, mean(AT, 2), '-', 'Color', cT, 'LineWidth', 2, 'DisplayName', 'true a_bar_z'), ...
         plot(t, mean(A(:, :, 1), 2), '-', 'Color', cON, 'LineWidth', 1.8, 'DisplayName', 'fe\_row4\_full ON (production)'), ...
         plot(t, mean(A(:, :, 2), 2), '--', 'Color', cOFF, 'LineWidth', 1.8, 'DisplayName', 'fe\_row4\_full OFF')];
    ylabel('a_bar_z (seed mean)'); legend(H, 'Location', 'northoutside', 'Orientation', 'horizontal');
    set(gca, 'FontSize', 16, 'FontWeight', 'bold', 'Box', 'on'); grid off;
    nexttile; hold on;
    for i = 1:n
        plot(t, E(:, i, 1), '-', 'Color', [cON 0.25], 'LineWidth', 0.6, 'HandleVisibility', 'off');
        plot(t, E(:, i, 2), '-', 'Color', [cOFF 0.35], 'LineWidth', 0.6, 'HandleVisibility', 'off');
    end
    H = [plot(t, mean(E(:, :, 1), 2), '-', 'Color', cON, 'LineWidth', 2, 'DisplayName', 'mean error ON'), ...
         plot(t, mean(E(:, :, 2), 2), '--', 'Color', [0.2 0.2 0.2], 'LineWidth', 2, 'DisplayName', 'mean error OFF'), ...
         plot(t,  mean(S(:, :, 1), 2), ':', 'Color', cT, 'LineWidth', 1.5, 'DisplayName', '\pm\surdP_{44} ON')];
    plot(t, -mean(S(:, :, 1), 2), ':', 'Color', cT, 'LineWidth', 1.5, 'HandleVisibility', 'off');
    plot(t,  mean(S(:, :, 2), 2), ':', 'Color', cOFF, 'LineWidth', 1.5, 'HandleVisibility', 'off');
    plot(t, -mean(S(:, :, 2), 2), ':', 'Color', cOFF, 'LineWidth', 1.5, 'HandleVisibility', 'off');
    ylabel('a_bar_z estimate - true'); legend(H, 'Location', 'northoutside', 'Orientation', 'horizontal');
    set(gca, 'FontSize', 16, 'FontWeight', 'bold', 'Box', 'on'); grid off; ylim([-0.06 0.06]);
    nexttile; hold on;
    H = [plot(t, mean(B(:, :, 1), 2), '-', 'Color', cON, 'LineWidth', 2, 'DisplayName', 'b\_hat ON'), ...
         plot(t, mean(B(:, :, 2), 2), '--', 'Color', [0.2 0.2 0.2], 'LineWidth', 2, 'DisplayName', 'b\_hat OFF'), ...
         plot(t, mean(A(:, :, 1) - A(:, :, 2), 2) * 100, '-', 'Color', [0 0.6 0.2], 'LineWidth', 1.5, 'DisplayName', '100 x (a\_hat ON - OFF)')];
    yline(8/9, ':k', 'HandleVisibility', 'off');
    xlabel('t [s]'); ylabel('b_hat  /  100 x delta a_bar');
    legend(H, 'Location', 'northoutside', 'Orientation', 'horizontal');
    set(gca, 'FontSize', 16, 'FontWeight', 'bold', 'Box', 'on'); grid off;
    fn = fullfile(od, 'fe_row4_full_ab.png');
    exportgraphics(f, fn, 'Resolution', 150); close(f);
    fprintf('figure -> %s\n', fn);
    out = struct('t', t, 'A', A, 'B', B, 'S', S, 'AT', AT, 'E', E, 'seeds', seeds, 'res', res);
    save(fullfile(od, 'fe_row4_full_ab.mat'), 'out');
end

function local_addpaths(root)
    p = strsplit(genpath(fullfile(root, 'model')), pathsep);
    p = p(~cellfun('isempty', p)); p = p(~contains(p, [filesep 'archive']));
    addpath(p{:});
    addpath(fullfile(root, 'test_script', 'integration'));
    addpath(fullfile(root, 'test_script', 'build_helpers'));
end
