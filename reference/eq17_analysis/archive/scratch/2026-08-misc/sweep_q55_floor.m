function out = sweep_q55_floor(opts)
%SWEEP_Q55_FLOOR  Diagnostic: let slot 5 (= b) carry process noise, Q55 != 0.
%   Production has Q55 = Q_theta_floor = 0 (the "Q_theta = 0 honesty" premise:
%   b is a constant, b_hat moves only through measurement). This sweep asks what
%   happens when b is ALLOWED to walk. It is a diagnostic, not a fix (charter:
%   never open Q to force a state to move).
%
%   Scale anchored to the prior, not guessed: a random walk that diffuses to the
%   prior width Pf_b_std over one run of N steps has
%       Q55_ref = Pf_b_std^2 / N
%   Arms: Q55 = {0, 0.1, 1, 10} x Q55_ref, same 8 seeds, canonical deep, arm best.
%
%   Pre-registered expectations (2026-08-28):
%     1. b_hat leaves 8/9 and tracks b_true(w_bar) locally in motion, drifts in hold
%     2. sqrt(P55) gets a floor instead of shrinking monotonically
%     3. a_hat trough bias (+23 %) moves < 2 pp (08-20: bias is law shape + y1, not b)
%     4. a_hat spread and honesty grow with Q55 (P does not track the randomised b)
%     5. side effects: b floor/ceil hits, gate hits
%
%   Output: test_results/flag_ab/q55_sweep.png + console table.

    if nargin < 1; opts = struct(); end
    if ~isfield(opts, 'seeds');  opts.seeds  = 1:8;          end
    if ~isfield(opts, 'mults');  opts.mults  = [0 0.1 1 10]; end
    if ~isfield(opts, 'replot'); opts.replot = false;       end   % true: redraw from the saved .mat, no runs
    root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    cd(root); local_addpaths(root);
    od = fullfile(root, 'test_results', 'flag_ab'); if ~exist(od, 'dir'); mkdir(od); end
    AX = 3; seeds = opts.seeds; n = numel(seeds); mults = opts.mults; na = numel(mults);

    if opts.replot
        L = load(fullfile(od, 'q55_sweep.mat')); out = L.out;
        t = out.t; A = out.A; B = out.B; S = out.S; SB = out.SB; AT = out.AT; BT = out.BT; E = out.E; seeds = out.seeds; n = numel(seeds);
        mults = out.res.mults; na = numel(mults); w = t > t(end) - 0.9;
        local_fig(t, A, B, S, SB, AT, BT, E, seeds, n, mults, na, w, fullfile(od, 'q55_sweep.png'));
        return;
    end
    % reference run to read the prior width and N
    evalc('R0 = run_formC_b(struct(''seeds'', seeds(1), ''verbose'', false));');
    K0 = R0.runs{1}.ctrl_const; N = numel(R0.runs{1}.tout);
    Pf_b = K0.Pf_b_std; if numel(Pf_b) > 1; Pf_b = Pf_b(AX); end
    b_floor = 0.60; b_ceil = 1.05;
    if isfield(K0, 'b_floor'); b_floor = K0.b_floor; end
    if isfield(K0, 'b_ceil');  b_ceil  = K0.b_ceil;  end
    Q55_ref = Pf_b^2 / N;
    fprintf('Pf_b_std = %.4f  N = %d  ->  Q55_ref = Pf_b_std^2/N = %.3e   (b_floor %.2f, b_ceil %.2f)\n', Pf_b, N, Q55_ref, b_floor, b_ceil);

    runs = cell(na, 1);
    for j = 1:na
        q = mults(j) * Q55_ref;
        evalc('c = run_formC_b(struct(''seeds'', seeds, ''verbose'', false, ''ctrl_const_override'', struct(''Q_theta_floor'', q)));');
        runs{j} = c.runs;
    end
    t = runs{1}{1}.tout; a_nom = runs{1}{1}.a_nom;
    A = zeros(N, n, na); B = zeros(N, n, na); S = zeros(N, n, na); SB = zeros(N, n, na); G = zeros(N, n, na);
    AT = zeros(N, n); BT = zeros(N, n);
    for i = 1:n
        r = runs{1}{i};
        AT(:, i) = r.a_true_out(:, AX) / a_nom;
        apt = r.a_prime_true_out(:, AX) / a_nom;                 % a_bar' true
        BT(:, i) = apt ./ max((1 - AT(:, i)).^2, 1e-12);        % b_true(w_bar) = a_bar'/(1-a_bar)^2
        for j = 1:na
            r = runs{j}{i};
            A(:, i, j) = r.a_bar_hat_out(:, AX);
            B(:, i, j) = r.b_hat_out(:, AX);
            S(:, i, j) = r.P_a_out(:, AX) / a_nom;
            SB(:, i, j) = r.P_b_out(:, AX);
            G(:, i, j) = r.gate_out(:, AX);
        end
    end
    fprintf('sanity: b_true far field (t<0.5) = %.4f (expect 8/9 = 0.8889), trough = %.4f\n', mean(BT(t < 0.5, :), 'all'), mean(BT(t > 3.9, :), 'all'));
    E = A - AT; w = t > t(end) - 0.9;
    seg = {t >= 0.5 & t < 1.5, 'descent'; t >= 1.5 & t < 3.5, 'osc'; w, 'hold-end'};

    fprintf('\n=== Q55 sweep, canonical deep, arm best, z, seeds %s ===\n', mat2str(seeds));
    fprintf('%-8s %-9s | %10s %9s %8s %7s | %8s %8s %8s | %6s %6s %5s\n', 'Q55/ref', 'segment', 'bias', 'pct', 'sd', 'hon', 'b_hat', 'b_true', 'sqrtP55', 'bflr', 'bceil', 'gate');
    res = struct('mults', mults, 'Q55_ref', Q55_ref);
    for j = 1:na
        for s = 1:size(seg, 1)
            m = seg{s, 1};
            bias = mean(E(m, :, j), 1); pct = mean(E(m, :, j) ./ AT(m, :), 1);
            hon = mean(std(E(m, :, j), 0, 2) ./ mean(S(m, :, j), 2));
            nflr = sum(B(m, :, j) <= b_floor + 1e-9, 'all'); ncl = sum(B(m, :, j) >= b_ceil - 1e-9, 'all');
            fprintf('%-8.3g %-9s | %+10.5f %+8.2f%% %8.5f %7.3f | %8.4f %8.4f %8.4f | %6d %6d %5d\n', mults(j), seg{s, 2}, ...
                    mean(bias), 100*mean(pct), std(bias), hon, mean(B(m, :, j), 'all'), mean(BT(m, :), 'all'), mean(SB(m, :, j), 'all'), nflr, ncl, sum(G(m, :, j), 'all'));
        end
    end
    fprintf('paired end-hold bias vs Q55=0 (a_bar x100):');
    for j = 2:na
        d = mean(E(w, :, j), 1) - mean(E(w, :, 1), 1);
        fprintf('  %.3gx: %+.3f +- %.3f', mults(j), 100*mean(d), 100*std(d)/sqrt(n));
    end
    fprintf('\n');

    local_fig(t, A, B, S, SB, AT, BT, E, seeds, n, mults, na, w, fullfile(od, 'q55_sweep.png'));
    out = struct('t', t, 'A', A, 'B', B, 'S', S, 'SB', SB, 'AT', AT, 'BT', BT, 'E', E, 'seeds', seeds, 'res', res);
    save(fullfile(od, 'q55_sweep.mat'), 'out');
end

function local_fig(t, A, B, S, SB, AT, BT, E, seeds, n, mults, na, w, fn)
    % ---- figure ----
    col = [0 0.2 0.9; 0 0.6 0.2; 0.9 0.5 0; 0.6 0 0.6]; cT = [0.85 0.1 0.1];
    f = figure('Position', [60 60 1400 1250], 'Color', 'w', 'Visible', 'off');
    tiledlayout(4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    nexttile; hold on; H = gobjects(0);
    for j = 1:na
        H(end+1) = plot(t, mean(E(:, :, j), 2), '-', 'Color', col(j, :), 'LineWidth', 1.8, 'DisplayName', sprintf('Q55 = %gx ref', mults(j))); %#ok<AGROW>
        plot(t,  mean(S(:, :, j), 2), ':', 'Color', col(j, :), 'LineWidth', 1.2, 'HandleVisibility', 'off');
        plot(t, -mean(S(:, :, j), 2), ':', 'Color', col(j, :), 'LineWidth', 1.2, 'HandleVisibility', 'off');
    end
    ylabel('a_bar_z error (mean, \pm\surdP_{44})'); legend(H, 'Location', 'northoutside', 'Orientation', 'horizontal');
    set(gca, 'FontSize', 15, 'FontWeight', 'bold', 'Box', 'on'); grid off; ylim([-0.04 0.04]);
    nexttile; hold on; H = gobjects(0);
    H(end+1) = plot(t, mean(BT, 2), '-', 'Color', cT, 'LineWidth', 2, 'DisplayName', 'b_true(w_bar)');
    for j = 1:na
        H(end+1) = plot(t, mean(B(:, :, j), 2), '-', 'Color', col(j, :), 'LineWidth', 1.8, 'DisplayName', sprintf('b_hat %gx', mults(j))); %#ok<AGROW>
    end
    yline(8/9, ':k', 'HandleVisibility', 'off');
    ylabel('b (seed mean)'); legend(H, 'Location', 'northoutside', 'Orientation', 'horizontal');
    set(gca, 'FontSize', 15, 'FontWeight', 'bold', 'Box', 'on'); grid off;
    nexttile; hold on; H = gobjects(0);
    for j = 1:na
        H(end+1) = semilogy(t, mean(SB(:, :, j), 2), '-', 'Color', col(j, :), 'LineWidth', 1.8, 'DisplayName', sprintf('\\surdP_{55} %gx', mults(j))); %#ok<AGROW>
        semilogy(t, std(B(:, :, j), 0, 2), '--', 'Color', col(j, :), 'LineWidth', 1.2, 'HandleVisibility', 'off');
    end
    ylabel('\surdP_{55} (solid) / sd_{seeds}(b_hat) (dashed)'); legend(H, 'Location', 'northoutside', 'Orientation', 'horizontal');
    set(gca, 'FontSize', 15, 'FontWeight', 'bold', 'Box', 'on', 'YScale', 'log'); grid off; ylim([3e-3 1e-1]);
    nexttile; hold on;
    bias_end = squeeze(mean(E(w, :, :), 1));    % n x na
    bb = bar(1:n, bias_end, 'grouped');
    for j = 1:na; bb(j).FaceColor = col(j, :); end
    set(gca, 'XTick', 1:n, 'XTickLabel', arrayfun(@num2str, seeds, 'UniformOutput', false));
    xlabel('seed'); ylabel('end-hold bias (a_bar)');
    legend(arrayfun(@(m) sprintf('%gx', m), mults, 'UniformOutput', false), 'Location', 'northoutside', 'Orientation', 'horizontal');
    set(gca, 'FontSize', 15, 'FontWeight', 'bold', 'Box', 'on'); grid off;
    exportgraphics(f, fn, 'Resolution', 150); close(f);
    fprintf('figure -> %s\n', fn);
end

function local_addpaths(root)
    p = strsplit(genpath(fullfile(root, 'model')), pathsep);
    p = p(~cellfun('isempty', p)); p = p(~contains(p, [filesep 'archive']));
    addpath(p{:});
    addpath(fullfile(root, 'test_script', 'integration'));
    addpath(fullfile(root, 'test_script', 'build_helpers'));
end
