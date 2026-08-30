function out = verify_if1_r22(opts)
%VERIFY_IF1_R22  Pre-registered acceptance for R(2,2) colour factor IF(s) -> IF(1).
%   Plan (2026-08-27): under y2_whiten the readout is the single increment
%   y2 = a_cov*u[k], so the colour penalty is the long-run IF_white (s = 1),
%   not the EWMA-output IF_eff (s = 1-a_cov). Builder exports IF_abc_white;
%   formC_b selects it by its y2_whiten flag. This script runs Steps 3/4/5;
%   Steps 0/2 (fixture 0.107505, negative control by reverting the selection
%   line) are run separately with l0_regression_fixture.m.
%
%   "pre" arm = the fix undone from inside the driver: ctrl_const_override
%   .IF_abc_white = ctrl_const.IF_abc (the s-weighted sums), which is exactly
%   what the controller read before the change. No git checkout needed.
%
%   Pre-registered criteria (no post-hoc edits):
%     Step 3  measured/model IF, 3 axes x 3 windows, all within 1.00 +- 0.04
%     Step 4  a_cov_scale {0.5,1,2,4}: post-fix a_bar_hat_z trajectories
%             max relative diff <= 1e-6, given G2 does not trigger (report
%             gate counts first; an arm whose G2 count differs is judged by
%             whether its diff starts AT the gated step); pre-fix sweep must
%             show a visible difference = instrument validity
%     Step 5  8 paired seeds, canonical deep, z, end hold (last 0.9 s):
%             |d bias| < 1 (x100, a_bar units); honesty change in [-10, 0] %
%             (prediction -5 +- 5, sqrt(P44) up 1-5 %); per-seed sd change
%             within the SEM of the bias.  |d honesty| > 10 % -> do not merge.
%
%   Output: test_results/if1_r22/{verify_if1_r22.mat, fig_step4_*.png,
%   fig_step5_*.png}; Step 3 regenerates test_results/am_r22_deep/fig11 (the
%   pre-fix copy is kept as fig11_r22_atrue_validation_prefix.png).

    if nargin < 1; opts = struct(); end
    if ~isfield(opts, 'seeds5');  opts.seeds5  = 1:8;            end
    if ~isfield(opts, 'scales4'); opts.scales4 = [0.5 1 2 4];    end
    if ~isfield(opts, 'do3');     opts.do3     = true;           end
    if ~isfield(opts, 'do4');     opts.do4     = true;           end
    if ~isfield(opts, 'do5');     opts.do5     = true;           end

    root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    cd(root);
    local_addpaths(root);
    od = fullfile(root, 'test_results', 'if1_r22');
    if ~exist(od, 'dir'); mkdir(od); end
    AX = 3;                                  % z
    out = struct();

    % ------------------------------------------------------------------
    % Step 3: IF direct check on the true-gain arm (data already on disk)
    % ------------------------------------------------------------------
    if opts.do3
        fprintf('\n=== Step 3: IF measured / model, true-gain arm (truea_100.mat) ===\n');
        f11 = fullfile(root, 'test_results', 'am_r22_deep', 'fig11_r22_atrue_validation.png');
        f11p = strrep(f11, '.png', '_prefix.png');
        if exist(f11, 'file') == 2 && exist(f11p, 'file') ~= 2
            copyfile(f11, f11p);
            fprintf('pre-fix fig11 kept as %s\n', f11p);
        end
        plot_r22_atrue_validation;           % script: leaves V3 in this workspace
        rw = []; rs = [];
        fprintf('%-5s %-8s %8s %12s %12s\n', 'axis', 'window', 'a_true', 'meas/IF(s)', 'meas/IF(1)');
        axl = 'xyz';
        for ax = 1:3
            for i = 1:numel(V3(ax).c)
                fprintf('%-5c %-8s %8.3f %12.3f %12.3f\n', axl(ax), V3(ax).win{i}, ...
                        V3(ax).c(i), V3(ax).r_s(i), V3(ax).r_white(i));
                rw(end+1) = V3(ax).r_white(i); rs(end+1) = V3(ax).r_s(i); %#ok<AGROW>
            end
        end
        ok3 = all(abs(rw - 1) <= 0.04);
        fprintf('Step 3: meas/IF(1) range [%.3f, %.3f] (pre-fix meas/IF(s) [%.3f, %.3f]); criterion 1.00 +- 0.04 -> %s\n', ...
                min(rw), max(rw), min(rs), max(rs), local_pf(ok3));
        out.step3 = struct('V3', V3, 'r_white', rw, 'r_s', rs, 'pass', ok3);
    end

    % ------------------------------------------------------------------
    % Step 4: a_cov invariance (the hardest criterion)
    % ------------------------------------------------------------------
    if opts.do4
        fprintf('\n=== Step 4: a_cov invariance, seed 7, a_cov_scale = %s ===\n', mat2str(opts.scales4));
        sc = opts.scales4; ns = numel(sc);
        traj = cell(ns, 2); gates = zeros(ns, 2); acov = zeros(ns, 1); t4 = []; post_gate = cell(ns, 1);
        for i = 1:ns
            post = local_run(struct('seeds', 7, 'a_cov_scale', sc(i)));
            K = post.ctrl_const;
            pre  = local_run(struct('seeds', 7, 'a_cov_scale', sc(i), ...
                             'ctrl_const_override', struct('IF_abc_white', K.IF_abc(:))));
            traj{i, 1} = pre.a_bar_hat_out(:, AX);  traj{i, 2} = post.a_bar_hat_out(:, AX);
            post_gate{i} = post.gate_out(:, AX); %#ok<AGROW>
            gates(i, :) = [sum(pre.gate_out(:, AX)), sum(post.gate_out(:, AX))];
            acov(i) = K.a_cov; t4 = post.tout;
            fprintf('a_cov %.4f (a_pd %.3f): gate count z pre %d post %d | IF_abc(s)=%s IF_abc_white=%s\n', ...
                    K.a_cov, K.a_pd, gates(i, 1), gates(i, 2), mat2str(K.IF_abc(:)', 5), mat2str(K.IF_abc_white(:)', 5));
        end
        iref = find(sc == 1, 1); if isempty(iref); iref = 1; end
        d4 = zeros(ns, 2);
        for i = 1:ns
            for arm = 1:2
                r = traj{iref, arm}; a = traj{i, arm};
                d4(i, arm) = max(abs(a - r) ./ max(abs(r), 1e-12));
            end
        end
        fprintf('%-8s %14s %14s\n', 'a_cov', 'pre max|rel d|', 'post max|rel d|');
        for i = 1:ns
            fprintf('%-8.4f %14.3e %14.3e\n', acov(i), d4(i, 1), d4(i, 2));
        end
        % Pre-registered proviso: the 1e-6 criterion holds "given G2 does not
        % trigger" -- G2 = (sigma2_dwr_hat - C_n*sigma2_n <= 0) reads the a_cov
        % EWMA itself, so a gated step drops one y2 sample in one arm only.
        % Arms whose gate count differs from the reference are reported
        % separately (first differing step), not counted against the criterion.
        gate_free = gates(:, 2) == gates(iref, 2);
        same_gates = all(gate_free);
        ok4_all  = max(d4(:, 2)) <= 1e-6;
        ok4 = max(d4(gate_free, 2)) <= 1e-6;
        fprintf('Step 4: post-fix max rel diff, ALL arms %.3e (<= 1e-6: %s); gate-free arms %.3e -> %s; pre-fix max rel diff %.3e (instrument sees the IF(s) leak)\n', ...
                max(d4(:, 2)), local_pf(ok4_all), max(d4(gate_free, 2)), local_pf(ok4), max(d4(:, 1)));
        for i = find(~gate_free)'
            k1 = find(abs(traj{i, 2} - traj{iref, 2}) ./ max(abs(traj{iref, 2}), 1e-12) > 1e-9, 1);
            kg = find(post_gate{i} ~= post_gate{iref}, 1);
            fprintf('  a_cov %.4f: gate count %d vs ref %d; first gate mismatch k=%d (t=%.4f s), first diff > 1e-9 at k=%d -> %s\n', ...
                    acov(i), gates(i, 2), gates(iref, 2), kg, t4(kg), k1, local_iff(isequal(k1, kg), 'diff starts AT the gated step (G2 proviso)', 'diff starts elsewhere: NOT the gate'));
        end
        out.step4 = struct('scales', sc, 'a_cov', acov, 'd', d4, 'gates', gates, 'pass', ok4, ...
                           'pass_all_arms', ok4_all, 'same_gates', same_gates, 'gate_free', gate_free);
        local_fig_step4(t4, traj, acov, d4, fullfile(od, 'fig_step4_acov_invariance.png'));
    end

    % ------------------------------------------------------------------
    % Step 5: 8 paired seeds, canonical deep
    % ------------------------------------------------------------------
    if opts.do5
        seeds = opts.seeds5; n = numel(seeds);
        fprintf('\n=== Step 5: paired seeds %s, canonical deep, z, end hold (last 0.9 s) ===\n', mat2str(seeds));
        P = local_run_many(struct('seeds', seeds));
        K = P.runs{1}.ctrl_const;
        Q = local_run_many(struct('seeds', seeds, 'ctrl_const_override', struct('IF_abc_white', K.IF_abc(:))));
        t = P.runs{1}.tout; a_nom = P.runs{1}.a_nom;
        w = t > t(end) - 0.9;
        E = zeros(numel(t), n, 2); S = zeros(numel(t), n, 2); AT = zeros(numel(t), n);
        for i = 1:n
            for arm = 1:2
                if arm == 1; r = Q.runs{i}; else; r = P.runs{i}; end
                AT(:, i) = r.a_true_out(:, AX) / a_nom;
                E(:, i, arm) = r.a_bar_hat_out(:, AX) - AT(:, i);
                S(:, i, arm) = r.P_a_out(:, AX) / a_nom;
            end
        end
        bias = squeeze(mean(E(w, :, :), 1));                 % n x 2, a_bar units
        pct  = squeeze(mean(E(w, :, :) ./ AT(w, :), 1));     % n x 2, relative
        hon  = squeeze(mean(std(E(w, :, :), 0, 2) ./ mean(S(w, :, :), 2), 1));   % 1 x 2
        sP   = squeeze(mean(mean(S(w, :, :), 1), 2));        % mean sqrtP over window, 1 x 2
        dbias = bias(:, 2) - bias(:, 1);
        sd = std(bias, 0, 1); sem = sd / sqrt(n);
        fprintf('%-6s %12s %12s %12s | %10s %10s\n', 'seed', 'bias pre', 'bias post', 'd(post-pre)', 'pct pre', 'pct post');
        for i = 1:n
            fprintf('%-6d %+12.5f %+12.5f %+12.5f | %+9.2f%% %+9.2f%%\n', seeds(i), bias(i, 1), bias(i, 2), dbias(i), 100*pct(i, 1), 100*pct(i, 2));
        end
        fprintf('mean bias (a_bar):  pre %+.5f  post %+.5f  paired d %+.5f +- %.5f (SEM)\n', ...
                mean(bias(:, 1)), mean(bias(:, 2)), mean(dbias), std(dbias)/sqrt(n));
        fprintf('mean bias (%%):      pre %+.2f  post %+.2f\n', 100*mean(pct(:, 1)), 100*mean(pct(:, 2)));
        fprintf('per-seed sd (a_bar): pre %.5f  post %.5f  (d %+.5f; SEM of bias %.5f)\n', sd(1), sd(2), sd(2)-sd(1), sem(1));
        fprintf('sqrt(P44) end-hold:  pre %.5f  post %.5f  (%+.2f %%)\n', sP(1), sP(2), 100*(sP(2)/sP(1)-1));
        dhon = 100*(hon(2)/hon(1) - 1);
        fprintf('honesty spread/sqrtP: pre %.3f  post %.3f  (%+.2f %%)\n', hon(1), hon(2), dhon);
        ok5a = abs(100*mean(dbias)) < 1;
        ok5b = dhon >= -10 && dhon <= 0;
        fail5b = abs(dhon) > 10;
        ok5c = abs(sd(2) - sd(1)) <= sem(1);
        fprintf('Step 5a |d bias| x100 = %.3f < 1 -> %s\n', abs(100*mean(dbias)), local_pf(ok5a));
        fprintf('Step 5b honesty d = %+.2f %% in [-10, 0] -> %s%s\n', dhon, local_pf(ok5b), local_iff(fail5b, '  (>10 %: DO NOT MERGE)', ''));
        fprintf('Step 5c |d sd| = %.5f <= SEM %.5f -> %s\n', abs(sd(2)-sd(1)), sem(1), local_pf(ok5c));
        v7 = NaN; i7 = find(seeds == 7, 1);
        if ~isempty(i7); v7 = P.runs{i7}.a_bar_hat_out(end, AX); end
        fprintf('new fixture candidate: seed 7 a_bar_hat_z[end] = %.9f (was 0.107505330)\n', v7);
        out.step5 = struct('seeds', seeds, 'bias', bias, 'pct', pct, 'hon', hon, 'sqrtP', sP, ...
                           'sd', sd, 'sem', sem, 'dhon', dhon, 'pass', ok5a && ok5b && ok5c, ...
                           'fail_hard', fail5b, 'fixture_seed7_end', v7);
        local_fig_step5(t, E, S, bias, seeds, fullfile(od, 'fig_step5_paired_seeds.png'));
    end

    save(fullfile(od, 'verify_if1_r22.mat'), 'out');
    fprintf('\nsaved %s\n', fullfile(od, 'verify_if1_r22.mat'));
end

% ======================= local helpers =======================
function r = local_run(o)
    o.verbose = false;
    evalc('c = run_formC_b(o);');
    r = c.runs{1};
end

function c = local_run_many(o)
    o.verbose = false;
    evalc('c = run_formC_b(o);');
end

function local_addpaths(root)
    p = strsplit(genpath(fullfile(root, 'model')), pathsep);
    p = p(~cellfun('isempty', p));
    p = p(~contains(p, [filesep 'archive']));
    addpath(p{:});
    addpath(fullfile(root, 'test_script', 'integration'));
    addpath(fullfile(root, 'test_script', 'build_helpers'));
    addpath(fullfile(root, 'test_script', 'scratch'));
end

function s = local_pf(ok)
    if ok; s = 'PASS'; else; s = 'FAIL'; end
end

function s = local_iff(c, a, b)
    if c; s = a; else; s = b; end
end

function local_fig_step4(t, traj, acov, d4, fn)
    ns = numel(acov);
    col = lines(ns);
    f = figure('Position', [80 80 1100 800], 'Color', 'w', 'Visible', 'off');
    tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    nexttile; hold on; H = gobjects(0);
    for i = 1:ns
        plot(t, traj{i, 1}, '--', 'Color', col(i, :), 'LineWidth', 1.2, 'HandleVisibility', 'off');
        H(end+1) = plot(t, traj{i, 2}, '-', 'Color', col(i, :), 'LineWidth', 1.6, ...
                        'DisplayName', sprintf('a_{cov} = %.3g', acov(i))); %#ok<AGROW>
    end
    ylabel('$\hat{\bar a}_z$', 'Interpreter', 'latex');
    legend(H, 'Location', 'northoutside', 'Orientation', 'horizontal');
    set(gca, 'FontSize', 18, 'FontWeight', 'bold', 'Box', 'on'); grid off;
    text(0.02, 0.08, 'solid: post-fix IF(1)   dashed: pre-fix IF(s)', 'Units', 'normalized', 'FontSize', 14);
    nexttile; hold on;
    iref = find(abs(acov - 0.05) < 1e-12, 1); if isempty(iref); iref = 1; end   % same reference as the table
    for i = 1:ns
        dd_pre  = abs(traj{i, 1} - traj{iref, 1}) ./ max(abs(traj{iref, 1}), 1e-12);
        dd_post = abs(traj{i, 2} - traj{iref, 2}) ./ max(abs(traj{iref, 2}), 1e-12);
        semilogy(t, max(dd_pre, 1e-18), '--', 'Color', col(i, :), 'LineWidth', 1.2);
        semilogy(t, max(dd_post, 1e-18), '-', 'Color', col(i, :), 'LineWidth', 1.6);
    end
    yline(1e-6, ':k', 'LineWidth', 1.2);
    xlabel('t [s]'); ylabel('|rel diff| vs a_{cov} ref');
    set(gca, 'FontSize', 18, 'FontWeight', 'bold', 'Box', 'on', 'YScale', 'log'); grid off;
    ylim([1e-18 1]);
    exportgraphics(f, fn, 'Resolution', 150); close(f);
    fprintf('figure -> %s   (max rel diff pre %.2e, post %.2e)\n', fn, max(d4(:, 1)), max(d4(:, 2)));
end

function local_fig_step5(t, E, S, bias, seeds, fn)
    n = numel(seeds);
    f = figure('Position', [80 80 1400 650], 'Color', 'w', 'Visible', 'off');
    tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    lab = {'pre-fix IF(s)', 'post-fix IF(1)'};
    yl = [min(E(:)) max(E(:))];
    for arm = 1:2
        nexttile; hold on;
        for i = 1:n
            plot(t, E(:, i, arm), '-', 'Color', [0 0.2 0.9 0.35], 'LineWidth', 0.8, 'HandleVisibility', 'off');
        end
        sp = mean(S(:, :, arm), 2);
        H1 = plot(t, mean(E(:, :, arm), 2), '-', 'Color', [0 0.2 0.9], 'LineWidth', 2, 'DisplayName', 'mean error');
        H2 = plot(t, sp, '--r', 'LineWidth', 1.6, 'DisplayName', '\pm mean \surd P_{44}');
        plot(t, -sp, '--r', 'LineWidth', 1.6, 'HandleVisibility', 'off');
        xlabel('t [s]'); ylabel('$\hat{\bar a}_z - \bar a_{z,true}$', 'Interpreter', 'latex');
        legend([H1 H2], 'Location', 'northoutside', 'Orientation', 'horizontal');
        set(gca, 'FontSize', 16, 'FontWeight', 'bold', 'Box', 'on'); grid off; ylim(yl);
        text(0.02, 0.05, lab{arm}, 'Units', 'normalized', 'FontSize', 14, 'FontWeight', 'bold');
    end
    nexttile; hold on;
    b = bar(1:n, bias, 'grouped');
    b(1).FaceColor = [0.6 0.6 0.6]; b(2).FaceColor = [0 0.2 0.9];
    set(gca, 'XTick', 1:n, 'XTickLabel', arrayfun(@num2str, seeds, 'UniformOutput', false));
    xlabel('seed'); ylabel('end-hold bias (a_{bar})');
    legend(lab, 'Location', 'northoutside', 'Orientation', 'horizontal');
    set(gca, 'FontSize', 16, 'FontWeight', 'bold', 'Box', 'on'); grid off;
    exportgraphics(f, fn, 'Resolution', 150); close(f);
    fprintf('figure -> %s\n', fn);
end
