% run_formB_ws_calib_chain.m -- ROOT-CURE demo: w_bar_s as a CALIBRATION
%   constant. Sequential posterior handoff across runs (run n's posterior
%   (ws_hat, sqrtP_ws) becomes run n+1's prior); the cross-chain spread must
%   shrink as the claimed sqrtP does (honesty), and every chain must converge
%   to the injected TRUE wall.
%   PURPOSE: doc page c5 (chain convergence + post-calibration operational
%   run); chat decision 2026-08-01 (root cure = information ledger, not more
%   filter surgery -- single-run honesty was completed by MA2 + echo).
%   EXPIRES: superseded when a chain mode moves into integration/.
%   Protocol: C2 config (lock p, estimate b + w_s), TRUE wall injected at
%   1.05 (opts.ws_inject, plant side), N_CHAIN chains x N_STAGE runs, fresh
%   seed every run; operational run per chain with the stage-N posterior.
%   Style: .claude/rules/figure-style.md.

here = fileparts(mfilename('fullpath'));
proj = fullfile(here, '..', '..');
res_dir = fullfile(proj, 'test_results');

AX_Z = 3; FS = 18;
SEED0   = [7 11 23 42 101 777];
N_CHAIN = 6;  N_STAGE = 8;
DELTA   = 0.05;            % injected true wall offset [R]
WS_TRUE = 1 + DELTA;
PRIOR0  = 0.111;           % session-start prior (the c2 house value)
SEED_BLUES = [0.00 0.20 0.75; 0.10 0.45 0.90; 0.35 0.60 0.95; ...
              0.00 0.30 0.55; 0.25 0.35 0.80; 0.50 0.70 1.00];

base_ov = struct('lock_b', false, 'lock_p', true, 'lock_ws', false);

ws_chain = zeros(N_CHAIN, N_STAGE); P_chain = zeros(N_CHAIN, N_STAGE);
for c = 1:N_CHAIN
    ws0 = 1; Pws0 = PRIOR0;
    for k = 1:N_STAGE
        o = struct(); o.seeds = SEED0(c) + 200*(k-1); o.ws_inject = DELTA;
        o.verbose = false;
        o.ctrl_const_override = base_ov;
        o.ctrl_const_override.Pf_ws_std = Pws0;
        o.ctrl_const_override.ws_init   = ws0;
        out_k = run_formB_ws(o);
        ws0  = out_k.runs{1}.ws_hat_out(end, AX_Z);
        Pws0 = out_k.runs{1}.P_ws_out(end, AX_Z);
        ws_chain(c, k) = ws0;  P_chain(c, k) = Pws0;
    end
end

% operational run per chain: fresh noise, the stage-N posterior as prior
ws_op = cell(N_CHAIN, 1); err_op = zeros(N_CHAIN, 1); desc_op = zeros(N_CHAIN, 1);
for c = 1:N_CHAIN
    o = struct(); o.seeds = SEED0(c) + 2000; o.ws_inject = DELTA; o.verbose = false;
    o.ctrl_const_override = base_ov;
    o.ctrl_const_override.Pf_ws_std = P_chain(c, N_STAGE);
    o.ctrl_const_override.ws_init   = ws_chain(c, N_STAGE);
    out_o = run_formB_ws(o);
    r = out_o.runs{1};
    ws_op{c}   = r.ws_hat_out(:, AX_Z);
    err_op(c)  = r.ws_hat_out(end, AX_Z) - WS_TRUE;
    desc_op(c) = out_o.metrics.rows(1, 1);
    t_op = r.tout(:);
end
save(fullfile(res_dir, 'run_formB_ws_calib_chain.mat'), ...
     'ws_chain', 'P_chain', 'ws_op', 'err_op', 'desc_op', 'SEED0', 'DELTA');

% ---- Figure A: chain convergence ----
figA = figure('Color', 'w', 'Position', [80 60 900 620]); drawnow;
axA = axes(figA); hold(axA, 'on');
hs = gobjects(1, N_CHAIN);
for c = 1:N_CHAIN
    hs(c) = plot(axA, 1:N_STAGE, ws_chain(c, :), '-o', ...
                 'Color', SEED_BLUES(c, :), 'LineWidth', 1.6, 'MarkerSize', 5, ...
                 'MarkerFaceColor', SEED_BLUES(c, :));
end
ht = plot(axA, [1 N_STAGE], WS_TRUE * [1 1], 'r', 'LineWidth', 2.6);
h0 = plot(axA, [1 N_STAGE], [1 1], 'k:', 'LineWidth', 1.6);
Pm = mean(P_chain, 1);
hb = plot(axA, 1:N_STAGE, WS_TRUE + Pm, 'k--', 'LineWidth', 1.2);
plot(axA, 1:N_STAGE, WS_TRUE - Pm, 'k--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
ylabel(axA, 'ws_{hat} after run  [-]', 'FontSize', FS, 'FontWeight', 'bold');
xlabel(axA, 'calibration run #', 'FontSize', FS, 'FontWeight', 'bold');
legend([ht h0 hb hs(1)], {'true wall 1.05', 'nominal (1)', ...
       'claimed \pm\surdP', 'chains (6, fresh noise each run)'}, ...
       'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS - 6);
xlim(axA, [1 N_STAGE]); xticks(axA, 1:N_STAGE);
set(axA, 'FontSize', FS - 2, 'FontWeight', 'bold', 'Box', 'on');
pngA = fullfile(res_dir, 'formB_c5_chain.png');
drawnow; exportgraphics(figA, pngA, 'Resolution', 150); close(figA);
fprintf('FIGURE saved: %s\n', pngA);

% ---- Figure B: operational run with the calibrated prior ----
figB = figure('Color', 'w', 'Position', [80 60 900 620]); drawnow;
axB = axes(figB); hold(axB, 'on');
for c = 1:N_CHAIN
    hs(c) = plot(axB, t_op, 100 * (ws_op{c} - 1), 'Color', SEED_BLUES(c, :), ...
                 'LineWidth', 1.4);
end
ht = plot(axB, t_op([1 end]), 100 * DELTA * [1 1], 'r', 'LineWidth', 2.6);
h0 = plot(axB, t_op([1 end]), [0 0], 'k:', 'LineWidth', 1.6);
ylabel(axB, '(ws_{hat}-1)  [% of R]', 'FontSize', FS, 'FontWeight', 'bold');
xlabel(axB, 'time  [s]', 'FontSize', FS, 'FontWeight', 'bold');
legend([ht h0 hs(1)], {'true wall (+5)', 'nominal (0)', ...
       'operational runs, calibrated prior (6 chains)'}, ...
       'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS - 6);
ylim(axB, [-26 8]); xlim(axB, [t_op(1) t_op(end)]);
set(axB, 'FontSize', FS - 2, 'FontWeight', 'bold', 'Box', 'on');
for x = [0.5 1.5 3.5]
    plot(axB, [x x], ylim(axB), '--', 'Color', [0.6 0.6 0.6], ...
         'LineWidth', 1.0, 'HandleVisibility', 'off');
end
pngB = fullfile(res_dir, 'formB_c5_operational.png');
drawnow; exportgraphics(figB, pngB, 'Resolution', 150); close(figB);
fprintf('FIGURE saved: %s\n', pngB);

% ---- console record ----
fprintf('CAPTION: calibration chains (C2 config, true wall 1.05): 6 chains x %d runs, fresh noise per run;\n', N_STAGE);
fprintf('         operational run with the calibrated prior on fresh noise.\n');
fprintf('spread across chains by run: '); fprintf('%.4f ', std(ws_chain, 0, 1)); fprintf('\n');
fprintf('claimed sqrtP by run:        '); fprintf('%.4f ', mean(P_chain, 1)); fprintf('\n');
fprintf('mean error vs 1.05 by run:   '); fprintf('%+.4f ', mean(ws_chain, 1) - WS_TRUE); fprintf('\n');
fprintf('operational: final ws error per chain: '); fprintf('%+.4f ', err_op);
fprintf('\n             std %.4f | desc pk mean %.3f %%\n', std(err_op), mean(desc_op));
