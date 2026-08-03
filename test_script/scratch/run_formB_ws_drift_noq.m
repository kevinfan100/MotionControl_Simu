% run_formB_ws_drift_noq.m -- PURPOSE: show what post-calibration operation
%   looks like when the wall DRIFTS and w_s carries NO drift q (current
%   production controller: Q on w_s is exactly zero). User request 2026-08-02
%   ("what do I expect after calibration if w_s has no drift q -- show me").
%   Protocol: calibrated prior (ws 1.05, sqrtP 0.028, the c5 cross-chain
%   result) fixed at the start of EVERY operational run; true wall drifts
%   +1.25 %/run for 10 runs (drift arm) vs stays at +5 % (control arm), same
%   fresh seeds. Wall static within a run (drift timescale >> 4.8 s).
%   Expected failure mode: tight prior => small Kalman gain => estimate lags
%   the ramp; claimed band frozen at the calibration width => truth exits the
%   claimed +-2 sqrtP band => "confidently wrong" + descent error grows.
%   EXPIRES: superseded by the drift-q implementation (five-stage plan,
%   Stage "D3 prior + drift q") and its acceptance runs.
%   Style: .claude/rules/figure-style.md.

here = fileparts(mfilename('fullpath'));
proj = fullfile(here, '..', '..');
pdirs = {'model', 'model/dual_track', 'model/diag', 'model/thermal_force', ...
         'model/config', 'model/wall_effect', 'model/controller', ...
         'model/trajectory', 'test_script/integration'};
for i = 1:numel(pdirs); addpath(fullfile(proj, pdirs{i})); end
res_dir  = fullfile(proj, 'test_results');
res_file = fullfile(res_dir, 'run_formB_ws_drift_noq.mat');

AX      = 3;
N_RUN   = 10;
SEEDS   = 4000 + (1:N_RUN);        % fresh band
WS_CAL  = 1.05;                    % calibrated prior mean  (c5: chain converged)
SQP_CAL = 0.028;                   % calibrated prior width (c5: cross-chain sqrtP)
DRIFT   = 0.0125;                  % wall drift per run [R]
INJ0    = 0.05;                    % wall at calibration time
FS      = 18;
LIGHT_BLUE = [0.45 0.72 0.95];

ov = struct('lock_b', false, 'lock_p', true, 'lock_ws', false, ...
            'Pf_ws_std', SQP_CAL, 'ws_init', WS_CAL);

if exist(res_file, 'file')
    load(res_file, 'D');
else
    D = struct('done', zeros(2, N_RUN));
end
for arm = 1:2                       % 1 = drifting wall, 2 = static control
    for n = 1:N_RUN
        if D.done(arm, n); continue; end
        if arm == 1; inj = INJ0 + DRIFT * (n - 1); else; inj = INJ0; end
        o = struct('seeds', SEEDS(n), 'ws_inject', inj, 'verbose', false, ...
                   'ctrl_const_override', ov);
        out = run_formB_ws(o);
        r = out.runs{1};
        D.ws_true(arm, n) = 1 + inj;
        D.ws_end(arm, n)  = r.ws_hat_out(end, AX);
        D.sqP_end(arm, n) = r.P_ws_out(end, AX);
        D.desc(arm, n)    = out.metrics.rows(1, 1);
        D.hold(arm, n)    = out.metrics.rows(1, 3);
        D.done(arm, n) = 1;  save(res_file, 'D');
        fprintf('arm %d run %2d: true %.4f  ws_end %.4f  desc %.2f%%\n', ...
                arm, n, 1 + inj, D.ws_end(arm, n), D.desc(arm, n));
    end
end

% ---------------- console table ----------------
fprintf('\n== no-q operation under wall drift (prior fixed at calibration: ws %.3f, sqrtP %.3f) ==\n', ...
        WS_CAL, SQP_CAL);
fprintf('run |  true    ws_end   err     |err|/sqP  desc%%   (drift arm)\n');
first_1s = 0; first_2s = 0;
for n = 1:N_RUN
    e = D.ws_end(1, n) - D.ws_true(1, n);  zr = abs(e) / D.sqP_end(1, n);
    if zr > 1 && ~first_1s; first_1s = n; end
    if zr > 2 && ~first_2s; first_2s = n; end
    fprintf('%3d | %.4f  %.4f  %+.4f   %5.2f     %5.2f\n', ...
            n, D.ws_true(1, n), D.ws_end(1, n), e, zr, D.desc(1, n));
end
fprintf('claimed band first broken: 1-sigma at run %d, 2-sigma at run %d\n', first_1s, first_2s);
fprintf('control arm (no drift): err std %.4f  desc %.2f +- %.2f %%\n', ...
        std(D.ws_end(2, :) - D.ws_true(2, :)), mean(D.desc(2, :)), std(D.desc(2, :)));

% ---------------- figure A: wall estimate vs drifting truth ----------------
x = 1:N_RUN;
figA = figure('Color', 'w', 'Position', [80 60 900 560]); drawnow;
axA = axes(figA); hold(axA, 'on');
ht = plot(axA, x, 100 * (D.ws_true(1, :) - 1), 'r-o', 'LineWidth', 2.4, ...
          'MarkerFaceColor', 'r', 'MarkerSize', 6);
he = errorbar(axA, x, 100 * (D.ws_end(1, :) - 1), 200 * D.sqP_end(1, :), ...
              'b-s', 'LineWidth', 1.8, 'MarkerFaceColor', 'b', 'MarkerSize', 6, ...
              'CapSize', 10);
ylabel(axA, '(ws - 1)  [% of R]', 'FontSize', FS, 'FontWeight', 'bold');
xlabel(axA, 'operational run #', 'FontSize', FS, 'FontWeight', 'bold');
set(axA, 'FontSize', FS, 'FontWeight', 'bold'); box(axA, 'on'); xlim(axA, [0.5, N_RUN + 0.5]);
legend(axA, [ht he], {'true wall (drifting +1.25 %/run)', 'ws_{hat} \pm 2\surdP (claimed)'}, ...
       'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS - 2);
exportgraphics(figA, fullfile(res_dir, 'formB_drift_noq_wall.png'), 'Resolution', 150);

% ---------------- figure B: descent error per run, both arms ----------------
figB = figure('Color', 'w', 'Position', [80 60 900 560]); drawnow;
axB = axes(figB); hold(axB, 'on');
hd = plot(axB, x, D.desc(1, :), 'b-s', 'LineWidth', 1.8, 'MarkerFaceColor', 'b', 'MarkerSize', 6);
hc = plot(axB, x, D.desc(2, :), '-o', 'Color', LIGHT_BLUE, 'LineWidth', 1.8, ...
          'MarkerFaceColor', LIGHT_BLUE, 'MarkerSize', 6);
hf = plot(axB, [0.5, N_RUN + 0.5], [1.69 1.69], 'k--', 'LineWidth', 1.4);
ylabel(axB, 'descent peak error  [%]', 'FontSize', FS, 'FontWeight', 'bold');
xlabel(axB, 'operational run #', 'FontSize', FS, 'FontWeight', 'bold');
set(axB, 'FontSize', FS, 'FontWeight', 'bold'); box(axB, 'on'); xlim(axB, [0.5, N_RUN + 0.5]);
legend(axB, [hd hc hf], {'drifting wall, no q', 'static wall (control)', ...
       'calibrated level 1.69 %'}, ...
       'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS - 2);
exportgraphics(figB, fullfile(res_dir, 'formB_drift_noq_desc.png'), 'Resolution', 150);
fprintf('figures: %s\n         %s\n', fullfile(res_dir, 'formB_drift_noq_wall.png'), ...
        fullfile(res_dir, 'formB_drift_noq_desc.png'));
