% run_formB_ws_c24_prior_sweep.m -- PURPOSE: companion of
%   run_formB_ws_c3_prior_sweep.m on doc configs c2 (lock p, no injection)
%   and c4 (lock p, TRUE wall injected at 1.05); same w_s prior sweep
%   {0.028, 0.111, 0.222}. User request 2026-08-02 ("re-run c1-4 with the
%   lowered value, update the doc"). c1 is excluded: lock_ws pins w_s, the
%   prior never enters. The c4 arm is the decisive one: a small prior only
%   achieves the goal if it is TRUE -- against a real +5 % wall the response
%   fraction collapses with the prior (Bayes K), so the tight-prior filter
%   cleanly misses the true wall. Same-prior wander (c2) vs same-prior
%   response (c4) are one derivation (same K), not a tunable trade-off.
%   EXPIRES: superseded when the D3 physical anchor replaces 0.111.
%   Style: .claude/rules/figure-style.md.

here = fileparts(mfilename('fullpath'));
proj = fullfile(here, '..', '..');
pdirs = {'model', 'model/dual_track', 'model/diag', 'model/thermal_force', ...
         'model/config', 'model/wall_effect', 'model/controller', ...
         'model/trajectory', 'test_script/integration'};
for i = 1:numel(pdirs); addpath(fullfile(proj, pdirs{i})); end
fig_dir  = fullfile(proj, 'reference', 'eq17_analysis', 'derivation', 'figures');
res_file = fullfile(proj, 'test_results', 'run_formB_ws_c24_prior_sweep.mat');

AX     = 3;
SEEDS  = [7 11];
SWEEP  = [0.028 0.111 0.222];
LABELS = {'prior 0.028', 'prior 0.111 (house)', 'prior 0.222'};
SHADES = [0.65 0.80 0.98; 0.20 0.45 0.90; 0.00 0.15 0.55];
CFGS   = {struct('key', 'c2', 'inj', 0), struct('key', 'c4', 'inj', 0.05)};
FS     = 18;

if exist(res_file, 'file')
    load(res_file, 'V');
else
    V = struct('done', zeros(numel(CFGS), numel(SEEDS), numel(SWEEP)));
end
for c = 1:numel(CFGS)
    for s = 1:numel(SEEDS)
        for w = 1:numel(SWEEP)
            if V.done(c, s, w); continue; end
            ov = struct('lock_b', false, 'lock_p', true, 'lock_ws', false, ...
                        'Pf_ws_std', SWEEP(w));
            o = struct('seeds', SEEDS(s), 'ws_inject', CFGS{c}.inj, ...
                       'verbose', false, 'ctrl_const_override', ov);
            out = run_formB_ws(o);
            r = out.runs{1};
            V.t{c, s, w}   = r.tout(:);
            V.dev{c, s, w} = 100 * (r.ws_hat_out(:, AX) - 1);
            V.sqP{c, s, w} = r.P_ws_out(:, AX);
            V.desc(c, s, w) = out.metrics.rows(1, 1);
            V.done(c, s, w) = 1;  save(res_file, 'V');
            fprintf('%s seed %d prior %.3f: dev_end %+.2f%%\n', ...
                    CFGS{c}.key, SEEDS(s), SWEEP(w), V.dev{c, s, w}(end));
        end
    end
end

for c = 1:numel(CFGS)
    inj = CFGS{c}.inj;
    fprintf('\n== %s (lock p, inject %+.2f), w_s prior sweep ==\n', CFGS{c}.key, inj);
    fprintf('seed  prior |  dev_end%%  err_vs_true%%  sqP_end  max|z|  resp  desc%%\n');
    for s = 1:numel(SEEDS)
        for w = 1:numel(SWEEP)
            dv = V.dev{c, s, w};  sq = V.sqP{c, s, w};
            errT = dv(end) - 100 * inj;
            z = abs(dv / 100 - inj) ./ max(sq, 1e-9);
            if inj > 0; resp = dv(end) / (100 * inj); else; resp = NaN; end
            fprintf('%4d  %.3f |  %+7.2f    %+7.2f      %.4f   %5.2f  %5.2f  %5.2f\n', ...
                    SEEDS(s), SWEEP(w), dv(end), errT, sq(end), max(z), resp, ...
                    V.desc(c, s, w));
        end
    end
end

for c = 1:numel(CFGS)
    inj = CFGS{c}.inj;
    for s = 1:numel(SEEDS)
        fig = figure('Color', 'w', 'Position', [80 60 900 560]); drawnow;
        ax = axes(fig); hold(ax, 'on');
        ht = plot(ax, [0 4.8], 100 * inj * [1 1], 'r', 'LineWidth', 2.4);
        if inj > 0
            plot(ax, [0 4.8], [0 0], 'k:', 'LineWidth', 1.4, 'HandleVisibility', 'off');
        end
        hh = gobjects(1, numel(SWEEP));
        for w = 1:numel(SWEEP)
            hh(w) = plot(ax, V.t{c, s, w}, V.dev{c, s, w}, '-', ...
                         'Color', SHADES(w, :), 'LineWidth', 1.8);
        end
        ylabel(ax, '(ws_{hat} - 1)  [% of R]', 'FontSize', FS, 'FontWeight', 'bold');
        xlabel(ax, 'time  [s]', 'FontSize', FS, 'FontWeight', 'bold');
        set(ax, 'FontSize', FS, 'FontWeight', 'bold'); box(ax, 'on');
        ylim(ax, [-32 27]);   % contains the deepest 0.222 drags, nothing clipped
        if inj > 0; tl = sprintf('true wall (ws = %.2f)', 1 + inj);
        else;       tl = 'true wall (nominal)'; end
        legend(ax, [ht hh], [{tl}, LABELS], 'Location', 'northoutside', ...
               'Orientation', 'horizontal', 'FontSize', FS - 4);
        exportgraphics(fig, fullfile(fig_dir, ...
            sprintf('formB_%s_priorsweep_seed%d.png', CFGS{c}.key, SEEDS(s))), ...
            'Resolution', 150);
    end
end
fprintf('figures written to %s\n', fig_dir);
