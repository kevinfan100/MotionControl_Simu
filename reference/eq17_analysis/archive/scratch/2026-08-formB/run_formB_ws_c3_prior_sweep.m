% run_formB_ws_c3_prior_sweep.m -- PURPOSE: doc-c3 (all three of b, p, w_s
%   free) rerun with the w_s prior width swept {0.028, 0.111, 0.222} =
%   {calibrated-grade / house placeholder / doubled}; user request 2026-08-02
%   ("what happens to c3 if the 0.111 is smaller or bigger -- update the doc").
%   Expected (Bayes): the ws_hat excursion scales with the claimed prior and
%   stays inside its own band at every width -- the D3 anchor buys a smaller
%   excursion, not a different truth. Figures go straight to the derivation
%   figures dir (doc page added to formB_ws.tex).
%   EXPIRES: superseded when the D3 physical anchor replaces 0.111.
%   Style: .claude/rules/figure-style.md.

here = fileparts(mfilename('fullpath'));
proj = fullfile(here, '..', '..');
pdirs = {'model', 'model/dual_track', 'model/diag', 'model/thermal_force', ...
         'model/config', 'model/wall_effect', 'model/controller', ...
         'model/trajectory', 'test_script/integration'};
for i = 1:numel(pdirs); addpath(fullfile(proj, pdirs{i})); end
fig_dir  = fullfile(proj, 'reference', 'eq17_analysis', 'derivation', 'figures');
res_file = fullfile(proj, 'test_results', 'run_formB_ws_c3_prior_sweep.mat');

AX     = 3;
SEEDS  = [7 11];                       % doc panel convention
SWEEP  = [0.028 0.111 0.222];          % calibrated-grade / house / doubled
LABELS = {'prior 0.028', 'prior 0.111 (house)', 'prior 0.222'};
SHADES = [0.65 0.80 0.98; 0.20 0.45 0.90; 0.00 0.15 0.55];
FS     = 18;

if exist(res_file, 'file')
    load(res_file, 'W');
else
    W = struct('done', zeros(numel(SEEDS), numel(SWEEP)));
end
for s = 1:numel(SEEDS)
    for w = 1:numel(SWEEP)
        if W.done(s, w); continue; end
        ov = struct('lock_b', false, 'lock_p', false, 'lock_ws', false, ...
                    'Pf_ws_std', SWEEP(w));
        o = struct('seeds', SEEDS(s), 'verbose', false, 'ctrl_const_override', ov);
        out = run_formB_ws(o);
        r = out.runs{1};
        W.t{s, w}   = r.tout(:);
        W.dev{s, w} = 100 * (r.ws_hat_out(:, AX) - 1);
        W.sqP{s, w} = r.P_ws_out(:, AX);
        W.desc(s, w)  = out.metrics.rows(1, 1);
        W.b_end(s, w) = r.b_hat_out(end, AX);
        W.p_end(s, w) = r.p_hat_out(end, AX);
        W.done(s, w) = 1;  save(res_file, 'W');
        fprintf('seed %d prior %.3f: dev_end %+.2f%%  sqP_end %.4f\n', ...
                SEEDS(s), SWEEP(w), W.dev{s, w}(end), W.sqP{s, w}(end));
    end
end

fprintf('\n== c3 (b, p, w_s all free), w_s prior sweep ==\n');
fprintf('seed  prior |  max|dev|%%  dev_end%%  sqP_end  max|z|  desc%%\n');
for s = 1:numel(SEEDS)
    for w = 1:numel(SWEEP)
        z = abs(W.dev{s, w} / 100) ./ max(W.sqP{s, w}, 1e-9);
        fprintf('%4d  %.3f |   %6.2f    %+6.2f    %.4f   %5.2f   %5.2f\n', ...
                SEEDS(s), SWEEP(w), max(abs(W.dev{s, w})), W.dev{s, w}(end), ...
                W.sqP{s, w}(end), max(z), W.desc(s, w));
    end
end

for s = 1:numel(SEEDS)
    fig = figure('Color', 'w', 'Position', [80 60 900 560]); drawnow;
    ax = axes(fig); hold(ax, 'on');
    ht = plot(ax, [0 4.8], [0 0], 'r', 'LineWidth', 2.4);
    hh = gobjects(1, numel(SWEEP));
    for w = 1:numel(SWEEP)
        hh(w) = plot(ax, W.t{s, w}, W.dev{s, w}, '-', 'Color', SHADES(w, :), ...
                     'LineWidth', 1.8);
    end
    ylabel(ax, '(ws_{hat} - 1)  [% of R]', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel(ax, 'time  [s]', 'FontSize', FS, 'FontWeight', 'bold');
    set(ax, 'FontSize', FS, 'FontWeight', 'bold'); box(ax, 'on');
    ylim(ax, [-32 27]);   % must contain the full 0.222-prior seed-11 drag (-29 %)
    legend(ax, [ht hh], [{'true wall (nominal)'}, LABELS], ...
           'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS - 4);
    exportgraphics(fig, fullfile(fig_dir, sprintf('formB_c3_priorsweep_seed%d.png', SEEDS(s))), ...
                   'Resolution', 150);
end
fprintf('figures written to %s\n', fig_dir);
