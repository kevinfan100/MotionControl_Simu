% run_formB_ws_c3_extra_seeds.m -- PURPOSE: representativeness check of the
%   doc c3 panels (seeds 7/11): run two further house seeds (23, 42) on the
%   same config 3 (b, p, w_s all free, house prior 0.111, no injection) and
%   plot ws_hat with its claimed +-2 sqrtP band. User request 2026-08-03.
%   EXPIRES: absorbed into the long-run self-lock investigation.
%   Style: .claude/rules/figure-style.md.

here = fileparts(mfilename('fullpath'));
proj = fullfile(here, '..', '..');
pdirs = {'model', 'model/dual_track', 'model/diag', 'model/thermal_force', ...
         'model/config', 'model/wall_effect', 'model/controller', ...
         'model/trajectory', 'test_script/integration'};
for i = 1:numel(pdirs); addpath(fullfile(proj, pdirs{i})); end
res_dir = fullfile(proj, 'test_results');

AX = 3; SEEDS = [23 42]; FS = 18;
ov = struct('lock_b', false, 'lock_p', false, 'lock_ws', false, 'Pf_ws_std', 0.111);
for s = 1:numel(SEEDS)
    o = struct('seeds', SEEDS(s), 'ws_inject', 0, 'verbose', false, ...
               'ctrl_const_override', ov);
    out = run_formB_ws(o);
    r = out.runs{1};
    t   = r.tout(:);
    dev = 100 * (r.ws_hat_out(:, AX) - 1);
    sqP = 100 * r.P_ws_out(:, AX);      % already the std, do NOT sqrt again
    z   = abs(dev) ./ max(sqP, 1e-9);
    fprintf('C3EXTRA seed %d: dev_end %+.2f%%  max|dev| %.2f%%  sqP_end %.2f%%  max|z| %.2f  desc %.2f%%\n', ...
            SEEDS(s), dev(end), max(abs(dev)), sqP(end), max(z), out.metrics.rows(1,1));

    fig = figure('Color', 'w', 'Position', [80 60 980 560]); drawnow;
    ax = axes(fig); hold(ax, 'on');
    ht = plot(ax, [0 t(end)], [0 0], 'r', 'LineWidth', 2.4);
    hb = plot(ax, t, dev + 2*sqP, 'k--', 'LineWidth', 1.0);
    plot(ax, t, dev - 2*sqP, 'k--', 'LineWidth', 1.0, 'HandleVisibility', 'off');
    he = plot(ax, t, dev, 'b', 'LineWidth', 1.8);
    ylabel(ax, '(ws_{hat} - 1)  [% of R]', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel(ax, 'time  [s]', 'FontSize', FS, 'FontWeight', 'bold');
    set(ax, 'FontSize', FS, 'FontWeight', 'bold'); box(ax, 'on');
    ylim(ax, [-30 30]);
    legend(ax, [ht he hb], {'true wall (nominal)', 'ws_{hat}', 'claimed \pm2\surdP'}, ...
           'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS - 4);
    exportgraphics(fig, fullfile(res_dir, sprintf('formB_c3_extra_seed%d.png', SEEDS(s))), ...
                   'Resolution', 150);
end
fprintf('figures: %s/formB_c3_extra_seed{23,42}.png\n', res_dir);
