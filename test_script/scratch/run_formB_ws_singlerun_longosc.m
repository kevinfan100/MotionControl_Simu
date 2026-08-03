% run_formB_ws_singlerun_longosc.m -- PURPOSE: refute "single-run real-time
%   w_s estimation is impossible" (user challenge 2026-08-02). The canonical
%   4.8 s scenario is control-designed, not identification-designed: it holds
%   only ~3.3 s near the wall, so one run buys +-8 %. Same filter, same honest
%   prior 0.111, same c4 injection (+5 % TRUE wall) -- only the trajectory
%   changes: n_cycles 2 -> 32 (near-wall 3.3 s -> 33 s, T = 34.8 s). If wall
%   accuracy = f(near-wall data seconds) is right, ONE run must converge
%   real-time to the +-3 % that the 8-run chain buys, with no chain at all.
%   EXPIRES: finding recorded (doc / memory); identification-trajectory
%   design, if adopted, becomes its own integration item.
%   Style: .claude/rules/figure-style.md.

here = fileparts(mfilename('fullpath'));
proj = fullfile(here, '..', '..');
pdirs = {'model', 'model/dual_track', 'model/diag', 'model/thermal_force', ...
         'model/config', 'model/wall_effect', 'model/controller', ...
         'model/trajectory', 'test_script/integration'};
for i = 1:numel(pdirs); addpath(fullfile(proj, pdirs{i})); end
res_dir  = fullfile(proj, 'test_results');
res_file = fullfile(res_dir, 'run_formB_ws_singlerun_longosc.mat');

AX    = 3;
SEEDS = [7 11];
INJ   = 0.05;
N_CYC = 32;
FS    = 18;

ov = struct('lock_b', false, 'lock_p', true, 'lock_ws', false, 'Pf_ws_std', 0.111);

if exist(res_file, 'file')
    load(res_file, 'L');
else
    L = struct('done', zeros(1, numel(SEEDS)));
end
for s = 1:numel(SEEDS)
    if L.done(s); continue; end
    o = struct('seeds', SEEDS(s), 'ws_inject', INJ, 'verbose', false, ...
               'ctrl_const_override', ov, ...
               'config_override', struct('n_cycles', N_CYC, ...
                                         'T_sim', 0.5 + 1.0 + N_CYC + 1.3));
    out = run_formB_ws(o);
    r = out.runs{1};
    L.t{s}   = r.tout(:);
    L.dev{s} = 100 * (r.ws_hat_out(:, AX) - 1);
    L.sqP{s} = 100 * r.P_ws_out(:, AX);
    L.done(s) = 1;  save(res_file, 'L');
    fprintf('seed %d: T %.1f s  ws_end dev %+.2f%%  sqP_end %.2f%%\n', ...
            SEEDS(s), L.t{s}(end), L.dev{s}(end), L.sqP{s}(end));
end

fprintf('\n== single long run (n_cycles = %d, T = %.1f s), c4 conditions ==\n', ...
        N_CYC, L.t{1}(end));
for s = 1:numel(SEEDS)
    k48 = find(L.t{s} <= 4.8, 1, 'last');
    fprintf(['seed %3d | @4.8 s (canonical end): dev %+6.2f%%  sqP %5.2f%% | ', ...
             'end: dev %+6.2f%%  err %+5.2f%%  sqP %5.2f%%  z %5.2f\n'], ...
            SEEDS(s), L.dev{s}(k48), L.sqP{s}(k48), L.dev{s}(end), ...
            L.dev{s}(end) - 100 * INJ, L.sqP{s}(end), ...
            abs(L.dev{s}(end) - 100 * INJ) / L.sqP{s}(end));
end

for s = 1:numel(SEEDS)
    fig = figure('Color', 'w', 'Position', [80 60 980 560]); drawnow;
    ax = axes(fig); hold(ax, 'on');
    ht = plot(ax, [0, L.t{s}(end)], 100 * INJ * [1 1], 'r', 'LineWidth', 2.4);
    plot(ax, [0, L.t{s}(end)], [0 0], 'k:', 'LineWidth', 1.4, 'HandleVisibility', 'off');
    hb = plot(ax, L.t{s}, L.dev{s} + 2 * L.sqP{s}, 'k--', 'LineWidth', 1.0);
    plot(ax, L.t{s}, L.dev{s} - 2 * L.sqP{s}, 'k--', 'LineWidth', 1.0, ...
         'HandleVisibility', 'off');
    he = plot(ax, L.t{s}, L.dev{s}, 'b', 'LineWidth', 1.8);
    hc = plot(ax, 4.8 * [1 1], [-25 25], '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.4);
    ylabel(ax, '(ws_{hat} - 1)  [% of R]', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel(ax, 'time  [s]', 'FontSize', FS, 'FontWeight', 'bold');
    set(ax, 'FontSize', FS, 'FontWeight', 'bold'); box(ax, 'on');
    ylim(ax, [-25 25]);
    legend(ax, [ht he hb hc], {'true wall (ws = 1.05)', 'ws_{hat} (real time)', ...
           'claimed \pm2\surdP', 'canonical run ends (4.8 s)'}, ...
           'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS - 4);
    exportgraphics(fig, fullfile(res_dir, ...
        sprintf('formB_singlerun_longosc_seed%d.png', SEEDS(s))), 'Resolution', 150);
end
fprintf('figures: %s/formB_singlerun_longosc_seed{7,11}.png\n', res_dir);
