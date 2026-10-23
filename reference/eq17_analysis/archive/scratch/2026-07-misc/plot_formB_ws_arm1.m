% FORK OF test_script/integration/plot_5state_powerlaw.m @ 9a082e3 | PURPOSE: tier-1 arm-1
% result figures for the Form B (w_s) gain law | EXPIRES: when the Form B shape is settled
% (b/p identifiability closed) or the arm is superseded | 產線改動不會自動跟上
%
% plot_formB_ws_arm1.m -- figures for run_formB_ws() arm 1 (t1_y2on, 6 seeds).
%   Lab figure convention (.claude/rules/figure-style.md): no grid, no title, box on,
%   legend northoutside horizontal, True = red / Estimate = blue / Measured = light blue,
%   FontSize 18 bold, exportgraphics Resolution 150. All statistics go to the console.
%
%   Outputs (test_results/, gitignored):
%       formB_arm1_seed7.png    typical seed
%       formB_arm1_seed11.png   the P[0]-budget violator
%       formB_arm1_bp_plane.png (b_hat, p_hat) ridge wander, all 6 seeds
%
%   Panel (c) carries two estimated parameters, so it departs from the three-role
%   colour rule: b_hat keeps Estimate blue, p_hat takes a second hue to stay legible.
%   b_hat / p_hat are DIAGNOSTIC ONLY -- neither is a controller output.

here = fileparts(mfilename('fullpath'));
proj = fullfile(here, '..', '..');

AX_Z         = 3;        % wall-normal axis (perpendicular)
NM           = 1e3;      % um/pN -> nm/pN display scale
B_ANCHOR     = 9/8;      % derived far-field reflection coefficient (b seed)
P_ANCHOR     = 1;        % derived far-field power (p seed)
PRIOR_STD_BP = 1/8;      % sqrt(P[0]) for both b and p
B_MINIMAX    = 1.0354;   % minimax shape pair, oracle arm
P_MINIMAX    = 0.9500;
ERR_BAND_PCT = 2;        % +-2 % reference lines on the error panel
SEED_TYPICAL = 7;
SEED_VIOLATE = 11;

FS         = 18;
RED        = 'r';
BLUE       = 'b';
LIGHT_BLUE = [0.45 0.72 0.95];   % Measured / raw readout
P_HUE      = [0.85 0.33 0.10];   % second estimated parameter (panel c only)

% ---- data: reuse the saved arm, regenerate it if the gitignored .mat is gone ----
res_dir  = fullfile(proj, 'test_results');
mat_file = fullfile(res_dir, 'run_formB_ws_t1_y2on.mat');
if exist(mat_file, 'file')
    S = load(mat_file);  out = S.out;
else
    fprintf('%s missing -- re-running arm 1 to regenerate it.\n', mat_file);
    out = run_formB_ws();
end
seeds = out.seeds;
cfg   = out.cfg;
tb    = [cfg.t_hold, cfg.t_hold + cfg.t_descend_override];   % phase boundaries [s]

% ================= per-seed three-panel figures =================
for target = [SEED_TYPICAL, SEED_VIOLATE]
    q = find(seeds == target, 1);
    assert(~isempty(q), 'plot_formB_ws_arm1:seedMissing', 'seed %d not in the arm.', target);
    r = out.runs{q};

    t    = r.tout(:);
    aT   = r.a_true_out(:, AX_Z);      % [um/pN] true motion gain
    aH   = r.a_hat_out(:, AX_Z);       % [um/pN] EKF estimate
    aXM  = r.a_xm_out(:, AX_Z);        % [um/pN] raw gain readout
    bH   = r.b_hat_out(:, AX_Z);       % [-]
    pH   = r.p_hat_out(:, AX_Z);       % [-]
    errz = 100 * (aH - aT) ./ aT;      % [%]

    fig = figure('Color', 'w', 'Position', [80 40 900 1000]);

    % (a) gain: true / estimate / raw readout
    ax1 = subplot(3, 1, 1); hold(ax1, 'on');
    hx = plot(ax1, t, aXM * NM, 'Color', LIGHT_BLUE, 'LineWidth', 0.5);
    ht = plot(ax1, t, aT  * NM, RED,  'LineWidth', 2.6);
    he = plot(ax1, t, aH  * NM, BLUE, 'LineWidth', 2.0);
    ylabel(ax1, 'a_z  [nm/pN]', 'FontSize', FS, 'FontWeight', 'bold');
    legend([ht he hx], {'a_{true}', 'a_{hat}', 'a_{wm} (raw)'}, ...
           'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS - 4);
    ylim(ax1, [0 24]); style_ax(ax1, FS); xlim(ax1, [t(1) t(end)]); add_phase(ax1, tb);

    % (b) relative gain error
    ax2 = subplot(3, 1, 2); hold(ax2, 'on');
    h0  = plot(ax2, t, zeros(size(t)), RED, 'LineWidth', 2.0);
    he2 = plot(ax2, t, errz, BLUE, 'LineWidth', 2.0);
    plot(ax2, t,  ERR_BAND_PCT * ones(size(t)), 'k--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
    plot(ax2, t, -ERR_BAND_PCT * ones(size(t)), 'k--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
    ylabel(ax2, 'a_z error  [%]', 'FontSize', FS, 'FontWeight', 'bold');
    legend([h0 he2], {'zero', '(a_{hat}-a_{true})/a_{true}'}, ...
           'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS - 4);
    % shared with plot_formB_ws_compare.m so the config pages stay visually comparable
    ylim(ax2, [-14 14]); style_ax(ax2, FS); xlim(ax2, [t(1) t(end)]); add_phase(ax2, tb);

    % (c) shape parameters, raw per-step, against their derived anchors
    ax3 = subplot(3, 1, 3); hold(ax3, 'on');
    hb  = plot(ax3, t, bH, 'Color', BLUE,  'LineWidth', 1.8);
    hp  = plot(ax3, t, pH, 'Color', P_HUE, 'LineWidth', 1.8);
    hba = plot(ax3, t, B_ANCHOR * ones(size(t)), 'k--', 'LineWidth', 1.2);
    plot(ax3, t, P_ANCHOR * ones(size(t)), 'k:', 'LineWidth', 1.4, 'HandleVisibility', 'off');
    ylabel(ax3, 'shape params', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel(ax3, 'time  [s]', 'FontSize', FS, 'FontWeight', 'bold');
    legend([hb hp hba], {'b_{hat} (diag only)', 'p_{hat} (diag only)', 'anchors 9/8, 1'}, ...
           'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS - 6);
    ylim(ax3, [0.70 1.30]); style_ax(ax3, FS); xlim(ax3, [t(1) t(end)]); add_phase(ax3, tb);

    outp = fullfile(res_dir, sprintf('formB_arm1_seed%d.png', target));
    exportgraphics(fig, outp, 'Resolution', 150);
    close(fig);

    m = out.metrics.rows(q, :);
    fprintf('FIGURE saved: %s\n', outp);
    fprintf('CAPTION: Form B arm-1 seed %d, z axis: gain tracking, relative error, and the diagnostic shape parameters over the hold-descend-oscillate-hold scenario.\n', target);
    fprintf('  seed %d: descent peak %.3f %%, osc RMS %.3f %%, hold mean %+.3f %%, b budget %.3f, p budget %.3f\n', ...
            target, m(1), m(2), m(3), m(4), m(5));
    fprintf('  b_hat range [%.4f %.4f] (travel %+.4f = %.2f prior widths), p_hat range [%.4f %.4f]\n', ...
            min(bH), max(bH), bH(end) - bH(1), (bH(end) - bH(1)) / PRIOR_STD_BP, min(pH), max(pH));
    fprintf('  a_wm raw clipped above %g nm/pN on %.1f %% of samples\n', 24, 100 * mean(aXM * NM > 24));
end

% ================= (b_hat, p_hat) plane, all seeds =================
fig = figure('Color', 'w', 'Position', [80 40 760 700]);
ax  = axes(fig); hold(ax, 'on');

% prior box: anchor +- one prior width in each parameter
bx = B_ANCHOR + PRIOR_STD_BP * [-1 1 1 -1 -1];
py = P_ANCHOR + PRIOR_STD_BP * [-1 -1 1 1 -1];
hbox = plot(ax, bx, py, 'k--', 'LineWidth', 1.4);

seed_col = lines(numel(seeds));
htraj = gobjects(numel(seeds), 1);
for q = 1:numel(seeds)
    r  = out.runs{q};
    bH = r.b_hat_out(:, AX_Z);
    pH = r.p_hat_out(:, AX_Z);
    htraj(q) = plot(ax, bH, pH, '-', 'Color', [seed_col(q, :) 0.75], 'LineWidth', 0.9);
    plot(ax, bH(1),   pH(1),   'o', 'Color', seed_col(q, :), 'MarkerSize', 6, ...
         'LineWidth', 1.2, 'HandleVisibility', 'off');
    plot(ax, bH(end), pH(end), 'o', 'Color', seed_col(q, :), 'MarkerFaceColor', seed_col(q, :), ...
         'MarkerSize', 9, 'HandleVisibility', 'off');
end

hanch = plot(ax, B_ANCHOR,  P_ANCHOR,  'kp', 'MarkerFaceColor', 'k', 'MarkerSize', 16);
hmini = plot(ax, B_MINIMAX, P_MINIMAX, 'rd', 'MarkerFaceColor', RED, 'MarkerSize', 12);

xlabel(ax, 'b_{hat}', 'FontSize', FS, 'FontWeight', 'bold');
ylabel(ax, 'p_{hat}', 'FontSize', FS, 'FontWeight', 'bold');
legend([htraj(1) hbox hanch hmini], ...
       {'seed trajectories', 'prior box \pm 1\sigma', 'anchor (9/8, 1)', 'minimax (1.0354, 0.95)'}, ...
       'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS - 7);
style_ax(ax, FS);
xlim(ax, [0.94 1.28]); ylim(ax, [0.74 1.15]);   % must contain the whole prior box

outp = fullfile(res_dir, 'formB_arm1_bp_plane.png');
exportgraphics(fig, outp, 'Resolution', 150);
close(fig);

fprintf('FIGURE saved: %s\n', outp);
fprintf('CAPTION: Form B arm-1 (b_hat, p_hat) plane: all six seeds start at the derived anchor and wander along a common ridge without converging on the minimax pair.\n');
bf = zeros(numel(seeds), 1); pf = bf;
for q = 1:numel(seeds)
    bf(q) = out.runs{q}.b_hat_out(end, AX_Z);
    pf(q) = out.runs{q}.p_hat_out(end, AX_Z);
end
fprintf('  final b_hat spread [%.4f %.4f] (mean %.4f), final p_hat spread [%.4f %.4f] (mean %.4f)\n', ...
        min(bf), max(bf), mean(bf), min(pf), max(pf), mean(pf));
fprintf('  seeds ending outside the prior box: %d of %d\n', ...
        sum(abs(bf - B_ANCHOR) > PRIOR_STD_BP | abs(pf - P_ANCHOR) > PRIOR_STD_BP), numel(seeds));
fprintf('  correlation of the final pairs: corr(b_f, p_f) = %+.3f\n', ...
        corr(bf, pf));


function style_ax(ax, FS)
    set(ax, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 1.3, 'Box', 'on');
    grid(ax, 'off');
    ax.Toolbar = [];   % keep the interactive toolbar out of exportgraphics
end

function add_phase(ax, tb)
    yl = ylim(ax);
    for x = tb
        plot(ax, [x x], yl, '--', 'Color', [0.55 0.55 0.55], 'LineWidth', 1.0, ...
             'HandleVisibility', 'off');
    end
    ylim(ax, yl);
end
