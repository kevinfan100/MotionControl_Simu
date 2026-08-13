function res = probe_seed_spread_onset(opts)
%PROBE_SEED_SPREAD_ONSET  When does the seed-to-seed spread in a_bar appear?
%
% STATUS: ACTIVE | scratch probe, no production dependency
%
%   Measured on 2026-08-12: on the canonical scenario the across-seed spread of
%   the gain error jumps 5.9x between t = 0.50 and t = 0.70 s -- i.e. right when
%   the descent starts -- while the flow amplification (a_bar' ratio) over that
%   window is only 1.2x. So the spread is NOT explained by the flow, and the
%   mechanism is unknown.
%
%   This probe separates two hypotheses by lengthening the initial hold:
%       H_time   the spread accumulates with TIME (noise integrating)
%                -> a longer hold should show it growing before motion starts
%       H_motion the spread is injected when MOTION starts
%                -> a longer hold should show it flat, then the same jump
%
%   Arms are the 4-state baseline (arm = 'base'); everything else is the
%   canonical scenario, only t_hold and T_sim move so the post-hold part is
%   identical in shape.
%
%   Figure -> derivation/figures/formC_seed_spread_onset.png
%       row 1  across-seed RMS of e_a vs time, both holds, aligned on t = 0
%       row 2  the same, aligned on MOTION START, so the jump lines up
%
%   Style: canonical house rules (18 pt bold sans, 2.0 pt axes, no grid, box on,
%   legend northoutside horizontal, no title, exportgraphics 150).

    if nargin < 1; opts = struct(); end
    if ~isfield(opts, 'seeds');  opts.seeds  = [7 11 23 42 101 777 27 31]; end
    if ~isfield(opts, 'holds');  opts.holds  = [0.5 2.0]; end
    if ~isfield(opts, 'Ts');     opts.Ts     = 6.25e-4; end

    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    fig_dir = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    AX = 3;  nH = numel(opts.holds);  res = cell(nH, 1);

    for h = 1:nH
        th = opts.holds(h);
        cov = struct('t_hold', th, 'T_sim', 4.8 + (th - 0.5));
        o = run_formC_dist(struct('arm', 'base', 'seeds', opts.seeds, ...
                                  'verbose', false, 'config_override', cov));
        N = size(o.runs{1}.a_bar_hat_out, 1);
        E = zeros(N, numel(opts.seeds));
        for q = 1:numel(opts.seeds)
            r  = o.runs{q};
            ad = r.a_hat_out(1, AX) / r.a_bar_hat_out(1, AX);
            E(:, q) = r.a_bar_hat_out(:, AX) - r.a_true_out(:, AX) / ad;
        end
        s = struct();
        s.t_hold = th;
        s.t      = (0:N-1).' * opts.Ts;
        s.rms    = sqrt(mean(E.^2, 2));
        s.sd     = std(E, 0, 2);
        s.hb     = o.runs{1}.h_bar_true_out;
        res{h}   = s;

        w = s.t <= th;
        fprintf(['hold %.1f s : spread at t=0.1 %.3e -> at motion start %.3e ' ...
                 '(x%.2f over the hold) | peak %.3e at t=%.2f\n'], th, ...
                s.sd(find(s.t >= 0.1, 1)), s.sd(find(w, 1, 'last')), ...
                s.sd(find(w, 1, 'last')) / s.sd(find(s.t >= 0.1, 1)), ...
                max(s.sd), s.t(find(s.sd == max(s.sd), 1)));
    end

    local_page(res, fullfile(fig_dir, 'formC_seed_spread_onset.png'));
    fprintf('[spread onset] wrote -> %s\n', fig_dir);
end

% --------------------------------------------------------------------------
function local_page(res, out)
    COL = [0 0.20 0.90; 0.93 0.49 0.06];
    FS = 18; LFS = 13; AXLW = 2.0; LW = 2.0;

    f = figure('Position', [60 60 1300 900], 'Color', 'w', ...
               'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(f, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    a1 = nexttile; hold(a1, 'on');  hh = gobjects(1, numel(res));
    for h = 1:numel(res)
        s = res{h};
        hh(h) = semilogy(a1, s.t, s.sd, '-', 'Color', COL(h, :), 'LineWidth', LW, ...
             'DisplayName', sprintf('hold %.1f s', s.t_hold));
        xline(a1, s.t_hold, '--', 'Color', COL(h, :), 'LineWidth', 1.4, ...
              'HandleVisibility', 'off');
    end
    set(a1, 'YScale', 'log');
    ylabel(a1, 'across-seed sd of e_a', 'FontSize', FS, 'FontWeight', 'bold');
    legend(a1, hh, 'Location', 'northoutside', 'Orientation', 'horizontal', ...
           'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    xlabel(a1, 'time  [s]   (dashed = motion start)', 'FontSize', FS, 'FontWeight', 'bold');
    local_ax(a1, FS, AXLW);

    a2 = nexttile; hold(a2, 'on');
    for h = 1:numel(res)
        s = res{h};
        semilogy(a2, s.t - s.t_hold, s.sd, '-', 'Color', COL(h, :), 'LineWidth', LW);
    end
    xline(a2, 0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.4);
    set(a2, 'YScale', 'log');
    xlim(a2, [-2.1 3.5]);
    ylabel(a2, 'across-seed sd of e_a', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel(a2, 'time since motion start  [s]', 'FontSize', FS, 'FontWeight', 'bold');
    local_ax(a2, FS, AXLW);

    exportgraphics(f, out, 'Resolution', 150);
    close(f);
end

function local_ax(a, FS, AXLW)
    set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, ...
           'Box', 'on', 'TickLabelInterpreter', 'tex');
    grid(a, 'off');
end
