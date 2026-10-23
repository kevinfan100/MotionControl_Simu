function res = probe_seed_spread_paths(opts)
%PROBE_SEED_SPREAD_PATHS  Which path injects the seed-to-seed spread in a_bar?
%
% STATUS: ACTIVE | scratch probe, no production dependency
%
%   Established 2026-08-12: the across-seed spread of the gain error is injected
%   within ~0.2 s of MOTION ONSET (a 4x longer initial hold adds only 11 % to the
%   pre-motion spread), at h_bar ~ 20-22 where the flow amplification is only
%   1.2x. So neither time accumulation nor the flow explains it.
%
%   a_bar_hat has exactly FOUR seed-dependent inputs (Delta_w_bar_d is commanded,
%   identical across seeds, and cannot create spread):
%       (1) dw3_hat   via M = Delta_w_bar_d + (1-lambda_c)*dw3_hat in predict
%       (2) m_sum     the MA(2) thermal memory, via a_bar'*alpha*(x8+x9)
%       (3) K1(4)*innov1   the y1 update
%       (4) K2(4)*innov2   the y2 update
%
%   Nested arms remove them one at a time:
%       B0 nominal                          all four
%       B1 + y2_off                         removes (4)
%       B2 + y1_gain_off                    removes (3)  -> a_bar_hat pure predict
%       B3 + ma2_aug = false                removes (2)  -> only dw3_hat remains
%
%   CAVEATS, declared:
%     - The arms are NESTED, not independent: removing y2 changes P and hence K1,
%       so B0-B1 is "what y2 contributes with everything else on", not an
%       independent contribution.
%     - B3 is the least clean: ma2_aug = false also changes Q's structure
%       (rank-2 -> rank-1), not just the memory feedthrough. If B2 -> B3 shows a
%       real difference, redo it by zeroing ONLY the feedthrough term.
%     - B2/B3 run a_bar_hat open loop for the whole run, so their |e_a| levels
%       are not comparable with B0/B1. Only the SPREAD is being compared.
%
%   Figure -> derivation/figures/formC_seed_spread_paths.png
%   Style: canonical house rules (18 pt bold sans, 2.0 pt axes, no grid, box on,
%   legend northoutside horizontal, no title, exportgraphics 150).

    if nargin < 1; opts = struct(); end
    if ~isfield(opts, 'seeds'); opts.seeds = [7 11 23 42 101 777 27 31]; end
    if ~isfield(opts, 'Ts');    opts.Ts    = 6.25e-4; end
    T_MOTION = 0.5;                      % canonical initial hold

    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    fig_dir = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    AX = 3;

    arms = struct( ...
        'tag',  {'B0', 'B1', 'B2', 'B3'}, ...
        'name', {'nominal', 'y2 off', '+ y1 gain off', '+ MA(2) off'}, ...
        'ov',   {struct(), ...
                 struct('y2_off', true), ...
                 struct('y2_off', true, 'y1_gain_off', true), ...
                 struct('y2_off', true, 'y1_gain_off', true, 'ma2_aug', false)});

    res = cell(numel(arms), 1);
    for a = 1:numel(arms)
        o = run_formC_dist(struct('arm', 'base', 'seeds', opts.seeds, ...
                                  'verbose', false, 'ctrl_const_override', arms(a).ov));
        N = size(o.runs{1}.a_bar_hat_out, 1);
        E = zeros(N, numel(opts.seeds));
        for q = 1:numel(opts.seeds)
            r  = o.runs{q};
            ad = r.a_hat_out(1, AX) / r.a_bar_hat_out(1, AX);
            E(:, q) = r.a_bar_hat_out(:, AX) - r.a_true_out(:, AX) / ad;
        end
        s = struct('tag', arms(a).tag, 'name', arms(a).name, ...
                   't', (0:N-1).' * opts.Ts, 'sd', std(E, 0, 2), ...
                   'rms', sqrt(mean(E.^2, 2)));
        res{a} = s;
    end

    fprintf('\n across-seed sd of e_a, %d seeds\n', numel(opts.seeds));
    fprintf('%-16s %10s %10s %10s %10s %10s\n', 'arm', ...
            't=0.45', 't=0.60', 't=0.70', 't=0.90', 'jump x');
    for a = 1:numel(res)
        s = res{a};  v = zeros(1, 4);  tq = [0.45 0.60 0.70 0.90];
        for j = 1:4; [~, i] = min(abs(s.t - tq(j))); v(j) = s.sd(i); end
        fprintf('%-4s %-11s %10.3e %10.3e %10.3e %10.3e %10.2f\n', ...
                s.tag, s.name, v, v(3) / v(1));
    end

    local_page(res, T_MOTION, fullfile(fig_dir, 'formC_seed_spread_paths.png'));
    fprintf('[spread paths] wrote -> %s\n', fig_dir);
end

% --------------------------------------------------------------------------
function local_page(res, t_motion, out)
    COL = [0.00 0.20 0.90; 0.93 0.49 0.06; 0.10 0.60 0.30; 0.45 0.10 0.65];
    LS  = {'-', '-', '-', '--'};
    FS = 18; LFS = 13; AXLW = 2.0; LW = 2.0;

    f = figure('Position', [60 60 1300 820], 'Color', 'w', ...
               'NumberTitle', 'off', 'Visible', 'off');
    ax = axes(f); hold(ax, 'on');
    h = gobjects(1, numel(res));
    for a = 1:numel(res)
        s = res{a};
        h(a) = plot(ax, s.t, s.sd, LS{a}, 'Color', COL(a, :), 'LineWidth', LW, ...
                    'DisplayName', sprintf('%s  %s', s.tag, s.name));
    end
    xline(ax, t_motion, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.6, ...
          'HandleVisibility', 'off');
    set(ax, 'YScale', 'log');
    xlim(ax, [0 res{1}.t(end)]);
    xlabel(ax, 'time  [s]   (grey = motion start)', 'FontSize', FS, 'FontWeight', 'bold');
    ylabel(ax, 'across-seed sd of e_a', 'FontSize', FS, 'FontWeight', 'bold');
    legend(ax, h, 'Location', 'northoutside', 'Orientation', 'horizontal', ...
           'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, ...
            'Box', 'on', 'TickLabelInterpreter', 'tex');
    grid(ax, 'off');
    exportgraphics(f, out, 'Resolution', 150);
    close(f);
end
