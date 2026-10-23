function res = analyze_formB_track_vs_gain(opts)
%ANALYZE_FORMB_TRACK_VS_GAIN  Why does b standing still cost nothing, while
%   beta tracking hard still costs 3-6x more gain error?
%
%   STATUS: ACTIVE -- reads test_results/temp_formB_amp_stage4.mat.
%   Feeds reference/eq17_analysis/derivation/formB_amp_bonly_probe.tex.
%
%   Both writings share an exact property: substitute the constant the truth
%   demands at the current height and the law returns the truth exactly.
%       Form B:  b = b_eff,B = (c-1)(w-1)   ->  a_bar = 1/c
%       Form C:  beta = b_eff,C = w(c-1)/c  ->  a_bar = 1 - b_eff/w = 1/c
%   So in BOTH cases the gain error is not caused by the constant moving or
%   standing still -- it is caused by the MISMATCH between the constant the
%   filter carries and the one the truth is asking for at that instant:
%
%       e_a  ~=  (dA/dtheta) * (theta_hat - theta_eff) / a_true
%
%   That single expression resolves the apparent paradox. The length arm's
%   constant does not move because its target does not move; the amplitude
%   arm's constant moves a lot because its target moves a lot -- and a
%   constant carried with Q = 0 cannot follow a target that swings within
%   every cycle, so tracking hard is not the same as arriving.
%
%   This script measures both factors separately (how far the target moves,
%   and how much gain a unit mismatch costs) and checks the predicted error
%   against the observed one.
%
%   Figure -> derivation/figures/formB_track_vs_gain.png

    if nargin < 1; opts = struct(); end
    if ~isfield(opts, 'seed'); opts.seed = 7; end

    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));
    fig_dir = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    L = load(fullfile(root, 'test_results', 'temp_formB_amp_stage4.mat'));
    r = L.res;
    q = find(r.seeds == opts.seed, 1);
    iL = find(strcmp(r.arm_names, 'len_bonly'));
    iA = find(strcmp(r.arm_names, 'amp'));
    aL = r.runs{iL, q};  aA = r.runs{iA, q};

    pc = physical_constants();
    a_disp = (pc.Ts / (pc.gamma_N * pc.R)) * pc.R;
    t  = aA.t;  w = aA.h_bar_d;
    ok = w > 1.01;

    % ---- what the truth demands, per writing --------------------------
    c = nan(size(w));
    for i = 1:numel(w)
        if ok(i); [~, c(i)] = calc_correction_functions(w(i), true); end
    end
    a_true = 1 ./ c;
    beffL = (c - 1) .* (w - 1);        % length reading
    beffC = w .* (c - 1) ./ c;         % amplitude reading

    % ---- carried constant (the PRIOR value the law was evaluated at) ---
    bL = [aL.b_hat(1); aL.b_hat(1:end-1)];
    bA = [aA.b_hat(1); aA.b_hat(1:end-1)];

    % ---- level sensitivity dA/dtheta at the carried constant ----------
    dAdL = -(w - 1) ./ ((w - 1) + bL).^2;      % Form B, p = 1
    dAdC = -1 ./ w;                            % Form C

    % ---- predicted vs observed gain error -----------------------------
    predL = 100 * dAdL .* (bL - beffL) ./ a_true;
    predC = 100 * dAdC .* (bA - beffC) ./ a_true;
    obsL  = 100 * (aL.a_hat / a_disp - a_true) ./ a_true;
    obsC  = 100 * (aA.a_hat / a_disp - a_true) ./ a_true;

    % ---- the two factors, over the oscillation window -----------------
    t3 = 1.5; t4 = 3.5;                        % oscillation window
    m  = ok & t > t3 & t <= t4;
    swingL = max(beffL(m)) - min(beffL(m));
    swingC = max(beffC(m)) - min(beffC(m));
    sensL  = mean(abs(dAdL(m)));
    sensC  = mean(abs(dAdC(m)));

    fprintf('\n=== why b standing still is free and beta tracking is not (seed %d) ===\n', opts.seed);
    fprintf('over the oscillation window w_bar in [%.2f, %.2f]:\n', min(w(m)), max(w(m)));
    fprintf('  target swing  |b_eff| :  length %.4f   amplitude %.4f   (%.2fx)\n', ...
            swingL, swingC, swingC / swingL);
    fprintf('  sensitivity |dA/dth| :  length %.4f   amplitude %.4f   (%.2fx)\n', ...
            sensL, sensC, sensC / sensL);
    fprintf('  product (gain cost of a full target swing)        -> %.2fx\n', ...
            (swingC * sensC) / (swingL * sensL));
    fprintf('  mismatch |theta_hat - theta_eff| RMS: length %.4f  amplitude %.4f\n', ...
            rms(bL(m) - beffL(m)), rms(bA(m) - beffC(m)));
    fprintf('  gain error RMS  observed  : length %+.2f %%   amplitude %+.2f %%\n', ...
            rms(obsL(m)), rms(obsC(m)));
    fprintf('  gain error RMS  predicted : length %+.2f %%   amplitude %+.2f %%\n', ...
            rms(predL(m)), rms(predC(m)));
    fprintf('  predicted / observed      : length %.2f       amplitude %.2f\n', ...
            rms(predL(m)) / rms(obsL(m)), rms(predC(m)) / rms(obsC(m)));

    res = struct('swingL', swingL, 'swingC', swingC, 'sensL', sensL, 'sensC', sensC);

    % ---- figure --------------------------------------------------------
    FS = 17; BLUE = [0 0.2 0.9]; RED = [0.8 0 0]; GREY = [0.45 0.45 0.45];
    tb = [0.5 1.5 3.5];
    f = figure('Position', [40 40 1250 900], 'Color', 'w', 'Visible', 'off');
    NAMES = {'Form B (length), only b free', 'Form C (amplitude), only \beta free'};
    for cc = 1:2
        if cc == 1
            bb = bL; be = beffL; pr = predL; ob = obsL; yl = [1.05 1.16];
        else
            bb = bA; be = beffC; pr = predC; ob = obsC; yl = [1.02 1.16];
        end
        % row 1: carried constant vs demanded constant
        ax = subplot(3, 2, cc); hold(ax, 'on');
        h1 = plot(ax, t, be, '--', 'Color', RED, 'LineWidth', 2.2);
        h2 = plot(ax, t, bb, '-', 'Color', BLUE, 'LineWidth', 2.0);
        ylim(ax, yl); xlim(ax, [t(1) t(end)]);
        ylabel(ax, 'constant  [-]', 'FontSize', FS, 'FontWeight', 'bold');
        legend([h1 h2], {'demanded  \theta_{eff}(w(t))', 'carried  \theta_{hat}'}, ...
               'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS - 5);
        text(ax, 0.03, 0.10, NAMES{cc}, 'Units', 'normalized', ...
             'FontSize', FS - 4, 'FontWeight', 'bold');
        local_ax(ax, FS, tb);
        % row 2: the mismatch
        ax = subplot(3, 2, 2 + cc); hold(ax, 'on');
        plot(ax, t, zeros(size(t)), '-', 'Color', GREY, 'LineWidth', 1.2);
        plot(ax, t, bb - be, '-', 'Color', BLUE, 'LineWidth', 2.0);
        ylim(ax, [-0.09 0.09]); xlim(ax, [t(1) t(end)]);
        ylabel(ax, '\theta_{hat} - \theta_{eff}', 'FontSize', FS, 'FontWeight', 'bold');
        local_ax(ax, FS, tb);
        % row 3: predicted vs observed gain error
        ax = subplot(3, 2, 4 + cc); hold(ax, 'on');
        plot(ax, t, zeros(size(t)), '-', 'Color', GREY, 'LineWidth', 1.2);
        h3 = plot(ax, t, ob, '-', 'Color', BLUE, 'LineWidth', 2.0);
        h4 = plot(ax, t, pr, '--', 'Color', RED, 'LineWidth', 1.8);
        ylim(ax, [-12 12]); xlim(ax, [t(1) t(end)]);
        ylabel(ax, 'a_z error  [%]', 'FontSize', FS, 'FontWeight', 'bold');
        xlabel(ax, 'time  [s]', 'FontSize', FS, 'FontWeight', 'bold');
        legend([h3 h4], {'observed', 'predicted from the mismatch'}, ...
               'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS - 5);
        local_ax(ax, FS, tb);
    end
    out = fullfile(fig_dir, 'formB_track_vs_gain.png');
    exportgraphics(f, out, 'Resolution', 150); close(f);
    fprintf('wrote %s\n', out);
end

function local_ax(ax, FS, tb)
    yl = ylim(ax);
    for x = tb
        plot(ax, [x x], yl, '--', 'Color', [0.55 0.55 0.55], 'LineWidth', 1.0, ...
             'HandleVisibility', 'off');
    end
    ylim(ax, yl);
    set(ax, 'FontSize', FS - 4, 'FontWeight', 'bold', 'LineWidth', 1.3, 'Box', 'on');
    grid(ax, 'off');
end
