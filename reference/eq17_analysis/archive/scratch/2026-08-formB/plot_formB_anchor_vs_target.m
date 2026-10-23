function res = plot_formB_anchor_vs_target(opts)
%PLOT_FORMB_ANCHOR_VS_TARGET  The estimate, and the target it is supposed to
%   reach, for two truth functions and two writings.
%
%   STATUS: ACTIVE -- reads test_results/temp_formB_two_boundaries.mat.
%   Candidate page 6 of formB_amp_functions.
%
%   Three lines per panel, and they are the whole story:
%       grey dotted   the ANCHOR -- one number, fixed before the run from the
%                     published far-field limit (9/8 perpendicular, 9/16
%                     parallel). Nothing is fitted.
%       red dashed    b_eff(w(t)) -- the constant the truth demands at the
%                     height the particle is at right now. A moving target,
%                     because the law's shape is not the truth's shape.
%       blue          the carried constant. Solid = free (estimated),
%                     dashed = locked (pinned at the anchor for the whole run).
%
%   "How far the anchor is from b_eff" is the vertical gap between the grey
%   line and the red curve. Where that gap is small there is nothing to
%   estimate; where it is large the estimator has something to do.
%
%   NOTE on "locked": only the SHAPE constant (slot 5) is pinned. The gain
%   state a_bar (slot 4) is estimated in both arms. So free-minus-locked is
%   the marginal value of also freeing the shape constant, not "estimator on
%   versus off".
%
%   Figure -> derivation/figures/formB_anchor_vs_target.png

    if nargin < 1; opts = struct(); end
    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));
    fig_dir = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    L = load(fullfile(root, 'test_results', 'temp_formB_two_boundaries.mat'));
    r = L.res;

    ANC = r.anchor;                       % [9/8 (perp), 9/16 (para)]
    BND = {'c-perp  (wall-normal truth)', 'c-para  (wall-parallel truth)'};
    WRT = {'Form B  (length)', 'Form C  (amplitude)'};
    YL  = {[1.03 1.17], [0.33 0.62]};

    t = r.trace{1,1,1}.t;  w = r.trace{1,1,1}.w;
    ok = w > 1.01;
    cP = nan(size(w)); cA = nan(size(w));
    for i = 1:numel(w)
        if ok(i)
            [cA(i), cP(i)] = calc_correction_functions(w(i), true);   % [para, perp]
        end
    end
    C = {cP, cA};                          % boundary index -> truth curve

    FS = 17; RED = [0.8 0 0]; BLU = [0 0.2 0.9]; GRY = [0.45 0.45 0.45];
    tb = [0.5 1.5 3.5];
    f = figure('Position', [40 40 1280 800], 'Color', 'w', 'Visible', 'off');
    res = struct();
    for ib = 1:2
        c = C{ib};
        for iw = 1:2
            if iw == 1; be = (c - 1) .* (w - 1); else; be = w .* (c - 1) ./ c; end
            k = (ib - 1) * 2 + iw;
            ax = subplot(2, 2, k); hold(ax, 'on');
            ha = plot(ax, [t(1) t(end)], ANC(ib) * [1 1], ':', 'Color', GRY, 'LineWidth', 2.2);
            ht = plot(ax, t, be, '--', 'Color', RED, 'LineWidth', 2.2);
            hl = plot(ax, r.trace{ib,iw,2}.t, r.trace{ib,iw,2}.b, '--', ...
                      'Color', BLU, 'LineWidth', 1.6);
            hf = plot(ax, r.trace{ib,iw,1}.t, r.trace{ib,iw,1}.b, '-', ...
                      'Color', BLU, 'LineWidth', 2.4);
            % the gap that decides everything, marked at the final hold
            bh = mean(be(t > 3.8));
            plot(ax, [4.45 4.45], [ANC(ib) bh], '-', 'Color', [0.15 0.55 0.25], ...
                 'LineWidth', 3.5);
            text(ax, 4.38, (ANC(ib) + bh) / 2, sprintf('%.4f', abs(ANC(ib) - bh)), ...
                 'HorizontalAlignment', 'right', 'FontSize', FS - 6, ...
                 'FontWeight', 'bold', 'Color', [0.10 0.42 0.18]);
            ylim(ax, YL{ib}); xlim(ax, [t(1) t(end)]);
            if iw == 1
                ylabel(ax, 'constant  [-]', 'FontSize', FS - 2, 'FontWeight', 'bold');
            end
            if ib == 2
                xlabel(ax, 'time  [s]', 'FontSize', FS - 2, 'FontWeight', 'bold');
            end
            text(ax, 0.03, 0.93, sprintf('%s   %s', BND{ib}, WRT{iw}), ...
                 'Units', 'normalized', 'FontSize', FS - 6, 'FontWeight', 'bold', 'Interpreter', 'none');
            if k == 1
                legend([ha ht hf hl], {'anchor (fixed, published)', ...
                        'b_{eff}(w(t))  demanded by the truth', ...
                        'carried, free', 'carried, locked'}, ...
                       'Location', 'northoutside', 'Orientation', 'horizontal', ...
                       'FontSize', FS - 7, 'NumColumns', 2);
            end
            yl = ylim(ax);
            for x = tb
                plot(ax, [x x], yl, '--', 'Color', [0.62 0.62 0.62], ...
                     'LineWidth', 0.9, 'HandleVisibility', 'off');
            end
            ylim(ax, yl);
            set(ax, 'FontSize', FS - 5, 'FontWeight', 'bold', 'LineWidth', 1.3, 'Box', 'on');
            grid(ax, 'off');
            res.gap(ib, iw) = abs(ANC(ib) - bh);
        end
    end
    out = fullfile(fig_dir, 'formB_anchor_vs_target.png');
    exportgraphics(f, out, 'Resolution', 150); close(f);

    fprintf('=== 錨 與 b_eff 的距離（末 hold）===\n');
    nm = {'c_perp', 'c_para'}; wn = {'Form B', 'Form C'};
    for ib = 1:2
        for iw = 1:2
            fprintf('  %-8s %-8s  anchor %.4f   b_eff %.4f   gap %.4f\n', ...
                    nm{ib}, wn{iw}, ANC(ib), ANC(ib) - sign(1) * res.gap(ib,iw) * ...
                    sign(ANC(ib) - ANC(ib) + 1), res.gap(ib, iw));
        end
    end
    fprintf('wrote %s\n', out);
end
