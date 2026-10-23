function out = check_delta_b_curve()
%CHECK_DELTA_B_CURVE  Stage 0, step 1 of the law-error budget: what is
%   delta_b(a_bar) = b_true(a_bar) - 8/9 along the canonical and Meng paths,
%   how big is the per-step forcing it puts on a_bar, and is that forcing
%   white or same-signed (the container question)?
%
%   out = check_delta_b_curve();
%
% STATUS: ACTIVE | arithmetic only; OFFLINE use of c(w) exactly as the
%   driver's local_envelope_b_range does (the controller never reads it).
%
% THE MISSING TERM (formC_state_b.tex, error-dynamics row 4):
%       e_law[k] = delta_b(a_bar[k]) * (1 - a_bar[k])^2 * dw_bar_d[k]
% This script measures, on the two command paths,
%   (1) delta_b(a_bar) itself and its two limits,
%   (2) e_law[k] per step and its running sum per segment,
%   (3) the correlation factor  |sum e| / sqrt(sum e^2)  -- 1 for white,
%       ~sqrt(N) for a same-signed run. This number decides the container.
%   (4) three candidate widths for Delta_b: the driver's envelope sup
%       (b_half, same provenance as the existing prior), the anchor gap
%       |1 - 8/9| (fully c-free), and the actual sup on the path.

    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'law_error_budget');
    if ~exist(od, 'dir'); mkdir(od); end
    B_ANCHOR = 8/9;

    % ---- (1) the curve --------------------------------------------------
    w = logspace(log10(1.02), log10(60), 4000).';
    [ab, bt] = local_b_true(w);
    db = bt - B_ANCHOR;
    fprintf('\nb_true limits:  w=1.02 -> %.4f   w=60 -> %.4f   (anchor 8/9 = %.4f, near-wall limit 1)\n', ...
            bt(1), bt(end), B_ANCHOR);
    [~, imn] = min(bt); [~, imx] = max(bt(w < 25));
    fprintf('on w in [1.02, 25]: min b_true %.4f at w %.2f (a %.3f); max %.4f at w %.2f (a %.3f)\n', ...
            bt(imn), w(imn), ab(imn), bt(imx), w(imx), ab(imx));

    % ---- (2)(3) along the two command paths ------------------------------
    P = struct();
    P.canonical = local_path_from_run(fullfile(root, 'test_results', 'run_formC_b_best_y2on.mat'));
    P.meng      = local_path_from_run(fullfile(root, 'test_results', 'inject_response', 'inject_response.mat'));
    NM = fieldnames(P);
    fprintf('\n%-10s %-11s %8s %10s %10s %10s %8s %8s\n', 'path', 'segment', 'n', ...
            'sum e', 'sqrt(sum e2)', 'corr fac', 'sup|db|', 'runs');
    for p = 1:numel(NM)
        S = P.(NM{p});
        [S.ab, S.bt] = local_b_true(S.h_d);
        S.db = S.bt - B_ANCHOR;
        S.dw = [0; diff(S.h_d)];
        S.e  = S.db .* (1 - S.ab).^2 .* S.dw;           % per-step forcing on a_bar
        for g = 1:size(S.SEG, 1)
            m = S.SEG{g, 2}(S.t);
            e = S.e(m);
            se = sum(e);  sw = sqrt(sum(e.^2));
            sg = sign(e(abs(e) > 0));  runs = 1 + sum(diff(sg) ~= 0);
            fprintf('%-10s %-11s %8d %+10.4f %10.4f %8.1f %8.4f %8d\n', NM{p}, S.SEG{g,1}, ...
                    numel(e), se, sw, abs(se)/max(sw, eps), max(abs(S.db(m))), runs);
        end
        P.(NM{p}) = S;
    end
    S = P.canonical;
    fprintf('\ncanonical: a_bar at trough %.4f ; law-alone integrated error over the descent = %+.4f a_o (= %+.1f %% of trough)\n', ...
            min(S.ab), sum(S.e(S.SEG{1,2}(S.t))), 100*sum(S.e(S.SEG{1,2}(S.t)))/min(S.ab));

    % ---- (4) candidate widths -------------------------------------------
    wlo = min(P.canonical.h_d); whi = max(P.canonical.h_d);
    wenv = linspace(wlo, whi, 20001).';  [~, benv] = local_b_true(wenv);
    b_half = max(abs(benv - B_ANCHOR));
    fprintf('\nDelta_b candidates:\n');
    fprintf('  envelope sup |b_true - 8/9| on [%.2f, %.2f]  = %.4f   (driver''s b_half, same provenance as sqrtP55[0])\n', wlo, whi, b_half);
    fprintf('  anchor gap |1 - 8/9|                          = %.4f   (fully c-free)\n', abs(1 - B_ANCHOR));
    fprintf('  actual sup |delta_b| on the canonical path     = %.4f\n', max(abs(P.canonical.db)));
    fprintf('  actual sup |delta_b| on the Meng path          = %.4f\n', max(abs(P.meng.db)));

    out = struct('w', w, 'ab', ab, 'bt', bt, 'db', db, 'P', P, 'b_half', b_half);
    local_fig(out, fullfile(od, 'delta_b_curve.png'));
    fprintf('\nfigure -> %s\n', od);
end

% =======================================================================
function [ab, bt] = local_b_true(w)
    ab = zeros(size(w)); bt = zeros(size(w));
    for i = 1:numel(w)
        [~, cp, d] = calc_correction_functions(w(i), true);
        ab(i) = 1 / cp;
        ap    = -d.dc_perp_dh / cp^2;
        bt(i) = ap / (1 - ab(i))^2;
    end
end

function S = local_path_from_run(fpath)
    L = load(fpath);
    fn = fieldnames(L);
    if numel(fn) == 1 && isstruct(L.(fn{1})) && isfield(L.(fn{1}), 'runs'); L = L.(fn{1}); end
    if isfield(L, 'runs')                         % driver save (out struct)
        r = L.runs{1};  t = r.tout(:);  h_d = r.h_bar_d_out(:);
        SEG = {'descend', @(t) t > 0.5 & t < 1.5; 'osc down 1', @(t) t > 2.0 & t < 2.5; ...
               'osc up 1', @(t) t > 2.5 & t < 3.0; 'osc down 2', @(t) t > 3.0 & t < 3.5; ...
               'hold end', @(t) t > 3.7};
    else                                          % inject_response stack
        t = L.t(:);  h_d = L.h_d(:);
        SEG = {'ramp', @(t) t > 0.5 & t < 10.5; 'hold end', @(t) t > 11.5};
    end
    kk = 2:numel(t);
    S = struct('t', t(kk), 'h_d', h_d(kk), 'SEG', {SEG});
end

% =======================================================================
function local_fig(o, fpath)
    FS = 17; AXLW = 2.0; RED = [0.85 0.1 0.1]; BLU = [0 0.2 0.9]; GRN = [0.1 0.6 0.2]; GRY = [0.5 0.5 0.5];
    f = figure('Position',[40 40 1400 1000],'Color','w','Visible','off');
    tl = tiledlayout(f, 2, 2, 'TileSpacing','compact','Padding','compact');

    a1 = nexttile(tl,1); hold(a1,'on'); box(a1,'on');
    plot(a1, o.ab, o.bt, '-', 'Color', RED, 'LineWidth', 2.4, 'DisplayName', 'b_{true}(a)');
    yline(a1, 8/9, '--', 'Color', BLU, 'LineWidth', 2.0, 'DisplayName', 'anchor 8/9 (far-field)');
    yline(a1, 1, ':', 'Color', GRN, 'LineWidth', 2.0, 'DisplayName', 'near-wall limit 1');
    xlabel(a1, 'a / a_o'); ylabel(a1, 'b_{true}');
    legend(a1, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', 11);
    set(a1, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);

    a2 = nexttile(tl,2); hold(a2,'on'); box(a2,'on');
    plot(a2, o.ab, o.db, '-', 'Color', RED, 'LineWidth', 2.4, 'DisplayName', '\deltab = b_{true} - 8/9');
    yline(a2, 0, '-', 'Color', GRY, 'LineWidth', 1.5, 'HandleVisibility', 'off');
    yline(a2,  o.b_half, '--', 'Color', BLU, 'LineWidth', 1.8, 'DisplayName', '\pm b_{half} (envelope sup)');
    yline(a2, -o.b_half, '--', 'Color', BLU, 'LineWidth', 1.8, 'HandleVisibility', 'off');
    xlabel(a2, 'a / a_o'); ylabel(a2, '\deltab');
    legend(a2, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', 11);
    set(a2, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);

    NM = {'canonical', 'meng'};
    for p = 1:2
        S = o.P.(NM{p});
        a = nexttile(tl, 2 + p); hold(a,'on'); box(a,'on');
        yline(a, 0, '-', 'Color', GRY, 'LineWidth', 1.5, 'HandleVisibility', 'off');
        yyaxis(a, 'left');
        plot(a, S.t, S.db, '-', 'Color', RED, 'LineWidth', 2.0, 'DisplayName', '\deltab(t)');
        ylabel(a, '\deltab'); set(a, 'YColor', RED);
        yyaxis(a, 'right');
        plot(a, S.t, cumsum(S.e), '-', 'Color', BLU, 'LineWidth', 2.2, 'DisplayName', '\Sigma e_{law}  [a_o]');
        ylabel(a, 'cumulative forcing on a'); set(a, 'YColor', BLU);
        xlabel(a, sprintf('t [s]   (%s path)', NM{p}));
        legend(a, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', 11);
        set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);
    end
    exportgraphics(f, fpath, 'Resolution', 150); close(f);
end
