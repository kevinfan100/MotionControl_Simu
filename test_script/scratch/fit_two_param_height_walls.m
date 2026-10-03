% PURPOSE (2026-09-22, user: "the law must be a'(h; params), no dependence on a; find a TWO-parameter form whose parameters are
%   constant on cell AND sphere"): candidate two-parameter slope forms in the HEIGHT coordinate only, least squares on a' per wall
%   (plane / cell / sphere), band [1.10, 22.2] and three segments near [1.1, 2] / mid [2, 5.5] / far [5.5, 22.2].
%   Constancy of a parameter = spread of its segment values.  Forms (theta = two numbers, h = w - w_s):
%     F1  (1 - c)/h^p                 advisor, both free                (reference, 09-22 morning)
%     F2  (1 - c)/(h + delta)^2       shift + level  = [1/1] slope kappa/(h + d1)^2 with kappa = 1 - c, d1 = delta
%     F3  (1 - c)/(h^2 + q)           level + quadratic offset
%     F4  1/(h + delta)^p             shift + exponent, level pinned at 1
%     F5  (1 - c)/(h (h + delta))     level + one shifted factor
%   Each linear level is closed form at fixed nonlinear parameter; the nonlinear one is a 1-D (F4: 2-D) search.
%   Figure for the form the user picks (default F2): columns walls, rows a' true vs band fit (h 1-5), e_a', theta_1 and theta_2 per
%   segment on a log axis with the band value.  Output reference/eq17_analysis/derivation/figures/slope_two_param_height_<form>.png
%   | EXPIRES: with the two-parameter height-law decision
function fit_two_param_height_walls(form_to_plot)
    if nargin < 1; form_to_plot = 'F2'; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    addpath(here);
    D_CELL = 1.05;  H_LO = 1.10;  H_HI = 22.2;
    Dp  = @(u) -9/8 + 3/2*u.^2 - 228/100*u.^3 + u.^4 + 77/200*u.^10 - 12/25*u.^11;
    ap_pl = @(h) -Dp(1 ./ h) ./ h.^2;
    ap_ce = @(h) ap_pl(h + D_CELL);
    a_sp  = @(h) 1 ./ sphere_cperp_fig11(h);  ap_sp = @(h) (a_sp(max(h, 1 + 2e-3) + 1e-4) - a_sp(max(h, 1 + 2e-3) - 1e-4)) / 2e-4;
    W = {'plane', ap_pl; 'cell', ap_ce; 'sphere', ap_sp};
    SEG = [1.10 2.0; 2.0 5.5; 5.5 22.2];  SEGN = {'near', 'mid', 'far'};
    % --- forms: law(h, t1, t2); fit(apf, hh) -> [t1 t2]; names of the two parameters
    F = struct('id', {}, 'law', {}, 'fit', {}, 'n1', {}, 'n2', {}, 'lab', {});
    F(end+1) = local_form('F1', @(h, p, c) (1 - c) ./ h.^p, ...
        @(apf, hh) local_fit_lin(apf, hh, @(h, p) 1 ./ h.^p, [1.2 4]), 'p', 'c', '(1-c)/h^p');
    F(end+1) = local_form('F2', @(h, dl, c) (1 - c) ./ (h + dl).^2, ...
        @(apf, hh) local_fit_lin(apf, hh, @(h, dl) 1 ./ (h + dl).^2, [-0.9 3]), '\delta', 'c', '(1-c)/(h+\delta)^2');
    F(end+1) = local_form('F3', @(h, q, c) (1 - c) ./ (h.^2 + q), ...
        @(apf, hh) local_fit_lin(apf, hh, @(h, q) 1 ./ (h.^2 + q), [-1.1 12]), 'q', 'c', '(1-c)/(h^2+q)');
    F(end+1) = local_form('F4', @(h, dl, p) 1 ./ (h + dl).^p, ...
        @(apf, hh) local_fit_2d(apf, hh, @(h, t) 1 ./ (h + t(1)).^t(2), [0 2]), '\delta', 'p', '1/(h+\delta)^p');
    F(end+1) = local_form('F5', @(h, dl, c) (1 - c) ./ (h .* (h + dl)), ...
        @(apf, hh) local_fit_lin(apf, hh, @(h, dl) 1 ./ (h .* (h + dl)), [-0.9 6]), '\delta', 'c', '(1-c)/(h(h+\delta))');
    hB = linspace(H_LO, H_HI, 4000)';
    fprintf('\ntwo-parameter slope forms in the HEIGHT coordinate, least squares on a''; band [1.10, 22.2] and segments near/mid/far\n');
    fprintf('  spread = max - min of the parameter over the three segments (constancy); sup = sup |a''_true - law| on the band\n');
    RES = struct();
    for f = 1:numel(F)
        fprintf('\n%s  %s\n', F(f).id, regexprep(F(f).lab, '\\', ''));
        fprintf('  %-7s %9s %9s %7s | %-27s | %-27s | %7s %7s\n', 'wall', F(f).n1, F(f).n2, 'sup', ...
            [regexprep(F(f).n1, '\\', '') ' near / mid / far'], [regexprep(F(f).n2, '\\', '') ' near / mid / far'], 'spread1', 'spread2');
        for w = 1:3
            apf = W{w, 2};  th = F(f).fit(apf, hB);  e = apf(hB) - F(f).law(hB, th(1), th(2));
            ts = zeros(3, 2);
            for s = 1:3; hS = linspace(SEG(s, 1), SEG(s, 2), 2000)';  ts(s, :) = F(f).fit(apf, hS); end
            fprintf('  %-7s %+9.3f %+9.3f %7.3f | %+8.3f %+8.3f %+8.3f | %+8.3f %+8.3f %+8.3f | %7.3f %7.3f\n', ...
                W{w, 1}, th(1), th(2), max(abs(e)), ts(:, 1), ts(:, 2), max(ts(:, 1)) - min(ts(:, 1)), max(ts(:, 2)) - min(ts(:, 2)));
            RES.(F(f).id).(W{w, 1}) = struct('th', th, 'ts', ts, 'sup', max(abs(e)));
        end
    end
    % --- figure for one form
    fi = find(strcmp({F.id}, form_to_plot), 1);  Ff = F(fi);
    COL_TRUE = [0.8 0 0]; COL_HAT = [0 0.2 0.9]; FS = 18; LFS = 12; AXLW = 2.0; CG = [0.5 0.5 0.5];
    hx = linspace(1, 5, 2400)';  segc = sqrt(SEG(:, 1) .* SEG(:, 2))';
    YL1 = [-1 3];  YL2 = [-1 1];
    if strcmp(Ff.id, 'F1'); YL1 = [1 3]; end
    if strcmp(Ff.id, 'F3'); YL1 = [-2 12]; end
    if strcmp(Ff.id, 'F4'); YL2 = [1 3]; end
    fig = figure('Position', [60 60 1800 1500], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(4, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    for w = 1:3
        r = RES.(Ff.id).(W{w, 1});  apf = W{w, 2};  th = r.th;
        nexttile(w); hold on; box on;
        h1 = plot(hx, apf(hx), '-', 'Color', COL_TRUE, 'LineWidth', 2.2, 'DisplayName', 'a''_{true}  (plant)');
        h2 = plot(hx, Ff.law(hx, th(1), th(2)), '-', 'Color', COL_HAT, 'LineWidth', 1.6, ...
            'DisplayName', sprintf('%s,  %s %+.3f  %s %+.3f  (band fit)', Ff.lab, Ff.n1, th(1), Ff.n2, th(2)));
        h0 = plot(NaN, NaN, 'w.', 'DisplayName', upper(W{w, 1}));
        local_band(H_LO, H_HI, CG);  xlim([1 5]); ylim([0 1.05]);
        if w == 1; ylabel('a''  [1/R]', 'FontSize', FS, 'FontWeight', 'bold'); end
        legend([h0 h1 h2], 'Location', 'northoutside', 'Orientation', 'vertical', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        local_ax(FS, AXLW);
        nexttile(3 + w); hold on; box on;
        h3 = plot(hx, apf(hx) - Ff.law(hx, th(1), th(2)), '-', 'Color', COL_HAT, 'LineWidth', 1.6, 'DisplayName', ['e_{a''} = a''_{true} - ' Ff.lab]);
        yline(0, '-', 'Color', [0.4 0.4 0.4], 'HandleVisibility', 'off');  local_band(H_LO, H_HI, CG);  xlim([1 5]); ylim([-0.15 0.15]);
        if w == 1; ylabel('e_{a''}  [1/R]', 'FontSize', FS, 'FontWeight', 'bold'); end
        legend(h3, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        local_ax(FS, AXLW);
        for j = 1:2
            nexttile(3 * (1 + j) + w); hold on; box on;
            nm = Ff.(['n' num2str(j)]);
            h4 = plot(segc, r.ts(:, j), 'o-', 'Color', COL_TRUE, 'MarkerFaceColor', COL_TRUE, 'MarkerSize', 9, 'LineWidth', 2.0, ...
                'DisplayName', [nm ' per segment (near / mid / far)']);
            h5 = yline(th(j), '-', 'Color', COL_HAT, 'LineWidth', 1.6, 'DisplayName', sprintf('%s band fit %+.3f', nm, th(j)));
            set(gca, 'XScale', 'log');  xlim([1 H_HI]);  if j == 1; ylim(YL1); else; ylim(YL2); end
            local_band(H_LO, H_HI, CG);
            if w == 1; ylabel(nm, 'FontSize', FS, 'FontWeight', 'bold'); end
            if j == 2 && w == 2; xlabel('h = w - w_s  (h / R),  segment centre (log axis)', 'FontSize', FS, 'FontWeight', 'bold'); end
            legend([h4 h5], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
            local_ax(FS, AXLW);
        end
    end
    fn = fullfile(od, sprintf('slope_two_param_height_%s.png', Ff.id));
    exportgraphics(fig, fn, 'Resolution', 150); close(fig); fprintf('\nfigure: %s\n', fn);
end

function s = local_form(id, law, fit, n1, n2, lab)
    s = struct('id', id, 'law', law, 'fit', fit, 'n1', n1, 'n2', n2, 'lab', lab);
end

function th = local_fit_lin(apf, hh, basis, rng)
    % law = (1 - c) * basis(h, t): for fixed t the best (1 - c) is linear LS; search t on rng
    y = apf(hh);
    kfit = @(t) basis(hh, t) \ y;
    cost = @(t) mean((kfit(t) * basis(hh, t) - y).^2);
    t = fminbnd(cost, rng(1), rng(2));  th = [t, 1 - kfit(t)];
end

function th = local_fit_2d(apf, hh, law2, t0)
    y = apf(hh);
    cost = @(t) mean((law2(hh, t) - y).^2) + 1e6 * double(min(hh) + t(1) <= 0);
    th = fminsearch(cost, t0, optimset('TolX', 1e-6, 'TolFun', 1e-12, 'MaxFunEvals', 4000, 'Display', 'off'));
end

function local_band(lo, hi, c)
    xline(lo, '--', 'Color', c, 'LineWidth', 1.0, 'HandleVisibility', 'off');
    xline(hi, '--', 'Color', c, 'LineWidth', 1.0, 'HandleVisibility', 'off');
end

function local_ax(FS, AXLW)
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
end
