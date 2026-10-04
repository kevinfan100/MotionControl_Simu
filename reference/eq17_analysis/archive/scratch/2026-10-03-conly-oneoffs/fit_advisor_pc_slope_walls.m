% PURPOSE (2026-09-22, user: "same scheme, but fit p and c together on a' and check how constant they are, per wall"):
%   the advisor slope form a' = (1 - c)/h^p with BOTH constants free, least squares on the SLOPE, on plane / cell / sphere:
%     (1) band fit [1.10, 22.2]                       -> (p, c) per wall, sup / pointwise e_a' on the band
%     (2) segment fits near [1.1, 2] / mid [2, 5.5] / far [5.5, 22.2]
%     (3) sliding windows [h, 2h] (one octave), h from 1.10 to 11.1 -> p(h), c(h): the constants each height range asks for
%   Constancy of a constant = how flat p(h) and c(h) are. For each p the best c is linear (closed form), so the joint fit is a
%   one-dimensional search over p.  Figure: columns walls; rows a' true vs band fit, e_a', p(h) with the band value, c(h) with
%   the band value (segment points, not sliding windows: on one octave p and c are near-collinear, the window curves showed
%   identifiability wiggles, user 09-22; a', e_a' zoomed to h = 1-5).  Output reference/eq17_analysis/derivation/figures/advisor_pc_slope_walls.png | EXPIRES: with the version-2 line
function fit_advisor_pc_slope_walls()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    addpath(here);
    D_CELL = 1.05;  H_LO = 1.10;  H_HI = 22.2;
    Dp  = @(u) -9/8 + 3/2*u.^2 - 228/100*u.^3 + u.^4 + 77/200*u.^10 - 12/25*u.^11;
    ap_pl = @(h) -Dp(1 ./ h) ./ h.^2;
    ap_ce = @(h) ap_pl(h + D_CELL);
    a_sp  = @(h) 1 ./ sphere_cperp_fig11(h);  ap_sp = @(h) (a_sp(max(h, 1 + 2e-3) + 1e-4) - a_sp(max(h, 1 + 2e-3) - 1e-4)) / 2e-4;
    lawp  = @(h, p, c) (1 - c) ./ h.^p;
    cfit  = @(p, apf, hh) 1 - ((1 ./ hh.^p) \ apf(hh));                        % best c at fixed p (linear LS on a')
    pcfit = @(apf, hh) local_pc(apf, hh, lawp, cfit);
    W = {'plane', ap_pl; 'cell', ap_ce; 'sphere', ap_sp};
    SEG = {'near [1.10, 2.0]', 1.10, 2.0; 'mid [2.0, 5.5]', 2.0, 5.5; 'far [5.5, 22.2]', 5.5, 22.2};
    hB = linspace(H_LO, H_HI, 4000)';
    hw0 = logspace(log10(1.10), log10(H_HI / 2), 40)';                          % window lower edges; window = [h, 2h]
    fprintf('\nadvisor slope form, BOTH (p, c) fitted on a'' (least squares on the slope)\n');
    RES = struct('name', {}, 'p', {}, 'c', {}, 'pseg', {}, 'cseg', {}, 'pw', {}, 'cw', {});
    for w = 1:3
        apf = W{w, 2};
        [p, c] = pcfit(apf, hB);  e = apf(hB) - lawp(hB, p, c);
        fprintf('%s\n  band [1.10, 22.2]: p = %.3f  c = %+.3f | e_a'' sup %.3f  at 1.1 %+.3f  2.0 %+.3f  5.5 %+.3f\n', ...
            upper(W{w, 1}), p, c, max(abs(e)), apf(1.1) - lawp(1.1, p, c), apf(2) - lawp(2, p, c), apf(5.5) - lawp(5.5, p, c));
        pseg = zeros(1, 3);  cseg = zeros(1, 3);
        for s = 1:3
            hS = linspace(SEG{s, 2}, SEG{s, 3}, 2000)';  [pseg(s), cseg(s)] = pcfit(apf, hS);
            fprintf('  %-18s p = %.3f  c = %+.3f\n', SEG{s, 1}, pseg(s), cseg(s));
        end
        pw = zeros(size(hw0));  cw = zeros(size(hw0));
        for i = 1:numel(hw0)
            hWn = linspace(hw0(i), 2 * hw0(i), 600)';  [pw(i), cw(i)] = pcfit(apf, hWn);
        end
        fprintf('  windows [h, 2h]: p from %.3f to %.3f (spread %.3f) | c from %+.3f to %+.3f (spread %.3f)\n', ...
            min(pw), max(pw), max(pw) - min(pw), min(cw), max(cw), max(cw) - min(cw));
        RES(w) = struct('name', W{w, 1}, 'p', p, 'c', c, 'pseg', pseg, 'cseg', cseg, 'pw', pw, 'cw', cw);
    end
    % --- figure
    COL_TRUE = [0.8 0 0]; COL_HAT = [0 0.2 0.9]; FS = 18; LFS = 12; AXLW = 2.0; CG = [0.5 0.5 0.5];
    hx = linspace(1, 5, 2400)';  segc = [sqrt(1.10 * 2.0), sqrt(2.0 * 5.5), sqrt(5.5 * 22.2)];   % zoom h = 1-5 for a', e_a'; segment centres (geometric) for p, c
    f = figure('Position', [60 60 1800 1500], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(4, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    for w = 1:3
        r = RES(w);  apf = W{w, 2};
        nexttile(w); hold on; box on;
        h1 = plot(hx, apf(hx), '-', 'Color', COL_TRUE, 'LineWidth', 2.2, 'DisplayName', 'a''_{true}  (plant)');
        h2 = plot(hx, lawp(hx, r.p, r.c), '-', 'Color', COL_HAT, 'LineWidth', 1.6, 'DisplayName', sprintf('(1-c)/h^p,  p %.3f  c %+.3f  (band fit)', r.p, r.c));
        h0 = plot(NaN, NaN, 'w.', 'DisplayName', upper(r.name));
        local_band(H_LO, H_HI, CG);  xlim([1 5]); ylim([0 1.05]);
        if w == 1; ylabel('a''  [1/R]', 'FontSize', FS, 'FontWeight', 'bold'); end
        legend([h0 h1 h2], 'Location', 'northoutside', 'Orientation', 'vertical', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        local_ax(FS, AXLW);
        nexttile(3 + w); hold on; box on;
        h3 = plot(hx, apf(hx) - lawp(hx, r.p, r.c), '-', 'Color', COL_HAT, 'LineWidth', 1.6, 'DisplayName', 'e_{a''} = a''_{true} - (1-c)/h^p');
        yline(0, '-', 'Color', [0.4 0.4 0.4], 'HandleVisibility', 'off');  local_band(H_LO, H_HI, CG);  xlim([1 5]); ylim([-0.15 0.15]);
        if w == 1; ylabel('e_{a''}  [1/R]', 'FontSize', FS, 'FontWeight', 'bold'); end
        legend(h3, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        local_ax(FS, AXLW);
        nexttile(6 + w); hold on; box on;
        h4 = plot(segc, r.pseg, 'o-', 'Color', COL_TRUE, 'MarkerFaceColor', COL_TRUE, 'MarkerSize', 9, 'LineWidth', 2.0, 'DisplayName', 'p per segment (near / mid / far)');
        h5 = yline(r.p, '-', 'Color', COL_HAT, 'LineWidth', 1.6, 'DisplayName', sprintf('p band fit %.3f', r.p));
        set(gca, 'XScale', 'log');  xlim([1 H_HI]); ylim([1.0 3.0]);  local_band(H_LO, H_HI, CG);
        if w == 1; ylabel('p', 'FontSize', FS, 'FontWeight', 'bold'); end
        legend([h4 h5], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        local_ax(FS, AXLW);
        nexttile(9 + w); hold on; box on;
        h6 = plot(segc, r.cseg, 'o-', 'Color', COL_TRUE, 'MarkerFaceColor', COL_TRUE, 'MarkerSize', 9, 'LineWidth', 2.0, 'DisplayName', 'c per segment (near / mid / far)');
        h7 = yline(r.c, '-', 'Color', COL_HAT, 'LineWidth', 1.6, 'DisplayName', sprintf('c band fit %+.3f', r.c));
        set(gca, 'XScale', 'log');  xlim([1 H_HI]); ylim([-1.0 1.0]);  local_band(H_LO, H_HI, CG);
        if w == 1; ylabel('c', 'FontSize', FS, 'FontWeight', 'bold'); end
        if w == 2; xlabel('h = w - w_s  (h / R),  segment centre (log axis)', 'FontSize', FS, 'FontWeight', 'bold'); end
        legend([h6 h7], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        local_ax(FS, AXLW);
    end
    fn = fullfile(od, 'advisor_pc_slope_walls.png');
    exportgraphics(f, fn, 'Resolution', 150); close(f); fprintf('figure: %s\n', fn);
end

function [p, c] = local_pc(apf, hh, lawp, cfit)
    cost = @(p) mean((lawp(hh, p, cfit(p, apf, hh)) - apf(hh)).^2);
    p = fminbnd(cost, 1.2, 4);  c = cfit(p, apf, hh);
end

function local_band(lo, hi, c)
    xline(lo, '--', 'Color', c, 'LineWidth', 1.0, 'HandleVisibility', 'off');
    xline(hi, '--', 'Color', c, 'LineWidth', 1.0, 'HandleVisibility', 'off');
end

function local_ax(FS, AXLW)
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
end
