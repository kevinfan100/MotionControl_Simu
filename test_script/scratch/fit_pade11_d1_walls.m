% PURPOSE (2026-09-21, user: "[1/1] with d fixed at 1, same walls"): the [1/1] Pade law a = (h + c0)/(h + d1) with d1 FIXED at 1,
%   one constant c0, on plane / cell (cell_cperp_delta) / sphere (sphere_cperp_fig11), band [1.10, 22.2] uniform in h.
%   c0 is linear in the residual: (h + d1) a = h + c0  =>  c0 = (h + 1) a - h, closed-form LS. Reported per wall: c0_fit,
%   sup / rms e_a, e_a at 1.1 / 2.0 / 5.5 / 22.2, the level walk c0(h) = (h + 1) a_true(h) - h (what one height asks for) and
%   sup|c0(h) - c0_fit|, the slope error e_a' (a'_law = (1 - c0)/(h + 1)^2), and a_law(1) = (1 + c0)/2 vs the plant.
%   Figure (house style, linear h): columns plane | cell | sphere; rows a, e_a, a', e_a'.
%   Output reference/eq17_analysis/derivation/figures/pade11_d1_fit_walls.png + test_results/temp_plaw_walls/pade11_d1_fit.mat
%   EXPIRES: with the advisor-law / one-constant-family test
function fit_pade11_d1_walls()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    addpath(here);
    H_LO = 1.10;  H_HI = 22.2;  D_CELL = 1.05;  D1 = 1.0;
    D   = @(u) 1 - 9/8*u + 1/2*u.^3 - 57/100*u.^4 + 1/5*u.^5 + 7/200*u.^11 - 1/25*u.^12;
    Dp  = @(u) -9/8 + 3/2*u.^2 - 228/100*u.^3 + u.^4 + 77/200*u.^10 - 12/25*u.^11;
    a_pl  = @(h) D(1 ./ h);  ap_pl = @(h) -Dp(1 ./ h) ./ h.^2;
    a_ce  = @(h) a_pl(h + D_CELL);  ap_ce = @(h) ap_pl(h + D_CELL);
    a_sp  = @(h) 1 ./ sphere_cperp_fig11(h);  ap_sp = @(h) (a_sp(max(h, 1 + 2e-3) + 1e-4) - a_sp(max(h, 1 + 2e-3) - 1e-4)) / 2e-4;   % central difference outside the plant's clamps (h >= 1, a >= 1e-3)
    law   = @(h, c0) (h + c0) ./ (h + D1);
    lawp  = @(h, c0) (D1 - c0) ./ (h + D1).^2;
    hU = linspace(H_LO, H_HI, 4000)';
    W = {'plane', a_pl, ap_pl;  'cell', a_ce, ap_ce;  'sphere', a_sp, ap_sp};
    fprintf('\n[1/1] with d1 = %.1f fixed, fit c0 per wall on [%.2f, %.1f] (uniform h):\n', D1, H_LO, H_HI);
    RES = struct('name', {}, 'c0', {}, 'a', {}, 'ap', {});
    for w = 1:3
        af = W{w, 2};  apf = W{w, 3};
        c0 = mean((hU + D1) .* af(hU) - hU);                                   % LS in a: residual is linear in c0 with unit weight 1/(h+1)... use plain mean of the level c0(h)
        c0 = ((1 ./ (hU + D1)) \ (af(hU) - hU ./ (hU + D1)));                  % exact LS in a: a = h/(h+1) + c0/(h+1)
        e = af(hU) - law(hU, c0);  ep = apf(hU) - lawp(hU, c0);
        c0h = (hU + D1) .* af(hU) - hU;
        fprintf('  %-6s c0_fit %+.4f | sup|e_a| %.4f rms %.4f | e_a at 1.1 %+.4f 2.0 %+.4f 5.5 %+.4f 22.2 %+.4f | sup|e_a''| %.3f (at 1.1 %+.3f) | a_law(1) = %.3f vs plant %.3f | level c0(h): 1.1 %+.3f 2.0 %+.3f 22.2 %+.3f, sup|c0(h) - c0_fit| %.3f\n', ...
            W{w, 1}, c0, max(abs(e)), rms(e), af(1.1) - law(1.1, c0), af(2) - law(2, c0), af(5.5) - law(5.5, c0), af(22.2) - law(22.2, c0), ...
            max(abs(ep)), apf(1.1) - lawp(1.1, c0), law(1, c0), af(1.0), (1.1 + D1) * af(1.1) - 1.1, (2 + D1) * af(2) - 2, (22.2 + D1) * af(22.2) - 22.2, max(abs(c0h - c0)));
        RES(w) = struct('name', W{w, 1}, 'c0', c0, 'a', af, 'ap', apf);
    end
    rd = fullfile(root, 'test_results', 'temp_plaw_walls');  if ~exist(rd, 'dir'); mkdir(rd); end
    save(fullfile(rd, 'pade11_d1_fit.mat'), 'RES', 'D1', 'H_LO', 'H_HI');
    % --- figure
    COL_TRUE = [0.8 0 0]; COL_HAT = [0 0.2 0.9]; FS = 18; LFS = 13; AXLW = 2.0; CG = [0.5 0.5 0.5];
    hx = linspace(1, 25, 2400)';
    f = figure('Position', [60 60 1800 1500], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(4, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    for w = 1:3
        r = RES(w);  al = law(hx, r.c0);  at = r.a(hx);  apl = lawp(hx, r.c0);  apt = r.ap(hx);
        nexttile(w); hold on; box on;
        h1 = plot(hx, at, '-', 'Color', COL_TRUE, 'LineWidth', 2.0, 'DisplayName', 'a_{true} / a_{nom}  (plant)');
        h2 = plot(hx, al, '-', 'Color', COL_HAT, 'LineWidth', 1.6, 'DisplayName', sprintf('[1/1]  d_1 = 1,  c_0 %+.3f', r.c0));
        h0 = plot(NaN, NaN, 'w.', 'DisplayName', upper(r.name));
        local_band(H_LO, H_HI, CG);  xlim([1 25]); ylim([0 1]);
        if w == 1; ylabel('a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold'); end
        legend([h0 h1 h2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        local_ax(FS, AXLW);
        nexttile(3 + w); hold on; box on;
        h3 = plot(hx, at - al, '-', 'Color', COL_HAT, 'LineWidth', 1.6, 'DisplayName', 'e_a = a_{true} - a_{law}');
        yline(0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off');
        local_band(H_LO, H_HI, CG);  xlim([1 25]); ylim([-0.15 0.15]);
        if w == 1; ylabel('e_a  [a / a_{nom}]', 'FontSize', FS, 'FontWeight', 'bold'); end
        legend(h3, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        local_ax(FS, AXLW);
        nexttile(6 + w); hold on; box on;
        h4 = plot(hx, apt, '-', 'Color', COL_TRUE, 'LineWidth', 2.0, 'DisplayName', 'a''_{true} = d(a/a_{nom})/dh  (plant)');
        h5 = plot(hx, apl, '-', 'Color', COL_HAT, 'LineWidth', 1.6, 'DisplayName', 'a''_{law} = (d_1 - c_0)/(h + d_1)^2');
        local_band(H_LO, H_HI, CG);  xlim([1 25]); ylim([0 1.1]);
        if w == 1; ylabel('a''  [1/R]', 'FontSize', FS, 'FontWeight', 'bold'); end
        legend([h4 h5], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        local_ax(FS, AXLW);
        nexttile(9 + w); hold on; box on;
        h6 = plot(hx, apt - apl, '-', 'Color', COL_HAT, 'LineWidth', 1.6, 'DisplayName', 'e_{a''} = a''_{true} - a''_{law}');
        yline(0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off');
        local_band(H_LO, H_HI, CG);  xlim([1 25]); ylim([-0.5 0.5]);
        if w == 1; ylabel('e_{a''}  [1/R]', 'FontSize', FS, 'FontWeight', 'bold'); end
        xlabel('h = w - w_s  (h / R)', 'FontSize', FS, 'FontWeight', 'bold');
        legend(h6, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        local_ax(FS, AXLW);
    end
    fn = fullfile(od, 'pade11_d1_fit_walls.png');
    exportgraphics(f, fn, 'Resolution', 150); close(f); fprintf('figure: %s\n', fn);
end

function local_band(lo, hi, c)
    xline(lo, '--', 'Color', c, 'LineWidth', 1.0, 'HandleVisibility', 'off');
    xline(hi, '--', 'Color', c, 'LineWidth', 1.0, 'HandleVisibility', 'off');
end

function local_ax(FS, AXLW)
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
end
