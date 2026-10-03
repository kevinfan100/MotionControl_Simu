% PURPOSE (2026-09-16, user: "fit each measured curve with a high-order function like the plane's a(w), then with the
%   plane function as the base judge how constant the gap scale s is; no extrapolation; w_s is KNOWN in our scenario,
%   do not solve it"): two steps, everything inside the data window.
%   Step 1  each Fig. 11 curve (plane / sphere / cell, digitised CSV of 11072733-fig-11-source-large.gif) gets its own
%           smooth function of the plane's type, far field pinned to 1, linear least squares:
%               a_fit(w) = 1 + c_1 / w + c_2 / w^2 + ... + c_n / w^n          (w = Fig. 11's h_d/R, origin = visible surface)
%           n = 12 / 6 / 4 -- the answer must not move with n (parameter invariance).
%   Step 2  the base is the plane polynomial with the GAP scaled,  a(w; w_s, s) = D_perp(1 / (1 + s (w - w_s))).
%     PRIMARY (respects the premise "w_s known"): w_s is set by physics, not solved -- rigid plane and rigid sphere touch
%           at the visible surface, w_s = 1 (both curves start at 1.03 R with a = 0.05). One unknown, value only:
%               s_eff(w) = (1 / D_perp^{-1}(a_fit(w)) - 1) / (w - w_s)
%           The cell's contact height is not in the data (a >= 0.5 everywhere), so it is shown for a few candidate w_s
%           and the one that makes s_eff flat is the reading.
%     CROSS-CHECK (no w_s assumed): value + slope give two local numbers for the two unknowns at every height,
%               u = D_perp^{-1}(a_fit),  s_eff = a_fit' / (-D_perp'(u) u^2),  ws_eff = w - (1/u - 1) / s_eff
%           ws_eff flat says the family has the right number of degrees of freedom for that surface. Ill-conditioned
%           where the slope -> 0 (far end), so read it in the band only.
%   Health checks: plane -> s_eff = 1 in both readings, ws_eff = 1 in the cross-check.
%   Output reference/eq17_analysis/derivation/figures/fig11_s_from_fit.png | EXPIRES: with the s line | 產線改動不會自動跟上
function out = probe_fig11_s_from_fit()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    T  = readtable(fullfile(root, 'reference', 'eq17_analysis', 'data', 'meng_11072733_fig11_three_surfaces.csv'));
    od = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');

    % --- the plane polynomial, its derivative and inverse ------------------------------
    D  = @(u) 1 - 9/8*u + 1/2*u.^3 - 57/100*u.^4 + 1/5*u.^5 + 7/200*u.^11 - 1/25*u.^12;
    Dp = @(u) -9/8 + 3/2*u.^2 - 228/100*u.^3 + u.^4 + 77/200*u.^10 - 12/25*u.^11;
    ug = linspace(1e-4, 1, 40000);  ag = D(ug);
    assert(all(diff(ag) < 0), 'D_perp must be monotone in u for the inversion');
    Dinv = @(a) interp1(ag, ug, min(max(a, ag(end)), ag(1)));

    S = {'plane', 'sphere', 'cell'};  NN = [12, 6, 4];  W_LO = 1.1;  W_HI = 5.5;
    WS_KNOWN = {1.0, 1.0, [1.0, -0.056]};  % contact height x_c in x = w - w_s (w_s = visible surface): rigid contact 1.0;
                                           % the cell's second candidate -0.056 = the [1/1] band fit's gain zero (-c0), i.e.
                                           % the effective wall one radius below the visible surface (0916 tex S6, C61)
    % the cell gets a second line at its known contact height (-0.05, the 09-07 Form B reading) because its a = 0
    % point is ~1 R below the visible top -- with w_s = 1 the cell reading is a wrong-position signature, not shape.
    B_LO = 1.5;  B_HI = 3.5;                                % well-conditioned band for the statistics
    out = struct();
    fprintf('=== PRIMARY: w_s given, s_eff from the value only (n = 12 fit), band [%.1f, %.1f] ===\n', B_LO, B_HI);
    fprintf('%-7s w_s    | s_eff median [min, max] in band | at w = 1.5 / 2 / 3 / 3.5\n', 'surface');
    for i = 1:3
        m = strcmp(T.surface, S{i});  x = T.hd_R(m);  y = T.lambda_norm(m);
        k = x >= W_LO & x <= W_HI;  x = x(k);  y = y(k);
        wg = linspace(max(W_LO, min(x)), W_HI, 600)';  inb = wg >= B_LO & wg <= B_HI;
        o = struct('x', x, 'y', y, 'w', wg);
        for jn = 1:numel(NN)
            n = NN(jn);
            A = (1 ./ x) .^ (1:n);  c = A \ (y - 1);                       % step 1
            afit  = @(w) 1 + ((1 ./ w) .^ (1:n)) * c;
            afitp = @(w) ((1 ./ w) .^ ((1:n) + 1)) * (-(1:n)' .* c);
            a = afit(wg);  ap = afitp(wg);  u = Dinv(a);  G = 1 ./ u - 1;   % G = s (w - w_s), the scaled gap
            se2 = ap ./ (-Dp(u) .* u.^2);  we2 = wg - G ./ se2;            % cross-check: value + slope
            se1 = G ./ (wg - WS_KNOWN{i});                                  % primary: w_s given (matrix if several)
            o.(sprintf('n%d', n)) = struct('c', c, 'rms', rms(y - afit(x)), 'a_fit', a, 's1', se1, 's2', se2, 'ws2', we2);
        end
        s1 = o.n12.s1;
        for jw = 1:numel(WS_KNOWN{i})
            sb = s1(inb, jw);  q = @(w0) interp1(wg, s1(:, jw), w0);
            fprintf('%-7s %+.2f  | %.3f  [%.3f, %.3f]              | %.3f / %.3f / %.3f / %.3f\n', ...
                    S{i}, WS_KNOWN{i}(jw), median(sb), min(sb), max(sb), q(1.5), q(2), q(3), q(3.5));
        end
        out.(S{i}) = o;
    end
    fprintf('\n=== CROSS-CHECK: value + slope, nothing assumed, band [%.1f, %.1f] ===\n', B_LO, B_HI);
    fprintf('%-7s n  | fit rms | s_eff median [min, max]     | ws_eff median [min, max]\n', 'surface');
    for i = 1:3
        o = out.(S{i});  inb = o.w >= B_LO & o.w <= B_HI;
        for jn = 1:numel(NN)
            r = o.(sprintf('n%d', NN(jn)));
            fprintf('%-7s %2d | %.4f  | %.3f  [%.3f, %.3f]        | %+.3f  [%+.3f, %+.3f]\n', S{i}, NN(jn), r.rms, ...
                    median(r.s2(inb)), min(r.s2(inb)), max(r.s2(inb)), median(r.ws2(inb)), min(r.ws2(inb)), max(r.ws2(inb)));
        end
    end

    % --- figure: rows a_bar / s_eff (w_s given) / s_eff (value+slope), columns plane / sphere / cell -----------
    COL_MEAS = [0.45 0.65 0.95];  COL_HAT = [0 0.2 0.9];  COL_ALT = [0.35 0.35 0.35];
    CELL_C = [0 0.2 0.9; 0.85 0 0; 0.1 0.6 0.1; 0.6 0.3 0.8];
    FS = 14;  LFS = 11;  AXLW = 1.8;
    YL_S1 = {[0.9 1.1], [1.0 1.6], [0.5 4.0]};
    f = figure('Position', [40 40 1500 700], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    for i = 1:3
        o = out.(S{i});
        nexttile(i); hold on; box on;
        hd = plot(o.x, o.y, '.', 'Color', COL_MEAS, 'MarkerSize', 6, 'DisplayName', sprintf('%s, Fig. 11', S{i}));
        hf = plot(o.w, o.n12.a_fit, '-', 'Color', COL_HAT, 'LineWidth', 2.0, 'DisplayName', '$\bar{a}_{\mathrm{fit}}(\bar{w}),\ n = 12$');
        xlim([W_LO W_HI]); ylim([0 1]);
        if i == 1; ylabel('$\bar{a}$', 'Interpreter', 'latex', 'FontSize', FS + 6); end
        legend([hd hf], 'Interpreter', 'latex', 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'Box', 'on');
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW); grid off;

        nexttile(3 + i); hold on; box on;   % primary: w_s given
        hh = gobjects(1, numel(WS_KNOWN{i}));
        for jw = 1:numel(WS_KNOWN{i})
            hh(jw) = plot(o.w, o.n12.s1(:, jw), '-', 'Color', CELL_C(jw, :), 'LineWidth', 2.0, ...
                          'DisplayName', sprintf('contact at $\\bar{w} - \\bar{w}_s = %+.2f$', WS_KNOWN{i}(jw)));
        end
        yline(1, ':', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.0, 'HandleVisibility', 'off');
        xline(B_LO, '-.', 'Color', [0.6 0.6 0.6], 'HandleVisibility', 'off');  xline(B_HI, '-.', 'Color', [0.6 0.6 0.6], 'HandleVisibility', 'off');
        xlim([W_LO W_HI]);  ylim(YL_S1{i});
        if i == 1; ylabel('$s_{\mathrm{eff}}$', 'Interpreter', 'latex', 'FontSize', FS + 6); end
        xlabel('$\bar{w} - \bar{w}_s$', 'Interpreter', 'latex', 'FontSize', FS + 6);
        legend(hh, 'Interpreter', 'latex', 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'Box', 'on');
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW); grid off;
    end
    fn = fullfile(od, 'fig11_s_from_fit.png');  exportgraphics(f, fn, 'Resolution', 150);  close(f);
    fprintf('figure: %s\n', fn);
end
