% PURPOSE (2026-09-23 evening, user: "(c, p) on the T_d descent, T_d fixed; can we see the observability height by height?";
%   then "unify p = 2 and repeat"): exact observability of the advisor's slope law a' = (1-c)/h^p (plaw_cp_slope wiring),
%   plane init, on the ramp template hold 0.5 s - constant-velocity descent 22.2 -> 1.10 R in T_d - hold 0.5 s, walls plane and
%   cell (plant only), one seed. Laws: 'f1' = c and p both states (priors 0.10); 'f1p2' = c only, p pinned at 2 (c prior 0.6,
%   slot 7 excluded from the free set -- the analysis is conditional on p).
%   From the logged F_e / H / R (obs_dump):
%     G(k) = sum_{j<=k} Phi' H' R^-1 H Phi accumulated from the descent start (and, for the whole-run figure, from t = 0),
%     free slots FREE (slot 6 locked = negative control). G v_i = sigma_i^2 v_i.
%   "Height by height": the accumulated sigma plotted against the commanded gap h_d -- where a curve steepens is where that
%   direction's information arrives (slope = information rate at that height). The constants are read through their BLOCK with
%   the other states marginalised: I_cc = ([G^-1]_{cc})^-1 (Schur complement); for two constants sigma_+ / sigma_- = sqrt of its
%   eigenvalues (strong / weak combination), for one constant a single sigma_c. a_w likewise through its 1 x 1 marginal.
%   Figures (house style, singular values and V only):
%     obs_ramp_<law>_V_Td<Td>.png         V tables over the whole descent, plane | cell (0907/0916 table style)
%     obs_ramp_<law>_sigma_h_Td<Td>.png   columns plane | cell; rows: sigma_aw, then the constants' marginal sigma(s), accumulated
%                                         from the descent start vs h_d (log, reversed), y shared per row
%     obs_ramp_<law>_wholerun_Td<Td>.png  columns plane | cell; rows: h_d(t), the constants' marginal sigma(s) from t = 0 (holds flat)
%   Usage: probe_obs_ramp_f1(Td, [], law)  compute + plot;  probe_obs_ramp_f1(Td, 'plot', law)  replot from cache
%   Cache test_results/temp_plaw_walls/obs_ramp_<law>_Td<Td>.mat   EXPIRES: with the (c, p) / c-only trajectory line
function out = probe_obs_ramp_f1(Td, arg, law)
    if nargin < 1 || isempty(Td); Td = 2.0; end
    if nargin < 3 || isempty(law); law = 'f1'; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration')); addpath(here);
    od = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    rd = fullfile(root, 'test_results', 'temp_plaw_walls');  if ~exist(rd, 'dir'); mkdir(rd); end
    fm = fullfile(rd, sprintf('obs_ramp_%s_Td%g.mat', law, Td));
    if nargin >= 2 && ischar(arg) && strcmp(arg, 'plot'); out = load(fm); local_plot(out, od); return; end
    pc = physical_constants();  Ts = pc.Ts;  T_HOLD = 0.5;  seed = 7;  NEG = 6;  STRIDE = 4;
    [cc0, FREE] = local_law(law);  nF = numel(FREE);  ICP = find(FREE >= 5);   % indices (within FREE) of the constants
    WALLS = {'plane', []; 'cell', @cell_cperp_delta};
    out = struct('Td', Td, 'T_HOLD', T_HOLD, 'seed', seed, 'law', law, 'walls', {WALLS(:, 1)'}, 'FREE', FREE, 'ICP', ICP, 'STRIDE', STRIDE);
    fprintf('\n%-6s | %-52s | %-28s | %s\n', 'wall', 'sigma_1 .. sigma_n (whitened, whole descent)', 'weakest v on the constants', 'neg-ctrl');
    for iw = 1:size(WALLS, 1)
        wall = WALLS{iw, 1};  cc = cc0;
        OV = struct('trajectory_type', 'ramp_descent', 't_hold', T_HOLD, 't_descend_override', Td, 'T_sim', T_HOLD + Td + 0.5, ...
                    'amplitude', 0, 'frequency', 1, 'n_cycles', 1);
        o = struct('arm', 'best', 'ctrl_const_override', cc, 'config_override', OV, 'scenario', 'deep', 'verbose', false, 'seeds', seed, 'log_P_full', false);
        if ~isempty(WALLS{iw, 2}); o.plant_cperp = WALLS{iw, 2}; end
        clear run_formC_b motion_control_law_formC_b;
        evalc('Rr = run_formC_b(o);');  L = obs_dump('get');  Lz = L([L.ax] == 3);
        t = [Lz.k]' * Ts;  wd = Rr.runs{1}.h_bar_d_out(:);
        idx_d = find(t >= T_HOLD & t < T_HOLD + Td);  idx_all = (1:numel(Lz))';
        [G, negz] = local_G(Lz, idx_d, FREE, NEG);
        [Vv, Dd] = eig((G + G') / 2);  [d, ord] = sort(diag(Dd), 'descend');  Vv = Vv(:, ord);
        for j = 1:nF; [~, im] = max(abs(Vv(:, j)));  Vv(:, j) = Vv(:, j) * sign(Vv(im, j)); end
        SIG = sqrt(max(d, 0));
        [SIGd, HKd, ~, Gd] = local_accum(Lz, idx_d, FREE, wd, STRIDE, t);            % from the descent start, vs h_d
        [SIGw, HKw, TKw, Gw] = local_accum(Lz, idx_all, FREE, wd, STRIDE, t);        % from t = 0, vs time
        [MGd, WDd] = local_marginal(Gd, 4, ICP);  [MGw, WDw] = local_marginal(Gw, 4, ICP);   % rows: sigma_aw, then the constants' block sigmas
        out.(wall) = struct('SIG', SIG, 'V', Vv, 'negctrl_ok', negz, 'SIGd', SIGd, 'HKd', HKd, 'Gd', Gd, 'MGd', MGd, 'WDd', WDd, ...
                            'SIGw', SIGw, 'TKw', TKw, 'HKw', HKw, 'Gw', Gw, 'MGw', MGw, 'WDw', WDw);
        fprintf('%-6s | %s | %s | %s\n', wall, sprintf('%9.1f', SIG), sprintf('%+.2f ', Vv(ICP, nF)), local_flag(negz));
    end
    save(fm, '-struct', 'out');
    local_plot(out, od);
end

function local_plot(out, od)
    law = out.law;  FREE = out.FREE;  ICP = out.ICP;  n = numel(FREE);  nC = numel(ICP);
    LAWL = struct('f1', '(c, p)  a'' = (1-c)/h^p', 'f1p2', 'c only,  a'' = (1-c)/h^2  (p = 2 pinned)');
    WLAB = struct('plane', 'PLANE', 'cell', 'CELL');  nW = numel(out.walls);
    labs = {'\deltaw_1', '\deltaw_2', '\deltaw_3', 'a_w', 'c', '(6)', 'p'};  lab = labs(FREE);
    FS = 15; LFS = 11;
    % ===== figure 1: V tables over the whole descent
    fig = figure('Position', [60 60 700 * nW + 100, 540], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(1, nW, 'TileSpacing', 'compact', 'Padding', 'compact');
    for iw = 1:nW
        wall = out.walls{iw};  r = out.(wall);  V = r.V;  sv = r.SIG;
        nexttile; hold on; box on;
        imagesc(abs(V), [0 1]);  colormap(gca, flipud(gray));
        for i = 1:n; for j = 1:n
            tc = 'k';  if abs(V(i, j)) > 0.55; tc = 'w'; end
            text(j, i, sprintf('%.2f', V(i, j)), 'HorizontalAlignment', 'center', 'FontSize', 12, 'FontWeight', 'bold', 'Color', tc);
        end; end
        set(gca, 'YDir', 'reverse', 'XTick', 1:n, 'YTick', 1:n, 'YTickLabel', lab, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.5, 'TickLength', [0 0]);
        xt = cell(1, n);  for j = 1:n; xt{j} = sprintf('v_%d\\newline%s', j, local_num(sv(j))); end
        set(gca, 'XTickLabel', xt, 'XTickLabelRotation', 0);
        xlabel('\sigma_i', 'FontSize', FS, 'FontWeight', 'bold');  xlim([0.5 n + 0.5]);  ylim([0.5 n + 0.5]);
        h = plot(NaN, NaN, 'w.');
        legend(h, {sprintf('%s wall   %s   descent 22.2 -> 1.10 R,  T_d = %g s   plane init', WLAB.(wall), LAWL.(law), out.Td)}, ...
               'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    end
    fn = fullfile(od, sprintf('obs_ramp_%s_V_Td%g.png', law, out.Td));  exportgraphics(fig, fn, 'Resolution', 150); close(fig); fprintf('figure: %s\n', fn);
    % ===== figure 2: marginal sigma of a_w and of the constants' block vs h_d, accumulated from the descent start
    COL_LINE = [0 0.2 0.9];  COL_CMD = [0.2 0.2 0.2];  FS2 = 16; LFS2 = 12; AXLW = 2.0;
    if nC == 2; RLAB = {'\sigma_{a_w}', '\sigma_+  (c, p)', '\sigma_-  (c, p)'}; else; RLAB = {'\sigma_{a_w}', '\sigma_c'}; end
    nR = 1 + nC;
    f = figure('Position', [60 60 560 * nW + 120, 280 * nR + 60], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(nR, nW, 'TileSpacing', 'compact', 'Padding', 'compact');  ax = gobjects(nR, nW);
    for iw = 1:nW
        wall = out.walls{iw};  r = out.(wall);
        for q = 1:nR
            ax(q, iw) = nexttile((q - 1) * nW + iw); hold on; box on;
            switch q
                case 1; dn = sprintf('%s wall,  %s,  plane init,  T_d = %g s:   \\sigma_{a_w}  (other states marginalised), from the descent start', WLAB.(wall), LAWL.(law), out.Td);
                case 2
                    if nC == 2; dn = '\sigma_+   strong combination of (c, p), other states marginalised';
                    else;       dn = '\sigma_c   c with the other states marginalised (p known)'; end
                case 3; wdir = r.WDd(:, end);  dn = sprintf('\\sigma_-   weak combination of (c, p);  at the descent end  %+.2f c %+.2f p', wdir(1), wdir(2));
            end
            hh = plot(r.HKd, r.MGd(q, :), '-', 'Color', COL_LINE, 'LineWidth', 2.0, 'DisplayName', dn);
            if q == 1 || iw == 1 || q == 3
                legend(hh, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS2, 'FontWeight', 'bold', 'Box', 'on');
            end
            set(ax(q, iw), 'XScale', 'log', 'XDir', 'reverse');  xlim(ax(q, iw), [1.0 25]);
            if iw == 1; ylabel(RLAB{q}, 'FontSize', FS2, 'FontWeight', 'bold'); end
            if q == nR; xlabel('commanded gap  h_d  (R)', 'FontSize', FS2, 'FontWeight', 'bold'); end
            set(ax(q, iw), 'FontSize', FS2, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid(ax(q, iw), 'off');
        end
    end
    for q = 1:nR; linkaxes(ax(q, :), 'y'); end
    fn = fullfile(od, sprintf('obs_ramp_%s_sigma_h_Td%g.png', law, out.Td));  exportgraphics(f, fn, 'Resolution', 150); close(f); fprintf('figure: %s\n', fn);
    % ===== figure 3: whole run vs time, rows h_d / constants' block sigma(s) (hold-contrast style)
    f = figure('Position', [60 60 560 * nW + 120, 300 * nR + 60], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(nR, nW, 'TileSpacing', 'compact', 'Padding', 'compact');  ax = gobjects(nR, nW);
    for iw = 1:nW
        wall = out.walls{iw};  r = out.(wall);  t = r.TKw(:);
        ax(1, iw) = nexttile(iw); hold on; box on;
        h1 = plot(t, r.HKw(:), '-', 'Color', COL_CMD, 'LineWidth', 2.0, 'DisplayName', sprintf('h_d   %s wall,  %s,  plane init,  T_d = %g s', WLAB.(wall), LAWL.(law), out.Td));
        legend(h1, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS2, 'FontWeight', 'bold', 'Box', 'on');
        if iw == 1; ylabel('h_d  (R)', 'FontSize', FS2, 'FontWeight', 'bold'); end
        for q = 1:nC
            ax(1 + q, iw) = nexttile(q * nW + iw); hold on; box on;
            hh = plot(t, r.MGw(1 + q, :), '-', 'Color', COL_LINE, 'LineWidth', 2.0, 'DisplayName', sprintf('%s   accumulated from t = 0, other states marginalised', RLAB{1 + q}));
            if iw == 1
                legend(hh, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS2, 'FontWeight', 'bold', 'Box', 'on');
                ylabel(RLAB{1 + q}, 'FontSize', FS2, 'FontWeight', 'bold');
            end
        end
        xlabel('Time (sec)', 'FontSize', FS2, 'FontWeight', 'bold');
        for q = 1:nR; set(ax(q, iw), 'FontSize', FS2, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid(ax(q, iw), 'off'); xlim(ax(q, iw), [0 t(end)]); end
    end
    for q = 1:nR; linkaxes(ax(q, :), 'y'); end
    for iw = 1:nW
        seg = [out.T_HOLD, out.T_HOLD + out.Td];
        for q = 1:nR
            yl = ylim(ax(q, iw));
            for x = seg; plot(ax(q, iw), [x x], yl, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, 'HandleVisibility', 'off'); end
            ylim(ax(q, iw), yl);
        end
    end
    fn = fullfile(od, sprintf('obs_ramp_%s_wholerun_Td%g.png', law, out.Td));  exportgraphics(f, fn, 'Resolution', 150); close(f); fprintf('figure: %s\n', fn);
end

function [cc, FREE] = local_law(law)
    cc = struct('ma2_aug', false, 'nw_mcorr', false, 'pred_mean2', false, 'pred_mean2_e4', false, 'law_exact_step', false, ...
                'lock_b', false, 'gain_algebraic', false, 'amodel_fe', false, 'amodel_h', false, 'b_ceil', 1.5, 'b_floor', 0.2, ...
                'ws0_perp', 0, 'Pf_w0_std', 0, 'Pf_a_floor', 0.002, 'obs_dump', true, 'plaw_cp_slope', true, 'plaw_p_lim', [1.2, 4]);
    switch law
        case 'f1';   cc.c_init = 0.096;  cc.Pf_c_std = 0.10;  cc.p_init = 1.894;  cc.Pf_p_std = 0.10;   FREE = [1 2 3 4 5 7];
        case 'f1p2'; cc.c_init = 0.0533; cc.Pf_c_std = 0.6;   cc.p_init = 2;      cc.Pf_p_std = 1e-6;   FREE = [1 2 3 4 5];   % p pinned: conditional analysis
        otherwise; error('unknown law %s', law);
    end
end

function [SIGk, hk, tk, Gk] = local_accum(Lz, idx, FREE, wd, STRIDE, t)
% accumulated G from idx(1); every STRIDE steps: ranked sigma (SIGk), commanded gap (hk), time (tk), and the free-slot G (Gk)
    n = 7;  G = zeros(n);  Phi = eye(n);  m = numel(idx);  ks = 1:STRIDE:m;  nF = numel(FREE);
    SIGk = zeros(nF, numel(ks));  hk = zeros(1, numel(ks));  tk = hk;  Gk = zeros(nF, nF, numel(ks));  c = 0;
    for j = 1:m
        i = idx(j);
        H1 = Lz(i).H{1};  H2 = Lz(i).H{2};  R1 = Lz(i).R(1);  R2 = Lz(i).R(2);
        r1 = H1 * Phi;  G = G + (r1' * r1) / R1;
        if ~isempty(H2) && R2 > 0; r2 = H2 * Phi;  G = G + (r2' * r2) / R2; end
        Phi = Lz(i).F * Phi;
        if any(j == ks)
            c = c + 1;  Gf = (G(FREE, FREE) + G(FREE, FREE)') / 2;  Gk(:, :, c) = Gf;
            d = sort(eig(Gf), 'descend');  SIGk(:, c) = sqrt(max(d, 0));
            k = Lz(i).k + 1;  hk(c) = wd(min(max(k, 1), numel(wd)));  tk(c) = t(i);
        end
    end
end

function [MG, WD] = local_marginal(Gk, IA, ICP)
% marginal information: a_w with everything else unknown (1 x 1 Schur complement) and the constants' block with everything else
% unknown (Schur complement I_cc = G_cc - G_cn G_nn^+ G_nc). MG rows: sigma_aw, then sqrt of the block's eigenvalues (descending);
% WD = the weak combination (unit vector, first component positive) when the block is 2 x 2, else the single direction.
% pinv handles the early steps where the constants carry no information yet (their rows are zero and drop out).
    nk = size(Gk, 3);  nF = size(Gk, 1);  nC = numel(ICP);  MG = zeros(1 + nC, nk);  WD = zeros(nC, nk);
    for c = 1:nk
        G = Gk(:, :, c);
        nn = setdiff(1:nF, IA);   S = G(IA, IA) - G(IA, nn) * pinv(G(nn, nn)) * G(nn, IA);   MG(1, c) = sqrt(max(S, 0));
        nn = setdiff(1:nF, ICP);  S = G(ICP, ICP) - G(ICP, nn) * pinv(G(nn, nn)) * G(nn, ICP);  S = (S + S') / 2;
        [E, D] = eig(S);  [d, ord] = sort(diag(D), 'descend');  E = E(:, ord);
        MG(2:end, c) = sqrt(max(d, 0));
        w = E(:, end);  if w(1) < 0; w = -w; end;  WD(:, c) = w;
    end
end

function [G, negz] = local_G(Lz, idx, FREE, NEG)
    n = 7;  G = zeros(n);  Phi = eye(n);
    for i = idx'
        H1 = Lz(i).H{1};  H2 = Lz(i).H{2};  R1 = Lz(i).R(1);  R2 = Lz(i).R(2);
        r1 = H1 * Phi;  G = G + (r1' * r1) / R1;
        if ~isempty(H2) && R2 > 0; r2 = H2 * Phi;  G = G + (r2' * r2) / R2; end
        Phi = Lz(i).F * Phi;
    end
    negz = all(abs(G(NEG, :)) < 1e-300) && all(abs(G(:, NEG)) < 1e-300);
    G = G(FREE, FREE);
end

function s = local_num(x)
    if x >= 1e4; s = sprintf('%.1e', x); elseif x >= 100; s = sprintf('%.0f', x); elseif x >= 1; s = sprintf('%.1f', x); else; s = sprintf('%.2g', x); end
end

function s = local_flag(negz)
    s = 'zero';  if ~negz; s = 'NEG-CTRL NOT ZERO'; end
end
