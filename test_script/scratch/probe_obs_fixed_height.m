% PURPOSE (2026-09-23, user: "new line, step by step: first check the fixed-point observability at the current highest and
%   lowest points"; then "use c and p, h notation, plane init on plane AND cell"): the estimator held at ONE commanded height
%   for the whole run (trajectory 'positioning'), plane init, law F1 = the advisor's a' = (1-c)/h^p with c and p both states
%   (plaw_cp_slope), walls plane and cell (plant only), heights = the two ends of the canon band 22.2 R and 1.10 R, one seed.
%   (The first pass, laws A = (delta, p) / B = (delta, c) on the plane, is kept selectable through LAWS.)
%   Exact observability from the logged F_e / H / R (obs_dump) over the whole run:
%       G = sum_k Phi(k,k0)' H_k' R_k^-1 H_k Phi(k,k0)   on the free slots [dw1 dw2 dw3 a_w slot5 slot7]   (slot 6 locked = negative control)
%       G v_i = sigma_i^2 v_i,  sigma_1 >= ... >= sigma_6;  1/sigma_i = data-only error bar along v_i (no prior, no Q).
%   Also sigma_i(t) accumulated from t = 0 every STRIDE steps (the "when does information arrive" curve).
%   Figures (house style, singular values and V only, h notation: h_d = commanded gap, w_s = 0 on both plants' visible surface):
%     obs_fixed_V_<law>.png            V tables: rows = walls; columns = heights 22.2, 1.10 R  (0907/0916 table style)
%     obs_fixed_sigma_<law>_<wall>.png columns = heights; rows = h_d, sigma_4, sigma_5, sigma_6 accumulated from t = 0 (linear, y shared per row)
%   Usage: probe_obs_fixed_height()        compute (walls x heights runs of T_HOLD s), save, plot;   probe_obs_fixed_height('plot') replot from cache
%   Cache test_results/temp_plaw_walls/obs_fixed_height_<law>.mat   EXPIRES: with the fixed-point / trajectory design line
function out = probe_obs_fixed_height(arg, LAWS)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration')); addpath(here);
    od = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    rd = fullfile(root, 'test_results', 'temp_plaw_walls');  if ~exist(rd, 'dir'); mkdir(rd); end
    if nargin < 2 || isempty(LAWS); LAWS = {'f1'}; end
    fm = fullfile(rd, sprintf('obs_fixed_height_%s.mat', strjoin(LAWS, '_')));
    if nargin >= 1 && ischar(arg) && strcmp(arg, 'plot'); out = load(fm); local_plot(out, od); return; end
    pc = physical_constants();  Ts = pc.Ts;  R = pc.R;
    HEIGHTS = [22.2 1.10];  T_HOLD = 3.0;  seed = 7;  NEG = 6;  STRIDE = 4;
    WALLS = {'plane', []; 'cell', @cell_cperp_delta};
    out = struct('HEIGHTS', HEIGHTS, 'laws', {LAWS}, 'walls', {WALLS(:, 1)'}, 'T_HOLD', T_HOLD, 'seed', seed, 'STRIDE', STRIDE);
    fprintf('\n%-4s %-6s %6s | %-52s | %-16s | %s\n', 'law', 'wall', 'h_d', 'sigma_1 .. sigma_n (whitened, whole run)', '|v_n| on slot5,7', 'neg-ctrl');
    for il = 1:numel(LAWS)
      law = LAWS{il};  FREE = local_free(law);  out.FREE = FREE;   % 'f1p2': p pinned -> slot 7 is not a free state, the analysis is conditional on p
      for iw = 1:size(WALLS, 1)
        wall = WALLS{iw, 1};
        for ih = 1:numel(HEIGHTS)
            h = HEIGHTS(ih);  cc = local_cc(law);
            OV = struct('trajectory_type', 'positioning', 'h_init', h * R, 'T_sim', T_HOLD, 'h_bottom', h * R, ...
                        'amplitude', 0, 't_hold', 0, 't_descend_override', 0);
            o = struct('arm', 'best', 'ctrl_const_override', cc, 'config_override', OV, 'scenario', 'deep', 'verbose', false, 'seeds', seed, 'log_P_full', false);
            if ~isempty(WALLS{iw, 2}); o.plant_cperp = WALLS{iw, 2}; end
            clear run_formC_b motion_control_law_formC_b;
            evalc('Rr = run_formC_b(o);');  L = obs_dump('get');  Lz = L([L.ax] == 3);
            t = [Lz.k]' * Ts;  idx = (1:numel(Lz))';  wd = Rr.runs{1}.h_bar_d_out(:);
            [G, negz] = local_G(Lz, idx, FREE, NEG);
            [Vv, Dd] = eig((G + G') / 2);  [d, ord] = sort(diag(Dd), 'descend');  Vv = Vv(:, ord);  nF = numel(FREE);
            for j = 1:nF; [~, im] = max(abs(Vv(:, j)));  Vv(:, j) = Vv(:, j) * sign(Vv(im, j)); end
            SIG = sqrt(max(d, 0));
            [SIGk, tk] = local_accum_sigma(Lz, idx, FREE, STRIDE, t);
            r = struct('h', h, 'SIG', SIG, 'V', Vv, 'negctrl_ok', negz, 'SIGk', SIGk, 'tk', tk, 'wd', wd, 't', t, 'FREE', FREE);
            out.(law).(wall)(ih) = r;
            v7 = NaN;  if nF >= 6; v7 = abs(Vv(6, nF)); end
            fprintf('%-4s %-6s %6.2f | %s | %5.2f %5.2f      | %s\n', law, wall, h, sprintf('%9.2f', SIG), abs(Vv(5, nF)), v7, local_flag(negz));
        end
      end
    end
    save(fm, '-struct', 'out');
    local_plot(out, od);
end

function local_plot(out, od)
    LAWL = struct('f4', 'A (\delta, p)', 'f2', 'B (\delta, c)', 'f1', '(c, p)  a'' = (1-c)/h^p', 'f1p2', 'c only,  a'' = (1-c)/h^2  (p = 2 pinned)');
    LAB5 = struct('f4', 'p', 'f2', 'c', 'f1', 'c', 'f1p2', 'c');  LAB7 = struct('f4', '\delta', 'f2', '\delta', 'f1', 'p', 'f1p2', 'p');
    WLAB = struct('plane', 'PLANE', 'cell', 'CELL');
    FS = 15; LFS = 11;  nH = numel(out.HEIGHTS);  nW = numel(out.walls);
    for il = 1:numel(out.laws)
        law = out.laws{il};  FREE = local_free(law);  n = numel(FREE);
        labs = {'\deltaw_1', '\deltaw_2', '\deltaw_3', 'a_w', LAB5.(law), '(6)', LAB7.(law)};  lab = labs(FREE);
        % ===== figure 1: V tables, rows = walls, columns = heights
        fig = figure('Position', [60 60 700 * nH + 100, 480 * nW + 60], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
        tiledlayout(nW, nH, 'TileSpacing', 'compact', 'Padding', 'compact');
        for iw = 1:nW
            wall = out.walls{iw};
            for ih = 1:nH
                r = out.(law).(wall)(ih);  V = r.V;  sv = r.SIG;
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
                legend(h, {sprintf('%s wall   %s   fixed  h_d = %g R   %g s   plane init', WLAB.(wall), LAWL.(law), r.h, out.T_HOLD)}, 'Location', 'northoutside', ...
                       'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
            end
        end
        fn = fullfile(od, sprintf('obs_fixed_V_%s.png', law));  exportgraphics(fig, fn, 'Resolution', 150); close(fig); fprintf('figure: %s\n', fn);
        %#ok<*AGROW>  (the V-table loop above uses n = numel(FREE) for the law)
        % ===== figure 2 (per wall): accumulated sigma_4..n vs time, columns = heights, rows share y
        COL_LINE = [0 0.2 0.9];  COL_CMD = [0.2 0.2 0.2];  FS2 = 16; LFS2 = 12; AXLW = 2.0;
        ROWI = 4:n;  nR = 1 + numel(ROWI);
        for iw = 1:nW
            wall = out.walls{iw};
            f = figure('Position', [60 60 560 * nH + 120, 260 * nR + 60], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
            tiledlayout(nR, nH, 'TileSpacing', 'compact', 'Padding', 'compact');
            ax = gobjects(nR, nH);
            for ih = 1:nH
                r = out.(law).(wall)(ih);  t = r.tk(:);
                ax(1, ih) = nexttile(ih); hold on; box on;
                h1 = plot(r.t, r.wd(1:numel(r.t)), '-', 'Color', COL_CMD, 'LineWidth', 2.0, ...
                          'DisplayName', sprintf('h_d   %s wall,  %s,  plane init,  fixed at %g R', WLAB.(wall), LAWL.(law), r.h));
                legend(h1, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS2, 'FontWeight', 'bold', 'Box', 'on');
                if ih == 1; ylabel('h_d  (R)', 'FontSize', FS2, 'FontWeight', 'bold'); end
                ylim([0 25]);
                for q = 1:numel(ROWI)
                    i = ROWI(q);
                    ax(1 + q, ih) = nexttile(q * nH + ih); hold on; box on;
                    hh = plot(t, r.SIGk(i, :), '-', 'Color', COL_LINE, 'LineWidth', 2.0, 'DisplayName', sprintf('\\sigma_%d   accumulated from t = 0   (which state: see the V table)', i));
                    if ih == 1
                        legend(hh, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS2, 'FontWeight', 'bold', 'Box', 'on');
                        ylabel(sprintf('\\sigma_%d', i), 'FontSize', FS2, 'FontWeight', 'bold');
                    end
                end
                xlabel('Time (sec)', 'FontSize', FS2, 'FontWeight', 'bold');
                for q = 1:nR
                    set(ax(q, ih), 'FontSize', FS2, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid(ax(q, ih), 'off');
                    xlim(ax(q, ih), [0 t(end)]);
                end
            end
            for q = 2:nR; linkaxes(ax(q, :), 'y'); end
            fn = fullfile(od, sprintf('obs_fixed_sigma_%s_%s.png', law, wall));  exportgraphics(f, fn, 'Resolution', 150); close(f); fprintf('figure: %s\n', fn);
        end
    end
end

function FREE = local_free(law)
    FREE = [1 2 3 4 5 7];  if strcmp(law, 'f1p2'); FREE = [1 2 3 4 5]; end   % p pinned: not a free state
end

function [SIGk, tk] = local_accum_sigma(Lz, idx, FREE, STRIDE, t)
    n = 7;  G = zeros(n);  Phi = eye(n);  m = numel(idx);  ks = 1:STRIDE:m;
    SIGk = zeros(numel(FREE), numel(ks));  tk = zeros(1, numel(ks));  c = 0;
    for j = 1:m
        i = idx(j);
        H1 = Lz(i).H{1};  H2 = Lz(i).H{2};  R1 = Lz(i).R(1);  R2 = Lz(i).R(2);
        r1 = H1 * Phi;  G = G + (r1' * r1) / R1;
        if ~isempty(H2) && R2 > 0; r2 = H2 * Phi;  G = G + (r2' * r2) / R2; end
        Phi = Lz(i).F * Phi;
        if any(j == ks)
            c = c + 1;  Gf = G(FREE, FREE);  d = sort(eig((Gf + Gf') / 2), 'descend');  SIGk(:, c) = sqrt(max(d, 0));  tk(c) = t(i);
        end
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

function cc = local_cc(law)
    cc = struct('ma2_aug', false, 'nw_mcorr', false, 'pred_mean2', false, 'pred_mean2_e4', false, 'law_exact_step', false, ...
                'lock_b', false, 'gain_algebraic', false, 'amodel_fe', false, 'amodel_h', false, 'b_ceil', 1.5, 'b_floor', 0.2, ...
                'ws0_perp', 0, 'Pf_w0_std', 0, 'Pf_a_floor', 0.002, 'plaw_g_min', 0.5, 'plaw_delta_lim', [-0.9, 3], ...
                'delta_init', 0.104, 'Pf_delta_std', 0.64, 'obs_dump', true);
    switch law
        case 'f2'; cc.plaw_dc_slope = true;  cc.plaw_p = 2;  cc.c_init = -0.085;  cc.Pf_c_std = 0.10;
        case 'f4'; cc.plaw_dp_slope = true;  cc.p_init = 1.950;  cc.Pf_p_std = 0.10;  cc.plaw_p_lim = [1.2, 4];  cc.delta_init = 0.059;
        case 'f1'                                                            % (c, p) both states, no shift: the plane's F1 band pair (fit_two_param_height_walls.m)
            cc = rmfield(cc, {'plaw_g_min', 'plaw_delta_lim', 'delta_init', 'Pf_delta_std'});
            cc.plaw_cp_slope = true;  cc.c_init = 0.096;  cc.Pf_c_std = 0.10;  cc.p_init = 1.894;  cc.Pf_p_std = 0.10;  cc.plaw_p_lim = [1.2, 4];
        case 'f1p2'                                                          % c only, p pinned at 2 (user 09-23 evening): plane c at p = 2 = 0.0533, family-width prior 0.6
            cc = rmfield(cc, {'plaw_g_min', 'plaw_delta_lim', 'delta_init', 'Pf_delta_std'});
            cc.plaw_cp_slope = true;  cc.c_init = 0.0533;  cc.Pf_c_std = 0.6;  cc.p_init = 2;  cc.Pf_p_std = 1e-6;  cc.plaw_p_lim = [1.2, 4];
    end
end

function s = local_num(x)
    if x >= 1e4; s = sprintf('%.1e', x); elseif x >= 100; s = sprintf('%.0f', x); elseif x >= 1; s = sprintf('%.1f', x); else; s = sprintf('%.2g', x); end
end

function s = local_flag(negz)
    s = 'zero';  if ~negz; s = 'NEG-CTRL NOT ZERO'; end
end
