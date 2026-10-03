% PURPOSE (2026-09-23, user: "hold - constant-velocity descent - hold, only T_d varies; A = (delta, p), B = (delta, c); show the
%   singular values numbered and the singular vectors mapped to the states"): exact observability of the two-constant slope law
%   along the ramp template, per wall x law x T_d, from ONE logged run each (obs_dump: F_e, H, R every step, plane init):
%       window = the descent [t_hold, t_hold + T_d];  Phi(k,k0) = F_e[k-1]...F_e[k0];
%       G = sum_k Phi' H' R^-1 H Phi over the free slots [dw1 dw2 dw3 a_w slot5 delta]  (slot 6 locked = negative control)
%       G v_i = sigma_i^2 v_i  (sigma_1 >= ... >= sigma_6);  CRLB_theta = sqrt([G^-1]_theta theta) by eigen-decomposition;
%       ratio = CRLB / prior (0.10 for p or c, 0.64 for delta).
%       channel split: y1 rows only / y2 rows only, each with the run's own P[0] added on the NON-constant slots 1-4 only
%       (y2 alone never sees dw1, dw2, so the joint solve is singular without it; the constants keep no prior in the split).
%   Trajectory: canon heights (22.2 -> 1.10 R), t_hold 0.5 s, T_d in TD, final hold 0.5 s (trajectory_generator ramp mode with
%   t_hold / t_descend_override, 09-23). Figures (user 09-23: singular values and V only, no CRLB panels; the agreed grid is
%   THREE speeds T_d = 0.25 / 2 / 16 s x two laws A, B x two walls plane, cell -- the 7-point T_d grid and the sphere are
%   computed and kept in the .mat but not plotted):
%     obs_ramp_sigma_Td_<wall>.png  (A | B) row 1 sigma_1..6 at the three T_d labelled 1..6; row 2 |v_6| on the six states
%     obs_ramp_V_<wall>.png         V tables in the 0907/0916 style (rows = states, columns = v_1..v_6 with sigma_i beneath),
%                                   A and B at T_d = 0.25, 2, 16 s
%   Usage: probe_obs_ramp_Td()          compute (42 single-seed runs, ~10 min), save, plot
%          probe_obs_ramp_Td('plot')    plot from test_results/temp_plaw_walls/obs_ramp_Td.mat
%   Output reference/eq17_analysis/derivation/figures/obs_ramp_Td_<wall>.png, the .mat, console.  EXPIRES: with the T_d design line
function out = probe_obs_ramp_Td(TD, seed)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration')); addpath(here);
    od = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    rd = fullfile(root, 'test_results', 'temp_plaw_walls');  if ~exist(rd, 'dir'); mkdir(rd); end
    fm = fullfile(rd, 'obs_ramp_Td.mat');
    if nargin >= 1 && ischar(TD) && strcmp(TD, 'plot')
        out = load(fm);  local_plot(out, od);  return
    end
    if nargin < 1 || isempty(TD); TD = [0.25 0.5 1 2 4 8 16]; end
    if nargin < 2 || isempty(seed); seed = 7; end
    pc = physical_constants();  Ts = pc.Ts;  T_HOLD = 0.5;
    SIG_D = 0.64;  SIG_5 = 0.10;  FREE = [1 2 3 4 5 7];  NEG = 6;
    LAWS  = {'f4', 'f2'};
    WALLS = {'plane', []; 'cell', @cell_cperp_delta; 'sphere_pt', @sphere_cperp_fig11_ptail};
    nT = numel(TD);
    out = struct('TD', TD, 'seed', seed, 'walls', {WALLS(:, 1)'}, 'laws', {LAWS});
    fprintf('\n%-9s %-4s %6s | %-52s | %-16s | %-16s | %-22s\n', 'wall', 'law', 'T_d', 'sigma_1 .. sigma_6 (whitened, whole descent)', '|v_6| on slot5,dl', 'CRLB/prior 5, dl', 'y1-only / y2-only (slot5, dl)');
    for iw = 1:size(WALLS, 1)
        wall = WALLS{iw, 1};
        for il = 1:2
            law = LAWS{il};
            SIG = zeros(6, nT);  V6 = zeros(6, nT);  VF = zeros(6, 6, nT);  CR = zeros(2, nT);  CR1 = zeros(2, nT);  CR2 = zeros(2, nT);  NEGOK = false(1, nT);
            for it = 1:nT
                Td = TD(it);
                cc = local_cc(law);
                OV = struct('trajectory_type', 'ramp_descent', 't_hold', T_HOLD, 't_descend_override', Td, 'T_sim', T_HOLD + Td + 0.5, ...
                            'amplitude', 0, 'frequency', 1, 'n_cycles', 1);
                o = struct('arm', 'best', 'ctrl_const_override', cc, 'config_override', OV, 'scenario', 'deep', 'verbose', false, 'seeds', seed, 'log_P_full', false);
                if ~isempty(WALLS{iw, 2}); o.plant_cperp = WALLS{iw, 2}; end
                clear run_formC_b motion_control_law_formC_b;
                evalc('R = run_formC_b(o);');  L = obs_dump('get');  Lz = L([L.ax] == 3);
                t = [Lz.k]' * Ts;  idx = find(t >= T_HOLD & t < T_HOLD + Td);
                P0 = Lz(idx(1)).P_pred;                                         % the run's own prior at the window start (nuisance slots only, for the split)
                [G, G1, G2, negz] = local_G(Lz, idx, FREE, NEG);
                Preg = zeros(numel(FREE));  for j = 1:4; Preg(j, j) = 1 / max(P0(FREE(j), FREE(j)), 1e-30); end
                [Vv, Dd] = eig((G + G') / 2);  [d, ord] = sort(diag(Dd), 'descend');  Vv = Vv(:, ord);
                for j = 1:6; [~, im] = max(abs(Vv(:, j)));  Vv(:, j) = Vv(:, j) * sign(Vv(im, j)); end   % sign: largest component positive
                SIG(:, it) = sqrt(max(d, 0));  V6(:, it) = abs(Vv(:, end));  VF(:, :, it) = Vv;  NEGOK(it) = negz;
                cr = local_crlb(G);  cr1 = local_crlb(G1 + Preg);  cr2 = local_crlb(G2 + Preg);
                CR(:, it) = [cr(5) / SIG_5; cr(6) / SIG_D];  CR1(:, it) = [cr1(5) / SIG_5; cr1(6) / SIG_D];  CR2(:, it) = [cr2(5) / SIG_5; cr2(6) / SIG_D];
                fprintf('%-9s %-4s %6.2f | %s | %5.2f %5.2f      | %7.4f %7.4f  | %6.3f %6.3f / %6.3f %6.3f%s\n', wall, law, Td, ...
                        sprintf('%9.1f', SIG(:, it)), V6(5, it), V6(6, it), CR(1, it), CR(2, it), CR1(1, it), CR1(2, it), CR2(1, it), CR2(2, it), local_flag(negz));
            end
            out.(wall).(law) = struct('SIG', SIG, 'V6', V6, 'V', VF, 'CR', CR, 'CR1', CR1, 'CR2', CR2, 'negctrl_ok', NEGOK);
        end
    end
    save(fm, '-struct', 'out');
    local_plot(out, od);
end

function local_plot(out, od)
    TD = out.TD;
    TSEL = [0.25 2 16];  [~, isel] = ismember(TSEL, TD);  isel = isel(isel > 0);  TDs = TD(isel);  nT = numel(isel);   % the agreed three speeds
    WPLOT = {'plane', 'cell'};                                                                                        % the agreed two walls
    LAWL = struct('f4', 'A (\delta, p)', 'f2', 'B (\delta, c)');  LAB5 = struct('f4', 'p', 'f2', 'c');
    WLAB = struct('plane', 'PLANE', 'cell', 'CELL', 'sphere_pt', 'SPHERE (plane tail)');
    FS = 15; LFS = 11;  cols = lines(6);
    for iw = 1:numel(WPLOT)
        wall = WPLOT{iw};  if ~isfield(out, wall); continue; end
        % ===== figure 1: sigma_i at the three T_d (numbered) and |v_6| on the states
        fig = figure('Position', [60 60 1500 950], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
        tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
        for il = 1:numel(out.laws)
            law = out.laws{il};  r = out.(wall).(law);  lab5 = LAB5.(law);
            nexttile(il); hold on; box on;
            for i = 1:6
                plot(TDs, r.SIG(i, isel), '-o', 'Color', cols(i, :), 'LineWidth', 1.6, 'MarkerFaceColor', cols(i, :), 'MarkerSize', 6);
                text(TDs(end) * 1.12, r.SIG(i, isel(end)), sprintf('%d', i), 'Color', cols(i, :), 'FontSize', 13, 'FontWeight', 'bold');
            end
            set(gca, 'XScale', 'log', 'YScale', 'log');  xlim([TDs(1) / 1.3, TDs(end) * 1.6]);
            set(gca, 'XTick', TDs, 'XTickLabel', arrayfun(@(x) sprintf('%g', x), TDs, 'UniformOutput', false));
            h0 = plot(NaN, NaN, 'w.', 'DisplayName', sprintf('%s wall,  %s,  plane init', WLAB.(wall), LAWL.(law)));
            legend(h0, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
            if il == 1; ylabel('\sigma_i  (whitened O, whole descent)', 'FontSize', FS, 'FontWeight', 'bold'); end
            xlabel('T_d  (s)', 'FontSize', FS, 'FontWeight', 'bold');
            local_ax(FS);
            nexttile(2 + il); hold on; box on;
            imagesc(1:nT, 1:6, r.V6(:, isel));  colormap(gca, flipud(gray));  caxis([0 1]);
            set(gca, 'YDir', 'reverse');  xlim([0.5 nT + 0.5]);  ylim([0.5 6.5]);
            set(gca, 'YTick', 1:6, 'YTickLabel', {'\deltaw_1', '\deltaw_2', '\deltaw_3', 'a_w', lab5, '\delta'}, ...
                     'XTick', 1:nT, 'XTickLabel', arrayfun(@(x) sprintf('%g', x), TDs, 'UniformOutput', false), 'TickLength', [0 0]);
            for q = 1:nT; for i = 1:6
                it = isel(q);
                tc = 'k';  if r.V6(i, it) > 0.55; tc = 'w'; end
                text(q, i, sprintf('%.2f', r.V6(i, it)), 'HorizontalAlignment', 'center', 'FontSize', 11, 'FontWeight', 'bold', 'Color', tc);
            end; end
            h0 = plot(NaN, NaN, 'w.', 'DisplayName', '|v_6|: the weakest direction, component on each state, per T_d');
            legend(h0, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
            if il == 1; ylabel('state', 'FontSize', FS, 'FontWeight', 'bold'); end
            xlabel('T_d  (s)', 'FontSize', FS, 'FontWeight', 'bold');
            local_ax(FS);
        end
        fn = fullfile(od, sprintf('obs_ramp_sigma_Td_%s.png', wall));  exportgraphics(fig, fn, 'Resolution', 150); close(fig); fprintf('figure: %s\n', fn);
        % ===== figure 2: V tables, A and B at the three T_d
        fig = figure('Position', [60 60 1500 1000], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
        tiledlayout(2, numel(isel), 'TileSpacing', 'compact', 'Padding', 'compact');
        for il = 1:numel(out.laws)
            law = out.laws{il};  r = out.(wall).(law);  lab5 = LAB5.(law);  lab = {'\deltaw_1', '\deltaw_2', '\deltaw_3', 'a_w', lab5, '\delta'};
            for q = 1:numel(isel)
                it = isel(q);  V = r.V(:, :, it);  sv = r.SIG(:, it);  n = 6;
                nexttile; hold on; box on;
                imagesc(abs(V), [0 1]);  colormap(gca, flipud(gray));
                for i = 1:n; for j = 1:n
                    tc = 'k';  if abs(V(i, j)) > 0.55; tc = 'w'; end
                    text(j, i, sprintf('%.2f', V(i, j)), 'HorizontalAlignment', 'center', 'FontSize', 12, 'FontWeight', 'bold', 'Color', tc);
                end; end
                set(gca, 'YDir', 'reverse', 'XTick', 1:n, 'YTick', 1:n, 'YTickLabel', lab, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.5, 'TickLength', [0 0]);
                xt = cell(1, n);  for j = 1:n; xt{j} = sprintf('v_%d\\newline%s', j, local_num(sv(j))); end
                set(gca, 'XTickLabel', xt, 'XTickLabelRotation', 0);
                xlabel('\sigma_i', 'FontSize', FS, 'FontWeight', 'bold');
                xlim([0.5 n + 0.5]);  ylim([0.5 n + 0.5]);
                h = plot(NaN, NaN, 'w.');
                legend(h, {sprintf('%s   %s   descent   T_d = %g s', WLAB.(wall), LAWL.(law), TD(it))}, 'Location', 'northoutside', ...
                       'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
            end
        end
        fn = fullfile(od, sprintf('obs_ramp_V_%s.png', wall));  exportgraphics(fig, fn, 'Resolution', 150); close(fig); fprintf('figure: %s\n', fn);
    end
end

function s = local_num(x)
    if x >= 1e4; s = sprintf('%.1e', x); elseif x >= 100; s = sprintf('%.0f', x); else; s = sprintf('%.1f', x); end
end

function cc = local_cc(law)
    cc = struct('ma2_aug', false, 'nw_mcorr', false, 'pred_mean2', false, 'pred_mean2_e4', false, 'law_exact_step', false, ...
                'lock_b', false, 'gain_algebraic', false, 'amodel_fe', false, 'amodel_h', false, 'b_ceil', 1.5, 'b_floor', 0.2, ...
                'ws0_perp', 0, 'Pf_w0_std', 0, 'Pf_a_floor', 0.002, 'plaw_g_min', 0.5, 'plaw_delta_lim', [-0.9, 3], ...
                'delta_init', 0.104, 'Pf_delta_std', 0.64, 'obs_dump', true);
    switch law
        case 'f2'; cc.plaw_dc_slope = true;  cc.plaw_p = 2;  cc.c_init = -0.085;  cc.Pf_c_std = 0.10;
        case 'f4'; cc.plaw_dp_slope = true;  cc.p_init = 1.950;  cc.Pf_p_std = 0.10;  cc.plaw_p_lim = [1.2, 4];  cc.delta_init = 0.059;
    end
end

function [G, G1, G2, negz] = local_G(Lz, idx, FREE, NEG)
% exact Fisher information over the window: G = sum Phi' H' R^-1 H Phi (free slots), plus y1-only and y2-only versions;
% negz = true when the negative-control slot has an identically zero row and column (rank-check instrument)
    n = 7;  G1 = zeros(n);  G2 = zeros(n);  Phi = eye(n);
    for i = idx'
        H1 = Lz(i).H{1};  H2 = Lz(i).H{2};  R1 = Lz(i).R(1);  R2 = Lz(i).R(2);
        r1 = H1 * Phi;  G1 = G1 + (r1' * r1) / R1;
        if ~isempty(H2) && R2 > 0; r2 = H2 * Phi;  G2 = G2 + (r2' * r2) / R2; end
        Phi = Lz(i).F * Phi;
    end
    G = G1 + G2;
    negz = all(abs(G(NEG, :)) < 1e-300) && all(abs(G(:, NEG)) < 1e-300);
    G = G(FREE, FREE);  G1 = G1(FREE, FREE);  G2 = G2(FREE, FREE);
end

function cr = local_crlb(G)
    [E, D] = eig((G + G') / 2);  d = diag(D);  ok = d > max(d) * 1e-13;
    cr = inf(size(G, 1), 1);
    if all(ok); Ci = E * diag(1 ./ d) * E';  cr = sqrt(abs(diag(Ci))); end
end

function s = local_flag(negz)
    s = '';  if ~negz; s = '   NEG-CTRL NOT ZERO'; end
end

function local_ax(FS)
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.5, 'Box', 'on'); grid off;
end
