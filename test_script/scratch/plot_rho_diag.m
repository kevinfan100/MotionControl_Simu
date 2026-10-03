% PURPOSE (2026-09-30): sanity figure of the Q/R-ratio design on slot 5 (q55_adapt = 'ratio', run_plaw_walls tag _sig010_rho_ramp2):
%   columns plane | cell (plane init), seed mean, log y where noted
%     row 1  I_k = 1/P55_upd - 1/P55_pred   (this step's information on c, both channels; logged as q55a_stat)
%     row 2  Q55[k] = P55[0]^2 I_{k-1}       (logged as q55a_q)  and rho_k = Q55 I_k (dimensionless Q/R ratio)
%     row 3  N_k = 1/sqrt(rho_k) = memory in steps of this quality; sqrt(P55) against the prior (the design holds it there)
%   Output reference/eq17_analysis/derivation/figures/rho_diag_walls.png | EXPIRES: with the c-only Q55 design line
function plot_rho_diag(suffix)
    if nargin < 1; suffix = '_sig010_rho_ramp2'; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    rd = fullfile(root, 'test_results', 'temp_plaw_walls');  od = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    PL = {'plane', 'cell'};  COL_HAT = [0 0.2 0.9]; FS = 16; LFS = 12; AXLW = 2.0; CG = [0.5 0.5 0.5];
    f = figure('Position', [60 60 1240 1250], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ip = 1:2
        r = load(fullfile(rd, sprintf('f1p2_%s_PI%s.mat', PL{ip}, suffix)));  t = r.t;  seg = r.seg;
        I = mean(r.q55a_stat, 2);  Q = mean(r.q55a_q, 2);  P0 = r.sig^2;
        rho = Q .* I;  N = 1 ./ sqrt(max(rho, 1e-30));
        nexttile(ip); hold on; box on;
        h1 = plot(t, max(I, 1e-6), '-', 'Color', COL_HAT, 'LineWidth', 1.6, 'DisplayName', 'I_k = 1/P_{55}^+ - 1/P_{55}^-  (this step''s information on c, seed mean)');
        h0 = plot(NaN, NaN, 'w.', 'DisplayName', sprintf('%s wall, plane init', upper(PL{ip})));
        set(gca, 'YScale', 'log'); ylim([1e-4 1e2]);  local_seg(seg, [1e-4 1e2]);  xlim([0 t(end)]);
        if ip == 1; ylabel('I_k  [1/c^2 per step]', 'FontSize', FS, 'FontWeight', 'bold'); end
        legend([h0 h1], 'Location', 'northoutside', 'NumColumns', 1, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');  local_ax(FS, AXLW);
        nexttile(2 + ip); hold on; box on;
        h2 = plot(t, max(Q, 1e-12), '-', 'Color', COL_HAT, 'LineWidth', 1.6, 'DisplayName', 'Q_{55}[k] = P_0^2 I_{k-1}  (added at predict)');
        h3 = plot(t, max(rho, 1e-12), '-', 'Color', [0.85 0.45 0], 'LineWidth', 1.6, 'DisplayName', '\rho_k = Q_{55} I_k  (Q/R ratio)');
        set(gca, 'YScale', 'log'); ylim([1e-10 1]);  local_seg(seg, [1e-10 1]);  xlim([0 t(end)]);
        if ip == 1; ylabel('Q_{55},  \rho_k', 'FontSize', FS, 'FontWeight', 'bold'); end
        legend([h2 h3], 'Location', 'northoutside', 'NumColumns', 1, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');  local_ax(FS, AXLW);
        nexttile(4 + ip); hold on; box on;
        h4 = plot(t, min(N, 1e6), '-', 'Color', COL_HAT, 'LineWidth', 1.6, 'DisplayName', 'N_k = 1/sqrt(\rho_k)  (memory, steps of this quality)');
        h5 = plot(t, mean(r.sP_c, 2) ./ sqrt(P0) * 1e3, '-', 'Color', [0.8 0 0], 'LineWidth', 1.6, 'DisplayName', 'sqrt(P_{55}) / sqrt(P_{55}[0]) x 10^3  (1e3 = at the prior)');
        set(gca, 'YScale', 'log'); ylim([1 1e6]);  local_seg(seg, [1 1e6]);  xlim([0 t(end)]);
        if ip == 1; ylabel('N_k,  sqrt(P_{55}) ratio', 'FontSize', FS, 'FontWeight', 'bold'); end
        xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        legend([h4 h5], 'Location', 'northoutside', 'NumColumns', 1, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');  local_ax(FS, AXLW);
        kd = t >= seg(1) & t <= seg(2);  kh = t > seg(2);
        fprintf('%-5s: sum I over descent %.0f, hold %.1f | sum Q55 descent %.4f | rho at descent end %.2e, N %.0f steps | sqrtP55 end of descent %.3f, end %.3f (prior %.3f)\n', ...
                PL{ip}, sum(I(kd)), sum(I(kh)), sum(Q(kd)), rho(find(kd, 1, 'last')), N(find(kd, 1, 'last')), mean(r.sP_c(find(kd, 1, 'last'), :)), mean(r.sP_c(end, :)), sqrt(P0));
    end
    fn = fullfile(od, sprintf('rho_diag_walls%s.png', strrep(suffix, '_sig010_rho_ramp2', '')));
    exportgraphics(f, fn, 'Resolution', 150); close(f); fprintf('figure: %s\n', fn);
end

function local_seg(seg, yl)
    for s = seg; plot([s s], yl, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, 'HandleVisibility', 'off'); end
end

function local_ax(FS, AXLW)
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
end
