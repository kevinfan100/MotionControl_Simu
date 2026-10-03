% PURPOSE (2026-10-02, user: "focus on the oscillation segment under the P_0 design"): is the e_a swing inside the oscillation
%   fully explained by the constant-c mismatch potential?  Reconstruct, from the LOGGED mean c_hat(t) and the commanded height,
%   the open-loop accumulation  e_pot[k] = sum (c_eff(h_d) - c_hat) * Delta(1/h_d)  restarted at every top of the oscillation
%   (3.0, 4.0 s), and overlay it on the measured e_a - e_a(top).  Rows: e_a measured vs potential (cell | plane), c_hat vs c_eff.
%   Reads test_results/temp_plaw_walls/f1p2_<plant>_PI_sig010<arm>_osc2.mat.  Output figures/osc_potential_check<arm>.png
%   EXPIRES: with the c-only oscillation judgement
function plot_osc_potential_check(arm)
    if nargin < 1; arm = '_rho20'; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    rd = fullfile(root, 'test_results', 'temp_plaw_walls');  od = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    PL = {'cell', 'plane'};  COL_TRUE = [0.8 0 0]; COL_HAT = [0 0.2 0.9]; COL_POT = [0.85 0.45 0]; FS = 16; LFS = 12; AXLW = 2.0;
    f = figure('Position', [60 60 1240 900], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ip = 1:2
        r = load(fullfile(rd, sprintf('f1p2_%s_PI_sig010%s_osc2.mat', PL{ip}, arm)));  t = r.t;  seg = r.seg;
        ea = mean(r.a_true - r.a_hat, 2);  ch = mean(r.c_hat, 2);  hd = max(r.wd(:), 1);  ce = r.c_eff_d(:);
        dinv = [0; diff(1 ./ hd)];                                        % Delta(1/h_d): positive going down
        inc  = (ce - ch) .* dinv;                                         % open-loop e_a increment of this step (c block only)
        tops = seg(2) + (0.5:1:1.5);                                      % tops of the oscillation at 3.0 and 4.0 s (descent ends at seg(2) = 2.5 s)
        pot = NaN(size(t));  eam = NaN(size(t));
        edges = [seg(2), tops, seg(3)];                                   % windows: trough->top, top->top, top->trough
        for w = 1:numel(edges) - 1
            kk = t >= edges(w) & t < edges(w + 1);  k0 = find(kk, 1);
            pot(kk) = cumsum(inc(kk));  eam(kk) = ea(kk) - ea(k0);        % both restarted at the window start
        end
        nexttile(ip); hold on; box on;
        h1 = plot(t, eam, '-', 'Color', COL_HAT, 'LineWidth', 1.8, 'DisplayName', 'e_a measured, restarted at each window start (seed mean)');
        h2 = plot(t, pot, '-', 'Color', COL_POT, 'LineWidth', 1.8, 'DisplayName', 'potential  \Sigma (c_{eff} - c_{hat}) \Delta(1/h_d)  from the logged c_{hat}');
        h0 = plot(NaN, NaN, 'w.', 'DisplayName', sprintf('%s wall, plane init, %s', upper(PL{ip}), strrep(arm, '_', ' ')));
        yline(0, '-', 'Color', [0.4 0.4 0.4], 'HandleVisibility', 'off');
        for s = [seg(2) tops seg(3)]; plot([s s], [-0.1 0.1], '--', 'Color', [0.5 0.5 0.5], 'HandleVisibility', 'off'); end
        xlim([seg(2) - 0.5, t(end)]); ylim([-0.1 0.1]);
        if ip == 1; ylabel('e_a  [a / a_{nom}]', 'FontSize', FS, 'FontWeight', 'bold'); end
        legend([h0 h1 h2], 'Location', 'northoutside', 'NumColumns', 1, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');  local_ax(FS, AXLW);
        nexttile(2 + ip); hold on; box on;
        h3 = plot(t, ce, '-', 'Color', COL_TRUE, 'LineWidth', 2.0, 'DisplayName', 'c_{eff}(h_d)');
        h4 = plot(t, ch, '-', 'Color', COL_HAT, 'LineWidth', 1.6, 'DisplayName', 'c_{hat}  (seed mean)');
        for s = [seg(2) tops seg(3)]; plot([s s], [-0.4 1], '--', 'Color', [0.5 0.5 0.5], 'HandleVisibility', 'off'); end
        xlim([seg(2) - 0.5, t(end)]); ylim([-0.4 1]);
        if ip == 1; ylabel('c', 'FontSize', FS, 'FontWeight', 'bold'); end
        xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        legend([h3 h4], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');  local_ax(FS, AXLW);
        for w = 1:numel(edges) - 1
            kk = t >= edges(w) & t < edges(w + 1);  [~, im] = max(abs(eam(kk)));  e1 = eam(kk); p1 = pot(kk);
            fprintf('%-5s window %.1f-%.1f s: measured extreme %+.3f, potential at the same instant %+.3f, potential extreme %+.3f\n', PL{ip}, edges(w), edges(w + 1), e1(im), p1(im), p1(find(abs(p1) == max(abs(p1)), 1)));
        end
    end
    fn = fullfile(od, sprintf('osc_potential_check%s.png', arm));  exportgraphics(f, fn, 'Resolution', 150); close(f); fprintf('figure: %s\n', fn);
end

function local_ax(FS, AXLW)
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
end
