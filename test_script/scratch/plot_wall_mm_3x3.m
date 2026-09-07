% FORK OF test_script/scratch/plot_three_walls.m (2026-09-08) | PURPOSE: the 3 plant x 3 prior wall-hypothesis figure.
%   One figure per trajectory, columns = plants (plane / sphere / cell), rows:
%     1  (a_hat_z - a_z)/a_nom, seed mean, one line per prior (shared y across the row)
%     2  dL1 = Lambda1_prior - Lambda1_(matching prior), y1 TERM ONLY (NIS1 ~ 1, calibrated; the y2 term is dominated by ln S2 because
%        R2 is the IF-inflated design value, NIS2 ~ 0.3, and R2 follows a_hat), seed mean +- sd, asinh axis; dashed ln 3, dotted ln 100
%     3  derived wall w_wall = w - a_hat/(b_hat (1 - a_hat)), seed mean +- sd per prior; red = the plant's own w_s + 1/b
%   Colours: prior plane blue, sphere green, cell orange (three hypotheses, so the two-colour house rule cannot apply); truth red.
%   Output wall_mm_3x3_<traj>.png | EXPIRES: with the unknown-wall line | 產線改動不會自動跟上
function plot_wall_mm_3x3(traj)
    if nargin < 1 || isempty(traj); traj = 'canon'; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    S = load(fullfile(od, sprintf('wall_mm_3x3_%s.mat', traj)));
    W = S.walls; PR = S.priors; nW = size(W,1); nP = numel(PR);
    COL = struct('plane', [0 0.2 0.9], 'sphere', [0 0.55 0.2], 'cell', [0.9 0.5 0]); COL_TRUE = [0.8 0 0];
    FS = 13; LFS = 10; AXLW = 1.6;  T_END = S.phases(4);
    f = figure('Units','inches','Position',[0 0 15 12], 'Color','w', 'Visible','off');
    tiledlayout(3, nW, 'TileSpacing','compact', 'Padding','compact');
    yE = [0 0]; yW = [0 0];
    for iw = 1:nW; for ip = 1:nP; d = S.(sprintf('%s_%s', W{iw,1}, PR{ip})); m = mean(d.E,2); yE = [min(yE(1),min(m)) max(yE(2),max(m))]; mw = mean(d.W,2); sw = std(d.W,0,2); ok = isfinite(mw); yW = [min(yW(1),min(mw(ok)-sw(ok))) max(yW(2),max(mw(ok)+sw(ok)))]; end; end
    yE = yE + 0.05*diff(yE)*[-1 1]; yW = [max(yW(1), -2.5) min(yW(2), 2.5)];
    asinh_t = [-100 -10 -1 0 1.1 4.6 10 100 1000 1e4];
    for iw = 1:nW
        wall = W{iw,1}; t = S.(sprintf('%s_%s', wall, PR{1})).t(:);
        if isnan(W{iw,2}); w_true = 0 + 9/8; else; w_true = W{iw,3} + 1/W{iw,2}; end     % plane: anchor law origin 0, b 8/9
        % row 1: gain error
        nexttile(iw); hold on; hs = []; nm = {};
        yline(0, '-', 'Color', [0.5 0.5 0.5], 'LineWidth', 0.8, 'HandleVisibility','off');
        for ip = 1:nP; d = S.(sprintf('%s_%s', wall, PR{ip})); hs(end+1) = plot(t, mean(d.E,2), '-', 'Color', COL.(PR{ip}), 'LineWidth', 1.6); nm{end+1} = sprintf('prior %s', PR{ip}); end
        legend(hs, nm, 'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
        if iw == 1; ylabel('(\^a_z - a_z) / a_{nom}', 'FontSize', FS, 'FontWeight','bold'); end
        title(sprintf('plant %s', wall), 'FontSize', FS, 'FontWeight','bold');
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off; xlim([0 T_END]); ylim(yE);
        % row 2: dL
        nexttile(nW + iw); hold on; hs = []; nm = {};
        if any(strcmp(PR, wall))
            Lr = S.(sprintf('%s_%s', wall, wall)).L1;
            for ip = 1:nP
                if strcmp(PR{ip}, wall); continue; end
                d = S.(sprintf('%s_%s', wall, PR{ip})); dL = d.L1 - Lr; m = mean(dL,2); s = std(dL,0,2);
                fill([t; flipud(t)], asinh([m+s; flipud(m-s)]), COL.(PR{ip}), 'FaceAlpha', 0.18, 'EdgeColor','none', 'HandleVisibility','off');
                hs(end+1) = plot(t, asinh(m), '-', 'Color', COL.(PR{ip}), 'LineWidth', 1.8); nm{end+1} = sprintf('\\Lambda_{1,%s} - \\Lambda_{1,%s}', PR{ip}, wall);
            end
        end
        yline(asinh(log(3)), '--', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.0, 'HandleVisibility','off');
        yline(asinh(log(100)), ':', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.2, 'HandleVisibility','off');
        yline(0, '-', 'Color', [0.5 0.5 0.5], 'LineWidth', 0.8, 'HandleVisibility','off');
        legend(hs, nm, 'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
        if iw == 1; ylabel('\Delta\Lambda_1  (y_1 only; asinh axis; -- ln 3, : ln 100)', 'FontSize', FS, 'FontWeight','bold'); end
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on','YTick',asinh(asinh_t),'YTickLabel',arrayfun(@(v) sprintf('%g',v), asinh_t, 'UniformOutput', false)); grid off; xlim([0 T_END]); ylim(asinh([-30 3e4]));
        % row 3: derived wall
        nexttile(2*nW + iw); hold on; hs = []; nm = {};
        for ip = 1:nP
            d = S.(sprintf('%s_%s', wall, PR{ip})); m = mean(d.W,2); s = std(d.W,0,2); ok = isfinite(m) & isfinite(s);
            fill([t(ok); flipud(t(ok))], [m(ok)+s(ok); flipud(m(ok)-s(ok))], COL.(PR{ip}), 'FaceAlpha', 0.18, 'EdgeColor','none', 'HandleVisibility','off');
            hs(end+1) = plot(t, m, '-', 'Color', COL.(PR{ip}), 'LineWidth', 1.6); nm{end+1} = sprintf('prior %s', PR{ip});
        end
        hs(end+1) = yline(w_true, '-', 'Color', COL_TRUE, 'LineWidth', 1.6); nm{end+1} = 'w_s + 1/b  of the plant';
        legend(hs, nm, 'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
        if iw == 1; ylabel('w_{wall} = w - \^a/(\^b(1-\^a))  [R]', 'FontSize', FS, 'FontWeight','bold'); end
        xlabel('time  [s]', 'FontSize', FS, 'FontWeight','bold');
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off; xlim([0 T_END]); ylim(yW);
    end
    png = fullfile(od, sprintf('wall_mm_3x3_%s.png', traj)); exportgraphics(f, png, 'Resolution', 150); close(f); fprintf('saved %s\n', png);
end
