% PURPOSE (2026-09-07): read-out of run_three_walls -- one figure per trajectory, columns = plane | sphere | cell (plant),
%   rows: (1) seed-mean (a_hat - a)/a_nom with +- sigma_seed band per estimator arm (prod grey, wide blue, lockb orange);
%         (2) b_hat seed mean per arm with the plant's b (red dashed; plane = Brenner's local b ~0.87);
%         (3) derived wall position w_wall = w - a_hat/(b_hat (1 - a_hat)) seed mean per arm, plant wall (red dashed).
%   Output three_walls_<traj>.png | EXPIRES: with the unknown-wall line | 產線改動不會自動跟上
function plot_three_walls(traj)
    if nargin < 1 || isempty(traj); traj = 'canon'; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    S = load(fullfile(od, sprintf('three_walls_%s.mat', traj)));
    W = {'plane', 0.870, 0.98; 'sphere', 1.156, 1.03; 'cell', 0.877, -0.06};     % plant b (formC writing) and wall position w_s,B
    ARMS = {'prod', [0.45 0.45 0.45], '-'; 'wide', [0 0.2 0.9], '-'; 'lockb', [0.85 0.33 0.10], '--'};
    LAB = {'production prior', 'wall-family prior (Pf_b 0.15, Pf_w0 1 R)', 'b locked 8/9'};
    FS = 13; LFS = 9; AXLW = 1.5;  ph = S.phases;
    f = figure('Units','inches','Position',[0 0 15 10.5], 'Color','w', 'Visible','off'); tiledlayout(3, 3, 'TileSpacing','compact', 'Padding','compact');
    for iw = 1:3
        for row = 1:3
            nexttile(3*(row-1) + iw); hold on; H = []; L = {};
            for ia = 1:size(ARMS,1)
                key = sprintf('%s_%s', W{iw,1}, ARMS{ia,1}); if ~isfield(S, key); continue; end
                d = S.(key); t = d.t(:);
                switch row
                    case 1; m = mean(d.E, 2); s = std(d.E, 0, 2);
                            fill([t; flipud(t)], [m+s; flipud(m-s)], ARMS{ia,2}, 'FaceAlpha', 0.12, 'EdgeColor','none', 'HandleVisibility','off');
                            H(end+1) = plot(t, m, ARMS{ia,3}, 'Color', ARMS{ia,2}, 'LineWidth', 1.6); L{end+1} = LAB{ia};
                    case 2; H(end+1) = plot(t, mean(d.B, 2), ARMS{ia,3}, 'Color', ARMS{ia,2}, 'LineWidth', 1.6); L{end+1} = LAB{ia};
                    case 3; m = mean(d.W, 2); s = std(d.W, 0, 2);
                            fill([t; flipud(t)], [m+s; flipud(m-s)], ARMS{ia,2}, 'FaceAlpha', 0.12, 'EdgeColor','none', 'HandleVisibility','off');
                            H(end+1) = plot(t, m, ARMS{ia,3}, 'Color', ARMS{ia,2}, 'LineWidth', 1.6); L{end+1} = LAB{ia};
                end
            end
            T_END = ceil(t(end));
            if row == 1; yline(0, '-', 'Color', [0.5 0.5 0.5], 'HandleVisibility','off'); end
            if row == 2; hr = yline(W{iw,2}, '--', 'Color', [0.8 0 0], 'LineWidth', 1.4); H(end+1) = hr; L{end+1} = 'plant b'; end
            if row == 3; hr = yline(W{iw,3}, '--', 'Color', [0.8 0 0], 'LineWidth', 1.4); H(end+1) = hr; L{end+1} = 'plant wall'; ylim([-1.5 2]); end
            xline(ph(1), ':', 'Color', [0.3 0.3 0.3]); xline(ph(2), ':', 'Color', [0.3 0.3 0.3]); xline(ph(3), '--', 'Color', [0.3 0.3 0.3]);
            if row == 1; legend(H, L, 'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on','NumColumns',3); end
            if iw == 1
                switch row
                    case 1; ylabel('(\^a_z - a_z)/a_{nom}   seed mean \pm \sigma_{seed}', 'FontSize', FS, 'FontWeight','bold');
                    case 2; ylabel('\^b   seed mean', 'FontSize', FS, 'FontWeight','bold');
                    case 3; ylabel('\^w_{wall} = w/R - \^a_z/(\^b(1-\^a_z))', 'FontSize', FS, 'FontWeight','bold');
                end
            end
            if row == 3; xlabel(sprintf('time [s]   plant = %s wall', W{iw,1}), 'FontSize', FS, 'FontWeight','bold'); end
            set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off; xlim([0 T_END]);
        end
    end
    png = fullfile(od, sprintf('three_walls_%s.png', traj)); exportgraphics(f, png, 'Resolution', 150); close(f); fprintf('saved %s\n', png);
end
