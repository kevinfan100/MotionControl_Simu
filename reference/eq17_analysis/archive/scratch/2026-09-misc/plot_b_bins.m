% PURPOSE (2026-09-09): the 4-height-bin arm (b_bins_on, swap implementation) against the constant-b arm and the b_true ceiling,
%   plane wall, no-hold trajectories. Columns Meng | canon; rows (1) seed-mean (a_hat - a)/a_nom, (2) seed-mean b_hat (the bin arm
%   is a staircase: slot 5 carries the b of the bin the probe is in) with b_true(w) at the true height in red and the bin edges
%   as vertical dotted lines in the height sense (drawn as the times at which the commanded height crosses an edge).
%   Output b_bins_plane.png | EXPIRES: with the unknown-wall line | 產線改動不會自動跟上
function plot_b_bins()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    NM = {'meng', 'canon'}; ED = {[1.611 2.111 2.611], [1.625 2.125 2.625]};
    FS = 13; LFS = 10; AXLW = 1.5;
    f = figure('Units','inches','Position',[0 0 14 8.5], 'Color','w', 'Visible','off'); tiledlayout(2, 2, 'TileSpacing','compact', 'Padding','compact');
    for row = 1:2
        for c = 1:2
            A = load(fullfile(od, sprintf('three_walls_%s_nohold_plane_bseed0-btseed0.mat', NM{c})));
            B = load(fullfile(od, sprintf('three_walls_%s_nohold_plane_bseed0_bins.mat', NM{c})));
            base = A.plane_bseed0; cei = A.plane_btseed0; bins = B.plane_bseed0_bins;
            t = base.t(:); hd = base.hd(:); T_END = ceil(t(end));
            nexttile(2*(row-1) + c); hold on; H = []; L = {};
            if row == 2
                H(end+1) = plot(t, mean(cei.B,2), '-', 'Color', [0.8 0 0], 'LineWidth', 2.2); L{end+1} = 'b_{true}(w)  at the true height';
                for e = ED{c}                                  % times at which the commanded height crosses a bin edge
                    xc = t(abs(diff([hd(1); hd])) > 0 & abs(hd - e) < 0.02);
                    for xv = xc(:)'; xline(xv, ':', 'Color', [0.75 0.75 0.75], 'HandleVisibility','off'); end
                end
            end
            if row == 1; yb = mean(base.E,2); yc = mean(cei.E,2); yn = mean(bins.E,2); else; yb = mean(base.B,2); yc = []; yn = mean(bins.B,2); end
            H(end+1) = plot(t, yb, '-', 'Color', [0.35 0.35 0.35], 'LineWidth', 1.6); L{end+1} = 'constant \^b (chord seed)';
            H(end+1) = plot(t, yn, '-', 'Color', [0 0.55 0.2], 'LineWidth', 1.6); L{end+1} = '\^b per height bin (4 bins)';
            if row == 1; H(end+1) = plot(t, yc, '--', 'Color', [0 0.2 0.9], 'LineWidth', 1.6); L{end+1} = 'b_{true}(w) fed (ceiling)'; end
            if row == 1; yline(0, '-', 'Color', [0.6 0.6 0.6], 'HandleVisibility','off'); ylim([-0.035 0.035]); if c == 1; ylabel('(\^a_z - a_z)/a_{nom}   seed mean', 'FontSize', FS, 'FontWeight','bold'); end
                title(sprintf('plane wall, no hold, %s', NM{c}), 'FontSize', FS, 'FontWeight','bold');
            else; ylim([0.855 0.935]); if c == 1; ylabel('b', 'FontSize', FS, 'FontWeight','bold'); end; xlabel('time [s]', 'FontSize', FS, 'FontWeight','bold'); end
            legend(H, L, 'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on','NumColumns',2);
            set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off; xlim([0 T_END]);
        end
    end
    png = fullfile(od, 'b_bins_plane.png'); exportgraphics(f, png, 'Resolution', 150); close(f); fprintf('saved %s\n', png);
end
