% PURPOSE (2026-09-09): the forgetting-factor comparison on the 'ramp' plant (b 1.16 near the wall -> 0.87 far, step at 2 R).
%   Columns Meng | canon; rows (1) seed-mean (a_hat - a)/a_nom per arm, (2) seed-mean b_hat per arm with the true local b(w) (red).
%   Arms: constant b_hat (chord seed) | lambda 0.999 | lambda 0.9999 | adaptive (forget when y2 is surprised) | b_true fed (ceiling).
%   Output forgetting_ramp.png | EXPIRES: with the unknown-wall line | 產線改動不會自動跟上
function plot_forgetting_ramp()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    TAG = 'plane-ramp_bseed0-bseed0_lf999-bseed0_lf9999-bseed0_lfa-btseed0';
    ARMS = {'bseed0', [0.35 0.35 0.35], '-', 'constant \^b (current)'; 'bseed0_lf999', [0.9 0.5 0], '-', '\lambda = 0.999'; ...
            'bseed0_lf9999', [0 0.55 0.2], '-', '\lambda = 0.9999'; 'bseed0_lfa', [0.55 0 0.7], '-', 'adaptive (forget when y_2 surprised)'; ...
            'btseed0', [0 0.2 0.9], '--', 'b_{true}(w) fed (ceiling)'};
    NM = {'meng', 'canon'}; FS = 13; LFS = 10; AXLW = 1.5;
    f = figure('Units','inches','Position',[0 0 14 8.5], 'Color','w', 'Visible','off'); tiledlayout(2, 2, 'TileSpacing','compact', 'Padding','compact');
    for row = 1:2
        for c = 1:2
            S = load(fullfile(od, sprintf('three_walls_%s_%s.mat', NM{c}, TAG)));
            nexttile(2*(row-1) + c); hold on; H = []; L = {};
            t = S.ramp_bseed0.t(:); T_END = ceil(t(end));
            if row == 2; bt = mean(S.ramp_btseed0.B, 2); H(end+1) = plot(t, bt, '-', 'Color', [0.8 0 0], 'LineWidth', 2.2); L{end+1} = 'b_{true}(w)  at the true height'; end
            for ia = 1:size(ARMS, 1)
                d = S.(sprintf('ramp_%s', ARMS{ia,1}));
                if row == 1; y = mean(d.E, 2); else; if strcmp(ARMS{ia,1}, 'btseed0'); continue; end; y = mean(d.B, 2); end
                H(end+1) = plot(t, y, ARMS{ia,3}, 'Color', ARMS{ia,2}, 'LineWidth', 1.6); L{end+1} = ARMS{ia,4};
            end
            if row == 1; yline(0, '-', 'Color', [0.6 0.6 0.6], 'HandleVisibility','off'); ylim([-0.06 0.06]); if c == 1; ylabel('(\^a_z - a_z)/a_{nom}   seed mean', 'FontSize', FS, 'FontWeight','bold'); end
            else; ylim([0.8 1.25]); if c == 1; ylabel('b', 'FontSize', FS, 'FontWeight','bold'); end; xlabel('time [s]', 'FontSize', FS, 'FontWeight','bold'); end
            ph = S.phases; xline(ph(2), ':', 'Color', [0.3 0.3 0.3]); xline(ph(3), '--', 'Color', [0.3 0.3 0.3]);
            if row == 1; title(sprintf('plant = ramp wall (b 1.16 near -> 0.87 far), %s', NM{c}), 'FontSize', FS, 'FontWeight','bold'); end
            legend(H, L, 'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on','NumColumns',3);
            set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off; xlim([0 T_END]);
        end
    end
    png = fullfile(od, 'forgetting_ramp.png'); exportgraphics(f, png, 'Resolution', 150); close(f); fprintf('saved %s\n', png);
end
