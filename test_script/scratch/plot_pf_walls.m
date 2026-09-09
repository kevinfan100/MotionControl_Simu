% PURPOSE (2026-09-09): the P55-floor arm (directional forgetting, steady-state form) against the constant-b arm and the
%   b_true ceiling, on one wall. Columns Meng | canon; rows (1) seed-mean (a_hat - a)/a_nom, (2) seed-mean b_hat with b_true(w) (red).
%   Merges the pf run (bseed0, bseed0_pf) with the forgetting run (btseed0 ceiling). Usage: plot_pf_walls('ramp') / ('plane').
%   Output pf_<wall>.png | EXPIRES: with the unknown-wall line | 產線改動不會自動跟上
function plot_pf_walls(wall)
    if nargin < 1; wall = 'ramp'; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    T1 = 'plane-ramp_bseed0-bseed0_pf'; T2 = 'plane-ramp_bseed0-bseed0_lf999-bseed0_lf9999-bseed0_lfa-btseed0';
    ARMS = {'bseed0', [0.35 0.35 0.35], '-', 'constant \^b (current)'; 'bseed0_pf', [0 0.55 0.2], '-', 'P_{55} floor at the prior (directional forgetting)'; 'btseed0', [0 0.2 0.9], '--', 'b_{true}(w) fed (ceiling)'};
    NM = {'meng', 'canon'}; FS = 13; LFS = 10; AXLW = 1.5;
    f = figure('Units','inches','Position',[0 0 14 8.5], 'Color','w', 'Visible','off'); tiledlayout(2, 2, 'TileSpacing','compact', 'Padding','compact');
    for row = 1:2
        for c = 1:2
            A = load(fullfile(od, sprintf('three_walls_%s_%s.mat', NM{c}, T1))); B = load(fullfile(od, sprintf('three_walls_%s_%s.mat', NM{c}, T2)), sprintf('%s_btseed0', wall));
            S = A; S.(sprintf('%s_btseed0', wall)) = B.(sprintf('%s_btseed0', wall));
            nexttile(2*(row-1) + c); hold on; H = []; L = {};
            t = S.(sprintf('%s_bseed0', wall)).t(:); T_END = ceil(t(end));
            if row == 2; bt = mean(S.(sprintf('%s_btseed0', wall)).B, 2); H(end+1) = plot(t, bt, '-', 'Color', [0.8 0 0], 'LineWidth', 2.2); L{end+1} = 'b_{true}(w)  at the true height'; end
            for ia = 1:size(ARMS, 1)
                d = S.(sprintf('%s_%s', wall, ARMS{ia,1}));
                if row == 1; y = mean(d.E, 2); else; if strcmp(ARMS{ia,1}, 'btseed0'); continue; end; y = mean(d.B, 2); end
                H(end+1) = plot(t, y, ARMS{ia,3}, 'Color', ARMS{ia,2}, 'LineWidth', 1.6); L{end+1} = ARMS{ia,4};
            end
            if row == 1; yline(0, '-', 'Color', [0.6 0.6 0.6], 'HandleVisibility','off'); if strcmp(wall,'ramp'); ylim([-0.06 0.06]); else; ylim([-0.03 0.03]); end; if c == 1; ylabel('(\^a_z - a_z)/a_{nom}   seed mean', 'FontSize', FS, 'FontWeight','bold'); end
            else; if strcmp(wall,'ramp'); ylim([0.8 1.25]); else; ylim([0.82 0.96]); end; if c == 1; ylabel('b', 'FontSize', FS, 'FontWeight','bold'); end; xlabel('time [s]', 'FontSize', FS, 'FontWeight','bold'); end
            ph = S.phases; xline(ph(2), ':', 'Color', [0.3 0.3 0.3]); xline(ph(3), '--', 'Color', [0.3 0.3 0.3]);
            if row == 1; title(sprintf('plant = %s wall, %s', wall, NM{c}), 'FontSize', FS, 'FontWeight','bold'); end
            legend(H, L, 'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on','NumColumns',2);
            set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off; xlim([0 T_END]);
        end
    end
    png = fullfile(od, sprintf('pf_%s.png', wall)); exportgraphics(f, png, 'Resolution', 150); close(f); fprintf('saved %s\n', png);
end
