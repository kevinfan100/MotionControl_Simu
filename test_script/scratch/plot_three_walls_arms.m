% FORK OF test_script/scratch/plot_three_walls.m (2026-09-08) | PURPOSE: the same three-row figure for an ARBITRARY arm set, merging
%   several three_walls_<traj>_<tag>.mat files (tags = cell of the '<walls>_<arms>' suffixes; '' = the untagged 09-07 file).
%   Rows: (1) (a_hat - a)/a_nom seed mean +- sigma_seed per arm, (2) b_hat seed mean per arm with the plant's b (red dashed),
%   (3) w_wall = w - a_hat/(b_hat(1 - a_hat)) seed mean +- sigma_seed per arm with the plant's contact height (red dashed).
%   Usage: plot_three_walls_arms('canon', {'plane-sphere-cell_prod-widew-wideb-wide15'}, {'prod','widew','wideb','wide15'}, 'split')
%   Output three_walls_<out>_<traj>.png | EXPIRES: with the unknown-wall line | 產線改動不會自動跟上
function plot_three_walls_arms(traj, tags, arms, out)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    S = struct();
    for i = 1:numel(tags)
        fn = sprintf('three_walls_%s%s.mat', traj, tags{i}); if ~isempty(tags{i}); fn = sprintf('three_walls_%s_%s.mat', traj, tags{i}); end
        T = load(fullfile(od, fn)); f = fieldnames(T); for k = 1:numel(f); S.(f{k}) = T.(f{k}); end
    end
    W = {'plane', 0.870, 1.00; 'sphere', 1.156, 1.027; 'cell', 0.877, -0.057};     % plant b (formC writing) and contact height w_s + 1/b
    COL = struct('prod', [0.45 0.45 0.45], 'widew', [0 0.2 0.9], 'wideb', [0 0.55 0.2], 'wide15', [0.85 0.33 0.10], 'wide', [0.85 0.33 0.10], 'bseed', [0 0.2 0.9], 'btseed', [0.8 0 0], 'bseed0', [0 0.2 0.9], 'btseed0', [0.8 0 0], 'lockb', [0.6 0 0.6]);
    LABS = struct('prod', 'production (Pf_b 0.039, Pf_w0 0.111 R)', 'widew', 'wall position opened (Pf_w0 1 R)', 'wideb', 'b opened (Pf_b 0.15)', 'wide15', 'both opened', 'wide', 'both opened (09-07, b_ceil 1.05)', 'bseed', 'b estimated, seeded at b_0 (start gain + contact height)', 'btseed', 'b_{true}(w) fed, same seed', 'bseed0', 'b estimated, b_0 seed, P_{44}[0] at start truth', 'btseed0', 'b_{true}(w) fed, P_{44}[0] at start truth', 'lockb', 'b locked 8/9');
    FS = 13; LFS = 9; AXLW = 1.5;  ph = S.phases;
    f = figure('Units','inches','Position',[0 0 15 10.5], 'Color','w', 'Visible','off'); tiledlayout(3, 3, 'TileSpacing','compact', 'Padding','compact');
    yE = [0 0];
    for iw = 1:3; for ia = 1:numel(arms); key = sprintf('%s_%s', W{iw,1}, arms{ia}); if ~isfield(S, key); continue; end; m = mean(S.(key).E,2); s = std(S.(key).E,0,2); yE = [min(yE(1), min(m-s)) max(yE(2), max(m+s))]; end; end
    yE = yE + 0.05*diff(yE)*[-1 1];
    for iw = 1:3
        for row = 1:3
            nexttile(3*(row-1) + iw); hold on; H = []; L = {};
            for ia = 1:numel(arms)
                key = sprintf('%s_%s', W{iw,1}, arms{ia}); if ~isfield(S, key); continue; end
                d = S.(key); t = d.t(:); c = COL.(arms{ia});
                switch row
                    case 1; m = mean(d.E, 2); s = std(d.E, 0, 2);
                            fill([t; flipud(t)], [m+s; flipud(m-s)], c, 'FaceAlpha', 0.10, 'EdgeColor','none', 'HandleVisibility','off');
                            H(end+1) = plot(t, m, '-', 'Color', c, 'LineWidth', 1.6); L{end+1} = LABS.(arms{ia});
                    case 2; H(end+1) = plot(t, mean(d.B, 2), '-', 'Color', c, 'LineWidth', 1.6); L{end+1} = LABS.(arms{ia});
                    case 3; m = mean(d.W, 2); s = std(d.W, 0, 2); ok = isfinite(m) & isfinite(s);
                            fill([t(ok); flipud(t(ok))], [m(ok)+s(ok); flipud(m(ok)-s(ok))], c, 'FaceAlpha', 0.10, 'EdgeColor','none', 'HandleVisibility','off');
                            H(end+1) = plot(t, m, '-', 'Color', c, 'LineWidth', 1.6); L{end+1} = LABS.(arms{ia});
                end
            end
            T_END = ceil(t(end));
            if row == 1; yline(0, '-', 'Color', [0.5 0.5 0.5], 'HandleVisibility','off'); ylim(yE); end
            if row == 2; hr = yline(W{iw,2}, '--', 'Color', [0.8 0 0], 'LineWidth', 1.4); H(end+1) = hr; L{end+1} = 'plant b'; ylim([0.8 1.25]); end
            if row == 3; hr = yline(W{iw,3}, '--', 'Color', [0.8 0 0], 'LineWidth', 1.4); H(end+1) = hr; L{end+1} = 'plant contact height'; ylim([-0.6 1.4]); end
            xline(ph(1), ':', 'Color', [0.3 0.3 0.3]); xline(ph(2), ':', 'Color', [0.3 0.3 0.3]); xline(ph(3), '--', 'Color', [0.3 0.3 0.3]);
            if row == 1; legend(H, L, 'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on','NumColumns',2); end
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
    png = fullfile(od, sprintf('three_walls_%s_%s.png', out, traj)); exportgraphics(f, png, 'Resolution', 150); close(f); fprintf('saved %s\n', png);
end
