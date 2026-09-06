% PURPOSE (2026-09-06): read-out of run_btrue_speed_swap -- sigma_seed (top) and sqrt(P44) (bottom) of the b_true arm's gain error
%   against the TRUE height during the descent only, kappa = 0.5 (left) and kappa = 1 (right); the standard Meng / canon arms come
%   from btrue_aa_scale_<traj>.mat, the swapped-speed arms from btrue_speed_swap_<arm>.mat. Legend carries the band approach speed.
%   Console: sigma_seed and sqrt(P44) at w/R = 2.0 / 1.5 / 1.2 per arm and kappa. Output btrue_speed_swap.png
%   | EXPIRES: with the b_true rung | 產線改動不會自動跟上
function plot_btrue_speed_swap()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    ARMS = {'Meng 15um 10s', 'btrue_aa_scale_meng.mat',        [0 0.2 0.9],      '-';  ...
            'Meng 15um 1s',  'btrue_speed_swap_mengfast.mat',  [0.55 0.65 0.95], '-';  ...
            'Meng 50um 10s', 'btrue_speed_swap_menghigh.mat',  [0.1 0.6 0.9],    ':';  ...
            'canon 50um 1s', 'btrue_aa_scale_canon.mat',       [0 0 0],          '--'; ...
            'canon 50um 10s','btrue_speed_swap_canonslow.mat', [0.45 0.45 0.45], '--'; ...
            'canon 50um 38s','btrue_speed_swap_canonvslow.mat',[0.72 0.72 0.72], '--'; ...
            'canon 15um 1s', 'btrue_speed_swap_canonlow.mat',  [0.8 0 0],        ':'; ...
            'Meng 15um P0=canon', 'btrue_speed_swap_mengp0a.mat', [0.9 0.5 0.1], '-'; ...
            'Meng 15um P0=canon@6.7R', 'btrue_speed_swap_mengp0b.mat', [0.6 0.3 0.0], '-'};
    KEYS = {'k050', 'k100'};  KLAB = {'\kappa = 0.5', '\kappa = 1'};
    FS = 14; LFS = 9; AXLW = 1.6;
    f = figure('Units','inches','Position',[0 0 15 9.5], 'Color','w', 'Visible','off'); tiledlayout(2, 2, 'TileSpacing','compact', 'Padding','compact');
    fprintf('%-12s %-8s | speed R/s | sigma_seed @ w/R 2.0 / 1.5 / 1.2 | sqrtP44 @ 2.0 / 1.5 / 1.2 | hold sd\n', 'arm', 'kappa');
    for ik = 1:2
        H = []; LB = {};
        for ia = 1:size(ARMS, 1)
            fn = fullfile(od, ARMS{ia, 2});  if ~exist(fn, 'file'); continue; end
            S = load(fn);  if ~isfield(S, KEYS{ik}); continue; end
            d = S.(KEYS{ik});  t = d.t(:);  hd = d.hd(:);  Ts = t(2) - t(1);  v = [0; diff(hd)];
            i0 = find(v < -1e-7, 1);  i1 = find(hd <= min(hd) + 1e-9, 1);
            x = mean(d.HB(i0:i1, :), 2);  s = std(d.E(i0:i1, :), 0, 2);  sp = mean(d.sP(i0:i1, :), 2);
            band = x < 2.5;  spd = mean(abs(v(i0:i1)))/Ts;  spd_b = mean(abs(v(i0 - 1 + find(band))))/Ts;
            [xu, iu] = unique(x);  lab = sprintf('%s  %.1f R/s', ARMS{ia, 1}, spd_b);
            nexttile(ik); hold on;  H(end+1) = plot(x, s, ARMS{ia, 4}, 'Color', ARMS{ia, 3}, 'LineWidth', 1.5);  LB{end+1} = lab;
            nexttile(2 + ik); hold on;  plot(x, sp, ARMS{ia, 4}, 'Color', ARMS{ia, 3}, 'LineWidth', 1.5);
            fprintf('%-12s %-8s | %5.2f (%5.2f mean) | %.4f %.4f %.4f | %.4f %.4f %.4f | %.4f\n', ARMS{ia, 1}, KEYS{ik}, spd_b, spd, ...
                interp1(xu, s(iu), [2.0 1.5 1.2]), interp1(xu, sp(iu), [2.0 1.5 1.2]), mean(std(d.E(t > S.t_hold, :), 0, 2)));
        end
        nexttile(ik); set(gca, 'XDir', 'reverse', 'XScale', 'log'); xlim([1.05 8]); ylim([0 0.04]);
        legend(H, LB, 'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on','NumColumns',4);
        if ik == 1; ylabel('\sigma_{seed}  (\^a_z - a_z)/a_{nom}   descent only', 'FontSize', FS, 'FontWeight','bold'); end
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off;
        nexttile(2 + ik); set(gca, 'XDir', 'reverse', 'XScale', 'log'); xlim([1.05 8]); ylim([0 0.04]);
        if ik == 1; ylabel('\surd P_{44}   descent only', 'FontSize', FS, 'FontWeight','bold'); end
        xlabel(sprintf('w / R  true  (wall to the right)    %s', KLAB{ik}), 'FontSize', FS, 'FontWeight','bold');
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off;
    end
    png = fullfile(od, 'btrue_speed_swap.png'); exportgraphics(f, png, 'Resolution', 150); close(f); fprintf('saved %s\n', png);
end
