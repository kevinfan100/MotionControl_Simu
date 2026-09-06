% PURPOSE (2026-09-06): read-out of run_jac_exact_arms -- ctrl_const.jac_exact_step off (grey) vs on (blue), Meng | canon:
%   row 1  b_true arm: sigma_seed(t) of (a_hat - a)/a_nom (solid) and sqrt(P44) (dashed)   -> P1 spread + honesty
%   row 2  b_true arm: l41 + a' l31, seed mean                                            -> P1 cancellation condition
%   row 3  lockb (b = 8/9) and prod (b_hat estimated): seed-mean (a_hat - a)/a_nom, off vs on -> P2 / P3 no collapse
%   Output jac_exact_arms.png | EXPIRES: with jac_exact_step | 產線改動不會自動跟上
function plot_jac_exact_arms()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    TR = {'meng', 'canon'}; NM = {'Meng', 'canon'};
    COFF = [0.45 0.45 0.45]; CON = [0 0.2 0.9]; CR = [0.8 0 0]; CL = [0.85 0.33 0.10];
    FS = 14; LFS = 10; AXLW = 1.6;
    f = figure('Units','inches','Position',[0 0 13 11], 'Color','w', 'Visible','off'); tiledlayout(3, 2, 'TileSpacing','compact', 'Padding','compact');
    for a = 1:2
        S = load(fullfile(od, sprintf('jac_exact_arms_%s.mat', TR{a})));  t = S.btrue_off.t(:); T_END = ceil(t(end));
        nexttile(a); hold on;
        h1 = plot(t, std(S.btrue_off.E, 0, 2), '-', 'Color', COFF, 'LineWidth', 1.4);  plot(t, mean(S.btrue_off.sP, 2), '--', 'Color', COFF, 'LineWidth', 1.2);
        h2 = plot(t, std(S.btrue_on.E, 0, 2), '-', 'Color', CON, 'LineWidth', 1.6);   h3 = plot(t, mean(S.btrue_on.sP, 2), '--', 'Color', CR, 'LineWidth', 1.2);
        xline(S.win(1), ':', 'Color', [0.3 0.3 0.3]); xline(S.win(2), ':', 'Color', [0.3 0.3 0.3]); xline(S.t_hold, '--', 'Color', [0.3 0.3 0.3]);
        legend([h1 h2 h3], {'\sigma_{seed}  Euler Jacobian (off)', '\sigma_{seed}  exact-step Jacobian (on)', '\surd P_{44}  (on; dashed grey = off)'}, 'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
        if a == 1; ylabel('b_{true} arm:  \sigma_{seed}  (\^a_z - a_z)/a_{nom}', 'FontSize', FS, 'FontWeight','bold'); end
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off; xlim([0 T_END]); ylim([0 0.04]);
        nexttile(2 + a); hold on; yline(0, '-', 'Color', [0.5 0.5 0.5], 'HandleVisibility','off');
        h1 = plot(t, mean(S.btrue_off.E_l, 2), '-', 'Color', COFF, 'LineWidth', 1.4);  h2 = plot(t, mean(S.btrue_on.E_l, 2), '-', 'Color', CON, 'LineWidth', 1.6);
        xline(S.win(1), ':', 'Color', [0.3 0.3 0.3]); xline(S.win(2), ':', 'Color', [0.3 0.3 0.3]); xline(S.t_hold, '--', 'Color', [0.3 0.3 0.3]);
        legend([h1 h2], {'l_{41} + a''_z l_{31}   off', 'l_{41} + a''_z l_{31}   on'}, 'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
        if a == 1; ylabel('b_{true} arm:  l_{41} + a''_z l_{31}  (seed mean)', 'FontSize', FS, 'FontWeight','bold'); end
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off; xlim([0 T_END]);
        nexttile(4 + a); hold on; yline(0, '-', 'Color', [0.5 0.5 0.5], 'HandleVisibility','off');
        h1 = plot(t, mean(S.lockb_off.E, 2), '-', 'Color', COFF, 'LineWidth', 1.2);  h2 = plot(t, mean(S.lockb_on.E, 2), '-', 'Color', CL, 'LineWidth', 1.6);
        h3 = plot(t, mean(S.prod_off.E, 2), '--', 'Color', COFF, 'LineWidth', 1.2);  h4 = plot(t, mean(S.prod_on.E, 2), '--', 'Color', CON, 'LineWidth', 1.6);
        xline(S.t_hold, '--', 'Color', [0.3 0.3 0.3]);
        legend([h1 h2 h3 h4], {'b = 8/9  off', 'b = 8/9  on', '\^b estimated  off', '\^b estimated  on'}, 'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
        if a == 1; ylabel('seed mean  (\^a_z - a_z)/a_{nom}', 'FontSize', FS, 'FontWeight','bold'); end
        xlabel(sprintf('time [s]   (%s; dotted = fast window, dashed = hold start)', NM{a}), 'FontSize', FS, 'FontWeight','bold');
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off; xlim([0 T_END]);
    end
    png = fullfile(od, 'jac_exact_arms.png'); exportgraphics(f, png, 'Resolution', 150); close(f); fprintf('saved %s\n', png);
end
