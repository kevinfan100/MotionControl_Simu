% STATUS: ACTIVE (scratch) | PURPOSE: "what does one run look like, and did the
%   compensation widen the spread?" (2026-09-03, user request). House style of
%   plot_var_ahat_6state.m (true red, estimate blue, +/- sigma_seed band).
%   Left column: Euler tangent step, slope @cmd (baseline). Right: arm 4 (exact step +
%   slope@est + pred_mean2). Same seeds in both columns, y shared per row.
%     row 1  ONE run (seed 7): a_bar_true and a_bar_hat, absolute
%     row 2  10 seeds: ensemble mean a_bar_hat +/- sigma_seed band over a_bar_true
%     row 3  10 seeds: (a_hat - a)/a_nom, seed mean +/- sigma_seed band
%   Console: hold spread (sigma_seed) per arm and the ratio arm4/Euler.
%   Data: canon aptrue_canon_lr.mat (left/right); meng arms30_seedtruth_pair.mat (aptrue,
%   seeds 1:10) vs arms10_seedtruth_comp_meng.mat (aptrue).
function plot_aptrue_single_vs_seeds(traj)
    if nargin < 1 || isempty(traj); traj = 'canon'; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    switch traj
        case 'canon'
            S = load(fullfile(od, 'aptrue_canon_lr.mat')); L = S.left; Rr = S.right; t_hold = 3.5;
        case 'meng'
            A = load(fullfile(od, 'arms30_seedtruth_pair.mat')); L = A.aptrue;
            L.E = L.E(:,1:10); L.C.a_bar_hat_out = L.C.a_bar_hat_out(:,1:10); L.C.a_true_norm = L.C.a_true_norm(:,1:10);
            B = load(fullfile(od, 'arms10_seedtruth_comp_meng.mat')); Rr = B.aptrue; t_hold = 10.6;
    end
    D = {L, Rr}; NM = {'Euler, slope@cmd', 'exact + slope@est + pred\_mean2'};
    SEED = 7; COL_TRUE = [0.8 0 0]; COL_HAT = [0 0.2 0.9]; BANDC = [0.45 0.55 0.95];
    FS = 15; LFS = 11; AXLW = 1.8;
    t = D{1}.t(:); T_END = ceil(t(end)); mh = t > t_hold;
    f = figure('Units','inches','Position',[0 0 13 12], 'Color','w', 'Visible','off');
    tiledlayout(3, 2, 'TileSpacing','compact', 'Padding','compact');
    ax = gobjects(3,2);
    for a = 1:2
        d = D{a}; AH = d.C.a_bar_hat_out; AT = d.C.a_true_norm; E = d.E; nS = size(E,2);
        % row 1: one run
        ax(1,a) = nexttile(a); hold on;
        ht = plot(t, AT(:,SEED), '-', 'Color', COL_TRUE, 'LineWidth', 2.0);
        hh = plot(t, AH(:,SEED), '-', 'Color', COL_HAT,  'LineWidth', 1.2);
        legend([hh ht], {sprintf('\\^a_z / a_{nom}   seed %d', SEED), 'a_z / a_{nom}  true'}, ...
               'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
        if a == 1; ylabel('a_z / a_{nom}', 'FontSize', FS, 'FontWeight','bold'); end
        text(0.03, 0.08, NM{a}, 'Units','normalized', 'FontSize', FS-2, 'FontWeight','bold');
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off; xlim([0 T_END]);
        % row 2: ensemble mean +/- sigma band, absolute
        ax(2,a) = nexttile(2+a); hold on;
        m = mean(AH,2); s = std(AH,0,2);
        hb = fill([t; flipud(t)], [m+s; flipud(m-s)], BANDC, 'FaceAlpha', 0.30, 'EdgeColor','none');
        ht = plot(t, mean(AT,2), '-', 'Color', COL_TRUE, 'LineWidth', 2.0);
        hh = plot(t, m, '-', 'Color', COL_HAT, 'LineWidth', 1.5);
        legend([hh ht hb], {sprintf('mean \\^a_z / a_{nom}  (%d seeds)', nS), 'a_z / a_{nom}  true', '\pm \sigma_{seed}'}, ...
               'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
        if a == 1; ylabel('a_z / a_{nom}', 'FontSize', FS, 'FontWeight','bold'); end
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off; xlim([0 T_END]);
        % row 3: error, bias +/- spread band
        ax(3,a) = nexttile(4+a); hold on;
        mE = mean(E,2); sE = std(E,0,2);
        hb = fill([t; flipud(t)], [mE+sE; flipud(mE-sE)], BANDC, 'FaceAlpha', 0.30, 'EdgeColor','none');
        yline(0, '-', 'Color', [0.5 0.5 0.5], 'LineWidth', 0.8, 'HandleVisibility','off');
        h1 = plot(t, E(:,SEED), '-', 'Color', [0.55 0.74 0.96], 'LineWidth', 0.6);
        hm = plot(t, mE, '-', 'Color', COL_HAT, 'LineWidth', 2.0);
        legend([hm hb h1], {'seed mean (\^a_z - a_z)/a_{nom}', '\pm \sigma_{seed}', sprintf('seed %d', SEED)}, ...
               'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
        if a == 1; ylabel('(\^a_z - a_z) / a_{nom}', 'FontSize', FS, 'FontWeight','bold'); end
        xlabel('time  [s]', 'FontSize', FS, 'FontWeight','bold');
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off; xlim([0 T_END]);
        fprintf('[%s %-32s] hold: bias %+.5f | spread sigma_seed %.5f | seed %d hold rms %.5f | max sigma_seed over run %.5f\n', ...
            traj, NM{a}, mean(E(mh,:),'all'), mean(sE(mh)), SEED, rms(E(mh,SEED)), max(sE));
        SP(a) = mean(sE(mh)); SPmax(a) = max(sE);
    end
    fprintf('[%s] spread ratio arm4/Euler: hold %.2f | max over run %.2f\n', traj, SP(2)/SP(1), SPmax(2)/SPmax(1));
    linkaxes(ax(1,:),'y'); linkaxes(ax(2,:),'y'); linkaxes(ax(3,:),'y');
    png = fullfile(od, sprintf('aptrue_single_vs_seeds_%s.png', traj));
    exportgraphics(f, png, 'Resolution', 150); close(f); fprintf('saved %s\n', png);
end
