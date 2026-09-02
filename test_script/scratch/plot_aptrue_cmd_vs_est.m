% FORK OF test_script/scratch/plot_aptrue_canon_lr.m (2026-09-02) | PURPOSE: 5x2 panel for
%   run_aptrue_cmd_vs_est.m -- left slope@cmd, right slope@est, both exact + pred_mean2,
%   same 10 seeds, one figure per trajectory | EXPIRES: with the parent | 產線改動不會自動跟上
function out = plot_aptrue_cmd_vs_est(traj, ylims)
    if nargin < 1 || isempty(traj);  traj = 'canon'; end
    if nargin < 2; ylims = []; end
    traj = lower(traj);
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    D0 = load(fullfile(od, sprintf('aptrue_cmd_vs_est_%s.mat', traj)));
    D = struct('left', D0.left, 'right', D0.right);
    pc = physical_constants();  R = pc.R;
    SM_WIN_S = 0.10;                       % [s] row-4 smoothing, << the 1 s period
    nsm = max(1, round(SM_WIN_S / pc.Ts));

    TAG = {'left','right'};  COLT = {sprintf('        a''_{true}  slope@cmd  (%s)', traj), sprintf('        a''_{true}  slope@est  (%s)', traj)};   % both arms: exact + pred_mean2
    S = cell(1,2);
    for a = 1:2
        d = D.(TAG{a});  t = d.t;  N = numel(t);  nS = size(d.E, 2);
        k1 = (2:N).';
        e3 = d.C.delta_x_hat_3_out(k1,:)/R - d.C.trk_true(k1-1,:);   % slot-3 error [R]
        S{a} = struct('t', t, 'tk', t(k1), 'nS', nS, ...
                      'E', d.E, 'mE', mean(d.E,2), 'sdE', std(d.E,0,2), 'sP', d.sP, ...
                      'L41', d.L41, 'L42', d.L42, ...
                      'sd3', movmean(std(e3,0,2), nsm), 'sP33', mean(d.C.P33_sqrt(k1,:),2));
        fprintf('[%s %s] N=%d seeds | end: mean E %+.5f | sd(E) %.5f vs sqrt(P44) %.5f (honesty %.2f) | sd3 %.5f vs sqrt(P33) %.5f (honesty %.2f)\n', ...
                traj, TAG{a}, nS, S{a}.mE(end), S{a}.sdE(end), S{a}.sP(end), S{a}.sdE(end)/S{a}.sP(end), ...
                S{a}.sd3(end), S{a}.sP33(end), S{a}.sd3(end)/S{a}.sP33(end));
    end

    % --- auto y-limits: robust to the few seeds that leave the frame ---
    if isempty(ylims)
        ylims = zeros(5,2);
        G = @(v) [min(quantile(v(:),0.002), 0), max(quantile(v(:),0.998), 0)];
        ylims(1,:) = G([S{1}.E(:); S{2}.E(:)]);
        ylims(2,:) = G([S{1}.L41(:); S{2}.L41(:)]);
        ylims(3,:) = G([S{1}.L42(:); S{2}.L42(:)]);
        ylims(4,:) = [0, 1.15*max([S{1}.sd3; S{2}.sd3; S{1}.sP33; S{2}.sP33])];
        ylims(5,:) = [0, 1.15*max([S{1}.sdE; S{2}.sdE; S{1}.sP; S{2}.sP])];
        for r = 1:3
            pad = 0.08*diff(ylims(r,:));  ylims(r,:) = ylims(r,:) + [-pad pad];
        end
    end

    % --- house style (figure-style.md; template plot_var_ahat_6state) ---
    COL_SEED = [0.55 0.74 0.96];  COL_MEAN = [0 0.2 0.9];  COL_P = [0.8 0 0];
    FS = 15; LFS = 11; AXLW = 1.6; T_END = [ceil(S{1}.t(end)) ceil(S{2}.t(end))];
    f = figure('Units','inches','Position',[0 0 12.74 20.46], 'Color','w', ...
               'NumberTitle','off', 'Visible','off');
    tl = tiledlayout(5, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    YL = {'(\^a_z - a_z) / a_{nom}', 'l_{41}', 'l_{42}', ...
          'spread of \delta w_3 / R', 'spread of a_z / a_{nom}'};
    for r = 1:5
        for a = 1:2
            s = S{a}; nexttile((r-1)*2 + a); hold on;
            yline(0, '-', 'Color', [0.55 0.55 0.55], 'LineWidth', 0.8, 'HandleVisibility','off');
            switch r
                case {1,2,3}
                    switch r
                        case 1; Y = s.E;   nm = '(\^a_z - a_z)/a_{nom}';
                        case 2; Y = s.L41; nm = 'l_{41}';
                        case 3; Y = s.L42; nm = 'l_{42}';
                    end
                    hs = plot(s.t, Y, '-', 'Color', COL_SEED, 'LineWidth', 0.5);
                    hm = plot(s.t, mean(Y,2), '-', 'Color', COL_MEAN, 'LineWidth', 2.0);
                    if r == 1
                        lg = {sprintf('%s   each seed (%d)', nm, s.nS), 'seed mean'};
                    else
                        lg = {sprintf('%s   each seed', nm), 'seed mean'};
                    end
                    H = [hs(1) hm];
                case 4
                    H(1) = plot(s.tk, s.sd3,  '-', 'Color', COL_MEAN, 'LineWidth', 2.0);
                    H(2) = plot(s.tk, s.sP33, '-', 'Color', COL_P,    'LineWidth', 2.0);
                    lg = {sprintf('sd((\\delta\\^w_3-\\delta w_3)/R)  seeds, %.2f s mean', SM_WIN_S), ...
                          sprintf('sqrt(P_{33})  (%d-seed mean)', s.nS)};
                case 5
                    H(1) = plot(s.t, s.sdE, '-', 'Color', COL_MEAN, 'LineWidth', 2.0);
                    H(2) = plot(s.t, s.sP,  '-', 'Color', COL_P,    'LineWidth', 2.0);
                    lg = {'sd((\^a_z-a_z)/a_{nom})  across seeds', ...
                          sprintf('sqrt(P_{44})/a_{nom}  (%d-seed mean)', s.nS)};
            end
            xlim([0 T_END(a)]); ylim(ylims(r,:));
            legend(H, lg, 'Location','northoutside', 'Orientation','horizontal', ...
                   'FontSize', LFS - 2*(r >= 4), 'FontWeight','bold', 'Box','on');
            if a == 1; ylabel(YL{r}, 'FontSize', FS, 'FontWeight','bold'); end
            if r == 5; xlabel('time  [s]', 'FontSize', FS, 'FontWeight','bold'); end
            if r == 1; title(COLT{a}, 'FontSize', FS+2, 'FontWeight','bold'); end
            set(gca, 'FontSize', FS, 'FontWeight','bold', 'LineWidth', AXLW, 'Box','on'); grid off;
        end
    end
    png = fullfile(od, sprintf('aptrue_cmd_vs_est_%s.png', traj));
    exportgraphics(f, png, 'Resolution', 150);
    close(f);
    fprintf('[%s] wrote %s\n', traj, png);
    out = struct('ylims', ylims, 'png', png, 'S', {S});
end
