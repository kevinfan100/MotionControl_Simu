% PURPOSE (2026-09-06): where and why the cross-seed spread of the b_true arm's gain error is large -- reads existing mats only
%   (no new runs): btrue_aa_scale_<traj>.mat (k100 = kappa 1, k050; E, sqrt(P44), l41 + a' l31), ladder_endpoints_<traj>.mat
%   (apest = a'_true@est arm), probe_btrue_descent_dip_<traj>.mat (30 seeds, per-step predict / y1 / y2 legs, both arms).
%   Figure: Meng | canon, three rows
%     row 1  sigma_seed(t) of (a_hat - a)/a_nom: b_true kappa 1 (blue), b_true kappa 0.5 (blue dashed), a'_true@est (green),
%            sqrt(P44) of the kappa 1 arm (red dashed)  -> spread lives in the fast near-wall segment, not in hold
%     row 2  per-step anatomy (30 seeds, sd over seeds, 0.1 s moving mean): law predict increment (purple), y1 leg (green),
%            their sum (black); b_true solid, a'_true dashed  -> the law injects, y1 cancels only in the a'_true arm
%     row 3  the injection gain a'_true = d(a/a_nom)/d(w/R) at the true height (red) and w/R (grey, right axis)
%   Console: per-segment sd table and a one-step table at the worst instant. Output btrue_spread_anatomy.png
%   | EXPIRES: with the b_true rung | 產線改動不會自動跟上
function plot_btrue_spread_anatomy()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    TR = {'meng', 'canon'};  NM = {'Meng', 'canon'};
    WIN = {[7 10], [1.0 1.5]};  TW = [9.24 1.41];
    SEG = {{'far 0.5-5',[0.5 5]; 'mid 5-7',[5 7]; 'near 7-10.5',[7 10.5]; 'hold 10.5-12.5',[10.5 12.5]}, ...
           {'hold 0-0.6',[0 0.6]; 'desc 0.6-1.0',[0.6 1.0]; 'fast 1.0-1.5',[1.0 1.5]; 'osc 1.5-3.5',[1.5 3.5]; 'hold 3.5-4.8',[3.5 4.8]}};
    wg = linspace(1.0, 8, 40000);  ag = zeros(size(wg));
    for i = 1:numel(wg); [~, c] = calc_correction_functions(wg(i)); ag(i) = 1/c; end
    apg = gradient(ag, wg);
    CB = [0 0.2 0.9]; CG = [0.2 0.6 0.2]; CR = [0.8 0 0]; CP = [0.49 0.18 0.56]; CK = [0 0 0]; CGR = [0.5 0.5 0.5];
    FS = 14; LFS = 10; AXLW = 1.6;
    f = figure('Units','inches','Position',[0 0 13 11], 'Color','w', 'Visible','off');
    tiledlayout(3, 2, 'TileSpacing','compact', 'Padding','compact');
    for a = 1:2
        K = load(fullfile(od, sprintf('btrue_aa_scale_%s.mat', TR{a})));
        L = load(fullfile(od, sprintf('ladder_endpoints_%s.mat', TR{a})));
        D = load(fullfile(od, sprintf('probe_btrue_descent_dip_%s.mat', TR{a})));
        t = K.k100.t(:);  T_END = ceil(t(end));  Ts = t(2) - t(1);  nw = round(0.1 / Ts);
        s1 = std(K.k100.E, 0, 2);  s05 = std(K.k050.E, 0, 2);  sA = std(L.apest.E, 0, 2);  sP = mean(K.k100.sP, 2);
        hb = mean(K.k100.HB, 2);  ap = interp1(wg, apg, min(max(hb, wg(1)), wg(end)));
        % row 1
        nexttile(a); hold on;
        h1 = plot(t, s1, '-', 'Color', CB, 'LineWidth', 1.8);
        h2 = plot(t, s05, '--', 'Color', CB, 'LineWidth', 1.2);
        h3 = plot(t, sA, '-', 'Color', CG, 'LineWidth', 1.4);
        h4 = plot(t, sP, '--', 'Color', CR, 'LineWidth', 1.4);
        xline(WIN{a}(1), ':', 'Color', [0.3 0.3 0.3]); xline(WIN{a}(2), ':', 'Color', [0.3 0.3 0.3]); xline(K.t_hold, '--', 'Color', [0.3 0.3 0.3]);
        legend([h1 h2 h3 h4], {'b_{true}  \kappa=1', 'b_{true}  \kappa=0.5', 'a''_{true}@est', '\surd P_{44}  \kappa=1'}, ...
               'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
        if a == 1; ylabel('\sigma_{seed}  (\^a_z - a_z)/a_{nom}', 'FontSize', FS, 'FontWeight','bold'); end
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off; xlim([0 T_END]); ylim([0 0.04]);
        % row 2: cumulative anatomy (30 seeds): E = sum(law predict - truth) + sum(y1 leg) + sum(y2 leg); sd over seeds of each running sum
        nexttile(2 + a); hold on;
        td = D.btest.t(:);  H = [];
        for ia = 1:2
            if ia == 1; d = D.btest; ls = '-'; else; d = D.apest; ls = '--'; end
            cP = cumsum(d.pred - d.tru, 1);  cY1 = cumsum(d.y1, 1);  cY2 = cumsum(d.y2, 1);
            H(ia,1) = plot(td, std(cP, 0, 2), ls, 'Color', CP, 'LineWidth', 1.4);
            H(ia,2) = plot(td, std(cY1, 0, 2), ls, 'Color', CG, 'LineWidth', 1.4);
            H(ia,3) = plot(td, std(cP + cY1 + cY2, 0, 2), ls, 'Color', CK, 'LineWidth', 1.6);
        end
        xline(WIN{a}(1), ':', 'Color', [0.3 0.3 0.3]); xline(WIN{a}(2), ':', 'Color', [0.3 0.3 0.3]); xline(K.t_hold, '--', 'Color', [0.3 0.3 0.3]);
        legend([H(1,1) H(1,2) H(1,3) H(2,3)], {'\Sigma law predict - truth', '\Sigma y_1 leg', '\Sigma all = \^a_z - a_z', 'a''_{true}@est (dashed)'}, ...
               'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
        if a == 1; ylabel('\sigma_{seed} of running sums', 'FontSize', FS, 'FontWeight','bold'); end
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off; xlim([0 T_END]);
        % row 3: injection gain and height
        nexttile(4 + a); hold on;
        yyaxis left;  h5 = plot(t, ap, '-', 'Color', CR, 'LineWidth', 1.8);  set(gca, 'YColor', CR);
        if a == 1; ylabel('a''_z  true', 'FontSize', FS, 'FontWeight','bold'); end
        yyaxis right; h6 = plot(t, hb, '-', 'Color', CGR, 'LineWidth', 1.4);  set(gca, 'YColor', CGR);
        if a == 2; ylabel('w / R  true', 'FontSize', FS, 'FontWeight','bold'); end
        xline(WIN{a}(1), ':', 'Color', [0.3 0.3 0.3]); xline(WIN{a}(2), ':', 'Color', [0.3 0.3 0.3]); xline(K.t_hold, '--', 'Color', [0.3 0.3 0.3]);
        legend([h5 h6], {'a''_z  at the true height', 'w / R'}, 'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
        xlabel(sprintf('time [s]   (%s; dotted = fast window, dashed = hold start)', NM{a}), 'FontSize', FS, 'FontWeight','bold');
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off; xlim([0 T_END]);
        % console: per-segment table
        fprintf('\n[%s] sigma_seed of (a_hat - a)/a_nom by segment (10 seeds) | b_true k=1 | b_true k=0.5 | a''_true@est | sqrtP44 k=1 | honesty k=1 | mean a''_true | mean w/R\n', NM{a});
        S = SEG{a};
        for s = 1:size(S,1)
            m = t >= S{s,2}(1) & t <= S{s,2}(2);
            fprintf('  %-16s %.4f  %.4f  %.4f  %.4f  %.2f  %.3f  %.2f\n', S{s,1}, mean(s1(m)), mean(s05(m)), mean(sA(m)), mean(sP(m)), mean(s1(m))/mean(sP(m)), mean(ap(m)), mean(hb(m)));
        end
        % window table (30 seeds): per-seed sums over the fast window of the three legs
        m = td >= WIN{a}(1) & td <= WIN{a}(2);  iw = find(td >= TW(a), 1);
        fprintf('[%s] fast window %.1f-%.1f s (30 seeds): sd over seeds of the per-seed window SUM of each leg of (a_hat - a)/a_nom\n', NM{a}, WIN{a});
        for ia = 1:2
            if ia == 1; d = D.btest; nm = 'b_true'; else; d = D.apest; nm = 'a''_true@est'; end
            pr = sum(d.pred(m,:) - d.tru(m,:), 1);  y1 = sum(d.y1(m,:), 1);  y2 = sum(d.y2(m,:), 1);
            fprintf('  %-12s law predict %.4f | y1 leg %.4f | corr(predict, y1) %+.2f | predict + y1 %.4f | y2 leg %.4f | all three %.4f | sd(E) at window end %.4f\n', nm, ...
                std(pr), std(y1), corr(pr(:), y1(:)), std(pr + y1), std(y2), std(pr + y1 + y2), std(d.E(find(td >= WIN{a}(2),1),:)));
            % per-step cross-correlation at lags 0..3 (y1 leg lags the kick by the measurement delay)
            pk = d.pred(m,:) - d.tru(m,:);  yk = d.y1(m,:);  pk = pk - mean(pk,2);  yk = yk - mean(yk,2);  cl = zeros(1,4);
            for lag = 0:3; A1 = pk(1:end-lag,:); B1 = yk(1+lag:end,:); cl(lag+1) = corr(A1(:), B1(:)); end
            fprintf('  %-12s per-step corr(predict[k], y1[k+lag]) lag 0..3: %+.2f %+.2f %+.2f %+.2f\n', nm, cl);
        end
        fprintf('  a''_true at t=%.2f: %.3f | w/R %.2f | l41 + a'' l31 (k=1) %+.3f, (k=0.5) %+.3f\n', td(iw), ap(find(t >= TW(a),1)), hb(find(t >= TW(a),1)), ...
            mean(K.k100.E_l(t >= WIN{a}(1) & t <= WIN{a}(2), :), 'all'), mean(K.k050.E_l(t >= WIN{a}(1) & t <= WIN{a}(2), :), 'all'));
    end
    png = fullfile(od, 'btrue_spread_anatomy.png'); exportgraphics(f, png, 'Resolution', 150); close(f); fprintf('saved %s\n', png);
    % second figure: the same spread plotted against HEIGHT during the descent only (Meng solid, canon dashed) -- discriminator
    % between "Meng is the same mechanism stretched in time" (curves overlap vs w/R) and "Meng is intrinsically stronger" (they do not)
    f = figure('Units','inches','Position',[0 0 12 4.6], 'Color','w', 'Visible','off'); tiledlayout(1, 2, 'TileSpacing','compact', 'Padding','compact');
    LS = {'-', '--'};  H1 = []; H2 = [];
    for a = 1:2
        K = load(fullfile(od, sprintf('btrue_aa_scale_%s.mat', TR{a})));
        t = K.k100.t(:);  hd = K.k100.hd(:);  v = [0; diff(hd)];                                  % descent = first descending step .. first arrival at the bottom
        i0 = find(v < -1e-7, 1);  i1 = find(hd <= min(hd) + 1e-9, 1);  desc = false(size(t));  desc(i0:i1) = true;
        hb = mean(K.k100.HB, 2);  x = hb(desc);
        nexttile(1); hold on;
        H1(a,1) = plot(x, std(K.k100.E(desc,:), 0, 2), LS{a}, 'Color', CB, 'LineWidth', 1.6);
        H1(a,2) = plot(x, std(K.k050.E(desc,:), 0, 2), LS{a}, 'Color', [0.55 0.65 0.95], 'LineWidth', 1.2);
        nexttile(2); hold on;
        H2(a,1) = plot(x, mean(K.k100.sP(desc,:), 2), LS{a}, 'Color', CR, 'LineWidth', 1.6);
        H2(a,2) = plot(x, mean(K.k050.sP(desc,:), 2), LS{a}, 'Color', [0.95 0.55 0.55], 'LineWidth', 1.2);
        fprintf('[%s] descent only: %d steps, w/R %.2f -> %.2f, %.2f s; sigma_seed at w/R = 2.0 / 1.5 / 1.2: k=1 %.4f %.4f %.4f | k=0.5 %.4f %.4f %.4f\n', NM{a}, sum(desc), x(1), x(end), sum(desc)*(t(2)-t(1)), ...
            interp1(x, std(K.k100.E(desc,:),0,2), [2.0 1.5 1.2]), interp1(x, std(K.k050.E(desc,:),0,2), [2.0 1.5 1.2]));
    end
    nexttile(1); set(gca, 'XDir', 'reverse', 'XScale', 'log'); xlim([1.05 8]); ylim([0 0.04]);
    legend([H1(1,1) H1(1,2) H1(2,1) H1(2,2)], {'Meng  \kappa=1', 'Meng  \kappa=0.5', 'canon  \kappa=1', 'canon  \kappa=0.5'}, 'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
    ylabel('\sigma_{seed}  (\^a_z - a_z)/a_{nom}   descent only', 'FontSize', FS, 'FontWeight','bold'); xlabel('w / R  true  (wall to the right)', 'FontSize', FS, 'FontWeight','bold');
    set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off;
    nexttile(2); set(gca, 'XDir', 'reverse', 'XScale', 'log'); xlim([1.05 8]); ylim([0 0.04]);
    legend([H2(1,1) H2(1,2) H2(2,1) H2(2,2)], {'Meng  \kappa=1', 'Meng  \kappa=0.5', 'canon  \kappa=1', 'canon  \kappa=0.5'}, 'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
    ylabel('\surd P_{44}   descent only', 'FontSize', FS, 'FontWeight','bold'); xlabel('w / R  true  (wall to the right)', 'FontSize', FS, 'FontWeight','bold');
    set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off;
    png = fullfile(od, 'btrue_spread_vs_height.png'); exportgraphics(f, png, 'Resolution', 150); close(f); fprintf('saved %s\n', png);
end
