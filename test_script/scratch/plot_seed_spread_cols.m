% STATUS: ACTIVE (scratch figure helper) | PURPOSE: the three-row seed-spread
%   page used by run_traj_seed_spread, for ANY list of run sets:
%     row 1  a_hat - a_true, every seed thin, seed mean thick
%     row 2  sd_seeds(a_hat) vs mean sqrt(P44)          (honesty)
%     row 3  the trajectory: w_d (command, red) and w true of seed 7 (blue), h/R
%   O = {run_formC_b output, ...}, NAMES = column titles, W = {[t1 t2; ...]}
%   analysis windows per column (rows: descent, motion, hold), WN window names.
function D = plot_seed_spread_cols(O, NAMES, fig, W, WN)
    ax = 3; nC = numel(O); nS = numel(O{1}.runs); D = cell(1, nC);
    for c = 1:nC
        t = O{c}.runs{1}.tout(:); N = numel(t);
        E = zeros(N, nS); AH = E; P44 = E; L1 = E; AT = E;
        for q = 1:nS
            r = O{c}.runs{q}; ad = r.a_hat_out(1,ax)/r.a_bar_hat_out(1,ax);
            AT(:,q) = r.a_true_out(:,ax)/ad; AH(:,q) = r.a_bar_hat_out(:,ax);
            E(:,q) = AH(:,q) - AT(:,q); P44(:,q) = r.P_a_out(:,ax); L1(:,q) = r.K_a_y1_out(:,ax);
        end
        D{c} = struct('t', t, 'E', E, 'at', mean(AT,2), 'sd', std(AH,0,2), 'sP', sqrt(mean(P44,2)), 'l41', mean(L1,2), 'name', NAMES{c});
    end
    fprintf('\n%d seeds, z\n%-30s %-8s %10s %10s %9s %10s %8s\n', nS, 'column', 'window', 'mean err', 'sd seeds', 'honesty', 'mean l41', 'frac<0');
    for c = 1:nC
        d = D{c}; t = d.t; Wc = W{c};
        for w = 1:size(Wc,1)
            m = t >= Wc(w,1) & t <= Wc(w,2);
            fprintf('%-30s %-8s %+10.4f %10.4f %9.2f %+10.4f %8.2f\n', d.name, WN{w}, mean(mean(d.E(m,:),2)), mean(d.sd(m)), ...
                    mean(d.sd(m)) / mean(d.sP(m)), mean(d.l41(m)), mean(d.l41(m) < 0));
        end
    end
    C_M = [0.55 0.78 1.0]; C_E = [0 0.2 0.9]; C_T = [0.8 0 0]; FS = 18; LFS = 13; AXLW = 2.0;
    f = figure('Position',[10 10 730*nC 1400],'Color','w','NumberTitle','off','Visible','off');
    tl = tiledlayout(f,3,nC,'TileSpacing','compact','Padding','compact'); A = gobjects(3,nC);
    yl1 = 0; yl2 = 0; yl3 = [0 0];
    for c = 1:nC; d = D{c}; yl1 = max(yl1, max(abs(d.E(:)))); yl2 = max(yl2, max([d.sd; d.sP])); yl3 = [min(yl3(1), min(d.l41)), max(yl3(2), max(d.l41))]; end
    for c = 1:nC
        d = D{c};
        a = nexttile(tl, c); A(1,c) = a; hold(a,'on');
        yline(a,0,'-','Color',[0.5 0.5 0.5],'LineWidth',1,'HandleVisibility','off');
        h1 = plot(a, d.t, d.E, '-', 'Color', C_M, 'LineWidth', 0.7);
        h2 = plot(a, d.t, mean(d.E,2), '-', 'Color', C_E, 'LineWidth', 2.4);
        legend(a,[h1(1) h2],{sprintf('\\^a_h - a   each seed (%d)', nS), 'seed mean'},'Location','northoutside', ...
               'Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
        title(a, d.name, 'FontSize', LFS+1, 'FontWeight', 'bold');
        if c == 1; ylabel(a,'\^a_h - a','FontSize',FS,'FontWeight','bold'); end
        ylim(a, 1.05*[-yl1 yl1]);
        a = nexttile(tl, nC+c); A(2,c) = a; hold(a,'on');
        h1 = plot(a, d.t, d.sd, '-', 'Color', C_E, 'LineWidth', 2.2);
        h2 = plot(a, d.t, d.sP, '--', 'Color', C_T, 'LineWidth', 2.2);
        legend(a,[h1 h2],{'sd_{seeds}(\^a_h)', 'mean \surdP_{44}'},'Location','northoutside', ...
               'Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
        if c == 1; ylabel(a,'spread','FontSize',FS,'FontWeight','bold'); end
        ylim(a, [0 1.05*yl2]);
        a = nexttile(tl, 2*nC+c); A(3,c) = a; hold(a,'on');
        r1 = O{c}.runs{1}; hd = r1.p_d_out(:,ax)/r1.R; ht = r1.h_bar_true_out(:,1);
        h1 = plot(a, r1.tout(:), hd, '-', 'Color', [0.8 0 0], 'LineWidth', 2.4);
        h2 = plot(a, r1.tout(:), ht, '-', 'Color', C_E, 'LineWidth', 1.0);
        legend(a,[h1 h2],{'w_d  (command)', 'w  true, seed 7'},'Location','northoutside','Orientation','horizontal', ...
               'FontSize',LFS,'FontWeight','bold','Box','on');
        if c == 1; ylabel(a,'h / R','FontSize',FS,'FontWeight','bold'); end
        ylim(a, [0 1.05*max(hd)]);
        xlabel(a,'time  [s]','FontSize',FS,'FontWeight','bold');
    end
    for k = 1:3*nC
        set(A(k),'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid(A(k),'off');
        [row, col] = ind2sub([3 nC], k); xlim(A(k), [0 D{col}.t(end)]);
        if row < 3; set(A(k),'XTickLabel',[]); end
        if col > 1; set(A(k),'YTickLabel',[]); end
    end
    exportgraphics(f, fig, 'Resolution', 150); close(f);
    fprintf('figure -> %s\n', fig);
end
