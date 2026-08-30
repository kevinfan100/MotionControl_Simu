% STATUS: ACTIVE (scratch figure helper) | PURPOSE: the four-arm page with the
%   feedback gains instead of spread/trajectory (2026-08-28, user request):
%     row 1  a_hat - a_true, every seed thin, seed mean thick
%     row 2  l_31  y1 -> current tracking-error slot   (K_dx_y1_out)
%     row 3  l_32  y2 -> current tracking-error slot   (K_dx_y2_out)
%     row 4  l_41  y1 -> a_hat                         (K_a_y1_out)
%     row 5  l_42  y2 -> a_hat                         (K_a_y2_out)
%     row 6  trajectory: w_d (command, red), w true seed 7 (blue), h/R
%   Gains are seed means; y shared across columns per row; zero line drawn.
%   Paper notation (Meng TIE 2025 eq 16): l_ij, i = state row, j = measurement.
function D = plot_seed_gain_cols(O, NAMES, fig, W, WN, tlim)
    if nargin < 6 || isempty(tlim); tlim = []; end   % [] = whole run; [t1 t2] = window shown (stats still use W); cell = per column
    ax = 3; nC = numel(O); nS = numel(O{1}.runs); D = cell(1, nC);
    for c = 1:nC
        t = O{c}.runs{1}.tout(:); N = numel(t); E = zeros(N, nS); L = zeros(N, nS, 4);
        for q = 1:nS
            r = O{c}.runs{q}; ad = r.a_hat_out(1,ax)/r.a_bar_hat_out(1,ax);
            E(:,q) = r.a_bar_hat_out(:,ax) - r.a_true_out(:,ax)/ad;
            L(:,q,1) = r.K_dx_y1_out(:,ax); L(:,q,2) = r.K_dx_y2_out(:,ax); L(:,q,3) = r.K_a_y1_out(:,ax); L(:,q,4) = r.K_a_y2_out(:,ax);
        end
        D{c} = struct('t', t, 'E', E, 'L', squeeze(mean(L, 2)), 'Lall', L, 'name', NAMES{c});
    end
    GN = {'{\itl}_{31}', '{\itl}_{32}', '{\itl}_{41}', '{\itl}_{42}'}; GY = {'l_{31}', 'l_{32}', 'l_{41}', 'l_{42}'};
    fprintf('\n%d seeds, z\n%-40s %-9s %9s %9s | %9s %9s %9s %9s\n', nS, 'column', 'window', 'mean err', 'sd seeds', 'l31', 'l32', 'l41', 'l42');
    for c = 1:nC
        d = D{c}; t = d.t; Wc = W{c};
        for w = 1:size(Wc,1)
            m = t >= Wc(w,1) & t <= Wc(w,2);
            fprintf('%-40s %-9s %+9.4f %9.4f | %+9.4f %+9.4f %+9.4f %+9.4f\n', d.name, WN{w}, mean(mean(d.E(m,:),2)), mean(std(d.E(m,:),0,2)), mean(d.L(m,:),1));
        end
    end
    C_M = [0.55 0.78 1.0]; C_E = [0 0.2 0.9]; FS = 16; LFS = 12; AXLW = 1.8;
    f = figure('Position',[10 10 600*nC 2000],'Color','w','NumberTitle','off','Visible','off');
    tl = tiledlayout(f,6,nC,'TileSpacing','compact','Padding','compact'); A = gobjects(6,nC);
    if isempty(tlim); tlim = [0, max(cellfun(@(d) d.t(end), D))]; end
    if ~iscell(tlim); tlim = repmat({tlim}, 1, nC); end
    yl1 = 0; for c = 1:nC; m = D{c}.t >= tlim{c}(1) & D{c}.t <= tlim{c}(2); yl1 = max(yl1, max(abs(D{c}.E(m,:)), [], 'all')); end
    ylg = zeros(4,2); for g = 1:4; for c = 1:nC; Lg = D{c}.Lall(:,:,g); ylg(g,:) = [min(ylg(g,1), min(Lg(:))), max(ylg(g,2), max(Lg(:)))]; end; end
    for c = 1:nC
        d = D{c};
        a = nexttile(tl, c); A(1,c) = a; hold(a,'on');
        yline(a,0,'-','Color',[0.5 0.5 0.5],'LineWidth',1,'HandleVisibility','off');
        h1 = plot(a, d.t, d.E, '-', 'Color', C_M, 'LineWidth', 0.6); h2 = plot(a, d.t, mean(d.E,2), '-', 'Color', C_E, 'LineWidth', 2.2);
        legend(a,[h1(1) h2],{sprintf('\\^a_h - a  each seed (%d)', nS), 'seed mean'},'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
        title(a, d.name, 'FontSize', LFS+1, 'FontWeight', 'bold');
        if c == 1; ylabel(a,'\^a_h - a','FontSize',FS,'FontWeight','bold'); end
        ylim(a, 1.05*[-yl1 yl1]);
        for g = 1:4
            a = nexttile(tl, g*nC + c); A(g+1,c) = a; hold(a,'on');
            yline(a,0,'-','Color',[0.5 0.5 0.5],'LineWidth',1,'HandleVisibility','off');
            hs = plot(a, d.t, squeeze(d.Lall(:,:,g)), '-', 'Color', C_M, 'LineWidth', 0.6);
            h = plot(a, d.t, d.L(:,g), '-', 'Color', C_E, 'LineWidth', 2.0);
            legend(a,[hs(1) h],{[GN{g} '   each seed'], 'seed mean'},'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
            if c == 1; ylabel(a, GY{g}, 'FontSize', FS, 'FontWeight', 'bold'); end
            pad = 0.05*(ylg(g,2)-ylg(g,1)+eps); ylim(a, [ylg(g,1)-pad, ylg(g,2)+pad]);
        end
        a = nexttile(tl, 5*nC + c); A(6,c) = a; hold(a,'on');
        r1 = O{c}.runs{1}; hd = r1.p_d_out(:,ax)/r1.R; ht = r1.h_bar_true_out(:,1);
        h1 = plot(a, r1.tout(:), hd, '-', 'Color', [0.8 0 0], 'LineWidth', 2.2); h2 = plot(a, r1.tout(:), ht, '-', 'Color', C_E, 'LineWidth', 0.9);
        legend(a,[h1 h2],{'w_d  (command)','w  true, seed 7'},'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
        if c == 1; ylabel(a,'h / R','FontSize',FS,'FontWeight','bold'); end
        ylim(a, [0 1.05*max(hd)]); xlabel(a,'time  [s]','FontSize',FS,'FontWeight','bold');
    end
    for k = 1:6*nC
        set(A(k),'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid(A(k),'off');
        [row, col] = ind2sub([6 nC], k); xlim(A(k), tlim{col});
        if row < 6; set(A(k),'XTickLabel',[]); end
        if col > 1; set(A(k),'YTickLabel',[]); end
    end
    exportgraphics(f, fig, 'Resolution', 150); close(f);
    fprintf('figure -> %s\n', fig);
end
