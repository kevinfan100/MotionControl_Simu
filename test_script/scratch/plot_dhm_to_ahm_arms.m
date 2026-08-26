% STATUS: ACTIVE (scratch figure) | PURPOSE: the seven-step readout chain for
%   SEVERAL arms side by side, one column per arm, y limits shared down each
%   row so the arms compare by eye (project rule: never make the reader flip
%   between two pages).
%
%   ARMS is a cell array {run_struct, label; ...}. The chain itself is computed
%   by plot_dhm_to_ahm(..., 'no_figure', true), so there is exactly one copy of
%   the arithmetic and it is the one asserted bit-exact against the controller.
%
%   Column labels ride in the row-1 legend; the equations appear once, in
%   column 1, because every column runs the same six equations.
function plot_dhm_to_ahm_arms(ARMS, ax, tag, tlim)

    if nargin < 2 || isempty(ax);   ax = 3;   end
    if nargin < 3 || isempty(tag);  tag = ''; end
    if nargin < 4;                  tlim = [0.5 10.5]; end

    nA = size(ARMS, 1);
    D = cell(1, nA);
    for c = 1:nA
        D{c} = plot_dhm_to_ahm(ARMS{c,1}, ax, ...
                   struct('no_figure', true, 'tlim', tlim));
    end

    COL_TRUE = [0.8 0 0];  COL_HAT = [0 0.2 0.9];  COL_MEAS = [0.45 0.72 0.95];
    COL_LP = [0.10 0.55 0.15]; COL_SQ = [0.35 0.35 0.35];
    COL_S2 = [0.85 0.33 0.10]; COL_CN = [0.40 0.40 0.40]; COL_BG = [0.70 0.70 0.70];
    FS = 15; LFS = 11; AXLW = 1.8; LWD = 0.5;
    SH = [char(963) char(770)];
    S2MAX = 2.6e-3;
    AHMAX = 0.048;      % shared across columns, [um/pN]

    f = figure('Position', [10 10 640*nA 2000], 'Color', 'w', ...
               'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(f, 7, nA, 'TileSpacing', 'compact', 'Padding', 'compact');
    A = gobjects(7, nA);
    for c = 1:nA
        o = D{c};  i = o.i;  t = o.t;
        sq = o.dhmr(i).^2;  s2 = o.s2(i);  ahT = o.ah(i);
        ident = o.den*ahT + o.Cn_s2n;

        a = nexttile(tl, c); A(1,c) = a; hold(a,'on');
        yline(a,0,'-','Color',[0.5 0.5 0.5],'LineWidth',1.0,'HandleVisibility','off');
        h1 = plot(a, t, o.dhm(i), '-', 'Color', COL_MEAS, 'LineWidth', LWD);
        legend(a, h1, {sprintf('%s      a_{pd} = %.4g ,  a_{cov} = %.4g', ...
               ARMS{c,2}, o.a_pd, o.a_cov)}, 'Location','northoutside', ...
               'Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
        ylim(a,[-0.12 0.12]); set(a,'YTick',-0.1:0.05:0.1);
        if c==1; ylabel(a,'\delta h_m  (\mum)','FontSize',FS,'FontWeight','bold'); end

        a = nexttile(tl, nA+c); A(2,c) = a; hold(a,'on');
        yline(a,0,'-','Color',[0.5 0.5 0.5],'LineWidth',1.0,'HandleVisibility','off');
        h0 = plot(a,t,o.dhm(i),'-','Color',COL_BG,'LineWidth',LWD);
        h1 = plot(a,t,o.dhmd(i),'-','Color',COL_LP,'LineWidth',2.2);
        if c==1
            legend(a,[h0 h1],{'\delta h_m','\delta h_{md}   (2)'},'Location','northoutside', ...
                'Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
            ylabel(a,'\delta h_{md}  (\mum)','FontSize',FS,'FontWeight','bold');
        end
        ylim(a,[-0.12 0.12]); set(a,'YTick',-0.1:0.05:0.1);

        a = nexttile(tl, 2*nA+c); A(3,c) = a; hold(a,'on');
        yline(a,0,'-','Color',[0.5 0.5 0.5],'LineWidth',1.0,'HandleVisibility','off');
        h1 = plot(a,t,o.dhmr(i),'-','Color',COL_HAT,'LineWidth',LWD);
        if c==1
            legend(a,h1,{'\delta h_{mr} = \delta h_m - \delta h_{md}   (3)'},'Location','northoutside', ...
                'Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
            ylabel(a,'\delta h_{mr}  (\mum)','FontSize',FS,'FontWeight','bold');
        end
        ylim(a,[-0.12 0.12]); set(a,'YTick',-0.1:0.05:0.1);

        a = nexttile(tl, 3*nA+c); A(4,c) = a; hold(a,'on');
        h1 = plot(a,t,sq,'-','Color',COL_SQ,'LineWidth',0.4);
        set(a,'YScale','log'); ylim(a,[1e-7 3e-2]); set(a,'YTick',10.^(-7:-2));
        if c==1
            legend(a,h1,{'( \delta h_{mr} )^2'},'Location','northoutside', ...
                'Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
            ylabel(a,'(\delta h_{mr})^2  (\mum^2)','FontSize',FS,'FontWeight','bold');
        end

        a = nexttile(tl, 4*nA+c); A(5,c) = a; hold(a,'on');
        h1 = plot(a,t,s2,'-','Color',COL_S2,'LineWidth',1.4);
        h2 = yline(a,o.Cn_s2n,'--','Color',COL_CN,'LineWidth',1.8);
        if c==1
            legend(a,[h1 h2],{[SH '^2_{\delta h_{mr}}   (4)'],'C_n \sigma_n^2'}, ...
                'Location','northoutside','Orientation','horizontal', ...
                'FontSize',LFS,'FontWeight','bold','Box','on');
            ylabel(a,[SH '^2_{\delta h_{mr}}  (\mum^2)'],'FontSize',FS,'FontWeight','bold');
        end
        ylim(a,[0 S2MAX]); set(a,'YTick',0:5e-4:2.5e-3);

        a = nexttile(tl, 5*nA+c); A(6,c) = a; hold(a,'on');
        h1 = plot(a,t,s2,'-','Color',COL_S2,'LineWidth',0.9);
        h2 = plot(a,t,ident,'-','Color',COL_TRUE,'LineWidth',2.6);
        h3 = yline(a,o.Cn_s2n,'--','Color',COL_CN,'LineWidth',1.8);
        if c==1
            legend(a,[h1 h2 h3],{[SH '^2_{\delta h_{mr}}'], ...
                'C_{dpmr} 4k_BT a_h + C_n \sigma_n^2   (5)','C_n \sigma_n^2'}, ...
                'Location','northoutside','Orientation','horizontal', ...
                'FontSize',LFS,'FontWeight','bold','Box','on');
            ylabel(a,[SH '^2_{\delta h_{mr}}  (\mum^2)'],'FontSize',FS,'FontWeight','bold');
        end
        ylim(a,[0 S2MAX]); set(a,'YTick',0:5e-4:2.5e-3);

        a = nexttile(tl, 6*nA+c); A(7,c) = a; hold(a,'on');
        h1 = plot(a,t,o.ahm(i),'-','Color',COL_MEAS,'LineWidth',0.7);
        h2 = plot(a,t,ahT,'-','Color',COL_TRUE,'LineWidth',2.6);
        if c==1
            legend(a,[h1 h2],{'a_{hm}   (6)','a_h'},'Location','northoutside', ...
                'Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
            ylabel(a,'a_h  (\mum/pN)','FontSize',FS,'FontWeight','bold');
        end
        % y limit must NOT use this arm's own den: C_dpmr changes with a_pd, so a
        % per-column limit would make the identical a_h truth look different.
        ylim(a,[0 AHMAX]); set(a,'YTick',0:0.01:0.04);
        xlabel(a,'Time (sec)','FontSize',FS,'FontWeight','bold');
    end

    for row = 1:7
        for c = 1:nA
            set(A(row,c),'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on');
            grid(A(row,c),'off');  xlim(A(row,c),[D{1}.t(1) D{1}.t(end)]);
            if row < 7;  set(A(row,c),'XTickLabel',[]); end
            if c   > 1;  set(A(row,c),'YTickLabel',[]); end
        end
    end

    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root,'test_results','apd_acov_meng');
    fn = fullfile(od, sprintf('dhm_to_ahm_arms%s.png', tag));
    exportgraphics(f, fn, 'Resolution', 150);  close(f);
    fprintf('figure -> %s\n', fn);
end
