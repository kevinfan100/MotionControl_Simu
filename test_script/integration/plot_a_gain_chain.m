function plot_a_gain_chain()
%PLOT_A_GAIN_CHAIN  Wall-effect gain chain c(h_bar) -> a, a', a'' vs h/R.
%   z-axis (perpendicular, c_perp) only for now; the parallel (x,y) case is
%   structurally identical (swap in c_para / K_h_para) and can be overlaid
%   later via a second gain_chain() call without changing the layout.
%
%   plot_a_gain_chain()
%
%   Produces a 4-panel vertically-stacked figure sharing the h/R axis
%   (c -> a -> a' -> a''), with a dual x-ruler (h/R on the bottom, h in um on
%   the top), and prints a numeric sanity table (h_bar = 1.1 / 1.5 / 22) to
%   the console. All panels use a linear y-scale; a'' is plotted with its sign
%   (a'' < 0 throughout). Wall effect is a near-wall phenomenon: every curve is
%   large near the wall and flattens beyond h_bar ~ 6.
%
%   Canonical style (mirrors plot_para_aprime): NO title, Box on / grid off,
%   FS=18 bold, exportgraphics 150 dpi, Visible off.
%
%   Reuses (SSOT, not re-coded):
%     model/wall_effect/calc_correction_functions.m  -> c_perp, K_h, K_h'
%     model/config/physical_constants.m              -> R, gamma_N, Ts
%   Companion doc: reference/eq17_analysis/derivation/a_gain_chain.tex.
%   Output PNG: test_results/a_gain_chain/fig_a_gain_chain_z.png (gitignored;
%   copy into reference/eq17_analysis/derivation/figures/ for the doc).

    [sd,~,~]=fileparts(mfilename('fullpath')); proj=fileparts(fileparts(sd));
    addpath(genpath(fullfile(proj,'model')));
    od=fullfile(proj,'test_results','a_gain_chain');
    if ~exist(od,'dir'); mkdir(od); end

    pc = physical_constants();
    R = pc.R; a_nom = pc.Ts / pc.gamma_N;

    % ---- h/R grid (z = perpendicular) ----
    hb_min = 1.1; hb_max = 22;
    g = linspace(hb_min, hb_max, 500).';
    cP = zeros(size(g)); Kh = zeros(size(g)); Khp = zeros(size(g));
    for i = 1:numel(g)
        [~, cP(i), dv] = calc_correction_functions(g(i), true);
        Kh(i)  = dv.K_h_perp;
        Khp(i) = dv.K_h_prime_perp;
    end
    [aZ, apZ, appZ] = gain_chain(cP, Kh, Khp, a_nom, R);

    % sign sanity (expected over [1.1,22]: a>0 increasing, a'>0, a''<0)
    if any(apZ<=0) || any(appZ>=0)
        warning('plot_a_gain_chain:sign', ...
            'Unexpected sign: min(a'')=%.3g, max(a'''')=%.3g', min(apZ), max(appZ));
    end

    % ---- numeric reference table (console) ----
    hb_ref = [1.10; 1.50; 22.0];
    fprintf('\n  h/R        c_perp        K_h         K_h''          a_z(um/pN)      a''_z(1/pN)    a''''_z(1/(pN*um))\n');
    fprintf(  '  -----   -----------   ---------   -----------   -------------   ------------   -----------------\n');
    for i = 1:numel(hb_ref)
        [~, cr, dr] = calc_correction_functions(hb_ref(i), true);
        [ar, apr, appr] = gain_chain(cr, dr.K_h_perp, dr.K_h_prime_perp, a_nom, R);
        fprintf('  %5.2f   %11.5f   %9.4f   %11.5f   %13.5e   %12.5e   %12.5e\n', ...
                hb_ref(i), cr, dr.K_h_perp, dr.K_h_prime_perp, ar, apr, appr);
    end
    fprintf('\n  a_nom = Ts/gamma_N = %.6e um/pN ; R = %.3f um\n', a_nom, R);
    fprintf('  far-field check: a_z(h/R=22) = %.5e (= %.1f%% of a_nom)\n\n', aZ(end), 100*aZ(end)/a_nom);

    % ===== figure: 4 stacked panels, z-only, dual x-ruler =====
    COL=[0.8 0 0]; FS=18; AXLW=2.0; LW=2.4;
    xL=0.175; xW=0.775; gap=0.052; yTop=0.895; yBot=0.075;
    pH=(yTop-yBot-3*gap)/4;
    posf=@(i)[xL, yTop-i*pH-(i-1)*gap, xW, pH];

    f=figure('Position',[80 30 720 980],'Color','w','NumberTitle','off','Visible','off');

    % panel 1: c(h_bar)
    ax1=axes('Parent',f,'Position',posf(1));
    plot(ax1,g,cP,'-','Color',COL,'LineWidth',LW);
    ylabel(ax1,'c_{\perp}','FontSize',FS,'FontWeight','bold');
    style_panel(ax1,FS,AXLW,[hb_min hb_max]);
    set(ax1,'XTickLabel',{});

    % panel 2: a(h) = a_nom / c
    ax2=axes('Parent',f,'Position',posf(2));
    plot(ax2,g,aZ,'-','Color',COL,'LineWidth',LW);
    ylabel(ax2,'a  (\mum/pN)','FontSize',FS,'FontWeight','bold');
    style_panel(ax2,FS,AXLW,[hb_min hb_max]);
    set(ax2,'XTickLabel',{});

    % panel 3: a'(h) = da/dh  (>0)
    ax3=axes('Parent',f,'Position',posf(3));
    plot(ax3,g,apZ,'-','Color',COL,'LineWidth',LW);
    ylabel(ax3,'a''  (1/pN)','FontSize',FS,'FontWeight','bold');
    style_panel(ax3,FS,AXLW,[hb_min hb_max]);
    set(ax3,'XTickLabel',{});

    % panel 4: a''(h) = d^2a/dh^2  (<0, plotted with sign)
    ax4=axes('Parent',f,'Position',posf(4));
    plot(ax4,g,appZ,'-','Color',COL,'LineWidth',LW);
    ylabel(ax4,'a''''  (1/(pN\cdot\mum))','FontSize',FS-2,'FontWeight','bold');
    xlabel(ax4,'h / R','FontSize',FS,'FontWeight','bold');
    style_panel(ax4,FS,AXLW,[hb_min hb_max]);

    % top ruler on panel 1: h (um) = R * (h/R)
    drawnow;
    axT=axes('Parent',f,'Position',posf(1),'Color','none','XAxisLocation','top',...
        'YAxisLocation','right','YTick',[],'Box','off',...
        'XLim',R*[hb_min hb_max],'XColor','k','FontSize',FS,'FontWeight','bold','LineWidth',AXLW);
    xlabel(axT,'h  (\mum)','FontSize',FS,'FontWeight','bold');

    set(findall(f,'Type','Axes'),{'Toolbar'},{[]});
    outpng=fullfile(od,'fig_a_gain_chain_z.png');
    exportgraphics(f,outpng,'Resolution',150); close(f);
    fprintf('  saved figure: %s\n\n', outpng);
end

function [a, ap, app] = gain_chain(c, K_h, K_h_prime, a_nom, R)
% GAIN_CHAIN  c(h_bar) + wall sensitivities -> a, a', a'' (per axis).
%   a   = a_nom / c
%   a'  = da/dh    = -(a/R) * K_h            (K_h  = (1/c) dc/dh_bar)
%   a'' = d^2a/dh2 =  (a/R^2) * (K_h^2 - K_h')  (K_h' = dK_h/dh_bar)
    a   = a_nom ./ c;
    ap  = -(a ./ R) .* K_h;
    app =  (a ./ R.^2) .* (K_h.^2 - K_h_prime);
end

function style_panel(ax, FS, AXLW, xlim_v)
    set(ax,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on');
    grid(ax,'off'); xlim(ax,xlim_v);
end
