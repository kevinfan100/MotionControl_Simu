function plot_formC_b_ride_along(S, opts)
%PLOT_FORMC_B_RIDE_ALONG  Simple check: does b's y2 correction move in lockstep
%   with a_bar's y2 correction during hold (the "riding along" hypothesis),
%   and does that lockstep break down when b has its own fresh information?
%
% STATUS: ACTIVE | quick companion to plot_formC_kf_gains
%   No new simulation: uses channels already in the 100-seed gains stack.

    if nargin < 2; opts = struct(); end
    if ~isfield(opts,'ax'); opts.ax = 3; end
    ax = opts.ax; ns = numel(S.seeds); t = S.t;

    L42 = mean(squeeze(S.K_a_y2_out(:,ax,1:ns)),2);
    L52 = mean(squeeze(S.K_b_y2_out(:,ax,1:ns)),2);
    ratio = L52 ./ L42;

    FS=18; LFS=12; AXLW=2.0; LW=1.8;
    f = figure('Position',[40 40 1300 750],'Color','w','NumberTitle','off','Visible','off');
    tl = tiledlayout(f,2,1,'TileSpacing','compact','Padding','compact');

    a = nexttile(tl); hold(a,'on');
    yyaxis(a,'left');
    h1=plot(a,t,L42,'-','Color',[0 0.2 0.9],'LineWidth',LW,'DisplayName','L_{42}  (a_{bar} \leftarrow y_2)');
    ylabel(a,'L_{42}[k]','FontSize',FS,'FontWeight','bold');
    yyaxis(a,'right');
    h2=plot(a,t,L52,'-','Color',[0.1 0.55 0.15],'LineWidth',LW,'DisplayName','L_{52}  (b \leftarrow y_2)');
    ylabel(a,'L_{52}[k]','FontSize',FS,'FontWeight','bold');
    legend(a,[h1 h2],'Location','northoutside','Orientation','horizontal', ...
           'FontSize',LFS,'FontWeight','bold','Box','on');
    set(a,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); axis(a,'tight');

    a = nexttile(tl); hold(a,'on');
    plot(a,t,ratio,'-','Color',[0.6 0.2 0.7],'LineWidth',LW);
    yline(a,0,'-','Color',[0.6 0.6 0.6],'LineWidth',1.0);
    xlabel(a,'time  [s]','FontSize',FS,'FontWeight','bold');
    ylabel(a,'L_{52} / L_{42}','FontSize',FS,'FontWeight','bold');
    set(a,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on');
    ylim(a,[-3 5]);
    text(a,3.7,-2.3,'hold: ratio flat -> b rides on a_{bar}''s correction', ...
         'FontSize',12,'FontWeight','bold','Color',[0.6 0.2 0.7]);
    text(a,1.7,4.2,'motion: ratio scatters -> b has its own fresh info', ...
         'FontSize',12,'FontWeight','bold','Color',[0.6 0.2 0.7]);

    sgtitle(f, sprintf('does b ride on a_{bar}''s y_2 correction? z axis, N = %d seeds', ns), ...
            'FontSize',FS-2,'FontWeight','normal');

    here = fileparts(mfilename('fullpath'));
    out = fullfile(fileparts(fileparts(here)), 'test_results', 'formC_cdpmr_var_check', 'var_kf_b_ride_along.png');
    exportgraphics(f,out,'Resolution',150);
    fprintf('[ride-along] wrote %s\n', out);
end
