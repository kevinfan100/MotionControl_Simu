function plot_formC_kf_gains(S, opts)
%PLOT_FORMC_KF_GAINS  The four Kalman gains L_ij on formC_b's actual state
%   ordering (row 3 = delta_w3_hat, current tracking error; row 4 = a_bar_hat,
%   the gain), and what they actually moved.
%
% STATUS: ACTIVE | companion to check_formC_var_identity / the error budget
%
%   Notation confirmed against reference/shared/writeup_architecture.tex
%   Sec.3.1 (L = P H' (H P H' + R)^-1) but re-indexed to formC_b's OWN state
%   vector [dw1 dw2 dw3 a_bar da ...], where the gain is slot 4, not slot 6
%   as in that shared 7-state template. Row 3 in this figure is formC_b's
%   slot 3 (delta_w3, current tracking error); the shared template's row 3
%   plays the same role in ITS OWN ordering, but the two matrices are not
%   the same object.
%
%   FIGURE A (var_kf_gains.png): the four gains L31 L32 L41 L42 vs time,
%   ensemble mean over seeds, one row per state, one column per measurement,
%   EACH PANEL ITS OWN Y-AXIS (L41 ~ 0.25, L42 ~ 0.006, 40x apart -- a shared
%   axis would flatten the small one to a line). A navigation row on top
%   shows a_bar_true / a_bar_hat so the reader knows where in the trajectory
%   each swing happens.
%
%   FIGURE B (var_kf_contrib.png): cumulative sum of L_ij[k]*innov_j[k],
%   same 2x2 layout. Row 4 reproduces the error-budget numbers exactly (y1
%   +0.078, y2 -0.012 in the final hold); row 3 is a KNOWN-ANSWER control,
%   since delta_w3 IS the measured tracking error and its own budget can be
%   read directly off dx_r_out / p_true_out for a cross-check.

    if nargin < 2; opts = struct(); end
    if ~isfield(opts, 'ax');  opts.ax  = 3;  end
    if ~isfield(opts, 'out_dir'); opts.out_dir = ''; end

    K = S.K; ax = opts.ax; ns = numel(S.seeds); t = S.t;
    AT = mean(squeeze(S.a_true_out(:,ax,1:ns)),2) / K.a_nom;
    AH = mean(squeeze(S.a_bar_hat_out(:,ax,1:ns)),2);

    L31 = mean(squeeze(S.K_dx_y1_out(:,ax,1:ns)),2);
    L32 = mean(squeeze(S.K_dx_y2_out(:,ax,1:ns)),2);
    L41 = mean(squeeze(S.K_a_y1_out(:,ax,1:ns)),2);
    L42 = mean(squeeze(S.K_a_y2_out(:,ax,1:ns)),2);
    have_b = isfield(S,'K_b_y1_out') && isfield(S,'K_b_y2_out');
    if have_b
        L51 = mean(squeeze(S.K_b_y1_out(:,ax,1:ns)),2);
        L52 = mean(squeeze(S.K_b_y2_out(:,ax,1:ns)),2);
        BH  = mean(squeeze(S.b_hat_out(:,ax,1:ns)),2);
    end

    I1 = squeeze(S.innov_y1_out(:,ax,1:ns));
    I2 = squeeze(S.innov_y2_out(:,ax,1:ns));
    K31 = squeeze(S.K_dx_y1_out(:,ax,1:ns));  K32 = squeeze(S.K_dx_y2_out(:,ax,1:ns));
    K41 = squeeze(S.K_a_y1_out(:,ax,1:ns));   K42 = squeeze(S.K_a_y2_out(:,ax,1:ns));

    C31 = mean(cumsum(K31.*I1,1),2);  C32 = mean(cumsum(K32.*I2,1),2);
    C41 = mean(cumsum(K41.*I1,1),2);  C42 = mean(cumsum(K42.*I2,1),2);
    if have_b
        K51 = squeeze(S.K_b_y1_out(:,ax,1:ns)); K52 = squeeze(S.K_b_y2_out(:,ax,1:ns));
        C51 = mean(cumsum(K51.*I1,1),2);        C52 = mean(cumsum(K52.*I2,1),2);
    end

    if isempty(opts.out_dir)
        here = fileparts(mfilename('fullpath'));
        opts.out_dir = fullfile(fileparts(fileparts(here)), 'test_results', 'formC_cdpmr_var_check');
    end
    if ~exist(opts.out_dir,'dir'); mkdir(opts.out_dir); end

    if have_b
        local_page_A(t, AT, AH, L31, L32, L41, L42, ns, ...
                     fullfile(opts.out_dir, 'var_kf_gains_b.png'), L51, L52, BH);
        local_page_B(t, AT, AH, C31, C32, C41, C42, ns, ...
                     fullfile(opts.out_dir, 'var_kf_contrib_b.png'), C51, C52, BH);
    else
        local_page_A(t, AT, AH, L31, L32, L41, L42, ns, fullfile(opts.out_dir, 'var_kf_gains.png'));
        local_page_B(t, AT, AH, C31, C32, C41, C42, ns, fullfile(opts.out_dir, 'var_kf_contrib.png'));
    end
end

% ----------------------------------------------------------------------
function local_page_A(t, AT, AH, L31, L32, L41, L42, ns, png, L51, L52, BH)
    have_b = nargin >= 12;
    FS = 18; LFS = 12; AXLW = 2.0; LW = 1.6;
    nrow = 3 + have_b;
    f = figure('Position',[40 40 1400 380*nrow+280],'Color','w','NumberTitle','off','Visible','off');
    tl = tiledlayout(f,nrow,2,'TileSpacing','compact','Padding','compact');

    a = nexttile(tl,[1 2]); hold(a,'on');
    h1=plot(a,t,AT,'-','Color',[0.8 0 0],'LineWidth',LW+0.4,'DisplayName','a_{bar,true}');
    h2=plot(a,t,AH,'-','Color',[0 0.2 0.9],'LineWidth',LW,'DisplayName','a_{bar,hat}');
    leg = [h1 h2]; labs={'a_{bar,true}','a_{bar,hat}'};
    if have_b
        h3=plot(a,t,BH,'-','Color',[0.1 0.55 0.15],'LineWidth',LW,'DisplayName','b_{hat}');
        leg=[leg h3];
    end
    legend(a,leg,'Location','northoutside','Orientation','horizontal', ...
           'FontSize',LFS,'FontWeight','bold','Box','on');
    ylabel(a,'a/a_o , b  [-]','FontSize',FS,'FontWeight','bold');
    set(a,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); axis(a,'tight');

    D = {L31,'L_{31}[k]   (\delta w_3  \leftarrow  y_1)'; ...
         L32,'L_{32}[k]   (\delta w_3  \leftarrow  y_2)'; ...
         L41,'L_{41}[k]   (a_{bar}  \leftarrow  y_1)'; ...
         L42,'L_{42}[k]   (a_{bar}  \leftarrow  y_2)'};
    if have_b
        D = [D; {L51,'L_{51}[k]   (b  \leftarrow  y_1)'; L52,'L_{52}[k]   (b  \leftarrow  y_2)'}];
    end
    for i = 1:size(D,1)
        a = nexttile(tl,2+i); hold(a,'on');
        plot(a,t,D{i,1},'-','Color',[0 0.2 0.9],'LineWidth',LW);
        yline(a,0,'-','Color',[0.6 0.6 0.6],'LineWidth',1.0,'HandleVisibility','off');
        xlabel(a,'time  [s]','FontSize',FS,'FontWeight','bold');
        ylabel(a,D{i,2},'FontSize',FS-2,'FontWeight','bold');
        set(a,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); axis(a,'tight');
    end
    sgtitle(f, sprintf('formC_b Kalman gains, z axis, N = %d seeds, ensemble mean', ns), ...
            'FontSize',FS-2,'FontWeight','normal');
    exportgraphics(f,png,'Resolution',150);
    fprintf('[kf-gains] wrote %s\n', png);
end

% ----------------------------------------------------------------------
function local_page_B(t, AT, AH, C31, C32, C41, C42, ns, png, C51, C52, BH)
    have_b = nargin >= 11;
    FS = 18; LFS = 12; AXLW = 2.0; LW = 1.8;
    nrow = 3 + have_b;
    f = figure('Position',[40 40 1400 380*nrow+280],'Color','w','NumberTitle','off','Visible','off');
    tl = tiledlayout(f,nrow,2,'TileSpacing','compact','Padding','compact');

    a = nexttile(tl,[1 2]); hold(a,'on');
    h1=plot(a,t,AT,'-','Color',[0.8 0 0],'LineWidth',LW,'DisplayName','a_{bar,true}');
    h2=plot(a,t,AH,'-','Color',[0 0.2 0.9],'LineWidth',LW-0.2,'DisplayName','a_{bar,hat}');
    leg=[h1 h2];
    if have_b
        h3=plot(a,t,BH,'-','Color',[0.1 0.55 0.15],'LineWidth',LW-0.2,'DisplayName','b_{hat}');
        leg=[leg h3];
    end
    legend(a,leg,'Location','northoutside','Orientation','horizontal', ...
           'FontSize',LFS,'FontWeight','bold','Box','on');
    ylabel(a,'a/a_o , b  [-]','FontSize',FS,'FontWeight','bold');
    set(a,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); axis(a,'tight');

    D = {C31,'\Sigma L_{31} e_{y1}   (into \delta w_3)'; ...
         C32,'\Sigma L_{32} e_{y2}   (into \delta w_3)'; ...
         C41,'\Sigma L_{41} e_{y1}   (into a_{bar})'; ...
         C42,'\Sigma L_{42} e_{y2}   (into a_{bar})'};
    if have_b
        D = [D; {C51,'\Sigma L_{51} e_{y1}   (into b)'; C52,'\Sigma L_{52} e_{y2}   (into b)'}];
    end
    for i = 1:size(D,1)
        a = nexttile(tl,2+i); hold(a,'on');
        plot(a,t,D{i,1},'-','Color',[0.1 0.55 0.15],'LineWidth',LW);
        yline(a,0,'-','Color',[0.6 0.6 0.6],'LineWidth',1.0,'HandleVisibility','off');
        xlabel(a,'time  [s]','FontSize',FS,'FontWeight','bold');
        ylabel(a,D{i,2},'FontSize',FS-3,'FontWeight','bold');
        set(a,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); axis(a,'tight');
    end
    sgtitle(f, sprintf('formC_b cumulative measurement contribution, z axis, N = %d seeds', ns), ...
            'FontSize',FS-2,'FontWeight','normal');
    exportgraphics(f,png,'Resolution',150);
    fprintf('[kf-gains] wrote %s\n', png);
end
