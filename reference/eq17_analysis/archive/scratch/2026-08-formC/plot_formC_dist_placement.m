function plot_formC_dist_placement(res, R_um, out_png, sq, pick, names, modes)
    if nargin < 4 || isempty(sq);    sq = 1; end
    if nargin < 5 || isempty(pick);  pick = [2 3 4]; end
    if nargin < 6 || isempty(names)
        names = {'no \\delta', '\\delta in law height', '\\delta additive on gain'};
    end
    if nargin < 7 || isempty(modes); modes = {'none','height','gain'}; end
%PLOT_FORMC_DIST_PLACEMENT  Where the disturbance acts: three arms, one page.
%
% STATUS: ACTIVE (scratch) | PURPOSE: read the 2026-08-19 placement comparison
%   (no delta / delta in the law height / delta additive on the gain) against a
%   wall whose law is 20% off. EXPIRES: when the placement question is decided.
%
%   Columns  BASE2 (no delta) | ARM A (height) | ARM B (additive)
%   Rows     1 gain, true vs estimate     2 gain error [%]
%            3 delta_hat with its +-sqrt(P55) band and the value it would need
%   Style: canonical (plot_var_ahat_6state.m).

    AX = 3;  Ts = 6.25e-4;  KAPPA = 1 - sqrt(0.8);   % needed delta = KAPPA * w_bar
    COL_T = [0.8 0 0]; COL_H = [0 0.2 0.9]; BANDC = [0.45 0.55 0.95];
    FS = 18; LFS = 12; LW = 2.0;

    d = cell(1,3);
    for c = 1:3
        r = res(pick(c)).o.runs{sq};
        ad = r.a_hat_out(1,AX) / r.a_bar_hat_out(1,AX);
        s.aT = r.a_true_out(:,AX) / ad;   s.aH = r.a_bar_hat_out(:,AX);
        s.e  = 100*(s.aH - s.aT)./s.aT;
        s.dh = r.b_hat_out(:,AX);         s.sd = r.P_b_out(:,AX);
        s.wb = r.p_true_out(:,AX)/R_um;
        % The value the disturbance would have to take is DIFFERENT for each
        % placement -- six orders of magnitude apart, and one flips sign:
        %   height : a_bar'(w - d) = a_bar'_true  =>  d = w (1 - sqrt(b_p))
        %   gain   : d = (a_bar'_true - a_bar'_model) M = (1/b_p - 1) M / w^2
        % Drawing the height curve on the gain column (as the first version of
        % this figure did) compares a number against a target 1e4 times too big.
        M = [diff(r.p_d_out(:,AX))/R_um; 0];
        switch modes{c}
            case 'none';   s.nd = nan(size(s.wb));
            case 'height'; s.nd = KAPPA * s.wb;
            case 'gain';   s.nd = (1/0.8 - 1) * M ./ s.wb.^2;
        end
        s.t  = (0:numel(s.aH)-1).'*Ts;    d{c} = s;
    end
    YL = {local_lim([d{1}.aT; d{1}.aH; d{2}.aH; d{3}.aH],0.05), ...
          local_lim([d{1}.e; d{2}.e; d{3}.e],0.08), ...
          []};   % row 3 scales PER COLUMN: the two placements are 1e4 apart

    f = figure('Position',[40 40 1720 1150],'Color','w','Visible','off');
    tl = tiledlayout(f,3,3,'TileSpacing','compact','Padding','compact');
    for row = 1:3
        for c = 1:3
            s = d{c}; a = nexttile(tl,(row-1)*3+c); hold(a,'on');
            switch row
                case 1
                    h1=plot(a,s.t,s.aT,'-','Color',COL_T,'LineWidth',LW+0.6,'DisplayName','a_{true}');
                    h2=plot(a,s.t,s.aH,'-','Color',COL_H,'LineWidth',LW,'DisplayName',['a_{hat}  ' names{c}]);
                    legend(a,[h1 h2],'Location','northoutside','Orientation','horizontal', ...
                           'FontSize',LFS,'FontWeight','bold','Box','on');
                    if c==1; ylabel(a,'a_z / a_o','FontSize',FS,'FontWeight','bold'); end
                case 2
                    yline(a,0,'-','Color',[.4 .4 .4],'LineWidth',1,'HandleVisibility','off');
                    plot(a,s.t,s.e,'-','Color',COL_H,'LineWidth',LW);
                    if c==1; ylabel(a,'a_z  error  [%]','FontSize',FS,'FontWeight','bold'); end
                case 3
                    fill(a,[s.t;flipud(s.t)],[s.dh+s.sd;flipud(s.dh-s.sd)],BANDC, ...
                         'EdgeColor','none','FaceAlpha',0.30,'DisplayName','\pm sqrt(P_{55})');
                    p1=plot(a,s.t,s.nd,'-','Color',COL_T,'LineWidth',LW,'DisplayName','needed');
                    p2=plot(a,s.t,s.dh,'-','Color',COL_H,'LineWidth',LW,'DisplayName','\delta_{hat}');
                    legend(a,[p1 p2],'Location','northoutside','Orientation','horizontal', ...
                           'FontSize',LFS,'FontWeight','bold','Box','on');
                    if c==1; ylabel(a,'\delta','FontSize',FS,'FontWeight','bold'); end
            end
            if row==3; xlabel(a,'time  [s]','FontSize',FS,'FontWeight','bold'); end
            xlim(a,[s.t(1) s.t(end)]);
            if row < 3
                ylim(a,YL{row});
            else
                v=[s.dh+s.sd; s.dh-s.sd; s.nd(isfinite(s.nd)); 0];
                ylim(a, local_lim(v,0.08));
            end
            set(a,'FontSize',FS,'FontWeight','bold','LineWidth',2.0,'Box','on');
            if row<3; set(a,'XTickLabel',[]); end
            if c>1 && row<3; set(a,'YTickLabel',[]); end
            grid(a,'off');
        end
    end
    exportgraphics(f,out_png,'Resolution',150); close(f);
    fprintf('wrote %s\n', out_png);
end

function L = local_lim(v,pad)
    lo=min(v); hi=max(v); r=hi-lo; if r<=0; r=1; end
    L=[lo-pad*r, hi+pad*r];
end
