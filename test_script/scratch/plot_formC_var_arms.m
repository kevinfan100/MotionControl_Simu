function plot_formC_var_arms(ARMS, opts)
%PLOT_FORMC_VAR_ARMS  The four arms of the variance-identity check on one page.
%
% STATUS: ACTIVE | companion to check_formC_var_identity
%
%   Row 1  Var(delta_x)   measured / formula, against a/a_o
%   Row 2  Var(delta_x_r) measured / formula, against a/a_o
%   One line per arm, 1.00 drawn as the reference. This is the DECIDING view:
%   the absolute panels show that the formula tracks a over 11x, this one
%   shows where it stops being true and by how much.
%
%   The formula is always evaluated at the DESIGNED pole lambda_c, because the
%   question the page answers is "does the identity as written hold", not
%   "can it be repaired". The lambda_eff repair is the other figure.

    if nargin < 2; opts = struct(); end
    if ~isfield(opts, 'n_bin'); opts.n_bin = 14; end
    if ~isfield(opts, 'ax');    opts.ax    = 3;  end
    if ~isfield(opts, 'out');   opts.out   = ''; end

    lc = ARMS{1,2}.K.lambda_c;  ax = opts.ax;  nA = size(ARMS,1);
    COL = [0 0.2 0.9; 0.85 0.1 0.1; 0.1 0.6 0.2; 0.55 0.3 0.75];
    MRK = {'o','s','^','v'};
    FS = 18; LFS = 12; AXLW = 2.0;

    f = figure('Position',[40 40 1200 950],'Color','w','NumberTitle','off','Visible','off');
    tl = tiledlayout(f,2,1,'TileSpacing','compact','Padding','compact');
    YL = {'var(\delta x)  measured / formula', 'var(\delta x_r)  measured / formula'};
    for row = 1:2
        a = nexttile(tl,row); hold(a,'on');
        yline(a,1,'-','Color',[0.5 0.5 0.5],'LineWidth',1.5,'HandleVisibility','off');
        h = gobjects(nA,1);
        for q = 1:nA
            [bx,by,be] = local_arm(ARMS{q,2}, ax, lc, opts.n_bin, row);
            h(q) = errorbar(a,bx,by,be,MRK{q},'Color',COL(q,:),'MarkerFaceColor',COL(q,:), ...
                            'MarkerSize',7,'LineWidth',1.8,'LineStyle','-', ...
                            'DisplayName',ARMS{q,1});
        end
        if row == 1
            legend(a,h,'Location','northoutside','Orientation','horizontal', ...
                   'FontSize',LFS,'FontWeight','bold','Box','on');
        end
        xlabel(a,'a / a_o  [-]','FontSize',FS,'FontWeight','bold');
        ylabel(a,YL{row},'FontSize',FS,'FontWeight','bold');
        set(a,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on');
        axis(a,'tight'); xlim(a,[0 1]);
    end
    if ~isempty(opts.out)
        exportgraphics(f,opts.out,'Resolution',150);
        fprintf('[arms] wrote %s\n', opts.out);
    end
end

% ----------------------------------------------------------------------
function [bx,by,be] = local_arm(S, ax, lc, nbin, row)
    K = S.K; ns = numel(S.seeds); N = numel(S.t);
    kap = 4*(K.kBT/K.R)*K.a_o;  sg = K.sigma2_n_s(ax)/K.R^2;
    ab = squeeze(S.a_true_out(:,ax,1))/K.a_nom;
    if row == 1
        X = squeeze(S.p_true_out(:,ax,1:ns));
        C  = 2 + 1/(1-lc^2);         Cn = (1-lc)/(1+lc);
    else
        X = squeeze(S.dx_r_out(:,ax,1:ns));
        apd = K.a_pd; om = 1-apd; dp = 1-om*lc;
        C  = om^2*( 2*om*(1-lc)/dp + (2/(2-apd))/((1+lc)*dp) );
        Cn = (2*om^2/(2-apd))*( 1 + om^2*apd*(1-lc)/dp + (1-lc)^2/((1+lc)*dp) );
    end
    kk = (81:N).';
    V = var(X(kk,:),0,2)/K.R^2;
    [xs,i] = sort(ab(kk));  Vs = V(i);  Xs = X(kk,:);  Xs = Xs(i,:);
    S1 = sum(Xs,2); S2 = sum(Xs.^2,2);
    e = round(linspace(1,numel(xs)+1,nbin+1));
    bx = zeros(nbin,1); by = bx; be = bx;
    for b = 1:nbin
        id = e(b):e(b+1)-1;  ai = mean(xs(id));  th = C*kap*ai + Cn*sg;
        bx(b) = ai;  by(b) = mean(Vs(id))/th;
        xj = Xs(id,:);
        Vloo = ((S2(id)-xj.^2) - (S1(id)-xj).^2/(ns-1))/(ns-2)/K.R^2;
        t2 = mean(Vloo,1)/th;
        be(b) = sqrt((ns-1)/ns*sum((t2-mean(t2)).^2));
    end
end
