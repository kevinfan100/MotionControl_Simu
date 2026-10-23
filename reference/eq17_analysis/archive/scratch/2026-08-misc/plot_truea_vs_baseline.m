% One figure for the decisive arm: measured / formula against a_bar, three
% axes, baseline against the true-a arm. Ratio on a linear y so a 30%
% departure is a third of the plot; log x because a spans 11x on z.
root = '/Users/kevin/Code/MotionControl_Simu-motion-test/';
T = load([root 'test_results/am_r22_deep/truea_100.mat']);
B = load([root 'test_results/formC_cdpmr_var_check/raw_seeds.mat']);
K = T.K; a_nom = T.a_nom; R = B.S400.K.R; a_cov = K.a_cov;
kT = 4*(B.S400.K.kBT/R)*B.S400.K.a_o; ab = K.IF_abc(:); nb = 100;
COL = [0.85 0.33 0.10; 0.47 0.67 0.19; 0 0.2 0.9];   % x, y, z
axl = 'xyz';
f = figure('Position',[80 80 1300 600],'Color','w','NumberTitle','off','Visible','off');
tiledlayout(1,2,'TileSpacing','compact','Padding','compact');
H = gobjects(0); H2 = gobjects(0);
for ax = [3 1 2]
    if ax == 3
        nexttile(1); hold on;
        yline(1,'-','Color',[0.35 0.35 0.35],'LineWidth',1.5,'HandleVisibility','off');
    elseif ax == 1
        nexttile(2); hold on;
        yline(1,'-','Color',[0.35 0.35 0.35],'LineWidth',1.5,'HandleVisibility','off');
    end
    s2n = B.S400.K.sigma2_n_s(ax)/R^2;
    xi  = (K.C_n/K.C_dpmr)*s2n/kT;
    ife = @(a) 1 + 2*(((kT*a).^2*ab(1) + 2*(kT*a)*s2n*ab(2) + s2n^2*ab(3)) ./ ...
                      ((K.C_dpmr*kT*a + K.C_n*s2n).^2));
    for arm = 1:2
        if arm == 1
            A = squeeze(B.S400.a_xm_out(:,ax,1:nb))/a_nom;
            Tr= squeeze(B.S400.a_true_out(:,ax,1:nb))/a_nom; tt = B.S400.t(:);
        else
            A = squeeze(T.A_xm(:,ax,:))/a_nom;
            Tr= squeeze(T.A_tr(:,ax,:))/a_nom;  tt = T.t(:);
        end
        A = A(2:end,:); Tr = Tr(2:end,:); tt = tt(2:end);
        am = mean(Tr,2); n = size(A,2);
        ed = linspace(min(am), max(am), 13);
        c=[]; r=[]; e=[];
        for b = 1:12
            in = am >= ed(b) & am < ed(b+1);
            if sum(in) < 30; continue; end
            Xi = A(in,:); S1 = sum(Xi,2); S2 = sum(Xi.^2,2);
            v  = mean((S2 - S1.^2/n)/(n-1));
            vl = (S2 - Xi.^2 - (S1-Xi).^2/(n-1))/(n-2);
            th = mean(vl,1); se = sqrt((n-1)/n*sum((th-mean(th)).^2));
            cc = mean(am(in)); tho = K.K_var*ife(cc)*(cc+xi)^2;
            c(end+1)=cc; r(end+1)=v/tho; e(end+1)=se/tho; %#ok<AGROW>
        end
        if arm == 1
            h = errorbar(c,r,e,'o--','Color',COL(ax,:),'MarkerSize',7, ...
                'LineWidth',1.6,'CapSize',3,'MarkerFaceColor','w', ...
                'DisplayName',sprintf('%c  baseline', axl(ax)));
        else
            h = errorbar(c,r,e,'o-','Color',COL(ax,:),'MarkerSize',7, ...
                'LineWidth',2.2,'CapSize',3,'MarkerFaceColor',COL(ax,:), ...
                'DisplayName',sprintf('%c  control law on a_{true}', axl(ax)));
        end
        if ax == 3; H(end+1) = h; else; H2(end+1) = h; end %#ok<AGROW>
    end
    if ax == 3
        set(gca,'XScale','log'); xlim([0.075 1.05]); ylim([0.85 1.45]);
        set(gca,'XTick',[0.1 0.2 0.3 0.5 0.7 1.0]);
        gh=gca; gh.XAxis.Exponent=0; xtickformat('%.2f');
        xlabel('a_z / a_o','FontSize',18,'FontWeight','bold');
        ylabel('Var(a_m)  measured / formula','FontSize',18,'FontWeight','bold');
        legend(H,'Location','northoutside','FontSize',13,'FontWeight','bold','Box','on');
        set(gca,'FontSize',18,'FontWeight','bold','LineWidth',1.2,'Box','on'); grid off;
    end
end
nexttile(2);
xlim([0.40 1.02]); ylim([0.85 1.45]);
set(gca,'XTick',[0.4 0.5 0.6 0.7 0.8 0.9 1.0]);
gh=gca; gh.XAxis.Exponent=0; xtickformat('%.1f');
xlabel('a_{x,y} / a_o','FontSize',18,'FontWeight','bold');
legend(H2([1 2 3 4]),'Location','northoutside','FontSize',12,'FontWeight','bold','Box','on','NumColumns',2);
set(gca,'FontSize',18,'FontWeight','bold','LineWidth',1.2,'Box','on'); grid off;
exportgraphics(f,[root 'test_results/am_r22_deep/fig10_truea_vs_baseline.png'],'Resolution',150);
close(f);
fprintf('figure -> test_results/am_r22_deep/fig10_truea_vs_baseline.png\n');
