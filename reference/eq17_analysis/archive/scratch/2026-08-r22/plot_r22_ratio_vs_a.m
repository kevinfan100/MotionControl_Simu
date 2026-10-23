% STATUS: ACTIVE (scratch) | PURPOSE: show the near-wall departure on an axis
%   that can display it | EXPIRES: with the R22 audit
%
% fig0 plots the variances themselves on log-log over two decades, which is
% the right picture for "the (a+xi)^2 law holds" and the WRONG picture for
% "these two disagree by 28%" -- on that axis 28% is a line width. Ratio on a
% linear axis puts the same 28% across a third of the plot height.
% The near-wall departure, on an axis that can actually show it.
% fig0 is log-log over two decades, where a 28% departure is a line width.
% This is measured/theory on a LINEAR axis against a_bar, so a 28% departure
% is 28% of the plot height. Deep band, 400 seeds; the shallow 200-seed set is
% overlaid faint to show the band that could not reach the problem at all.
root = '/Users/kevin/Code/MotionControl_Simu-motion-test/';
D = load([root 'test_results/am_r22_deep/stack_deep400.mat']);
Sh = load([root 'test_results/am_r22/stack_200.mat']);
sets = {D, 'deep 400 seeds'; Sh, 'shallow 200 seeds'};
COL = {[0 0.2 0.9], [0.45 0.72 0.95]};
f = figure('Position',[80 80 1100 660],'Color','w','NumberTitle','off','Visible','off');
hold on;
yline(1,'-','Color',[0.35 0.35 0.35],'LineWidth',1.5,'HandleVisibility','off');
H = gobjects(0);
for si = 1:2
    S = sets{si,1};
    a_cov = S.cc.a_cov; A = S.A_wm; T = S.A_tr;
    Y = A(2:end,:) - (1-a_cov)*A(1:end-1,:);
    am = mean(T,2); am2 = am(2:end); n = size(A,2);
    xi = S.xi_bar(3); kT = S.kappa_T; s2n = S.s2n_nd(3);
    IFf = @(a) 1 + 2*(((kT*a).^2*S.cc.IF_abc(1) + 2*(kT*a)*s2n*S.cc.IF_abc(2) + ...
                        s2n^2*S.cc.IF_abc(3)) ./ ((S.cc.C_dpmr*kT*a + S.cc.C_n*s2n).^2));
    ed = linspace(min(am), max(am), 19);
    for q = 1:2
        if q == 1; X = A; ar = am; else; X = Y; ar = am2; end
        c=[]; r=[]; e=[];
        for b = 1:18
            in = ar >= ed(b) & ar < ed(b+1);
            if sum(in) < 20; continue; end
            Xi = X(in,:); S1 = sum(Xi,2); S2 = sum(Xi.^2,2);
            v = mean((S2 - S1.^2/n)/(n-1));
            vl = (S2 - Xi.^2 - (S1-Xi).^2/(n-1))/(n-2);
            th_jk = mean(vl,1); se = sqrt((n-1)/n*sum((th_jk-mean(th_jk)).^2));
            cc_ = mean(ar(in));
            if q == 1; th = S.cc.K_var*IFf(cc_)*(cc_+xi)^2; else; th = 2*a_cov^2*(cc_+xi)^2; end
            c(end+1)=cc_; r(end+1)=v/th; e(end+1)=se/th; %#ok<AGROW>
        end
        mk = {'o','s'}; ls = {'-','-'};
        % shallow band drawn washed out: same colour blended toward white
        col = COL{q}; if si == 2; col = 1 - 0.35*(1 - col); end
        h = errorbar(c, r, e, [mk{q} ls{q}], 'Color', col, ...
            'MarkerFaceColor', col, 'MarkerSize', 6, 'LineWidth', 1.6, 'CapSize', 3);
        if si == 1
            if q==1; h.DisplayName = 'Var(a_m) / formula   (deep)';
            else;    h.DisplayName = 'Var(y_2) / formula   (deep)'; end
            H(end+1) = h; %#ok<AGROW>
        elseif q == 1
            h.DisplayName = 'shallow band (both)'; H(end+1) = h; %#ok<AGROW>
        else
            h.HandleVisibility = 'off';
        end
    end
end
set(gca,'XScale','log'); xlim([0.075 1.02]); ylim([0.85 1.45]);
set(gca,'XTick',[0.1 0.15 0.2 0.3 0.5 0.7 0.9]);
gh = gca; gh.XAxis.Exponent = 0; xtickformat('%.2f');
xlabel('a_z / a_o','FontSize',18,'FontWeight','bold');
ylabel('measured / formula','FontSize',18,'FontWeight','bold');
legend(H,'Location','northoutside','Orientation','horizontal','FontSize',13,'FontWeight','bold','Box','on');
set(gca,'FontSize',18,'FontWeight','bold','LineWidth',1.2,'Box','on'); grid off;
exportgraphics(f,[root 'test_results/am_r22_deep/fig9_ratio_vs_a.png'],'Resolution',150);
close(f); fprintf('figure -> test_results/am_r22_deep/fig9_ratio_vs_a.png\n');
