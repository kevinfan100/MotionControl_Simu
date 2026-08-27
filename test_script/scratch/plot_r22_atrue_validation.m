% R22/y2 formula validated at a_TRUE (true-gain control-law arm), so nothing
% here is polluted by a_hat's own bias. Binned along a_true, log-log, three
% axes, jackknife bars over 100 seeds.
root = '/Users/kevin/Code/MotionControl_Simu-motion-test/';
T = load([root 'test_results/am_r22_deep/truea_100.mat']);
B = load([root 'test_results/formC_cdpmr_var_check/raw_seeds.mat']);
K = T.K; a_nom = T.a_nom; R = B.S400.K.R; a_cov = K.a_cov;
% IF_abc rebuilt from the CURRENT builder, not the copy saved in the .mat
% (2026-08-27): abs = s-weighted (EWMA-output IF_eff, the pre-fix model),
% abw = s = 1 (whitened IF_white, the fix). Both plotted in panel (b).
Kc = build_eq17_6state_constants(struct('lambda_c', K.lambda_c, 'sigma2_n_s', B.S400.K.sigma2_n_s(:), ...
        'kBT', B.S400.K.kBT, 'a_cov', a_cov, 'a_pd', K.a_pd, 'd', K.d));
abs_ = Kc.IF_abc(:); abw = Kc.IF_abc_white(:); ab = abw;
fprintf('IF_abc (s=1-a_cov) = %s\nIF_abc_white (s=1) = %s\n', mat2str(abs_', 6), mat2str(abw', 6));
kT = 4*(B.S400.K.kBT/R)*B.S400.K.a_o;
axl = 'xyz'; COL = [0.85 0.33 0.10; 0.47 0.67 0.19; 0 0.2 0.9];

f = figure('Position',[80 80 1300 600],'Color','w','NumberTitle','off','Visible','off');
tiledlayout(1,2,'TileSpacing','compact','Padding','compact');

H1 = gobjects(0); H2 = gobjects(0);
for ax = 1:3
    s2n = B.S400.K.sigma2_n_s(ax)/R^2; xi = (K.C_n/K.C_dpmr)*s2n/kT;
    A = squeeze(T.A_xm(:,ax,:))/a_nom;  A=A(2:end,:);
    AT= squeeze(T.A_tr(:,ax,:))/a_nom;  AT=AT(2:end,:);
    Y = A(2:end,:) - (1-a_cov)*A(1:end-1,:);
    am = mean(AT,2); am2 = am(2:end); n = size(A,2);
    ifg = @(a, v) 1 + 2*(((kT*a).^2*v(1) + 2*(kT*a)*s2n*v(2) + s2n^2*v(3)) ./ ...
                      ((K.C_dpmr*kT*a + K.C_n*s2n).^2));
    ife = @(a) ifg(a, abw);          % model = IF_white (s = 1), the fix
    ifs = @(a) ifg(a, abs_);         % pre-fix model = IF_eff (s = 1-a_cov)
    ed = linspace(min(am), max(am), 13);
    c1=[]; r1=[]; e1=[];   % (a) per-sample Var(y2)/formula
    c2=[]; r2=[]; e2=[];   % (b) IF: measured/model
    r2s=[];                % (b) same measurement over the pre-fix model IF_eff(s)
    for b = 1:12
        i2 = am2 >= ed(b) & am2 < ed(b+1);
        if sum(i2) < 30; continue; end
        cc = mean(am2(i2));
        Yi = Y(i2,:); S1v = sum(Yi,2); S2v = sum(Yi.^2,2);
        v = mean((S2v - S1v.^2/n)/(n-1));
        vl = (S2v - Yi.^2 - (S1v-Yi).^2/(n-1))/(n-2);
        th = mean(vl,1); se = sqrt((n-1)/n*sum((th-mean(th)).^2));
        thform = 2*a_cov^2*(cc+xi)^2;
        c1(end+1)=cc; r1(end+1)=v/thform; e1(end+1)=se/thform; %#ok<AGROW>
    end
    % IF measured via integrated acf, per-window (coarser, 4 windows -> stable acf)
    t2 = T.t(3:end);
    Wd = {[0.5 1.5],'descent'; [1.75 3.5],'osc'; [3.75 4.8],'hold'};
    for i = 1:3
        in = t2>=Wd{i,1}(1) & t2<=Wd{i,1}(2);
        cc = mean(am2(in));
        IFm = local_acf_int(Y(in,:), 40);
        IFf = ife(cc);
        c2(end+1)=cc; r2(end+1)=IFm/IFf; r2s(end+1)=IFm/ifs(cc); %#ok<AGROW>
        fprintf('axis %c %-8s a=%.3f  IF meas %.3f  IF_eff(s) %.3f  IF_white %.3f  meas/s %.3f  meas/white %.3f\n', ...
                axl(ax), Wd{i,2}, cc, IFm, ifs(cc), IFf, IFm/ifs(cc), IFm/IFf);
    end
    V3(ax) = struct('c', c2, 'r_white', r2, 'r_s', r2s, 'win', {Wd(:,2)'}); %#ok<AGROW,NASGU>
    nexttile(1); hold on;
    h = errorbar(c1, r1, e1, 'o-', 'Color', COL(ax,:), 'MarkerFaceColor', COL(ax,:), ...
        'MarkerSize',6,'LineWidth',1.6,'CapSize',3, 'DisplayName', sprintf('axis %c', axl(ax)));
    H1(end+1) = h; %#ok<AGROW>
    nexttile(2); hold on;
    plot(c2, r2s, 'o--', 'Color', COL(ax,:), 'MarkerSize',8,'LineWidth',1.2, ...
        'HandleVisibility','off');                       % pre-fix model IF_eff(s), hollow
    h2 = plot(c2, r2, 'o-', 'Color', COL(ax,:), 'MarkerFaceColor', COL(ax,:), ...
        'MarkerSize',8,'LineWidth',1.8, 'DisplayName', sprintf('axis %c', axl(ax)));
    H2(end+1) = h2; %#ok<AGROW>
end
nexttile(1);
yline(1,'-','Color',[0.4 0.4 0.4],'LineWidth',1.2,'HandleVisibility','off');
xlabel('a_{true}','FontSize',16,'FontWeight','bold');
ylabel('Var(y_2) measured / per-sample formula','FontSize',15,'FontWeight','bold');
title('(a) per-sample form: 2a_{cov}^2(a+\xi)^2','FontSize',13);
legend(H1,'Location','northoutside','FontSize',12,'FontWeight','bold','Box','on','NumColumns',3);
set(gca,'FontSize',15,'FontWeight','bold','LineWidth',1.2,'Box','on'); grid off; ylim([0.85 1.25]);

nexttile(2);
yline(1,'-','Color',[0.4 0.4 0.4],'LineWidth',1.2,'HandleVisibility','off');
xlabel('a_{true} (by window: descent/osc/hold)','FontSize',15,'FontWeight','bold');
ylabel('IF measured / IF model','FontSize',15,'FontWeight','bold');
title('(b) filled: IF_{white} (s=1, fix);  hollow: IF_{eff} (s=1-a_{cov}, pre-fix)','FontSize',12);
legend(H2,'Location','northoutside','FontSize',12,'FontWeight','bold','Box','on','NumColumns',3);
set(gca,'FontSize',15,'FontWeight','bold','LineWidth',1.2,'Box','on'); grid off; ylim([0.85 1.25]);

exportgraphics(f, [root 'test_results/am_r22_deep/fig11_r22_atrue_validation.png'], 'Resolution', 150);
close(f);
fprintf('figure -> test_results/am_r22_deep/fig11_r22_atrue_validation.png\n');

function f = local_acf_int(X, L)
    Xf = X-mean(X,2); n=size(Xf,1); r=zeros(1,L);
    for k=1:L
        a=Xf(1:n-k,:); b=Xf(1+k:n,:); a=a(:)-mean(a(:)); b=b(:)-mean(b(:));
        r(k)=(a'*b)/sqrt((a'*a)*(b'*b));
    end
    f = 1+2*sum(r);
end
