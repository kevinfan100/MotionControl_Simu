% Match the reference figure style: absolute-value overlay (not ratio),
% time axis (raw, unsmoothed) + a-binned with error bars. Z AXIS ONLY.
% Row 1: Var(y2) vs the per-sample formula 2*a_cov^2*(a+xi)^2 -- this is
%   the half of R22 that SHOULD overlay cleanly (no IF baked in).
% Row 2: IF_eff -- NOT the same kind of quantity (it's a per-window scalar
%   from an integrated autocorrelation, not a time-indexed variance), so it
%   keeps the ratio-vs-a style instead of an absolute overlay -- plotted as
%   THE MODEL CURVE ITSELF (not constant -- shown varying with a) with the
%   measured points on top, so the reader can see directly that IF_eff is a
%   function of a, and see where data sits relative to it.
root = '/Users/kevin/Code/MotionControl_Simu-motion-test/';
T = load([root 'test_results/am_r22_deep/truea_100.mat']);
B = load([root 'test_results/formC_cdpmr_var_check/raw_seeds.mat']);
K = T.K; a_nom = T.a_nom; R = B.S400.K.R; a_cov = K.a_cov; ab = K.IF_abc(:);
kT = 4*(B.S400.K.kBT/R)*B.S400.K.a_o;
ax = 3;   % z axis only
s2n = B.S400.K.sigma2_n_s(ax)/R^2; xi = (K.C_n/K.C_dpmr)*s2n/kT;
ife = @(a) 1 + 2*(((kT*a).^2*ab(1) + 2*(kT*a)*s2n*ab(2) + s2n^2*ab(3)) ./ ...
                  ((K.C_dpmr*kT*a + K.C_n*s2n).^2));

A = squeeze(T.A_xm(:,ax,:))/a_nom;  A=A(2:end,:);
AT= squeeze(T.A_tr(:,ax,:))/a_nom;  AT=AT(2:end,:);
t = T.t(2:end);
Y = A(2:end,:) - (1-a_cov)*A(1:end-1,:);  t2 = t(2:end);
am = mean(AT,2); am2 = am(2:end); n = size(A,2);

% ---- time axis: raw cross-seed Var(y2) vs formula, unsmoothed ----
vy_meas_t = var(Y,0,2);
vy_form_t = 2*a_cov^2*(am2+xi).^2;

% ---- a-binned ----
ed = linspace(min(am2), max(am2), 15);
cB=[]; vB=[]; eB=[];
for b = 1:14
    in = am2 >= ed(b) & am2 < ed(b+1);
    if sum(in) < 30; continue; end
    Yi = Y(in,:); S1=sum(Yi,2); S2=sum(Yi.^2,2);
    v = mean((S2-S1.^2/n)/(n-1));
    vl = (S2-Yi.^2-(S1-Yi).^2/(n-1))/(n-2);
    th = mean(vl,1); se = sqrt((n-1)/n*sum((th-mean(th)).^2));
    cB(end+1)=mean(am2(in)); vB(end+1)=v; eB(end+1)=se; %#ok<AGROW>
end
a_grid = linspace(min(am2),max(am2),200)';
form_grid = 2*a_cov^2*(a_grid+xi).^2;

% ---- IF_eff panel: model curve (showing it is NOT constant) + measured ----
Wd = {[0.5 1.5],'descent'; [1.75 3.5],'osc'; [3.75 4.8],'hold'};
cI=[]; IFmeas=[];
for i=1:3
    in = t2>=Wd{i,1}(1) & t2<=Wd{i,1}(2);
    cI(end+1) = mean(am2(in)); %#ok<AGROW>
    IFmeas(end+1) = local_acf_int(Y(in,:),40); %#ok<AGROW>
end
IF_grid = ife(a_grid);

% ================= FIGURE =================
f = figure('Position',[80 80 1500 850],'Color','w','NumberTitle','off','Visible','off');
tiledlayout(2,2,'TileSpacing','compact','Padding','compact');

nexttile(1); hold on;
h1 = plot(t2, vy_form_t, '-', 'Color',[0 0.55 0.2], 'LineWidth',2.5, 'DisplayName','formula');
h2 = plot(t2, vy_meas_t, '-', 'Color',[0.2 0.4 0.9], 'LineWidth',0.6, 'DisplayName','measured (cross-seed var, N=100)');
legend([h1 h2],'Location','northoutside','FontSize',12,'FontWeight','bold','Box','on');
xlabel('time [s]','FontSize',15,'FontWeight','bold');
ylabel('Var(y_2)  [-]','FontSize',15,'FontWeight','bold');
title('z axis -- time series (raw, unsmoothed)','FontSize',12);
set(gca,'FontSize',14,'FontWeight','bold','LineWidth',1.2,'Box','on'); grid off;

nexttile(2); hold on;
h3 = plot(a_grid, form_grid, '-', 'Color',[0 0.55 0.2], 'LineWidth',2.5, 'DisplayName','formula');
h4 = errorbar(cB, vB, eB, 'o', 'Color',[0.2 0.4 0.9], 'MarkerFaceColor',[0.2 0.4 0.9], ...
    'MarkerSize',7,'LineWidth',1.4,'CapSize',3, 'DisplayName','measured (cross-seed var, N=100)');
legend([h3 h4],'Location','northoutside','FontSize',12,'FontWeight','bold','Box','on');
xlabel('a_z / a_o  [-]','FontSize',15,'FontWeight','bold');
ylabel('Var(y_2)  [-]','FontSize',15,'FontWeight','bold');
title('z axis -- binned along a_{true}','FontSize',12);
set(gca,'FontSize',14,'FontWeight','bold','LineWidth',1.2,'Box','on'); grid off;

nexttile(3); hold on;
h5 = plot(a_grid, IF_grid, '-', 'Color',[0 0.55 0.2], 'LineWidth',2.5, 'DisplayName','IF_{eff} model (NOT constant)');
h6 = plot(cI, IFmeas, 'o', 'Color',[0.85 0.1 0.1], 'MarkerFaceColor',[0.85 0.1 0.1], ...
    'MarkerSize',9,'LineWidth',1.6, 'DisplayName','IF measured (integrated acf)');
legend([h5 h6],'Location','northoutside','FontSize',12,'FontWeight','bold','Box','on');
xlabel('a_z / a_o  [-]','FontSize',15,'FontWeight','bold');
ylabel('IF_{eff}  [-]','FontSize',15,'FontWeight','bold');
title('IF_{eff}: model varies with a, data sits ~10-16% above it','FontSize',12);
ylim([2.5 4.2]);
set(gca,'FontSize',14,'FontWeight','bold','LineWidth',1.2,'Box','on'); grid off;

nexttile(4); hold on;
ratio = IFmeas./ife(cI);
h7 = plot(cI, ratio, 'o-', 'Color',[0.85 0.1 0.1], 'MarkerFaceColor',[0.85 0.1 0.1], ...
    'MarkerSize',9,'LineWidth',1.6, 'DisplayName','IF measured / IF_{eff} model');
yline(1,'-','Color',[0.4 0.4 0.4],'LineWidth',1.2,'HandleVisibility','off');
legend(h7,'Location','northoutside','FontSize',12,'FontWeight','bold','Box','on');
xlabel('a_z / a_o  [-]','FontSize',15,'FontWeight','bold');
ylabel('measured / model','FontSize',15,'FontWeight','bold');
title('the isolated defect: 10-16%, flat across a','FontSize',12);
ylim([0.95 1.25]);
set(gca,'FontSize',14,'FontWeight','bold','LineWidth',1.2,'Box','on'); grid off;

exportgraphics(f, [root 'test_results/am_r22_deep/fig12_r22_z_styled.png'], 'Resolution', 150);
close(f);
fprintf('figure -> test_results/am_r22_deep/fig12_r22_z_styled.png\n');

function f = local_acf_int(X, L)
    Xf = X-mean(X,2); n=size(Xf,1); r=zeros(1,L);
    for k=1:L
        a=Xf(1:n-k,:); b=Xf(1+k:n,:); a=a(:)-mean(a(:)); b=b(:)-mean(b(:));
        r(k)=(a'*b)/sqrt((a'*a)*(b'*b));
    end
    f = 1+2*sum(r);
end
