% STATUS: ACTIVE (scratch figure) | PURPOSE: ONE window of real simulation
%   data walked down the gain-readout pipeline, one panel per processing step,
%   all sharing the same time axis, so each stage can be compared against the
%   stage above it. Companion to rebuild_am_chain.m (which proves the offline
%   chain is bit-exact) and walk_one_sample_to_am.m (single-sample influence).
%
%   Every operation drawn here is CAUSAL and per-step: the LP and the EWMA are
%   one-line recursions the controller runs at 1600 Hz, not offline filters.
%
%   Rows (= the seven steps):
%     1  p_d[k-d] and p_m[k]              the two raw signals, in um
%     2  dw_m = (p_d[k-d] - p_m[k]) / R   steps 1-2, drawn in nm
%     3  dw_m with dw_bar = LP(a_pd)      step 3, the deterministic part
%     4  dw_r = dw_m - dw_bar             step 4, the jiggle
%     5  dw_r^2                           step 5a, one sample's contribution
%     6  sigma2 = EWMA(a_cov) of dw_r^2   step 5c, with the C_n*sigma2_n floor
%                                         drawn as the line step 6 subtracts
%     7  a_m = (sigma2 - C_n*s2n)/(C_dpmr*kappa_T)   step 7, against a_true
%
%   nwin = 0 -> FULL-RUN mode: the whole run on one time axis. Rows 2, 4 and 5
%   become solid bands at that density, so rows 2 and 4 carry a moving +-sd
%   envelope drawn on top. THE ENVELOPE IS A READING AID, NOT PART OF THE
%   CHAIN -- the controller never computes it; it is there because the thing
%   worth seeing at full-run scale is how the band narrows as the wall is
%   approached, and a solid band cannot show that.
function out = plot_am_stepwise(r, ax, k0, nwin)

    if nargin < 2 || isempty(ax);   ax = 3;    end
    if nargin < 3 || isempty(k0);   k0 = 5000; end
    if nargin < 4 || isempty(nwin); nwin = 400; end   % steps shown; 0 = full run

    cc = r.ctrl_const;  P = r.meta.params_value;
    a_disp = r.a_hat_out(1, ax) / r.a_bar_hat_out(1, ax);   % = a_o*R
    a_o = a_disp / r.R;
    kT  = 4 * (P.ctrl.k_B * P.ctrl.T / r.R) * a_o;
    s2n = P.ctrl.sigma2_noise(ax) / r.R^2;
    Cn_s2n = cc.C_n * s2n;  den = cc.C_dpmr * kT;

    dw = r.dh_m_out(:, ax) / r.R;
    aM = r.a_xm_out(:, ax) / a_disp;
    N  = numel(dw);
    bar = zeros(N,1); s2 = zeros(N,1); am = nan(N,1);
    bar(1) = dw(1);  s2(1) = aM(1)*den + Cn_s2n;  am(1) = (s2(1)-Cn_s2n)/den;
    for k = 2:N
        bar(k) = (1-cc.a_pd)*bar(k-1) + cc.a_pd*dw(k);
        s2(k)  = (1-cc.a_cov)*s2(k-1) + cc.a_cov*(dw(k)-bar(k))^2;
        am(k)  = (s2(k)-Cn_s2n)/den;
    end
    assert(max(abs((am(2000:end)-aM(2000:end))./aM(2000:end))) < 1e-12, ...
           'plot_am_stepwise:chain', 'offline chain no longer matches the controller');

    full_run = (nwin <= 0);
    if full_run
        i = (2:N).';                                  % row 1 is init-only
        t = r.tout(i);                                % seconds, absolute
    else
        i = (k0 - round(nwin/2)) : (k0 + round(nwin/2));
        i = i(i >= 1 & i <= N);
        t = (r.tout(i) - r.tout(k0)) * 1e3;           % ms, 0 = the marked sample
    end
    aT = r.a_true_out(:, ax) / a_disp;

    out = struct('i', i, 't', t, 'dw', dw, 'bar', bar, 's2', s2, 'am', am, ...
                 'aT', aT, 'Cn_s2n', Cn_s2n, 'den', den, 'k0', k0, 'R', r.R, ...
                 'a_pd', cc.a_pd, 'a_cov', cc.a_cov, 'full_run', full_run);
    if full_run
        local_page_full(out, r, ax, i, t, aT, cc);
    else
        local_page(out, r, ax, i, t, aT, cc, a_disp);
    end
end

% ---------------------------------------------------------------------
function local_page(o, r, ax, i, t, aT, cc, a_disp)
    FS=14; LFS=10.5; AXLW=1.7; k0=o.k0; R=o.R;
    COL_T=[0.85 0.10 0.10]; COL_E=[0.00 0.20 0.90]; COL_M=[0.45 0.72 0.95];
    COL_O=[0.85 0.33 0.10]; COL_G=[0.55 0.55 0.55];
    f=figure('Position',[20 20 1350 2050],'Color','w','NumberTitle','off','Visible','off');
    tl=tiledlayout(f,7,1,'TileSpacing','compact','Padding','compact');
    A=gobjects(7,1);
    mk=@(a,y) plot(a,0,y,'o','Color',[0 0 0],'MarkerFaceColor',[1 1 0], ...
                   'MarkerSize',10,'LineWidth',1.5,'HandleVisibility','off');

    % --- 1 : the two raw signals -------------------------------------
    a=nexttile(tl); A(1)=a; hold(a,'on');
    pd = r.p_d_out(i, ax);  pm = r.p_m_out(i, ax);
    h1=plot(a,t,pd,'-','Color',COL_T,'LineWidth',2.2);
    h2=plot(a,t,pm,'-','Color',COL_M,'LineWidth',1.2);
    legend(a,[h1 h2],{'p_d[k-d]  commanded','p_m[k]  measured'}, ...
        'Location','northoutside','Orientation','horizontal','FontSize',LFS,'Box','on');
    ylabel(a,{'step 0','p  [\mum]'},'FontSize',FS,'FontWeight','bold');

    % --- 2 : dw_m ------------------------------------------------------
    a=nexttile(tl); A(2)=a; hold(a,'on');
    yline(a,0,'-','Color',COL_G,'LineWidth',1.0,'HandleVisibility','off');
    h1=plot(a,t,o.dw(i)*R*1e3,'-','Color',COL_M,'LineWidth',1.2);
    mk(a,o.dw(k0)*R*1e3);
    legend(a,h1,{'dw_m = ( p_d[k-d] - p_m[k] ) / R      (this is also y_1)'}, ...
        'Location','northoutside','Orientation','horizontal','FontSize',LFS,'Box','on');
    ylabel(a,{'steps 1-2','dw_m  [nm]'},'FontSize',FS,'FontWeight','bold');

    % --- 3 : the LP ----------------------------------------------------
    a=nexttile(tl); A(3)=a; hold(a,'on');
    yline(a,0,'-','Color',COL_G,'LineWidth',1.0,'HandleVisibility','off');
    h1=plot(a,t,o.dw(i)*R*1e3,'-','Color',[0.80 0.86 0.94],'LineWidth',1.0);
    h2=plot(a,t,o.bar(i)*R*1e3,'-','Color',COL_T,'LineWidth',2.6);
    mk(a,o.bar(k0)*R*1e3);
    legend(a,[h1 h2],{'dw_m  (from above)', ...
        sprintf('dw_{bar}[k] = (1-a_{pd})dw_{bar}[k-1] + a_{pd} dw_m[k],  a_{pd} = %.3g', cc.a_pd)}, ...
        'Location','northoutside','Orientation','horizontal','FontSize',LFS,'Box','on');
    ylabel(a,{'step 3','dw_{bar}  [nm]'},'FontSize',FS,'FontWeight','bold');

    % --- 4 : the residual ---------------------------------------------
    a=nexttile(tl); A(4)=a; hold(a,'on');
    yline(a,0,'-','Color',COL_G,'LineWidth',1.0,'HandleVisibility','off');
    h1=plot(a,t,(o.dw(i)-o.bar(i))*R*1e3,'-','Color',COL_E,'LineWidth',1.2);
    mk(a,(o.dw(k0)-o.bar(k0))*R*1e3);
    legend(a,h1,{'dw_r = dw_m - dw_{bar}      the jiggle, measured from the moving mean'}, ...
        'Location','northoutside','Orientation','horizontal','FontSize',LFS,'Box','on');
    ylabel(a,{'step 4','dw_r  [nm]'},'FontSize',FS,'FontWeight','bold');

    % --- 5 : the square ------------------------------------------------
    a=nexttile(tl); A(5)=a; hold(a,'on');
    h1=plot(a,t,(o.dw(i)-o.bar(i)).^2,'-','Color',COL_G,'LineWidth',0.9);
    mk(a,(o.dw(k0)-o.bar(k0))^2);
    set(a,'YScale','log');
    legend(a,h1,{'dw_r^2      one sample''s contribution -- note the log axis'}, ...
        'Location','northoutside','Orientation','horizontal','FontSize',LFS,'Box','on');
    ylabel(a,{'step 5a','dw_r^2  [-]'},'FontSize',FS,'FontWeight','bold');

    % --- 6 : the EWMA + the noise floor --------------------------------
    a=nexttile(tl); A(6)=a; hold(a,'on');
    h1=plot(a,t,(o.dw(i)-o.bar(i)).^2,'-','Color',[0.82 0.82 0.82],'LineWidth',0.8);
    h2=plot(a,t,o.s2(i),'-','Color',COL_O,'LineWidth',2.6);
    h3=yline(a,o.Cn_s2n,'--','Color',[0.10 0.55 0.15],'LineWidth',2.0);
    mk(a,o.s2(k0));
    set(a,'YScale','log');
    legend(a,[h1 h2 h3],{'dw_r^2  (from above)', ...
        sprintf('\\sigma^2[k] = (1-a_{cov})\\sigma^2[k-1] + a_{cov} dw_r^2[k],  a_{cov} = %.3g', cc.a_cov), ...
        'C_n \sigma^2_n   (step 6 subtracts this)'}, ...
        'Location','northoutside','Orientation','horizontal','FontSize',LFS,'Box','on');
    ylabel(a,{'steps 5c, 6','\sigma^2  [-]'},'FontSize',FS,'FontWeight','bold');
    ylim(a,[1e-11 1e-2]);

    % --- 7 : a_m -------------------------------------------------------
    a=nexttile(tl); A(7)=a; hold(a,'on');
    h1=plot(a,t,aT(i),'-','Color',COL_T,'LineWidth',2.6);
    h2=plot(a,t,o.am(i),'-','Color',COL_M,'LineWidth',1.3);
    h3=plot(a,t,r.a_bar_hat_out(i,ax),'-','Color',COL_E,'LineWidth',2.0);
    mk(a,o.am(k0));
    legend(a,[h1 h2 h3],{'a_{true}', ...
        'a_m = ( \sigma^2 - C_n\sigma^2_n ) / ( C_{dpmr}\kappa_T )', ...
        'a_{hat}  (what the KF makes of it)'}, ...
        'Location','northoutside','Orientation','horizontal','FontSize',LFS,'Box','on');
    ylabel(a,{'step 7','a_z / a_o'},'FontSize',FS,'FontWeight','bold');
    xlabel(a,sprintf('time  [ms]     (0 = k %d, t = %.4f s)', k0, r.tout(k0)), ...
           'FontSize',FS,'FontWeight','bold');

    for q=1:7
        set(A(q),'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on');
        xlim(A(q),[t(1) t(end)]);
        xline(A(q),0,'-','Color',[0.45 0.45 0.45],'LineWidth',1.4,'HandleVisibility','off');
        if q < 7; set(A(q),'XTickLabel',[]); end
    end

    here=fileparts(mfilename('fullpath')); root=fileparts(fileparts(here));
    od=fullfile(root,'test_results','apd_acov_meng');
    if ~exist(od,'dir'); mkdir(od); end
    fn=fullfile(od,sprintf('am_stepwise_k%05d.png',k0));
    exportgraphics(f,fn,'Resolution',150); close(f);
    fprintf('figure -> %s\n', fn);
end

% ---------------------------------------------------------------------
function local_page_full(o, r, ax, i, t, aT, cc)
%LOCAL_PAGE_FULL  The whole run, one panel per step, one shared time axis.
    FS=14; LFS=10.5; AXLW=1.7; R=o.R; ENV=160;   % 160 steps = 0.1 s envelope
    COL_T=[0.85 0.10 0.10]; COL_E=[0.00 0.20 0.90]; COL_M=[0.45 0.72 0.95];
    COL_O=[0.85 0.33 0.10]; COL_G=[0.55 0.55 0.55]; COL_F=[0.78 0.86 0.94];
    dwn = o.dw(i);  barn = o.bar(i);  dwr = dwn - barn;  s2n_ = o.s2(i);
    f=figure('Position',[20 20 1500 2050],'Color','w','NumberTitle','off','Visible','off');
    tl=tiledlayout(f,7,1,'TileSpacing','compact','Padding','compact');
    A=gobjects(7,1);

    % --- 1 : the two raw signals, with h_bar on the right --------------
    a=nexttile(tl); A(1)=a; hold(a,'on');
    h1=plot(a,t,r.p_d_out(i,ax),'-','Color',COL_T,'LineWidth',2.2);
    h2=plot(a,t,r.p_m_out(i,ax),'-','Color',COL_M,'LineWidth',0.6);
    legend(a,[h1 h2],{'p_d[k-d]  commanded','p_m[k]  measured'}, ...
        'Location','northoutside','Orientation','horizontal','FontSize',LFS,'Box','on');
    ylabel(a,{'step 0','p  [\mum]'},'FontSize',FS,'FontWeight','bold');
    ylim(a,[0 16]);
    yyaxis(a,'right'); set(a,'YColor',[0.35 0.35 0.35]);
    ylim(a,[0 16]/R);                       % w_bar = p/R, locked to the left axis
    ylabel(a,'w-bar  [-]','FontSize',FS-2,'FontWeight','bold'); yyaxis(a,'left');

    % --- 2 : dw_m ------------------------------------------------------
    a=nexttile(tl); A(2)=a; hold(a,'on');
    yline(a,0,'-','Color',COL_G,'LineWidth',1.0,'HandleVisibility','off');
    h1=plot(a,t,dwn*R*1e3,'-','Color',COL_M,'LineWidth',0.4);
    e=movstd(dwn,ENV)*R*1e3;
    h2=plot(a,t, e,'-','Color',[0.05 0.15 0.55],'LineWidth',2.2);
         plot(a,t,-e,'-','Color',[0.05 0.15 0.55],'LineWidth',2.2,'HandleVisibility','off');
    legend(a,[h1 h2],{'dw_m = ( p_d[k-d] - p_m[k] ) / R      (this is also y_1)', ...
        '\pm moving sd, 0.1 s   (reading aid, not part of the chain)'}, ...
        'Location','northoutside','Orientation','horizontal','FontSize',LFS,'Box','on');
    ylabel(a,{'steps 1-2','dw_m  [nm]'},'FontSize',FS,'FontWeight','bold');

    % --- 3 : the LP ----------------------------------------------------
    a=nexttile(tl); A(3)=a; hold(a,'on');
    yline(a,0,'-','Color',COL_G,'LineWidth',1.0,'HandleVisibility','off');
    h1=plot(a,t,dwn*R*1e3,'-','Color',COL_F,'LineWidth',0.4);
    h2=plot(a,t,barn*R*1e3,'-','Color',COL_T,'LineWidth',2.0);
    legend(a,[h1 h2],{'dw_m  (from above)', ...
        sprintf('dw_{bar}[k] = (1-a_{pd})dw_{bar}[k-1] + a_{pd} dw_m[k],  a_{pd} = %.3g', cc.a_pd)}, ...
        'Location','northoutside','Orientation','horizontal','FontSize',LFS,'Box','on');
    ylabel(a,{'step 3','dw_{bar}  [nm]'},'FontSize',FS,'FontWeight','bold');

    % --- 4 : the residual ---------------------------------------------
    a=nexttile(tl); A(4)=a; hold(a,'on');
    yline(a,0,'-','Color',COL_G,'LineWidth',1.0,'HandleVisibility','off');
    h1=plot(a,t,dwr*R*1e3,'-','Color',[0.55 0.70 0.92],'LineWidth',0.4);
    e=movstd(dwr,ENV)*R*1e3;
    h2=plot(a,t, e,'-','Color',COL_E,'LineWidth',2.2);
         plot(a,t,-e,'-','Color',COL_E,'LineWidth',2.2,'HandleVisibility','off');
    legend(a,[h1 h2],{'dw_r = dw_m - dw_{bar}', ...
        '\pm moving sd, 0.1 s   (reading aid)'}, ...
        'Location','northoutside','Orientation','horizontal','FontSize',LFS,'Box','on');
    ylabel(a,{'step 4','dw_r  [nm]'},'FontSize',FS,'FontWeight','bold');

    % --- 5 : the square ------------------------------------------------
    a=nexttile(tl); A(5)=a; hold(a,'on');
    h1=plot(a,t,dwr.^2,'-','Color',COL_G,'LineWidth',0.3);
    set(a,'YScale','log');
    legend(a,h1,{'dw_r^2      one sample''s contribution -- log axis, six decades of spread'}, ...
        'Location','northoutside','Orientation','horizontal','FontSize',LFS,'Box','on');
    ylabel(a,{'step 5a','dw_r^2  [-]'},'FontSize',FS,'FontWeight','bold');
    ylim(a,[1e-11 1e-2]);

    % --- 6 : the EWMA + the noise floor --------------------------------
    a=nexttile(tl); A(6)=a; hold(a,'on');
    h1=plot(a,t,dwr.^2,'-','Color',[0.86 0.86 0.86],'LineWidth',0.3);
    h2=plot(a,t,s2n_,'-','Color',COL_O,'LineWidth',2.4);
    h3=yline(a,o.Cn_s2n,'--','Color',[0.10 0.55 0.15],'LineWidth',2.0);
    set(a,'YScale','log');
    legend(a,[h1 h2 h3],{'dw_r^2  (from above)', ...
        sprintf('\\sigma^2[k] = (1-a_{cov})\\sigma^2[k-1] + a_{cov} dw_r^2[k],  a_{cov} = %.3g', cc.a_cov), ...
        'C_n \sigma^2_n   (step 6 subtracts this)'}, ...
        'Location','northoutside','Orientation','horizontal','FontSize',LFS,'Box','on');
    ylabel(a,{'steps 5c, 6','\sigma^2  [-]'},'FontSize',FS,'FontWeight','bold');
    ylim(a,[1e-11 1e-2]);

    % --- 7 : a_m -------------------------------------------------------
    a=nexttile(tl); A(7)=a; hold(a,'on');
    h2=plot(a,t,o.am(i),'-','Color',COL_M,'LineWidth',0.5);
    h1=plot(a,t,aT(i),'-','Color',COL_T,'LineWidth',2.6);
    h3=plot(a,t,r.a_bar_hat_out(i,ax),'-','Color',COL_E,'LineWidth',2.0);
    legend(a,[h1 h2 h3],{'a_{true}', ...
        'a_m = ( \sigma^2 - C_n\sigma^2_n ) / ( C_{dpmr}\kappa_T )', ...
        'a_{hat}  (what the KF makes of it)'}, ...
        'Location','northoutside','Orientation','horizontal','FontSize',LFS,'Box','on');
    ylabel(a,{'step 7','a_z / a_o'},'FontSize',FS,'FontWeight','bold');
    ylim(a,[0 2]);   % a_m spikes past 2 are CLIPPED; the tail is long by construction
    xlabel(a,'time  [s]','FontSize',FS,'FontWeight','bold');

    % phase boundaries, on every panel
    tb = [0.5, 10.5, 11.5];
    for q=1:7
        set(A(q),'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on');
        xlim(A(q),[t(1) t(end)]);
        for b=tb; xline(A(q),b,':','Color',[0.45 0.45 0.45],'LineWidth',1.6, ...
                        'HandleVisibility','off'); end
        if q < 7; set(A(q),'XTickLabel',[]); end
    end

    here=fileparts(mfilename('fullpath')); root=fileparts(fileparts(here));
    od=fullfile(root,'test_results','apd_acov_meng');
    if ~exist(od,'dir'); mkdir(od); end
    fn=fullfile(od,'am_stepwise_fullrun.png');
    exportgraphics(f,fn,'Resolution',150); close(f);
    fprintf('figure -> %s\n', fn);
end
