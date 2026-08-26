% FORK OF test_script/integration/plot_formC_dist_compare.m @ 10e51db |
%   PURPOSE: the same house comparison page for the (a_pd, a_cov) arms, with
%   row 2 as the ABSOLUTE error instead of the percentage | EXPIRES: with the
%   a_pd / a_cov decision | production changes do NOT follow.
%
%   Rows, exactly the house layout:
%     1  gain tracking     a_true (red) / a_hat (blue) / a_m readout (light blue)
%     2  gain error        a_hat - a_true          <- ABSOLUTE, house uses %
%     3  b                 b_true at the particle (red) / b_hat (blue) + sqrt(P55)
%     4  tracking error    R*dw_3 [um]
%   Row y limits are shared across the two columns.
%
%   TWO ALIGNMENT FACTS, both inherited or found while forking:
%   (a) R*dw_3 = p_d_out[k] - p_true_out[k-1]. Differencing at the same index
%       adds one commanded step of bias -- the house caught this 2026-08-18
%       (28.455 nm on the canonical scenario). On the Meng ramp the descent is
%       10x slower so the offset is only 0.780 nm, but the aligned form is used
%       regardless.
%   (b) h_bar_true_out[k] == p_true_out[k-1]/R exactly (verified: shift -1 gives
%       max|diff| = 0). It is one sample behind p_true_out. Row 3 therefore
%       reads b_true at p_true_out(:,ax)/R, the house choice; using
%       h_bar_true_out instead shifts b_true by 6e-5, which is negligible here
%       but is a real logging inconsistency in the driver.
%
%   a_m is r.a_xm_out(:,ax)/ad with ad = a_hat_out(1,ax)/a_bar_hat_out(1,ax),
%   byte-identical to the house formula (checked).
function out = plot_arms_pair(ARMS, seed_idx, ax, tlim)

    if nargin < 2 || isempty(seed_idx); seed_idx = 1; end
    if nargin < 3 || isempty(ax);       ax = 3;       end
    if nargin < 4;                      tlim = [];    end

    pc = physical_constants();
    nA = size(ARMS,1);  d = cell(1,nA);
    for c = 1:nA
        r  = ARMS{c,1}.runs{seed_idx};
        ad = r.a_hat_out(1,ax) / r.a_bar_hat_out(1,ax);
        s.t  = r.tout(:);
        s.aT = r.a_true_out(:,ax) / ad;
        s.aH = r.a_bar_hat_out(:,ax);
        s.aM = r.a_xm_out(:,ax) / ad;
        s.e  = s.aH - s.aT;                                   % ABSOLUTE
        s.bH = r.b_hat_out(:,ax);
        s.sd = r.P_b_out(:,ax);                               % already a std
        s.bT = local_b_true(max(r.p_true_out(:,ax)/pc.R, 1.0005));
        s.dw = [NaN; r.p_d_out(2:end,ax) - r.p_true_out(1:end-1,ax)];
        d{c} = s;
    end
    if isempty(tlim); tlim = [d{1}.t(1) d{1}.t(end)]; end

    fprintf('\n%-34s %11s %11s %11s %11s\n','arm','e@2s','e@6s','e@10s','trkRMS nm');
    for c = 1:nA
        s = d{c};  k = @(tt) find(s.t>=tt,1);  m = s.t>0.5 & s.t<10.5;
        fprintf('%-34s %+11.5f %+11.5f %+11.5f %11.2f\n', ARMS{c,2}, ...
                s.e(k(2)), s.e(k(6)), s.e(k(10)), 1e3*rms(s.dw(m),'omitnan'));
    end
    fprintf('  b_hat[end] : ');  for c=1:nA; fprintf('%.5f  ', d{c}.bH(end)); end
    fprintf(' | b_true[end] %.5f\n', d{1}.bT(end));

    local_page(d, ARMS(:,2), ARMS{1,1}.seeds(seed_idx), tlim);
    out = d;
end

% ---------------------------------------------------------------------
function local_page(d, NAME, seed, tlim)
    COL_TRUE=[0.8 0 0]; COL_HAT=[0 0.2 0.9]; COL_MEAS=[0.45 0.72 0.95];
    BANDC=[0.45 0.55 0.95];
    FS=18; LFS=13; AXLW=2.0; LW=2.0;  nA=numel(d);
    YL=cell(4,1);
    YL{1}=local_lim([local_cat(d,'aT'); local_cat(d,'aH'); local_cat(d,'aM')], 0.05);
    YL{2}=local_lim(local_cat(d,'e'), 0.08);
    YL{3}=local_lim([local_cat(d,'bH')+local_cat(d,'sd'); ...
                     local_cat(d,'bH')-local_cat(d,'sd'); local_cat(d,'bT')], 0.10);
    YL{4}=local_lim(local_cat(d,'dw'), 0.08);

    f=figure('Position',[40 40 750*nA 1380],'Color','w','NumberTitle','off','Visible','off');
    tl=tiledlayout(f,4,nA,'TileSpacing','compact','Padding','compact');
    for row=1:4
        for c=1:nA
            s=d{c}; a=nexttile(tl,(row-1)*nA+c); hold(a,'on');
            switch row
                case 1
                    h0=plot(a,s.t,s.aM,'-','Color',COL_MEAS,'LineWidth',1.0,'DisplayName','a_m readout');
                    h1=plot(a,s.t,s.aT,'-','Color',COL_TRUE,'LineWidth',LW+0.6,'DisplayName','a_{true}');
                    h2=plot(a,s.t,s.aH,'-','Color',COL_HAT,'LineWidth',LW, ...
                            'DisplayName',sprintf('a_{hat}   %s   seed %d',NAME{c},seed));
                    legend(a,[h1 h2 h0],'Location','northoutside','Orientation','horizontal', ...
                           'FontSize',LFS,'FontWeight','bold','Box','on');
                    if c==1; ylabel(a,'a_z / a_o','FontSize',FS,'FontWeight','bold'); end
                case 2
                    yline(a,0,'-','Color',[0.4 0.4 0.4],'LineWidth',1.0,'HandleVisibility','off');
                    plot(a,s.t,s.e,'-','Color',COL_HAT,'LineWidth',LW);
                    if c==1; ylabel(a,{'a_z / a_o  error','(est - true)'},'FontSize',FS,'FontWeight','bold'); end
                case 3
                    fill(a,[s.t;flipud(s.t)],[s.bH+s.sd;flipud(s.bH-s.sd)],BANDC, ...
                         'EdgeColor','none','FaceAlpha',0.30,'DisplayName','\pm sqrt(P_{55})');
                    plot(a,s.t,s.bT,'-','Color',COL_TRUE,'LineWidth',LW,'DisplayName','b_{true} at the particle');
                    plot(a,s.t,s.bH,'-','Color',COL_HAT,'LineWidth',LW,'DisplayName','b_{hat}');
                    legend(a,'Location','northoutside','Orientation','horizontal', ...
                           'FontSize',LFS,'FontWeight','bold','Box','on');
                    if c==1; ylabel(a,'b','FontSize',FS,'FontWeight','bold'); end
                case 4
                    plot(a,s.t,s.dw,'-','Color',COL_HAT,'LineWidth',0.8);
                    yline(a,0,'-','Color',[0.5 0.5 0.5],'LineWidth',1.0);
                    if c==1; ylabel(a,'R \delta w_3   [\mum]','FontSize',FS,'FontWeight','bold'); end
                    xlabel(a,'time  [s]','FontSize',FS,'FontWeight','bold');
            end
            xlim(a,tlim); ylim(a,YL{row});
            set(a,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on', ...
                  'TickLabelInterpreter','tex'); grid(a,'off');
            if row<4;  set(a,'XTickLabel',[]); end
            if row==4; ytickformat(a,'%.2f'); end
            if c>1;    set(a,'YTickLabel',[]); end
        end
    end
    here=fileparts(mfilename('fullpath')); root=fileparts(fileparts(here));
    fn=fullfile(root,'test_results','apd_acov_meng',sprintf('arms_pair_s%03d.png',seed));
    exportgraphics(f,fn,'Resolution',150); close(f);
    fprintf('figure -> %s\n', fn);
end

function v=local_cat(d,f)
    v=[]; for c=1:numel(d); v=[v; d{c}.(f)]; end   %#ok<AGROW>
end

function L=local_lim(v,pad)
    v=v(isfinite(v)); lo=min(v); hi=max(v); r=hi-lo; if r<=0; r=1; end
    L=[lo-pad*r, hi+pad*r];
end

function b=local_b_true(w)
    persistent wq btr
    if isempty(wq)
        wq=linspace(1.0005,40,6000); cp=zeros(size(wq));
        for i=1:numel(wq); [~,c]=calc_correction_functions(wq(i)); cp(i)=c; end
        at=1./cp; btr=gradient(at,wq)./(1-at).^2;
    end
    b=interp1(wq,btr,w,'linear','extrap');
end
