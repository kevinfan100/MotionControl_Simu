% STATUS: ACTIVE (scratch figure) | PURPOSE: how much y2 actually contributes
%   to a-hat, shown two ways side by side because the two ways disagree.
%
%   NOTATION follows Meng/Long/Menq, IEEE TIE 72(1) 2025, eqs (16)-(19):
%   the estimator feedback matrix is L[k] with entries l_ij[k] (paper: script
%   ell; MATLAB tex has no \ell, so an italic l is drawn), i = state row,
%   j = measurement; the two feedback signals are the estimation errors
%       e_h1[k] = dh_m[k]  - dh_hat_1[k]      (paper e_x1, axis x -> h)
%       e_ah[k] = a_hm[k]  - a_hat_h[k]       (paper e_ax; here the whitened,
%                                              echo-scaled y2 innovation)
%   In this controller a_hat is state row 4, so the gain row is l_41, l_42
%   (paper: row 6, l_61, l_62). Controller logs: K_a_y1_out = l_41,
%   K_a_y2_out = l_42, innov_y1_out = e_h1, innov_y2_out = e_ah.
%
%   THE LEDGER CLOSES EXACTLY. a-hat moves only by the law (predict) plus the
%   two measurement corrections:
%       a_hat[end] - a_hat[1] = sum(predict increments) + sum(l41 e_h1) + sum(l42 e_ah)
%   so cumsum of each term is that channel's NET contribution, in a_bar units,
%   with its sign. Nothing is normalised, smoothed or fitted in row 1.
%
%   ROW 1  cumulative (the integral)  -- NET push. Signed, no smoothing.
%   ROW 2  per-step share             -- EFFORT. |l42 e_ah| / (|l41 e_h1| + |l42 e_ah|),
%          0.25 s moving mean because a per-step ratio is unreadable raw.
%
%   The two disagree on purpose: the tuned arm has y2 working HARDER (share
%   0.098 -> 0.145) yet pushing a-hat LESS (+0.0313 -> +0.0165), because most
%   of y2's corrections are noise that cancels. Effort is not effect.
function out = plot_y2_contribution(ARMS, seed_idx, ax, tag)

    if nargin < 2 || isempty(seed_idx); seed_idx = 1; end
    if nargin < 3 || isempty(ax);       ax = 3;       end
    if nargin < 4;                      tag = '';     end   % file-name suffix
    W = 400;                                        % 0.25 s at 1600 Hz
    nA = size(ARMS,1);  d = cell(1,nA);
    for c = 1:nA
        r = ARMS{c,1}.runs{seed_idx};
        s.t  = r.tout(:);
        ahat = r.a_bar_hat_out(:,ax);
        s.c1 = r.K_a_y1_out(:,ax) .* r.innov_y1_out(:,ax);
        s.c2 = r.K_a_y2_out(:,ax) .* r.innov_y2_out(:,ax);
        s.c1(1) = 0;  s.c2(1) = 0;
        s.C1 = cumsum(s.c1);  s.C2 = cumsum(s.c2);
        s.tot = ahat - ahat(1);
        s.law = s.tot - s.C1 - s.C2;                % predict, by difference
        s.sh  = movmean(abs(s.c2)./max(abs(s.c1)+abs(s.c2),realmin), W);
        d{c} = s;
    end

    fprintf('\n%-30s %11s %11s %11s %11s\n','arm','total','law','sum l41 e_h1','sum l42 e_ah');
    for c=1:nA
        s=d{c};
        fprintf('%-30s %+11.5f %+11.5f %+11.5f %+11.5f\n', ARMS{c,2}, ...
                s.tot(end), s.law(end), s.C1(end), s.C2(end));
    end
    fprintf('  ledger residual (must be 0): ');
    for c=1:nA; fprintf('%.2e  ', d{c}.tot(end)-d{c}.law(end)-d{c}.C1(end)-d{c}.C2(end)); end
    fprintf('\n  per-step share, median over t 1-4 s: ');
    for c=1:nA; m=d{c}.t>=1&d{c}.t<=4; fprintf('%.4f  ', median(d{c}.sh(m))); end; fprintf('\n');

    COL_LAW=[0.55 0.30 0.75]; COL_Y1=[0 0.2 0.9]; COL_Y2=[0.10 0.55 0.15];
    FS=18; LFS=13; AXLW=2.0;
    YL1=local_lim([local_cat(d,'tot');local_cat(d,'law');local_cat(d,'C1');local_cat(d,'C2')],0.06);
    f=figure('Position',[30 30 760*nA 1000],'Color','w','NumberTitle','off','Visible','off');
    tl=tiledlayout(f,2,nA,'TileSpacing','compact','Padding','compact');
    A=gobjects(2,nA);
    for c=1:nA
        s=d{c};
        a=nexttile(tl,c); A(1,c)=a; hold(a,'on');
        yline(a,0,'-','Color',[0.5 0.5 0.5],'LineWidth',1.0,'HandleVisibility','off');
        h1=plot(a,s.t,s.law,'-','Color',COL_LAW,'LineWidth',2.4);
        h2=plot(a,s.t,s.C1,'-','Color',COL_Y1,'LineWidth',2.4);
        h3=plot(a,s.t,s.C2,'-','Color',COL_Y2,'LineWidth',2.4);
        h4=plot(a,s.t,s.tot,'--','Color',[0 0 0],'LineWidth',1.8);
        legend(a,[h1 h2 h3 h4],{'law (predict)','\Sigma {\itl}_{41} e_{h1}','\Sigma {\itl}_{42} e_{ah}', ...
            'total  \^a_h[k] - \^a_h[0]'}, ...
            'Location','northoutside','Orientation','horizontal', ...
            'FontSize',LFS,'FontWeight','bold','Box','on');
        ylim(a,YL1);
        if c==1; ylabel(a,'cumulative \Delta \^a_h','FontSize',FS,'FontWeight','bold'); end
        text(a,0.985,0.10,ARMS{c,2},'Units','normalized','HorizontalAlignment','right', ...
             'FontSize',LFS+1,'FontWeight','bold');

        a=nexttile(tl,nA+c); A(2,c)=a; hold(a,'on');
        h=plot(a,s.t,s.sh,'-','Color',COL_Y2,'LineWidth',2.4);
        if c==1
            legend(a,h,{'|{\itl}_{42} e_{ah}| / ( |{\itl}_{41} e_{h1}| + |{\itl}_{42} e_{ah}| )   0.25 s moving mean'}, ...
                'Location','northoutside','Orientation','horizontal', ...
                'FontSize',LFS,'FontWeight','bold','Box','on');
            ylabel(a,'e_{ah} share','FontSize',FS,'FontWeight','bold');
        end
        ylim(a,[0 max(0.5, 1.05*max(cellfun(@(q) max(q.sh(q.t>0.5)), d)))]);   % 0.5 unless the share exceeds it (1 Hz descent does)
        xlabel(a,'time  [s]','FontSize',FS,'FontWeight','bold');
    end
    for row=1:2
        for c=1:nA
            set(A(row,c),'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on', ...
                'TickLabelInterpreter','tex'); grid(A(row,c),'off');
            xlim(A(row,c),[d{1}.t(1) d{1}.t(end)]);
            if row<2; set(A(row,c),'XTickLabel',[]); end
            if c>1;   set(A(row,c),'YTickLabel',[]); end
        end
    end
    here=fileparts(mfilename('fullpath')); root=fileparts(fileparts(here));
    fn=fullfile(root,'test_results','apd_acov_meng',sprintf('y2_contribution%s.png',tag));
    exportgraphics(f,fn,'Resolution',150); close(f);
    fprintf('figure -> %s\n', fn);
    out=d;
end

function v=local_cat(d,f)
    v=[]; for c=1:numel(d); v=[v; d{c}.(f)]; end   %#ok<AGROW>
end
function L=local_lim(v,pad)
    v=v(isfinite(v)); lo=min(v); hi=max(v); r=hi-lo; if r<=0; r=1; end
    L=[lo-pad*r, hi+pad*r];
end
