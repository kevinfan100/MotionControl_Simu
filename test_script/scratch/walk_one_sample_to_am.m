% STATUS: ACTIVE (scratch instrument) | PURPOSE: take ONE tracking-error
%   sample out of a finished run and walk it all the way to a_m, both as the
%   seven arithmetic steps and as an INFLUENCE FUNCTION (perturb that one
%   sample, propagate, watch what it leaves behind and for how long).
%   Companion to rebuild_am_chain.m, which proves the offline chain is
%   bit-exact against the controller; this one uses that chain to answer
%   "what does one measurement actually do to the gain readout".
%
% Two routes exist and the figure separates them:
%   A (EWMA)  dw_r[k]^2 enters sigma2 with weight a_cov and decays as
%             (1-a_cov)^n. One-signed, size 2*dw_r*delta.
%   B (LP)    the same sample enters dw_bar with weight a_pd, so every LATER
%             residual is pulled by -a_pd*(1-a_pd)^n*delta. Its sign follows
%             the sign of the later dw_r, so it is a scatter, not an offset.
% The finite difference contains both; route A alone is drawn for contrast.
function out = walk_one_sample_to_am(r, ax, k0, delta_nm)

    if nargin < 2 || isempty(ax);       ax = 3;        end
    if nargin < 3 || isempty(k0);       k0 = 5000;     end
    if nargin < 4 || isempty(delta_nm); delta_nm = 1;  end   % perturbation [nm]

    cc = r.ctrl_const;  P = r.meta.params_value;
    a_disp = r.a_hat_out(1, ax) / r.a_bar_hat_out(1, ax);    % = a_o*R
    a_o    = a_disp / r.R;
    kT     = 4 * (P.ctrl.k_B * P.ctrl.T / r.R) * a_o;
    s2n    = P.ctrl.sigma2_noise(ax) / r.R^2;
    den    = cc.C_dpmr * kT;                                 % the step-7 divisor

    dw  = r.dh_m_out(:, ax) / r.R;
    aM  = r.a_xm_out(:, ax) / a_disp;
    s2_0 = aM(1) * cc.C_dpmr * kT + cc.C_n * s2n;

    [bar, s2, am] = local_chain(dw, cc.a_pd, cc.a_cov, s2_0, cc.C_n * s2n, den);

    % ---- the seven steps at k0 ---------------------------------------
    fprintf('\n===== ONE SAMPLE, k = %d, t = %.4f s, axis %d =====\n', k0, r.tout(k0), ax);
    fprintf('  1  p_d[k-d] - p_m[k]        %+9.4f nm\n', r.dh_m_out(k0, ax) * 1e3);
    fprintf('  2  dw_m  = /R               %+.6e\n', dw(k0));
    fprintf('  3  dw_bar = LP(a_pd)        %+.6e   (%+.4f nm)\n', bar(k0), bar(k0)*r.R*1e3);
    fprintf('  4  dw_r  = 2 - 3            %+.6e   (%+.4f nm)  <- this sample''s jiggle\n', ...
            dw(k0)-bar(k0), (dw(k0)-bar(k0))*r.R*1e3);
    fprintf('  5a dw_r^2                   %.6e   <- what it contributes\n', (dw(k0)-bar(k0))^2);
    fprintf('  5b sigma2 BEFORE            %.6e\n', s2(k0-1));
    fprintf('  5c sigma2 AFTER  = (1-a_cov)*5b + a_cov*5a  %.6e\n', s2(k0));
    fprintf('     -> this one sample moved sigma2 by        %+.4e (%.3f %%)\n', ...
            s2(k0)-s2(k0-1), 100*(s2(k0)-s2(k0-1))/s2(k0-1));
    fprintf('  6  - C_n*sigma2_n           %.6e\n', cc.C_n * s2n);
    fprintf('  7  / (C_dpmr*kappa_T)       %.6e\n', den);
    fprintf('  =>  a_m %.4f -> %.4f   (jump %+.4f)   a_true %.4f\n', ...
            am(k0-1), am(k0), am(k0)-am(k0-1), r.a_true_out(k0, ax)/a_disp);

    % ---- influence function: perturb THAT sample only ----------------
    d_nd = (delta_nm * 1e-3) / r.R;                    % nm -> um -> /R
    dwp = dw;  dwp(k0) = dwp(k0) + d_nd;
    [~, ~, amp] = local_chain(dwp, cc.a_pd, cc.a_cov, s2_0, cc.C_n * s2n, den);
    infl = amp - am;                                   % exact, both routes

    n  = (0:600).';
    kk = k0 + n;  kk = kk(kk <= numel(am));  n = n(1:numel(kk));
    % route A alone, analytic
    dwr0  = dw(k0) - bar(k0);
    dA    = cc.a_cov * (2*dwr0*d_nd + d_nd^2) / den;
    inflA = dA * (1 - cc.a_cov).^n;

    fprintf('\n----- influence of a %+.2f nm change in that ONE sample -----\n', delta_nm);
    fprintf('  immediate jump in a_m            %+.5e   (route A analytic %+.5e)\n', ...
            infl(k0), dA);
    fprintf('  still left after  20 steps (12 ms) %+.5e  (%.1f %% of the jump)\n', ...
            infl(k0+20), 100*infl(k0+20)/infl(k0));
    fprintf('  still left after 100 steps (63 ms) %+.5e  (%.1f %%)\n', ...
            infl(k0+100), 100*infl(k0+100)/infl(k0));
    fprintf('  half-life  = ln(2)/a_cov = %.1f steps = %.2f ms\n', ...
            log(2)/cc.a_cov, log(2)/cc.a_cov * 6.25e-4 * 1e3);
    fprintf('  the jump is %.4f %% of a_m itself (a_m = %.4f)\n', ...
            100*infl(k0)/am(k0), am(k0));

    out = struct('t', r.tout, 'dw_m', dw, 'dw_bar', bar, 'dw_r', dw-bar, 's2', s2, ...
                 'am', am, 'infl', infl, 'n', n, 'kk', kk, 'inflA', inflA, ...
                 'k0', k0, 'delta_nm', delta_nm, 'R', r.R, 'a_cov', cc.a_cov, ...
                 'a_pd', cc.a_pd, 'den', den, 'Cn_s2n', cc.C_n*s2n, ...
                 'a_true', r.a_true_out(:, ax)/a_disp);
    local_page(out, r, ax);
end

% ---------------------------------------------------------------------
function [bar, s2, am] = local_chain(dw, a_pd, a_cov, s2_0, Cn_s2n, den)
    N = numel(dw);  bar = zeros(N,1);  s2 = zeros(N,1);  am = nan(N,1);
    bar(1) = dw(1);  s2(1) = s2_0;  am(1) = (s2(1)-Cn_s2n)/den;
    for k = 2:N
        bar(k) = (1-a_pd)*bar(k-1) + a_pd*dw(k);
        s2(k)  = (1-a_cov)*s2(k-1) + a_cov*(dw(k)-bar(k))^2;
        am(k)  = (s2(k)-Cn_s2n)/den;
    end
end

% ---------------------------------------------------------------------
function local_page(o, r, ax)
    FS=15; LFS=11; AXLW=1.8;
    COL_T=[0.85 0.10 0.10]; COL_E=[0.00 0.20 0.90]; COL_M=[0.45 0.72 0.95];
    % ONE shared window on every panel: 200 ms before the sample, 400 ms after,
    % so the vertical marker sits at the same x in all four.
    k0=o.k0; iL=320; iR=640; i=(k0-iL):min(k0+iR,numel(o.t));
    ti=(o.t(i)-o.t(k0))*1e3;  XL=[-200 400];
    f=figure('Position',[30 30 1400 1250],'Color','w','NumberTitle','off','Visible','off');
    tl=tiledlayout(f,4,1,'TileSpacing','compact','Padding','compact');

    a=nexttile(tl); hold(a,'on');
    h1=plot(a,ti,o.dw_m(i)*o.R*1e3,'-','Color',COL_M,'LineWidth',1.0);
    h2=plot(a,ti,o.dw_bar(i)*o.R*1e3,'-','Color',COL_T,'LineWidth',2.4);
    h3=plot(a,ti,o.dw_r(i)*o.R*1e3,'-','Color',COL_E,'LineWidth',1.0);
    xline(a,0,'-','Color',[0.4 0.4 0.4],'LineWidth',1.6,'HandleVisibility','off');
    plot(a,0,o.dw_r(k0)*o.R*1e3,'o','Color',[0 0 0],'MarkerFaceColor',[1 1 0], ...
         'MarkerSize',11,'LineWidth',1.6,'HandleVisibility','off');
    legend(a,[h1 h2 h3],{'dw_m  (steps 1-2)','dw_{bar}  LP(a_{pd})  (step 3)', ...
           'dw_r = dw_m - dw_{bar}  (step 4)'},'Location','northoutside', ...
           'Orientation','horizontal','FontSize',LFS,'Box','on');
    ylabel(a,{'tracking error','[nm]'},'FontSize',FS,'FontWeight','bold');
    set(a,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on','XTickLabel',[]); xlim(a,XL);

    a=nexttile(tl); hold(a,'on');
    h1=plot(a,ti,o.dw_r(i).^2,'-','Color',[0.55 0.55 0.55],'LineWidth',0.8);
    h2=plot(a,ti,o.s2(i),'-','Color',[0.85 0.33 0.10],'LineWidth',2.4);
    xline(a,0,'-','Color',[0.4 0.4 0.4],'LineWidth',1.6,'HandleVisibility','off');
    plot(a,0,o.dw_r(k0)^2,'o','Color',[0 0 0],'MarkerFaceColor',[1 1 0], ...
         'MarkerSize',11,'LineWidth',1.6,'HandleVisibility','off');
    set(a,'YScale','log');
    legend(a,[h1 h2],{'dw_r^2  (step 5a, one sample)','\sigma^2 EWMA(a_{cov})  (step 5c)'}, ...
           'Location','northoutside','Orientation','horizontal','FontSize',LFS,'Box','on');
    ylabel(a,{'variance','[-]'},'FontSize',FS,'FontWeight','bold');
    set(a,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on','XTickLabel',[]); xlim(a,XL);

    a=nexttile(tl); hold(a,'on');
    h1=plot(a,ti,o.a_true(i),'-','Color',COL_T,'LineWidth',2.4);
    h2=plot(a,ti,o.am(i),'-','Color',COL_M,'LineWidth',1.4);
    xline(a,0,'-','Color',[0.4 0.4 0.4],'LineWidth',1.6,'HandleVisibility','off');
    plot(a,0,o.am(k0),'o','Color',[0 0 0],'MarkerFaceColor',[1 1 0], ...
         'MarkerSize',11,'LineWidth',1.6,'HandleVisibility','off');
    legend(a,[h1 h2],{'a_{true}','a_m  (step 7 output)'},'Location','northoutside', ...
           'Orientation','horizontal','FontSize',LFS,'Box','on');
    ylabel(a,{'a_z / a_o'},'FontSize',FS,'FontWeight','bold');
    set(a,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on','XTickLabel',[]); xlim(a,XL);

    a=nexttile(tl); hold(a,'on');
    tn=(o.t(o.kk)-o.t(k0))*1e3;
    h1=plot(a,tn,o.infl(o.kk),'-','Color',COL_E,'LineWidth',2.4);
    h2=plot(a,tn,o.inflA,'--','Color',[0.10 0.55 0.15],'LineWidth',2.0);
    yline(a,0,'-','Color',[0.5 0.5 0.5],'LineWidth',1.0,'HandleVisibility','off');
    legend(a,[h1 h2],{sprintf('exact influence of %+.1f nm on that ONE sample',o.delta_nm), ...
           'route A alone: a_{cov}\cdot2\cdot dw_r\cdot\delta / (C_{dpmr}\kappa_T) \cdot (1-a_{cov})^n'}, ...
           'Location','northoutside','Orientation','horizontal','FontSize',LFS,'Box','on');
    xline(a,0,'-','Color',[0.4 0.4 0.4],'LineWidth',1.6,'HandleVisibility','off');
    xlabel(a,'time since that sample  [ms]   (0 = the marked sample)','FontSize',FS,'FontWeight','bold');
    ylabel(a,{'\Delta a_m'},'FontSize',FS,'FontWeight','bold');
    set(a,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); xlim(a,XL);

    here=fileparts(mfilename('fullpath')); root=fileparts(fileparts(here));
    od=fullfile(root,'test_results','apd_acov_meng');
    if ~exist(od,'dir'); mkdir(od); end
    fn=fullfile(od,sprintf('walk_one_sample_k%05d.png',k0));
    exportgraphics(f,fn,'Resolution',150); close(f);
    fprintf('\nfigure -> %s\n', fn);
end
