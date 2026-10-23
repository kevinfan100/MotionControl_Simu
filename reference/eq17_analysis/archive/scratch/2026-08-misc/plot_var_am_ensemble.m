% STATUS: ACTIVE (scratch figure) | PURPOSE: Var(a_m) checked the ENSEMBLE
%   way -- at every sample time k take the variance ACROSS seeds of
%   a_m[k] (a_true[k] of that seed subtracted), and compare with the formula
%   evaluated at that time. No time window, no moving statistic.
%     row 1  sd_seeds(a_m - a)     vs  sqrt(K_var IF) (a + xi)
%     row 2  sd_seeds(y2 - a_cov a) vs  sqrt(2) a_cov (a + xi)   and sqrt(R2)
%   Console: ratio var/formula averaged over the far field and the trough hold.
function out = plot_var_am_ensemble(mat, arm_idx, ax)

    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    if nargin < 1 || isempty(mat);     mat = fullfile(od, 'pair_if_meng_100.mat'); end
    if nargin < 2 || isempty(arm_idx); arm_idx = 1; end
    if nargin < 3 || isempty(ax);      ax = 3; end
    S = load(mat); if isfield(S,'out'); O = S.out.O{arm_idx}; else; O = S.oBoth.O{arm_idx}; end
    nS = numel(O.runs); cc = O.runs{1}.ctrl_const;
    P = O.runs{1}.meta.params_value; fourkT = 4*P.ctrl.k_B*P.ctrl.T; s2n = P.ctrl.sigma2_noise(ax);
    xi = cc.C_n*s2n/(cc.C_dpmr*fourkT); Kvar = 2*cc.a_cov/(2-cc.a_cov);
    IFfun = @(a) 1 + 2*((fourkT*a).^2*cc.IF_abc(1) + 2*(fourkT*a)*s2n*cc.IF_abc(2) + s2n^2*cc.IF_abc(3)) ...
                    ./ (cc.C_dpmr*fourkT*a + cc.C_n*s2n).^2;

    t = O.runs{1}.tout(:); N = numel(t);
    AM = zeros(N, nS); AT = AM; Y2 = AM; R2 = AM;
    for q = 1:nS
        r = O.runs{q}; ad = r.a_hat_out(1,ax)/r.a_bar_hat_out(1,ax);
        AM(:,q) = r.a_xm_out(:,ax); AT(:,q) = r.a_true_out(:,ax);
        Y2(:,q) = [NaN; AM(2:end,q) - (1-cc.a_cov)*AM(1:end-1,q)];
        R2(:,q) = r.R2_out(:,ax)*ad^2;
    end
    at   = mean(AT, 2);
    sd_am = std(AM - AT, 0, 2);                       % ensemble sd at each k
    sd_y2 = std(Y2 - cc.a_cov*AT, 0, 2);
    f_am = sqrt(Kvar*IFfun(at)).*(at + xi);
    f_y2 = sqrt(2)*cc.a_cov*(at + xi);
    sqR2 = sqrt(mean(R2, 2));

    fprintf('%d seeds, ensemble variance at each sample (a_pd %.3g, a_cov %.3g)\n', nS, cc.a_pd, cc.a_cov);
    W = [1 4; 10.5 12.5]; WN = {'far 1-4 s', 'hold 10.5-12.5 s'};
    for w = 1:2
        m = t >= W(w,1) & t <= W(w,2);
        r1 = sd_am(m).^2 ./ f_am(m).^2; r2 = sd_y2(m).^2 ./ f_y2(m).^2; r3 = sd_y2(m).^2 ./ sqR2(m).^2;
        fprintf('  %-18s Var(a_m)/formula %.3f +- %.3f | Var(y2)/formula %.3f +- %.3f | Var(y2)/R2 %.3f\n', ...
                WN{w}, mean(r1), std(r1)/sqrt(nnz(m)/20), mean(r2), std(r2)/sqrt(nnz(m)/5), mean(r3));
    end
    fprintf('  (SE uses ~20-step / ~5-step correlation lengths of a_m / y2)\n');

    C_T = [0.8 0 0]; C_E = [0 0.2 0.9]; C_G = [0.45 0.45 0.45]; FS = 18; LFS = 13; AXLW = 2.0;
    tl_ = [0.5 10.5]; m = t >= tl_(1) & t <= tl_(2);
    f = figure('Position',[10 10 1500 1000],'Color','w','NumberTitle','off','Visible','off');
    tl = tiledlayout(f,2,1,'TileSpacing','compact','Padding','compact'); A = gobjects(2,1);

    a = nexttile(tl); A(1) = a; hold(a,'on');
    h1 = plot(a, t(m), sd_am(m), '-', 'Color', C_E, 'LineWidth', 1.2);
    h2 = plot(a, t(m), f_am(m),  '-', 'Color', C_T, 'LineWidth', 2.4);
    legend(a,[h1 h2],{sprintf('sd_{seeds}(a_m - a)   %d seeds, each sample', nS), '[K_{var} IF]^{1/2} (a + \xi)'}, ...
           'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
    ylabel(a, 'sd(a_m)  (\mum/pN)', 'FontSize', FS, 'FontWeight', 'bold');

    a = nexttile(tl); A(2) = a; hold(a,'on');
    h1 = plot(a, t(m), sd_y2(m), '-', 'Color', C_E, 'LineWidth', 1.2);
    h2 = plot(a, t(m), f_y2(m),  '-', 'Color', C_T, 'LineWidth', 2.4);
    h3 = plot(a, t(m), sqR2(m), '--', 'Color', C_G, 'LineWidth', 1.6);
    legend(a,[h1 h2 h3],{'sd_{seeds}(y_2 - a_{cov} a)', '2^{1/2} a_{cov} (a + \xi)', 'R_2^{1/2}'}, ...
           'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
    ylabel(a, 'sd(y_2)  (\mum/pN)', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel(a, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    for q = 1:2
        set(A(q),'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid(A(q),'off'); xlim(A(q), tl_);
        yl = ylim(A(q)); ylim(A(q), [0 yl(2)]); if q < 2; set(A(q),'XTickLabel',[]); end
    end
    fn = fullfile(od, 'var_am_ensemble.png'); exportgraphics(f, fn, 'Resolution', 150); close(f);
    fprintf('figure -> %s\n', fn);
    out = struct('t', t, 'sd_am', sd_am, 'f_am', f_am, 'sd_y2', sd_y2, 'f_y2', f_y2, 'sqR2', sqR2);
end
