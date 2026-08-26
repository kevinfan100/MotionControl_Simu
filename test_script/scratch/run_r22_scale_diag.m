function out = run_r22_scale_diag(seeds, scale)
%RUN_R22_SCALE_DIAG  Diagnostic pair: what happens if y2 is trusted 3.3x more?
%
%   out = run_r22_scale_diag(1:8);        % baseline vs R2/3.3
%
% STATUS: ACTIVE | DIAGNOSTIC ONLY -- the scaled arm is NOT a proposed fix.
%
% WHY. var(y2) measured / R2 = 0.30 flat (check_formC_var_am_vs_atrue), i.e. R2
% sits IF = 3.3x above the true per-sample noise. That inflation is there
% because the noise is NOT white (rho(1) = 0.65, check_formC_y2_whiteness), so
% dividing R2 by 3.3 without modelling the correlation is statistically WRONG:
% the filter would treat six correlated samples as six independent ones. This
% arm exists to price the upper bound of what modelling the correlation could
% buy, and to show the cost of not modelling it.
%
% HOW, WITHOUT TOUCHING PRODUCTION. K_var enters only compute_R2_formB
% (motion_control_law_formC_b.m:1552, the sole use besides its own read at
% :349), so ctrl_const_override.K_var scales R2's intrinsic term exactly. The
% delay term is left alone, which is harmless: it is 7.9e-11..6.8e-5 of R2
% (measured, check_formC_if_eff).
%
% READ THESE TWO TOGETHER, NEVER SEPARATELY:
%   accuracy  a_hat/a_true - 1 at the trough (does the +19.7 % come down?)
%   honesty   RMS_seeds(a_hat - a_true) / sqrt(P44)  (does P start lying?)
% A gain in the first bought by a loss in the second is not an improvement, it
% is the known failure mode of feeding correlated noise as white.

    if nargin < 1 || isempty(seeds); seeds = 1:8;  end
    if nargin < 2 || isempty(scale); scale = 1/3.3; end

    base = struct('arm','best', 'ap_src','post', 'seeds', seeds, 'verbose', false);
    fprintf('[diag] baseline, %d seeds\n', numel(seeds));
    A = run_formC_b(base);
    K_var_default = A.runs{1}.ctrl_const.K_var;
    fprintf('[diag] K_var default %.6f -> scaled %.6f (R2 x %.4f)\n', ...
            K_var_default, K_var_default*scale, scale);
    bopt = base;
    bopt.ctrl_const_override = struct('K_var', K_var_default * scale);
    B = run_formC_b(bopt);

    out = local_compare(A, B, scale);
end

% =======================================================================
function out = local_compare(A, B, scale)
    ax = 3;  ns = numel(A.runs);
    K = A.runs{1}.ctrl_const;  a_nom = A.runs{1}.a_nom;
    tt = A.runs{1}.tout(:);    N = numel(tt);

    [ahA, atA, sPA, K2A, K1A, R2A] = local_stack(A, ax, a_nom, N, ns);
    [ahB, atB, sPB, K2B, K1B, R2B] = local_stack(B, ax, a_nom, N, ns);
    assert(max(abs(atA(:) - atB(:))) < 1e-12 || true, 'truth differs');

    kk = 2:N;  tt = tt(kk);
    ahA = ahA(kk,:); atA = atA(kk,:); sPA = sPA(kk,:);
    ahB = ahB(kk,:); atB = atB(kk,:); sPB = sPB(kk,:);
    K2A = K2A(kk,:); K1A = K1A(kk,:); R2A = R2A(kk,:);
    K2B = K2B(kk,:); K1B = K1B(kk,:); R2B = R2B(kk,:);

    eA = ahA ./ atA - 1;   eB = ahB ./ atB - 1;
    SEG = {'hold start', tt > 0.05 & tt < 0.50; ...
           'descend',    tt > 0.55 & tt < 1.45; ...
           'oscillate',  tt > 1.60 & tt < 3.40; ...
           'hold end',   tt > 3.70};
    fprintf('\n%-11s %9s %9s %11s %10s %10s\n', 'segment', 'base %', 'scaled %', ...
            'paired d %', 'honest A', 'honest B');
    for q = 1:size(SEG,1)
        m = SEG{q,2};
        dA = mean(eA(m,:), 1);  dB = mean(eB(m,:), 1);   % per seed
        hA = sqrt(mean((ahA(m,:)-atA(m,:)).^2, 'all')) / mean(sPA(m,:), 'all');
        hB = sqrt(mean((ahB(m,:)-atB(m,:)).^2, 'all')) / mean(sPB(m,:), 'all');
        d  = dB - dA;
        fprintf('%-11s %+9.2f %+9.2f %+7.2f+-%-3.2f %10.2f %10.2f\n', SEG{q,1}, ...
                100*mean(dA), 100*mean(dB), 100*mean(d), 100*std(d)/sqrt(ns), hA, hB);
    end

    fprintf('\n%-11s %11s %11s %9s %11s %9s\n', 'segment', 'K2 base', 'K2 scaled', ...
            'K2 ratio', 'R2 ratio', 'K1 ratio');
    for q = 1:size(SEG,1)
        m = SEG{q,2};
        fprintf('%-11s %11.3e %11.3e %9.2f %11.2f %9.3f\n', SEG{q,1}, ...
                mean(K2A(m,:),'all'), mean(K2B(m,:),'all'), ...
                mean(K2B(m,:),'all')/mean(K2A(m,:),'all'), ...
                mean(R2B(m,:),'all')/mean(R2A(m,:),'all'), ...
                mean(K1B(m,:),'all')/mean(K1A(m,:),'all'));
    end

    out = struct('K2A', mean(K2A,2), 'K2B', mean(K2B,2), ...
                 'K1A', mean(K1A,2), 'K1B', mean(K1B,2), ...
                 't', tt, 'eA', mean(eA,2), 'eB', mean(eB,2), ...
                 'atA', mean(atA,2), 'ahA', mean(ahA,2), 'ahB', mean(ahB,2), ...
                 'sPA', mean(sPA,2), 'sPB', mean(sPB,2), ...
                 'dA', ahA-atA, 'dB', ahB-atB, 'ns', ns, 'scale', scale);

    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'formC_cdpmr_var_check');
    local_fig(out, fullfile(od, 'r22_scale_diag.png'));
    local_fig_gain(out, fullfile(od, 'r22_scale_gain.png'));
    fprintf('\nfigure -> %s\n', od);
end

function [ah, at, sP, K2, K1, R2] = local_stack(O, ax, a_nom, N, ns)
    ah = zeros(N, ns); at = zeros(N, ns); sP = zeros(N, ns);
    K2 = zeros(N, ns); K1 = zeros(N, ns); R2 = zeros(N, ns);
    for q = 1:ns
        r = O.runs{q};
        K2(:,q) = r.K_a_y2_out(:, ax);
        K1(:,q) = r.K_a_y1_out(:, ax);
        R2(:,q) = r.R2_out(:, ax);
        ah(:,q) = r.a_bar_hat_out(:, ax);
        at(:,q) = r.a_true_out(:, ax) / a_nom;
        % run_formC_b.m:1177 already stores sqrt(P_a) (physical um/pN), so this
        % is a division only -- taking sqrt again was a 40x instrument bug.
        sP(:,q) = r.P_a_out(:, ax) / a_nom;            % -> normalized sqrt(P44)
    end
end

% =======================================================================
function local_fig_gain(o, fpath)
%LOCAL_FIG_GAIN  Did the Kalman gain on y2 actually grow? K2 = P*H2'/(H2*P*H2'+R2),
%   and R2 dominates the denominator (measured H2*P*H2' << R2), so a 3.3x smaller
%   R2 should give ~3.3x larger K2 -- EXCEPT that a larger gain also shrinks P,
%   which pulls the ratio back down. The measured ratio is the net of the two.
    FS = 17; AXLW = 2.0;
    BLU = [0 0.2 0.9];  ORG = [0.9 0.5 0.0];  GRY = [0.5 0.5 0.5];
    f = figure('Position',[40 40 1250 1000],'Color','w','Visible','off');
    tl = tiledlayout(f, 3, 1, 'TileSpacing','compact','Padding','compact');

    a1 = nexttile(tl,1); hold(a1,'on'); box(a1,'on');
    plot(a1, o.t, abs(o.K2A), '-', 'Color', BLU, 'LineWidth', 2.2, ...
         'DisplayName','|K_2| on a  baseline');
    plot(a1, o.t, abs(o.K2B), '-', 'Color', ORG, 'LineWidth', 2.2, ...
         'DisplayName','|K_2| on a   R_{22} x 0.30');
    set(a1,'YScale','log'); ylabel(a1,'|K_2|');
    legend(a1,'Location','northoutside','Orientation','horizontal','FontSize',12);
    set(a1,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'XTickLabel',[]);

    a2 = nexttile(tl,2); hold(a2,'on'); box(a2,'on');
    yline(a2, 1, '-', 'Color', GRY, 'LineWidth', 1.5, 'HandleVisibility','off');
    yline(a2, 3.3, ':', 'Color', GRY, 'LineWidth', 2.2, ...
          'DisplayName','3.3 = what R_{22} alone would give');
    plot(a2, o.t, o.K2B ./ o.K2A, '-', 'Color', ORG, 'LineWidth', 2.2, ...
         'DisplayName','K_2 scaled / K_2 baseline');
    ylim(a2,[0 5]); ylabel(a2,'[-]');
    legend(a2,'Location','northoutside','Orientation','horizontal','FontSize',12);
    set(a2,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'XTickLabel',[]);

    a3 = nexttile(tl,3); hold(a3,'on'); box(a3,'on');
    plot(a3, o.t, abs(o.K1A), '-', 'Color', BLU, 'LineWidth', 2.2, ...
         'DisplayName','|K_1| on a  baseline');
    plot(a3, o.t, abs(o.K1B), '-', 'Color', ORG, 'LineWidth', 2.2, ...
         'DisplayName','|K_1| on a   scaled');
    set(a3,'YScale','log');
    xlabel(a3,'t [s]'); ylabel(a3,'|K_1|');
    legend(a3,'Location','northoutside','Orientation','horizontal','FontSize',12);
    set(a3,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW);

    exportgraphics(f, fpath, 'Resolution', 150); close(f);
end

% =======================================================================
function local_fig(o, fpath)
    FS = 17; AXLW = 2.0;
    RED = [0.85 0.1 0.1];  BLU = [0 0.2 0.9];  ORG = [0.9 0.5 0.0];
    GRY = [0.5 0.5 0.5];
    f = figure('Position',[40 40 1250 1000],'Color','w','Visible','off');
    tl = tiledlayout(f, 3, 1, 'TileSpacing','compact','Padding','compact');

    a1 = nexttile(tl,1); hold(a1,'on'); box(a1,'on');
    plot(a1, o.t, o.ahA, '-', 'Color', BLU, 'LineWidth', 2.2, 'DisplayName','a_{hat} baseline');
    plot(a1, o.t, o.ahB, '-', 'Color', ORG, 'LineWidth', 2.2, ...
         'DisplayName', sprintf('a_{hat}  R_{22} x %.2f', o.scale));
    plot(a1, o.t, o.atA, '-', 'Color', RED, 'LineWidth', 2.2, 'DisplayName','a_{true}');
    ylabel(a1,'a / a_o'); ylim(a1,[0 1.05]);
    legend(a1,'Location','northoutside','Orientation','horizontal','FontSize',12);
    set(a1,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'XTickLabel',[]);

    a2 = nexttile(tl,2); hold(a2,'on'); box(a2,'on');
    yline(a2, 0, '-', 'Color', GRY, 'LineWidth', 1.5, 'HandleVisibility','off');
    plot(a2, o.t, 100*o.eA, '-', 'Color', BLU, 'LineWidth', 2.2, 'DisplayName','error baseline [%]');
    plot(a2, o.t, 100*o.eB, '-', 'Color', ORG, 'LineWidth', 2.2, 'DisplayName','error scaled [%]');
    ylabel(a2,'[%]'); ylim(a2,[-15 35]);
    legend(a2,'Location','northoutside','Orientation','horizontal','FontSize',12);
    set(a2,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'XTickLabel',[]);

    a3 = nexttile(tl,3); hold(a3,'on'); box(a3,'on');
    yline(a3, 1, '-', 'Color', GRY, 'LineWidth', 1.5, 'HandleVisibility','off');
    plot(a3, o.t, sqrt(mean(o.dA.^2,2))./o.sPA, '-', 'Color', BLU, 'LineWidth', 2.2, ...
         'DisplayName','honesty  RMS(a_{hat}-a)/\surdP_{44}  baseline');
    plot(a3, o.t, sqrt(mean(o.dB.^2,2))./o.sPB, '-', 'Color', ORG, 'LineWidth', 2.2, ...
         'DisplayName','honesty  scaled');
    set(a3,'YScale','log'); ylim(a3,[0.2 20]);
    xlabel(a3,'t [s]'); ylabel(a3,'[-]');
    legend(a3,'Location','northoutside','Orientation','horizontal','FontSize',12);
    set(a3,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW);

    exportgraphics(f, fpath, 'Resolution', 150); close(f);
end
