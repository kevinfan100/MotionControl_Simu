% STATUS: ACTIVE (scratch instrument) | PURPOSE: the most direct check of
%   C_dpmr: estimator OUT of the loop (control on a_true, loop pole = lambda_c
%   exactly), a FIXED by holding at one height, and Var(dh_mr) read against
%       Var(dh_mr) = C_dpmr(lc, a_pd) 4kBT a  +  C_n(lc, a_pd) sigma_n^2
%   over 8 heights (four of them a_bar < 0.22) and three noise arms:
%   full (thermal + sensor) / thermal only / sensor only.
%
%   out = verify_cdpmr_hold()             % collect (15 seeds x 8 h x 3 arms) + figure
%   out = verify_cdpmr_hold([], true)     % replot from the saved .mat
%
%   Pre-registered instrument checks, read BEFORE the ratios:
%     I1  far-field full-noise ratio must reproduce ~1.00 (100 seeds gave 1.004)
%     I2  sensor-only Var must not depend on height
%     I3  full = thermal + sensor at every height (linear system, independent noises)
%     I4  loop premise at w_bar 1.10: mean dh_m ~ 0, acf(dh_mr) lag 1 ~ 0.72 (z model)
function out = verify_cdpmr_hold(seeds, replot)
    if nargin < 1 || isempty(seeds); seeds = [7 11 23 42 101 777 1 2 3 4 5 6 8 9 10]; end
    if nargin < 2 || isempty(replot); replot = false; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'apd_acov_meng'); fn = fullfile(od, 'verify_cdpmr_hold.mat');
    pc = physical_constants(); ax = 3;
    WB = [1.10 1.15 1.20 1.30 1.50 2.0 3.0 6.67];
    ARMS = {'thermal + sensor', true,  true;  'thermal only', true, false;  'sensor only', false, true};
    T_SIM = 4.5; T0 = 2.5;                      % SAME hold length and window as verify_cdpmr_hold_estimated (paired, 2026-08-28)

    if ~replot
        nS = numel(seeds); nH = numel(WB); nA = 3;
        DH = cell(nA, nH);                       % dh_m [um], N x nS
        AT = zeros(nA, nH, nS); cc0 = []; P0 = []; tt = [];
        for a = 1:nA
            for ih = 1:nH
                h = WB(ih)*pc.R;
                ov = struct('trajectory_type','osc','h_init',h,'h_bottom',h,'amplitude',0,'frequency',1,'n_cycles',1, ...
                            't_hold',0.5,'t_descend_override',0.01,'T_sim',T_SIM,'h_min',1.1*pc.R, ...
                            'thermal_enable',ARMS{a,2},'meas_noise_enable',ARMS{a,3});
                clear run_formC_b motion_control_law_formC_b;
                [~, O] = evalc("run_formC_b(struct('arm','best','seeds',seeds,'config_override',ov,'a_ctrl_override','true'));");
                N = numel(O.runs{1}.tout); M = zeros(N, nS);
                for q = 1:nS; r = O.runs{q}; M(:,q) = r.dh_m_out(:,ax); AT(a,ih,q) = mean(r.a_true_out(r.tout>=T0,ax)); end
                DH{a,ih} = M; if isempty(cc0); cc0 = O.runs{1}.ctrl_const; P0 = O.runs{1}.meta.params_value; tt = O.runs{1}.tout(:); end
                fprintf('collected %-17s w_bar %.2f  (a_bar %.3f)\n', ARMS{a,1}, WB(ih), AT(a,ih,1)/(O.runs{1}.a_hat_out(1,ax)/O.runs{1}.a_bar_hat_out(1,ax)));
            end
        end
        save(fn, 'DH', 'AT', 'cc0', 'P0', 'tt', 'WB', 'ARMS', 'seeds', 'T0', '-v7.3');
    else
        S = load(fn); DH = S.DH; AT = S.AT; cc0 = S.cc0; P0 = S.P0; tt = S.tt; WB = S.WB; ARMS = S.ARMS; seeds = S.seeds; T0 = S.T0;
        nS = numel(seeds); nH = numel(WB); nA = 3;
    end

    % ---------------- readout chain offline, variances, formula ---------------
    fourkT = 4*P0.ctrl.k_B*P0.ctrl.T; s2n = P0.ctrl.sigma2_noise(ax); a_pd = cc0.a_pd; Cd = cc0.C_dpmr; Cn = cc0.C_n;
    ao = P0.common.Ts / P0.common.gamma_N;                      % a_o [um/pN] far-field (a_bar = a/a_o), as run_formC_b's a_nom_drv
    m = tt >= T0; G = 5; grp = mod(0:nS-1, G) + 1;
    V = zeros(nA, nH); Vse = V; F = V; AB = V; ACF = zeros(nH, 4); MU = zeros(nH, 1);
    for a = 1:nA
        for ih = 1:nH
            M = DH{a,ih}; N = size(M,1); R = zeros(N, nS);
            for q = 1:nS
                md = zeros(N+1,1); for k = 1:N; md(k+1) = (1-a_pd)*md(k) + a_pd*M(k,q); end
                R(:,q) = M(:,q) - md(2:end);                        % dh_mr, exactly the controller's chain
            end
            v_t = var(R(m,:), 0, 2);                                % cross-seed var at each sample
            V(a,ih) = mean(v_t);
            gv = zeros(1,G); for g = 1:G; gv(g) = mean(var(R(m, grp==g), 0, 2)); end; Vse(a,ih) = std(gv)/sqrt(G);
            ah = mean(AT(a,ih,:)); AB(a,ih) = ah/ao;
            F(a,ih) = Cd*fourkT*ah*ARMS{a,2} + Cn*s2n*ARMS{a,3};
            if a == 1
                x = R(m,1) - mean(R(m,1)); for L = 1:4; ACF(ih,L) = sum(x(1+L:end).*x(1:end-L))/sum(x.^2); end
                MU(ih) = mean(mean(M(m,:)));
            end
        end
    end

    fprintf('\nC_dpmr check: control on a_true, fixed height, %d seeds, hold %.1f-%.1f s\n', nS, T0, tt(end));
    fprintf('constants: C_dpmr %.4f  C_n %.4f  a_pd %.3g  4kBT %.4g pN um  sigma_n %.3g um\n', Cd, Cn, a_pd, fourkT, sqrt(s2n));
    fprintf('%6s %6s | %-34s | %-34s | %-34s\n', 'w_bar', 'a_bar', 'full: Var / formula', 'thermal only: Var / formula', 'sensor only: Var / formula');
    for ih = 1:nH
        fprintf('%6.2f %6.3f |', WB(ih), AB(1,ih));
        for a = 1:nA; fprintf(' %9.3e / %9.3e = %5.3f+-%.3f |', V(a,ih), F(a,ih), V(a,ih)/F(a,ih), Vse(a,ih)/F(a,ih)); end; fprintf('\n');
    end
    fprintf('\nI1 far-field full ratio: %.3f (expect ~1.00)\n', V(1,end)/F(1,end));
    fprintf('I2 sensor-only Var across heights: mean %.3e, spread (max/min) %.3f (expect ~1)\n', mean(V(3,:)), max(V(3,:))/min(V(3,:)));
    fprintf('I3 additivity (thermal + sensor)/full per height: %s\n', mat2str((V(2,:)+V(3,:))./V(1,:), 3));
    fprintf('I4 premise at w_bar 1.10 (full): mean dh_m %+.3f nm ; acf(dh_mr) lag1..4 = %s  (z model lag1 ~0.72)\n', MU(1)*1e3, mat2str(ACF(1,:), 3));
    slope = (AB(2,:)*ao).' \ V(2,:).';                          % through-origin LS on the thermal-only arm
    fprintf('slope check (thermal only, all heights): fitted C_dpmr*4kBT = %.4e  vs  %.4e  (ratio %.3f)\n', slope, Cd*fourkT, slope/(Cd*fourkT));

    % ---------------- figure (simple, 2026-08-28): full-noise arm only ----------
    %   left  var(dh_mr) vs a/a_o, 8 heights, formula line     right  zoom a/a_o < 0.22
    %   Reading: the points sit on the line <=> C_dpmr (slope) and C_n sigma_n^2 (intercept) are right.
    C_M = [0 0.2 0.9]; C_F = [0.10 0.60 0.20]; FS = 18; LFS = 13; AXLW = 2.0;
    f = figure('Position',[10 10 1700 700],'Color','w','NumberTitle','off','Visible','off');
    tl = tiledlayout(f,1,2,'TileSpacing','compact','Padding','compact'); A = gobjects(1,2);
    XL = {[0 0.9], [0.07 0.23]};
    for col = 1:2
        a1 = nexttile(tl, col); A(col) = a1; hold(a1,'on');
        ag = linspace(XL{col}(1), XL{col}(2), 200); fl = Cd*fourkT*ag*ao + Cn*s2n;
        h1 = plot(a1, ag, fl, '-', 'Color', C_F, 'LineWidth', 2.8);
        h2 = errorbar(a1, AB(1,:), V(1,:), Vse(1,:), 'o', 'Color', C_M, 'MarkerFaceColor', C_M, 'MarkerSize', 8, 'LineWidth', 1.6, 'CapSize', 8);
        legend(a1, [h1 h2], {'C_{dpmr} 4k_BT a + C_n \sigma_n^2', sprintf('measured  var(\\delta h_{mr}),  %d seeds', nS)}, ...
               'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
        if col == 1; ylabel(a1, 'var(\delta h_{mr})   [\mum^2]', 'FontSize', FS, 'FontWeight', 'bold'); end
        xlabel(a1, 'a / a_o   [-]', 'FontSize', FS, 'FontWeight', 'bold'); xlim(a1, XL{col}); ylim(a1, [0 1.1*max(fl)]);
        set(a1,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid(a1,'off');
    end
    fig = fullfile(od, 'verify_cdpmr_hold.png'); exportgraphics(f, fig, 'Resolution', 150); close(f);
    fprintf('figure -> %s\n', fig);
    out = struct('WB', WB, 'a_bar', AB, 'V', V, 'Vse', Vse, 'F', F, 'ACF', ACF, 'MU', MU, 'file', fig);
end
