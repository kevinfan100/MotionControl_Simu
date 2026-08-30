% STATUS: ACTIVE (scratch instrument) | PURPOSE: the companion of
%   verify_cdpmr_hold: same fixed-height holds, but the loop runs on the
%   ESTIMATE (production control, a_ctrl = a_hat). Three things per height,
%   all read against the true a of that height:
%     1  a_hat / a - 1            what the estimator settles to at that height
%     2  a_m   / a - 1            what the readout reports
%     3  Var(dh_mr) / formula     formula at the DESIGN pole lambda_c
%   and the one prediction that says whether C_dpmr is still right:
%     the loop pole under a gain error is lambda_eff = 1 - (a/a_hat)(1-lambda_c);
%     evaluate C_dpmr(lambda_eff), C_n(lambda_eff) with the closed forms of
%     calc_ctrl_params.m and predict Var and a_m from that.  Points on the
%     prediction  <=>  the deviation is the pole, not the constant.
%   Hold 4 s per height (estimator transient + hold drift), analysis window
%   = last 2 s. 15 seeds, 8 heights (four with a_bar < 0.22).
function out = verify_cdpmr_hold_estimated(seeds, replot)
    if nargin < 1 || isempty(seeds); seeds = [7 11 23 42 101 777 1 2 3 4 5 6 8 9 10]; end
    if nargin < 2 || isempty(replot); replot = false; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'apd_acov_meng'); fn = fullfile(od, 'verify_cdpmr_hold_estimated.mat');
    pc = physical_constants(); ax = 3; WB = [1.10 1.15 1.20 1.30 1.50 2.0 3.0 6.67]; T_SIM = 4.5; T0 = 2.5;

    if ~replot
        nS = numel(seeds); nH = numel(WB); DH = cell(1,nH); AM = DH; AH = DH; AT = zeros(nH, nS); AD = AT;
        for ih = 1:nH
            h = WB(ih)*pc.R;
            ov = struct('trajectory_type','osc','h_init',h,'h_bottom',h,'amplitude',0,'frequency',1,'n_cycles',1, ...
                        't_hold',0.5,'t_descend_override',0.01,'T_sim',T_SIM,'h_min',1.1*pc.R);
            clear run_formC_b motion_control_law_formC_b;
            [~, O] = evalc("run_formC_b(struct('arm','best','seeds',seeds,'config_override',ov));");   % production: control on a_hat
            N = numel(O.runs{1}.tout); M = zeros(N,nS); A1 = M; A2 = M;
            for q = 1:nS
                r = O.runs{q}; M(:,q) = r.dh_m_out(:,ax); A1(:,q) = r.a_xm_out(:,ax); A2(:,q) = r.a_hat_out(:,ax);
                AT(ih,q) = mean(r.a_true_out(r.tout>=T0,ax)); AD(ih,q) = r.a_hat_out(1,ax)/r.a_bar_hat_out(1,ax);
            end
            DH{ih} = M; AM{ih} = A1; AH{ih} = A2;
            if ih == 1; cc0 = O.runs{1}.ctrl_const; P0 = O.runs{1}.meta.params_value; tt = O.runs{1}.tout(:); end
            fprintf('collected w_bar %.2f\n', WB(ih));
        end
        save(fn, 'DH', 'AM', 'AH', 'AT', 'AD', 'cc0', 'P0', 'tt', 'WB', 'seeds', 'T0', '-v7.3');
    else
        S = load(fn); DH = S.DH; AM = S.AM; AH = S.AH; AT = S.AT; AD = S.AD; cc0 = S.cc0; P0 = S.P0; tt = S.tt; WB = S.WB; seeds = S.seeds; T0 = S.T0;
        nS = numel(seeds); nH = numel(WB);
    end

    fourkT = 4*P0.ctrl.k_B*P0.ctrl.T; s2n = P0.ctrl.sigma2_noise(ax); a_pd = cc0.a_pd; lc = cc0.lambda_c;
    Cd_of = @(l) (1-a_pd)^2 * (2*(1-a_pd)*(1-l)./(1-(1-a_pd)*l) + 2./((2-a_pd)*(1+l).*(1-(1-a_pd)*l)));
    Cn_of = @(l) (1-a_pd)^2 * (2/(2-a_pd) + 2*(1-a_pd)^2*a_pd*(1-l)./((2-a_pd)*(1-(1-a_pd)*l)) + 2*(1-l).^2./((2-a_pd)*(1+l).*(1-(1-a_pd)*l)));
    fprintf('closed-form check at lambda_c: C_dpmr %.4f (ctrl_const %.4f)  C_n %.4f (%.4f)\n', Cd_of(lc), cc0.C_dpmr, Cn_of(lc), cc0.C_n);
    ao = P0.common.Ts / P0.common.gamma_N; m = tt >= T0; G = 5; grp = mod(0:nS-1, G) + 1;
    AB = zeros(1,nH); EH = AB; EHse = AB; RM = AB; RMse = AB; VR = AB; VRse = AB; VRp = AB; RMp = AB; LE = AB;
    for ih = 1:nH
        M = DH{ih}; N = size(M,1); R = zeros(N,nS);
        for q = 1:nS; md = zeros(N+1,1); for k = 1:N; md(k+1) = (1-a_pd)*md(k) + a_pd*M(k,q); end; R(:,q) = M(:,q) - md(2:end); end
        a = mean(AT(ih,:)); AB(ih) = a/ao;
        eh_q = mean(AH{ih}(m,:),1)./AT(ih,:) - 1;  EH(ih) = mean(eh_q);  EHse(ih) = std(eh_q)/sqrt(nS);
        rm_q = mean(AM{ih}(m,:),1)./AT(ih,:) - 1;  RM(ih) = mean(rm_q);  RMse(ih) = std(rm_q)/sqrt(nS);
        F = cc0.C_dpmr*fourkT*a + cc0.C_n*s2n;
        VR(ih) = mean(var(R(m,:),0,2))/F; gv = zeros(1,G); for g = 1:G; gv(g) = mean(var(R(m,grp==g),0,2))/F; end; VRse(ih) = std(gv)/sqrt(G);
        gq = a ./ mean(AH{ih}(m,:),1); le = 1 - gq*(1-lc); LE(ih) = mean(le);                     % lambda_eff per seed from a_hat
        VRp(ih) = mean((Cd_of(le)*fourkT*a + Cn_of(le)*s2n)/F);                                     % pole prediction of Var
        RMp(ih) = mean((Cd_of(le)*fourkT*a + Cn_of(le)*s2n - cc0.C_n*s2n)/(cc0.C_dpmr*fourkT*a)) - 1;  % pole prediction of a_m/a - 1
    end
    fprintf('\nESTIMATED loop (control on a_hat), fixed height, %d seeds, window %.1f-%.1f s\n', nS, T0, tt(end));
    fprintf('%6s %6s | %9s | %9s %9s | %9s %9s | %8s\n', 'w_bar', 'a_bar', 'a_hat/a-1', 'a_m/a-1', 'pole pred', 'Var/frm', 'pole pred', 'lam_eff');
    for ih = 1:nH
        fprintf('%6.2f %6.3f | %+8.2f%% | %+8.2f%% %+8.2f%% | %9.3f %9.3f | %8.4f\n', WB(ih), AB(ih), 100*EH(ih), 100*RM(ih), 100*RMp(ih), VR(ih), VRp(ih), LE(ih));
    end

    % ---------------- figure (simple, same reading as verify_cdpmr_hold.png) ------
    %   green line = formula at the design pole, blue points = measured under the
    %   ESTIMATED loop (control on a_hat). Points above the line = the loop the
    %   estimator actually runs has more residual variance than the design loop.
    C_M = [0 0.2 0.9]; C_F = [0.10 0.60 0.20]; FS = 18; LFS = 13; AXLW = 2.0;
    Vabs = zeros(1,nH); Vabs_se = zeros(1,nH);
    for ih = 1:nH; F = cc0.C_dpmr*fourkT*mean(AT(ih,:)) + cc0.C_n*s2n; Vabs(ih) = VR(ih)*F; Vabs_se(ih) = VRse(ih)*F; end
    f = figure('Position',[10 10 1700 700],'Color','w','NumberTitle','off','Visible','off');
    tl = tiledlayout(f,1,2,'TileSpacing','compact','Padding','compact'); A = gobjects(1,2); XL = {[0 0.9], [0.07 0.23]};
    for col = 1:2
        a1 = nexttile(tl, col); A(col) = a1; hold(a1,'on');
        ag = linspace(XL{col}(1), XL{col}(2), 200); fl = cc0.C_dpmr*fourkT*ag*ao + cc0.C_n*s2n;
        h1 = plot(a1, ag, fl, '-', 'Color', C_F, 'LineWidth', 2.8);
        h2 = errorbar(a1, AB, Vabs, Vabs_se, 'o', 'Color', C_M, 'MarkerFaceColor', C_M, 'MarkerSize', 8, 'LineWidth', 1.6, 'CapSize', 8);
        legend(a1, [h1 h2], {'C_{dpmr} 4k_BT a + C_n \sigma_n^2', sprintf('measured  var(\\delta h_{mr}),  control on \\^a,  %d seeds', nS)}, ...
               'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
        if col == 1; ylabel(a1, 'var(\delta h_{mr})   [\mum^2]', 'FontSize', FS, 'FontWeight', 'bold'); end
        xlabel(a1, 'a / a_o   [-]', 'FontSize', FS, 'FontWeight', 'bold'); xlim(a1, XL{col}); ylim(a1, [0 1.1*max(fl)]);
        set(a1,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid(a1,'off');
    end
    fig = fullfile(od, 'verify_cdpmr_hold_estimated.png'); exportgraphics(f, fig, 'Resolution', 150); close(f);
    fprintf('figure -> %s\n', fig);
    out = struct('WB', WB, 'a_bar', AB, 'ahat_bias', EH, 'readout_bias', RM, 'readout_pred', RMp, 'var_ratio', VR, 'var_pred', VRp, 'lam_eff', LE, 'file', fig);
end
