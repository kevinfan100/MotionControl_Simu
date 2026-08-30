% STATUS: ACTIVE (scratch instrument) | PURPOSE: the two C_dpmr pictures, done
%   properly paired and with the estimator arriving the way production does:
%   every run starts in the far field (w_bar 6.67), descends in 2 s to the
%   target height, sits there 4 s; the analysis window is the last 2 s.
%   Same seeds, same trajectory, same window in both arms:
%     arm TRUE  control on a_true  (loop pole = lambda_c; C_dpmr's premise)
%     arm EST   control on a_hat   (production; the loop the estimator runs)
%   Figures (identical style, green = formula, blue = measured):
%     verify_cdpmr_true.png   verify_cdpmr_estimated.png
%   out = verify_cdpmr_descend_hold()          out = verify_cdpmr_descend_hold([], true)  % replot
function out = verify_cdpmr_descend_hold(seeds, replot)
    if nargin < 1 || isempty(seeds); seeds = [7 11 23 42 101 777 1 2 3 4 5 6 8 9 10]; end
    if nargin < 2 || isempty(replot); replot = false; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'apd_acov_meng'); fn = fullfile(od, 'verify_cdpmr_descend_hold.mat');
    pc = physical_constants(); ax = 3; WB = [1.10 1.15 1.20 1.30 1.50 2.0 3.0 6.67];
    T_DESC = 2.0; T_SIM = 0.5 + T_DESC + 1.0 + 3.0; T0 = T_SIM - 2.0;      % hold 2.5 -> 6.5 s, window 4.5 -> 6.5 s
    ARM = {'true', 'est'};

    if ~replot
        nS = numel(seeds); nH = numel(WB); DH = cell(2,nH); AM = DH; AH = DH; AT = zeros(2,nH,nS); cc0 = []; P0 = []; tt = [];
        for a = 1:2
            for ih = 1:nH
                h = WB(ih)*pc.R;
                ov = struct('trajectory_type','osc','h_init',15.0,'h_bottom',h,'amplitude',0,'frequency',1,'n_cycles',1, ...
                            't_hold',0.5,'t_descend_override',T_DESC,'T_sim',T_SIM,'h_min',1.1*pc.R);
                if WB(ih) >= 6.67; ov.h_init = 15.0; ov.t_descend_override = 0.01; end    % far-field point: no descent
                o = struct('arm','best','seeds',seeds,'config_override',ov); if a == 1; o.a_ctrl_override = 'true'; end
                clear run_formC_b motion_control_law_formC_b; [~, O] = evalc("run_formC_b(o);");
                N = numel(O.runs{1}.tout); M = zeros(N,nS); A1 = M; A2 = M;
                for q = 1:nS; r = O.runs{q}; M(:,q) = r.dh_m_out(:,ax); A1(:,q) = r.a_xm_out(:,ax); A2(:,q) = r.a_hat_out(:,ax); AT(a,ih,q) = mean(r.a_true_out(r.tout>=T0,ax)); end
                DH{a,ih} = M; AM{a,ih} = A1; AH{a,ih} = A2;
                if isempty(cc0); cc0 = O.runs{1}.ctrl_const; P0 = O.runs{1}.meta.params_value; tt = O.runs{1}.tout(:); end
                fprintf('collected arm %-4s w_bar %.2f\n', ARM{a}, WB(ih));
            end
        end
        save(fn, 'DH', 'AM', 'AH', 'AT', 'cc0', 'P0', 'tt', 'WB', 'seeds', 'T0', 'T_SIM', '-v7.3');
    else
        S = load(fn); DH = S.DH; AM = S.AM; AH = S.AH; AT = S.AT; cc0 = S.cc0; P0 = S.P0; tt = S.tt; WB = S.WB; seeds = S.seeds; T0 = S.T0;
        nS = numel(seeds); nH = numel(WB);
    end

    fourkT = 4*P0.ctrl.k_B*P0.ctrl.T; s2n = P0.ctrl.sigma2_noise(ax); a_pd = cc0.a_pd; Cd = cc0.C_dpmr; Cn = cc0.C_n;
    ao = P0.common.Ts/P0.common.gamma_N; m = tt >= T0; G = 5; grp = mod(0:nS-1,G)+1;
    V = zeros(2,nH); Vse = V; F = V; AB = V; EH = V; RM = V; Vq = zeros(2,nH,nS);
    for a = 1:2
        for ih = 1:nH
            M = DH{a,ih}; N = size(M,1); R = zeros(N,nS);
            for q = 1:nS; md = zeros(N+1,1); for k = 1:N; md(k+1) = (1-a_pd)*md(k) + a_pd*M(k,q); end; R(:,q) = M(:,q) - md(2:end); end
            ah = mean(AT(a,ih,:)); AB(a,ih) = ah/ao; F(a,ih) = Cd*fourkT*ah + Cn*s2n;
            V(a,ih) = mean(var(R(m,:),0,2)); gv = zeros(1,G); for g = 1:G; gv(g) = mean(var(R(m,grp==g),0,2)); end; Vse(a,ih) = std(gv)/sqrt(G);
            for q = 1:nS; Vq(a,ih,q) = var(R(m,q)); end
            EH(a,ih) = mean(mean(AH{a,ih}(m,:),1)./squeeze(AT(a,ih,:)).') - 1;
            RM(a,ih) = mean(mean(AM{a,ih}(m,:),1)./squeeze(AT(a,ih,:)).') - 1;
        end
    end
    fprintf('\nC_dpmr, far-field start -> 2 s descent -> 4 s hold, window last 2 s, %d seeds, paired\n', nS);
    fprintf('%6s %6s | %-22s | %-22s | %-16s | %-10s %-10s\n', 'w_bar', 'a_bar', 'TRUE  Var/formula', 'EST   Var/formula', 'paired EST-TRUE', 'a_hat/a-1', 'a_m/a-1(EST)');
    for ih = 1:nH
        d = squeeze(Vq(2,ih,:)-Vq(1,ih,:))/F(1,ih);
        fprintf('%6.2f %6.3f | %6.3f +- %.3f        | %6.3f +- %.3f        | %+6.3f +- %.3f  | %+8.2f%%  %+8.2f%%\n', WB(ih), AB(1,ih), V(1,ih)/F(1,ih), Vse(1,ih)/F(1,ih), V(2,ih)/F(2,ih), Vse(2,ih)/F(2,ih), mean(d), std(d)/sqrt(nS), 100*EH(2,ih), 100*RM(2,ih));
    end

    C_M = [0 0.2 0.9]; C_F = [0.10 0.60 0.20]; FS = 18; LFS = 13; AXLW = 2.0; XL = {[0 0.9],[0.07 0.23]};
    NAME = {'control on a_{true}', 'control on \^a  (production)'}; FILE = {'verify_cdpmr_true.png', 'verify_cdpmr_estimated.png'}; files = cell(1,2);
    for a = 1:2
        f = figure('Position',[10 10 1700 700],'Color','w','NumberTitle','off','Visible','off');
        tl = tiledlayout(f,1,2,'TileSpacing','compact','Padding','compact');
        for col = 1:2
            a1 = nexttile(tl, col); hold(a1,'on');
            ag = linspace(XL{col}(1), XL{col}(2), 200); fl = Cd*fourkT*ag*ao + Cn*s2n;
            h1 = plot(a1, ag, fl, '-', 'Color', C_F, 'LineWidth', 2.8);
            h2 = errorbar(a1, AB(a,:), V(a,:), Vse(a,:), 'o', 'Color', C_M, 'MarkerFaceColor', C_M, 'MarkerSize', 8, 'LineWidth', 1.6, 'CapSize', 8);
            legend(a1, [h1 h2], {'C_{dpmr} 4k_BT a + C_n \sigma_n^2', sprintf('measured  var(\\delta h_{mr}),  %s,  %d seeds', NAME{a}, nS)}, ...
                   'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
            if col == 1; ylabel(a1, 'var(\delta h_{mr})   [\mum^2]', 'FontSize', FS, 'FontWeight', 'bold'); end
            xlabel(a1, 'a / a_o   [-]', 'FontSize', FS, 'FontWeight', 'bold'); xlim(a1, XL{col}); ylim(a1, [0 1.1*max(fl)]);
            set(a1,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid(a1,'off');
        end
        files{a} = fullfile(od, FILE{a}); exportgraphics(f, files{a}, 'Resolution', 150); close(f); fprintf('figure -> %s\n', files{a});
    end
    out = struct('WB', WB, 'a_bar', AB, 'V', V, 'Vse', Vse, 'F', F, 'ahat_bias', EH, 'readout_bias', RM, 'files', {files});
end
