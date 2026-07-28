function out = temp_aprime_order_compare(varargin)
%TEMP_APRIME_ORDER_COMPARE  1st- vs 2nd-order a' modeling comparison (canonical 1Hz).
%
%   out = temp_aprime_order_compare()                % seed 1, beta 0.99
%   out = temp_aprime_order_compare('seed',2,...)    % override any PAR field
%
%   Tests whether promoting a' from a 1st-order random walk (arm 1, 5-state) to a
%   2nd-order Singer model with a rate state delta_a' (arm 2, 6-state) tracks the
%   oscillating a'(h_d(t)) with less lag -- under the REAL weak observability
%   (a' seen only via F_e(4,5)=Delta_h_d; aprime_source='state').
%
%   FAIR Q (bypass unknown-c for the comparison): both arms fed an ORACLE process
%   noise computed from the true wall curve. Q55 (arm 1) = mean-square 1st diff of
%   the oracle a'(h_d[k]); Q66 (arm 2) = mean-square 2nd diff, both over the osc
%   segment. Isolates "architecture (order)" from "Q value". No a''' needed.
%
%   Scenario: canonical 1Hz, hold(0.5s)->descent(1.0s)->osc(4 cycles), h_bar 22->2.
%   Outputs fig1 (3-panel: a'(t)/a_z(t)/tracking) + fig2 (osc-zoom a'(t)); prints
%   metrics (a' RMS / phase lag / corr / honesty / tracking / a_z) per hold/osc seg.
%
%   TEMP diagnostic (chat 2026-07-22); nothing committed, delete after use.

    [script_dir, ~, ~] = fileparts(mfilename('fullpath'));
    project_root = fileparts(script_dir);
    addpath(fullfile(project_root, 'model'), fullfile(project_root, 'model', 'config'), ...
            fullfile(project_root, 'model', 'wall_effect'), ...
            fullfile(project_root, 'model', 'thermal_force'), ...
            fullfile(project_root, 'model', 'trajectory'), ...
            fullfile(project_root, 'model', 'controller'), ...
            fullfile(project_root, 'model', 'dual_track'), script_dir);
    out_dir = fullfile(project_root, 'test_results', 'temp_aprime_order_figs');
    if ~exist(out_dir, 'dir'); mkdir(out_dir); end

    PAR.seed = 1;
    PAR.beta = 1.0;         % arm-2 rate pole (1 = IRW free integrator; <1 = Singer decay)
    PAR.replot = false;     % re-draw figures from saved results.mat (skip the 3 sims)
    for i = 1:2:numel(varargin); PAR.(varargin{i}) = varargin{i+1}; end
    if PAR.replot
        L = load(fullfile(script_dir, 'temp_aprime_order_results.mat'));
        out = L.out; fig_main(out, out.S); fig_osc_zoom(out, out.S);
        fprintf('replotted from saved results (clipped y-axes).\n'); return;
    end

    % ------------------------------------------------------------------
    % Canonical 1Hz scenario (= temp_var_centered_verify), a'-as-state weak obs
    % ------------------------------------------------------------------
    pc = physical_constants(); R = pc.R;
    cfg = user_config();
    cfg.eq17_variant='4state'; cfg.trajectory_type='osc';
    cfg.h_init=50; cfg.h_bottom=2.0*R; cfg.amplitude=2.5;
    cfg.frequency=1; cfg.n_cycles=4; cfg.t_hold=0.5; cfg.t_descend_override=1.0;
    cfg.h_min=1.05*R;
    cfg.T_sim = cfg.t_hold + cfg.t_descend_override + cfg.n_cycles/cfg.frequency + 0.5; % 6.0
    cfg.ctrl_enable=true; cfg.thermal_enable=true; cfg.meas_noise_enable=true;
    cfg.lambda_c=0.7; cfg.a_pd=0.05; cfg.a_cov=0.05;
    cfg.meas_noise_std=[0.00062;0.00057;0.00331];
    cfg.suppress_xD=true; cfg.h_bar_safe=1;
    cfg.use_taylor_gain=true; cfg.aprime_source='state';   % weak-obs a'-as-state
    cfg.aprime_ff_oracle=true;   % Option 1: oracle a' FF (a''*Dh_d) + thermal-grounded Q (both arms)

    Pp = calc_simulation_params(cfg); Pp = Pp.Value;
    a_nom = Pp.common.Ts / Pp.ctrl.gamma; Ts = Pp.common.Ts; w = Pp.wall.w_hat; pz = Pp.wall.pz;

    % segment boundaries
    t1 = cfg.t_hold; t2 = t1 + cfg.t_descend_override; t3 = t2 + cfg.n_cycles/cfg.frequency; % 0.5,1.5,5.5
    S.R=R; S.a_nom=a_nom; S.Ts=Ts; S.w=w; S.pz=pz; S.t1=t1; S.t2=t2; S.t3=t3; S.out_dir=out_dir;
    S.freq=cfg.frequency; S.beta=PAR.beta;

    fprintf('==================================================================\n');
    fprintf(' 1st vs 2nd-order a'' compare -- canonical 1Hz, weak-obs as-state, seed %d\n', PAR.seed);
    fprintf(' segments: hold[0,%.1f] descent[%.1f,%.1f] osc[%.1f,%.1f]  beta(arm2)=%.3f\n', ...
            t1, t1, t2, t2, t3, PAR.beta);
    fprintf('==================================================================\n\n');

    % ------------------------------------------------------------------
    % Prelim run: get the (deterministic) desired trajectory p_d -> oracle a', Q
    % ------------------------------------------------------------------
    warning('off','MATLAB:nearlySingularMatrix');
    so0 = temp_run_pure_sim_aprime2nd(cfg, struct('seed',PAR.seed,'collect_diag',true));
    t = so0.tout; N = numel(t);
    h_d = so0.p_d_out*w - pz;                       % desired height [um]
    oracle_ap = zeros(N,1);
    for k = 1:N
        hbd = max(h_d(k)/R, 1.001);
        [~, cp, drv] = calc_correction_functions(hbd, true);
        oracle_ap(k) = -(a_nom/cp) * drv.K_h_perp / R;   % a'(h_d[k]) [um/pN/um]
    end
    % time-varying oracle Q: Q[k] = (unmodeled increment)^2, computed inside the
    % controller from a'(h_d[k]) 1st/2nd differences. Here (reference) report the
    % osc-mean vs descent-mean to show the time variation.
    osc = (t >= t2) & (t <= t3);
    dsc = (t > t1) & (t < t2);
    d1 = [0; diff(oracle_ap)];                          % a'[k]-a'[k-1]
    d2 = [0; 0; oracle_ap(3:end) - 2*oracle_ap(2:end-1) + oracle_ap(1:end-2)];
    Q55 = mean(d1(osc).^2);  Q66 = mean(d2(osc).^2);    % osc-mean (reference)
    Q55d = mean(d1(dsc).^2); Q66d = mean(d2(dsc).^2);   % descent-mean (much larger)
    out.Q55 = Q55; out.Q66 = Q66; out.oracle_ap = oracle_ap; out.t = t;
    fprintf('  time-varying oracle Q[k]=(unmodeled incr)^2  [~0 hold / big descent / oscillating osc]:\n');
    fprintf('    osc-mean     Q55=%.3e  Q66=%.3e  (ratio %.2e)\n', Q55, Q66, Q66/Q55);
    fprintf('    descent-mean Q55=%.3e  Q66=%.3e  (descent >> osc)\n\n', Q55d, Q66d);

    % Option 1 thermal-grounded Q (reference): a''(h_d), sigma2_T = 4 kBT a_det_z,
    % Var(Delta x_ram) = (2/(1+lc)) sigma2_T, Var(Delta^2 x_ram) = (6-8r1+2r2) C_dx sigma2_T.
    kBT = Pp.ctrl.k_B * Pp.ctrl.T; lc = cfg.lambda_c;
    oracle_app = zeros(N,1); adet_z = zeros(N,1);
    for k = 1:N
        hbd = max(h_d(k)/R, 1.001);
        [~, cp, drv] = calc_correction_functions(hbd, true);
        adet_z(k)     = a_nom/cp;
        oracle_app(k) = -(a_nom/cp)/R^2 * (drv.K_h_prime_perp - drv.K_h_perp^2);  % a''(h_d) [um/pN/um^2]
    end
    C_dx = 2 + 1/(1-lc^2); r1 = lc + 2*(1-lc)/C_dx; r2 = lc*r1;
    s2T = 4*kBT*adet_z;
    Q55_th = oracle_app.^2 .* (2/(1+lc)) .* s2T;                 % 1st-order thermal Q55
    Q66_th = oracle_app.^2 .* ((6-8*r1+2*r2)*C_dx) .* s2T;       % 2nd-order thermal Q66
    out.oracle_app = oracle_app; out.Q55_th = Q55_th; out.Q66_th = Q66_th;
    fprintf('  Option 1 thermal-grounded Q  [a_pp = a''''(h_d), sigma2_T = 4kBT a_z]:\n');
    fprintf('    osc-mean     Q55_th=%.3e  Q66_th=%.3e  | a_pp_z osc-mean %.3e peak %.3e [um/pN/um^2]\n', ...
            mean(Q55_th(osc)), mean(Q66_th(osc)), mean(abs(oracle_app(osc))), max(abs(oracle_app)));
    fprintf('    descent-mean Q55_th=%.3e  Q66_th=%.3e  (sigma2_T osc-mean %.3e um^2)\n\n', ...
            mean(Q55_th(dsc)), mean(Q66_th(dsc)), mean(s2T(osc)));

    % ------------------------------------------------------------------
    % Arm 1 (order 1, RW) and Arm 2 (order 2, Singer), same seed, oracle Q
    % ------------------------------------------------------------------
    % small c-free far-field prior on a' (both arms): reflects "start is commanded
    % far, a'~=0 with confidence" -> kills the descent-onset overshoot without c.
    P55_small = (0.005 * a_nom / R)^2;   % 0.5% of a_nom/R, c-free (a_nom far-field, R geom)
    S.P55_small = P55_small;
    fprintf('  P55_0 (small c-free far-field prior, both arms) = %.3e  [default would be %.3e]\n\n', ...
            P55_small, (0.1*a_nom/R)^2);
    cfg1 = cfg; cfg1.aprime_order = 1; cfg1.aprime_Q_oracle_tv = true; cfg1.aprime_state_P0 = P55_small;
    so1 = temp_run_pure_sim_aprime2nd(cfg1, struct('seed',PAR.seed,'collect_diag',true));
    % arm 2 = NEW curvature-state model: slot 6 = a'' (curvature), a'[k+1]=a'+Dh_d*a''.
    % P66 = wall-aware [a''(h_bar_init)]^2 (6state_curvature_taylor.tex §7): far-field
    % start -> a''~=0 -> pinned -> no pent-up release at descent onset. Tiny floor.
    P66_small = max(oracle_app(1)^2, (1e-3 * a_nom / R^2)^2);
    cfg2 = cfg; cfg2.aprime_order = 2; cfg2.aprime_curvature_state = true; cfg2.aprime_ff_oracle = false;
    cfg2.aprime_state_P0 = P55_small; cfg2.aprime_delta_P0 = P66_small;
    so2 = temp_run_pure_sim_aprime2nd(cfg2, struct('seed',PAR.seed,'collect_diag',true));
    warning('on','MATLAB:nearlySingularMatrix');

    ap1 = so1.diag.a_prime_used(:,3);  P1 = so1.diag.P_aprime(:,3);
    ap2 = so2.diag.a_prime_used(:,3);  P2 = so2.diag.P_aprime(:,3);
    az_true = so1.a_true_out(:,3);
    az1 = so1.diag.a_hat(:,3);          az2 = so2.diag.a_hat(:,3);
    dxz1 = (so1.p_d_out*w - pz - (so1.p_true_out*w - pz))*1e3;
    dxz2 = (so2.p_d_out*w - pz - (so2.p_true_out*w - pz))*1e3;
    out.ap1=ap1; out.ap2=ap2; out.P1=P1; out.P2=P2; out.az_true=az_true;
    out.az1=az1; out.az2=az2; out.dxz1=dxz1; out.dxz2=dxz2; out.PAR=PAR; out.S=S;

    % ------------------------------------------------------------------
    % Metrics per segment (console; not embedded in PDF)
    % ------------------------------------------------------------------
    hold_win = (t >= 0.1) & (t <= t1);
    osc_win  = (t >= t2+0.5) & (t <= t3);          % skip first half-cycle settling
    per_samp = round((1/S.freq)/Ts);               % samples per osc cycle
    fprintf('  metrics  (a1 = 1st-order, a2 = 2nd-order; a'' in nm/pN/um)\n');
    fprintf('  %-6s %-8s %-9s %-9s %-11s %-9s %-9s\n','seg','arm','RMS','corr','lag[ms]','honesty','trk[nm]');
    out.metrics = struct();
    for seg = {'hold', hold_win; 'osc', osc_win}'
        nm = seg{1}; win = seg{2};
        for arm = 1:2
            if arm==1; ap=ap1; Pp_=P1; dxz=dxz1; else; ap=ap2; Pp_=P2; dxz=dxz2; end
            e = ap(win) - oracle_ap(win);
            rmsv  = rms(e)*1e3;
            cc = corrcoef(ap(win), oracle_ap(win)); corrv = cc(1,2);
            honv = rms(sqrt(max(Pp_(win),0))) / max(rms(e), realmin);
            trkv = std(dxz(win));
            if strcmp(nm,'osc')
                L = xcorr_lag(oracle_ap(win), ap(win), per_samp);  % + = arm lags oracle
                lagms = L*Ts*1e3;
            else
                lagms = NaN;
            end
            fprintf('  %-6s a%-7d %-9.4f %-9.3f %-11s %-9.2f %-9.1f\n', ...
                    nm, arm, rmsv, corrv, num2str_lag(lagms), honv, trkv);
            out.metrics.(sprintf('%s_a%d',nm,arm)) = struct('rms',rmsv,'corr',corrv,'lag_ms',lagms,'honesty',honv,'trk',trkv);
        end
    end
    az_r1 = mean(az1(osc_win)./az_true(osc_win)); az_r2 = mean(az2(osc_win)./az_true(osc_win));
    fprintf('  a_z/true (osc): arm1=%.4f  arm2=%.4f\n', az_r1, az_r2);
    m = out.metrics;
    fprintf('\n  VERDICT (osc): 2nd-order vs 1st  ->  RMS %.4f vs %.4f | lag %.1f vs %.1f ms | corr %.3f vs %.3f\n', ...
            m.osc_a2.rms, m.osc_a1.rms, m.osc_a2.lag_ms, m.osc_a1.lag_ms, m.osc_a2.corr, m.osc_a1.corr);
    fprintf('           hold cost: trk %.1f vs %.1f nm | RMS %.4f vs %.4f\n\n', ...
            m.hold_a2.trk, m.hold_a1.trk, m.hold_a2.rms, m.hold_a1.rms);

    % ------------------------------------------------------------------
    % Figures + save
    % ------------------------------------------------------------------
    fig_main(out, S);
    fig_osc_zoom(out, S);
    save(fullfile(script_dir,'temp_aprime_order_results.mat'),'out');
    fprintf('saved: temp_aprime_order_results.mat\n');
end


%% ==================== xcorr lag (samples; + = sig lags ref) ====================
function L = xcorr_lag(ref, sig, maxlag)
    ref = ref(:) - mean(ref); sig = sig(:) - mean(sig);
    best = 0; bestc = -inf;
    for LL = -maxlag:maxlag
        if LL >= 0
            a = ref(1:end-LL); b = sig(1+LL:end);
        else
            a = ref(1-LL:end); b = sig(1:end+LL);
        end
        if numel(a) < 8; continue; end
        c = sum(a.*b) / sqrt(sum(a.^2)*sum(b.^2) + realmin);
        if c > bestc; bestc = c; best = LL; end
    end
    L = best;
end

function s = num2str_lag(x)
    if isnan(x); s = '--'; else; s = sprintf('%+.1f', x); end
end


%% ==================== figures ====================
function fig_main(out, S)
    GRN=[0 0.6 0]; BLU=[0 0.2 0.8]; RED=[0.8 0 0]; FS=14; LFS=10; LW=1.6;
    t = out.t; T = t(end);
    apHi=max([out.ap1;out.ap2])*1e3; apLo=min([out.ap1;out.ap2])*1e3;   % a' excursions
    azHi=max([out.az1;out.az2])*1e3;                                     % a_z peak
    dxHi=max([out.dxz1;out.dxz2]); dxLo=min([out.dxz1;out.dxz2]);        % tracking excursions
    f = figure('Position',[60 60 1150 1050],'Color','w','Visible','off');
    tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

    % Panel a: a'(t)
    nexttile; hold on;
    h0 = plot(t, out.oracle_ap*1e3,'-','Color',GRN,'LineWidth',2.6,'DisplayName','a''_{true}(h_d(t))');
    h1 = plot(t, out.ap1*1e3,'-','Color',BLU,'LineWidth',1.3,'DisplayName','\^a'' 1st-order');
    h2 = plot(t, out.ap2*1e3,'-','Color',RED,'LineWidth',1.3,'DisplayName','\^a'' 2nd-order');
    for x=[S.t1 S.t2 S.t3]; xline(x,'--','Color',[.6 .6 .6],'LineWidth',1.1,'HandleVisibility','off'); end
    xlim([0 T]); ylim([-1 3.5]); ylabel('a'' (nm/pN/\mum)','FontSize',FS,'FontWeight','bold');
    if apHi>3.5 || apLo<-1
        text(0.985,0.90,sprintf('a'' excursion peak %+.0f / %+.0f (axis clips)',apHi,apLo),'Units','normalized', ...
             'HorizontalAlignment','right','FontSize',LFS-1,'FontWeight','bold','BackgroundColor','w','EdgeColor','k');
    end
    yl=ylim; text(S.t1/2,yl(2)*0.82,'hold','FontSize',LFS,'HorizontalAlignment','center','FontWeight','bold');
    text((S.t1+S.t2)/2,yl(2)*0.82,'descent','FontSize',LFS,'HorizontalAlignment','center','FontWeight','bold');
    text((S.t2+S.t3)/2,yl(2)*0.82,'osc','FontSize',LFS,'HorizontalAlignment','center','FontWeight','bold');
    title('a''(t): does 2nd-order (red) track truth (green) better than 1st (blue)?','FontSize',FS-1,'FontWeight','bold');
    legend([h0 h1 h2],'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
    set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',LW,'Box','on'); grid off;

    % Panel b: a_z(t)
    nexttile; hold on;
    g0 = plot(t, out.az_true*1e3,'-','Color',GRN,'LineWidth',2.6,'DisplayName','a_{z,true}');
    g1 = plot(t, out.az1*1e3,'-','Color',BLU,'LineWidth',1.2,'DisplayName','\^a_z 1st');
    g2 = plot(t, out.az2*1e3,'-','Color',RED,'LineWidth',1.2,'DisplayName','\^a_z 2nd');
    for x=[S.t1 S.t2 S.t3]; xline(x,'--','Color',[.6 .6 .6],'LineWidth',1.1,'HandleVisibility','off'); end
    xlim([0 T]); ylim([0 22]); ylabel('a_z (nm/pN)','FontSize',FS,'FontWeight','bold');
    if azHi > 22
        text(0.985,0.90,sprintf('a_z peak %.0f (axis clips)',azHi),'Units','normalized', ...
             'HorizontalAlignment','right','FontSize',LFS-1,'FontWeight','bold','BackgroundColor','w','EdgeColor','k');
    end
    title('a_z(t): downstream gain','FontSize',FS-1,'FontWeight','bold');
    legend([g0 g1 g2],'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
    set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',LW,'Box','on'); grid off;

    % Panel c: tracking
    nexttile; hold on;
    d1 = plot(t, out.dxz1,'-','Color',BLU,'LineWidth',0.7,'DisplayName','\deltax_z 1st');
    d2 = plot(t, out.dxz2,'-','Color',RED,'LineWidth',0.7,'DisplayName','\deltax_z 2nd');
    for x=[S.t1 S.t2 S.t3]; xline(x,'--','Color',[.6 .6 .6],'LineWidth',1.1,'HandleVisibility','off'); end
    xlim([0 T]); ylim([-150 150]); ylabel('\deltax_z (nm)','FontSize',FS,'FontWeight','bold'); xlabel('Time (sec)','FontSize',FS,'FontWeight','bold');
    if dxHi>150 || dxLo<-150
        text(0.985,0.90,sprintf('\\deltax_z peak %+.0f / %+.0f (axis clips)',dxHi,dxLo),'Units','normalized', ...
             'HorizontalAlignment','right','FontSize',LFS-1,'FontWeight','bold','BackgroundColor','w','EdgeColor','k');
    end
    title('tracking error','FontSize',FS-1,'FontWeight','bold');
    legend([d1 d2],'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
    set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',LW,'Box','on'); grid off;

    exportgraphics(f, fullfile(S.out_dir,'aprime_order_compare.png'),'Resolution',200); close(f);
    fprintf('  [aprime_order_compare]\n');
end

function fig_osc_zoom(out, S)
    GRN=[0 0.6 0]; BLU=[0 0.2 0.8]; RED=[0.8 0 0]; FS=15; LFS=12; LW=1.9;
    t = out.t;
    z0 = S.t2 + 1.0; z1 = min(z0 + 2/S.freq, S.t3);   % ~2 osc cycles
    win = (t >= z0) & (t <= z1);
    f = figure('Position',[60 60 1150 520],'Color','w','Visible','off'); hold on;
    h0 = plot(t(win), out.oracle_ap(win)*1e3,'-','Color',GRN,'LineWidth',3.0,'DisplayName','a''_{true}(h_d(t))');
    h1 = plot(t(win), out.ap1(win)*1e3,'-','Color',BLU,'LineWidth',1.8,'DisplayName','\^a'' 1st-order');
    h2 = plot(t(win), out.ap2(win)*1e3,'-','Color',RED,'LineWidth',1.8,'DisplayName','\^a'' 2nd-order');
    xlim([z0 z1]); ylabel('a'' (nm/pN/\mum)','FontSize',FS,'FontWeight','bold'); xlabel('Time (sec)','FontSize',FS,'FontWeight','bold');
    title('osc-zoom: phase lag of \^a'' vs truth (1st vs 2nd order)','FontSize',FS-1,'FontWeight','bold');
    m = out.metrics;
    text(0.985,0.06,sprintf('osc lag: 1st %+.1f ms, 2nd %+.1f ms', m.osc_a1.lag_ms, m.osc_a2.lag_ms), ...
         'Units','normalized','HorizontalAlignment','right','FontSize',LFS,'FontWeight','bold','BackgroundColor','w','EdgeColor','k');
    legend([h0 h1 h2],'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
    set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',LW,'Box','on'); grid off;
    exportgraphics(f, fullfile(S.out_dir,'aprime_order_osc_zoom.png'),'Resolution',200); close(f);
    fprintf('  [aprime_order_osc_zoom]\n');
end
