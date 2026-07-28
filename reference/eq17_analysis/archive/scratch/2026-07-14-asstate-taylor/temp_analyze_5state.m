% TEMP_ANALYZE_5STATE  Metrics + figure for the four-arm a'-as-state study
% (aprime_source='state', 5state_taylor_aprime.pdf). Reads the per-run caches
% written by temp_compare_5state and writes summary.txt + fig under
% test_results/aprime_4state/state5/.
%
%   Conventions from analyze_aprime_4state (z axis, noisy runs only, exclude
%   diverged): osc window [2.5,3.5) s, descend window [0.7,1.3] s.
%
%   Temp script - delete after use.

seeds = 1:20;

script_dir   = fileparts(mfilename('fullpath'));
project_root = fileparts(fileparts(script_dir));
addpath(fullfile(project_root, 'model'), fullfile(project_root, 'model', 'config'), ...
        fullfile(project_root, 'model', 'wall_effect'), ...
        fullfile(project_root, 'model', 'thermal_force'), ...
        fullfile(project_root, 'model', 'trajectory'), ...
        fullfile(project_root, 'model', 'controller'), ...
        fullfile(project_root, 'model', 'dual_track'), script_dir);

out_dir   = fullfile(project_root, 'test_results', 'aprime_4state', 'state5');
cache_dir = fullfile(out_dir, 'cache');

arm_names = {'REF', 'A2', 'A6a', 'A6b'};
n_arm = numel(arm_names);

% --- shared desired trajectory + oracle a'_true (one clean run) ---
cfg = user_config();
cfg.eq17_variant='4state'; cfg.trajectory_type='osc';
cfg.h_init=50; cfg.h_bottom=2.7; cfg.amplitude=2.5; cfg.frequency=1;
cfg.n_cycles=2; cfg.t_hold=0.5; cfg.t_descend_override=1.0; cfg.T_sim=4.0;
pc=physical_constants(); cfg.h_min=1.05*pc.R;
cfg.ctrl_enable=true; cfg.thermal_enable=false; cfg.meas_noise_enable=false;
cfg.lambda_c=0.7; cfg.a_pd=0.05; cfg.a_cov=0.05;
cfg.meas_noise_std=[0.00062;0.00057;0.00331]; cfg.suppress_xD=true; cfg.h_bar_safe=1;
so0 = run_pure_simulation(cfg, struct('seed',0,'verbose',false,'collect_diag',false,'use_true_gain',true));
t   = so0.tout;
P   = so0.meta.params_value;
w_hat = P.wall.w_hat; pz = P.wall.pz; R = P.common.R;
a_nom = P.ctrl.Ts / P.ctrl.gamma;
hbd = max((so0.p_d_out * w_hat - pz) / R, 1.001);
N_t = numel(t);
ap_oracle = zeros(N_t, 1);
for k = 1:N_t
    [~, c_pe, drv] = calc_correction_functions(hbd(k), true);
    ap_oracle(k)   = -(a_nom / c_pe) * drv.K_h_perp / R;   % z-axis a'_true(t) [um/pN/um]
end

osc = (t >= 2.5) & (t < 3.5);
des = (t >= 0.7) & (t <= 1.3);
ap_osc_ref = mean(ap_oracle(osc));      % oracle a' osc-window mean (rel-error denom)

% --- gather per-arm stacks (noisy, non-diverged) ---
stacks = struct();
summ   = struct();
for ai = 1:n_arm
    nm = arm_names{ai};
    dxc={}; ahz={}; atz={}; apz={}; p55={}; okseed=[];
    ndiv = 0;
    det = load(fullfile(cache_dir, sprintf('%s_s00.mat', nm)));
    ndiv_det = det.diverged;
    for s = seeds
        r = load(fullfile(cache_dir, sprintf('%s_s%02d.mat', nm, s)));
        if r.diverged; ndiv = ndiv + 1; continue; end
        dxc{end+1}=r.dx; ahz{end+1}=r.a_hat_z; atz{end+1}=r.a_true_z; %#ok<*SAGROW>
        apz{end+1}=r.aprime_z; p55{end+1}=r.P55_z; okseed(end+1)=s;
    end
    Ns = numel(okseed);
    DX = [dxc{:}]; AH=[ahz{:}]; AT=[atz{:}]; AP=[apz{:}]; PP=[p55{:}];  % [N_t x Ns]

    % a_hat metrics (osc window)
    rel = AH(osc,:)./AT(osc,:) - 1;                 % [Nosc x Ns]
    per_seed_bias = mean(rel,1);
    bias = mean(per_seed_bias); bias_sem = std(per_seed_bias)/sqrt(Ns);
    a_std = std(rel(:));
    track_nm = 1e3 * std(reshape(DX(osc,:),[],1));

    summ.(nm) = struct('Ns',Ns,'ndiv',ndiv+ndiv_det,'bias',bias,'bias_sem',bias_sem, ...
                       'a_std',a_std,'track_nm',track_nm);
    stacks.(nm) = struct('DX',DX,'AH',AH,'AT',AT,'AP',AP,'PP',PP,'okseed',okseed,'Ns',Ns);
end

% --- a' metrics for A6a/A6b (both windows), P55 honesty ---
ap_arms = {'A6a','A6b'};
for ii = 1:2
    nm = ap_arms{ii}; st = stacks.(nm); Ns = st.Ns;
    err = st.AP - ap_oracle;                        % [N_t x Ns] a' error [um/pN/um]
    for W = ["des","osc"]
        if W=="des"; msk=des; else; msk=osc; end
        per_seed = mean(err(msk,:),1);              % [1 x Ns]
        m_abs = mean(per_seed);                     % [um/pN/um]
        sem   = std(per_seed)/sqrt(Ns);
        pstd  = std(reshape(err(msk,:),[],1));
        rel   = m_abs / ap_osc_ref;                 % relative to oracle osc mean
        summ.(nm).(sprintf('ap_%s_abs_nm', W))  = 1e3*m_abs;   % nm/pN/um
        summ.(nm).(sprintf('ap_%s_sem_nm', W))  = 1e3*sem;
        summ.(nm).(sprintf('ap_%s_pstd_nm', W)) = 1e3*pstd;
        summ.(nm).(sprintf('ap_%s_rel', W))     = rel;
    end
    % P55 honesty: median over osc of mean-seed P55 / cross-seed var(a' error)
    if Ns >= 2 && any(st.PP(:) ~= 0)
        p55_mean = mean(st.PP,2);
        var_err  = var(err,0,2);
        ratio    = p55_mean(osc)./var_err(osc);
        summ.(nm).p55_ratio = median(ratio);
    else
        summ.(nm).p55_ratio = NaN;
    end
end

% ================= console + summary.txt =================
if ~exist(out_dir,'dir'); mkdir(out_dir); end
fid = fopen(fullfile(out_dir,'summary.txt'),'w');
for fo = [1, fid]
    fprintf(fo,'a''-as-state four-arm study (aprime_source=state)  z axis, osc [2.5,3.5) s, descend [0.7,1.3] s\n');
    fprintf(fo,'oracle a''_true osc-window mean = %.4e um/pN/um (= %.3f nm/pN/um)\n\n', ap_osc_ref, 1e3*ap_osc_ref);
    fprintf(fo,'--- a_hat gain metrics (all arms) ---\n');
    fprintf(fo,'%-4s %-4s %-4s %11s %9s %11s\n','arm','Ns','div','bias[%]','std[%]','track[nm]');
    for ai=1:n_arm
        nm=arm_names{ai}; m=summ.(nm);
        fprintf(fo,'%-4s %-4d %-4d %+7.2f+-%.2f %9.2f %11.1f\n', ...
            nm,m.Ns,m.ndiv,100*m.bias,100*m.bias_sem,100*m.a_std,m.track_nm);
    end
    fprintf(fo,'\n--- a'' slope metrics (A6a/A6b), abs error in nm/pN/um, rel to oracle osc mean ---\n');
    fprintf(fo,'%-4s %-7s %14s %10s %12s\n','arm','window','abs[nm]+-SEM','rel[%]','pstd[nm]');
    for ii=1:2
        nm=ap_arms{ii}; m=summ.(nm);
        fprintf(fo,'%-4s %-7s %8.3f+-%.3f %9.1f %11.3f\n', nm,'descend', ...
            m.ap_des_abs_nm,m.ap_des_sem_nm,100*m.ap_des_rel,m.ap_des_pstd_nm);
        fprintf(fo,'%-4s %-7s %8.3f+-%.3f %9.1f %11.3f\n', nm,'osc', ...
            m.ap_osc_abs_nm,m.ap_osc_sem_nm,100*m.ap_osc_rel,m.ap_osc_pstd_nm);
    end
    fprintf(fo,'\n--- P55 honesty (median osc  mean-seed P55 / cross-seed var(a'' err)) ---\n');
    fprintf(fo,'A6a: %.3f    A6b: %.3f   (~1 honest, <1 overconfident, >1 conservative)\n', ...
        summ.A6a.p55_ratio, summ.A6b.p55_ratio);
end
fclose(fid);

% ================= FIG 1: seed-1 overlay (2x1) =================
% pick seed 1 if present & non-diverged in each arm, else first available
pick = @(st) find(st.okseed==1,1);
COL_OR=[0 0 0]; COL_A6a=[0 0.35 0.8]; COL_A6b=[0.85 0.2 0.1];
COL_A2=[0 0.6 0]; FS=18; LFS=13; AXLW=2.0; T_END=ceil(t(end));

sA6a = stacks.A6a; sA6b = stacks.A6b; sA2 = stacks.A2;
i6a = pick(sA6a); if isempty(i6a); i6a=1; end
i6b = pick(sA6b); if isempty(i6b); i6b=1; end
i2  = pick(sA2);  if isempty(i2);  i2=1;  end

f = figure('Position',[80 80 1150 820],'Color','w','NumberTitle','off','Visible','off');
tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

COL_TRU = [0 0.6 0]; COL_KNW = [0.8 0 0]; COL_STA = [0 0.35 0.8];
nexttile; hold on;                                   % --- top: a_hat overlay
h1 = plot(t, sA6a.AT(:,i6a)*1e3, '-', 'Color', COL_TRU, 'LineWidth', 3.0, 'DisplayName','a_{true}');
h2 = plot(t, sA2.AH(:,i2)  *1e3, '-', 'Color', COL_KNW, 'LineWidth', 1.4, 'DisplayName','a\_hat, a'' known c(h)');
h3 = plot(t, sA6a.AH(:,i6a)*1e3, '-', 'Color', COL_STA, 'LineWidth', 1.4, 'DisplayName','a\_hat, a'' state');
xline(0.5,'--','Color',[0.5 0.5 0.5],'LineWidth',1.0);
xline(1.5,'--','Color',[0.5 0.5 0.5],'LineWidth',1.0);
xline(3.5,'--','Color',[0.5 0.5 0.5],'LineWidth',1.0);
xlim([0 T_END]);
ylabel('a_z  (nm/pN)','FontSize',FS,'FontWeight','bold');
legend([h1 h2 h3],'Location','northoutside','Orientation','horizontal','FontSize',LFS,'Box','on');
set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off;

nexttile; hold on;                                   % --- bottom: a' overlay
g1 = plot(t, ap_oracle*1e3, '-', 'Color', COL_TRU, 'LineWidth', 3.0, 'DisplayName','a''_{true}');
g2 = plot(t, sA2.AP(:,i2)  *1e3, '--', 'Color', COL_KNW, 'LineWidth', 1.4, 'DisplayName','a'' known c(h)');
g3 = plot(t, sA6a.AP(:,i6a)*1e3, '-', 'Color', COL_STA, 'LineWidth', 1.4, 'DisplayName','a''\_hat state');
xline(0.5,'--','Color',[0.5 0.5 0.5],'LineWidth',1.0);
xline(1.5,'--','Color',[0.5 0.5 0.5],'LineWidth',1.0);
xline(3.5,'--','Color',[0.5 0.5 0.5],'LineWidth',1.0);
yline(0,'-','Color',[0.7 0.7 0.7],'LineWidth',0.8);
xlim([0 T_END]);
ylabel('a''_z  (nm/pN/\mum)','FontSize',FS,'FontWeight','bold');
xlabel('Time (sec)','FontSize',FS,'FontWeight','bold');
legend([g1 g2 g3],'Location','northoutside','Orientation','horizontal','FontSize',LFS,'Box','on');
set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off;

fig_path = fullfile(out_dir,'fig_state5_overlay.png');
exportgraphics(f, fig_path, 'Resolution', 150);
fig_doc = fullfile(project_root, 'reference', 'eq17_analysis', 'derivation', ...
                   'figures', 'fig_5state_aprime_overlay_z.png');
exportgraphics(f, fig_doc, 'Resolution', 150);
close(f);
fprintf('\n[analyze5] summary.txt + %s written\n', fig_path);
