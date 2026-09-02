% FORK OF test_script/scratch/run_aptrue_force_step.m (2026-09-02) | PURPOSE: arm 4 (exact +
%   slope@est + pred_mean2) plus the A2 closed-form term a''(1-lc)^2 K31 R1
%   (ctrl_const.pred_mean2_kr1), both trajectories, same 10 seeds, paired against the
%   slope@est arm of run_aptrue_cmd_vs_est.m. PRE-REGISTERED hold (est - true):
%   canon 0 +- 0.0003 (from +0.0006), meng 0 +- 0.0007 (from +0.0020).
%   | EXPIRES: with the parent | 產線改動不會自動跟上
function run_aptrue_kr1(seeds)
    if nargin < 1 || isempty(seeds); seeds = 1:10; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'apd_acov_meng');  pc = physical_constants();
    TR = {'canon','meng'};  RES = struct();
    for it = 1:2
        traj = TR{it};
        switch traj
            case 'meng'
                OV = struct('trajectory_type','osc','h_init',15,'h_bottom',2.5,'amplitude',0,'frequency',1,'n_cycles',1,'t_hold',0.5,'t_descend_override',10,'T_sim',12.5,'h_min',2.475); cfg0 = OV;
            case 'canon'
                OV = struct(); cfg0 = canonical_scenario(0.05, 1.1, 'deep');
        end
        w0bar = cfg0.h_init / pc.R; [~, cp] = calc_correction_functions(w0bar); at = 1/cp; ws0 = 1 + w0bar - 1/((8/9)*(1 - at));
        t3 = cfg0.t_hold + cfg0.t_descend_override + cfg0.n_cycles/cfg0.frequency;  t2 = cfg0.t_hold + cfg0.t_descend_override;
        o = struct('arm','best','ap_known',true,'ap_known_at','est','app_known',true, ...
                   'ctrl_const_override',struct('lock_b',true,'ws0_perp',ws0,'law_exact_step',true,'pred_mean2',true,'pred_mean2_kr1',true), ...
                   'config_override',OV,'scenario','deep','verbose',false,'seeds',seeds,'log_P_full',false);
        clear run_formC_b motion_control_law_formC_b;
        evalc('R = run_formC_b(o);');
        t = R.runs{1}.tout(:); nS = numel(seeds); E = zeros(numel(t), nS); M2 = E; AH = E;
        for q = 1:nS
            r = R.runs{q}; ad = r.a_hat_out(1,3)/r.a_bar_hat_out(1,3);
            AH(:,q) = r.a_bar_hat_out(:,3); E(:,q) = AH(:,q) - r.a_true_out(:,3)/ad; M2(:,q) = r.pred_mean2_out(:,3);
        end
        clear R;
        old = load(fullfile(od, sprintf('aptrue_cmd_vs_est_%s.mat', traj)));  Eo = old.right.E;
        m_h = t > t3; m_o = t > t2 & t <= t3; pm = mean(E(m_h,:),1); po = mean(Eo(m_h,:),1);
        fprintf('[%s kr1] health: min a_hat %.5f | NaN %d | E(0) %+.1e | mean2 sum/seed %+.5f\n', traj, min(AH(:)), sum(~isfinite(E(:))), mean(E(1,:)), mean(sum(M2,1)));
        fprintf('[%s] hold  est(proxy) %+.5f (SEM %.5f) -> est(kr1) %+.5f (SEM %.5f) | paired diff %+.5f (SEM %.5f)\n', traj, mean(po), std(po)/sqrt(nS), mean(pm), std(pm)/sqrt(nS), mean(pm-po), std(pm-po)/sqrt(nS));
        fprintf('[%s] osc   est(proxy) %+.5f -> est(kr1) %+.5f | hold sd across seeds %.5f -> %.5f\n', traj, mean(mean(Eo(m_o,:),1)), mean(mean(E(m_o,:),1)), mean(std(Eo(m_h,:),0,2)), mean(std(E(m_h,:),0,2)));
        RES.(traj) = struct('t', t, 'E', E, 'Eold', Eo, 'M2', M2, 'seeds', seeds);
        save(fullfile(od, sprintf('aptrue_kr1_%s.mat', traj)), '-struct', 'RES', traj);
    end
    FS = 15; C1 = [0.47 0.67 0.19]; C2 = [0.49 0.18 0.56];
    f = figure('Units','inches','Position',[0 0 13 5], 'Color','w', 'Visible','off'); tiledlayout(1,2,'TileSpacing','compact','Padding','compact');
    for it = 1:2
        d = RES.(TR{it}); nexttile; hold on; yline(0,'-','Color',[0.55 0.55 0.55],'HandleVisibility','off');
        h1 = plot(d.t, mean(d.Eold,2), '-', 'Color', C1, 'LineWidth', 1.4);
        h2 = plot(d.t, mean(d.E,2), '-', 'Color', C2, 'LineWidth', 1.4);
        legend([h1 h2], {'slope@est, proxy increment', 'slope@est, + A2 closed form (K_{31} R_1)'}, 'Location','northoutside','Orientation','horizontal','FontSize',10,'FontWeight','bold','Box','on');
        title(sprintf('%s  a''_{true}  exact + pred\\_mean2, %d seeds', TR{it}, numel(d.seeds)), 'FontSize', FS, 'FontWeight','bold');
        if it == 1; ylabel('mean (\^a_z - a_z) / a_{nom}', 'FontSize', FS, 'FontWeight','bold'); end
        xlabel('time  [s]', 'FontSize', FS, 'FontWeight','bold'); xlim([0 ceil(d.t(end))]); ylim([-4e-3 4e-3]);
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',1.6,'Box','on'); grid off;
    end
    png = fullfile(od, 'aptrue_kr1.png'); exportgraphics(f, png, 'Resolution', 150); close(f); fprintf('wrote %s\n', png);
end
