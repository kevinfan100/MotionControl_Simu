% STATUS: ACTIVE (scratch) | PURPOSE: the y2 readout level near the wall. The hold level mode
%   (0902 tex, last section) says a clean predict converges to est - true = +beta2/(1-S), with
%   beta2 = readout level offset (a_wm - a_true at fixed truth). Measure it directly, same 10
%   seeds, both trajectories, two arms on the recipe exact + @est + pred_mean2 (no kr1):
%     loop      normal closed loop (readout carries the echo S (a_hat - a) on top of beta2)
%     trueloop  a_ctrl_override = 'true': the loop runs on the true gain, the estimate does not
%               act, so a_wm - a_true IS the readout chain's own offset (no echo)
%   Readout y2[k] = a_bar_w[k-d] + n_a, so a_wm(k) is compared with a_true(k-2); a_hat(k) with a_true(k).
%   PRE-REGISTERED (hold, a_wm - a_true, a_bar units): canon +0.0025..+0.0035, meng +0.001..+0.002
%   (from the long-hold slopes / kappa / (1-S)); both arms equal within SEM if the offset is the
%   readout chain's own; loop arm = trueloop + 0.32 (a_hat - a) otherwise.
%   Output: y2_readout_level<tag>.mat/.png; run_y2_readout_level(1:10, 10, '_long') extends the final hold by 10 s
function out = run_y2_readout_level(seeds, extra_hold, tag)
    if nargin < 1 || isempty(seeds); seeds = 1:10; end
    if nargin < 2 || isempty(extra_hold); extra_hold = 0; end     % [s] final hold extension (sharpens beta2: SEM ~ 1/sqrt(T))
    if nargin < 3 || isempty(tag); tag = ''; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'apd_acov_meng');  pc = physical_constants();  S_ECHO = 0.32;
    TR = {'canon','meng'};  ARM = {'loop','trueloop'};  out = struct();  nS = numel(seeds);
    for it = 1:2
        traj = TR{it};
        switch traj
            case 'meng'; OV = struct('trajectory_type','osc','h_init',15,'h_bottom',2.5,'amplitude',0,'frequency',1,'n_cycles',1,'t_hold',0.5,'t_descend_override',10,'T_sim',12.5 + extra_hold,'h_min',2.475); cfg0 = OV;
            case 'canon'; cfg0 = canonical_scenario(0.05, 1.1, 'deep'); OV = struct('T_sim', cfg0.T_sim + extra_hold); cfg0.T_sim = cfg0.T_sim + extra_hold;
        end
        w0bar = cfg0.h_init / pc.R; [~, cp] = calc_correction_functions(w0bar); at = 1/cp; ws0 = 1 + w0bar - 1/((8/9)*(1 - at));
        t3 = cfg0.t_hold + cfg0.t_descend_override + cfg0.n_cycles/cfg0.frequency;  t2 = cfg0.t_hold + cfg0.t_descend_override;
        for a = 1:2
            o = struct('arm','best','ap_known',true,'ap_known_at','est','app_known',true, ...
                       'ctrl_const_override',struct('lock_b',true,'ws0_perp',ws0,'law_exact_step',true,'pred_mean2',true), ...
                       'config_override',OV,'scenario','deep','verbose',false,'seeds',seeds,'log_P_full',false);
            if strcmp(ARM{a}, 'trueloop'); o.a_ctrl_override = 'true'; end
            clear run_formC_b motion_control_law_formC_b;
            evalc('R = run_formC_b(o);');
            t = R.runs{1}.tout(:); N = numel(t);
            AW = zeros(N,nS); AT = AW; AH = AW; HB = AW; I2 = AW;
            for q = 1:nS
                r = R.runs{q}; ad = r.a_hat_out(1,3)/r.a_bar_hat_out(1,3);
                AW(:,q) = r.a_xm_out(:,3)/ad; AT(:,q) = r.a_true_out(:,3)/ad; AH(:,q) = r.a_bar_hat_out(:,3);
                HB(:,q) = r.h_bar_true_out(:,1); I2(:,q) = r.innov_y2_out(:,3);
            end
            clear R;
            ATd = [AT(1:2,:); AT(1:end-2,:)];                       % a_true(k-2) for the delayed readout
            dW = AW - ATd;  dH = AH - AT;                            % readout - truth, estimate - truth [a_bar]
            fprintf('\n[%s %-8s] health: min w %.4f | min a_wm %.4f | NaN %d\n', traj, ARM{a}, min(HB(:)), min(AW(:)), sum(~isfinite(dW(:))));
            switch traj
                case 'canon'; SEG = {'osc 1.5-3.5', t>1.5&t<=3.5; 'hold >3.5', t>t3; 'hold last 1 s', t>cfg0.T_sim-1; 'hold >t3+1', t>t3+1};
                case 'meng';  SEG = {'near 8-10.5', t>8&t<=t2; 'hold >10.5', t>t2; 'hold last 1 s', t>cfg0.T_sim-1; 'hold >t3+1', t>t3+1};
            end
            fprintf('   %-14s %12s %12s %12s %10s | %12s\n', 'segment', 'a_wm-a_true', '(SEM)', 'a_hat-a_true', '(SEM)', 'beta2_impl');
            for si = 1:size(SEG,1)
                m = SEG{si,2}; pw = mean(dW(m,:),1); ph = mean(dH(m,:),1); at_m = mean(AT(m,:),'all');
                if strcmp(ARM{a},'loop'); b2 = mean(pw) - S_ECHO*mean(ph); else; b2 = mean(pw); end
                fprintf('   %-14s %+12.5f %12.5f %+12.5f %10.5f | %+12.5f  (%.1f%% of a_true %.4f)\n', SEG{si,1}, mean(pw), std(pw)/sqrt(nS), mean(ph), std(ph)/sqrt(nS), b2, 100*b2/at_m, at_m);
            end
            EDG = [1.08 1.2 1.6 2.5 4 Inf];  fprintf('   height bins (a_wm - a_true, seed mean +- SEM, whole run):');
            for b = 1:numel(EDG)-1
                pw = arrayfun(@(q) mean(dW(HB(:,q)>=EDG(b) & HB(:,q)<EDG(b+1), q)), 1:nS);
                fprintf('  [%.2f,%.2f) %+.5f (%.5f)', EDG(b), EDG(b+1), mean(pw,'omitnan'), std(pw,'omitnan')/sqrt(nS));
            end; fprintf('\n');
            out.(traj).(ARM{a}) = struct('t',t,'AW',AW,'AT',AT,'AH',AH,'HB',HB,'I2',I2,'dW',dW,'dH',dH,'phases',[t2 t3 cfg0.T_sim]);
        end
    end
    save(fullfile(od, sprintf('y2_readout_level%s.mat', tag)), '-struct', 'out', '-v7.3');
    % --- figure: row 1 readout - truth (both arms) and estimate - truth (loop), row 2 a_true and a_wm levels ---
    SM = round(0.5/pc.Ts);  COL = {[0.49 0.18 0.56], [0.85 0.45 0.10]};  FS = 14;
    f = figure('Units','inches','Position',[0 0 14 9], 'Color','w', 'Visible','off'); tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
    for it = 1:2
        traj = TR{it}; nexttile(it); hold on; yline(0,'-','Color',[0.55 0.55 0.55],'HandleVisibility','off'); H = gobjects(1,3); L = cell(1,3);
        for a = 1:2
            d = out.(traj).(ARM{a}); Es = movmean(d.dW, SM, 1); m = mean(Es,2); s = std(Es,0,2)/sqrt(nS);
            fill([d.t; flipud(d.t)], [m+2*s; flipud(m-2*s)], COL{a}, 'FaceAlpha', 0.18, 'EdgeColor','none', 'HandleVisibility','off');
            H(a) = plot(d.t, m, '-', 'Color', COL{a}, 'LineWidth', 2.2); L{a} = sprintf('a_{wm} - a_z/a_{nom}   %s', ARM{a});
        end
        d = out.(traj).loop; H(3) = plot(d.t, movmean(mean(d.dH,2), SM), '--', 'Color', [0 0.2 0.9], 'LineWidth', 1.6); L{3} = '\^a_z - a_z   (loop)';
        xline(d.phases(2), '--', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.0, 'HandleVisibility','off');
        legend(H, L, 'Location','northoutside','Orientation','horizontal','FontSize',10,'FontWeight','bold','Box','on');
        if it == 1; ylabel('readout / estimate minus truth  [a_{nom}], 10-seed mean, 0.5 s, \pm 2 SEM', 'FontSize', 11, 'FontWeight','bold'); end
        xlim([0 ceil(d.t(end))]); ylim([-4e-3 8e-3]); set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',1.6,'Box','on'); grid off;
        title(traj, 'FontSize', FS+1, 'FontWeight','bold');
        nexttile(2+it); hold on;
        H2 = gobjects(1,3);
        H2(1) = plot(d.t, mean(d.AT,2), '-', 'Color', [0.8 0 0], 'LineWidth', 2.0);
        H2(2) = plot(d.t, movmean(mean(d.AW,2), SM), '-', 'Color', COL{1}, 'LineWidth', 1.6);
        H2(3) = plot(d.t, movmean(mean(out.(traj).trueloop.AW,2), SM), '-', 'Color', COL{2}, 'LineWidth', 1.6);
        legend(H2, {'a_z/a_{nom}  true', 'a_{wm}  loop (0.5 s mean)', 'a_{wm}  trueloop (0.5 s mean)'}, 'Location','northoutside','Orientation','horizontal','FontSize',10,'FontWeight','bold','Box','on');
        if it == 1; ylabel('gain / a_{nom}', 'FontSize', FS, 'FontWeight','bold'); end
        xlabel('time  [s]', 'FontSize', FS, 'FontWeight','bold'); xlim([0 ceil(d.t(end))]); set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',1.6,'Box','on'); grid off;
    end
    png = fullfile(od, sprintf('y2_readout_level%s.png', tag)); exportgraphics(f, png, 'Resolution', 150); close(f); fprintf('wrote %s\n', png);
end
