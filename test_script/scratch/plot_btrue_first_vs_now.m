% FORK OF test_script/scratch/plot_aptrue_two_traj_final.m (2026-09-04) | PURPOSE: ONE seed, the b_true arm as it
%   was first run (Euler law step, no mean term, no correlated-noise predict) against the b_true arm with the four
%   blocks of the ladder (exact step + pred_mean2 + nw_mcorr + pred_mean2_e4). Same seed, same seed-at-truth init
%   (ws0_perp), same trajectories (standard length): left Meng ramp, right canon deep. Rows: absolute gain, gain
%   error, same-instant tracking error. Runs the four simulations itself (~1 min) and saves btrue_first_vs_now.mat.
%   | EXPIRES: with the b_true rung | 產線改動不會自動跟上
function plot_btrue_first_vs_now(seed)
    if nargin < 1 || isempty(seed); seed = 7; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'apd_acov_meng');  pc = physical_constants();  R_um = pc.R;
    TR = {'meng','canon'};  NM = {'Meng', 'canon'};
    ARM = {'first','now'};
    KN  = {struct(), struct('law_exact_step',true,'pred_mean2',true,'nw_mcorr',true,'pred_mean2_e4',true)};
    out = struct('seed', seed);
    for it = 1:2
        switch TR{it}
            case 'meng'; OV = struct('trajectory_type','osc','h_init',15,'h_bottom',2.5,'amplitude',0,'frequency',1,'n_cycles',1,'t_hold',0.5,'t_descend_override',10,'T_sim',12.5,'h_min',2.475); cfg0 = OV;
            case 'canon'; OV = struct(); cfg0 = canonical_scenario(0.05, 1.1, 'deep');
        end
        w0bar = cfg0.h_init / pc.R; [~, cp] = calc_correction_functions(w0bar); at = 1/cp; ws0 = 1 + w0bar - 1/((8/9)*(1 - at));
        t3 = cfg0.t_hold + cfg0.t_descend_override + cfg0.n_cycles/cfg0.frequency;
        for a = 1:2
            cc = struct('ws0_perp', ws0);  fn = fieldnames(KN{a}); for i = 1:numel(fn); cc.(fn{i}) = KN{a}.(fn{i}); end
            o = struct('arm','best','b_true',true,'b_true_at','true','ctrl_const_override',cc, ...
                       'config_override',OV,'scenario','deep','verbose',false,'seeds',seed,'log_P_full',false);
            clear run_formC_b motion_control_law_formC_b;
            evalc('Rr = run_formC_b(o);');  r = Rr.runs{1};  ad = r.a_hat_out(1,3)/r.a_bar_hat_out(1,3);
            D = struct('t', r.tout(:), 'AH', r.a_bar_hat_out(:,3), 'AT', r.a_true_out(:,3)/ad, 'HB', r.h_bar_true_out(:,1), ...
                       'hd', r.p_d_out(:,3)/r.R, 'sP', sqrt(max(r.P_a_out(:,3), 0)), 't_hold', t3);
            D.E = D.AH - D.AT;  out.(TR{it}).(ARM{a}) = D;  clear Rr r;
            mh = D.t > t3;
            fprintf('[%s %-5s seed %d] hold est-true mean %+.5f | hold sd %.5f | end-of-descent %+.5f | min w %.4f\n', ...
                TR{it}, ARM{a}, seed, mean(D.E(mh)), std(D.E(mh)), D.E(find(D.t > cfg0.t_hold + cfg0.t_descend_override, 1)), min(D.HB));
        end
    end
    save(fullfile(od, 'btrue_first_vs_now.mat'), '-struct', 'out', '-v7.3');
    % ---------------- figure ----------------
    COL_TRUE = [0.8 0 0]; COL_FIRST = [0.55 0.74 0.96]; COL_NOW = [0 0.2 0.9]; FS = 15; LFS = 11; AXLW = 1.8;
    f = figure('Units','inches','Position',[0 0 13 11.7], 'Color','w', 'Visible','off');
    tiledlayout(3, 2, 'TileSpacing','compact', 'Padding','compact');
    for a = 1:2
        F = out.(TR{a}).first;  N = out.(TR{a}).now;  t = N.t;  T_END = ceil(t(end));  TH = N.t_hold;
        nexttile(a); hold on;
        ht = plot(t, N.AT, '-', 'Color', COL_TRUE, 'LineWidth', 2.0);
        hf = plot(t, F.AH, '-', 'Color', COL_FIRST, 'LineWidth', 1.4);
        hn = plot(t, N.AH, '-', 'Color', COL_NOW, 'LineWidth', 1.2);
        legend([ht hf hn], {'a_z / a_{nom}  true', '\^a_z / a_{nom}  b_{true} first (Euler)', '\^a_z / a_{nom}  b_{true} now (4 blocks)'}, ...
               'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
        if a == 1; ylabel('a_z / a_{nom}', 'FontSize', FS, 'FontWeight','bold'); end
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off; xlim([0 T_END]);
        xline(TH, '--', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.0, 'HandleVisibility', 'off');
        nexttile(a + 2); hold on; yline(0, '-', 'Color', [0.55 0.55 0.55], 'HandleVisibility', 'off');
        fill([t; flipud(t)], [2*N.sP; flipud(-2*N.sP)], COL_NOW, 'FaceAlpha', 0.10, 'EdgeColor', 'none', 'HandleVisibility', 'off');
        hf = plot(t, F.E, '-', 'Color', COL_FIRST, 'LineWidth', 1.4);
        hn = plot(t, N.E, '-', 'Color', COL_NOW, 'LineWidth', 1.2);
        legend([hf hn], {'(\^a_z - a_z)/a_{nom}  first', '(\^a_z - a_z)/a_{nom}  now   (band: \pm 2 \surd P_{44} of now)'}, ...
               'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
        if a == 1; ylabel('(\^a_z - a_z) / a_{nom}', 'FontSize', FS, 'FontWeight','bold'); end
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off; xlim([0 T_END]); ylim([-0.03 0.03]);
        xline(TH, '--', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.0, 'HandleVisibility', 'off');
        nexttile(a + 4); hold on; yline(0, '-', 'Color', [0.55 0.55 0.55], 'HandleVisibility', 'off');
        hf = plot(t, (F.hd - F.HB) * R_um, '-', 'Color', COL_FIRST, 'LineWidth', 1.2);
        hn = plot(t, (N.hd - N.HB) * R_um, '-', 'Color', COL_NOW, 'LineWidth', 1.0);
        legend([hf hn], {'R \delta w_3 [\mum]  first', 'R \delta w_3 [\mum]  now'}, ...
               'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
        if a == 1; ylabel('R \delta w_3  [\mum]', 'FontSize', FS, 'FontWeight','bold'); end
        xlabel(sprintf('time [s]   (%s; dashed: hold start)', NM{a}), 'FontSize', FS, 'FontWeight','bold');
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off; xlim([0 T_END]);
        xline(TH, '--', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.0, 'HandleVisibility', 'off');
    end
    png = fullfile(od, 'btrue_first_vs_now_seed7.png'); exportgraphics(f, png, 'Resolution', 150); close(f); fprintf('wrote %s\n', png);
end
