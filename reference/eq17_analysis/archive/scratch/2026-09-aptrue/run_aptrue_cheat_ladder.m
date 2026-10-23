% STATUS: ACTIVE (scratch) | PURPOSE: the four-arm "cheat ladder" of the a'_true
%   oracle on the canonical deep trajectory, same seeds in every arm. Each arm
%   removes ONE piece of the second-order decomposition of
%   derivation/0902_formC_aptrue_4state.tex (probe: analyze_aptrue_predict_drift.m),
%   so the jump between consecutive curves IS that piece:
%     1 Euler        tangent predict                         (baseline)
%     2 exact        law_exact_step                          removes the omitted curvature on the estimator's own steps
%     3 exact+est    slope read at the ESTIMATED height      removes D1 (deterministic slope-evaluation term)
%     4 exact+est+m2 + pred_mean2 with exogenous a''_true    removes D2 (Jensen) + D1 stochastic part + curvature gap
%   PRE-REGISTERED hold means (est - true), 2026-09-02:
%     1: +0.0053 (measured)   2: -0.0022 (measured)   3: ~ +0.006   4: ~ +0.0005 (D3 left)
%   Stop line: any arm off its prediction by more than 0.001.
%   Output: test_results/apd_acov_meng/aptrue_cheat_ladder.{mat,png}
function out = run_aptrue_cheat_ladder(seeds)
    if nargin < 1 || isempty(seeds); seeds = 1:8; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    pc = physical_constants();  cfg0 = canonical_scenario(0.05, 1.1, 'deep');
    w0bar = cfg0.h_init / pc.R;  [~, cp] = calc_correction_functions(w0bar);  at = 1/cp;
    ws0 = 1 + w0bar - 1/((8/9)*(1 - at));
    base = struct('arm','best','ap_known',true,'ap_known_at','cmd','app_known',false, ...
                  'ctrl_const_override',struct('lock_b',true,'ws0_perp',ws0), ...
                  'config_override',struct(),'scenario','deep','verbose',false,'seeds',seeds,'log_P_full',false);
    ARMS = {'1 Euler', '2 exact', '3 exact + slope@est', '4 exact + slope@est + pred\_mean2'};
    O = cell(1,4);
    O{1} = base;
    O{2} = base;  O{2}.ctrl_const_override.law_exact_step = true;
    O{3} = O{2};  O{3}.ap_known_at = 'est';
    O{4} = O{3};  O{4}.ctrl_const_override.pred_mean2 = true;  O{4}.app_known = true;
    PRED = [+0.0053, -0.0022, +0.006, +0.0005];
    nS = numel(seeds);  E = cell(1,4);  M2 = cell(1,4);  t = [];
    for a = 1:4
        clear run_formC_b motion_control_law_formC_b;
        evalc('R = run_formC_b(O{a});');
        t = R.runs{1}.tout(:);  N = numel(t);  E{a} = zeros(N, nS);  M2{a} = zeros(N, nS);
        for q = 1:nS
            r = R.runs{q};  ad = r.a_hat_out(1,3)/r.a_bar_hat_out(1,3);
            E{a}(:,q) = r.a_bar_hat_out(:,3) - r.a_true_out(:,3)/ad;
            M2{a}(:,q) = r.pred_mean2_out(:,3);
        end
        fprintf('[ladder] %-36s health: min a_bar_hat %.4f | NaN %d | E(0) %+.1e | mean2 sum/seed %+.5f\n', ...
            ARMS{a}, min(min(cellfun(@(x) min(x.a_bar_hat_out(:,3)), R.runs))), sum(~isfinite(E{a}(:))), mean(E{a}(1,:)), mean(sum(M2{a},1)));
        clear R;
    end
    m_h = t > 3.5;  m_o = t > 1.5 & t <= 3.5;
    fprintf('\n%-38s %10s %10s %10s %12s\n', 'arm', 'osc', 'hold', 'SEM(hold)', 'pre-reg hold');
    for a = 1:4
        pm = mean(E{a}(m_h,:), 1);
        fprintf('%-38s %+10.5f %+10.5f %10.5f %+12.4f\n', ARMS{a}, mean(mean(E{a}(m_o,:),1)), mean(pm), std(pm)/sqrt(nS), PRED(a));
    end
    for a = 2:4
        d = mean(E{a}(m_h,:),1) - mean(E{a-1}(m_h,:),1);
        fprintf('  jump %d -> %d (paired, hold): %+.5f  (SEM %.5f)\n', a-1, a, mean(d), std(d)/sqrt(nS));
    end
    % --- figure: four seed-mean curves ---
    COL = {[0.55 0.74 0.96], [0 0.2 0.9], [0.85 0.33 0.10], [0.47 0.67 0.19]};  FS = 15;
    f = figure('Units','inches','Position',[0 0 12 6], 'Color','w', 'Visible','off');
    hold on; yline(0,'-','Color',[0.55 0.55 0.55],'HandleVisibility','off');
    H = gobjects(1,4);
    for a = 1:4; H(a) = plot(t, mean(E{a},2), '-', 'Color', COL{a}, 'LineWidth', 1.6); end
    legend(H, ARMS, 'Location','northoutside','Orientation','horizontal','FontSize',11,'FontWeight','bold','Box','on');
    ylabel('mean (\^a_z - a_z) / a_{nom}', 'FontSize', FS, 'FontWeight','bold');
    xlabel('time  [s]', 'FontSize', FS, 'FontWeight','bold'); xlim([0 5]);
    set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',1.6,'Box','on'); grid off;
    png = fullfile(od, 'aptrue_cheat_ladder.png');
    exportgraphics(f, png, 'Resolution', 150); close(f);
    fprintf('[ladder] wrote %s\n', png);
    out = struct('t', t, 'E', {E}, 'M2', {M2}, 'arms', {ARMS}, 'pred', PRED, 'seeds', seeds);
    save(fullfile(od, 'aptrue_cheat_ladder.mat'), '-struct', 'out');
end
