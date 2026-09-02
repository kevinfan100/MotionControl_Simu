% STATUS: ACTIVE (scratch) | PURPOSE: paired-seed check of the claim
%   "canon Euler (+0.0053) = canon exact (-0.0022) + the curvature term Euler
%   omits on the estimator's OWN steps". From the exact run's per-step logs
%   (probe_aptrue_predict_drift.m, obs_dump) the omitted increment is computed
%   EXACTLY (no Taylor):
%       d[k] = a' B / (1 + a' B/(1-a)) - a' B  = - a'^2 B^2 / ((1-a)(1 + a' B/(1-a)))
%   (exact step minus tangent step, same a', same B_hat, same a_hat), so
%   Euler - exact (est - true) ~ -sum d  open loop. The same seeds are then run
%   with the tangent step and the measured paired difference is overlaid.
%   Output: test_results/apd_acov_meng/seedtruth_euler_vs_exact_ratchet.png
function out = plot_seedtruth_ratchet_check()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    D = load(fullfile(od, 'aptrue_predict_drift_probe.mat'));  S = D.S;  ws0 = D.ws0;
    lc = 0.7;  al = 1 - lc;  nS = numel(S);  seeds = [S.seed];
    % --- exact arm (from probe) : E(t) per seed and the exactly-omitted increment ---
    N = numel(S(1).t);  Eex = zeros(N, nS);  Dpred = zeros(N, nS);
    for q = 1:nS
        s = S(q);  n = size(s.x_upd,1);  j = (2:n).';  kk = j + 1;
        Eex(2:end, q) = s.x_upd(:,4) - s.a_true(2:end);              % est - true, rows k = 2..N
        hd = s.hd;  dwd = hd(kk) - hd(kk-1);
        Bh = dwd + al*s.x_upd(j-1,3) + al*(s.x_upd(j-1,8) + s.x_upd(j-1,9));
        a1 = s.a_prime(kk);  x4 = s.x_upd(j-1,4);
        d  = a1.*Bh./(1 + a1.*Bh./(1-x4)) - a1.*Bh;                  % exact - tangent, per step (<= 0)
        Dpred(kk, q) = cumsum(-d);                                   % Euler - exact, open loop (>= 0)
    end
    % --- Euler arm, same seeds, same everything except law_exact_step ---
    clear run_formC_b motion_control_law_formC_b;
    o = struct('arm','best','ap_known',true,'ap_known_at','cmd', ...
               'ctrl_const_override',struct('lock_b',true,'ws0_perp',ws0), ...
               'config_override',struct(),'scenario','deep','verbose',false,'seeds',seeds,'log_P_full',false);
    evalc('R = run_formC_b(o);');
    t = R.runs{1}.tout(:);  Eeu = zeros(N, nS);
    for q = 1:nS
        r = R.runs{q};  ad = r.a_hat_out(1,3)/r.a_bar_hat_out(1,3);
        Eeu(:,q) = r.a_bar_hat_out(:,3) - r.a_true_out(:,3)/ad;
    end
    mEu = mean(Eeu,2);  mEx = mean(Eex,2);  mD = mean(Dpred,2);  dMeas = mean(Eeu - Eex, 2);
    m = t > 3.5;
    fprintf('[ratchet check] %d paired seeds, canon deep, a''_true@cmd\n', nS);
    fprintf('  hold mean  Euler %+.5f | exact %+.5f | Euler-exact %+.5f | open-loop prediction %+.5f | ratio %.2f\n', ...
        mean(mEu(m)), mean(mEx(m)), mean(dMeas(m)), mean(mD(m)), mean(dMeas(m))/mean(mD(m)));
    mo = t > 1.5 & t <= 3.5;
    fprintf('  osc  mean  Euler %+.5f | exact %+.5f | Euler-exact %+.5f | open-loop prediction %+.5f | ratio %.2f\n', ...
        mean(mEu(mo)), mean(mEx(mo)), mean(dMeas(mo)), mean(mD(mo)), mean(dMeas(mo))/mean(mD(mo)));
    fprintf('  end        Euler %+.5f | exact %+.5f | Euler-exact %+.5f | open-loop prediction %+.5f\n', mEu(end), mEx(end), dMeas(end), mD(end));
    % split of the predicted term: command-step part vs jitter part (hold has no command step)
    fprintf('  predicted omitted term accumulated: by end of osc %+.5f | hold only %+.5f (jitter steps)\n', mD(find(t<=3.5,1,'last')), mD(end) - mD(find(t<=3.5,1,'last')));
    % --- figure (house style) ---
    C_EU = [0.55 0.74 0.96]; C_EX = [0 0.2 0.9]; C_P = [0.8 0 0]; FS = 15; LW = 2;
    f = figure('Units','inches','Position',[0 0 12 8], 'Color','w', 'Visible','off');
    tiledlayout(2,1,'TileSpacing','compact','Padding','compact');
    nexttile; hold on; yline(0,'-','Color',[0.55 0.55 0.55],'HandleVisibility','off');
    h1 = plot(t, mEu, '-', 'Color', C_EU, 'LineWidth', LW);
    h2 = plot(t, mEx, '-', 'Color', C_EX, 'LineWidth', LW);
    h3 = plot(t, mEx + mD, '--', 'Color', C_P, 'LineWidth', LW);
    legend([h1 h2 h3], {'Euler (tangent step)', 'exact step', 'exact + omitted curvature term (open loop, 0 param)'}, ...
           'Location','northoutside','Orientation','horizontal','FontSize',11,'FontWeight','bold','Box','on');
    ylabel('mean (\^a_z - a_z) / a_{nom}', 'FontSize', FS, 'FontWeight','bold'); xlim([0 5]);
    set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',1.6,'Box','on'); grid off;
    nexttile; hold on; yline(0,'-','Color',[0.55 0.55 0.55],'HandleVisibility','off');
    h4 = plot(t, dMeas, '-', 'Color', C_EX, 'LineWidth', LW);
    h5 = plot(t, mD, '--', 'Color', C_P, 'LineWidth', LW);
    legend([h4 h5], {sprintf('measured Euler - exact, same %d seeds', nS), '-\Sigma [exact step - tangent step]  from exact-run logs'}, ...
           'Location','northoutside','Orientation','horizontal','FontSize',11,'FontWeight','bold','Box','on');
    ylabel('difference / a_{nom}', 'FontSize', FS, 'FontWeight','bold'); xlabel('time  [s]', 'FontSize', FS, 'FontWeight','bold'); xlim([0 5]);
    set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',1.6,'Box','on'); grid off;
    png = fullfile(od, 'seedtruth_euler_vs_exact_ratchet.png');
    exportgraphics(f, png, 'Resolution', 150); close(f);
    fprintf('  wrote %s\n', png);
    out = struct('t', t, 'Eeu', Eeu, 'Eex', Eex, 'Dpred', Dpred, 'png', png);
    save(fullfile(od, 'seedtruth_euler_vs_exact_ratchet.mat'), '-struct', 'out');
end
