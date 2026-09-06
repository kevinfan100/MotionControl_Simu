% PURPOSE (2026-09-04): the b_true arm (slope read at a_hat, four blocks) shows a NEGATIVE gain-error dip in the fast
%   near-wall part of the descent (Meng 7-10 s mean -0.015 at 10 seeds, sigma_seed 0.03; canon 1.2-1.5 s -0.02) that the
%   a'_true@est arm does not. Two questions, one run (both arms, 30 seeds, standard length, seed-at-truth):
%     (1) is the dip a MEAN at all? mean +- SEM, median and skewness of E at the dip time (the amplifier
%         F_e(4,4) = 1 + 2 b (1 - a_hat)|dw| is multiplicative and can skew without shifting the median);
%     (2) if it is, which leg carries it? per-step budget of d(a_hat) - d(a_true) from the driver logs:
%         predict = (a_hat[k] - K41 innov1[k] - K42 innov2[k]) - a_hat[k-1]  (law step incl. pred_mean2),
%         y1 leg = K41 innov1[k],  y2 leg = K42 innov2[k];  truth = a_true[k] - a_true[k-1];
%         cumulative seed means of (predict - truth), y1 leg, y2 leg add up to E(t) (checked).
%   Opponents written before the run: (a) y1 leg carries a non-zero-mean innovation in the descent (tracking-lag
%   rectification) amplified by the law; (b) a mean term missing from the predict; (c) pure skew, mean ~ 0.
%   Output: probe_btrue_descent_dip_<traj>.mat + probe_btrue_descent_dip_<traj>.png | EXPIRES: with the b_true rung
function out = probe_btrue_descent_dip(traj, seeds)
    if nargin < 1 || isempty(traj); traj = 'canon'; end
    if nargin < 2 || isempty(seeds); seeds = 1:30; end
    traj = lower(traj);
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'apd_acov_meng');  pc = physical_constants();
    switch traj
        case 'meng'; OV = struct('trajectory_type','osc','h_init',15,'h_bottom',2.5,'amplitude',0,'frequency',1,'n_cycles',1,'t_hold',0.5,'t_descend_override',10,'T_sim',12.5,'h_min',2.475); cfg0 = OV;  WIN = [7 10];
        case 'canon'; OV = struct(); cfg0 = canonical_scenario(0.05, 1.1, 'deep');  WIN = [1.0 1.5];
    end
    w0bar = cfg0.h_init / pc.R; [~, cp] = calc_correction_functions(w0bar); at = 1/cp; ws0 = 1 + w0bar - 1/((8/9)*(1 - at));
    t1 = cfg0.t_hold;  t2 = t1 + cfg0.t_descend_override;  t3 = t2 + cfg0.n_cycles/cfg0.frequency;
    ON4 = struct('law_exact_step',true,'pred_mean2',true,'nw_mcorr',true,'pred_mean2_e4',true,'ws0_perp',ws0);
    ON3 = ON4;  ON3.pred_mean2_e4 = false;  ON3.lock_b = true;
    DEF = struct('btest', struct('o', struct('b_true',true,'b_true_at','true'), 'cc', ON4), ...
                 'apest', struct('o', struct('ap_known',true,'ap_known_at','est','app_known',true), 'cc', ON3));
    ARM = {'btest','apest'};  nS = numel(seeds);  out = struct('traj', traj, 'seeds', seeds, 'phases', [t1 t2 t3 cfg0.T_sim], 'win', WIN);
    for ia = 1:2
        A = DEF.(ARM{ia});
        o = struct('arm','best','ctrl_const_override',A.cc,'config_override',OV,'scenario','deep','verbose',false,'seeds',seeds,'log_P_full',false);
        fn = fieldnames(A.o); for i = 1:numel(fn); o.(fn{i}) = A.o.(fn{i}); end
        clear run_formC_b motion_control_law_formC_b;
        evalc('R = run_formC_b(o);');
        t = R.runs{1}.tout(:);  N = numel(t);  E = zeros(N,nS); PRED = E; Y1 = E; Y2 = E; PM2 = E; TRU = E;
        for q = 1:nS
            r = R.runs{q}; ad = r.a_hat_out(1,3)/r.a_bar_hat_out(1,3);
            ah = r.a_bar_hat_out(:,3);  atr = r.a_true_out(:,3)/ad;  E(:,q) = ah - atr;
            y1 = r.K_a_y1_out(:,3) .* r.innov_y1_out(:,3);  y2 = r.K_a_y2_out(:,3) .* r.innov_y2_out(:,3);
            pred = [0; diff(ah)] - y1 - y2;                 % predict increment of step k = a_hat^-[k] - a_hat[k-1]
            Y1(:,q) = y1; Y2(:,q) = y2; PRED(:,q) = pred; TRU(:,q) = [0; diff(atr)]; PM2(:,q) = r.pred_mean2_out(:,3);
        end
        clear R;
        out.(ARM{ia}) = struct('t', t, 'E', E, 'pred', PRED, 'y1', Y1, 'y2', Y2, 'pm2', PM2, 'tru', TRU);
        % (1) is it a mean?
        mw = t >= WIN(1) & t <= WIN(2);  Ew = mean(E(mw,:), 1);                    % per-seed window mean
        [mn, im] = min(mean(E, 2));  Ed = E(im, :);                                  % at the dip of the seed mean
        fprintf('[%s %s] window %.1f-%.1f s: mean %+.5f (SEM %.5f, %.1f sigma) | median %+.5f | skew %+.2f | dip at t=%.2f: mean %+.5f (SEM %.5f) median %+.5f | hold mean %+.5f (SEM %.5f)\n', ...
            traj, ARM{ia}, WIN, mean(Ew), std(Ew)/sqrt(nS), mean(Ew)/(std(Ew)/sqrt(nS)), median(Ew), skewness(Ew), t(im), mn, std(Ed)/sqrt(nS), median(Ed), ...
            mean(E(t > t3,:),'all'), std(mean(E(t > t3,:),1))/sqrt(nS));
        % (2) budget: cumulative seed means over the run, and their increments over the window
        cP = cumsum(mean(PRED - TRU, 2)); cY1 = cumsum(mean(Y1, 2)); cY2 = cumsum(mean(Y2, 2)); cM = cumsum(mean(PM2, 2));
        chk = max(abs(cP + cY1 + cY2 - mean(E,2) + mean(E(1,:))));
        i0 = find(t >= WIN(1), 1); i1 = find(t >= WIN(2), 1);
        fprintf('[%s %s] budget over %.1f-%.1f s (seed mean, est-true): predict-truth %+.5f (of which pred_mean2 %+.5f) | y1 leg %+.5f | y2 leg %+.5f | sum %+.5f vs dE %+.5f | closure check %.1e\n', ...
            traj, ARM{ia}, WIN, cP(i1)-cP(i0), cM(i1)-cM(i0), cY1(i1)-cY1(i0), cY2(i1)-cY2(i0), (cP(i1)-cP(i0))+(cY1(i1)-cY1(i0))+(cY2(i1)-cY2(i0)), mean(E(i1,:))-mean(E(i0,:)), chk);
        % SEM of the window increments (per seed)
        dP = sum(PRED(i0:i1,:) - TRU(i0:i1,:), 1); dY1 = sum(Y1(i0:i1,:), 1); dY2 = sum(Y2(i0:i1,:), 1);
        fprintf('[%s %s] window increments per seed: predict-truth SEM %.5f | y1 SEM %.5f | y2 SEM %.5f\n', traj, ARM{ia}, std(dP)/sqrt(nS), std(dY1)/sqrt(nS), std(dY2)/sqrt(nS));
    end
    save(fullfile(od, sprintf('probe_btrue_descent_dip_%s.mat', traj)), '-struct', 'out', '-v7.3');
    % figure: E band (both arms) + cumulative budget (btest), one column
    f = figure('Units','inches','Position',[0 0 9 9], 'Color','w', 'Visible','off'); tiledlayout(2,1,'TileSpacing','compact','Padding','compact');
    COL = {[0.85 0.33 0.10], [0 0.2 0.9]};  LAB = {'b_{true}, four blocks (slope at \^a)', 'a''_{true} @ est, four blocks'};
    nexttile; hold on; yline(0,'-','Color',[0.5 0.5 0.5],'HandleVisibility','off'); H = [];
    for ia = 1:2
        d = out.(ARM{ia}); m = mean(d.E,2); s = std(d.E,0,2)/sqrt(nS);
        fill([d.t; flipud(d.t)], [m+2*s; flipud(m-2*s)], COL{ia}, 'FaceAlpha', 0.15, 'EdgeColor','none','HandleVisibility','off');
        H(ia) = plot(d.t, m, '-', 'Color', COL{ia}, 'LineWidth', 1.8);
    end
    xline(WIN(1), ':', 'Color', [0.3 0.3 0.3]); xline(WIN(2), ':', 'Color', [0.3 0.3 0.3]); xline(t3, '--', 'Color', [0.3 0.3 0.3]);
    legend(H, LAB, 'Location','northoutside','Orientation','horizontal','FontSize',10,'FontWeight','bold','Box','on');
    ylabel(sprintf('(\\^a_z - a_z)/a_{nom}   %d-seed mean \\pm 2 SEM', nS), 'FontSize', 12, 'FontWeight','bold');
    set(gca,'FontSize',13,'FontWeight','bold','LineWidth',1.5,'Box','on'); grid off; xlim([0 ceil(t(end))]);
    nexttile; hold on; yline(0,'-','Color',[0.5 0.5 0.5],'HandleVisibility','off');
    d = out.btest;  cP = cumsum(mean(d.pred - d.tru, 2)); cY1 = cumsum(mean(d.y1, 2)); cY2 = cumsum(mean(d.y2, 2)); cM = cumsum(mean(d.pm2, 2));
    h1 = plot(d.t, cP, '-', 'Color', [0.49 0.18 0.56], 'LineWidth', 1.8);
    h2 = plot(d.t, cY1, '-', 'Color', [0.47 0.67 0.19], 'LineWidth', 1.8);
    h3 = plot(d.t, cY2, '-', 'Color', [0.93 0.69 0.13], 'LineWidth', 1.8);
    h4 = plot(d.t, cM, ':', 'Color', [0.49 0.18 0.56], 'LineWidth', 1.4);
    h5 = plot(d.t, mean(d.E,2), '-', 'Color', COL{1}, 'LineWidth', 1.2);
    xline(WIN(1), ':', 'Color', [0.3 0.3 0.3]); xline(WIN(2), ':', 'Color', [0.3 0.3 0.3]); xline(t3, '--', 'Color', [0.3 0.3 0.3]);
    legend([h1 h2 h3 h4 h5], {'\Sigma (predict - truth)', '\Sigma K_{41} innov_1', '\Sigma K_{42} innov_2', '\Sigma pred\_mean2 (part of predict)', 'E = sum'}, ...
           'Location','northoutside','Orientation','horizontal','FontSize',9,'FontWeight','bold','Box','on');
    ylabel('cumulative seed-mean increment of (\^a_z - a_z)/a_{nom}', 'FontSize', 11, 'FontWeight','bold');
    xlabel(sprintf('time [s]   (%s; dotted: window, dashed: hold start)', traj), 'FontSize', 12, 'FontWeight','bold');
    set(gca,'FontSize',13,'FontWeight','bold','LineWidth',1.5,'Box','on'); grid off; xlim([0 ceil(t(end))]);
    png = fullfile(od, sprintf('probe_btrue_descent_dip_%s.png', traj)); exportgraphics(f, png, 'Resolution', 150); close(f); fprintf('wrote %s\n', png);
end
