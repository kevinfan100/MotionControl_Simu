function plot_obs_b_walk_one_window(seed, win)
%PLOT_OBS_B_WALK_ONE_WINDOW  What one column of O means, walked step by step (0907_estb_5state_core.tex S7).
%   PURPOSE (2026-09-09, explanation-style rule 9): take ONE error, e_b = one prior sigma, put it in at k0 with
%   every other error zero, and walk the 5-state core (F_e, H of 0907 at a real production linearisation point):
%     row 1  how far y2 is thrown off at each step, in units of that step's own noise sigma
%     row 2  the running total sqrt(sum of squares) -- the end value is how many sigma the window sees in all
%   Two windows of canon deep side by side: the descent (b is seen) and the final hold (b is not).
%   b resolvable from the window = e_b / total. Output test_results/apd_acov_meng/obs_b_walk_one_window.png
%   EXPIRES: teaching figure, with the 0907 reading copy | 產線改動不會自動跟上
    if nargin < 1 || isempty(seed); seed = 7; end
    if nargin < 2 || isempty(win);  win  = 500; end
    here = fileparts(mfilename('fullpath')); root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'apd_acov_meng'); pc = physical_constants();
    COL_TRUE = [0.8 0 0]; COL_HAT = [0 0.2 0.9]; COL_SEED = [0.55 0.74 0.96];
    FS = 15; LFS = 11; AXLW = 1.8; K_INIT = 4;
    cfg = canonical_scenario(0.05, 1.1, 'deep');
    w0 = cfg.h_init / pc.R; [~, cp] = calc_correction_functions(w0, true); a0 = 1/cp;
    b_ch = (1/(1 - a0) - 1) / (w0 - 1.0); ws_ch = 1 + 1.0 - 1/b_ch;
    cc = struct('b_ceil',1.5,'Pf_w0_std',0,'Pf_a_floor',1e-5,'b_init',b_ch,'ws0_perp',ws_ch);
    o  = struct('arm','best','ctrl_const_override',cc,'scenario','deep','verbose',false,'seeds',seed);
    clear run_formC_b motion_control_law_formC_b; evalc('Rr = run_formC_b(o);'); S = Rr.runs{1};
    R = pc.R; Ts = pc.Ts; lc = cfg.lambda_c; d = 2;  prior_b = S.ctrl_const.Pf_b_std;
    wd = S.h_bar_d_out(:); ah = S.a_bar_hat_out(:,3); bh = S.b_hat_out(:,3);
    dw3 = S.delta_x_hat_3_out(:,3)/R; fb = S.f_bar_out(:,3); R2 = S.R2_out(:,3);
    n = numel(wd); t = (0:n-1)'*Ts; R1 = (cfg.meas_noise_std(3)/R)^2;
    dwd = [diff(wd); 0]; dwd(1:K_INIT) = 0;
    Fdw = fb + (1-lc)*([0; fb(1:end-1)] + [0; 0; fb(1:end-2)]);
    grad = [zeros(d,1); wd(d+1:end)-wd(1:end-d)]; grad(1:K_INIT) = 0;
    W = {'descent', 1.25; 'final hold', 4.06};
    fig = figure('Units','inches','Position',[0 0 13 7.8], 'Color','w', 'Visible','off');
    tiledlayout(2, 2, 'TileSpacing','compact', 'Padding','compact');
    for c = 1:2
        k0 = find(t >= W{c,2}, 1);  Ow = zeros(2*(win+1), 5); Phi = eye(5); tk = zeros(win+1,1);
        for i = 0:win
            k = k0+i; M = dwd(k) + (1-lc)*dw3(k); om = 1-ah(k);
            F = [0 1 0 0 0; 0 0 1 0 0; 0 0 lc -Fdw(k) 0;
                 0 0 (1-lc)*bh(k)*om^2, 1+bh(k)*om^2*Fdw(k)-2*bh(k)*om*M, om^2*M; 0 0 0 0 1];
            H = [1 0 0 0 0; 0 0 0, 1+2*bh(k)*om*grad(k), -om^2*grad(k)];
            Ow(2*i+(1:2),:) = diag([1/sqrt(R1), 1/sqrt(max(R2(k),eps))]) * (H*Phi);
            Phi = F*Phi; tk(i+1) = t(k);
        end
        col_b = Ow(:,5) * prior_b;                       % y deviation caused by e_b = one prior sigma [in noise sigma]
        dev_y1 = col_b(1:2:end); dev_y2 = col_b(2:2:end);
        run_tot = sqrt(cumsum(col_b.^2));  run_tot = run_tot(2:2:end);
        tot_known = norm(Ow(:,5));                        % others known
        s = svd(Ow);  sig_min = s(end);                   % others unknown
        fprintf('[%s] k0 t = %.2f s | e_b = %.4f (one prior sigma)\n', W{c,1}, t(k0), prior_b);
        fprintf('   total imprint over the window: %.1f sigma (y1 %.1f, y2 %.1f)\n', norm(col_b), norm(dev_y1), norm(dev_y2));
        fprintf('   b resolvable, others KNOWN   : %.4f  (= 1/%.1f)\n', 1/tot_known, tot_known);
        fprintf('   b resolvable, others UNKNOWN : %.4f  (= 1/%.1f = 1/sigma_min)\n', 1/sig_min, sig_min);
        fprintf('   eaten by the other four states: %.1fx\n\n', tot_known/sig_min);
        ax1 = nexttile(c); hold on;
        h2 = plot(tk, dev_y2, '-', 'Color', COL_HAT, 'LineWidth', 1.6);
        h1 = plot(tk, dev_y1, '-', 'Color', COL_SEED, 'LineWidth', 1.2);
        yline(0, '-', 'Color', [0.5 0.5 0.5], 'LineWidth', 0.8, 'HandleVisibility','off');
        legend([h2 h1], {'y_2', 'y_1'}, 'Location','northoutside','Orientation','horizontal', ...
               'FontSize',LFS,'FontWeight','bold','Box','on');
        if c == 1; ylabel('offset per step   [noise \sigma]', 'FontSize', FS, 'FontWeight','bold'); end
        title(''); set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off;
        xlim([tk(1) tk(end)]); ylim([-10 10]);                 % shared across the two windows
        ax2 = nexttile(2+c); hold on;
        hr = plot(tk, run_tot, '-', 'Color', COL_HAT, 'LineWidth', 2.0);
        hk = yline(tot_known*prior_b, '--', 'Color', [0.35 0.35 0.35], 'LineWidth', 1.2);
        hu = yline(sig_min*prior_b, '--', 'Color', COL_TRUE, 'LineWidth', 1.4);
        legend([hr hk hu], {'running total', 'total', 'left after the other four'}, 'Location','northoutside', ...
               'Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
        if c == 1; ylabel('total offset   [noise \sigma]', 'FontSize', FS, 'FontWeight','bold'); end
        xlabel(sprintf('time [s]   %s window, N = %d steps', W{c,1}, win), 'FontSize', FS, 'FontWeight','bold');
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off; xlim([tk(1) tk(end)]);
        set(gca, 'YScale', 'log'); ylim([1e-3 1e3]);
    end
    if ~exist(od,'dir'); mkdir(od); end
    exportgraphics(fig, fullfile(od, 'obs_b_walk_one_window.png'), 'Resolution', 150);
    fprintf('saved %s\n', fullfile(od, 'obs_b_walk_one_window.png'));
end
