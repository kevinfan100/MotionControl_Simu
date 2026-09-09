function out = plot_obs_b_5state_two_traj(seed, win)
%PLOT_OBS_B_5STATE_TWO_TRAJ  Observability of the ESTIMATED b, 5-state core, both trajectories.
%   PURPOSE (2026-09-09): the pair (F_e, H) of 0907_estb_5state_core.tex S4/S5 -- x = [dw1 dw2 dw3 a_w b], slot 5
%   FREE and estimated -- evaluated at the linearisation point of a REAL production run (arm 'best', chord seed
%   per stacked-fix-audit B.8), then per 500-step window:
%       O_w(k0,N) = R_w^{-1/2} O(k0,N),   O(k0,N) = [H[k] Phi(k,k0)]  stacked
%       O_w v_i = sigma_i u_i,  sigma_1 >= ... >= sigma_5 >= 0
%   WHY WHITEN FIRST. R1 = (sigma_n/R)^2 = 2.2e-6 and R2 ~ 1.7e-2 differ by ~8000, so the singular values of the
%   raw O are set by the y2 rows and say nothing about what the filter can resolve. After dividing each row by its
%   own noise std the singular values are in units of "how many sigma of imprint", 1/sigma_min is the std of the
%   worst-determined direction, and G = O_w' O_w, so CRLB_j = sqrt([G^-1]_jj) needs no new object.
%   Rows: (1) w_bar_d  (2) sigma_min/sigma_max, the structural check (a value like 1e-6 = the weakest direction is
%   an alias; it has NO threshold for grading -- measured on canon, windows that win and windows that lose overlap
%   between 1.2e-2 and 1.5e-2)  (3) b's error bar from that window alone, 1/sigma_min, against sqrt(P55[0]). Below the red
%   line = the data is sharper than the seed. Reading 1/sigma_min as b's own error bar is valid only while the
%   weakest combination IS b: |V_55| goes to the console (>= 0.97 on both trajectories). House style =
%   plot_ladder_tw_abs.m.
%   The 5-state core drops production's correlated-noise column and MA(2) slots, so shapes carry over, absolute
%   numbers do not: grade b on the production dump (verify_obs_b_of_w.m).
%   Output test_results/apd_acov_meng/obs_b_5state_two_traj.png
%   STATUS: ACTIVE | produces the last page of reference/eq17_analysis/derivation/0907_estb_5state_core.pdf
%   (figures/obs_b_5state_two_traj.png). Re-run and copy into derivation/figures/ after any change to formC_b's
%   row-4/H Jacobians or to the canon / Meng scenarios.
    if nargin < 1 || isempty(seed); seed = 7; end
    if nargin < 2 || isempty(win);  win  = 500; end
    here = fileparts(mfilename('fullpath')); root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(here);
    od = fullfile(root, 'test_results', 'apd_acov_meng'); pc = physical_constants();
    TR = {'canon', 'meng'};  NAME = {'canon deep', 'Meng ramp'};
    COL_TRUE = [0.8 0 0]; COL_HAT = [0 0.2 0.9];            % house colours (plot_ladder_tw_abs.m)
    FS = 15; LFS = 11; AXLW = 1.8; K_INIT = 4;              % log rows 1..3 are the init call
    COL_SEED = [0.55 0.74 0.96];
    fig = figure('Units','inches','Position',[0 0 10 12.6], 'Color','w', 'Visible','off');
    tiledlayout(3, 2, 'TileSpacing','compact', 'Padding','compact');
    out = struct();
    for c = 1:2
        [S, cfg, SEG] = local_run(TR{c}, seed, root);
        prior_b = S.ctrl_const.Pf_b_std;                      % sqrt(P55[0]), the width the data must beat
        R = pc.R; Ts = pc.Ts; lc = cfg.lambda_c; d = 2;
        wd  = S.h_bar_d_out(:);  ah = S.a_bar_hat_out(:, 3);  bh = S.b_hat_out(:, 3);
        dw3 = S.delta_x_hat_3_out(:, 3) / R;  fb = S.f_bar_out(:, 3);  R2 = S.R2_out(:, 3);
        n = numel(wd);  t = (0:n-1)' * Ts;  R1 = (cfg.meas_noise_std(3) / R)^2;
        dwd = [diff(wd); 0];  dwd(1:K_INIT) = 0;
        Fdw = fb + (1 - lc) * ([0; fb(1:end-1)] + [0; 0; fb(1:end-2)]);
        grad = [zeros(d, 1); wd(d+1:end) - wd(1:end-d)];  grad(1:K_INIT) = 0;
        starts = K_INIT:win:(n - win - 1);  nw = numel(starts);  tw = t(starts);
        smin = zeros(nw,1); smax = zeros(nw,1); rk = zeros(nw,1); vb = zeros(nw,1); cb = zeros(nw,1);
        for q = 1:nw
            k0 = starts(q);  Ow = zeros(2*(win+1), 5);  Phi = eye(5);
            for i = 0:win
                k = k0 + i;  M = dwd(k) + (1-lc)*dw3(k);  om = 1 - ah(k);
                F = [0 1 0 0 0; 0 0 1 0 0; 0 0 lc -Fdw(k) 0;
                     0 0 (1-lc)*bh(k)*om^2, 1 + bh(k)*om^2*Fdw(k) - 2*bh(k)*om*M, om^2*M;
                     0 0 0 0 1];
                H = [1 0 0 0 0; 0 0 0, 1 + 2*bh(k)*om*grad(k), -om^2*grad(k)];
                Ow(2*i + (1:2), :) = diag([1/sqrt(R1), 1/sqrt(max(R2(k), eps))]) * (H * Phi);
                Phi = F * Phi;
            end
            [~, Sv, V] = svd(Ow, 'econ');  s = diag(Sv);
            tol = max(size(Ow)) * eps(s(1));  rk(q) = sum(s > tol);
            smax(q) = s(1);  smin(q) = s(end);  vb(q) = abs(V(5, end));
            if rk(q) == 5; cb(q) = sqrt(sum((V(5,:).^2) ./ (s.^2)')); else; cb(q) = Inf; end
        end
        out.(TR{c}) = struct('t_win', tw, 'rank', rk, 'smin', smin, 'smax', smax, 'vb', vb, 'crlb_b', cb, 'seg', {SEG});
        tb = local_bounds(cfg);  FLOORV = 10^floor(log10(min(smin(smin > 0))) - 0.5);
        ax1 = nexttile(c); hold on;
        hw = plot(t(K_INIT:end), wd(K_INIT:end), '-', 'Color', COL_HAT, 'LineWidth', 2.0); yl = [0 1.05*max(wd)];
        for i = 1:numel(tb); plot([tb(i) tb(i)], yl, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, 'HandleVisibility','off'); end
        legend(hw, {NAME{c}}, 'Location','northoutside','Orientation','horizontal', ...
               'FontSize', LFS, 'FontWeight','bold', 'Box','on');
        if c == 1; ylabel('w_d / R', 'FontSize', FS, 'FontWeight','bold'); end
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off; xlim([0 t(end)]); ylim(yl);
        ax2 = nexttile(2 + c); hold on;                      % ratio: is the weakest direction structurally there
        hr = semilogy(tw, smin ./ smax, '-o', 'Color', COL_HAT, 'LineWidth', 1.8, 'MarkerSize', 5, 'MarkerFaceColor', COL_HAT);
        set(gca, 'YScale', 'log'); ylim([1e-7 1]); set(gca, 'YTick', 10.^(-7:0));
        for i = 1:numel(tb); plot([tb(i) tb(i)], [1e-7 1], '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, 'HandleVisibility','off'); end
        legend(hr, {'\sigma_{min} / \sigma_{max}'}, 'Location','northoutside','Orientation','horizontal', ...
               'FontSize',LFS,'FontWeight','bold','Box','on');
        if c == 1; ylabel('\sigma_{min} / \sigma_{max}   [-]', 'FontSize', FS, 'FontWeight','bold'); end
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off; xlim([0 t(end)]);
        ax3 = nexttile(4 + c); hold on;
        h1 = semilogy(tw, cb, '-o', 'Color', COL_HAT, 'LineWidth', 1.8, 'MarkerSize', 5, 'MarkerFaceColor', COL_HAT);
        set(gca, 'YScale', 'log');
        h0 = yline(prior_b, '--', 'Color', COL_TRUE, 'LineWidth', 1.4);
        hh = [h1 h0];  ll = {'1/\sigma_{min}', '\surd P_{55}[0]'};
        bad = rk < 5;
        if any(bad)
            h2 = plot(tw(bad), 1e3*ones(nnz(bad),1), 'x', 'Color', COL_TRUE, 'MarkerSize', 10, 'LineWidth', 1.6);
            hh = [hh h2]; ll = [ll, {'rank < 5'}];
        end
        ylim([1e-3 1e3]); set(gca, 'YTick', [1e-3 1e-2 1e-1 1 1e1 1e2 1e3]);
        for i = 1:numel(tb); plot([tb(i) tb(i)], [1e-3 1e3], '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, 'HandleVisibility','off'); end
        legend(hh, ll, 'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
        if c == 1; ylabel('b uncertainty   [-]', 'FontSize', FS, 'FontWeight','bold'); end
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off; xlim([0 t(end)]);
        xlabel(sprintf('time [s]   window start k_0, N = %d steps', win), 'FontSize', FS, 'FontWeight','bold');
        linkaxes([ax1 ax2 ax3], 'x');
        fprintf('\n[%s] seed %d | window %d steps = %.2f s | b_hat %.4f -> %.4f | sqrt(P55[0]) %.4f | 1/sqrt(P55[0]) %.1f\n', ...
                TR{c}, seed, win, win*Ts, bh(K_INIT), bh(end), prior_b, 1/prior_b);
        fprintf('   %6s %-9s %5s %11s %11s %10s %11s %11s\n', 't0','segment','rank','sigma_max','sigma_min','|v_min,5|','CRLB_b','CRLB_b/prior');
        for q = 1:nw
            sg = 'n/a';
            for r2 = 1:size(SEG,1); if tw(q) >= SEG{r2,2} && tw(q) < SEG{r2,3}; sg = SEG{r2,1}; end; end
            fprintf('   %6.2f %-9s %5d %11.2e %11.2e %10.3f %11.2e %11.2f\n', tw(q), sg, rk(q), smax(q), smin(q), vb(q), cb(q), cb(q)/prior_b);
        end
    end
    if ~exist(od, 'dir'); mkdir(od); end
    exportgraphics(fig, fullfile(od, 'obs_b_5state_two_traj.png'), 'Resolution', 150);
    fprintf('\nsaved %s\n', fullfile(od, 'obs_b_5state_two_traj.png'));
end

function [S, cfg, SEG] = local_run(traj, seed, root)
    pc = physical_constants();
    switch traj
        case 'meng'
            OV = struct('trajectory_type','osc','h_init',15,'h_bottom',2.5,'amplitude',0,'frequency',1, ...
                        'n_cycles',1,'t_hold',0.5,'t_descend_override',10,'T_sim',12.5,'h_min',2.475);
            cfg = canonical_scenario(0.05, 1.1, 'deep');
            f = fieldnames(OV); for i = 1:numel(f); cfg.(f{i}) = OV.(f{i}); end
            SEG = {'hold0',0,0.5; 'far',0.5,7; 'near',7,10.5; 'hold',10.5,12.5};  p0 = 3e-4;
        otherwise
            OV = struct(); cfg = canonical_scenario(0.05, 1.1, 'deep');
            SEG = {'hold0',0,0.5; 'descent',0.5,1.5; 'osc',1.5,3.5; 'hold',3.5,4.8};  p0 = 1e-5;
    end
    w0 = cfg.h_init / pc.R;  [~, cp] = calc_correction_functions(w0, true);  a0 = 1/cp;
    b_ch = (1/(1 - a0) - 1) / (w0 - 1.0);  ws_ch = 1 + 1.0 - 1/b_ch;
    cc = struct('b_ceil',1.5,'Pf_w0_std',0,'Pf_a_floor',p0,'b_init',b_ch,'ws0_perp',ws_ch);
    o  = struct('arm','best','ctrl_const_override',cc,'config_override',OV,'scenario','deep', ...
                'verbose',false,'seeds',seed,'log_P_full',false);
    clear run_formC_b motion_control_law_formC_b;
    evalc('Rr = run_formC_b(o);');
    S = Rr.runs{1};
end

function tb = local_bounds(cfg)
    t1 = cfg.t_hold; t2 = t1 + cfg.t_descend_override;
    if cfg.amplitude > 0; tb = [t1 t2 t2 + cfg.n_cycles / cfg.frequency]; else; tb = [t1 t2]; end
end
