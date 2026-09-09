function plot_obs_sigma_two_traj()
%PLOT_OBS_SIGMA_TWO_TRAJ  Observability of b on the two trajectories, singular values only (0907 S7 presentation).
%   PURPOSE (2026-09-09): read verify_obs_b_of_w_{canon,meng}_w500.mat (production 9-state dump, 09-08, chord-seed
%   arm, window 500) and show per window sigma_min/sigma_max of O for the constant-b free set [1 2 3 4 5 8 9] and
%   for the linear-b set [.. 10] (the alias example). Top row = the commanded height, bottom row = the ratio on a
%   log axis; phase boundaries dashed. Output test_results/apd_acov_meng/obs_sigma_two_traj.png
%   EXPIRES: presentation figure, with the 0907 reading copy | 產線改動不會自動跟上
    here = fileparts(mfilename('fullpath')); root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    T = {'canon', 'meng'};  ttl = {'canon deep', 'Meng ramp'};
    fig = figure('Position', [80 80 1400 760], 'Color', 'w');
    for c = 1:2
        S = load(fullfile(od, sprintf('verify_obs_b_of_w_%s_w500.mat', T{c})));
        cfg = canonical_scenario(0.05, 1.1, 'deep');
        if c == 2
            OV = struct('h_init',15,'h_bottom',2.5,'amplitude',0,'frequency',1,'n_cycles',1,'t_hold',0.5,'t_descend_override',10,'T_sim',12.5,'h_min',2.475);
            f = fieldnames(OV); for i = 1:numel(f); cfg.(f{i}) = OV.(f{i}); end
        end
        params = calc_simulation_params(cfg);  P = params.Value;  Ts = P.common.Ts;  R = P.common.R;
        N = round(cfg.T_sim / Ts) + 1;  w_hat = P.wall.w_hat(:);  pz = P.wall.pz;
        clear trajectory_generator;  t = (0:N-1)' * Ts;  wd = zeros(N, 1);  pd_k = P.common.p0;
        for k = 1:N; [pd_kp1, ~] = trajectory_generator(t(k), P); wd(k) = (dot(pd_k, w_hat) - pz) / R; pd_k = pd_kp1; end
        clear trajectory_generator;
        tb = [cfg.t_hold, cfg.t_hold + cfg.t_descend_override, cfg.t_hold + cfg.t_descend_override + cfg.n_cycles / cfg.frequency * (cfg.amplitude > 0)];
        tb = unique(tb);
        ax1 = subplot(2, 2, c);
        plot(t, wd, 'b-', 'LineWidth', 1.8); hold on; yl = ylim;
        for i = 1:numel(tb); plot([tb(i) tb(i)], yl, 'k--', 'LineWidth', 0.8); end
        ylabel('$\bar{w}_d$', 'Interpreter', 'latex'); box on; set(gca, 'FontSize', 13); xlim([0 cfg.T_sim]);
        legend({['$\bar{w}_d$ (' ttl{c} ')'], 'phase boundary'}, 'Interpreter', 'latex', 'Location', 'northoutside', 'Orientation', 'horizontal');
        ax2 = subplot(2, 2, 2 + c);
        rc = S.const; rl = S.lin;                                   % rank / srat are windows x 3 (both channels, y1 only, y2 only)
        tc = rc.t_win(:); sc = rc.srat(:, 1); kc = rc.rank(:, 1);
        tl = rl.t_win(:); sl = rl.srat(:, 1); kl = rl.rank(:, 1);
        floor_v = 1e-8;
        h1 = semilogy(tc, max(sc, floor_v), 'b-o', 'LineWidth', 1.8, 'MarkerSize', 5, 'MarkerFaceColor', 'b'); hold on;
        h2 = semilogy(tl, max(sl, floor_v), '-s', 'Color', [0.45 0.70 1.0], 'LineWidth', 1.6, 'MarkerSize', 5);
        dropc = kc < numel(rc.free); dropl = kl < numel(rl.free);
        h3 = semilogy([tc(dropc); tl(dropl)], floor_v * ones(nnz(dropc) + nnz(dropl), 1), 'rx', 'MarkerSize', 9, 'LineWidth', 1.5);
        yl = [floor_v 1]; ylim(yl);
        for i = 1:numel(tb); plot([tb(i) tb(i)], yl, 'k--', 'LineWidth', 0.8); end
        ylabel('$\sigma_{\min}/\sigma_{\max}$ of $O(k_0,N)$', 'Interpreter', 'latex'); xlabel('time [s]  (window start $k_0$, $N$ = 500)', 'Interpreter', 'latex');
        box on; set(gca, 'FontSize', 13); xlim([0 cfg.T_sim]);
        legend([h1 h2 h3], {'b constant, free = [1 2 3 4 5 8 9]', 'b = b$_0$ + b$_1$($\bar{w}$ $-$ 2.2), slot 10 added', 'rank $<$ $|$free$|$'}, ...
               'Interpreter', 'latex', 'Location', 'northoutside', 'Orientation', 'horizontal');
        linkaxes([ax1 ax2], 'x');
        % console: per-segment minimum of the ratio and the rank range
        switch T{c}; case 'canon'; SEG = {'hold0',0,0.5;'descent',0.5,1.5;'osc',1.5,3.5;'hold',3.5,4.8}; otherwise; SEG = {'hold0',0,0.5;'far',0.5,7;'near',7,10.5;'hold',10.5,12.5}; end
        fprintf('[%s] neg control ok: const %d, lin %d\n', T{c}, rc.neg_ok, rl.neg_ok);
        fprintf('   %-8s | const: rank    srat_min   | lin: rank    srat_min\n', 'segment');
        for q = 1:size(SEG, 1)
            mc = tc >= SEG{q,2} & tc < SEG{q,3}; ml = tl >= SEG{q,2} & tl < SEG{q,3};
            fprintf('   %-8s | %d..%d    %8.1e   | %d..%d    %8.1e\n', SEG{q,1}, min(kc(mc)), max(kc(mc)), min(sc(mc)), min(kl(ml)), max(kl(ml)), min(sl(ml)));
        end
    end
    exportgraphics(fig, fullfile(od, 'obs_sigma_two_traj.png'), 'Resolution', 150);
    fprintf('saved %s\n', fullfile(od, 'obs_sigma_two_traj.png'));
end
