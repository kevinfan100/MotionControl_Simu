function results = verify_eq17_6state_h50(opts)
%VERIFY_EQ17_6STATE_H50  End-to-end h=50 positioning verification for the
%   RevisedControl_Vpersonal 6-state controller (pure-MATLAB).
%
%   results = verify_eq17_6state_h50()
%   results = verify_eq17_6state_h50(opts)
%
%   Mirrors verify_eq17_v2_h50_e2e (7-state). Runs N seeds of h=50
%   positioning with config.eq17_variant='6state', aggregates per-axis
%   metrics, and compares the gain estimate a_hat against the physics
%   ground-truth a_x_true (computed from the noise-free position probe
%   simOut.p_true_out -- the pure-MATLAB advantage over Simulink).
%
%   PASS criteria (per axis):
%       tracking std (p_d - p_m)        < opts.trk_thresh_nm   (default 40 nm)
%       |a_hat bias vs a_x_true|        < opts.bias_thresh_pct (default 5 %)
%       a_hat rel-std                   < opts.relstd_thresh_pct (default 5 %)
%       |delta_x_D^d| (disturbance est) < opts.xD_thresh_um    (default 0.01 um)
%
%   ----- opts (all optional) -----
%       seeds (default 1:5), T_sim (5), h_init (50), t_warmup (0.5),
%       save_fig (true), verbose (true), thresholds above.
%
%   Outputs results struct + writes test_results/eq17_6state_h50/
%   {summary.md, summary.mat, h50_1..4_*.png}. (test_results/ is gitignored.)
%
%   See also: motion_control_law_eq17_6state, run_pure_simulation

    % --- 0. Defaults ---
    if nargin < 1 || isempty(opts); opts = struct(); end
    dflt = struct('seeds', 1:5, 'T_sim', 5, 'h_init', 50, 't_warmup', 0.5, ...
                  'save_fig', true, 'verbose', true, ...
                  'trk_thresh_nm', 40, 'bias_thresh_pct', 5, ...
                  'relstd_thresh_pct', 5, 'xD_thresh_um', 0.01);
    fn = fieldnames(dflt);
    for i = 1:numel(fn)
        if ~isfield(opts, fn{i}) || isempty(opts.(fn{i})); opts.(fn{i}) = dflt.(fn{i}); end
    end

    % --- 1. Paths (script in test_script/integration/ -> up two levels) ---
    [script_dir, ~, ~] = fileparts(mfilename('fullpath'));
    project_root = fileparts(fileparts(script_dir));
    addpath(fullfile(project_root, 'model'), fullfile(project_root, 'model', 'config'), ...
            fullfile(project_root, 'model', 'wall_effect'), ...
            fullfile(project_root, 'model', 'thermal_force'), ...
            fullfile(project_root, 'model', 'trajectory'), ...
            fullfile(project_root, 'model', 'controller'), ...
            fullfile(project_root, 'model', 'dual_track'));

    % --- 2. Config (h=50 positioning, 6-state variant) ---
    config = user_config();
    config.h_init          = opts.h_init;
    config.h_bottom        = opts.h_init;
    config.amplitude       = 0;
    config.trajectory_type = 'positioning';
    config.T_sim           = opts.T_sim;
    config.ctrl_enable     = true;
    config.meas_noise_enable = true;
    config.thermal_enable  = true;
    config.lambda_c        = 0.7;
    config.a_pd            = 0.05;
    config.a_cov           = 0.05;
    config.meas_noise_std  = [0.00062; 0.000057; 0.00331];
    config.eq17_variant    = '6state';        % <-- dispatch A1

    % --- 3. Params + wall geometry ---
    params = calc_simulation_params(config);
    P = params.Value;
    Ts = P.common.Ts; gamma_N = P.common.gamma_N; R_radius = P.common.R;
    w_hat = P.wall.w_hat; pz_wall = P.wall.pz;
    a_nom = Ts / gamma_N;

    n_seeds = numel(opts.seeds);
    n_warmup = round(opts.t_warmup / Ts);

    trk_meas   = zeros(n_seeds, 3);   % std(p_d - p_m)  [nm], axes [x y z]
    trk_true   = zeros(n_seeds, 3);   % std(p_d - p_true)
    a_hat_mean = zeros(n_seeds, 3);   % [x y z] (remapped from ekf_out [x z y])
    a_hat_std  = zeros(n_seeds, 3);
    a_true_mean = zeros(n_seeds, 3);  % [x y z]
    xD_absmean = zeros(n_seeds, 3);   % mean |delta_x_D^d| [um]

    if opts.verbose
        fprintf('[verify_eq17_6state_h50] T_sim=%.1fs h_init=%g n_seeds=%d\n', ...
                opts.T_sim, opts.h_init, n_seeds);
    end

    sample = struct();   % keep one seed's time series for figures
    for s = 1:n_seeds
        ro = struct('seed', opts.seeds(s), 'verbose', false, 'collect_diag', true);
        out = run_pure_simulation(config, ro);
        N = numel(out.tout); idx = (n_warmup+1):N;

        % Tracking error (measured + noise-free) [nm]
        trk_meas(s, :) = std(out.p_d_out(idx,:) - out.p_m_out(idx,:), 0, 1) * 1e3;
        trk_true(s, :) = std(out.p_d_out(idx,:) - out.p_true_out(idx,:), 0, 1) * 1e3;

        % a_x_true per step from noise-free position
        h_true = (out.p_true_out * w_hat - pz_wall) / R_radius;   % N x 1
        a_true_para = zeros(N,1); a_true_perp = zeros(N,1);
        for kk = idx
            hb = max(h_true(kk), 1.001);
            [cpa, cpe] = calc_correction_functions(hb);
            a_true_para(kk) = a_nom / cpa;
            a_true_perp(kk) = a_nom / cpe;
        end
        % ekf_out cols: [a_hat_x, a_hat_z, a_hat_y, h_bar]; remap to [x y z]
        ah = [out.ekf_out(idx,1), out.ekf_out(idx,3), out.ekf_out(idx,2)];   % [x y z]
        a_hat_mean(s,:) = mean(ah, 1);
        a_hat_std(s,:)  = std(ah, 0, 1);
        a_true_mean(s,:) = [mean(a_true_para(idx)), mean(a_true_para(idx)), mean(a_true_perp(idx))];

        % delta_x_D^d disturbance estimate (diag.x_D_hat = slot 4)
        xD = out.diag.x_D_hat(idx, :);   % [N x 3], axes [x y z] (controller order)
        xD_absmean(s,:) = mean(abs(xD), 1);

        if s == 1
            sample.tout = out.tout; sample.idx = idx;
            sample.trk  = (out.p_d_out - out.p_m_out) * 1e3;   % nm
            sample.a_hat = [out.ekf_out(:,1), out.ekf_out(:,3), out.ekf_out(:,2)];  % [x y z]
            sample.a_true = [a_true_para, a_true_para, a_true_perp];
            sample.xD = out.diag.x_D_hat;
        end
    end

    % --- 4. Aggregate ---
    agg.trk_meas_mean = mean(trk_meas,1);   agg.trk_meas_std = std(trk_meas,0,1);
    agg.trk_true_mean = mean(trk_true,1);
    agg.a_hat_mean    = mean(a_hat_mean,1);
    agg.a_true_mean   = mean(a_true_mean,1);
    agg.a_hat_bias_pct = (agg.a_hat_mean - agg.a_true_mean) ./ agg.a_true_mean * 100;
    agg.a_hat_relstd_pct = mean(a_hat_std,1) ./ agg.a_hat_mean * 100;
    agg.xD_absmean    = mean(xD_absmean,1);

    % --- 5. PASS/FAIL ---
    pass_trk    = all(agg.trk_meas_mean < opts.trk_thresh_nm);
    pass_bias   = all(abs(agg.a_hat_bias_pct) < opts.bias_thresh_pct);
    pass_relstd = all(agg.a_hat_relstd_pct < opts.relstd_thresh_pct);
    pass_xD     = all(agg.xD_absmean < opts.xD_thresh_um);
    all_pass    = pass_trk && pass_bias && pass_relstd && pass_xD;

    ax = {'x','y','z'};
    fprintf('\n================ verify_eq17_6state_h50 (%d seeds, %.0fs) ================\n', n_seeds, opts.T_sim);
    fprintf('tracking std (p_d-p_m) [x y z] nm : [%.2f %.2f %.2f]  thresh %g  -> %s\n', ...
        agg.trk_meas_mean, opts.trk_thresh_nm, passstr(pass_trk));
    fprintf('a_hat bias vs a_true   [x y z] %% : [%.2f %.2f %.2f]  thresh %g  -> %s\n', ...
        agg.a_hat_bias_pct, opts.bias_thresh_pct, passstr(pass_bias));
    fprintf('a_hat rel-std          [x y z] %% : [%.2f %.2f %.2f]  thresh %g  -> %s\n', ...
        agg.a_hat_relstd_pct, opts.relstd_thresh_pct, passstr(pass_relstd));
    fprintf('|delta_x_D^d| mean     [x y z] um: [%.2e %.2e %.2e]  thresh %g -> %s\n', ...
        agg.xD_absmean, opts.xD_thresh_um, passstr(pass_xD));
    fprintf('-------------------------------------------------------------------------\n');
    fprintf('OVERALL: %s\n', passstr(all_pass));

    % --- 6. Save summary + figures ---
    out_dir = fullfile(project_root, 'test_results', 'eq17_6state_h50');
    if ~exist(out_dir, 'dir'); mkdir(out_dir); end
    save(fullfile(out_dir, 'summary.mat'), 'agg', 'trk_meas', 'trk_true', ...
         'a_hat_mean', 'a_hat_std', 'a_true_mean', 'xD_absmean', 'opts', 'config');
    write_summary_md(fullfile(out_dir, 'summary.md'), opts, agg, ...
                     pass_trk, pass_bias, pass_relstd, pass_xD, all_pass);
    if opts.save_fig
        make_figures(out_dir, sample, agg, ax);
    end
    if opts.verbose
        fprintf('Saved: %s\n', out_dir);
    end

    results = struct('aggregate', agg, 'pass', all_pass, ...
                     'pass_detail', struct('trk',pass_trk,'bias',pass_bias, ...
                                           'relstd',pass_relstd,'xD',pass_xD), ...
                     'trk_meas', trk_meas, 'a_hat_mean', a_hat_mean, 'a_hat_std', a_hat_std);
end


% ------------------------------------------------------------------
function s = passstr(b)
    if b; s = 'PASS'; else; s = 'FAIL'; end
end


function write_summary_md(path, opts, agg, pT, pB, pR, pX, all_pass)
    fid = fopen(path, 'w');
    fprintf(fid, '# verify_eq17_6state_h50\n\n');
    fprintf(fid, '6-state (RevisedControl_Vpersonal) controller, h=50 positioning, %d seeds x %.0fs.\n\n', ...
            numel(opts.seeds), opts.T_sim);
    fprintf(fid, '**OVERALL: %s**\n\n', passstr(all_pass));
    fprintf(fid, '| metric | x | y | z | thresh | result |\n|---|---|---|---|---|---|\n');
    fprintf(fid, '| tracking std (p_d-p_m) [nm] | %.2f | %.2f | %.2f | <%g | %s |\n', ...
            agg.trk_meas_mean, opts.trk_thresh_nm, passstr(pT));
    fprintf(fid, '| tracking std (p_d-p_true) [nm] | %.2f | %.2f | %.2f | -- | -- |\n', agg.trk_true_mean);
    fprintf(fid, '| a_hat bias vs a_true [%%] | %.2f | %.2f | %.2f | <%g | %s |\n', ...
            agg.a_hat_bias_pct, opts.bias_thresh_pct, passstr(pB));
    fprintf(fid, '| a_hat rel-std [%%] | %.2f | %.2f | %.2f | <%g | %s |\n', ...
            agg.a_hat_relstd_pct, opts.relstd_thresh_pct, passstr(pR));
    fprintf(fid, '| \\|delta_x_D^d\\| mean [um] | %.2e | %.2e | %.2e | <%g | %s |\n', ...
            agg.xD_absmean, opts.xD_thresh_um, passstr(pX));
    fprintf(fid, '\na_hat mean [x y z] = [%.4g %.4g %.4g], a_true mean = [%.4g %.4g %.4g] um/pN.\n', ...
            agg.a_hat_mean, agg.a_true_mean);
    fclose(fid);
end


function make_figures(out_dir, sample, agg, ax)
    GREEN = [0 0.6 0]; RED = [0.8 0 0]; BLUE = [0 0.2 0.8];
    col = {BLUE, GREEN, RED};
    t = sample.tout;

    % Fig 1: tracking error vs time (seed 1, 3 axes)
    f1 = figure('Visible','off','Position',[100 100 900 650]);
    hold on; box on; grid off;
    for a = 1:3
        plot(t, sample.trk(:,a), 'Color', col{a}, 'LineWidth', 1.5);
    end
    set(gca,'FontSize',20,'FontWeight','bold','LineWidth',2);
    xlabel('time [s]'); ylabel('tracking error p_d - p_m [nm]');
    legend({'x','y','z'}, 'Location','northoutside','Orientation','horizontal');
    exportgraphics(f1, fullfile(out_dir,'h50_1_tracking_error.png'),'Resolution',150); close(f1);

    % Fig 2: a_hat vs a_true (seed 1, 3 axes)
    f2 = figure('Visible','off','Position',[100 100 900 650]);
    hold on; box on; grid off;
    for a = 1:3
        plot(t, sample.a_hat(:,a)*1e3, 'Color', col{a}, 'LineWidth', 1.5);
        plot(t, sample.a_true(:,a)*1e3, '--', 'Color', col{a}, 'LineWidth', 2.5);
    end
    set(gca,'FontSize',20,'FontWeight','bold','LineWidth',2);
    xlabel('time [s]'); ylabel('a_x [10^{-3} um/pN]');
    legend({'x est','x true','y est','y true','z est','z true'}, ...
           'Location','northoutside','Orientation','horizontal','NumColumns',3);
    exportgraphics(f2, fullfile(out_dir,'h50_2_gain_est.png'),'Resolution',150); close(f2);

    % Fig 3: delta_x_D^d vs time (should be ~0)
    f3 = figure('Visible','off','Position',[100 100 900 650]);
    hold on; box on; grid off;
    for a = 1:3
        plot(t, sample.xD(:,a)*1e3, 'Color', col{a}, 'LineWidth', 1.5);
    end
    set(gca,'FontSize',20,'FontWeight','bold','LineWidth',2);
    xlabel('time [s]'); ylabel('delta\_x_D^d estimate [nm]');
    legend({'x','y','z'}, 'Location','northoutside','Orientation','horizontal');
    exportgraphics(f3, fullfile(out_dir,'h50_3_disturbance_est.png'),'Resolution',150); close(f3);

    % Fig 4: bias / rel-std bar summary
    f4 = figure('Visible','off','Position',[100 100 900 650]);
    bar([agg.a_hat_bias_pct; agg.a_hat_relstd_pct].');
    box on; grid off;
    set(gca,'FontSize',20,'FontWeight','bold','LineWidth',2,'XTickLabel',ax);
    xlabel('axis'); ylabel('%');
    legend({'a_{hat} bias','a_{hat} rel-std'}, 'Location','northoutside','Orientation','horizontal');
    exportgraphics(f4, fullfile(out_dir,'h50_4_ahat_stats.png'),'Resolution',150); close(f4);
end
