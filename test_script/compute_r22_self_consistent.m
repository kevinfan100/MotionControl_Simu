function [R22_scaling, R22_abs, history, diag_out] = ...
                    compute_r22_self_consistent(config, Q77_scaling, Q66_scaling, opts)
%COMPUTE_R22_SELF_CONSISTENT Solve R(2,2) as closed-loop self-consistent fixed point
%
%   [R22_scaling, R22_abs, history, diag_out] = ...
%       compute_r22_self_consistent(config, Q77_scaling, Q66_scaling, opts)
%
%   Per qr_theoretical_values.md Section 7:
%       R(2,2) = chi_sq * rho_a * (beta * C_dpmr_eff * Sigma_e(3,3))^2
%              / (C_dpmr_paper * 4*k_B*T)^2
%   where Sigma_e(3,3) = Var(delta_x[k-2]) in closed loop, itself depending
%   on R(2,2) through DARE -> L_ss -> F_e -> Lyapunov. Fixed-point iteration.
%
%   Under backward-diff beta interpretation, Q(6,6) = Q(7,7).
%
%   Inputs:
%       config        user config struct (defaults from user_config())
%       Q77_scaling   scalar, Q(7,7) scaling on sigma2_dXT
%       Q66_scaling   scalar, Q(6,6) scaling on sigma2_dXT
%                     (under beta: = Q77_scaling; under forward-diff: = 0)
%       opts          struct:
%           .tol      (1e-4)  relative convergence
%           .max_iter (30)
%           .alpha    (0.5)   under-relaxation factor
%           .a_phys   (a_nom) physical motion gain for operating point
%           .rho_a    ([])    [] => compute rigorously; scalar => use as override
%           .save_mat (true)
%           .save_plot (true)
%           .save_report (true)
%           .verbose (true)
%
%   Outputs:
%       R22_scaling   converged R(2,2) scaling (on sigma2_dXT)
%       R22_abs       converged R(2,2) absolute (um^2/pN^2)
%       history       N_iter x 5 matrix: [R22_scaling, Sigma_e_33, V_meas,
%                                         C_dpmr_eff, rel_change]
%       diag_out      struct with final values + infrastructure

    % ---------------------------------------------------------------
    % 0. Defaults
    % ---------------------------------------------------------------
    if nargin < 1 || isempty(config),   config = user_config(); end
    if nargin < 2 || isempty(Q77_scaling), Q77_scaling = 0; end
    if nargin < 3 || isempty(Q66_scaling), Q66_scaling = 0; end
    if nargin < 4 || isempty(opts),     opts = struct(); end

    if ~isfield(opts, 'tol'),          opts.tol = 1e-4; end
    if ~isfield(opts, 'max_iter'),     opts.max_iter = 30; end
    if ~isfield(opts, 'alpha'),        opts.alpha = 0.5; end
    if ~isfield(opts, 'a_phys'),       opts.a_phys = []; end
    if ~isfield(opts, 'rho_a'),        opts.rho_a = []; end
    if ~isfield(opts, 'save_mat'),     opts.save_mat = true; end
    if ~isfield(opts, 'save_plot'),    opts.save_plot = true; end
    if ~isfield(opts, 'save_report'),  opts.save_report = true; end
    if ~isfield(opts, 'verbose'),      opts.verbose = true; end
    if ~isfield(opts, 'rho_a_Lmax'),   opts.rho_a_Lmax = 200; end

    defaults = user_config();
    f = fieldnames(defaults);
    for i = 1:numel(f)
        if ~isfield(config, f{i}), config.(f{i}) = defaults.(f{i}); end
    end

    % ---------------------------------------------------------------
    % 1. Constants and physical quantities
    % ---------------------------------------------------------------
    phys = physical_constants();
    a_nom      = phys.Ts / phys.gamma_N;
    sigma2_dXT = 4 * phys.k_B * phys.T * a_nom;

    lc       = config.lambda_c;
    a_pd     = config.a_pd;
    a_cov    = config.a_cov;

    % Measurement noise variance (z-axis as reference; per-axis could be added)
    if config.meas_noise_enable
        sigma2_n = (config.meas_noise_std(3))^2;
    else
        sigma2_n = 0;
    end

    if isempty(opts.a_phys), opts.a_phys = a_nom; end
    a_phys = opts.a_phys;
    a_ratio = a_phys / a_nom;

    % Q(3,3) adaptive value (paper Eq.21)
    Q33_scaling = a_ratio^2;   % = 1 at a_phys = a_nom

    % ---------------------------------------------------------------
    % 2. Load beta (IIR finite-sample bias) from lookup
    % ---------------------------------------------------------------
    [script_dir, ~, ~] = fileparts(mfilename('fullpath'));
    project_root = fileparts(script_dir);
    bf_path = fullfile(project_root, 'test_results', 'verify', 'bias_factor_lookup.mat');

    if exist(bf_path, 'file')
        BF = load(bf_path);
        lc_cl = max(min(lc, BF.lc_grid(end)), BF.lc_grid(1));
        beta = interp1(BF.lc_grid, BF.bias_factor_tab, lc_cl, 'linear');
    else
        warning('compute_r22_self_consistent:bias_factor_missing', ...
                'bias_factor_lookup.mat not found; using beta=1');
        beta = 1;
    end

    % ---------------------------------------------------------------
    % 3. Structural constants
    % ---------------------------------------------------------------
    C_dpmr_paper = 2 + 2 / (1 - lc^2);
    chi_sq = 2 * a_cov / (2 - a_cov);

    % ---------------------------------------------------------------
    % 4. Fixed-point iteration
    % ---------------------------------------------------------------
    R22_scaling = 0.176 * a_ratio^2;    % starting guess per build_cdpmr_eff_lookup

    history = zeros(opts.max_iter, 6);
    converged = false;

    for iter = 1:opts.max_iter
        % Q and R vectors for this iteration (beta interpretation: Q66=Q77)
        Q_kf_scale = [0; 0; Q33_scaling; 0; 0; Q66_scaling; Q77_scaling];
        R_kf_scale = [sigma2_n/sigma2_dXT; R22_scaling];

        % Physical-scaling opts for compute_7state_cdpmr_eff
        ps = struct();
        ps.sigma2_dXT = sigma2_dXT;
        ps.a_phys     = a_phys;
        ps.a_nom      = a_nom;
        ps.sigma2_n   = sigma2_n;
        ps.Q66_abs    = Q66_scaling * sigma2_dXT;
        ps.Q77_abs    = Q77_scaling * sigma2_dXT;

        opts_in = struct('f0', 0, 'verbose', false, 'physical_scaling', ps);

        [C_dpmr_eff, ~, ~, A_aug, d] = compute_7state_cdpmr_eff( ...
            lc, sigma2_n/sigma2_dXT, a_pd, Q_kf_scale, R_kf_scale, opts_in);

        % Extract closed-loop tracking-error variance at time index [k-2]
        % (what del_pm actually measures, matching C_dpmr_eff definition)
        Sigma_e_33 = d.Sigma_e_dx_d2_phys;

        % Compute rigorous rho_a on first iteration
        if iter == 1 && isempty(opts.rho_a)
            rho_a = compute_rho_a_rigorous(A_aug, d, a_pd, a_cov, ...
                                            sigma2_n, opts.rho_a_Lmax);
        elseif iter == 1
            rho_a = opts.rho_a;
        end

        % Expected V_meas under closed-loop
        V_meas = beta * C_dpmr_eff * Sigma_e_33;

        % Chi-sq variance of V_meas estimator (finite-sample EMA on correlated samples)
        Var_V_meas = chi_sq * rho_a * V_meas^2;

        % Back-calculated R(2,2)
        R22_abs_new     = Var_V_meas / (C_dpmr_paper * 4 * phys.k_B * phys.T)^2;
        R22_scaling_new = R22_abs_new / sigma2_dXT;

        rel_change = abs(R22_scaling_new - R22_scaling) / max(R22_scaling, eps);

        history(iter, :) = [R22_scaling, Sigma_e_33, V_meas, C_dpmr_eff, ...
                            rel_change, R22_scaling_new];

        if opts.verbose
            fprintf('  iter %2d  R22=%.6f  Sigma_e=%.3e  V_meas=%.3e  C_dpmr_eff=%.4f  rel=%.2e\n', ...
                    iter, R22_scaling, Sigma_e_33, V_meas, C_dpmr_eff, rel_change);
        end

        if rel_change < opts.tol
            converged = true;
            R22_scaling = R22_scaling_new;   % final exact value
            history(iter, 1) = R22_scaling;
            break;
        end

        % Under-relax to damp oscillation
        R22_scaling = opts.alpha * R22_scaling + (1 - opts.alpha) * R22_scaling_new;
    end

    history = history(1:iter, :);
    R22_abs = R22_scaling * sigma2_dXT;

    % ---------------------------------------------------------------
    % 5. Sanity gates
    % ---------------------------------------------------------------
    gate = struct();
    gate.converged = converged;
    gate.n_iter = iter;

    % G3: converged value within +-30% of 0.176 (free-space baseline)
    gate.G3_free_space_ratio = R22_scaling / (0.176 * a_ratio^2);
    gate.G3_pass = gate.G3_free_space_ratio > 0.7 && gate.G3_free_space_ratio < 1.3;

    % G4: history rel_change monotonic (allowing small non-monotonicity from under-relaxation)
    if size(history, 1) >= 3
        rel_changes = history(:, 5);
        % Count increases in rel_change (should be few)
        n_increases = sum(diff(rel_changes) > 1e-6);
        gate.G4_n_increases = n_increases;
        gate.G4_pass = n_increases <= 2;   % tolerate mild oscillation
    else
        gate.G4_pass = NaN;
        gate.G4_n_increases = NaN;
    end

    % ---------------------------------------------------------------
    % 6. Assemble diagnostics
    % ---------------------------------------------------------------
    diag_out = struct();
    diag_out.R22_scaling = R22_scaling;
    diag_out.R22_abs = R22_abs;
    diag_out.Sigma_e_33 = Sigma_e_33;
    diag_out.V_meas = V_meas;
    diag_out.C_dpmr_eff = C_dpmr_eff;
    diag_out.C_dpmr_paper = C_dpmr_paper;
    diag_out.chi_sq = chi_sq;
    diag_out.rho_a = rho_a;
    diag_out.beta = beta;
    diag_out.Q33_scaling = Q33_scaling;
    diag_out.Q66_scaling = Q66_scaling;
    diag_out.Q77_scaling = Q77_scaling;
    diag_out.sigma2_dXT = sigma2_dXT;
    diag_out.sigma2_n = sigma2_n;
    diag_out.a_nom = a_nom;
    diag_out.a_phys = a_phys;
    diag_out.a_ratio = a_ratio;
    diag_out.operating_point = struct('lc', lc, 'a_pd', a_pd, 'a_cov', a_cov, ...
                                      'sigma2_n', sigma2_n, 'meas_noise_enable', ...
                                      config.meas_noise_enable);
    diag_out.history = history;
    diag_out.gate = gate;
    diag_out.R_kf_scale_final = [sigma2_n/sigma2_dXT; R22_scaling];
    diag_out.Q_kf_scale_final = [0; 0; Q33_scaling; 0; 0; Q66_scaling; Q77_scaling];

    % ---------------------------------------------------------------
    % 7. Console summary
    % ---------------------------------------------------------------
    if opts.verbose
        fprintf('\n===== compute_r22_self_consistent summary =====\n');
        fprintf('Operating point: lc=%.3f a_pd=%.3f a_cov=%.3f a_phys=a_nom=%.4e\n', ...
                lc, a_pd, a_cov, a_nom);
        fprintf('Noise: sigma2_n=%.3e um^2  (R(1,1)_scaling=%.4f)\n', ...
                sigma2_n, sigma2_n/sigma2_dXT);
        fprintf('Q(3,3) scaling = %.4f  (= (a_phys/a_nom)^2)\n', Q33_scaling);
        fprintf('Q(6,6) scaling = %.4e  (beta: same as Q(7,7))\n', Q66_scaling);
        fprintf('Q(7,7) scaling = %.4e\n', Q77_scaling);
        fprintf('Converged: %s (%d iterations)\n', tern(converged, 'YES', 'NO'), iter);
        fprintf('R(2,2)_scaling = %.6f\n', R22_scaling);
        fprintf('R(2,2)_abs     = %.4e um^2/pN^2\n', R22_abs);
        fprintf('Sigma_e(3,3)   = %.4e um^2\n', Sigma_e_33);
        fprintf('V_meas         = %.4e um^2\n', V_meas);
        fprintf('C_dpmr_eff     = %.4f   C_dpmr_paper = %.4f\n', C_dpmr_eff, C_dpmr_paper);
        fprintf('beta           = %.4f   chi_sq = %.4f   rho_a = %.4f\n', ...
                beta, chi_sq, rho_a);
        fprintf('\nSanity gates:\n');
        fprintf('   G3 (free-space +-30%% of 0.176): %s (ratio=%.3f)\n', ...
                tern(gate.G3_pass, 'PASS', 'WARN'), gate.G3_free_space_ratio);
        fprintf('   G4 (convergence monotonic):     %s (%d increases)\n\n', ...
                tern(gate.G4_pass == true, 'PASS', 'WARN'), gate.G4_n_increases);
    end

    % ---------------------------------------------------------------
    % 8. Save outputs
    % ---------------------------------------------------------------
    if opts.save_mat
        out_dir = fullfile(project_root, 'test_results', 'verify');
        if ~exist(out_dir, 'dir'), mkdir(out_dir); end
        out_file = fullfile(out_dir, 'r22_self_consistent.mat');
        save(out_file, '-struct', 'diag_out');
        if opts.verbose, fprintf('Saved: %s\n', out_file); end
    end

    if opts.save_plot
        fig_dir = fullfile(project_root, 'reference', 'for_test');
        if ~exist(fig_dir, 'dir'), mkdir(fig_dir); end
        fig_file = fullfile(fig_dir, 'fig_r22_convergence.png');

        fig = figure('Position', [100, 100, 900, 500], 'Visible', 'off');
        subplot(1,2,1);
        semilogy(history(:,1), 'b-o', 'LineWidth', 1.3); hold on;
        semilogy(history(:,6), 'r--x', 'LineWidth', 1.0);
        legend({'R22 (pre-update)', 'R22 (proposed)'}, 'Location', 'northoutside', ...
               'Orientation', 'horizontal');
        xlabel('iteration');
        ylabel('R(2,2) scaling');
        set(gca, 'FontSize', 12);

        subplot(1,2,2);
        semilogy(history(:,5), 'k-o', 'LineWidth', 1.3);
        xlabel('iteration');
        ylabel('relative change');
        set(gca, 'FontSize', 12);
        hold on;
        yline(opts.tol, 'r--', 'LineWidth', 1.0);

        saveas(fig, fig_file);
        close(fig);
        if opts.verbose, fprintf('Saved: %s\n', fig_file); end
    end

    if opts.save_report
        rep_dir = fullfile(project_root, 'reference', 'for_test');
        if ~exist(rep_dir, 'dir'), mkdir(rep_dir); end
        rep_file = fullfile(rep_dir, 'r22_self_consistent_report.md');
        write_report(rep_file, diag_out, gate, opts);
        if opts.verbose, fprintf('Saved: %s\n', rep_file); end
    end
end

% =========================================================================
function rho_a = compute_rho_a_rigorous(A_aug, d, a_pd, a_cov, sigma2_n, Lmax)
%COMPUTE_RHO_A_RIGOROUS Autocorrelation amplification for IIR variance estimator
%   rho_a = 1 + 2 * sum_{L=1}^{Lmax} rho(L)^2 * (1 - a_cov)^L
%   where rho(L) = normalized autocorrelation of del_pmr at lag L.
%   Only state-contribution part is used (exogenous noise is white so
%   contributes no correlation at L>=1; its variance only dilutes rho_state).

    n_aug = size(A_aug, 1);
    gain = (1 - a_pd);

    % Output selector c such that state part of del_pmr = c' * x_aug
    % del_pmr[k] = gain * (dx_d2[k] - pmd_prev[k] + n_p[k])
    c = zeros(n_aug, 1);
    c(d.idx_dx_d2)    =  gain;
    c(d.idx_pmd_prev) = -gain;

    % Build physical Sigma_aug if provided, else use unit-variance combination
    if isfield(d, 'Sigma_aug_phys')
        Sigma_full = d.Sigma_aug_phys;
    else
        % fallback: unit-variance combination (thermal only)
        Sigma_full = d.Sigma_th;
    end

    % State-contribution variance and autocorrelation
    V_state = c' * Sigma_full * c;
    if V_state <= 0
        rho_a = 1;   % degenerate; no correlation effect
        return;
    end

    % Noise contribution to del_pmr variance (exogenous, adds to L=0 only)
    V_noise = gain^2 * sigma2_n;
    V_total = V_state + V_noise;

    % Autocorrelation at lag L: state part propagates via A_aug
    rho_sum = 0;
    cA = c';           % 1 x n_aug
    damp = 1 - a_cov;
    damp_L = damp;     % (1-a_cov)^L accumulating

    for L = 1:Lmax
        cA = cA * A_aug;                  % c' * A^L
        corr_L = (cA * Sigma_full * c);   % scalar
        rho_L = corr_L / V_total;          % normalized by total (state+noise)
        rho_sum = rho_sum + (rho_L^2) * damp_L;
        damp_L = damp_L * damp;
        if damp_L < 1e-20, break; end
    end

    rho_a = 1 + 2 * rho_sum;
end

% =========================================================================
function write_report(rep_file, diag_out, gate, opts)
    fid = fopen(rep_file, 'w');
    if fid < 0, return; end
    fprintf(fid, '# R(2,2) Self-Consistent Fixed-Point Report\n\n');
    fprintf(fid, '**Generated**: %s\n\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
    fprintf(fid, '## Operating point\n\n');
    op = diag_out.operating_point;
    fprintf(fid, '- lc = %.3f\n- a_pd = %.3f\n- a_cov = %.3f\n', ...
            op.lc, op.a_pd, op.a_cov);
    fprintf(fid, '- a_phys = a_nom = %.4e um/pN (free-space)\n', diag_out.a_nom);
    fprintf(fid, '- sigma2_n = %.3e um^2 (noise %s)\n', ...
            op.sigma2_n, tern(op.meas_noise_enable, 'ON', 'OFF'));
    fprintf(fid, '- sigma2_dXT = %.4e um^2\n\n', diag_out.sigma2_dXT);

    fprintf(fid, '## Q vector deployed\n\n');
    fprintf(fid, '```\nQz_diag_scaling = [0; 0; %.4f; 0; 0; %.4e; %.4e]\n```\n\n', ...
            diag_out.Q33_scaling, diag_out.Q66_scaling, diag_out.Q77_scaling);

    fprintf(fid, '## Converged R(2,2)\n\n');
    fprintf(fid, '- **R(2,2) scaling** (on sigma2_dXT): **%.6f**\n', ...
            diag_out.R22_scaling);
    fprintf(fid, '- R(2,2) absolute: %.4e um^2/pN^2\n', diag_out.R22_abs);
    fprintf(fid, '- Sigma_e(3,3) = Var(delta_x[k-2]) = %.4e um^2\n', ...
            diag_out.Sigma_e_33);
    fprintf(fid, '- V_meas = beta*C_dpmr_eff*Sigma_e = %.4e um^2\n', diag_out.V_meas);
    fprintf(fid, '- Converged: %s (%d iterations)\n\n', ...
            tern(gate.converged, 'YES', 'NO'), gate.n_iter);

    fprintf(fid, '## Structural constants\n\n');
    fprintf(fid, '- C_dpmr_paper = 2 + 2/(1-lc^2) = %.4f\n', diag_out.C_dpmr_paper);
    fprintf(fid, '- C_dpmr_eff (closed-loop, augmented Lyapunov) = %.4f\n', ...
            diag_out.C_dpmr_eff);
    fprintf(fid, '- chi_sq = 2*a_cov/(2-a_cov) = %.4f\n', diag_out.chi_sq);
    fprintf(fid, '- rho_a (autocorr amplification) = %.4f\n', diag_out.rho_a);
    fprintf(fid, '- beta (finite-sample IIR bias) = %.4f\n\n', diag_out.beta);

    fprintf(fid, '## Convergence history\n\n');
    fprintf(fid, '| iter | R22_scaling | Sigma_e(3,3) | V_meas | C_dpmr_eff | rel_change | R22_proposed |\n');
    fprintf(fid, '|---|---|---|---|---|---|---|\n');
    for k = 1:size(diag_out.history, 1)
        r = diag_out.history(k, :);
        fprintf(fid, '| %d | %.6f | %.3e | %.3e | %.4f | %.2e | %.6f |\n', ...
                k, r(1), r(2), r(3), r(4), r(5), r(6));
    end
    fprintf(fid, '\n## Sanity gates\n\n');
    fprintf(fid, '- G3 (free-space +-30%% of 0.176): **%s** (ratio %.3f)\n', ...
            tern(gate.G3_pass, 'PASS', 'WARN'), gate.G3_free_space_ratio);
    fprintf(fid, '- G4 (convergence monotonic):     **%s** (%d non-mono steps)\n', ...
            tern(gate.G4_pass == true, 'PASS', 'WARN'), gate.G4_n_increases);

    fclose(fid);
end

% =========================================================================
function s = tern(cond, a, b)
    if cond, s = a; else, s = b; end
end
