function report = analyze_R22_validation()
%ANALYZE_R22_VALIDATION  Phase 9 Stage 0 R(2,2) validation analyzer
%
%   report = analyze_R22_validation()
%
%   Consumes:
%     reference/eq17_analysis/phase9_predictions.mat                 (Agent X)
%     reference/eq17_analysis/phase9_results_{tag}_seed{i}.mat       (Agent Y)
%
%   Produces metrics V1-V6 per axis [x, y, z], then determines Path A/B/C
%   per the phase9_R22_validation_plan.md acceptance criteria. Writes
%   reference/eq17_analysis/phase9_validation_report.md.
%
%   Tag convention (Agent Y interface, per run_phase9_wave1.m):
%     V1_seed{1..5}                                       — baseline a_cov=0.05, sigma2_n_factor=1
%     V2_acov{0p01,0p02,0p1,0p2}_seed1                    — V2 a_cov sweep (seed 1)
%     V3_sigma2n{0p5,2,4}_seed1                           — V3 sigma2_n_factor sweep (seed 1)
%     (V2 a_cov=0.05 baseline reuses V1_seed1; V3 sigma2n=1 baseline reuses V1_seed1.)
%
%   The function does NOT execute MATLAB simulations — it only loads .mat
%   files and aggregates statistics. Path-fail diagnostics suggest which
%   formula/code line to inspect next.
%
%   Field-name interface contracts (declared by the user):
%     predictions:
%       lambda_c, IF_var
%       C_dpmr_paper, C_n_paper
%       C_dpmr_eff_per_axis [3], C_n_eff_per_axis [3]
%       a_true_per_axis [3]
%       xi_paper_per_axis [3], xi_eff_per_axis [3]
%       R22_pred_paper_per_axis [3], R22_pred_eff_per_axis [3]
%       V2_R22_pred_paper [5,3], V2_R22_pred_eff [5,3]
%       V3_R22_pred_paper [4,3], V3_R22_pred_eff [4,3]
%       V4_ACF_predicted_optionA [51]
%
%     sim results (per file, saved with `save -struct results -v7.3`):
%       diag.a_xm [N,3], diag.sigma2_dxr_hat [N,3]
%       diag.delta_a_hat [N,3], diag.a_hat [N,3]
%       diag.dx_r [N,3], diag.P77 [N,3], diag.Q77 [N,3]
%       diag.guards_individual [N,3,3]   (dims = [time, guard_index, axis_index];
%                                         guard 1=G1 warmup, 2=G2 SNR, 3=G3 wall)
%       config.a_cov, config.meas_noise_std, config.T_sim
%       a_true_per_axis [3]

    % ---------- 0. Setup paths and constants ----------
    [script_dir, ~, ~] = fileparts(mfilename('fullpath'));
    project_root = fileparts(script_dir);
    eq17_dir     = fullfile(project_root, 'reference', 'eq17_analysis');

    Ts          = 1 / 1600;
    N_skip      = ceil(2.0 / Ts);          % 3200
    AXIS_LABELS = {'x', 'y', 'z'};
    SEEDS       = 1:5;
    A_COV_GRID  = [0.01, 0.02, 0.05, 0.1, 0.2];
    N_FAC_GRID  = [0.5, 1, 2, 4];
    BASELINE_ACOV_IDX = find(abs(A_COV_GRID - 0.05) < 1e-12, 1);   % 3
    BASELINE_NFAC_IDX = find(abs(N_FAC_GRID - 1)    < 1e-12, 1);   % 2

    % Tolerances (plan §7)
    V1_RATIO_TOL_MEAN = 0.05;
    V1_RATIO_TOL_SE   = 0.03;
    V2_SLOPE_TOL      = 0.05;
    V2_R2_MIN         = 0.99;
    V3_RATIO_TOL      = 0.10;
    V4_ACF_TOL        = 0.10;
    V4_IFVAR_TOL      = 0.10;
    V5_BIAS_PCT_TOL   = 2.0;

    % V6 "indistinguishable" threshold:
    % per Agent X's predictions audit (reference/eq17_analysis/phase9_predictions_audit.md),
    % at h=50 production conditions xi_eff/xi_paper differs by ~0.5-0.8% across axes,
    % so |dist_paper - dist_eff| < 0.01 (1 percentage point in the variance ratio)
    % means paper and eff predictions are within the empirical noise floor and
    % cannot be distinguished by V1 ratios alone. This threshold sits well below
    % V1_RATIO_TOL_MEAN (5%) so it is meaningful only when both predictions
    % already pass V1.
    V6_INDISTINGUISHABLE_TOL = 0.01;

    % ---------- 1. Load predictions ----------
    pred_path = fullfile(eq17_dir, 'phase9_predictions.mat');
    assert(isfile(pred_path), ...
        'analyze_R22_validation:missingPredictions', ...
        'Predictions file not found: %s', pred_path);
    pred = load(pred_path);

    required_pred_fields = { ...
        'lambda_c', 'IF_var', ...
        'C_dpmr_paper', 'C_n_paper', ...
        'C_dpmr_eff_per_axis', 'C_n_eff_per_axis', ...
        'a_true_per_axis', ...
        'xi_paper_per_axis', 'xi_eff_per_axis', ...
        'R22_pred_paper_per_axis', 'R22_pred_eff_per_axis', ...
        'V2_R22_pred_paper', 'V2_R22_pred_eff', ...
        'V3_R22_pred_paper', 'V3_R22_pred_eff', ...
        'V4_ACF_predicted_optionA'};
    for i = 1:numel(required_pred_fields)
        fn = required_pred_fields{i};
        assert(isfield(pred, fn), ...
            'analyze_R22_validation:missingPredField', ...
            'Predictions file missing field: %s', fn);
    end

    a_true_per_axis = pred.a_true_per_axis(:).';                        % 1x3
    R22_paper       = pred.R22_pred_paper_per_axis(:).';                % 1x3
    R22_eff         = pred.R22_pred_eff_per_axis(:).';                  % 1x3
    xi_paper        = pred.xi_paper_per_axis(:).';                      % 1x3
    IF_var          = pred.IF_var;                                      % scalar
    V2_pred_paper   = pred.V2_R22_pred_paper;                           % 5x3
    V2_pred_eff     = pred.V2_R22_pred_eff;                             % 5x3
    V3_pred_paper   = pred.V3_R22_pred_paper;                           % 4x3
    V3_pred_eff     = pred.V3_R22_pred_eff;                             % 4x3
    V4_ACF_pred     = pred.V4_ACF_predicted_optionA(:);                 % 51x1

    % ---------- 2. V1 seed-1 sanity gates ----------
    v1_seed1_path = phase9_filepath(eq17_dir, 'V1_seed1');
    sim1 = load_sim(v1_seed1_path);
    sanity = run_sanity_gates(sim1, N_skip);

    if ~sanity.all_pass
        report = build_halt_report(sanity, eq17_dir);
        write_report_md(report, eq17_dir);
        return;
    end

    % ---------- 3. V1: variance ratio (5 seeds) ----------
    n_seeds = numel(SEEDS);
    emp_mean_v1 = zeros(3, n_seeds);
    emp_var_v1  = zeros(3, n_seeds);

    for s = 1:n_seeds
        if SEEDS(s) == 1
            sim_s = sim1;
        else
            sim_s = load_sim(phase9_filepath(eq17_dir, sprintf('V1_seed%d', SEEDS(s))));
        end
        a_xm_ss = sim_s.diag.a_xm(N_skip:end, :);          % [N_ss x 3]
        emp_mean_v1(:, s) = mean(a_xm_ss, 1).';            % 3x1
        emp_var_v1(:,  s) = var(a_xm_ss, 0, 1).';          % 3x1
    end

    ratio_paper = emp_var_v1 ./ R22_paper(:);          % 3 x n_seeds
    ratio_eff   = emp_var_v1 ./ R22_eff(:);            % 3 x n_seeds

    V1.emp_mean_per_seed = emp_mean_v1;
    V1.emp_var_per_seed  = emp_var_v1;
    V1.ratio_paper_mean  = mean(ratio_paper, 2).';     % 1x3
    V1.ratio_paper_se    = std(ratio_paper, 0, 2).' / sqrt(n_seeds);
    V1.ratio_eff_mean    = mean(ratio_eff,   2).';
    V1.ratio_eff_se      = std(ratio_eff,   0, 2).' / sqrt(n_seeds);
    V1.pass_paper        = (abs(V1.ratio_paper_mean - 1) <= V1_RATIO_TOL_MEAN) ...
                          & (V1.ratio_paper_se <= V1_RATIO_TOL_SE);
    V1.pass_eff          = (abs(V1.ratio_eff_mean   - 1) <= V1_RATIO_TOL_MEAN) ...
                          & (V1.ratio_eff_se   <= V1_RATIO_TOL_SE);

    % ---------- 4. V2: a_cov linearity (single-seed sweep + V1 seed1) ----------
    emp_var_v2 = zeros(numel(A_COV_GRID), 3);
    for k = 1:numel(A_COV_GRID)
        if k == BASELINE_ACOV_IDX
            sim_k = sim1;                          % reuse V1 seed1
        else
            tag = sprintf('V2_acov%s_seed1', num2tag(A_COV_GRID(k)));
            sim_k = load_sim(phase9_filepath(eq17_dir, tag));
        end
        emp_var_v2(k, :) = var(sim_k.diag.a_xm(N_skip:end, :), 0, 1);   % 1x3
    end

    % Linear regression through origin per axis: emp_var = slope * a_cov
    V2.emp_var       = emp_var_v2;
    V2.a_cov_grid    = A_COV_GRID(:);
    V2.slope         = zeros(1, 3);
    V2.R2            = zeros(1, 3);
    V2.pred_slope_paper = zeros(1, 3);
    V2.pred_slope_eff   = zeros(1, 3);
    for ax = 1:3
        x = A_COV_GRID(:);
        y = emp_var_v2(:, ax);
        slope = (x.' * y) / (x.' * x);             % through-origin LS
        ss_res = sum((y - slope*x).^2);
        ss_tot = sum(y.^2);                        % through-origin R^2 vs zero
        V2.slope(ax) = slope;
        V2.R2(ax)    = 1 - ss_res / ss_tot;

        % predicted slope: emp_var(a_cov) = a_cov * IF_var * (a_true + xi)^2
        V2.pred_slope_paper(ax) = mean(V2_pred_paper(:, ax) ./ A_COV_GRID(:));
        V2.pred_slope_eff(ax)   = mean(V2_pred_eff(:,   ax) ./ A_COV_GRID(:));
    end
    V2.slope_ratio_paper = V2.slope ./ V2.pred_slope_paper;
    V2.slope_ratio_eff   = V2.slope ./ V2.pred_slope_eff;
    V2.pass_paper = (abs(V2.slope_ratio_paper - 1) <= V2_SLOPE_TOL) & (V2.R2 >= V2_R2_MIN);
    V2.pass_eff   = (abs(V2.slope_ratio_eff   - 1) <= V2_SLOPE_TOL) & (V2.R2 >= V2_R2_MIN);

    % ---------- 5. V3: sigma2_n factor sweep ----------
    emp_var_v3 = zeros(numel(N_FAC_GRID), 3);
    for k = 1:numel(N_FAC_GRID)
        if k == BASELINE_NFAC_IDX
            sim_k = sim1;
        else
            tag = sprintf('V3_sigma2n%s_seed1', num2tag(N_FAC_GRID(k)));
            sim_k = load_sim(phase9_filepath(eq17_dir, tag));
        end
        emp_var_v3(k, :) = var(sim_k.diag.a_xm(N_skip:end, :), 0, 1);   % 1x3
    end

    V3.n_fac_grid     = N_FAC_GRID(:);
    V3.emp_var        = emp_var_v3;
    V3.ratio_paper    = emp_var_v3 ./ V3_pred_paper;
    V3.ratio_eff      = emp_var_v3 ./ V3_pred_eff;
    V3.pass_paper     = all(abs(V3.ratio_paper - 1) <= V3_RATIO_TOL, 1);
    V3.pass_eff       = all(abs(V3.ratio_eff   - 1) <= V3_RATIO_TOL, 1);
    % y-axis xi tiny -> V3 weakly informative on y; record but flag
    V3.informative    = [true, false, true];   % x mixed, y thermal, z sensor

    % ---------- 6. V4: ACF (V1 seed 1) ----------
    V4.acf_emp = zeros(51, 3);
    V4.IF_var_emp = zeros(1, 3);
    for ax = 1:3
        x_steady = sim1.diag.dx_r(N_skip:end, ax);          % column vec
        V4.acf_emp(:, ax) = compute_acf_fft(x_steady, 50);
    end
    V4.acf_pred = V4_ACF_pred;
    V4.acf_dev  = abs(V4.acf_emp - V4.acf_pred);
    V4.max_dev_2_51 = max(V4.acf_dev(2:51, :), [], 1);                   % 1x3
    % IF_var = 1 + 2*sum(rho^2) per Agent X audit (Phase 2 §7 / phase9_predictions_audit.md:136)
    % For Gaussian dx_r, this equals sum of ACF of dx_r^2 (Isserlis), which matches
    % Var(EWMA of dx_r^2) = a_cov * IF_var * (sigma2_dxr)^2.
    for ax = 1:3
        V4.IF_var_emp(ax) = 1 + 2 * sum(V4.acf_emp(2:end, ax).^2);
    end
    V4.IF_var_ratio = V4.IF_var_emp / IF_var;
    V4.pass = (V4.max_dev_2_51 <= V4_ACF_TOL) ...
            & (abs(V4.IF_var_ratio - 1) <= V4_IFVAR_TOL);

    % ---------- 7. V5: DC bias (5-seed aggregate) ----------
    V5.mean_per_axis = mean(emp_mean_v1, 2).';                            % 1x3
    V5.bias_pct      = (V5.mean_per_axis - a_true_per_axis) ./ a_true_per_axis * 100;
    V5.pass          = abs(V5.bias_pct) <= V5_BIAS_PCT_TOL;

    % ---------- 8. V6: paper vs eff ----------
    V6.dist_paper = abs(V1.ratio_paper_mean - 1);                         % 1x3
    V6.dist_eff   = abs(V1.ratio_eff_mean   - 1);
    V6.indistinguishable_tol = V6_INDISTINGUISHABLE_TOL;
    V6.closer_to_one = repmat({''}, 1, 3);   % 'eff' | 'paper' | 'indistinguishable'
    for ax = 1:3
        if abs(V6.dist_paper(ax) - V6.dist_eff(ax)) <= V6_INDISTINGUISHABLE_TOL
            % paper and eff predictions are within the empirical noise floor
            V6.closer_to_one{ax} = 'indistinguishable';
        elseif V6.dist_eff(ax) < V6.dist_paper(ax)
            V6.closer_to_one{ax} = 'eff';
        else
            V6.closer_to_one{ax} = 'paper';
        end
    end
    % y-axis structurally uninformative (xi_y ~ 0)
    V6.uninformative_axes = abs(xi_paper) < 1e-12 * max(abs(a_true_per_axis));

    % Aggregate verdict over informative axes (x, z).
    % 4 outcome categories: paper_wins, eff_wins, indistinguishable, mixed.
    informative_v6 = ~V6.uninformative_axes;
    inf_labels = V6.closer_to_one(informative_v6);
    V6.eff_wins_overall      = ~isempty(inf_labels) && all(strcmp(inf_labels, 'eff'));
    V6.paper_wins_overall    = ~isempty(inf_labels) && all(strcmp(inf_labels, 'paper'));
    V6.indistinguishable_overall = ~isempty(inf_labels) ...
        && all(strcmp(inf_labels, 'indistinguishable'));

    % ---------- 9. Path determination ----------
    informative_v3 = V3.informative;       % only require pass on informative axes for V3

    paper_v1_all  = all(V1.pass_paper);
    eff_v1_all    = all(V1.pass_eff);
    v2_paper_all  = all(V2.pass_paper);
    v2_eff_all    = all(V2.pass_eff);
    v3_paper_inf  = all(V3.pass_paper(informative_v3));
    v3_eff_inf    = all(V3.pass_eff(informative_v3));
    v4_all        = all(V4.pass);
    v5_all        = all(V5.pass);

    common_pass = v4_all && v5_all;

    if paper_v1_all && eff_v1_all && (v2_paper_all || v2_eff_all) ...
            && (v3_paper_inf || v3_eff_inf) && common_pass
        Path = 'A';
        if V6.indistinguishable_overall
            % Empirically expected at h=50 per Agent X's predictions audit:
            % xi_eff/xi_paper differ by <1%, so V6 cannot favor either form.
            path_reason = ['V1/V2/V3/V4/V5 all pass for informative axes. ' ...
                           'V6 INDISTINGUISHABLE: paper and eff predictions ' ...
                           'differ by less than the empirical noise floor at ' ...
                           'h=50, so the formula choice has no measurable ' ...
                           'impact at this operating point. The Wave 4 v3 ' ...
                           'a_hat bias is therefore NOT explained by the ' ...
                           'paper-vs-eff xi choice; consult ' ...
                           'reference/eq17_analysis/phase9_predictions_audit.md.'];
        else
            path_reason = ['V1/V2/V3/V4/V5 all pass for informative axes; ' ...
                           'paper and eff predictions are equivalent.'];
        end
    elseif eff_v1_all && ~paper_v1_all && v2_eff_all && v3_eff_inf && common_pass
        Path = 'B';
        path_reason = ['V1 paper fails on x and/or z but eff passes; ' ...
                       'remaining metrics consistent. ' ...
                       'Recommend keeping per-axis effective C_dpmr/C_n in R(2,2).'];
    else
        Path = 'C';
        path_reason = collect_path_c_reasons(V1, V2, V3, V4, V5, AXIS_LABELS, ...
                                             informative_v3);
    end

    % ---------- 10. Build report struct ----------
    report.timestamp        = char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss'));
    report.predictions_path = pred_path;
    report.eq17_dir         = eq17_dir;
    report.N_skip           = N_skip;
    report.Ts               = Ts;
    report.axes             = AXIS_LABELS;
    report.a_true_per_axis  = a_true_per_axis;
    report.IF_var           = IF_var;
    report.xi_paper         = xi_paper;
    report.R22_paper        = R22_paper;
    report.R22_eff          = R22_eff;
    report.sanity           = sanity;
    report.V1               = V1;
    report.V2               = V2;
    report.V3               = V3;
    report.V4               = V4;
    report.V5               = V5;
    report.V6               = V6;
    report.path             = Path;
    report.path_reason      = path_reason;

    % ---------- 11. Write markdown ----------
    write_report_md(report, eq17_dir);
end

% =====================================================================
%                       Helper functions
% =====================================================================

function p = phase9_filepath(eq17_dir, tag)
    p = fullfile(eq17_dir, sprintf('phase9_results_%s.mat', tag));
end

function s = load_sim(filepath)
    assert(isfile(filepath), ...
        'analyze_R22_validation:missingSimFile', ...
        'Sim result file not found: %s', filepath);
    s = load(filepath);
    % expect fields: diag, config, a_true_per_axis
    required = {'diag', 'config', 'a_true_per_axis'};
    for i = 1:numel(required)
        assert(isfield(s, required{i}), ...
            'analyze_R22_validation:missingSimField', ...
            'Sim file %s missing field: %s', filepath, required{i});
    end
end

function tag = num2tag(x)
    % 0.05 -> '0p05', 4 -> '4', 0.5 -> '0p5'
    if x == round(x)
        tag = sprintf('%d', round(x));
    else
        s = sprintf('%g', x);
        tag = strrep(s, '.', 'p');
    end
end

function gates = run_sanity_gates(sim, N_skip)
    d = sim.diag;
    N = size(d.a_xm, 1);                                       % time is dim=1
    idx_ss = N_skip:N;

    g1 = max(abs(d.delta_a_hat(:))) < 1e-30;                   % slot 7 zero
    a_hat_ss = d.a_hat(idx_ss, :);                             % [N_ss x 3]
    % Per-axis time-series range (not flat across axes — x/y differ from z by design)
    a_hat_per_axis_range = max(a_hat_ss, [], 1) - min(a_hat_ss, [], 1);  % 1x3
    g2 = max(a_hat_per_axis_range) < 1e-30;                    % a_hat frozen per axis
    g3 = all(d.Q77(:) == 0);                                   % Q77 zero
    g_ss = d.guards_individual(idx_ss, :, :);                  % [N_ss x 3 x 3]
    g4 = ~any(g_ss(:));                                        % guards inactive

    gates.delta_a_hat_zero = g1;
    gates.a_hat_frozen     = g2;
    gates.Q77_zero         = g3;
    gates.guards_inactive  = g4;
    gates.all_pass         = g1 && g2 && g3 && g4;
end

function r = build_halt_report(sanity, eq17_dir)
    r = struct();
    r.timestamp   = char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss'));
    r.eq17_dir    = eq17_dir;
    r.sanity      = sanity;
    r.path        = 'HALT';
    r.path_reason = ['Sanity gates failed; aborting. ' ...
                     'Inspect controller config (a_design freeze, Q77, guards).'];
end

function acf = compute_acf_fft(x, max_lag)
    % Manual unbiased autocorrelation via FFT, normalized by lag-0.
    x = x(:) - mean(x);
    n = numel(x);
    nfft = 2^nextpow2(2*n - 1);
    Fx = fft(x, nfft);
    r_full = real(ifft(Fx .* conj(Fx)));
    r = r_full(1:max_lag+1);
    acf = r / r(1);
end

function reasons = collect_path_c_reasons(V1, V2, V3, V4, V5, AXIS_LABELS, informative_v3)
    lines = {};
    for ax = 1:3
        if ~V1.pass_paper(ax) && ~V1.pass_eff(ax)
            lines{end+1} = sprintf( ...
                ['V1 fails on axis %s (paper ratio=%.3f+/-%.3f, ' ...
                 'eff ratio=%.3f+/-%.3f). Inspect R22 formula in ' ...
                 'motion_control_law_eq17_7state.m R(2,2) assembly ' ...
                 'and Phase 6 derivation.'], ...
                AXIS_LABELS{ax}, V1.ratio_paper_mean(ax), V1.ratio_paper_se(ax), ...
                V1.ratio_eff_mean(ax), V1.ratio_eff_se(ax)); %#ok<AGROW>
        end
        if ~V2.pass_paper(ax) && ~V2.pass_eff(ax)
            lines{end+1} = sprintf( ...
                ['V2 fails on axis %s (slope ratio paper=%.3f, eff=%.3f, R^2=%.3f). ' ...
                 'Linearity in a_cov broken; suspect IF_var or (a_true+xi)^2.'], ...
                AXIS_LABELS{ax}, V2.slope_ratio_paper(ax), V2.slope_ratio_eff(ax), V2.R2(ax)); %#ok<AGROW>
        end
        if informative_v3(ax) && (~V3.pass_paper(ax) && ~V3.pass_eff(ax))
            lines{end+1} = sprintf( ...
                ['V3 fails on axis %s. Suspect xi(sigma2_n) scaling — ' ...
                 'check linearity of xi in sigma2_n (Phase 6 derivation).'], ...
                AXIS_LABELS{ax}); %#ok<AGROW>
        end
        if ~V4.pass(ax)
            lines{end+1} = sprintf( ...
                ['V4 fails on axis %s (max ACF dev=%.3f, IF_var ratio=%.3f). ' ...
                 'Inspect MA(2) closed-form coeffs vs lambda_c=%.2g.'], ...
                AXIS_LABELS{ax}, V4.max_dev_2_51(ax), V4.IF_var_ratio(ax), 0); %#ok<AGROW>
        end
        if ~V5.pass(ax)
            lines{end+1} = sprintf( ...
                ['V5 fails on axis %s (DC bias=%.2f%%). ' ...
                 'Inspect a_xm formula constant terms (C_n*sigma2_n_s subtraction).'], ...
                AXIS_LABELS{ax}, V5.bias_pct(ax)); %#ok<AGROW>
        end
    end
    if isempty(lines)
        reasons = 'Path C triggered but no per-axis failure isolated; review aggregate criteria.';
    else
        reasons = strjoin(lines, sprintf('\n  - '));
        reasons = sprintf('  - %s', reasons);
    end
end

function write_report_md(report, eq17_dir)
    out_path = fullfile(eq17_dir, 'phase9_validation_report.md');
    fid = fopen(out_path, 'w');
    if fid < 0
        warning('analyze_R22_validation:cannotWrite', ...
                'Could not open %s for writing.', out_path);
        return;
    end
    cleanup = onCleanup(@() fclose(fid));

    fprintf(fid, '# Phase 9 Stage 0: R(2,2) Validation Report\n\n');
    fprintf(fid, 'Generated: %s\n\n', report.timestamp);

    if isfield(report, 'sanity')
        fprintf(fid, '## Sanity Gates (V1 seed 1)\n\n');
        fprintf(fid, '| Gate | Result |\n|---|---|\n');
        fprintf(fid, '| max\\|delta_a_hat\\| < 1e-30 | %s |\n', tickmark(report.sanity.delta_a_hat_zero));
        fprintf(fid, '| range(a_hat steady) < 1e-30 | %s |\n', tickmark(report.sanity.a_hat_frozen));
        fprintf(fid, '| Q77 == 0 everywhere | %s |\n', tickmark(report.sanity.Q77_zero));
        fprintf(fid, '| guards inactive in steady state | %s |\n', tickmark(report.sanity.guards_inactive));
        fprintf(fid, '\n');
    end

    if strcmp(report.path, 'HALT')
        fprintf(fid, '## HALT — sanity gate failure\n\n');
        fprintf(fid, '%s\n', report.path_reason);
        return;
    end

    fprintf(fid, '## Constants\n\n');
    fprintf(fid, '- Ts = %g s, N_skip = %d (t_warmup = 2.0 s)\n', report.Ts, report.N_skip);
    fprintf(fid, '- IF_var (predicted) = %.4f\n', report.IF_var);
    fprintf(fid, '- a_true per axis [x, y, z] = [%.4e, %.4e, %.4e] um/pN\n', ...
            report.a_true_per_axis(1), report.a_true_per_axis(2), report.a_true_per_axis(3));
    fprintf(fid, '- xi_paper per axis [x, y, z] = [%.3e, %.3e, %.3e]\n', ...
            report.xi_paper(1), report.xi_paper(2), report.xi_paper(3));
    fprintf(fid, '\n');

    fprintf(fid, '## Phase 9 outcomes\n\n');
    fprintf(fid, ['Phase 9 Stage 0 has four possible verdicts on the ' ...
                  'paper-vs-eff comparison (V6):\n\n']);
    fprintf(fid, '1. **paper wins** — paper R(2,2) prediction matches V1 closer than eff on informative axes.\n');
    fprintf(fid, '2. **eff wins** — per-axis effective C_dpmr/C_n prediction matches V1 closer than paper.\n');
    fprintf(fid, ['3. **INDISTINGUISHABLE** — at h=50 production conditions, ' ...
                  'xi_eff/xi_paper ratio is ~1.005-1.008 across all 3 axes ' ...
                  '(per Agent X audit), so paper and eff predictions differ ' ...
                  'by less than 1%% in R(2,2). |dist_paper - dist_eff| sits ' ...
                  'below the empirical noise floor and V6 cannot pick a ' ...
                  'winner. This is the **expected** Wave 1 outcome at h=50.\n']);
    fprintf(fid, '4. **mixed** — informative axes disagree (rare).\n\n');
    fprintf(fid, ['When V6 is INDISTINGUISHABLE and V1-V5 all pass, Path A ' ...
                  'still applies. The Wave 4 v3 a_hat bias is therefore NOT ' ...
                  'explained by the paper-vs-eff xi choice and a separate ' ...
                  'mechanism must be investigated. Reference: ' ...
                  '`reference/eq17_analysis/phase9_predictions_audit.md`.\n\n']);

    write_v1_table(fid, report.V1, report.axes);
    write_v2_table(fid, report.V2, report.axes);
    write_v3_table(fid, report.V3, report.axes);
    write_v4_table(fid, report.V4, report.axes);
    write_v5_table(fid, report.V5, report.axes);
    write_v6_table(fid, report.V6, report.axes);

    fprintf(fid, '## Path Verdict: %s\n\n', report.path);
    fprintf(fid, '%s\n\n', report.path_reason);

    if strcmp(report.path, 'B')
        fprintf(fid, '### Suggested fix (Path B)\n\n');
        fprintf(fid, ['Per-axis effective C_dpmr/C_n predictions match empirical ' ...
                      'variance whereas paper closed-form does not. Update R(2,2) ' ...
                      'assembly in motion_control_law_eq17_7state.m to use ' ...
                      'C_dpmr_eff_per_axis / C_n_eff_per_axis (the per-axis ' ...
                      'lookup-derived constants) when computing the y_2 ' ...
                      'measurement variance. See Phase 6 §4 / Stage 11 Option I.\n\n']);
    elseif strcmp(report.path, 'C')
        fprintf(fid, '### Diagnostic notes (Path C)\n\nRecheck:\n');
        fprintf(fid, '- Phase 6 R(2,2) closed-form derivation\n');
        fprintf(fid, '- IF_var theoretical vs empirical\n');
        fprintf(fid, '- xi_paper / xi_eff sign and scaling with sigma2_n\n');
        fprintf(fid, '- a_xm formula constants (C_n*sigma2_n_s subtraction)\n\n');
    end
end

function write_v1_table(fid, V1, AX)
    fprintf(fid, '## V1 — Variance ratio (5 seeds)\n\n');
    fprintf(fid, ['| Axis | emp_var mean | ratio_paper mean +/- SE | pass paper | ' ...
                  'ratio_eff mean +/- SE | pass eff |\n']);
    fprintf(fid, '|---|---|---|---|---|---|\n');
    for ax = 1:3
        emp_var_mean = mean(V1.emp_var_per_seed(ax, :));
        fprintf(fid, '| %s | %.4e | %.3f +/- %.3f | %s | %.3f +/- %.3f | %s |\n', ...
            AX{ax}, emp_var_mean, ...
            V1.ratio_paper_mean(ax), V1.ratio_paper_se(ax), tickmark(V1.pass_paper(ax)), ...
            V1.ratio_eff_mean(ax),   V1.ratio_eff_se(ax),   tickmark(V1.pass_eff(ax)));
    end
    fprintf(fid, '\n');
end

function write_v2_table(fid, V2, AX)
    fprintf(fid, '## V2 — a_cov linearity\n\n');
    fprintf(fid, '| Axis | slope | pred slope (paper) | slope/pred (paper) | pred slope (eff) | slope/pred (eff) | R^2 | pass paper | pass eff |\n');
    fprintf(fid, '|---|---|---|---|---|---|---|---|---|\n');
    for ax = 1:3
        fprintf(fid, '| %s | %.3e | %.3e | %.3f | %.3e | %.3f | %.4f | %s | %s |\n', ...
            AX{ax}, V2.slope(ax), ...
            V2.pred_slope_paper(ax), V2.slope_ratio_paper(ax), ...
            V2.pred_slope_eff(ax),   V2.slope_ratio_eff(ax), ...
            V2.R2(ax), tickmark(V2.pass_paper(ax)), tickmark(V2.pass_eff(ax)));
    end
    fprintf(fid, '\n');
end

function write_v3_table(fid, V3, AX)
    fprintf(fid, '## V3 — sigma2_n factor sweep\n\n');
    fprintf(fid, 'Factors: 0.5, 1, 2, 4. Note: y axis xi_y ~ 0 -> V3 weakly informative on y.\n\n');
    fprintf(fid, '| Axis | informative | max\\|ratio_paper-1\\| | max\\|ratio_eff-1\\| | pass paper | pass eff |\n');
    fprintf(fid, '|---|---|---|---|---|---|\n');
    for ax = 1:3
        fprintf(fid, '| %s | %s | %.3f | %.3f | %s | %s |\n', ...
            AX{ax}, tickmark(V3.informative(ax)), ...
            max(abs(V3.ratio_paper(:, ax) - 1)), ...
            max(abs(V3.ratio_eff(:,   ax) - 1)), ...
            tickmark(V3.pass_paper(ax)), tickmark(V3.pass_eff(ax)));
    end
    fprintf(fid, '\n');
end

function write_v4_table(fid, V4, AX)
    fprintf(fid, '## V4 — ACF (lags 0-50, dx_r, V1 seed 1)\n\n');
    fprintf(fid, '| Axis | max\\|ACF dev\\| (lags 2-51) | IF_var emp / IF_var pred | pass |\n');
    fprintf(fid, '|---|---|---|---|\n');
    for ax = 1:3
        fprintf(fid, '| %s | %.3f | %.3f | %s |\n', ...
            AX{ax}, V4.max_dev_2_51(ax), V4.IF_var_ratio(ax), tickmark(V4.pass(ax)));
    end
    fprintf(fid, '\n');
end

function write_v5_table(fid, V5, AX)
    fprintf(fid, '## V5 — DC bias (5-seed aggregate)\n\n');
    fprintf(fid, '| Axis | mean(a_xm) | a_true | bias %% | pass |\n');
    fprintf(fid, '|---|---|---|---|---|\n');
    for ax = 1:3
        a_true_ax = V5.mean_per_axis(ax) / (1 + V5.bias_pct(ax)/100);
        fprintf(fid, '| %s | %.4e | %.4e | %.2f | %s |\n', ...
            AX{ax}, V5.mean_per_axis(ax), a_true_ax, V5.bias_pct(ax), tickmark(V5.pass(ax)));
    end
    fprintf(fid, '\n');
end

function write_v6_table(fid, V6, AX)
    fprintf(fid, '## V6 — paper vs eff (informative on x, z)\n\n');
    fprintf(fid, '| Axis | informative | dist_paper | dist_eff | |dist_paper-dist_eff| | verdict |\n');
    fprintf(fid, '|---|---|---|---|---|---|\n');
    for ax = 1:3
        gap = abs(V6.dist_paper(ax) - V6.dist_eff(ax));
        fprintf(fid, '| %s | %s | %.3f | %.3f | %.3f | %s |\n', ...
            AX{ax}, tickmark(~V6.uninformative_axes(ax)), ...
            V6.dist_paper(ax), V6.dist_eff(ax), gap, V6.closer_to_one{ax});
    end
    if V6.indistinguishable_overall
        verdict = sprintf(['INDISTINGUISHABLE — paper/eff difference below ' ...
                           'noise floor (|dist_paper - dist_eff| <= %.3f) on ' ...
                           'all informative axes; formula choice has no ' ...
                           'measurable impact at this operating point'], ...
                           V6.indistinguishable_tol);
    elseif V6.eff_wins_overall
        verdict = 'eff wins on informative axes (x, z)';
    elseif V6.paper_wins_overall
        verdict = 'paper wins on informative axes (x, z)';
    else
        verdict = 'mixed across informative axes';
    end
    fprintf(fid, '\nAggregate verdict: %s.\n', verdict);
    fprintf(fid, ['\nNote: y-axis xi_y is structurally near zero, so paper ' ...
                  'and eff predictions are nearly identical for y; y is ' ...
                  'excluded from V6 verdict.\n\n']);
    fprintf(fid, ['Reference: Agent X predictions audit at ' ...
                  '`reference/eq17_analysis/phase9_predictions_audit.md` ' ...
                  '(xi_eff/xi_paper ~ 1.005-1.008 at h=50 production ' ...
                  'conditions, predicting V6 INDISTINGUISHABLE).\n\n']);
end

function s = tickmark(flag)
    if flag
        s = 'PASS';
    else
        s = 'FAIL';
    end
end
