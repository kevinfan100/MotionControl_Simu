function res = analyze_formB_info_7b_vs_7a(opts)
%ANALYZE_FORMB_INFO_7B_VS_7A  Fisher information on the shape constant b:
%   gain-as-state (7b, production) vs algebraic gain (7a) writing.
%
% STATUS: ACTIVE -- established that the 7a algebraic rewrite buys nothing: information
%   ratio 0.997 +- 0.037, guaranteed by the mixed-partial identity.
%   See memory project-formb-7a-falsified-quadrature-ratchet-2026-08-06.
%
% PURPOSE: answer "how many times more information about b does one trajectory
%   give under the 7a (algebraic) writing than under the 7b (gain-as-state)
%   writing?".  The 7b y2 row only sees b instantaneously through
%   -Grad_wbar_d * J_b (small, and exactly zero during hold), but b ALSO enters
%   through the predict chain a_bar[k+1] = a_bar[k] + a_bar'(b,p,ws)*M[k], which
%   accumulates over thousands of steps.  Comparing instantaneous sensitivities
%   alone is therefore not decisive; this script accumulates the exact
%   sensitivity along the realized canonical trajectory.
%
% SCOPE: OFFLINE ONLY.  Nothing here is a filter, nothing modifies production.
%   The canonical run is produced by the production driver run_formB_ws; a thin
%   fork of its per-seed loop (local_run_extended) re-runs the SAME scenario
%   only to capture two diag rows the driver does not log (delta_x_hat_3 and
%   Delta_h_d / Delta_H_d), and is bit-checked against the driver output.
%
%   Sensitivity definitions (both writings, same trajectory, same R2, same y2
%   gate).  d = 2, H2_scale = a_cov (y2_whiten), echo = 1 - S (y2_echo_corr):
%
%     7a  ("a_bar is a function of theta"):
%         dy2/db = H2_scale * ( dA_db - echo*Grad_wbar_d*J_b )
%         dA_db  = -(g/b)*a_bar' = -p*g*b^(p-1)*(g+b)^(-p-1),  g = w_bar_d - w_s
%
%     7b  ("a_bar is a state"):  s[k] = d a_bar[k] / d b propagated OPEN LOOP
%         through the predict chain (Ljung sensitivity equations; the
%         estimator's own feedback is deliberately excluded -- we are asking
%         how the DATA depend on b, not how the filter reacts):
%         s[k]   = s[k-1] + J_b[k]*M[k] ,  M[k] = Dwbar_d[k-1] + (1-lc)*dw3[k-1]
%         s[0]   = dA_db at the INIT height/seed (a_bar[0] is computed BY the
%                  law from the (b,p,ws) seeds, so it is NOT zero)
%         dy2/db = H2_scale * ( s[k] - echo*Grad_wbar_d*J_b )
%
%     I_b = sum_k (dy2/db)^2 / R2[k]  over the steps where the y2 gate is open.
%
% CRITERION (set by the requester before running):
%     I_b(7a)/I_b(7b) <  3   -> rewriting the controller is NOT worth it
%                      > 100 -> clearly worth it
%     in between            -> undecided, needs a separate argument
%
% RESULT (2026-08-06, canonical t1_y2on scenario, z axis):
%     ratio = 0.997 +- 0.037 over seeds [7 11 23 42 101]  ->  NOT WORTH IT.
%     Reason: equality of mixed partials.  d/dw_bar (dA_db) = d/db (a_bar') = J_b
%     is exactly the increment the 7b predict adds, and a_bar[0] is seeded BY the
%     law, so s[k] integrates to the same curve 7a evaluates in closed form
%     (sup|s/dA_db - 1| = 3.0%, and 0.97% with the dw3 term switched off).  The
%     instantaneous -Grad*J_b term carries 2e-5 % of I_b in BOTH writings; the
%     ~1350x instantaneous ratio is therefore not the information ratio.
%
% opts (all optional):
%     .seed        7      canonical house seed; full report + figure
%     .seeds       []      extra seeds -> compact ratio table (sampling check)
%     .save_fig    true    write test_results/temp_formB_info_7b_vs_7a.png
%     .use_dw3     true    include the (1-lc)*dw3 term in M (false = command
%                          step only; robustness arm)
%
%   See also: motion_control_law_formB_ws, run_formB_ws

    if nargin < 1 || isempty(opts); opts = struct(); end
    if ~isfield(opts, 'seed');     opts.seed = 7;        end
    if ~isfield(opts, 'seeds');    opts.seeds = [];      end
    if ~isfield(opts, 'save_fig'); opts.save_fig = true; end
    if ~isfield(opts, 'use_dw3');  opts.use_dw3 = true;  end

    AX_Z      = 3;        % wall-normal axis: the only one running the perp law
    FIG_NAME  = 'temp_formB_info_7b_vs_7a.png';

    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));

    % ------------------------------------------------------------------
    % [1] Canonical run through the production driver (cfg + full ctrl_const)
    % ------------------------------------------------------------------
    base = run_formB_ws(struct('seeds', opts.seed));
    cfg        = base.cfg;
    ref        = base.runs{1};
    ctrl_const = ref.ctrl_const;          % FINAL merged constants the filter saw

    % ------------------------------------------------------------------
    % [2] Extended-logging fork of the same run; bit-check against [1]
    % ------------------------------------------------------------------
    ext = local_run_extended(cfg, opts.seed, ctrl_const);
    d_abar = max(abs(ext.a_bar_hat_out(:) - ref.a_bar_hat_out(:)));
    d_b    = max(abs(ext.b_hat_out(:)     - ref.b_hat_out(:)));
    d_R2   = max(abs(ext.R2_out(:)        - ref.R2_out(:)));
    fprintf('\n[fork check] max|dev| vs driver: a_bar %.3e  b %.3e  R2 %.3e\n', ...
            d_abar, d_b, d_R2);
    assert(d_abar == 0 && d_b == 0 && d_R2 == 0, ...
           'analyze_formB_info_7b_vs_7a:forkMismatch', ...
           'extended-logging fork is not bit-identical to run_formB_ws.');

    % ------------------------------------------------------------------
    % [3] Constants needed to rebuild the law Jacobians / echo factor
    % ------------------------------------------------------------------
    P        = ref.meta.params_value;
    R_radius = P.common.R;                                 % [um]
    Ts       = P.common.Ts;                                % [s]
    a_o      = Ts / (P.ctrl.gamma * R_radius);             % [1/pN]
    kappa_T  = 4 * (P.ctrl.k_B * P.ctrl.T / R_radius) * a_o;   % [-]
    s2n_nd   = P.ctrl.sigma2_noise(:) / R_radius^2;        % [-]
    lambda_c = ctrl_const.lambda_c;
    a_cov    = ctrl_const.a_cov;
    a_pd     = ctrl_const.a_pd;
    xi_bar   = (ctrl_const.C_n / ctrl_const.C_dpmr) * s2n_nd / kappa_T;

    b_clamp     = local_field(ctrl_const, 'b_clamp',     [0.05, 5]);
    p_clamp     = local_field(ctrl_const, 'p_clamp',     [0.05, 5]);
    a_bar_floor = local_field(ctrl_const, 'a_bar_floor', 0.05);
    ws_margin   = local_field(ctrl_const, 'ws_margin',   1e-3);
    gap_floor   = ws_margin;
    ws0_perp    = local_field(ctrl_const, 'ws0_perp',    1);
    y2_whiten   = local_field(ctrl_const, 'y2_whiten',   true);
    if y2_whiten; H2_scale = a_cov; else; H2_scale = 1; end

    % Echo shares from the exact mismatched-loop Lyapunov (same construction as
    % the controller init; deterministic, depends only on lambda_c and a_pd).
    if local_field(ctrl_const, 'y2_echo_corr', true)
        [S_echo_T, S_echo_n] = local_echo_shares(lambda_c, a_pd);
    else
        S_echo_T = 0; S_echo_n = 0;
    end

    % ------------------------------------------------------------------
    % [4] Per-step sensitivities on the wall-normal axis
    % ------------------------------------------------------------------
    N  = numel(ext.tout);
    t  = ext.tout;

    % PRIOR-side quantities (what x_curr held when step k ran) = row k-1.
    b_prior   = [NaN; ext.b_hat_out(1:end-1,  AX_Z)];
    p_prior   = [NaN; ext.p_hat_out(1:end-1,  AX_Z)];
    ws_prior  = [NaN; ext.ws_hat_out(1:end-1, AX_Z)];
    ab_prior  = [NaN; ext.a_bar_hat_out(1:end-1, AX_Z)];
    dw3_prior = [0;   ext.dw3_out(1:end-1,    AX_Z)];      % dw_3 posterior[k-1]
    dwd_prev  = [0;   ext.Delta_wbar_d_out(1:end-1)];      % Delta_wbar_d[k-1]

    grad_wd = ext.Grad_wbar_d_out;      % Grad_wbar_d[k]
    w_bar_d = ext.h_bar_d_out;          % w_bar_d[k]
    R2      = ext.R2_out(:, AX_Z);
    open_y2 = ~ext.gate_out(:, AX_Z);
    open_y2(1) = false;                 % row 1 is the controller init call

    a_prime = zeros(N, 1);  J_b = zeros(N, 1);  dA_db = zeros(N, 1);
    echo    = ones(N, 1);   gap_v = zeros(N, 1);
    for k = 2:N
        law_b  = min(max(b_prior(k), b_clamp(1)), b_clamp(2));
        law_p  = min(max(p_prior(k), p_clamp(1)), p_clamp(2));
        law_ws = ws_prior(k) + (ws0_perp - 1);
        gap    = max(w_bar_d(k) - law_ws, gap_floor);
        gap_v(k) = gap;
        [ap, jb, dadb] = local_law_jacobians(gap, law_b, law_p);
        a_prime(k) = ap;  J_b(k) = jb;  dA_db(k) = dadb;

        a_bar_i  = max(ab_prior(k), a_bar_floor);
        S_i      = (S_echo_T * a_bar_i + S_echo_n * xi_bar(AX_Z)) / (a_bar_i + xi_bar(AX_Z));
        echo(k)  = 1 - S_i;
    end
    % Rebuilt-vs-logged a_bar' check: proves the law re-implementation and the
    % prior-row alignment are exactly the controller's.
    d_ap = max(abs(a_prime(2:end) - ext.a_prime_bar_out(2:end, AX_Z)));
    fprintf('[law check] max|a_bar''_rebuilt - a_bar''_logged| = %.3e\n', d_ap);
    assert(d_ap < 1e-14, 'analyze_formB_info_7b_vs_7a:lawMismatch', ...
           'rebuilt a_bar'' does not match the controller log.');

    % --- 7b accumulation: s[k] = s[k-1] + J_b[k]*M[k], s[0] from the init law
    if opts.use_dw3
        M_mult = dwd_prev + (1 - lambda_c) * dw3_prior;
    else
        M_mult = dwd_prev;
    end
    w_bar_seed = (dot(P.common.p0(:), P.wall.w_hat(:)) - P.wall.pz) / R_radius;
    b_seed  = ext.b_hat_out(1,  AX_Z);
    p_seed  = ext.p_hat_out(1,  AX_Z);
    ws_seed = ext.ws_hat_out(1, AX_Z) + (ws0_perp - 1);
    [~, ~, s0] = local_law_jacobians(max(w_bar_seed - ws_seed, gap_floor), b_seed, p_seed);

    s_acc = zeros(N, 1);
    s_acc(1) = s0;
    for k = 2:N
        s_acc(k) = s_acc(k-1) + J_b(k) * M_mult(k);
    end

    % --- sensitivities of the PREDICTED measurement
    D_inst = -H2_scale * echo .* grad_wd .* J_b;      % shared by both writings
    D_acc  =  H2_scale * s_acc;                       % 7b accumulated path
    D_lvl  =  H2_scale * dA_db;                       % 7a direct level path
    D_7b   =  D_acc + D_inst;
    D_7a   =  D_lvl + D_inst;

    inv_R2 = zeros(N, 1);
    inv_R2(open_y2) = 1 ./ R2(open_y2);

    i_7a = D_7a.^2 .* inv_R2;
    i_7b = D_7b.^2 .* inv_R2;
    i_acc  = D_acc.^2  .* inv_R2;
    i_inst = D_inst.^2 .* inv_R2;
    i_cross = 2 * D_acc .* D_inst .* inv_R2;

    I_7a = sum(i_7a);  I_7b = sum(i_7b);
    ratio = I_7a / I_7b;

    % ------------------------------------------------------------------
    % [5] Report
    % ------------------------------------------------------------------
    win = local_windows(cfg);
    names = fieldnames(win);

    fprintf('\n================ Fisher information on b (z axis, seed %d) ================\n', opts.seed);
    fprintf('H2_scale = %.4g (y2_whiten=%d)   echo factor range [%.4f %.4f]\n', ...
            H2_scale, y2_whiten, min(echo(2:end)), max(echo(2:end)));
    fprintf('steps with y2 open: %d / %d (%.1f%%)\n', sum(open_y2), N-1, ...
            100 * sum(open_y2) / (N-1));
    fprintf('\nTOTAL   I_b(7a) = %.6g    I_b(7b) = %.6g    ratio 7a/7b = %.4f\n', ...
            I_7a, I_7b, ratio);
    if ratio < 3
        verdict = 'NOT WORTH REWRITING (ratio < 3)';
    elseif ratio > 100
        verdict = 'CLEARLY WORTH REWRITING (ratio > 100)';
    else
        verdict = 'UNDECIDED (3 <= ratio <= 100)';
    end
    fprintf('VERDICT: %s\n', verdict);

    fprintf('\n--- per-segment breakdown -------------------------------------------------\n');
    fprintf('%-12s %8s %8s | %12s %12s %8s | %10s\n', ...
            'segment', 't0[s]', 't1[s]', 'I_b(7a)', 'I_b(7b)', 'ratio', 'share7b%%');
    seg = struct();
    for j = 1:numel(names)
        wj = win.(names{j});
        m  = t > wj(1) & t <= wj(2);
        Ia = sum(i_7a(m)); Ib = sum(i_7b(m));
        seg.(names{j}) = [Ia, Ib];
        fprintf('%-12s %8.2f %8.2f | %12.6g %12.6g %8.3f | %10.2f\n', ...
                names{j}, wj(1), wj(2), Ia, Ib, Ia / max(Ib, eps), 100 * Ib / I_7b);
    end

    fprintf('\n--- 7b decomposition (accumulated vs instantaneous) -----------------------\n');
    fprintf('I_acc  = %.6g  (%.2f%% of I_7b)\n', sum(i_acc),  100 * sum(i_acc)  / I_7b);
    fprintf('I_inst = %.6g  (%.4f%% of I_7b)\n', sum(i_inst), 100 * sum(i_inst) / I_7b);
    fprintf('I_cross= %+.6g (%.4f%% of I_7b)\n', sum(i_cross), 100 * sum(i_cross) / I_7b);
    fprintf('per-step |D| medians: 7a %.4g   7b %.4g   inst-only %.4g   (ratio 7a/inst %.4g)\n', ...
            median(abs(D_7a(open_y2))), median(abs(D_7b(open_y2))), ...
            median(abs(D_inst(open_y2))), ...
            median(abs(D_7a(open_y2))) / max(median(abs(D_inst(open_y2))), eps));

    fprintf('\n--- does the 7b chain reproduce the 7a level derivative? -----------------\n');
    m = open_y2;
    fprintf('max|s - dA_db| = %.4g   rms = %.4g   (|dA_db| range [%.4g %.4g])\n', ...
            max(abs(s_acc(m) - dA_db(m))), rms(s_acc(m) - dA_db(m)), ...
            min(abs(dA_db(m))), max(abs(dA_db(m))));
    fprintf('relative gap sup|s/dA_db - 1| = %.4f%%\n', ...
            100 * max(abs(s_acc(m) ./ dA_db(m) - 1)));

    % Size of the term this analysis omits: the MA(2) memory feedthrough
    % a_bar' * alpha * (m1 + m2) in the predict, whose b-derivative is
    % J_b*alpha*(m1+m2).  m1/m2 are zero-mean thermal-memory estimates, so the
    % term random-walks; bound it with the TRUE thermal step scale (an upper
    % bound, the filter's estimate is shrunk toward zero).
    alpha_ma2 = 1 - lambda_c;
    m_scale   = sqrt(kappa_T * max(ab_prior(2:end)));           % [-] per-step
    rw_bound  = alpha_ma2 * m_scale * sqrt(sum(J_b(2:end).^2)); % random-walk sd
    fprintf('\nomitted MA(2)-memory term: random-walk sd on s <= %.3g  (vs |s| ~ %.3g)\n', ...
            rw_bound, median(abs(s_acc(m))));

    % R2 comparability: the only piece of R2 that exists ONLY because a_bar is a
    % state is the a_cov^2*d*Q44 delay term.
    Q44 = ext.Q44_out(:, AX_Z);
    d_delay = ctrl_const.d;
    r2_state_share = a_cov^2 * d_delay * Q44 ./ max(R2, eps);
    fprintf('R2 share that exists only because a_bar is a state (a_cov^2*d*Q44/R2): max %.3g%% mean %.3g%%\n', ...
            100 * max(r2_state_share(m)), 100 * mean(r2_state_share(m)));

    % ------------------------------------------------------------------
    % [5b] Is this information worth anything against the PRIOR?
    %      1/I_b is the y2-channel CRLB on b from one run; compare with the
    %      envelope prior sqrt(P_bb[0]) the controller starts from, and check
    %      the predicted variance reduction against the filter's realized one.
    % ------------------------------------------------------------------
    Pb0 = ref.P_b_out(1,   AX_Z)^2;    % P_*_out store SQRT of the variance
    Pbe = ref.P_b_out(end, AX_Z)^2;
    % Echo convention.  y2_pred keeps the FULL a_bar_hat term (the echo share S
    % of the reading follows the APPLIED gain, which is the estimate), so the
    % level term above carries no (1-S).  The controller's H2 row, which is the
    % sensitivity to the estimation ERROR, does carry it.  Under that second
    % convention the (1-S) is a near-constant common factor on both writings:
    % reported here so the ratio can be seen not to move.
    D_7a_e = H2_scale * echo .* (dA_db - grad_wd .* J_b);
    D_7b_e = H2_scale * echo .* (s_acc  - grad_wd .* J_b);
    I_7a_e = sum(D_7a_e.^2 .* inv_R2);
    I_7b_e = sum(D_7b_e.^2 .* inv_R2);
    fprintf('\nalt. echo convention (1-S on the LEVEL term too): I_7a %.4g  I_7b %.4g  ratio %.4f\n', ...
            I_7a_e, I_7b_e, I_7a_e / I_7b_e);

    fprintf('\n--- information vs prior --------------------------------------------------\n');
    fprintf('prior 1/P_bb[0]   = %.4g   (sqrt P = %.5f)\n', 1/Pb0, sqrt(Pb0));
    fprintf('data  I_b(7b)     = %.4g   -> y2-channel CRLB sigma_b = %.5f (%.1fx the prior width)\n', ...
            I_7b, 1/sqrt(I_7b), (1/sqrt(I_7b)) / sqrt(Pb0));
    fprintf('predicted P_bb shrink: 7b %.2f%%   7a %.2f%%   |  filter realized %.2f%%\n', ...
            100 * I_7b / (1/Pb0 + I_7b), 100 * I_7a / (1/Pb0 + I_7a), ...
            100 * (1 - Pbe/Pb0));

    % ------------------------------------------------------------------
    % [5b'] The OTHER axis on which the two writings differ, which the Fisher
    %       ratio does not price: in 7b the gain STATE is never re-anchored on
    %       (b, p, w_s) after init, so it can sit at a level its own parameters
    %       do not produce.  7a cannot: its gain IS the law value by definition.
    %       This is the third amplification condition of the defect-3 note
    %       ("a_bar never re-anchored"), measured here.
    % ------------------------------------------------------------------
    g_law   = max(w_bar_d - ext.ws_hat_out(:, AX_Z), gap_floor);
    a_law   = 1 - (1 + g_law ./ ext.b_hat_out(:, AX_Z)).^(-ext.p_hat_out(:, AX_Z));
    a_state = ext.a_bar_hat_out(:, AX_Z);
    rel_law = 100 * (a_state - a_law) ./ a_law;
    a_true_bar = ext.a_true_out(:, AX_Z) / (P.common.Ts / P.common.gamma_N);
    fprintf('\n--- 7b gain state vs its OWN law value (7a would be 0 by construction) ---\n');
    fprintf('whole run: mean %+.3f%%  rms %.3f%%  max|.| %.3f%%   (a_hat vs a_true rms = %.3f%%)\n', ...
            mean(rel_law(2:end)), rms(rel_law(2:end)), max(abs(rel_law(2:end))), ...
            rms(100 * (a_state(2:end) - a_true_bar(2:end)) ./ a_true_bar(2:end)));
    for j = 1:numel(names)
        wj = win.(names{j});
        mj = t > wj(1) & t <= wj(2);
        fprintf('  %-12s mean %+.3f%%  rms %.3f%%\n', names{j}, mean(rel_law(mj)), rms(rel_law(mj)));
    end

    % ------------------------------------------------------------------
    % [5c] Sampling check: the ratio is a per-realization number (the dw3 term
    %      in M is noise-driven), so it needs more than one seed to read.
    % ------------------------------------------------------------------
    sweep = [];
    if ~isempty(opts.seeds)
        sw_seeds = opts.seeds(:).';
        sweep = zeros(numel(sw_seeds), 3);
        for j = 1:numel(sw_seeds)
            sub = struct('seed', sw_seeds(j), 'seeds', [], 'save_fig', false, ...
                         'use_dw3', opts.use_dw3);                     %#ok<NASGU>
            % evalc keeps the per-seed driver banner out of the table
            [~, rj] = evalc('analyze_formB_info_7b_vs_7a(sub)');
            sweep(j, :) = [rj.I_7a, rj.I_7b, rj.ratio];
        end
        fprintf('\n--- seed sweep ------------------------------------------------------------\n');
        fprintf('%6s %12s %12s %8s\n', 'seed', 'I_b(7a)', 'I_b(7b)', 'ratio');
        fprintf('%6d %12.4f %12.4f %8.4f\n', opts.seed, I_7a, I_7b, ratio);
        for j = 1:numel(sw_seeds)
            fprintf('%6d %12.4f %12.4f %8.4f\n', sw_seeds(j), sweep(j, 1), sweep(j, 2), sweep(j, 3));
        end
        all_r = [ratio; sweep(:, 3)];
        fprintf('ratio over %d seeds: mean %.4f  sd %.4f  [min %.4f, max %.4f]\n', ...
                numel(all_r), mean(all_r), std(all_r), min(all_r), max(all_r));
    end

    % ------------------------------------------------------------------
    % [6] Figure
    % ------------------------------------------------------------------
    res = struct('I_7a', I_7a, 'I_7b', I_7b, 'ratio', ratio, 'verdict', verdict, ...
                 'seg', seg, 'I_acc', sum(i_acc), 'I_inst', sum(i_inst), ...
                 'I_cross', sum(i_cross), 't', t, 'D_7a', D_7a, 'D_7b', D_7b, ...
                 'D_inst', D_inst, ...
                 's_acc', s_acc, 'dA_db', dA_db, 'i_7a', i_7a, 'i_7b', i_7b, ...
                 'open_y2', open_y2, 'cfg', cfg, 'seed', opts.seed, ...
                 'sweep', sweep, 'P_bb_prior', Pb0, 'P_bb_post', Pbe);

    if ~opts.save_fig; return; end
    out_dir = fullfile(root, 'test_results');
    if ~exist(out_dir, 'dir'); mkdir(out_dir); end
    local_make_figure(res, fullfile(out_dir, FIG_NAME));
    fprintf('\nsaved figure: %s\n', fullfile(out_dir, FIG_NAME));
end


%% =================== Local Helpers ===================

function [a_prime, J_b, dA_db] = local_law_jacobians(gap, b, p)
%LOCAL_LAW_JACOBIANS  Form B slope, slope-Jacobian and LEVEL Jacobian wrt b.
%   Mirrors local_gain_law_formB inside motion_control_law_formB_ws (S1/S2):
%       a_bar' = (p/b)*(1 + gap/b)^(-p-1)
%       J_b    = [-1/b + (p+1)*gap/(b*(gap+b))] * a_bar'      (d a_bar'/d b)
%       dA_db  = -(gap/b)*a_bar'                              (d a_bar  /d b)
    u       = 1 + gap / b;
    a_prime = (p / b) * u^(-p) / u;
    J_b     = (-1 / b + (p + 1) * gap / (b * (gap + b))) * a_prime;
    dA_db   = -(gap / b) * a_prime;
end


function [S_T, S_n] = local_echo_shares(lambda_c, a_pd)
%LOCAL_ECHO_SHARES  y2 self-echo sensitivities S_T / S_n (controller init copy).
%   Exact 6-state Lyapunov covariance of the mismatched hold loop, differentiated
%   log-log about a unit gain mismatch.  Zero tuning; d = 2 by construction.
    alE = 1 - lambda_c;
    epE = 1e-4;
    vE  = zeros(2, 3);
    gE_list = [1, 1/(1+epE), 1/(1-epE)];
    for iN = 1:2
        for iG = 1:3
            gE = gE_list(iG);
            AE = zeros(6); BqE = zeros(6,1); BnE = zeros(6,1);
            AE(1,1)=1; AE(1,3)=-gE*alE; AE(1,4)=-gE*alE; AE(1,5)=-gE*alE;
            BnE(1)=-gE*alE; BqE(1)=1;
            AE(2,1)=1; AE(3,2)=1;
            AE(4,3)=-alE; AE(4,4)=-alE; AE(4,5)=-alE; BnE(4)=-alE;
            AE(5,4)=1;
            AE(6,3)=a_pd; AE(6,6)=1-a_pd; BnE(6)=a_pd;
            if iN == 1; QE = BqE*BqE.'; extraE = 0;
            else;       QE = BnE*BnE.'; extraE = (1-a_pd)^2; end
            XE = reshape((eye(36) - kron(AE,AE)) \ QE(:), 6, 6);
            cE = zeros(1,6); cE(3) = 1-a_pd; cE(6) = -(1-a_pd);
            vE(iN,iG) = cE*XE*cE.' + extraE;
        end
    end
    S_T = (log(vE(1,2)) - log(vE(1,3))) / (2*epE);
    S_n = (log(vE(2,2)) - log(vE(2,3))) / (2*epE);
end


function v = local_field(s, name, default)
%LOCAL_FIELD  s.(name) if present and non-empty, else default.
    if isfield(s, name) && ~isempty(s.(name))
        v = s.(name);
    else
        v = default;
    end
end


function w = local_windows(cfg)
%LOCAL_WINDOWS  Phase windows [s]; the driver's settle margins are not applied
%   because every step carries information, settled or not.
    t1 = cfg.t_hold;
    t2 = t1 + cfg.t_descend_override;
    t3 = t2 + cfg.n_cycles / cfg.frequency;
    w = struct('hold_init', [0, t1], 'descent', [t1, t2], ...
               'oscillation', [t2, t3], 'hold_final', [t3, cfg.T_sim]);
end


function simOut = local_run_extended(config, seed, ctrl_const)
%LOCAL_RUN_EXTENDED  Thin fork of run_formB_ws/local_run_once that logs the two
%   diag rows the driver drops (delta_x_hat_3, Delta_h_d / Delta_H_d) plus Q44.
%   ctrl_const is passed in WHOLE (the driver's final merged struct), so no
%   prior derivation is duplicated here.  Bit-checked by the caller.
    clear motion_control_law_formB_ws trajectory_generator calc_thermal_force;
    rng(seed);

    params = calc_simulation_params(config);
    P = params.Value;

    Ts = P.common.Ts;
    N  = round(config.T_sim / Ts) + 1;
    tout = (0:N-1)' * Ts;

    p0 = P.common.p0;
    p_curr = p0;
    d_delay = ctrl_const.d;
    p_m_buffer = repmat(p0, 1, d_delay + 1);
    pd_for_ctrl = p0;

    R_drv     = P.common.R;
    a_nom_drv = P.common.Ts / P.common.gamma_N;             % [um/pN]
    wall_on   = isfield(P, 'wall') && P.wall.enable_wall_effect > 0.5;
    if wall_on && isfield(P.wall, 'h_bar_min')
        h_bar_floor = P.wall.h_bar_min;
    else
        h_bar_floor = 1.001;
    end

    a_bar_hat_out    = zeros(N, 3);
    a_prime_bar_out  = zeros(N, 3);
    dw3_out          = zeros(N, 3);   % [-] posterior dw_3 estimate
    b_hat_out        = zeros(N, 3);
    p_hat_out        = zeros(N, 3);
    ws_hat_out       = zeros(N, 3);
    R2_out           = zeros(N, 3);
    Q44_out          = zeros(N, 3);
    gate_out         = false(N, 3);
    a_true_out       = zeros(N, 3);
    h_bar_d_out      = zeros(N, 1);
    Delta_wbar_d_out = zeros(N, 1);   % [-] Delta_wbar_d[k]
    Grad_wbar_d_out  = zeros(N, 1);   % [-] Grad_wbar_d[k]

    for k = 1:N
        t_now = tout(k);
        [pd_kp1, del_pd_k] = trajectory_generator(t_now, P);
        pd_k = pd_for_ctrl;
        p_m_delayed = p_m_buffer(:, 1);

        if wall_on
            h_bar_true_k = max((dot(p_curr, P.wall.w_hat) - P.wall.pz) / R_drv, h_bar_floor);
            [c_para_k, c_perp_k] = calc_correction_functions(h_bar_true_k);
            a_true_k = [a_nom_drv / c_para_k; a_nom_drv / c_para_k; a_nom_drv / c_perp_k];
        else
            a_true_k = a_nom_drv * ones(3, 1);
        end

        [f_d_k, ~, diag_k] = motion_control_law_formB_ws(del_pd_k, pd_k, ...
                                 p_m_delayed, P, ctrl_const, []);

        if P.thermal.enable > 0.5
            f_th_k = calc_thermal_force(p_curr, P);
        else
            f_th_k = zeros(3, 1);
        end
        p_curr = step_dynamics(p_curr, f_d_k + f_th_k, P, Ts);

        if config.meas_noise_enable
            n_meas = config.meas_noise_std(:) .* randn(3, 1);
        else
            n_meas = zeros(3, 1);
        end
        p_m_buffer = [p_m_buffer(:, 2:end), p_curr + n_meas];

        a_bar_hat_out(k, :)   = diag_k.a_bar_hat(:).';
        a_prime_bar_out(k, :) = diag_k.a_prime_bar(:).';
        dw3_out(k, :)         = diag_k.delta_x_hat_3(:).' / R_drv;
        b_hat_out(k, :)       = diag_k.b_hat(:).';
        p_hat_out(k, :)       = diag_k.p_hat(:).';
        ws_hat_out(k, :)      = diag_k.ws_hat(:).';
        R2_out(k, :)          = diag_k.R2(:).';
        Q44_out(k, :)         = diag_k.Q77(:).';     % legacy alias for Q(4,4)
        gate_out(k, :)        = diag_k.gate_active_per_axis(:).';
        a_true_out(k, :)      = a_true_k.';
        h_bar_d_out(k)        = diag_k.h_bar_d;
        Delta_wbar_d_out(k)   = diag_k.Delta_h_d / R_drv;
        Grad_wbar_d_out(k)    = diag_k.Delta_H_d / R_drv;

        pd_for_ctrl = pd_kp1;
    end

    simOut = struct('tout', tout, 'a_bar_hat_out', a_bar_hat_out, ...
                    'a_prime_bar_out', a_prime_bar_out, 'dw3_out', dw3_out, ...
                    'b_hat_out', b_hat_out, 'p_hat_out', p_hat_out, ...
                    'ws_hat_out', ws_hat_out, 'R2_out', R2_out, ...
                    'Q44_out', Q44_out, 'gate_out', gate_out, ...
                    'a_true_out', a_true_out, 'h_bar_d_out', h_bar_d_out, ...
                    'Delta_wbar_d_out', Delta_wbar_d_out, ...
                    'Grad_wbar_d_out', Grad_wbar_d_out);
end


function local_make_figure(res, png_path)
%LOCAL_MAKE_FIGURE  Top: |dy2/db| per step, both writings.  Bottom: cumulative
%   Fisher information I_b(t).  House style (plot_var_ahat_6state template).
    COL_7A = [0.8 0 0];        % 7a (algebraic)  -- house "True" red
    COL_7B = [0 0.2 0.9];      % 7b (production) -- house "Estimate" blue
    COL_IN = [0.35 0.35 0.35]; % 7b instantaneous term alone
    FS = 18; LFS = 13; AXLW = 2.0;

    t = res.t;
    m = res.open_y2;
    d7a = abs(res.D_7a);   d7a(~m) = NaN;
    d7b = abs(res.D_7b);   d7b(~m) = NaN;
    din = abs(res.D_inst); din(~m) = NaN;

    f = figure('Position', [80 80 1100 760], 'Color', 'w', ...
               'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    nexttile; hold on;
    h1 = plot(t, d7a, '-', 'Color', COL_7A, 'LineWidth', 2.5, ...
              'DisplayName', '7a  algebraic gain');
    h2 = plot(t, d7b, '--', 'Color', COL_7B, 'LineWidth', 2.0, ...
              'DisplayName', '7b  gain-as-state');
    h3 = plot(t, din, '-', 'Color', COL_IN, 'LineWidth', 1.5, ...
              'DisplayName', '7b instantaneous term only');
    xlim([0 t(end)]);
    % floor the log axis: the instantaneous term goes exactly to zero whenever
    % the command holds, which would otherwise drag the axis down decades
    ylim([1e-8, 5e-2]);
    ylabel('|dy_2/db|', 'FontSize', FS, 'FontWeight', 'bold');
    legend([h1 h2 h3], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
           'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, ...
             'Box', 'on', 'YScale', 'log', 'YTick', 10.^(-8:2:-2));
    grid off;

    nexttile; hold on;
    plot(t, cumsum(res.i_7a), '-', 'Color', COL_7A, 'LineWidth', 2.0);
    plot(t, cumsum(res.i_7b), '-', 'Color', COL_7B, 'LineWidth', 2.0);
    xlim([0 t(end)]);
    ylabel('I_b(t)', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');
    grid off;

    exportgraphics(f, png_path, 'Resolution', 150);
    close(f);
end
