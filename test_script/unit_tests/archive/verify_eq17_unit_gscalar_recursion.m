function verify_eq17_unit_gscalar_recursion()
%VERIFY_EQ17_UNIT_GSCALAR_RECURSION  Unit tests for the pooled scalar-gain
%   information / weighted-least-squares recursion used by
%   motion_control_law_eq17_gscalar (steps [4]-[5]). The recursion is exercised
%   here in isolation on synthetic measurements a_xm = s_true*a_det + sqrt(R22)*n
%   (the simulation condition: a_x = g*m, a_det = a_nom*m, so a_x = s_true*a_det
%   exactly). R22 = K_var*IF*(a_det+xi)^2 reproduces the near-wall auto-downweight
%   via the xi (sensor-noise) floor: as a_det -> 0, p = a_det^2/R22 -> 0.
%
%   T1 recovery        : whole-run WLS recovers s_true within 2% over a swept h_bar.
%   T2 var calibration : reported sqrt(var_s)=1/sqrt(I_g) matches the across-seed
%                        std of s_hat (MC) within 20%.
%   T3 unbiased (raw)  : feeding RAW a_xm (incl. negative shots) keeps |bias|<1%,
%                        whereas G2-style skipping of a_xm<=0 biases s_hat HIGH.
%   T4 pooling         : 3 identical-SNR axes give var_s ~ single-axis/3.
%   T5 tracking        : with forgetting beta_s, a mid-run step in g re-converges
%                        within ~3/beta_s steps.
%   T6 prior/clamp     : all-gated -> s_hat==1 exactly; a pathological shot is
%                        clamped to s_max.

    fprintf('=== verify_eq17_unit_gscalar_recursion ===\n');

    this_dir  = fileparts(mfilename('fullpath'));
    repo_root = fullfile(this_dir, '..', '..');
    addpath(fullfile(repo_root, 'model'));
    addpath(fullfile(repo_root, 'model', 'wall_effect'));

    % --- shared synthetic constants (realistic scales) ---
    a_nom = 0.0146;            % [um/pN] bulk mobility (= Ts/gamma_N)
    K_var = 2 * 0.05 / (2 - 0.05);
    IF    = 4.0;               % nominal color-inflation
    xi    = 2.0e-4;            % [um/pN] sensor-noise floor (drives downweight)
    s_min = 0.5; s_max = 2.0;
    I_prior = 200;

    n_pass = 0;

    % ===================================================================
    % T1: whole-run recovery (beta_s=0) over a swept h_bar
    % ===================================================================
    rng(11);
    N = 12000;
    s_true = 1.05;
    h_bar = 1.4 + 4.0 * (1 - cos(2*pi*(1:N)'/2000)) / 2;   % sweeps 1.4 .. 5.4
    cperp = arrayfun(@(h) cperp_of(h), h_bar);
    a_det = a_nom ./ cperp;                                % [N x 1]
    R22   = K_var * IF * (a_det + xi).^2;
    a_xm  = s_true * a_det + sqrt(R22) .* randn(N, 1);
    gate  = false(N, 1);
    [s_hat, var_s] = gscalar_recursion(a_xm, a_det, R22, 0.0, I_prior, gate, s_min, s_max);
    rel_err = abs(s_hat(end) - s_true) / s_true;
    fprintf('T1: s_hat=%.5f (true %.5f), rel.err=%.3f%%, sqrt(var_s)=%.4f\n', ...
            s_hat(end), s_true, 100*rel_err, sqrt(var_s(end)));
    assert(rel_err < 0.02, 'T1: WLS did not recover s_true within 2%%');
    n_pass = n_pass + 1; fprintf('T1 PASS recovery\n');

    % ===================================================================
    % T2: uncertainty calibration -- reported sqrt(var_s) vs MC across-seed std
    % ===================================================================
    n_seed = 300;
    s_finals = zeros(n_seed, 1);
    var_rep = 0;
    for sd = 1:n_seed
        rng(1000 + sd);
        axm = s_true * a_det + sqrt(R22) .* randn(N, 1);
        [sh, vr] = gscalar_recursion(axm, a_det, R22, 0.0, I_prior, gate, s_min, s_max);
        s_finals(sd) = sh(end);
        var_rep = vr(end);          % deterministic (same a_det,R22) across seeds
    end
    emp_std = std(s_finals);
    rep_std = sqrt(var_rep);
    cal_ratio = rep_std / emp_std;
    fprintf('T2: reported std=%.4e, empirical std=%.4e, ratio=%.3f\n', ...
            rep_std, emp_std, cal_ratio);
    assert(abs(cal_ratio - 1) < 0.20, 'T2: var_s miscalibrated >20%%');
    n_pass = n_pass + 1; fprintf('T2 PASS var calibration\n');

    % ===================================================================
    % T3: raw a_xm unbiased vs G2-skip (a_xm<=0) biased high -- near wall
    % ===================================================================
    rng(23);
    Nt = 20000;
    a_det_nw = (a_nom / cperp_of(1.2)) * ones(Nt, 1);      % fixed near-wall point
    R22_nw   = K_var * IF * (a_det_nw + xi).^2;
    % heavy noise so a_xm<=0 occurs a few % of the time
    axm_nw   = s_true * a_det_nw + sqrt(R22_nw) .* randn(Nt, 1);
    frac_neg = mean(axm_nw <= 0);
    g0 = false(Nt, 1);
    [sh_raw, ~]  = gscalar_recursion(axm_nw, a_det_nw, R22_nw, 0.0, I_prior, g0, s_min, s_max);
    [sh_skip, ~] = gscalar_recursion_skipneg(axm_nw, a_det_nw, R22_nw, 0.0, I_prior, s_min, s_max);
    bias_raw  = sh_raw(end)  / s_true - 1;
    bias_skip = sh_skip(end) / s_true - 1;
    fprintf('T3: frac(a_xm<=0)=%.1f%%, bias_raw=%.3f%%, bias_skip=%.3f%%\n', ...
            100*frac_neg, 100*bias_raw, 100*bias_skip);
    assert(frac_neg > 0.01, 'T3: setup did not produce negative shots');
    assert(abs(bias_raw) < 0.01, 'T3: raw recursion biased >1%%');
    assert(bias_skip > bias_raw + 0.01, 'T3: G2-skip not demonstrably more biased');
    n_pass = n_pass + 1; fprintf('T3 PASS unbiased-raw vs biased-skip\n');

    % ===================================================================
    % T4: 3-axis pooling -- var_s ~ single-axis var / 3
    % ===================================================================
    rng(44);
    Np = 6000;
    ad1 = a_det(1:Np);              R1 = R22(1:Np);
    axm1 = s_true*ad1 + sqrt(R1).*randn(Np,1);
    [~, vs1] = gscalar_recursion(axm1, ad1, R1, 0.0, I_prior, false(Np,1), s_min, s_max);
    ad3 = [ad1 ad1 ad1];           R3 = [R1 R1 R1];
    axm3 = s_true*ad3 + sqrt(R3).*randn(Np,3);
    [~, vs3] = gscalar_recursion(axm3, ad3, R3, 0.0, I_prior, false(Np,1), s_min, s_max);
    % subtract the shared prior contribution before taking the ratio
    pool_ratio = vs1(end) / vs3(end);
    fprintf('T4: var_1axis/var_3axis = %.3f (expect ~3)\n', pool_ratio);
    assert(abs(pool_ratio - 3) / 3 < 0.10, 'T4: pooling not ~3x precision');
    n_pass = n_pass + 1; fprintf('T4 PASS pooling\n');

    % ===================================================================
    % T5: tracking -- mid-run step in g re-converges within ~3/beta_s
    % ===================================================================
    rng(55);
    beta_s = 0.01;                 % window ~100 steps
    Ns = 4000;
    a_det_s = (a_nom / cperp_of(3.0)) * ones(Ns, 1);
    R22_s   = K_var * IF * (a_det_s + xi).^2;
    s_seq = ones(Ns, 1); s_seq(Ns/2+1:end) = 1.20;      % step 1.0 -> 1.20
    axm_s = s_seq .* a_det_s + sqrt(R22_s) .* randn(Ns, 1);
    [sh_s, ~] = gscalar_recursion(axm_s, a_det_s, R22_s, beta_s, I_prior, false(Ns,1), s_min, s_max);
    k_settle = Ns/2 + round(3 / beta_s);
    final_err = abs(sh_s(k_settle) - 1.20) / 1.20;
    fprintf('T5: post-step err at +3/beta_s = %.3f%%\n', 100*final_err);
    assert(final_err < 0.05, 'T5: did not re-converge after step');
    n_pass = n_pass + 1; fprintf('T5 PASS tracking\n');

    % ===================================================================
    % T6: prior/clamp
    % ===================================================================
    % (a) all-gated -> s_hat stays exactly 1
    Ng = 50;
    [sh_g, ~] = gscalar_recursion(5*ones(Ng,1), a_det(1:Ng), R22(1:Ng), 0.0, I_prior, true(Ng,1), s_min, s_max);
    assert(all(sh_g == 1), 'T6a: gated s_hat drifted from 1');
    % (b) pathological huge shot -> clamp at s_max
    axm_bad = 1e3 * a_det(1:Ng);
    [sh_b, ~] = gscalar_recursion(axm_bad, a_det(1:Ng), R22(1:Ng), 0.0, 1e-6, false(Ng,1), s_min, s_max);
    assert(sh_b(end) <= s_max + eps && sh_b(end) >= s_max - 1e-9, 'T6b: clamp to s_max failed');
    fprintf('T6: gated s_hat==1 ok; clamp -> s_max=%.2f ok\n', sh_b(end));
    n_pass = n_pass + 1; fprintf('T6 PASS prior/clamp\n');

    fprintf('=== ALL %d/6 PASS ===\n', n_pass);
end


%% =================== Local reference recursion (mirrors controller [4]-[5]) ===
function [s_hat, var_s] = gscalar_recursion(a_xm, a_det, R22, beta_s, I_prior, gate, s_min, s_max)
    N = size(a_xm, 1); A = size(a_xm, 2);
    I_g = I_prior; J_g = I_prior; s = 1;
    s_hat = zeros(N, 1); var_s = zeros(N, 1);
    for k = 1:N
        sum_p = 0; sum_J = 0;
        for ax = 1:A
            p_i = a_det(k,ax)^2 / R22(k,ax);
            sum_p = sum_p + p_i;
            sum_J = sum_J + a_det(k,ax) * a_xm(k,ax) / R22(k,ax);
        end
        if ~gate(k)
            I_g = (1 - beta_s) * I_g + sum_p;
            J_g = (1 - beta_s) * J_g + sum_J;
            s = min(max(J_g / I_g, s_min), s_max);
        end
        s_hat(k) = s; var_s(k) = 1 / I_g;
    end
end


function [s_hat, var_s] = gscalar_recursion_skipneg(a_xm, a_det, R22, beta_s, I_prior, s_min, s_max)
%   Variant that mimics applying a G2-style guard (skip axes where a_xm<=0).
    N = size(a_xm, 1); A = size(a_xm, 2);
    I_g = I_prior; J_g = I_prior;
    s_hat = zeros(N, 1); var_s = zeros(N, 1);
    for k = 1:N
        sum_p = 0; sum_J = 0;
        for ax = 1:A
            if a_xm(k,ax) <= 0; continue; end       % <-- the (wrong) G2 skip
            p_i = a_det(k,ax)^2 / R22(k,ax);
            sum_p = sum_p + p_i;
            sum_J = sum_J + a_det(k,ax) * a_xm(k,ax) / R22(k,ax);
        end
        I_g = (1 - beta_s) * I_g + sum_p;
        J_g = (1 - beta_s) * J_g + sum_J;
        s = min(max(J_g / I_g, s_min), s_max);
        s_hat(k) = s; var_s(k) = 1 / I_g;
    end
end


function cp = cperp_of(h_bar)
%CPERP_OF  perpendicular wall correction at h_bar (clamped >1).
    [~, cp] = calc_correction_functions(max(h_bar, 1.001));
end
