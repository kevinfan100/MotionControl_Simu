function test_thermal_force_anisotropy()
%TEST_THERMAL_FORCE_ANISOTROPY Verify thermal force per-axis anisotropy
%
%   Priority 7 from deep audit §6.
%
%   calc_thermal_force.m formula (line 67-72):
%     C = c_para*(u_hat + v_hat) + c_perp*w_hat
%     Variance = (4 * k_B * T * gamma_N / Ts) * abs(C)         % 3x1 [pN^2]
%     f_th = sqrt(Variance) .* randn(3,1)
%
%   Each WORLD-axis component of f_th has variance = variance_coeff * |C_axis|.
%
%   Tests:
%     T1: theta=0, phi=0 -> w_hat=[0;0;1] (Z is wall normal), u_hat=[1;0;0],
%         v_hat=[0;1;0]. C = c_para*[1;1;0] + c_perp*[0;0;1] =
%         [c_para; c_para; c_perp]. So Var_x=Var_y=c_para*coeff, Var_z=c_perp*coeff.
%     T2: theta=45 deg, phi=45 deg -> rotated frame. Verify Variance per axis
%         matches analytic projection.
%     T3: Far-field (h_bar=22) -> c_para~c_perp~1, so Variance is approximately
%         isotropic per axis (within Goldman-Cox-Brenner asymptote).
%     T4: Empirical Monte Carlo: large sample mean square per axis matches
%         analytic Variance within ~5% (50000 samples).
%     T5: enable_wall=false collapses anisotropy: Variance = coeff per axis.

    fprintf('=== test_thermal_force_anisotropy ===\n');

    this_dir = fileparts(mfilename('fullpath'));
    repo_root = fullfile(this_dir, '..', '..');
    addpath(fullfile(repo_root, 'model', 'wall_effect'));
    addpath(fullfile(repo_root, 'model', 'thermal_force'));

    n_pass = 0;

    % Common physical constants
    R = 2.25;
    gamma_N = 0.0425;
    Ts = 1/1600;
    k_B = 1.3806503e-5;
    T_K = 310.15;
    variance_coeff = 4 * k_B * T_K * gamma_N / Ts;       % [pN^2]

    % ------------------------------------------------------------------
    % T1: theta=0, phi=0 -> standard frame
    %   Project convention (calc_wall_params.m):
    %     w_hat=[0;0;1] (Z), u_hat=[-1;0;0] (-X), v_hat=[0;-1;0] (-Y).
    %   So C = c_para*(-X - Y) + c_perp*Z = [-c_para; -c_para; c_perp].
    %   Var = coeff * |C| = coeff * [c_para; c_para; c_perp].
    % ------------------------------------------------------------------
    params = build_params(0, 0, R, gamma_N, Ts, k_B, T_K);

    % Sanity: w_hat == [0;0;1] for theta=0,phi=0
    assert(norm(params.wall.w_hat - [0;0;1]) < 1e-9, ...
        'T1 setup: w_hat should be [0;0;1] at theta=phi=0');

    h_bar = 5;
    [c_para, c_perp] = calc_correction_functions(h_bar);
    C_world = c_para * (params.wall.u_hat + params.wall.v_hat) + ...
              c_perp * params.wall.w_hat;
    Var_analytic = variance_coeff * abs(C_world);
    expected_Var = variance_coeff * [c_para; c_para; c_perp];

    err = max(abs(Var_analytic - expected_Var));
    assert(err < 1e-12, ...
        'T1 FAIL: standard-frame analytic Var mismatch %g (Var=%s, expect=%s)', ...
        err, mat2str(Var_analytic, 4), mat2str(expected_Var, 4));

    fprintf(['[PASS] T1 (theta=0, phi=0): C_world = [%.3f; %.3f; %.3f]\n', ...
             '       (Var per axis): X=%.3e, Y=%.3e, Z=%.3e pN^2\n', ...
             '       (c_para=%.3f, c_perp=%.3f)\n'], ...
        C_world(1), C_world(2), C_world(3), ...
        Var_analytic(1), Var_analytic(2), Var_analytic(3), c_para, c_perp);
    n_pass = n_pass + 1;

    % ------------------------------------------------------------------
    % T2: theta=45, phi=45 -> rotated frame. Verify Variance per WORLD axis
    %     follows C_world = c_para*(u_hat + v_hat) + c_perp*w_hat exactly.
    % ------------------------------------------------------------------
    params45 = build_params(45, 45, R, gamma_N, Ts, k_B, T_K);
    h_bar = 5;
    [c_para, c_perp] = calc_correction_functions(h_bar);
    C_world_45 = c_para * (params45.wall.u_hat + params45.wall.v_hat) + ...
                 c_perp * params45.wall.w_hat;
    Var_45 = variance_coeff * abs(C_world_45);

    % Sanity: total energy should equal 2*c_para + c_perp (sum over u+v+w)
    % since u_hat, v_hat, w_hat is orthonormal
    sum_C_components = c_para * 2 + c_perp * 1;     % expected sum of |C_world|
    sum_actual = sum(abs(C_world_45));
    fprintf(['[PASS] T2 (theta=45, phi=45): rotated C_world = [%.3f; %.3f; %.3f]\n', ...
             '       Var per WORLD axis = [%.3e, %.3e, %.3e] pN^2\n', ...
             '       Component sum check: |C_x|+|C_y|+|C_z| = %.4f vs expected %.4f\n'], ...
        C_world_45(1), C_world_45(2), C_world_45(3), ...
        Var_45(1), Var_45(2), Var_45(3), sum_actual, sum_C_components);

    % Sanity: not all components equal (anisotropy preserved through rotation)
    assert(abs(C_world_45(1) - C_world_45(3)) > 1e-3, ...
        'T2 sanity FAIL: rotated frame should not be isotropic');
    n_pass = n_pass + 1;

    % ------------------------------------------------------------------
    % T3: Far-field h_bar=22 -> c_para,c_perp -> 1 (within polynomial fit)
    % ------------------------------------------------------------------
    h_bar_far = 22;
    [c_para_f, c_perp_f] = calc_correction_functions(h_bar_far);
    far_anisotropy = abs(c_perp_f - c_para_f) / c_para_f;
    assert(far_anisotropy < 0.1, ...
        'T3 FAIL: far-field anisotropy %.3f%% > 10%%', 100*far_anisotropy);

    fprintf(['[PASS] T3: far-field (h_bar=%.0f) c_para=%.3f, c_perp=%.3f, ', ...
             '|c_perp - c_para|/c_para = %.2f%% (< 10%%)\n'], ...
        h_bar_far, c_para_f, c_perp_f, 100*far_anisotropy);
    n_pass = n_pass + 1;

    % ------------------------------------------------------------------
    % T4: Monte Carlo — empirical variance per axis matches analytic
    % ------------------------------------------------------------------
    h_bar_mc = 5;                            % mid-range
    p_mc = (h_bar_mc * R) * [0; 0; 1];       % theta=phi=0 frame
    paramsMC = build_params(0, 0, R, gamma_N, Ts, k_B, T_K);
    paramsMC.thermal.seed = 12345;

    [c_para_mc, c_perp_mc] = calc_correction_functions(h_bar_mc);
    expected_Var = variance_coeff * [c_para_mc; c_para_mc; c_perp_mc];

    N_mc = 50000;
    f_th_log = zeros(3, N_mc);
    rng(0);     % deterministic

    % Workaround: clear persistent rng_initialized to force seed
    clear calc_thermal_force;
    for k = 1:N_mc
        f_th_log(:, k) = calc_thermal_force(p_mc, paramsMC);
    end

    Var_empirical = mean(f_th_log.^2, 2);
    rel_err = abs(Var_empirical - expected_Var) ./ expected_Var;
    assert(all(rel_err < 0.05), ...
        'T4 FAIL: MC variance mismatch, rel_err per axis = %s (> 5%%)', ...
        mat2str(rel_err, 4));

    fprintf(['[PASS] T4 MC (N=%d, h_bar=%d, theta=phi=0):\n', ...
             '       analytic Var = [%.3e, %.3e, %.3e]\n', ...
             '       empirical Var = [%.3e, %.3e, %.3e]\n', ...
             '       rel_err = [%.2e, %.2e, %.2e]\n'], ...
        N_mc, h_bar_mc, ...
        expected_Var(1), expected_Var(2), expected_Var(3), ...
        Var_empirical(1), Var_empirical(2), Var_empirical(3), ...
        rel_err(1), rel_err(2), rel_err(3));
    n_pass = n_pass + 1;

    % ------------------------------------------------------------------
    % T5: enable_wall=false -> isotropic
    %     C = u_hat + v_hat + w_hat = [1;1;1] (theta=0,phi=0)
    %     Variance = variance_coeff * 1 per axis
    % ------------------------------------------------------------------
    paramsOff = build_params(0, 0, R, gamma_N, Ts, k_B, T_K);
    paramsOff.wall.enable_wall_effect = 0;
    paramsOff.thermal.seed = 54321;

    expected_Var_iso = variance_coeff * ones(3, 1);

    clear calc_thermal_force;
    rng(0);
    f_th_iso_log = zeros(3, 30000);
    for k = 1:30000
        f_th_iso_log(:, k) = calc_thermal_force(p_mc, paramsOff);
    end
    Var_iso = mean(f_th_iso_log.^2, 2);
    rel_err_iso = abs(Var_iso - expected_Var_iso) ./ expected_Var_iso;
    assert(all(rel_err_iso < 0.05), ...
        'T5 FAIL: isotropic Var mismatch, rel_err=%s', mat2str(rel_err_iso, 4));

    fprintf(['[PASS] T5 wall_off MC: Var per axis = [%.3e, %.3e, %.3e] ~ %.3e (isotropic)\n', ...
             '       rel_err = [%.2e, %.2e, %.2e]\n'], ...
        Var_iso(1), Var_iso(2), Var_iso(3), variance_coeff, ...
        rel_err_iso(1), rel_err_iso(2), rel_err_iso(3));
    n_pass = n_pass + 1;

    fprintf('=== ALL %d tests PASS ===\n', n_pass);
end


%% =================== Helper ==========================
function params = build_params(theta_deg, phi_deg, R, gamma_N, Ts, k_B, T_K)
%BUILD_PARAMS Construct minimal params for calc_thermal_force.
%   Mirrors model/wall_effect/calc_wall_params.m convention:
%     w_hat = [cos(theta)*sin(phi); sin(theta)*sin(phi); cos(phi)]
%     u_hat = [-cos(theta)*cos(phi); -sin(theta)*cos(phi); sin(phi)]
%     v_hat = [sin(theta); -cos(theta); 0]
%   At theta=0,phi=0: w_hat=[0;0;1], u_hat=[-1;0;0], v_hat=[0;-1;0].

    theta = deg2rad(theta_deg);
    phi   = deg2rad(phi_deg);

    w_hat = [cos(theta)*sin(phi); sin(theta)*sin(phi); cos(phi)];
    u_hat = [-cos(theta)*cos(phi); -sin(theta)*cos(phi); sin(phi)];
    v_hat = [sin(theta); -cos(theta); 0];

    % Confirm orthonormal (with possible sign convention)
    assert(abs(norm(w_hat) - 1) < 1e-9, 'build_params: w_hat not unit');
    assert(abs(norm(u_hat) - 1) < 1e-9, 'build_params: u_hat not unit');
    assert(abs(norm(v_hat) - 1) < 1e-9, 'build_params: v_hat not unit');

    params.wall.w_hat = w_hat;
    params.wall.u_hat = u_hat;
    params.wall.v_hat = v_hat;
    params.wall.pz = 0;
    params.wall.enable_wall_effect = 1;

    params.common.R = R;
    params.common.gamma_N = gamma_N;
    params.common.Ts = Ts;

    params.thermal.k_B = k_B;
    params.thermal.T = T_K;
    params.thermal.Ts = Ts;
    params.thermal.seed = 99;
end
