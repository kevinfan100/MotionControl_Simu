% STATUS: SCRATCH -- 2026-10-15, regression for the plant wall-angle change (user 10-15, option b): config.theta/phi
%   (azimuth, polar) -> config.theta_x/theta_y (rotation about x then y, V = R_y(theta_y) R_x(theta_x), u = x, v = y, w = z
%   at zero angles, as in Estimator_and_Control__.pdf p.6). Old u_hat = -x, v_hat = -y at zero angles, so tangential
%   amplitudes flip sign (amp_u -9 -> +9) to keep the same desired path.
%   mode 'baseline' (run BEFORE the change) saves test_results/temp_wall_angle_xy_baseline.mat;
%   mode 'check' (AFTER) reruns and compares. Pass criteria:
%     A  ramp_tri 5state_noap seed 1: every numeric output bit-identical (isequaln);
%     B  same with x/y triangle sines (amp_u, amp_v): bit-identical, desired path identical;
%     C  tilted wall: new (theta_x, theta_y) gives the same w_hat as the old (theta, phi) it maps to, so Gamma_inv agrees.
function verify_wall_angle_xy(mode)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));  addpath(fullfile(root, 'test_script', 'integration'));  addpath(here);
    fn = fullfile(root, 'test_results', 'temp_wall_angle_xy_baseline.mat');
    sgn = 1;  if strcmp(mode, 'baseline'); sgn = -1; end            % old convention u_hat = -x: amp -9 gives +9 along x
    R = struct();
    cfg = canonical_scenario(0.05, 1.10, 'ramp_tri', 1);  cfg.eq17_variant = '5state_noap';  cfg.lambda_c = 0.4;
    R.A = local_run(cfg);
    cfg.amp_u = sgn * 9;  cfg.freq_u = 1;  cfg.amp_v = sgn * 9;  cfg.freq_v = 1;  cfg.tan_shape = 1;
    R.B = local_run(cfg);
    if strcmp(mode, 'baseline')
        save(fn, '-struct', 'R');  fprintf('baseline saved: %s\n', fn);  return
    end
    B0 = load(fn);
    fprintf('A ramp_tri noap seed 1        : bit-identical %d\n', local_same(B0.A, R.A));
    fprintf('B with x/y sines              : bit-identical %d, desired path identical %d\n', local_same(B0.B, R.B), isequaln(B0.B.p_d_out, R.B.p_d_out));
    T = [0 20; 90 20; 30 15];                                         % old (theta_az, phi) in deg
    for i = 1:size(T, 1)
        ta = T(i, 1) * pi / 180;  ph = T(i, 2) * pi / 180;
        w_old = [cos(ta) * sin(ph); sin(ta) * sin(ph); cos(ph)];
        tx = -asin(sin(ta) * sin(ph));  ty = atan2(cos(ta) * sin(ph), cos(ph));
        c = user_config();  c.theta_x = tx * 180 / pi;  c.theta_y = ty * 180 / pi;
        P = calc_simulation_params(c);  w_new = P.Value.wall.w_hat;
        fprintf('C old (az %3d, phi %2d) -> theta_x %6.2f, theta_y %6.2f: |w_new - w_old| = %.2e, V orthonormal err %.1e\n', ...
                T(i, 1), T(i, 2), c.theta_x, c.theta_y, norm(w_new - w_old), norm(P.Value.wall.V' * P.Value.wall.V - eye(3)));
    end
    c = user_config();  P = calc_simulation_params(c);
    fprintf('C zero angles: u_hat %s  v_hat %s  w_hat %s\n', mat2str(P.Value.wall.u_hat'), mat2str(P.Value.wall.v_hat'), mat2str(P.Value.wall.w_hat'));
    local_tilt(root);
end

function local_tilt(root)                                   % E3/E4 (10-15): tilted wall, plant level
    c = user_config();  c.theta_y = 20;  c.enable_wall_effect = true;  P = calc_simulation_params(c);  Pv = P.Value;
    hb = 1.2;  p = (Pv.wall.pz + hb * Pv.common.R) * Pv.wall.w_hat;  V = Pv.wall.V;
    [cpa, cpe] = calc_correction_functions(hb);  coeff = 4 * Pv.thermal.k_B * Pv.thermal.T * Pv.common.gamma_N / Pv.thermal.Ts;
    n = 200000;  F = zeros(3, n);  clear calc_thermal_force;  for i = 1:n; F(:, i) = calc_thermal_force(p, Pv); end
    S_th = coeff * V * diag([cpa cpa cpe]) * V';  S_m = cov(F');
    fprintf('E3 thermal force covariance / coeff (theta_y 20, h/R 1.2), measured vs V diag(c) V'':\n');
    disp([S_m / coeff; S_th / coeff]);
    fprintf('E3 x-z correlation: measured %.3f, expected %.3f\n', S_m(1,3) / sqrt(S_m(1,1) * S_m(3,3)), S_th(1,3) / sqrt(S_th(1,1) * S_th(3,3)));
    G = calc_gamma_inv(p, Pv);  D = Pv.thermal.Ts * G * F;  S_d = cov(D');          % one-step thermal displacement
    ang = 0.5 * atan2(-2 * S_d(1,3), S_d(1,1) - S_d(3,3)) * 180 / pi;
    fprintf('E3 one-step thermal displacement: principal direction from the x-z covariance = %.2f deg (wall 20)\n', ang);
    a_nom = Pv.common.Ts / Pv.common.gamma_N;  A = V * diag(a_nom ./ [cpa cpa cpe]) * V';
    fprintf('E4 A_true vs Ts*Gamma_inv: max rel diff %.1e; eig(A)/a_nom = %s (a_T %.3f, a_N %.3f)\n', ...
            max(abs(A(:) - Pv.common.Ts * G(:))) / max(abs(A(:))), mat2str(sort(eig(A))' / a_nom, 4), 1 / cpa, 1 / cpe);
end

function S = local_run(cfg)
    S = run_pure_simulation(cfg, struct('seed', 1, 'collect_diag', true));
end

function ok = local_same(a, b)                               % every numeric leaf equal (isequaln), struct fields recursed
    ok = true;  f = fieldnames(a);
    SKIP = {'theta', 'phi', 'theta_x', 'theta_y', 'u_hat', 'v_hat', 'V', 'amp_u', 'amp_v'};   % parameter echoes that change by design
    f = setdiff(f, SKIP, 'stable');
    for i = 1:numel(f)
        x = a.(f{i});  if ~isfield(b, f{i}); fprintf('   missing field %s\n', f{i}); ok = false; continue; end
        y = b.(f{i});
        if isstruct(x); if ~local_same(x, y); ok = false; end
        elseif isnumeric(x) || islogical(x)
            if ~isequaln(x, y); fprintf('   differs: %s\n', f{i}); ok = false; end
        end
    end
end
