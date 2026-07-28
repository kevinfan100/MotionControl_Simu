% TEMP dither pilot (chat 2026-07-21): validate "dither restores slope
% observability" (5state_aprime_coupdate.tex §11-12). z-axis only, seed 1,
% pure-MATLAB. Baseline vs dither f=5/10/20 Hz, A=15 nm, hold windows.
% Delete after use.

here = fileparts(mfilename('fullpath'));
proj = fileparts(here);
addpath(genpath(fullfile(proj, 'model')));
addpath(here);

% ---------------- STEP 2: config (REC scenario, as-state learn-from-start) ----------------
pc = physical_constants();  R = pc.R;
cfg = user_config();
cfg.eq17_variant = '4state';   cfg.trajectory_type = 'osc';
cfg.h_init = 50;   cfg.h_bottom = 2.0*R;   cfg.amplitude = 2.5;
cfg.frequency = 1; cfg.n_cycles = 4;
cfg.t_hold = 0.5;  cfg.t_descend_override = 1.0;  cfg.h_min = 1.05*R;  cfg.T_sim = 6.0;
cfg.ctrl_enable = true;  cfg.thermal_enable = true;  cfg.meas_noise_enable = true;
cfg.lambda_c = 0.7;  cfg.a_pd = 0.05;  cfg.a_cov = 0.05;
cfg.meas_noise_std = [0.00062; 0.00057; 0.00331];
cfg.suppress_xD = true;  cfg.h_bar_safe = 1;
cfg.use_taylor_gain = true;
cfg.aprime_source = 'state';  cfg.aprime_state_selfmod = true;  cfg.aprime_learn_t0 = 0;

Pp = calc_simulation_params(cfg);  Pp = Pp.Value;
a_nom = Pp.common.Ts / Pp.ctrl.gamma;      % [um/pN]
w  = Pp.wall.w_hat;    pz = Pp.wall.pz;
cfg.aprime_state_P0 = (0.5*a_nom/R)^2;

lc = cfg.lambda_c;

% ---------------- STEP 3: run arms (all seed 1) ----------------
arm_defs = { ...
    'baseline', 0,  0;      ...
    'dither5',  5,  0.015;  ...
    'dither10', 10, 0.015;  ...
    'dither20', 20, 0.015   };
n_arm = size(arm_defs, 1);
arms = struct('label', {}, 'f', {}, 'A', {}, 't', {}, 'az', {}, 'azt', {}, ...
    'ap', {}, 'oracle', {}, 'Q44', {}, 'P44', {}, 'L4', {}, 'dh3', {}, ...
    'Sy2', {}, 'phi', {}, 'hb', {}, 'dz', {}, 'dxz', {}, 'hdz', {});

for ia = 1:n_arm
    label = arm_defs{ia, 1};  f = arm_defs{ia, 2};  A = arm_defs{ia, 3};
    opts = struct('seed', 1, 'collect_diag', true);
    if f > 0
        opts.dither = struct('on', true, 'A', A, 'f', f);
    else
        opts.dither = struct('on', false, 'A', 0, 'f', 0);
    end
    so = temp_run_pure_sim_dither(cfg, opts);

    t   = so.tout;
    az  = so.diag.a_hat(:, 3);
    azt = so.a_true_out(:, 3);
    ap  = so.diag.a_prime_used(:, 3);
    Q44 = so.diag.var_da_ram(:, 3);
    P44 = so.diag.P_a(:, 3);
    L4  = so.diag.K_kf_a_y2(:, 3);
    dh3 = so.diag.delta_x_hat_3(:, 3);
    Sy2 = so.diag.S_y2(:, 3);
    hb  = so.diag.h_bar(:);
    dz  = so.dither_z(:);
    gate3 = so.diag.gate_active(:, 3);

    % desired z height [um] and tracking error along w [nm]
    hdz = so.p_d_out * w - pz;                                % [N x 1, um]
    hz_true = so.p_true_out * w - pz;                         % [N x 1, um]
    dxz = (hdz - hz_true) * 1e3;                              % [N x 1, nm]

    % oracle slope a'(h_d) [um/pN/um]
    N = numel(t);   oracle = zeros(N, 1);
    for k = 1:N
        hbd = max((so.p_d_out(k, :) * w - pz) / R, 1.001);
        [~, cp, drv] = calc_correction_functions(hbd, true);
        oracle(k) = -(a_nom / cp) * drv.K_h_perp / R;
    end

    % regressor phi = dh_d + (1-lc)*dh3   (dh_d = hdz[k+1]-hdz[k], pad last)
    dh_d = [hdz(2:end) - hdz(1:end-1); hdz(end) - hdz(end-1)];
    phi  = dh_d + (1 - lc) * dh3;

    arms(ia).label = label;  arms(ia).f = f;  arms(ia).A = A;
    arms(ia).t = t;      arms(ia).az = az;    arms(ia).azt = azt;
    arms(ia).ap = ap;    arms(ia).oracle = oracle;
    arms(ia).Q44 = Q44;  arms(ia).P44 = P44;  arms(ia).L4 = L4;
    arms(ia).dh3 = dh3;  arms(ia).Sy2 = Sy2;  arms(ia).phi = phi;
    arms(ia).hb = hb;    arms(ia).dz = dz;    arms(ia).dxz = dxz;
    arms(ia).hdz = hdz;
    arms(ia).gate3 = gate3;
end

% ---------------- STEP 4: metrics ----------------
t = arms(1).t;
fh = t < 0.5;          % first hold
Fh = t >= 5.5;         % final hold

fisher = @(phi, Sy2, mask) sum( (phi(mask).^2) ./ Sy2(mask) );
maskfun = @(win, Sy2, g) (win & (Sy2 > 0) & ~g);

M = struct();
for ia = 1:n_arm
    a = arms(ia);
    mfh = maskfun(fh, a.Sy2, a.gate3);
    mFh = maskfun(Fh, a.Sy2, a.gate3);
    M(ia).label = a.label;  M(ia).f = a.f;  M(ia).A = a.A;
    % first hold
    M(ia).fh_std_az   = std(a.az(fh)) * 1e3;                 % nm/pN
    M(ia).fh_ap_mean  = mean(a.ap(fh)) * 1e3;                % nm/pN/um
    M(ia).fh_ap_std   = std(a.ap(fh))  * 1e3;
    M(ia).fh_I        = fisher(a.phi, a.Sy2, mfh);
    M(ia).fh_rms_dxz  = rms(a.dxz(fh));                      % nm
    % final hold
    M(ia).Fh_std_az   = std(a.az(Fh)) * 1e3;
    M(ia).Fh_ap_mean  = mean(a.ap(Fh)) * 1e3;
    M(ia).Fh_ap_std   = std(a.ap(Fh))  * 1e3;
    M(ia).Fh_I        = fisher(a.phi, a.Sy2, mFh);
    M(ia).Fh_rms_dxz  = rms(a.dxz(Fh));
    % commanded dither rms
    M(ia).cmd_dither_rms = a.A / sqrt(2) * 1e3;              % nm (0 for baseline)
end

% ---------------- print ----------------
fprintf('\n================ DITHER PILOT (seed 1, z-axis) ================\n');
fprintf('a_nom = %.6f um/pN (%.3f nm/pN)\n', a_nom, a_nom*1e3);
fprintf('frozen-a'' floor target for std(az) = 0.015 nm/pN\n\n');

fprintf('%-9s | %10s %10s | FIRST HOLD (t<0.5)\n', 'arm', 'f[Hz]', 'A[nm]');
fprintf('%-9s | %11s %10s %12s %12s | %10s\n', '', ...
        'std(az)', 'a''mean', 'a''std', 'Fisher I', 'rms(dxz)');
fprintf('%-9s | %11s %10s %12s %12s | %10s\n', '', ...
        '[nm/pN]', '[/pN/um]', '[/pN/um]', '[proxy]', '[nm]');
for ia = 1:n_arm
    fprintf('%-9s | %10.1f %10.1f | %11.4f %10.2f %12.3f %12.4e | %10.2f\n', ...
        M(ia).label, M(ia).f, M(ia).A*1e3, ...
        M(ia).fh_std_az, M(ia).fh_ap_mean, M(ia).fh_ap_std, M(ia).fh_I, M(ia).fh_rms_dxz);
end

fprintf('\n%-9s | FINAL HOLD (t>=5.5)\n', 'arm');
fprintf('%-9s | %11s %10s %12s %12s | %10s\n', '', ...
        'std(az)', 'a''mean', 'a''std', 'Fisher I', 'rms(dxz)');
for ia = 1:n_arm
    fprintf('%-9s | %11.4f %10.2f %12.3f %12.4e | %10.2f\n', ...
        M(ia).label, ...
        M(ia).Fh_std_az, M(ia).Fh_ap_mean, M(ia).Fh_ap_std, M(ia).Fh_I, M(ia).Fh_rms_dxz);
end

fprintf('\n---- Fisher-proxy observability gain I_dither / I_baseline ----\n');
I0_fh = M(1).fh_I;  I0_Fh = M(1).Fh_I;
for ia = 2:n_arm
    fprintf('  f=%2d Hz: first hold %.3f x   final hold %.3f x\n', ...
        M(ia).f, M(ia).fh_I / I0_fh, M(ia).Fh_I / I0_Fh);
end

fprintf('\n---- probe-follows-dither: first-hold rms(dxz) vs commanded dither rms ----\n');
fprintf('  baseline rms(dxz) = %.2f nm (commanded dither rms = 0)\n', M(1).fh_rms_dxz);
for ia = 2:n_arm
    fprintf('  f=%2d Hz  rms(dxz) = %.2f nm   (commanded dither rms = %.2f nm)\n', ...
        M(ia).f, M(ia).fh_rms_dxz, M(ia).cmd_dither_rms);
end

fprintf('\n---- does first-hold std(az) drop toward frozen-a'' floor 0.015 nm/pN? ----\n');
for ia = 1:n_arm
    fprintf('  %-9s std(az)_fh = %.4f nm/pN\n', M(ia).label, M(ia).fh_std_az);
end

% ---------------- save ----------------
save(fullfile(here, 'temp_dither_pilot.mat'), 'arms', 'cfg', 'a_nom', 'M', '-v7.3');
fprintf('\nSaved: %s\n', fullfile(here, 'temp_dither_pilot.mat'));
