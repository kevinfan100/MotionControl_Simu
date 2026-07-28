% TEMP dither amplitude/frequency sweep (chat 2026-07-21): confirm the
% observability-gain scaling I_dither/I_baseline ~ 1 + (A*wd*Ts/sqrt2/sigma_phi)^2
% and locate where first-hold std(az) drops toward the frozen-a' floor.
% Same REC config + seed 1 as temp_dither_pilot.m. Delete after use.

here = fileparts(mfilename('fullpath'));
proj = fileparts(here);
addpath(genpath(fullfile(proj, 'model')));
addpath(here);

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
a_nom = Pp.common.Ts / Pp.ctrl.gamma;
w = Pp.wall.w_hat;  pz = Pp.wall.pz;  Ts = Pp.common.Ts;  lc = cfg.lambda_c;
cfg.aprime_state_P0 = (0.5*a_nom/R)^2;

runarm = @(f, A) local_run(cfg, f, A, w, pz, lc);

% baseline
B = runarm(0, 0);

specs = [ 5 0.150; 5 0.300; 5 0.600; ...
         20 0.050; 20 0.150; 20 0.300; 20 0.600 ];

fprintf('\n=== dither amplitude sweep (first hold, seed 1) ===\n');
fprintf('baseline: std(phi)=%.4e um  I=%.4e  std(az)=%.4f nm/pN\n\n', ...
    B.sphi, B.I, B.stdaz);
fprintf('%3s %7s | %11s %11s | %9s %9s | %10s %10s\n', ...
    'f', 'A[nm]', 'stdDHd[nm]', 'predRatio', 'I/I0', 'obs-1', 'std(az)', 'a''std');
for i = 1:size(specs,1)
    f = specs(i,1);  A = specs(i,2);
    r = runarm(f, A);
    sdhd = A*2*pi*f*Ts/sqrt(2);                 % um, coherent regressor rms
    predRatio = 1 + (sdhd / B.sphi)^2;          % predicted 1 + power ratio
    fprintf('%3d %7.1f | %11.3f %11.4f | %9.4f %9.4f | %10.4f %10.3f\n', ...
        f, A*1e3, sdhd*1e3, predRatio, r.I/B.I, r.I/B.I - 1, r.stdaz, r.apstd);
end
fprintf('\n(frozen-a'' floor target std(az) = 0.015 nm/pN; baseline = %.4f)\n', B.stdaz);

function r = local_run(cfg, f, A, w, pz, lc)
    opts = struct('seed', 1, 'collect_diag', true);
    if f > 0
        opts.dither = struct('on', true, 'A', A, 'f', f);
    else
        opts.dither = struct('on', false, 'A', 0, 'f', 0);
    end
    so = temp_run_pure_sim_dither(cfg, opts);
    t = so.tout;  fh = t < 0.5;
    az  = so.diag.a_hat(:,3);
    ap  = so.diag.a_prime_used(:,3);
    dh3 = so.diag.delta_x_hat_3(:,3);
    Sy2 = so.diag.S_y2(:,3);
    g3  = so.diag.gate_active(:,3);
    hdz = so.p_d_out*w - pz;
    dh_d = [hdz(2:end)-hdz(1:end-1); hdz(end)-hdz(end-1)];
    phi = dh_d + (1-lc)*dh3;
    m = fh & (Sy2>0) & ~g3;
    r.I     = sum(phi(m).^2 ./ Sy2(m));
    r.sphi  = std(phi(fh));
    r.stdaz = std(az(fh))*1e3;
    r.apstd = std(ap(fh))*1e3;
end
