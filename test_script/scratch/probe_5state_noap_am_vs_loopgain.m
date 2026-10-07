% PURPOSE (2026-09-29, user: "a_m near the wall is above a_true -- the thermal motion at the true height should give a_true"):
%   plane, held at 1.10 R (no motion), first order, lambda_f = 1. The control law is forced to use the gain
%   scale * a_true (driver opts.use_true_gain / true_gain_scale), so the only thing that changes is how hard the
%   controller corrects. The thermal kicks are the same physics in every arm. Prints mean a_m / a_true and the
%   spread of the random part dx_r against the spread the readout formula expects at a_true.   STATUS: SCRATCH
function probe_5state_noap_am_vs_loopgain()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));  addpath(fullfile(root, 'test_script', 'integration'));
    pc = physical_constants();  R = pc.R;  a_nom = pc.Ts / pc.gamma_N;  kBT = pc.k_B * pc.T;  h = 1.10;
    cfg = canonical_scenario(0.05, 1.10, 'deep');  cfg.eq17_variant = '5state_noap';
    cfg.trajectory_type = 'positioning';  cfg.h_init = h * R;  cfg.h_bottom = h * R;  cfg.amplitude = 0;
    cfg.t_hold = 0;  cfg.t_descend_override = 0;  cfg.T_sim = 3.0;
    cc = struct('lf_mode', 'const', 'lambda_f', 1, 'gain_order', 1, 'seed_mode', 'true_h0');
    scales = [1 3 9];  seeds = 1:3;
    fprintf('\nplane, held at %.2f R. control law uses scale x a_true; a_hat[k]/a_true of the arm is irrelevant to the physics.\n', h);
    fprintf('%-6s | %-10s | %-14s | %-22s | %-22s\n', 'scale', 'loop pole', 'mean a_m/a_true', 'std dx_r [nm] measured', 'expected at a_true [nm]');
    for sc = scales
        r_am = [];  r_sd = [];  r_ex = [];
        for sd = seeds
            s = run_pure_simulation(cfg, struct('seed', sd, 'collect_diag', true, 'ctrl_const_override', cc, ...
                'use_true_gain', true, 'true_gain_scale', sc));
            t = s.tout(:);  m = t > 0.5;
            at = s.a_true_out(m, 3);  am = s.diag.a_xm(m, 3);  dxr = s.diag.dx_r(m, 3);
            ex = sqrt(s.ctrl_const.C_dpmr * 4 * kBT * mean(at) + s.ctrl_const.C_n * cfg.meas_noise_std(3)^2);
            r_am(end + 1) = mean(am) / mean(at);  r_sd(end + 1) = std(dxr) * 1e3;  r_ex(end + 1) = ex * 1e3; %#ok<AGROW>
        end
        pole = 1 - (1 / sc) * (1 - cfg.lambda_c);
        fprintf('%-6g | %-10.3f | %-14.2f | %-22.2f | %-22.2f\n', sc, pole, mean(r_am), mean(r_sd), mean(r_ex));
    end
    fprintf('a_true at %.2f R = %.4f a_nom\n', h, mean(at) / a_nom);
end
