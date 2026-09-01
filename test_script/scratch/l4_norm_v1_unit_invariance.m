% l4_norm_v1_unit_invariance.m -- normalization-boundary audit, arm V1 (2026-08-30)
% PURPOSE: the formC_b loop is claimed dimensionless with a_o = Ts/(gamma_N R) as the
%          only gain scale. If that is true, re-expressing every LENGTH in nm instead of
%          um (R, k_B, gamma_N, h_init, amplitude, meas_noise_std, pz all rescaled; force
%          in pN and time in s untouched) must leave every internal dimensionless log
%          unchanged and every physical log scaled by exactly 1e3^n.
% PRE-REGISTERED CRITERIA (written before running):
%   dimensionless logs (a_bar_hat, Q33, Q44, R2, innov_y1/y2, h_bar, f_bar, b_hat, K_*):
%       max relative difference < 1e-9   (only 1e3-rounding noise allowed)
%   length logs (p_true, p_m, delta_x_hat_3, dx_r):                     ratio 1e3
%   gain logs  (a_hat, a_xm [um/pN -> nm/pN], sqrt(P_a)):               ratio 1e3
%   force logs (f_d, F_th [pN]):                                        ratio 1
%   A miss on ONE family points at the boundary that leaks: U0 (kappa_T/sigma2_n),
%   U1 (a length not divided by R), U3 (force out), U4 (display), or a hard-coded um
%   constant somewhere in the driver/plant. A miss everywhere = the shadow did not take.
% MECHANISM: physical_constants() is shadowed by a temp copy placed FIRST on the path
%   for the second run only; removed afterwards. Same seed, same rng streams.
function l4_norm_v1_unit_invariance()
    here = fileparts(mfilename('fullpath'));
    root = fullfile(here, '..', '..');
    addpath(genpath(fullfile(root, 'model')));
    addpath(fullfile(root, 'test_script', 'integration'));

    SEED = 7;
    S    = 1e3;        % um -> nm

    % ---------------- baseline (um) ----------------
    clear physical_constants motion_control_law_formC_b calc_thermal_force; rehash;
    pc0  = physical_constants();
    assert(abs(pc0.R - 2.25) < 1e-12, 'baseline physical_constants is not the house one');
    fprintf('[V1] baseline run (um), seed %d ...\n', SEED);
    o0   = run_formC_b(struct('seeds', SEED, 'scenario', 'deep'));   % full driver path (priors derived)
    out0 = o0.runs{1};  cfg0 = o0.cfg;

    % ---------------- shadowed (nm) ----------------
    shadow = fullfile(tempdir, 'l4_norm_v1_shadow');
    if ~exist(shadow, 'dir'); mkdir(shadow); end
    fid = fopen(fullfile(shadow, 'physical_constants.m'), 'w');
    fprintf(fid, 'function c = physical_constants()\n');
    fprintf(fid, '%% SHADOW for l4_norm_v1_unit_invariance: lengths in nm\n');
    fprintf(fid, 'c.R = %.17g;\n',       pc0.R * S);          % nm
    fprintf(fid, 'c.gamma_N = %.17g;\n', pc0.gamma_N / S);    % pN s / nm
    fprintf(fid, 'c.Ts = %.17g;\n',      pc0.Ts);             % s
    fprintf(fid, 'c.k_B = %.17g;\n',     pc0.k_B * S);        % pN nm / K
    fprintf(fid, 'c.T = %.17g;\n',       pc0.T);              % K
    fprintf(fid, 'end\n');
    fclose(fid);
    addpath(shadow, '-begin');
    clear physical_constants motion_control_law_formC_b calc_thermal_force; rehash;
    pc1 = physical_constants();
    assert(abs(pc1.R - pc0.R * S) < 1e-9, 'shadow physical_constants did not take');

    % h_min / h_bottom follow pc.R inside canonical_scenario; the um literals are overridden
    ov = struct('h_init', cfg0.h_init * S, 'amplitude', cfg0.amplitude * S, ...
                'meas_noise_std', cfg0.meas_noise_std * S);
    if isfield(cfg0, 'pz'); ov.pz = cfg0.pz * S; end
    fprintf('[V1] shadowed run (nm), seed %d ...\n', SEED);
    try
        o1   = run_formC_b(struct('seeds', SEED, 'scenario', 'deep', 'config_override', ov));
        out1 = o1.runs{1};
    catch ME
        rmpath(shadow); clear physical_constants; rehash;
        rethrow(ME);
    end
    rmpath(shadow);
    clear physical_constants motion_control_law_formC_b calc_thermal_force; rehash;
    assert(abs(physical_constants().R - pc0.R) < 1e-12, 'path not restored');

    % ---------------- compare ----------------
    fprintf('\nrun lengths: %d vs %d steps\n', numel(out0.tout), numel(out1.tout));
    fprintf('a_nom: %.6e um/pN vs %.6e nm/pN (ratio %.6f, expect %.0f)\n', ...
            out0.a_nom, out1.a_nom, out1.a_nom / out0.a_nom, S);

    dimless = {'a_bar_hat_out', 'Q33_out', 'Q44_out', 'R2_out', 'innov_y1_out', ...
               'innov_y2_out', 'h_bar_true_out', 'h_bar_out', 'f_bar_out', 'b_hat_out', ...
               'a_bar_Q_out', 'K_a_y1_out', 'K_a_y2_out', 'K_b_y1_out', 'K_dx_y1_out', ...
               'gate_out'};
    scaled  = {'p_true_out', S; 'p_m_out', S; 'p_d_out', S; 'delta_x_hat_3_out', S; ...
               'dx_r_out', S; 'dh_m_out', S; 'a_hat_out', S; 'a_xm_out', S; 'P_a_out', S; ...
               'f_d_out', 1; 'F_th_out', 1};

    fprintf('\n%-20s %12s %10s   %s\n', 'field', 'max rel diff', 'expected', 'verdict');
    worst = 0;
    for i = 1:numel(dimless)
        f = dimless{i};
        if ~isfield(out0, f) || ~isfield(out1, f); fprintf('%-20s (missing)\n', f); continue; end
        [rel, tag] = local_rel(out0.(f), out1.(f), 1);
        worst = max(worst, rel);
        fprintf('%-20s %12.3e %10s   %s\n', f, rel, 'x1', tag);
    end
    for i = 1:size(scaled, 1)
        f = scaled{i, 1}; fac = scaled{i, 2};
        if ~isfield(out0, f) || ~isfield(out1, f); fprintf('%-20s (missing)\n', f); continue; end
        [rel, tag] = local_rel(out0.(f), out1.(f), fac);
        worst = max(worst, rel);
        fprintf('%-20s %12.3e %10s   %s\n', f, rel, sprintf('x%g', fac), tag);
    end
    fprintf('\nworst relative difference over all families: %.3e  (criterion 1e-9)\n', worst);
    if worst < 1e-9
        fprintf('[V1] PASS -- unit-system invariance holds; no boundary leaks R or a_o.\n');
    else
        fprintf('[V1] FAIL -- see the family that misses; that is the leaking boundary.\n');
    end
end

function [rel, tag] = local_rel(x0, x1, fac)
    x0 = double(x0(:)); x1 = double(x1(:));
    if numel(x0) ~= numel(x1); rel = Inf; tag = 'SIZE MISMATCH'; return; end
    m = isfinite(x0) & isfinite(x1);
    if ~any(m); rel = 0; tag = '(all NaN)'; return; end
    d = abs(x1(m) - fac * x0(m));
    s = max(abs(fac * x0(m)));
    if s == 0; rel = max(d); else; rel = max(d) / s; end
    if rel < 1e-9; tag = 'ok'; elseif rel < 1e-6; tag = 'rounding?'; else; tag = 'LEAK'; end
end
