% l4_norm_v2_kappaT_plant.m -- normalization-boundary audit, arm V2 (2026-08-30)
% PURPOSE: kappa_T = 4*kB*T*a_o/R (formC_b.m:359) must equal the plant's realised
%          one-step thermal displacement variance / R^2 at far field, and scale as
%          a_bar = 1/c near the wall. Checks the factor 4 and the placement of c;
%          says nothing about the controller wiring (that is arm V1).
% PRE-REGISTERED CRITERIA (written before running):
%   far field (h_bar = 50):  Var/R^2 / kappa_T        = 1 +- 0.5%   (N = 1e5 -> 1 sigma ~0.45%)
%   in band  (3.32, 1.10):   Var/R^2 / (kappa_T*a_bar) = 1 +- 1%    per axis, a_bar = 1/c_para (x,y), 1/c_perp (z)
%   the wrong-a_o variant (a_nom = Ts/gamma_N in place of a_o) must land at R = 2.25 -> ratio 0.444, so the
%   test is sensitive to the R vs R^2 question.
% Uses the same calc_thermal_force / calc_gamma_inv the plant integrates with, so the
% ratio isolates the container formula, not the sampling scheme.
function l4_norm_v2_kappaT_plant()
    here = fileparts(mfilename('fullpath'));
    root = fullfile(here, '..', '..');
    addpath(genpath(fullfile(root, 'model')));

    cfg = canonical_scenario(0.05, 1.1, 'deep');      % driver constants A_COV_BASE, H_BAR_MIN_PRIOR
    P   = calc_simulation_params(cfg);
    if isa(P, 'Simulink.Parameter'); P = P.Value; end   % driver does the same (run_formC_b.m:966)

    R   = P.common.R;  gN = P.common.gamma_N;  Ts = P.common.Ts;
    kBT = P.thermal.k_B * P.thermal.T;
    a_nom   = Ts / gN;                % [um/pN]
    a_o     = Ts / (gN * R);          % [1/pN]   formC_b.m:357
    kappa_T = 4 * kBT * a_o / R;      % [-]      formC_b.m:359
    kappa_wrong = 4 * kBT * a_nom / R;  % what "a_o = Ts/gamma_N" would give with the same /R line

    fprintf('R = %.3f um, Ts = %.4e s, gamma_N = %.4f pN s/um, kBT = %.4e pN um\n', R, Ts, gN, kBT);
    fprintf('a_nom = %.5e um/pN | a_o = %.5e 1/pN | f_R = 1/a_o = %.2f pN | a_disp = a_o*R = %.5e um/pN\n', ...
            a_nom, a_o, 1/a_o, a_o*R);
    fprintf('kappa_T = %.5e [-]  -> sd/step = %.4e R = %.2f nm ; Einstein 4*D*Ts/R^2 = %.5e (D = kBT/gamma_N)\n', ...
            kappa_T, sqrt(kappa_T), sqrt(kappa_T)*R*1e3, 4*(kBT/gN)*Ts/R^2);
    fprintf('(if a_o were Ts/gamma_N with the same /R: kappa = %.5e, ratio expected %.3f)\n\n', ...
            kappa_wrong, kappa_T/kappa_wrong);

    w_hat = P.wall.w_hat;  pz = P.wall.pz;
    N = 1e5;
    rng(20260830);
    P.thermal.seed = 20260830;   % calc_thermal_force seeds itself once (persistent)
    fprintf('%8s %9s %9s | %11s %11s %11s | %8s %8s %8s\n', 'h_bar', 'c_para', 'c_perp', ...
            'VarX/R^2', 'VarY/R^2', 'VarZ/R^2', 'rX', 'rY', 'rZ');
    for hb = [50, 3.32, 1.10]
        p = (pz + hb * R) * w_hat;
        [c_para, c_perp] = calc_correction_functions(hb);
        if isfield(P.wall, 'plant_cperp') && ~isempty(P.wall.plant_cperp)
            c_perp = P.wall.plant_cperp(hb);
        end
        F = zeros(3, N);
        for k = 1:N
            F(:, k) = calc_thermal_force(p, P);
        end
        Ginv = calc_gamma_inv(p, P);           % same mobility the plant integrates with
        D = Ginv * F * Ts;                     % one-step thermal displacement [um]
        v = var(D, 0, 2) / R^2;                % [-]
        a_bar = [1/c_para; 1/c_para; 1/c_perp];
        ratio = v ./ (kappa_T * a_bar);
        fprintf('%8.2f %9.4f %9.4f | %11.4e %11.4e %11.4e | %8.4f %8.4f %8.4f\n', ...
                hb, c_para, c_perp, v, ratio);
    end
    fprintf('\nsampling 1-sigma on a variance ratio at N = %g: %.2f%%\n', N, 100*sqrt(2/N));
end
