function gates_part1()
%GATES_PART1 Dual gates for packaging part 1 (physics world).
%
%   Equivalence gates (vs mother repo, bit-exact, max|diff| = 0):
%       E1  wall_corrections  vs calc_correction_functions  (grid + derivs)
%       E2  gamma_inv         vs calc_gamma_inv             (grid positions)
%       E3  thermal_force     vs calc_thermal_force         (same rng -> same sequence)
%       E4  step_dynamics(SA) vs step_dynamics(mother)      (fixed inputs)
%
%   Theory gates (construction correctness, closed forms):
%       T1  analytic dc/dh_bar, K_h' vs central finite differences (< 1e-5)
%       T2  Gamma_inv symmetric, positive definite, eigenvalues
%           {1/(gamma*c_para) x2, 1/(gamma*c_perp)}
%       T3  thermal per-axis variance vs 4*kBT*gamma*c/Ts  (< 3%, 2e5 samples)
%       T4  free-space MSD ratio vs Einstein 2*D*Ts == 2.0 (one-sided convention)
%       T5  h_bar -> inf limit: c_para, c_perp -> 1
%
%   Errors out on first failure; prints PASS per gate otherwise.

    here = fileparts(mfilename('fullpath'));
    sa_root = fileparts(here);                          % standalone/
    repo    = fileparts(sa_root);                       % mother repo root

    % Restore path + cwd on exit: standalone/sim shadows the mother repo's
    % step_dynamics while this script's paths are active -- must not leak
    % into the rest of the MATLAB session.
    orig_path = path; orig_dir = pwd;
    cleanup = onCleanup(@() local_restore(orig_path, orig_dir));

    addpath(fullfile(sa_root, 'physics'), fullfile(sa_root, 'sim'), ...
            fullfile(repo, 'model', 'wall_effect'), ...
            fullfile(repo, 'model', 'thermal_force'), ...
            fullfile(repo, 'model', 'dual_track'));

    % --- Shared physical constants (project values) ---
    R       = 2.25;            % [um]
    gamma_N = 0.0425;          % [pN*sec/um]
    k_B     = 1.3806503e-5;    % [pN*um/K]
    T       = 310.15;          % [K]
    Ts      = 1 / 1600;        % [sec]

    params = struct();
    params.wall = struct('w_hat', [0;0;1], 'u_hat', [1;0;0], 'v_hat', [0;1;0], ...
                         'pz', 0, 'enable_wall_effect', 1);
    params.common  = struct('R', R, 'gamma_N', gamma_N);
    params.thermal = struct('k_B', k_B, 'T', T, 'Ts', Ts, 'seed', 42);

    hb_grid = [1.05, 1.2, 1.5, 2, 3, 5, 10, 22.2, 50];

    % ================= E1: wall_corrections bit-exact =================
    for hb = hb_grid
        [ca_n, ce_n, dn] = wall_corrections(hb, true);
        [ca_o, ce_o, do] = calc_correction_functions(hb, true);
        assert(ca_n == ca_o && ce_n == ce_o, 'E1 c mismatch at h_bar=%g', hb);
        fn = fieldnames(dn);
        for i = 1:numel(fn)
            assert(dn.(fn{i}) == do.(fn{i}), 'E1 derivs.%s mismatch at h_bar=%g', fn{i}, hb);
        end
    end
    fprintf('E1 PASS  wall_corrections bit-exact (%d grid points, all derivs)\n', numel(hb_grid));

    % ================= E2: gamma_inv bit-exact =================
    for hb = hb_grid
        p = [0.3; -0.2; hb * R];
        [Gn, hn] = gamma_inv(p, params);
        [Go, ho] = calc_gamma_inv(p, params);
        assert(isequal(Gn, Go) && hn == ho, 'E2 mismatch at h_bar=%g', hb);
    end
    pw = [0.3; -0.2; 5]; pf = params; pf.wall.enable_wall_effect = 0;
    assert(isequal(gamma_inv(pw, pf), calc_gamma_inv(pw, pf)), 'E2 free-space mismatch');
    fprintf('E2 PASS  gamma_inv bit-exact (%d points + free space)\n', numel(hb_grid));

    % ================= E3: thermal_force bit-exact sequence =================
    % NOTE: the stream is set by the in-function seed chain itself -- the
    % first call after `clear` reseeds the global rng with
    % params.thermal.seed (= 42). No external rng() call needed.
    Nseq = 1000;
    p_th = [0; 0; 2 * R];
    clear thermal_force calc_thermal_force;
    seq_n = zeros(3, Nseq);
    for k = 1:Nseq, seq_n(:, k) = thermal_force(p_th, params); end
    clear thermal_force calc_thermal_force;
    seq_o = zeros(3, Nseq);
    for k = 1:Nseq, seq_o(:, k) = calc_thermal_force(p_th, params); end
    assert(isequal(seq_n, seq_o), 'E3 thermal sequence mismatch');
    fprintf('E3 PASS  thermal_force bit-exact (%d samples, seed chain incl.)\n', Nseq);

    % ================= E4: step_dynamics bit-exact =================
    cd(fullfile(sa_root, 'sim'));            fh_new = @step_dynamics;
    cd(fullfile(repo, 'model', 'dual_track')); fh_old = @step_dynamics;
    cd(here);
    F = [0.8; -1.1; 0.4];
    for hb = [1.2, 2, 22.2]
        p0 = [0.5; -0.3; hb * R];
        pn = fh_new(p0, F, params, Ts);
        po = fh_old(p0, F, params, Ts);
        assert(isequal(pn, po), 'E4 step_dynamics mismatch at h_bar=%g', hb);
    end
    fprintf('E4 PASS  step_dynamics bit-exact (h_bar = 1.2 / 2 / 22.2)\n');

    % ================= T1: derivatives vs finite differences =================
    tol_fd = 1e-5;
    for hb = hb_grid
        [~, ~, d0] = wall_corrections(hb, true);
        eps_h = 1e-6 * hb;
        [cap, cep] = wall_corrections(hb + eps_h);
        [cam, cem] = wall_corrections(hb - eps_h);
        fd_ca = (cap - cam) / (2 * eps_h);
        fd_ce = (cep - cem) / (2 * eps_h);
        assert(abs(fd_ca - d0.dc_para_dh) / abs(d0.dc_para_dh) < tol_fd, 'T1 dc_para h=%g', hb);
        assert(abs(fd_ce - d0.dc_perp_dh) / abs(d0.dc_perp_dh) < tol_fd, 'T1 dc_perp h=%g', hb);
        [~, ~, dp] = wall_corrections(hb + eps_h, true);
        [~, ~, dm] = wall_corrections(hb - eps_h, true);
        fd_khp_para = (dp.K_h_para - dm.K_h_para) / (2 * eps_h);
        fd_khp_perp = (dp.K_h_perp - dm.K_h_perp) / (2 * eps_h);
        assert(abs(fd_khp_para - d0.K_h_prime_para) / abs(d0.K_h_prime_para) < tol_fd, 'T1 Kh''para h=%g', hb);
        assert(abs(fd_khp_perp - d0.K_h_prime_perp) / abs(d0.K_h_prime_perp) < tol_fd, 'T1 Kh''perp h=%g', hb);
    end
    fprintf('T1 PASS  analytic derivatives vs FD (rel err < %g)\n', tol_fd);

    % ================= T2: Gamma_inv structure =================
    for hb = hb_grid
        p = [0; 0; hb * R];
        Gi = gamma_inv(p, params);
        assert(max(abs(Gi - Gi'), [], 'all') <= 1e-15 * norm(Gi), 'T2 symmetry h=%g', hb);
        ev = sort(eig(Gi));
        [ca, ce] = wall_corrections(hb);
        ev_expect = sort([1/(gamma_N*ca); 1/(gamma_N*ca); 1/(gamma_N*ce)]);
        assert(all(ev > 0), 'T2 PD h=%g', hb);
        assert(max(abs(ev - ev_expect) ./ ev_expect) < 1e-12, 'T2 eigenvalues h=%g', hb);
    end
    fprintf('T2 PASS  Gamma_inv symmetric / PD / eigenvalues {1/(g*c_para) x2, 1/(g*c_perp)}\n');

    % ================= T3: thermal variance vs closed form =================
    Mv = 2e5;
    pT3 = params; pT3.thermal.seed = 11;     % distinct stream for this gate
    clear thermal_force;
    S = zeros(3, Mv);
    for k = 1:Mv, S(:, k) = thermal_force(p_th, pT3); end
    [ca2, ce2] = wall_corrections(2);
    var_expect = (4 * k_B * T * gamma_N / Ts) * [ca2; ca2; ce2];
    var_emp = var(S, 0, 2);
    rel = abs(var_emp - var_expect) ./ var_expect;
    assert(all(rel < 0.03), 'T3 variance rel err %s', mat2str(rel', 3));
    fprintf('T3 PASS  thermal variance vs 4kBT*gamma*c/Ts (rel err < 3%%: %s)\n', mat2str(rel', 2));

    % ================= T4: free-space MSD ratio = 2.0 =================
    Nmsd = 2e4;
    pf = params; pf.wall.enable_wall_effect = 0;
    pf.thermal.seed = 13;                    % distinct stream for this gate
    clear thermal_force;
    pos = zeros(3, Nmsd + 1);
    for k = 1:Nmsd
        f_th = thermal_force(pos(:, k), pf);
        pos(:, k+1) = fh_new(pos(:, k), f_th, pf, Ts);
    end
    msd = mean(diff(pos, 1, 2).^2, 2);
    D = k_B * T / gamma_N;
    ratio = msd / (2 * D * Ts);
    assert(all(ratio > 1.9 & ratio < 2.1), 'T4 MSD ratio %s', mat2str(ratio', 3));
    fprintf('T4 PASS  free-space MSD ratio vs Einstein 2D*Ts = %s (convention fingerprint 2.0)\n', ...
            mat2str(ratio', 3));

    % ================= T5: far-field limit =================
    [ca_inf, ce_inf] = wall_corrections(1e6);
    assert(abs(ca_inf - 1) < 1e-5 && abs(ce_inf - 1) < 1e-5, 'T5 far-field limit');
    fprintf('T5 PASS  h_bar -> inf: c_para, c_perp -> 1\n');

    % ================= T6: randomness quality =================
    % E3 proves reproducibility, T3/T4 prove magnitude; T6 checks the
    % stochastic QUALITY of the thermal stream: whiteness (lags 1-10),
    % cross-axis independence, Gaussianity (moments), and independence
    % of different driver seeds. All bounds = 4 sigma of the estimator.
    Nrq = 1e5;
    pq = params; pq.thermal.seed = 17;
    clear thermal_force;
    Sq = zeros(3, Nrq);
    for k = 1:Nrq, Sq(:, k) = thermal_force(p_th, pq); end
    Z = (Sq - mean(Sq, 2)) ./ std(Sq, 0, 2);

    tol_corr = 4 / sqrt(Nrq);
    max_rho = 0;
    for ax = 1:3
        z = Z(ax, :); den = sum(z.^2);
        for lag = 1:10
            rho = sum(z(1:end-lag) .* z(1+lag:end)) / den;
            max_rho = max(max_rho, abs(rho));
            assert(abs(rho) < tol_corr, 'T6 autocorr ax=%d lag=%d rho=%g', ax, lag, rho);
        end
    end
    Cx = corrcoef(Z'); off = Cx - eye(3);
    assert(max(abs(off(:))) < tol_corr, 'T6 cross-axis corr %g', max(abs(off(:))));
    skew = mean(Z.^3, 2); kurt = mean(Z.^4, 2);
    assert(all(abs(skew) < 4*sqrt(6/Nrq)), 'T6 skewness %s', mat2str(skew', 3));
    assert(all(abs(kurt - 3) < 4*sqrt(24/Nrq)), 'T6 kurtosis %s', mat2str(kurt', 3));
    pq2 = params; pq2.thermal.seed = 18;
    clear thermal_force;
    Sq2 = zeros(3, Nrq);
    for k = 1:Nrq, Sq2(:, k) = thermal_force(p_th, pq2); end
    for ax = 1:3
        cc = corrcoef(Sq(ax, :), Sq2(ax, :));
        assert(abs(cc(1, 2)) < tol_corr, 'T6 seed-independence ax=%d corr=%g', ax, cc(1, 2));
    end
    fprintf(['T6 PASS  whiteness (max|rho| %.4f < %.4f), cross-axis indep, ' ...
             'Gaussian moments, seed independence\n'], max_rho, tol_corr);

    fprintf('\n=== gates_part1: ALL PASS (E1-E4 bit-exact, T1-T6 theory) ===\n');
end


function local_restore(orig_path, orig_dir)
%LOCAL_RESTORE  Undo the path/cwd mutations on exit (incl. on error).
    path(orig_path);
    cd(orig_dir);
end
