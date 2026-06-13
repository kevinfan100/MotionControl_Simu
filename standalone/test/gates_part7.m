function gates_part7()
%GATES_PART7 Closed-loop equivalence vs the mother repo (packaging part 7).
%
%   The headline equivalence gate: the standalone closed loop reproduces
%   the mother repo (feat/eq17-6state post-D6) to FLOATING-POINT ROUNDING
%   PRECISION wherever the mother's guards do not fire. It is NOT bit-exact
%   (max|diff|=0): the standalone is a clean re-implementation (loop
%   fusion, inlined constants, guards removed), and mathematically-
%   identical refactors differ in IEEE-754 association by ~1 ulp per step.
%   At h50/h10 the closed loop is contractive, so that rounding does NOT
%   amplify -- it stays at ~3 ulps over the whole 8000-step run, 9 orders
%   of magnitude below the ~30 nm tracking precision. Threshold: rel < 1e-12.
%
%   H1  h50 closed loop, 3 seeds: standalone run_simulation vs mother
%       run_pure_simulation, world-coord series (f_d/p_m/p_true/p_d/F_th)
%       agree to rel < 1e-12. (Mother guards never fire at h_bar = 22.2.)
%   H2  h10 closed loop, 3 seeds: same (h_bar = 4.44, guards never fire).
%   H3  ramp 50 -> 2.7 um (h_bar 22.2 -> 1.2): the package vs mother
%       boundary demonstration. For h_bar > 1.5 (mother guards OFF) the two
%       are rounding-floor equivalent; for h_bar < 1.5 the mother's G3
%       gates y_2 off (1D update) while the standalone keeps the full 2D
%       update, so a_hat DELIBERATELY diverges there -- the documented L2
%       limitation. The standalone stays numerically stable and tracking
%       bounded throughout. This pins WHERE and WHY the package differs.
%
%   Path isolation: each side runs with ONLY its own dirs on the path
%   (no step_dynamics / run_simulation cross-wiring), restored on exit.

    here = fileparts(mfilename('fullpath'));
    sa_root = fileparts(here);
    repo    = fileparts(sa_root);

    orig_path = path; orig_dir = pwd;
    cleanup = onCleanup(@() local_restore(orig_path, orig_dir));

    sa_dirs = {sa_root, fullfile(sa_root, 'physics'), fullfile(sa_root, 'sim')};
    mo_dirs = {fullfile(repo, 'model'), fullfile(repo, 'model', 'config'), ...
               fullfile(repo, 'model', 'wall_effect'), ...
               fullfile(repo, 'model', 'thermal_force'), ...
               fullfile(repo, 'model', 'trajectory'), ...
               fullfile(repo, 'model', 'controller'), ...
               fullfile(repo, 'model', 'dual_track')};

    tol = 1e-12;
    wflds = {'f_d_out', 'p_m_out', 'p_true_out', 'p_d_out', 'F_th_out'};

    % a_hat axis-reorder: mother ekf_out is [a_x; a_z; a_y; h_bar] (cols
    % [1 3 2 4]); standalone is [a_x; a_y; a_z; h_bar]. Reorder mother to
    % standalone order for a direct a_hat comparison.
    mo2sa = [1 3 2 4];

    % ================= H1: h50 closed loop, 3 seeds =================
    maxrel_h1 = 0;
    for seed = 1:3
        [osa, omo] = run_pair(sa_dirs, mo_dirs, 'h50', 'positioning', 50, 50, 1.5*2.25, 5, seed);
        maxrel_h1 = max(maxrel_h1, compare_fields(osa, omo, wflds, mo2sa, tol, 'H1', seed));
    end
    fprintf('H1 PASS  h50 closed loop vs mother (3 seeds): rounding-floor equiv, max rel = %.2e\n', maxrel_h1);

    % ================= H2: h10 closed loop, 3 seeds =================
    maxrel_h2 = 0;
    for seed = 1:3
        [osa, omo] = run_pair(sa_dirs, mo_dirs, 'h10', 'positioning', 10, 10, 1.5*2.25, 5, seed);
        maxrel_h2 = max(maxrel_h2, compare_fields(osa, omo, wflds, mo2sa, tol, 'H2', seed));
    end
    fprintf('H2 PASS  h10 closed loop vs mother (3 seeds): rounding-floor equiv, max rel = %.2e\n', maxrel_h2);

    % ================= H3: ramp boundary demonstration =================
    [osa, omo] = run_pair(sa_dirs, mo_dirs, 'ramp2p7', 'ramp_descent', 50, 1.2*2.25, 1.2*2.25, 20, 1);
    % Use the MORE CONSERVATIVE of the two h_bar probes for the safe mask
    % (both agree to rounding floor; min() avoids misclassifying the single
    % boundary step where one is just above 1.5 and the other just below).
    hb = min(osa.ekf_out(:, 4), omo.ekf_out(:, 4));
    safe = hb > 1.5;  near = hb < 1.5;

    % (a) safe segment (h_bar > 1.5, guards OFF): rounding-floor equivalent
    maxrel_safe = 0;
    for f = {'f_d_out', 'p_m_out'}
        d = abs(osa.(f{1})(safe, :) - omo.(f{1})(safe, :));
        rel = max(d, [], 'all') / max(max(abs(omo.(f{1})(safe, :)), [], 'all'), eps);
        maxrel_safe = max(maxrel_safe, rel);
    end
    assert(maxrel_safe < tol, 'H3 safe-segment rel = %.3e (expected rounding floor)', maxrel_safe);

    % (b) standalone numerically stable + tracking bounded throughout
    assert(all(isfinite(osa.p_m_out(:))) && all(osa.ekf_out(:, 1:3) > 0, 'all'), ...
           'H3 standalone unstable on ramp');
    n_warm = round(0.5 / osa.meta.params.common.Ts);
    trk = std(osa.p_d_out(n_warm+1:end, :) - osa.p_m_out(n_warm+1:end, :), 0, 1) * 1e3;
    assert(all(trk < 40), 'H3 standalone tracking %s nm', mat2str(trk, 4));

    % (c) near-wall (h_bar < 1.5): a_hat DELIBERATELY diverges (mother gates
    %     y_2, standalone does not) -- descriptive, the documented L2 boundary
    az_sa = osa.ekf_out(near, 3);                 % standalone a_hat_z
    az_mo = omo.ekf_out(near, 2);                 % mother a_hat_z (col order x,z,y)
    if any(near)
        nearrel = max(abs(az_sa - az_mo)) / max(mean(abs(az_mo)), eps);
        fprintf(['H3 PASS  ramp boundary: h_bar>1.5 rounding-floor equiv (rel %.2e); ' ...
                 'h_bar<1.5 a_hat_z diverges rel %.2e (mother gates, pkg does not -- L2); ' ...
                 'standalone stable, tracking [%.1f %.1f %.1f] nm\n'], ...
                maxrel_safe, nearrel, trk);
    else
        fprintf(['H3 PASS  ramp: h_bar>1.5 rounding-floor equiv (rel %.2e); ' ...
                 'no h_bar<1.5 samples; standalone tracking [%.1f %.1f %.1f] nm\n'], ...
                maxrel_safe, trk);
    end

    fprintf('\n=== gates_part7: ALL PASS (H1 h50, H2 h10 closed-loop equiv; H3 ramp L2 boundary) ===\n');
end


function maxrel = compare_fields(osa, omo, wflds, mo2sa, tol, tag, seed)
%COMPARE_FIELDS  World-coord series + a_hat (axis-reordered) rounding-floor
%   equivalence. a_hat is checked directly (not only transitively via f_d).
    maxrel = 0;
    for f = wflds
        rel = max(abs(osa.(f{1}) - omo.(f{1})), [], 'all') / ...
              max(max(abs(omo.(f{1})), [], 'all'), eps);
        maxrel = max(maxrel, rel);
        assert(rel < tol, '%s %s seed %d rel = %.3e', tag, f{1}, seed, rel);
    end
    % a_hat (cols 1:3) -- reorder mother [a_x a_z a_y] to standalone order
    omo_ah = omo.ekf_out(:, mo2sa);
    rel_ah = max(abs(osa.ekf_out(:, 1:3) - omo_ah(:, 1:3)), [], 'all') / ...
             max(max(abs(omo_ah(:, 1:3)), [], 'all'), eps);
    maxrel = max(maxrel, rel_ah);
    assert(rel_ah < tol, '%s a_hat seed %d rel = %.3e', tag, seed, rel_ah);
end


function [osa, omo] = run_pair(sa_dirs, mo_dirs, scn, traj, h_init, h_bottom, h_min, T_sim, seed)
%RUN_PAIR  Run the standalone and mother closed loops with symmetric path
%   isolation (each side sees only its own dirs).
    addpath(sa_dirs{:});
    osa = run_simulation(scn, struct('seed', seed));
    rmpath(sa_dirs{:});

    addpath(mo_dirs{:});
    cm = user_config();
    % Scenario + tunables (mirror standalone config.m)
    cm.h_init = h_init; cm.h_bottom = h_bottom; cm.h_min = h_min;
    cm.amplitude = 0; cm.trajectory_type = traj; cm.T_sim = T_sim;
    cm.ctrl_enable = true; cm.meas_noise_enable = true; cm.thermal_enable = true;
    cm.lambda_c = 0.7; cm.a_pd = 0.05; cm.a_cov = 0.05;
    cm.meas_noise_std = [0.00062; 0.00057; 0.00331]; cm.eq17_variant = '6state';
    % Load-bearing fields set EXPLICITLY (do not inherit user_config defaults,
    % so a future user_config.m edit cannot silently desync this comparison).
    cm.t_hold = 0.5; cm.enable_wall_effect = true;
    cm.theta = 0; cm.phi = 0; cm.pz = 0;
    cm.iir_warmup_mode = 'prefill'; cm.t_warmup_kf = 0;
    omo = run_pure_simulation(cm, struct('seed', seed, 'collect_diag', false));
    rmpath(mo_dirs{:});
end


function local_restore(orig_path, orig_dir)
%LOCAL_RESTORE  Undo path/cwd mutations on exit (incl. on error).
    path(orig_path);
    cd(orig_dir);
end
