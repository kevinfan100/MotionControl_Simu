function gates_part2()
%GATES_PART2 Dual gates for packaging part 2 (simulation skeleton).
%
%   Equivalence gates (vs mother repo, bit-exact):
%       G1  config() params vs calc_simulation_params field-by-field,
%           INCLUDING the two randi-derived seeds (meas_noise_seed,
%           thermal.seed) under the same rng precondition -- proves the
%           RNG consumption order is replicated. (h50 + ramp2p7)
%       G2  trajectory_ref vs trajectory_generator bit-exact over the
%           full time grid (positioning + ramp), same params fed to both.
%       G3  OPEN-LOOP full simulation bit-exact: standalone run_simulation
%           vs mother run_pure_simulation, ctrl off, same seed. Compares
%           p_m / p_true / F_th / p_d / f_d time series (max|diff| = 0).
%           Mother runs with the standalone paths REMOVED (no step_dynamics
%           cross-wiring), then restored.
%
%   Theory gates:
%       T1  ramp trajectory shape: p_d starts at p0, h(t) slope
%           -(h_init-h_bottom)/T_sim, monotone, reaches h_bottom.
%       T2  open-loop Brownian increments of p_true vs closed form
%           4*kBT*Ts/(gamma*c_axis) (first 1600 steps, h_bar ~ const).
%       T3  measurement-noise injection: std(p_m - p_true) vs
%           meas_noise_std per axis.
%
%   NOTE: the d-step sensor-delay k-table check needs the controller's
%   delta_x_m log and is part of gates_part3 (stub controller ignores
%   its inputs here).

    here = fileparts(mfilename('fullpath'));
    sa_root = fileparts(here);                          % standalone/
    repo    = fileparts(sa_root);                       % mother repo root

    orig_path = path; orig_dir = pwd;
    cleanup = onCleanup(@() local_restore(orig_path, orig_dir));

    sa_dirs = {sa_root, fullfile(sa_root, 'physics'), fullfile(sa_root, 'sim')};
    mo_dirs = {fullfile(repo, 'model'), fullfile(repo, 'model', 'config'), ...
               fullfile(repo, 'model', 'wall_effect'), ...
               fullfile(repo, 'model', 'thermal_force'), ...
               fullfile(repo, 'model', 'trajectory'), ...
               fullfile(repo, 'model', 'controller'), ...
               fullfile(repo, 'model', 'dual_track')};
    addpath(sa_dirs{:});
    addpath(mo_dirs{:});

    % ================= G1: params equality (incl. derived seeds) =========
    for sc = {{'h50', 'positioning', 50, 50, 5, 1.5*2.25}, ...
              {'h10', 'positioning', 10, 10, 5, 1.5*2.25}, ...
              {'ramp2p7', 'ramp_descent', 50, 1.2*2.25, 20, 1.2*2.25}}
        s = sc{1};
        rng(1);
        params_sa = config(s{1});
        rng(1);
        cm = local_mother_config(s{2}, s{3}, s{4}, s{5}, s{6});
        Pm = calc_simulation_params(cm);
        Pm = Pm.Value;

        chk = { ...
            'common', {'Ts', 'R', 'gamma_N', 'p0'}; ...
            'wall',   {'theta', 'phi', 'pz', 'h_min', 'h_bar_min', 'w_hat', ...
                       'u_hat', 'v_hat', 'enable_wall_effect'}; ...
            'traj',   {'t_hold', 'h_init', 'h_bottom', 'amplitude', ...
                       'frequency', 'n_cycles', 't_descend_override', ...
                       'trajectory_type'}; ...
            'ctrl',   {'enable', 'lambda_c', 'gamma', 'Ts', 'k_B', 'T', ...
                       'meas_noise_enable', 'meas_noise_std', ...
                       'meas_noise_seed', 'a_pd', 'a_cov', 'sigma2_noise'}; ...
            'thermal', {'enable', 'k_B', 'T', 'Ts', 'variance_coeff', 'seed'}};
        for g = 1:size(chk, 1)
            grp = chk{g, 1};
            for f = chk{g, 2}
                fld = f{1};
                assert(isequal(params_sa.(grp).(fld), Pm.(grp).(fld)), ...
                       'G1 %s: params.%s.%s mismatch', s{1}, grp, fld);
            end
        end
        if isfield(Pm.common, 'T_sim')
            assert(isequal(params_sa.common.T_sim, Pm.common.T_sim), ...
                   'G1 %s: common.T_sim mismatch', s{1});
        end
    end
    fprintf('G1 PASS  config() == calc_simulation_params (h50 + h10 + ramp2p7, incl. both randi seeds)\n');

    % ================= G2: trajectory bit-exact =================
    for scn = {'h50', 'h10', 'ramp2p7'}
        rng(2); params_sa = config(scn{1});
        Ts = params_sa.common.Ts;
        N = round(params_sa.common.T_sim / Ts) + 1;
        clear trajectory_ref trajectory_generator;
        pd_n = zeros(3, N); dn = zeros(3, N);
        for k = 1:N, [pd_n(:, k), dn(:, k)] = trajectory_ref((k-1)*Ts, params_sa); end
        clear trajectory_ref trajectory_generator;
        pd_o = zeros(3, N); do_ = zeros(3, N);
        for k = 1:N, [pd_o(:, k), do_(:, k)] = trajectory_generator((k-1)*Ts, params_sa); end
        assert(isequal(pd_n, pd_o) && isequal(dn, do_), 'G2 %s mismatch', scn{1});
    end
    fprintf('G2 PASS  trajectory_ref bit-exact (positioning + ramp, full grid)\n');

    % ================= G3: open-loop full-sim bit-exact =================
    % Symmetric path isolation: each side runs with ONLY its own dirs on
    % the path (no step_dynamics cross-wiring in either direction).
    seed_g3 = 5;
    rmpath(mo_dirs{:});
    out_sa = run_simulation('h50', struct('seed', seed_g3, 'ctrl_enable_override', false));
    addpath(mo_dirs{:});

    rmpath(sa_dirs{:});
    cm = local_mother_config('positioning', 50, 50, 5, 1.5*2.25);
    cm.ctrl_enable = false;
    out_mo = run_pure_simulation(cm, struct('seed', seed_g3));
    addpath(sa_dirs{:});

    for f = {'p_d_out', 'f_d_out', 'F_th_out', 'p_m_out', 'p_true_out', 'tout'}
        dmax = max(abs(out_sa.(f{1}) - out_mo.(f{1})), [], 'all');
        assert(dmax == 0, 'G3 %s max|diff| = %g (not bit-exact)', f{1}, dmax);
    end
    fprintf('G3 PASS  open-loop full sim bit-exact vs mother (h50, seed %d, all series)\n', seed_g3);

    % ================= T1: ramp trajectory shape =================
    % (trajectory_ref returns p_d[k+1], so the first sample is h(0+Ts),
    %  one step below h_init -- the asserts below use that semantics)
    rng(3); params_sa = config('ramp2p7');
    Ts = params_sa.common.Ts;
    N = round(params_sa.common.T_sim / Ts) + 1;
    clear trajectory_ref;
    pd = zeros(3, N);
    for k = 1:N, pd(:, k) = trajectory_ref((k-1)*Ts, params_sa); end
    h = pd(3, :);                                       % w_hat = [0;0;1], pz = 0
    rate = (50 - 2.7) / params_sa.common.T_sim;
    assert(abs(h(1) - (50 - rate*Ts)) < 1e-12, 'T1 first sample');
    assert(all(diff(h) <= 1e-12), 'T1 monotone');
    assert(abs(h(end) - 2.7) < 1e-9, 'T1 endpoint h_bottom');
    mid = round(N/2);
    assert(abs((h(mid) - h(1))/((mid-1)*Ts) + rate) < 1e-9, 'T1 slope');
    fprintf('T1 PASS  ramp shape (slope %.4g um/s, endpoint h = 2.7 um)\n', -rate);

    % ================= T2: open-loop Brownian increments =================
    rng(4); params_sa = config('h50');                  % match out_sa's scenario
    inc = diff(out_sa.p_true_out(1:1600, :), 1, 1);     % h_bar ~ 22 segment
    k_B = params_sa.thermal.k_B; T = params_sa.thermal.T;
    gamma_N = params_sa.common.gamma_N;
    h_bar0 = params_sa.traj.h_init / params_sa.common.R;
    [ca, ce] = wall_corrections(h_bar0);
    var_expect = 4 * k_B * T * Ts ./ (gamma_N * [ca; ca; ce]);
    rel = abs(var(inc, 0, 1)' - var_expect) ./ var_expect;
    assert(all(rel < 0.10), 'T2 increment variance rel err %s', mat2str(rel', 3));
    fprintf('T2 PASS  open-loop Brownian increment variance vs 4kBT*Ts/(gamma*c) (rel err %s)\n', ...
            mat2str(rel', 2));

    % ================= T3: measurement-noise injection point =================
    nstd_emp = std(out_sa.p_m_out - out_sa.p_true_out, 0, 1)';
    nstd_cfg = params_sa.ctrl.meas_noise_std;
    rel = abs(nstd_emp - nstd_cfg) ./ nstd_cfg;
    assert(all(rel < 0.03), 'T3 noise std rel err %s', mat2str(rel', 3));
    fprintf('T3 PASS  p_m - p_true noise std vs config (rel err %s)\n', mat2str(rel', 2));

    fprintf('\n=== gates_part2: ALL PASS (G1-G3 bit-exact, T1-T3 theory) ===\n');
end


function cm = local_mother_config(traj_type, h_init, h_bottom, T_sim, h_min)
%LOCAL_MOTHER_CONFIG  Mother-repo config matching standalone config.m
%   (mirrors verify_eq17_6state section 2 + explicit h_min).
    cm = user_config();
    cm.h_init            = h_init;
    cm.h_bottom          = h_bottom;
    cm.h_min             = h_min;
    cm.amplitude         = 0;
    cm.trajectory_type   = traj_type;
    cm.T_sim             = T_sim;
    cm.ctrl_enable       = true;
    cm.meas_noise_enable = true;
    cm.thermal_enable    = true;
    cm.lambda_c          = 0.7;
    cm.a_pd              = 0.05;
    cm.a_cov             = 0.05;
    cm.meas_noise_std    = [0.00062; 0.00057; 0.00331];
    cm.eq17_variant      = '6state';
end


function local_restore(orig_path, orig_dir)
%LOCAL_RESTORE  Undo path/cwd mutations on exit (incl. on error).
    path(orig_path);
    cd(orig_dir);
end
