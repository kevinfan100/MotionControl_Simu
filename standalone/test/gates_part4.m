function gates_part4()
%GATES_PART4 Dual gates for packaging part 4 (Eq.17 control law).
%
%   D1  Perfect-knowledge lambda_c decay (theory gate, EXACT algebra):
%       drive the controller against the paper's own discrete plant
%       x[k+1] = x[k] + a_true*f_d[k] with a d=2 delayed noise-free
%       measurement, a_hat == a_true (the frozen wall-aware seed) and
%       xD = 0. Paper Eq.18: the ideal closed loop is
%           delta_x[k+1] = lambda_c * delta_x[k]
%       EXACTLY -- including through the d=2 delay, which is the whole
%       point of the Eq.17 Sigma f_d compensation. Asserts the geometric
%       ratio == lambda_c to 1e-9 over the transient. This pins the
%       control-law algebra (bracket terms, ACTIVE weighting, buffer
%       timing) end-to-end.
%   D2  First closed-loop smoke (h50, thermal + noise ON, frozen
%       perfect-knowledge a_hat): tracking std < 40 nm per axis.
%
%   Closed-loop equivalence vs the mother repo arrives in part 7 (needs
%   the EKF: mother's a_hat is estimated, ours is frozen until then).

    here = fileparts(mfilename('fullpath'));
    sa_root = fileparts(here);

    orig_path = path; orig_dir = pwd;
    cleanup = onCleanup(@() local_restore(orig_path, orig_dir));

    addpath(sa_root, fullfile(sa_root, 'physics'), fullfile(sa_root, 'sim'));

    % ================= D1: perfect-knowledge lambda_c decay =================
    clear controller_6state;
    rng(31); params = config('h50');
    lc = params.ctrl.lambda_c;
    Ts = params.common.Ts;
    a_nom = Ts / params.common.gamma_N;
    hb0 = params.traj.h_init / params.common.R;
    [cpa0, cpe0] = wall_corrections(hb0);
    a_true = [a_nom / cpa0; a_nom / cpa0; a_nom / cpe0];  % == frozen a_x_init
    % (bit-identity relies on config aliasing ctrl.Ts == common.Ts and
    %  ctrl.gamma == common.gamma_N, and both sides calling the same
    %  wall_corrections at the same exact h_bar -- guarded by the D1 assert)

    pd_const = params.common.p0;
    e0 = 1.0;                                  % [um] initial z tracking error
    Nd = 40;
    xs = zeros(3, Nd + 1);
    xs(:, 1) = pd_const - [0; 0; e0];          % particle starts e0 below target

    for k = 1:Nd
        pm_k = xs(:, max(k - 2, 1));           % d=2 delayed, noise-free; IC x0
        fd_k = controller_6state(zeros(3, 1), pd_const, pm_k, params);
        xs(:, k + 1) = xs(:, k) + a_true .* fd_k;   % paper discrete plant
    end
    dx = pd_const(3) - xs(3, :);               % delta_x_z[k], k = 1..Nd+1

    % k = 1 is the controller init step (f_d = 0 -> ratio 1); the lambda_c
    % recursion holds from k = 2 on (verified by hand: consistent ICs).
    % Window endpoint: the dominant error is a FIXED absolute fp residual
    % (~1e-14 um from 50-um-scale cancellation), so the ratio deviation
    % grows as 1/delta[k] ~ lc^-k. Stopping at dx(31) (delta ~ 4.6e-5 um)
    % keeps dev ~ 1e-10 < tol 1e-9; extending the window would flake.
    ratios = dx(3:31) ./ dx(2:30);
    dev = max(abs(ratios - lc));
    assert(dev < 1e-9, 'D1 lambda_c decay: max|ratio - lc| = %.3e', dev);
    % x/y axes start with zero error and must stay exactly at zero force
    assert(all(all(xs(1:2, :) == xs(1:2, 1))), 'D1 x/y axes moved');
    fprintf('D1 PASS  perfect-knowledge decay: ratio == lambda_c (max dev %.2e over 29 steps)\n', dev);

    % ================= D2: first closed-loop smoke (h50, full noise) ========
    out = run_simulation('h50', struct('seed', 3, 'T_sim', 2));
    n_warmup = round(0.5 / Ts);
    idx = (n_warmup + 1):numel(out.tout);
    trk = std(out.p_d_out(idx, :) - out.p_m_out(idx, :), 0, 1) * 1e3;   % [nm]
    assert(all(trk < 40), 'D2 tracking std %s nm exceeds 40 nm', mat2str(trk, 4));
    assert(all(isfinite(out.ekf_out(:, 4))) && abs(out.ekf_out(end, 4) - 22.2) < 1, ...
           'D2 h_bar probe off');
    fprintf('D2 PASS  first closed loop: tracking std [%.1f %.1f %.1f] nm (frozen perfect a_hat)\n', trk);

    fprintf('\n=== gates_part4: ALL PASS (D1 exact lambda_c algebra, D2 closed-loop smoke) ===\n');
end


function local_restore(orig_path, orig_dir)
%LOCAL_RESTORE  Undo path/cwd mutations on exit (incl. on error).
    path(orig_path);
    cd(orig_dir);
end
