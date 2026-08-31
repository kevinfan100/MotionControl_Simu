function out = capture_loop_matrices(seed)
%CAPTURE_LOOP_MATRICES  Step 1 of the closed-loop E[nu] derivation instrument.
%   Runs one canonical seed (b_true + exact step, noisy) with obs_dump on,
%   collects the per-step linearisation (F_e, H1, H2, R, P_pred, P_upd,
%   x_pred, x_upd) for the z axis, REBUILDS the Kalman gains from
%   P_pred H' / (H P_pred H' + R) with the same sequential y1-then-y2 order the
%   filter uses, and VALIDATES the rebuild against the run itself:
%       (a) gain identity      max |K1_rebuilt(4) - K_a_y1_out|
%       (b) update identity    max |x_upd - x_pred - K1*nu1 - K2*nu2|
%       (c) covariance identity max |P_upd - (I-K2H2)(I-K1H1)P_pred| (sym part)
%   The instrument is only usable if (a)-(c) pass; the propagation script
%   (propagate_mean_forcing.m) refuses a capture without the PASS stamp.
% STATUS: ACTIVE | closed-loop mean-bias derivation line (2026-08-31)

    if nargin < 1 || isempty(seed); seed = 7; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'loop_mean_bias');
    if ~exist(od, 'dir'); mkdir(od); end
    ax = 3;
    r0 = 0;  try r0 = obs_dump('resets'); catch; end  %#ok<CTCH>  % arming count BEFORE this run (persists across sequential captures)
    O = run_formC_b(struct('arm', 'best', 'ap_src', 'post', 'seeds', seed, 'verbose', false, ...
                           'b_true', true, 'b_true_at', 'cmd', ...
                           'ctrl_const_override', struct('law_exact_step', true, 'obs_dump', true)));
    r = O.runs{1};
    L = obs_dump('get');
    assert(obs_dump('resets') - r0 == 1, 'buffer armed %d times during this capture; run exactly one seed', obs_dump('resets') - r0);
    Lz = L([L.ax] == ax);
    n = numel(Lz);  ns = numel(Lz(1).x_pred);
    fprintf('captured %d z-axis steps, state dim %d\n', n, ns);

    F  = zeros(ns, ns, n);  A  = zeros(ns, ns, n);
    K1 = zeros(ns, n);      K2 = zeros(ns, n);  H2S = zeros(ns, n);
    XP = zeros(ns, n);      XU = zeros(ns, n);  KK = zeros(1, n);  GT = false(1, n);
    e_gain = 0;  e_upd = 0;  e_cov = 0;
    for i = 1:n
        s = Lz(i);
        F(:,:,i) = s.F;  XP(:,i) = s.x_pred;  XU(:,i) = s.x_upd;  KK(i) = s.k;  GT(i) = s.gate;
        H1 = s.H{1};  H2 = s.H{2};  P = s.P_pred;
        S1 = H1 * P * H1.' + s.R(1);
        k1 = (P * H1.') / S1;
        P1 = (eye(ns) - k1 * H1) * P;
        I2 = eye(ns);
        if ~isempty(H2) && ~s.gate
            S2 = H2 * P1 * H2.' + s.R(2);
            k2 = (P1 * H2.') / S2;
            I2 = eye(ns) - k2 * H2;
            P2 = I2 * P1;
        else
            k2 = zeros(ns, 1);  P2 = P1;  H2 = zeros(1, ns);
        end
        K1(:,i) = k1;  K2(:,i) = k2;  H2S(:,i) = H2(:);
        A(:,:,i) = I2 * (eye(ns) - k1 * H1) * F(:,:,i);      % loop matrix e_upd -> e_upd (one step)
        % validations. LOG_OFFSET: obs_dump's k_step is one BEHIND the driver's
        % log row (found 2026-08-31: offset +1 makes both identities exactly 0).
        kr = s.k + 1;
        if kr > size(r.innov_y1_out, 1); continue; end
        e_gain = max(e_gain, abs(k1(4) - r.K_a_y1_out(kr, ax)));
        nu1 = r.innov_y1_out(kr, ax);  nu2 = r.innov_y2_out(kr, ax);
        if ~isfinite(nu2); nu2 = 0; end
        e_upd = max(e_upd, max(abs(s.x_upd - (s.x_pred + k1 * nu1 + k2 * nu2))));
        e_cov = max(e_cov, max(abs(s.P_upd - P2), [], 'all') / max(abs(s.P_upd), [], 'all'));
    end
    ok = e_gain < 1e-9 && e_upd < 1e-9 && e_cov < 1e-6;
    fprintf('[a] gain identity   max %.2e\n[b] update identity max %.2e\n[c] cov identity    max %.2e (rel)\n=> %s\n', ...
            e_gain, e_upd, e_cov, string(ternary(ok, 'PASS -- instrument usable', 'FAIL -- do not propagate')));
    LOG_OFFSET = 1;  KK = KK + LOG_OFFSET;      % KK now indexes the DRIVER's log rows
    t = r.tout(min(KK, numel(r.tout)));
    save(fullfile(od, sprintf('loop_capture_seed%d.mat', seed)), ...
         'F', 'A', 'K1', 'K2', 'H2S', 'XP', 'XU', 'KK', 'GT', 't', 'seed', 'ok', 'LOG_OFFSET', '-v7.3');
    out = struct('n', n, 'ok', ok, 'e', [e_gain e_upd e_cov], 'file', fullfile(od, sprintf('loop_capture_seed%d.mat', seed)));
    % keep the paired run log too (innovations, states, truths)
    run_log = r; %#ok<NASGU>
    save(fullfile(od, sprintf('run_log_seed%d.mat', seed)), 'run_log', '-v7.3');
    fprintf('CAPTURE DONE\n');
end

function y = ternary(c, a, b); if c; y = a; else; y = b; end; end
