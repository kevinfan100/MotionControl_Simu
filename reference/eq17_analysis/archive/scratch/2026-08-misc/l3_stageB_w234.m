% L3 Stage B -- W2 liveness + W3 replay closure + W4 replay D on the 'mid' arm.
% Registration: test_script/scratch/l3_stageB_prereg.txt (read first).
% STATUS: ACTIVE (scratch, L3 instrument) | PURPOSE: wiring checks of the
%   ctrl_const.predict_quad flag before any bias is read | EXPIRES: Stage B closes.
% Seed 7, canonical deep, arm 'best'. Never reads test_results/run_formC_b_*.mat.
cd('/Users/kevin/code/MotionControl_Simu-motion-test');
root = pwd;
p = strsplit(genpath(fullfile(root, 'model')), pathsep);
p = p(~cellfun('isempty', p)); p = p(~contains(p, [filesep 'archive']));
addpath(p{:});
addpath(fullfile(root, 'test_script', 'integration'));
addpath(fullfile(root, 'test_script', 'build_helpers'));
addpath(fullfile(root, 'test_script', 'scratch'));
OUT_DIR = fullfile(root, 'test_results', 'l3_stageB');
if ~exist(OUT_DIR, 'dir'); mkdir(OUT_DIR); end
SEED = 7;  AX = 3;

% ---------------------------------------------------------------- W2 liveness
runs = struct();
for q = {'left', 'mid', 'trap'}
    clear run_formC_b motion_control_law_formC_b;
    ov = struct('predict_quad', q{1}, 'obs_dump', true);
    D = run_formC_b(struct('seeds', SEED, 'arm', 'best', 'ctrl_const_override', ov));
    r = D.runs{1};
    L = obs_dump('get');
    sel = [L.ax] == AX;  Lz = L(sel);
    kk = [Lz.k];  nst = numel(Lz(1).x_pred);
    Xp = nan(numel(kk), nst);  Xu = nan(numel(kk), nst);
    for i = 1:numel(Lz); Xp(i, :) = Lz(i).x_pred(:).'; Xu(i, :) = Lz(i).x_upd(:).'; end
    r.dump = struct('k', kk(:), 'x_pred', Xp, 'x_upd', Xu, 'n_state', nst, 'resets', obs_dump('resets'));
    runs.(q{1}) = r;
    fprintf('[run] predict_quad=%s : N=%d, dump records(z)=%d, n_state=%d\n', q{1}, numel(r.tout), numel(kk), nst);
end
ah_L = runs.left.a_bar_hat_out;  ah_M = runs.mid.a_bar_hat_out;  ah_T = runs.trap.a_bar_hat_out;
fprintf('\n=== W2 LIVENESS (seed %d, arm best, deep) ===\n', SEED);
for ax = 1:3
    fprintf('  axis %s: max|a_bar_hat mid - left| = %.6e   max|trap - left| = %.6e   max|mid - trap| = %.6e\n', ...
            char('w' + ax), max(abs(ah_M(:, ax) - ah_L(:, ax))), max(abs(ah_T(:, ax) - ah_L(:, ax))), ...
            max(abs(ah_M(:, ax) - ah_T(:, ax))));
end
fprintf('  first differing step (z): mid %d, trap %d\n', find(ah_M(:, AX) ~= ah_L(:, AX), 1), find(ah_T(:, AX) ~= ah_L(:, AX), 1));
% left arm with obs_dump on must equal the fixture (obs_dump bit-identity, proven in L2; re-checked here)
FIX = load(fullfile(root, 'test_results', 'l0_fixture', 'fixture_formC_b.mat'));
fprintf('  left(obs_dump on) vs fixture seed 7 a_bar_hat_out: max|diff| = %.3e [must be 0]\n', ...
        max(abs(ah_L(:) - FIX.fixture.data.seed7.a_bar_hat_out(:))));

% ---------------------------------------------------------------- W3 closure / W4 replay D
cc = runs.mid.ctrl_const;
a_floor = local_gfd(cc, 'a_bar_floor', 0.05);  a_ceil = local_gfd(cc, 'a_bar_ceil', 1 - 1e-4);
b_floor = local_gfd(cc, 'b_floor', 0.60);      b_ceil = local_gfd(cc, 'b_ceil', 1.05);
lc = cc.lambda_c;  al = 1 - lc;
fprintf('\n=== W3 CLOSURE + W4 REPLAY D (seed %d, z) ===\n', SEED);
for q = {'mid', 'trap', 'left'}
    r = runs.(q{1});  E = r.dump;  t = r.tout(:);  N = numel(t);  a_nom = r.a_nom;
    assert(isequal(E.k, (1:N-1).'), 'k_step not 1..N-1');
    Xp = E.x_pred;  Xu = E.x_upd;  n = E.n_state;
    ah = r.a_bar_hat_out(:, AX);  bh = r.b_hat_out(:, AX);
    K1 = r.K_a_y1_out(:, AX);  I1 = r.innov_y1_out(:, AX);  K2 = r.K_a_y2_out(:, AX);  I2 = r.innov_y2_out(:, AX);
    ap_log = r.a_prime_out(:, AX) / a_nom;                  % a'(a) the controller used, normalised
    hd = r.h_bar_d_out(:);
    xu_prev4 = [ah(1); Xu(1:end-1, 4)];                     % x_curr(4) at record i
    xu_prev5 = [bh(1); Xu(1:end-1, 5)];                     % x_curr(5)
    xu_prev3 = [0; Xu(1:end-1, 3)];
    if n >= 9; m_prev = [0; Xu(1:end-1, 8) + Xu(1:end-1, 9)]; else; m_prev = zeros(N-1, 1); end
    dwd_prev = hd(2:end) - hd(1:end-1);
    M_tot = dwd_prev + al * xu_prev3 + al * m_prev;         % the bracket the predict multiplies
    a_prev = min(max(xu_prev4, a_floor), a_ceil);           % a_bar_sl (ap_src = post)
    b_prev = min(max(xu_prev5, b_floor), b_ceil);           % b_hat_i
    ap_a   = b_prev .* (1 - a_prev).^2;                     % a'(a) rebuilt
    apk    = ap_log(2:end);                                 % logged a'(a) for record i (row k = i+1)
    a_half = min(max(a_prev + 0.5 * ap_a .* M_tot, a_floor), a_ceil);
    ap_half = b_prev .* (1 - a_half).^2;
    a_end  = min(max(a_prev + ap_a .* M_tot, a_floor), a_ceil);
    ap_end = b_prev .* (1 - a_end).^2;
    ap_trap = 0.5 * (ap_a + ap_end);
    pred_exact = Xp(:, 4) - xu_prev4;
    pred_ident = ah(2:end) - ah(1:end-1) - K1(2:end) .* I1(2:end) - K2(2:end) .* I2(2:end);
    switch q{1}
        case 'mid';  ap_rule = ap_half;
        case 'trap'; ap_rule = ap_trap;
        case 'left'; ap_rule = ap_a;
    end
    fprintf(' arm %-4s: a''(a) rebuilt vs logged a_prime_out/a_nom  max|diff| %.3e\n', q{1}, max(abs(ap_a - apk)));
    fprintf('           Kalman identity  max|ident - exact| %.3e (n>1e-15: %d)\n', ...
            max(abs(pred_ident - pred_exact)), nnz(abs(pred_ident - pred_exact) > 1e-15));
    dcl = abs(pred_exact - ap_rule .* M_tot);
    fprintf('           predict_exact vs %s-rule a''*M_tot   max|diff| %.3e (i=1: %.3e ; i>=2: %.3e)  [W3 target 1e-15]\n', ...
            q{1}, max(dcl), dcl(1), max(dcl(2:end)));
    for alt = {'left', 'mid', 'trap'}
        switch alt{1}; case 'left'; ap_alt = ap_a; case 'mid'; ap_alt = ap_half; case 'trap'; ap_alt = ap_trap; end
        fprintf('           (vs %-4s rule: max|diff| %.3e)\n', alt{1}, max(abs(pred_exact - ap_alt .* M_tot)));
    end
    fprintf('           clamp events on the half/end point: a_half clamped %d, a_end clamped %d (of %d)\n', ...
            nnz(a_half ~= a_prev + 0.5 * ap_a .* M_tot), nnz(a_end ~= a_prev + ap_a .* M_tot), N - 1);
    tk = t(2:end);
    aT = mean(r.a_true_out(t >= 3.8 & t <= 4.8, AX)) / a_nom;
    W = {'desc', [0.5 1.5]; 'osc', [1.5 3.5]; 'desc+osc', [0.5 3.5]; 'hold', [3.8 4.8]; 'all', [0 4.8]};
    fprintf('           W4 on this arm''s own path [%% of a_T = %.5f]  window : sum(left) sum(mid) sum(trap) | D_left-mid  D_mid-trap\n', aT);
    for w = 1:size(W, 1)
        in = tk >= W{w, 2}(1) & tk <= W{w, 2}(2);
        sL = 100 * sum(ap_a(in) .* M_tot(in)) / aT;
        sM = 100 * sum(ap_half(in) .* M_tot(in)) / aT;
        sT = 100 * sum(ap_trap(in) .* M_tot(in)) / aT;
        fprintf('             %-9s %+9.3f %+9.3f %+9.3f | %+8.3f  %+8.3f\n', W{w, 1}, sL, sM, sT, sL - sM, sM - sT);
    end
    runs.(q{1}).w4 = struct('aT', aT, 'M_tot', M_tot, 'ap_a', ap_a, 'ap_half', ap_half, 'ap_trap', ap_trap, 'tk', tk);
end

% keep the three single-seed runs (small) for the report
S = struct();
for q = {'left', 'mid', 'trap'}
    r = runs.(q{1});
    S.(q{1}) = rmfield(r, 'dump');
    S.(q{1}).dump_x_pred4 = r.dump.x_pred(:, 4);  S.(q{1}).dump_x_upd4 = r.dump.x_upd(:, 4);
end
save(fullfile(OUT_DIR, 'l3_w234_seed7.mat'), '-struct', 'S', '-v7.3');
fprintf('\nsaved %s\nL3 W2/W3/W4 DONE\n', fullfile(OUT_DIR, 'l3_w234_seed7.mat'));

function v = local_gfd(s, f, d)
    if isfield(s, f); v = s.(f); else; v = d; end
end
