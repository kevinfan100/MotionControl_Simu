% L2 predict replay -- bit-identical check of the driver log line, plus the
% obs_dump-on bit-identity check and the Route B-exact capture (seed 7, z).
% Registration: test_script/scratch/l2_replay_prereg.txt
cd('/Users/kevin/code/MotionControl_Simu-motion-test');
addpath(genpath('test_script')); addpath(genpath('model'));

SEED = 7;  AX = 3;
ref = struct();
for arm = {'base', 'apknown'}
    m = matfile(sprintf('test_results/am_r22_deep/stageA_bmid_%s_100.mat', arm{1}));
    ou = m.opts_used;  assert(ou.seeds(SEED) == SEED, 'seed list is not 1:100');
    ref.(arm{1}) = m.a_bar_hat_out(:, :, SEED);
end

% ---- (b) log line only, base arm, seed 7 ----
clear run_formC_b motion_control_law_formC_b;
D = run_formC_b(struct('seeds', SEED, 'arm', 'bmid'));
r = D.runs{1};
d = max(abs(r.a_bar_hat_out(:) - ref.base(:)));
fprintf('\nBITCHECK (log line only, base, seed %d): max|a_bar_hat_out - stageA| = %.3e  [must be exactly 0]\n', SEED, d);
assert(isfield(r, 'delta_x_hat_3_out'), 'delta_x_hat_3_out not logged');
fprintf('  delta_x_hat_3_out present: size %s, |z| max %.4g um, row1 %s\n', mat2str(size(r.delta_x_hat_3_out)), ...
        max(abs(r.delta_x_hat_3_out(:, AX))), mat2str(r.delta_x_hat_3_out(1, :), 4));
if d ~= 0
    error('BITCHECK FAILED -- STOP');
end

% ---- obs_dump on: bit-identity + Route B-exact capture, both arms ----
EX = struct();
for arm = {'base', 'apknown'}
    clear run_formC_b motion_control_law_formC_b;
    o = struct('seeds', SEED, 'arm', 'bmid', 'ctrl_const_override', struct('obs_dump', true));
    if strcmp(arm{1}, 'apknown'); o.ap_known = true; end
    D = run_formC_b(o);
    r = D.runs{1};
    d2 = max(abs(r.a_bar_hat_out(:) - ref.(arm{1})(:)));
    L = obs_dump('get');
    fprintf('BITCHECK (obs_dump on, %s, seed %d): max|diff| = %.3e  [must be 0]; records %d, resets %d\n', ...
            arm{1}, SEED, d2, numel(L), obs_dump('resets'));
    sel = [L.ax] == AX;  Lz = L(sel);
    kk = [Lz.k];  nst = numel(Lz(1).x_pred);
    Xp = nan(numel(kk), nst);  Xu = nan(numel(kk), nst);  G = false(numel(kk), 1);
    for i = 1:numel(Lz); Xp(i, :) = Lz(i).x_pred(:).'; Xu(i, :) = Lz(i).x_upd(:).'; G(i) = Lz(i).gate; end
    E = struct('k', kk(:), 'x_pred', Xp, 'x_upd', Xu, 'gate', G, 'n_state', nst, 'bitdiff', d2);
    % carry the run's own logs alongside
    fl = {'a_bar_hat_out', 'K_a_y1_out', 'K_a_y2_out', 'innov_y1_out', 'innov_y2_out', 'a_prime_out', ...
          'a_prime_true_out', 'a_true_out', 'h_bar_d_out', 'h_bar_true_out', 'delta_x_hat_3_out', 'tout', 'a_nom'};
    for f = fl; E.(f{1}) = r.(f{1}); end
    E.K = r.ctrl_const;
    EX.(arm{1}) = E;
end
save('/Users/kevin/.claude/jobs/8581427c/tmp/l2r/l2_bexact_seed7.mat', '-struct', 'EX');
fprintf('saved B-exact capture (seed %d, z) to tmp/l2r/l2_bexact_seed7.mat\n', SEED);
