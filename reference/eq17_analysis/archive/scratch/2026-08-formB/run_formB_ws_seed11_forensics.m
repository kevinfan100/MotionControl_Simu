% run_formB_ws_seed11_forensics.m -- PURPOSE: E2+E3 of the seed-11 self-lock
%   investigation (2026-08-03). On the LONG identification run (n_cycles 32,
%   T 34.8 s) under c4 conditions (lock_p, TRUE wall injected +5 %, honest
%   prior 0.111), seed 11 ends stuck at dev ~ -12 % (z = 3.36) while seed 7
%   converges (z = 0.96). Two pre-registered hypotheses:
%     H1  EKF linearization self-lock -- ws_hat low => believed sensitivity
%         a_bar' low => weak updates. Fingerprint: base11 believed/true
%         sensitivity ratio well below 1 AND oracle_bp still trapped.
%     H2  (b, ws) ridge -- free b absorbs the gain error at the trough so the
%         innovations stay small while ws is wrong. Fingerprint: b_end far
%         from its anchor 1.125 in base11 AND oracle_bp NOT trapped (z < 2).
%   Third path: if oracle_ctrl (exact gain to the CONTROL LAW only) un-traps
%   seed 11, the echo / control path is implicated instead.
%   Arms: base11, base7 (healthy contrast), oracle_ctrl, oracle_bp.
%   EXPIRES: absorbed into the seed-11 self-lock case file.
%   NOTE: no production file is modified; the driver is called as-is.

here = fileparts(mfilename('fullpath'));
proj = fullfile(here, '..', '..');
pdirs = {'model','model/dual_track','model/diag','model/thermal_force', ...
         'model/config','model/wall_effect','model/controller', ...
         'model/trajectory','test_script/integration'};
for i = 1:numel(pdirs); addpath(fullfile(proj, pdirs{i})); end

res_dir  = fullfile(proj, 'test_results');
if ~exist(res_dir, 'dir'); mkdir(res_dir); end
res_file = fullfile(res_dir, 'formB_seed11_forensics.mat');

AX      = 3;        % wall-normal (perpendicular) axis
INJ     = 0.05;     % [-] TRUE wall offset injected on the plant side
WS_TRUE = 1 + INJ;  % [-] true contact in estimator coordinates
N_CYC   = 32;       % identification-length oscillation
T_SIM   = 34.8;     % [s] 0.5 hold + 1.0 descend + 32 s osc + 1.3 s hold
B_TRUE  = 1.0354;   % [-] minimax (b, p) pair for c_perp on this envelope
P_TRUE  = 0.9500;   % [-]
B_ANCHOR = 9/8;     % [-] derived far-field reflection anchor (H2 reference)
GAP_FLOOR = 0.01;   % [-] law-argument floor, matches the controller guard
DS      = 20;       % storage downsample factor (1600 Hz -> 80 Hz)
HOLD_T0 = 33.5;     % [s] final-hold window start
OSC_T0  = 23.5;     % [s] last-10-s-of-oscillation window
OSC_T1  = 33.5;     % [s]

ov_base = struct('lock_b', false, 'lock_p', true, 'lock_ws', false, ...
                 'Pf_ws_std', 0.111);

arm_name = {'base11', 'base7', 'oracle_ctrl', 'oracle_bp'};
arm_seed = [11, 7, 11, 11];

if exist(res_file, 'file')
    load(res_file, 'S');
else
    S = struct('done', zeros(1, numel(arm_name)), 'arm', {cell(1, numel(arm_name))});
end

for i = 1:numel(arm_name)
    if S.done(i); continue; end

    o = struct('seeds', arm_seed(i), 'ws_inject', INJ, 'verbose', false, ...
               'config_override', struct('n_cycles', N_CYC, 'T_sim', T_SIM), ...
               'ctrl_const_override', ov_base);
    switch arm_name{i}
        case 'oracle_ctrl'
            o.a_ctrl_override = 'true';   % exact gain to the control law only
        case 'oracle_bp'
            o.oracle_bp = [B_TRUE P_TRUE];
            % The driver applies ctrl_const_override LAST, i.e. after the
            % oracle_bp arm flags, so ov_base's lock_b = false would release b
            % again. Re-assert the lock here: severing the b ridge IS the arm.
            o.ctrl_const_override.lock_b = true;
    end

    out = run_formB_ws(o);
    r = out.runs{1};

    t    = r.tout(:);
    wsh  = r.ws_hat_out(:, AX);
    dev  = 100 * (wsh - 1);            % [%] deviation from the nominal wall
    sqP  = 100 * r.P_ws_out(:, AX);    % [%] already a STD, do not sqrt
    bh   = r.b_hat_out(:, AX);
    ph   = r.p_hat_out(:, AX);
    wd   = r.h_bar_d_out(:);           % [-] w_bar_d the gain law is evaluated at
    d1   = r.dws_y1_out(:, AX);        % [-] per-step ws update via y1
    d2   = r.dws_y2_out(:, AX);        % [-] per-step ws update via y2

    % --- believed vs true sensitivity a_bar' = (p/b)(1 + g/b)^(-p-1) -------
    gap_hat   = max(wd - wsh, GAP_FLOOR);
    sens_hat  = (ph ./ bh) .* (1 + gap_hat ./ bh).^(-ph - 1);
    gap_true  = max(wd - WS_TRUE, GAP_FLOOR);
    sens_true = (P_TRUE / B_TRUE) * (1 + gap_true / B_TRUE).^(-P_TRUE - 1);
    ratio     = sens_hat ./ sens_true;

    W = struct();
    win_mask = {t >= HOLD_T0, t >= OSC_T0 & t < OSC_T1};
    win_name = {'hold', 'osc10'};
    for j = 1:2
        m = win_mask{j};
        w = struct();
        w.n          = nnz(m);
        w.wd_mean    = mean(wd(m));
        w.dev_mean   = mean(dev(m));
        w.dev_net    = dev(find(m, 1, 'last')) - dev(find(m, 1, 'first'));
        w.sqP_mean   = mean(sqP(m));
        w.b_mean     = mean(bh(m));
        w.p_mean     = mean(ph(m));
        w.sens_hat   = mean(sens_hat(m));
        w.sens_true  = mean(sens_true(m));
        w.ratio_mean = mean(ratio(m));
        w.sum_dws_y1 = sum(d1(m));     % full-rate ledger, NOT downsampled
        w.sum_dws_y2 = sum(d2(m));
        w.abs_dws_y1 = sum(abs(d1(m)));
        w.abs_dws_y2 = sum(abs(d2(m)));
        W.(win_name{j}) = w;
    end

    a = struct();
    a.name    = arm_name{i};
    a.seed    = arm_seed(i);
    a.dev_end = dev(end);
    a.sqP_end = sqP(end);
    a.z_end   = abs(dev(end) - 100 * INJ) / sqP(end);
    a.b_end   = bh(end);
    a.p_end   = ph(end);
    a.dev_min = min(dev);
    a.dev_max = max(dev);
    a.win     = W;
    a.tds     = t(1:DS:end);
    a.dev_ds  = dev(1:DS:end);
    a.sqP_ds  = sqP(1:DS:end);
    a.b_ds    = bh(1:DS:end);
    a.p_ds    = ph(1:DS:end);
    a.wd_ds   = wd(1:DS:end);
    a.d1_ds   = d1(1:DS:end);
    a.d2_ds   = d2(1:DS:end);
    a.sens_ratio_ds = ratio(1:DS:end);

    S.arm{i}  = a;
    S.done(i) = 1;
    save(res_file, 'S');

    fprintf(['FORENSICS %s: dev_end %+.2f%%  sqP_end %.2f%%  z_end %.2f  ' ...
             'b_end %.4f  p_end %.4f\n'], ...
            a.name, a.dev_end, a.sqP_end, a.z_end, a.b_end, a.p_end);
end

% ---------------------------------------------------------------------------
% Report
% ---------------------------------------------------------------------------
fprintf('\n===== seed-11 forensics (long run, n_cycles %d, T %.1f s, c4) =====\n', ...
        N_CYC, T_SIM);
fprintf('true wall dev = %+.2f%%, trap criterion z_end > 2, b anchor %.4f\n', ...
        100 * INJ, B_ANCHOR);
for i = 1:numel(arm_name)
    a = S.arm{i};
    fprintf(['FORENSICS %s: dev_end %+.2f%%  sqP_end %.2f%%  z_end %.2f  ' ...
             'b_end %.4f  p_end %.4f\n'], ...
            a.name, a.dev_end, a.sqP_end, a.z_end, a.b_end, a.p_end);
end

for j = 1:2
    wn = {'hold', 'osc10'};
    lab = {sprintf('FINAL HOLD  (t >= %.1f s)', HOLD_T0), ...
           sprintf('OSC last 10 s (%.1f <= t < %.1f s)', OSC_T0, OSC_T1)};
    fprintf('\n-- %s --\n', lab{j});
    fprintf('%12s | %7s %8s %8s | %9s %9s %7s | %10s %10s\n', ...
            'arm', 'wd_mean', 'dev_mean', 'b_mean', 'sens_hat', 'sens_true', ...
            'ratio', 'sum dws_y1', 'sum dws_y2');
    for i = 1:numel(arm_name)
        a = S.arm{i}; w = a.win.(wn{j});
        fprintf('%12s | %7.3f %+8.2f %8.4f | %9.5f %9.5f %7.3f | %+10.5f %+10.5f\n', ...
                a.name, w.wd_mean, w.dev_mean, w.b_mean, w.sens_hat, ...
                w.sens_true, w.ratio_mean, w.sum_dws_y1, w.sum_dws_y2);
    end
    fprintf('%12s | %s\n', '', ...
            'net dev change over window [%] and |dws| totals:');
    for i = 1:numel(arm_name)
        a = S.arm{i}; w = a.win.(wn{j});
        fprintf('%12s | net dev %+7.3f%%  sqP_mean %5.2f%%  sum|dws_y1| %8.5f  sum|dws_y2| %8.5f\n', ...
                a.name, w.dev_net, w.sqP_mean, w.abs_dws_y1, w.abs_dws_y2);
    end
end

fprintf('\nsaved: %s\n', res_file);
