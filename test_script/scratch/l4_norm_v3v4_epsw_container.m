% l4_norm_v3v4_epsw_container.m -- normalization-boundary audit, arms V3 + V4 (2026-08-30)
% V3 PURPOSE: rebuild the realised process noise eps_w from the TRUTH chain and compare its
%   variance with the Q33 container the tex S3 definition implies:
%       eps_w[k] = -(e[k+1] - lc e[k]) - [ f_d e_a ]_k - al*([f_d e_a]_{k-1} + [f_d e_a]_{k-2})
%       e[k]     = w_bar_d[k] - w_bar_true[k]          (tex S3: w = w_d - dw_3)
%       e_a[k]   = a_bar_true[k] - a_bar_ctrl[k]        (applied gain = posterior of k-1)
%       Q_full[k]  = kappa_T*(a_bar[k] + al^2*(a_bar[k-1] + a_bar[k-2])) + al^2*sigma2_nw   (S3 full)
%       Q_trunc[k] = kappa_T*a_bar[k]                                                       (tex S8 line)
%   PRE-REGISTERED: per window Var(eps_w)/mean(Q_full) = 1 +- 10%; Var/mean(Q_trunc) ~ 1.18
%   (=1+2 al^2 at lc 0.7 plus the n_w share). mean(eps_w) ~ 0 (a bias = misalignment or a
%   deterministic term the reconstruction missed). The identity is exact up to the plant's
%   ode4 sub-stepping and the evaluation point of a_bar_true, both second order.
% V4 PURPOSE: the readout-chain variance identity behind xi:
%       Var(dw_bar_r) = C_dpmr*kappa_T*a_bar + C_n*sigma2_nw = C_dpmr*kappa_T*(a_bar + xi)
%   on the two holds (stationary). PRE-REGISTERED: ratio 1 +- 10%; xi(z) ~ 1.53e-2 (house).
% Sample alignment of h_bar_true_out vs f_bar_out is not documented; both shifts are tried and
% the one with no descent bias is the physical one (the wrong shift leaks one step of command
% displacement into eps as a bias; the far-field variance does NOT separate them -- measured).
function l4_norm_v3v4_epsw_container()
    here = fileparts(mfilename('fullpath'));
    root = fullfile(here, '..', '..');
    addpath(genpath(fullfile(root, 'model')));
    addpath(fullfile(root, 'test_script', 'integration'));
    clear physical_constants motion_control_law_formC_b calc_thermal_force; rehash;

    o   = run_formC_b(struct('seeds', 7, 'scenario', 'deep'));
    s   = o.runs{1};  cfg = o.cfg;  cc = s.ctrl_const;
    pc  = physical_constants();
    R = pc.R; Ts = pc.Ts; gN = pc.gamma_N; kBT = pc.k_B * pc.T;
    a_o = Ts / (gN * R);  kappa_T = 4 * kBT * a_o / R;
    lc = cc.lambda_c;  al = 1 - lc;
    AX = 3;
    sig2n = (cfg.meas_noise_std(AX) / R)^2;

    t      = s.tout(:);
    w_d    = s.h_bar_d_out(:);
    fbar   = s.f_bar_out(:, AX);
    a_true = s.a_true_out(:, AX) / s.a_nom;                      % 1/c_perp at the true height
    a_ctrl = [a_true(1); s.a_bar_hat_out(1:end-1, AX)];          % applied gain at k = posterior of k-1
    N = numel(t);

    win = struct('name', {'hold1 (far)', 'descent', 'osc', 'hold2 (band top)'}, ...
                 'lo', {0.15, 0.5, 1.5, 3.9}, 'hi', {0.5, 1.5, 3.5, 4.8});

    fprintf('\n=== V3: realised eps_w vs Q33 container (z, seed 7, lc %.2f, kappa_T %.4e, sigma2_nw %.3e) ===\n', ...
            lc, kappa_T, sig2n);
    best_shift = NaN; best_v = Inf;
    for shift = [0 1]
        w_true = s.h_bar_true_out(:);
        if shift == 1; w_true = [w_true(2:end); w_true(end)]; end
        e   = w_d - w_true;
        e_a = a_true - a_ctrl;
        fe  = fbar .* e_a;
        k   = (3:N-1)';
        eps_hat = -(e(k+1) - lc * e(k)) - (fe(k) + al * (fe(k-1) + fe(k-2)));
        Q_full  = kappa_T * (a_true(k) + al^2 * (a_true(k-1) + a_true(k-2))) + al^2 * sig2n;
        Q_trunc = kappa_T * a_true(k);
        tk = t(k);
        fprintf('-- alignment shift %d --\n', shift);
        fprintf('%-18s %8s %11s %11s %9s %9s %10s\n', 'window', 'n', 'Var(eps)', 'mean(Q_full)', 'r_full', 'r_trunc', 'mean(eps)/sd');
        for i = 1:numel(win)
            m = tk >= win(i).lo & tk < win(i).hi;
            v = var(eps_hat(m));
            fprintf('%-18s %8d %11.3e %11.3e %9.3f %9.3f %10.3f\n', win(i).name, nnz(m), v, mean(Q_full(m)), ...
                    v / mean(Q_full(m)), v / mean(Q_trunc(m)), mean(eps_hat(m)) / sqrt(v));
            % alignment tie-break: the far-field variance cannot separate the shifts (both
            % ~0.99, measured 2026-08-30); the DESCENT bias can (|mean/sd| 0.02 vs 0.56),
            % because the wrong shift leaks one step of command displacement into eps.
            if i == 2 && abs(mean(eps_hat(m))) / sqrt(v) < best_v
                best_v = abs(mean(eps_hat(m))) / sqrt(v); best_shift = shift;
            end
        end
        % what the deterministic term is worth (how much the identity relies on it)
        m = tk >= 1.5 & tk < 3.5;
        raw = -(e(k+1) - lc * e(k));
        fprintf('   osc: Var(raw -(e+ - lc e)) / Var(eps_hat) = %.3f  (deterministic gain-error term removed)\n', ...
                var(raw(m)) / var(eps_hat(m)));
    end
    fprintf('physical alignment by descent bias |mean/sd|: shift %d\n', best_shift);
    fprintf('expected r_trunc from S3: 1 + 2*al^2 + al^2*sigma2_nw/(kappa_T*a_bar) = %.3f (a_bar 0.98) / %.3f (a_bar 0.087)\n', ...
            1 + 2*al^2 + al^2*sig2n/(kappa_T*0.98), 1 + 2*al^2 + al^2*sig2n/(kappa_T*0.087));

    % production container for reference (built at a_bar_hat; ma2 arm carries only the current step)
    q33 = s.Q33_out(:, AX);
    fprintf('production Q33_out (ma2 arm, current step only): osc mean %.3e vs kappa_T*a_true mean %.3e\n', ...
            mean(q33(t >= 1.5 & t < 3.5)), mean(kappa_T * a_true(t >= 1.5 & t < 3.5)));

    fprintf('\n=== V4: readout residual variance vs C_dpmr*kappa_T*(a_bar + xi) ===\n');
    xi = cc.C_n * sig2n / (cc.C_dpmr * kappa_T);
    fprintf('C_dpmr %.4f  C_n %.4f  xi(z) = %.4e\n', cc.C_dpmr, cc.C_n, xi);
    dwr = s.dx_r_out(:, AX) / R;
    for i = [1 4]
        m = t >= win(i).lo & t < win(i).hi;
        v = var(dwr(m));
        ab = mean(a_true(m));
        pred = cc.C_dpmr * kappa_T * (ab + xi);
        fprintf('%-18s n %5d  Var(dw_r) %.3e  pred %.3e  ratio %.3f  (a_bar %.3f, xi/a_bar %.3f)\n', ...
                win(i).name, nnz(m), v, pred, v / pred, ab, xi / ab);
    end
end
