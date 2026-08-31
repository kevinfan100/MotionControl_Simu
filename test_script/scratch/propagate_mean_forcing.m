function out = propagate_mean_forcing(seed)
%PROPAGATE_MEAN_FORCING  Step 3 of the closed-loop E[nu] derivation: push each
%   candidate second-order forcing through the RECORDED loop and compare with
%   the measured signatures.
%
%   Mean-error recursion (e = truth - estimate, per z axis, forcing g on the
%   position row 3 = the plant's mean displacement the model does not have):
%       e_pred[i] = F_i e_upd[i-1] + g_i
%       nubar[i]  = H1 e_pred[i]                       (H1 = e_1')
%       e_upd[i]  = (I - K2 H2)(I - K1 H1) e_pred[i]
%   All matrices are the PASS-stamped capture (machine-exact identities).
%
%   Candidates (per step, physics closed forms / measured moments, no filter P):
%     unit    1e-6 during motion                (loop response calibration)
%     F1      1/2 (a''/a) Var_w * dwd           mobility Jensen (concave a(w))
%     F2      -(1-lc) a' Var_w / a              closed-loop Cov(a(w), force)
%     F3      dwd * Var(a_hat)/a_hat^2          controller 1/a_hat convexity
%   Var_w = stationary thermal position variance from the run's own thermal
%   kicks (w_T = a_true*F_th/R, smoothed, /(1-lc^2)); Var(a_hat) = cross-seed
%   variance from the 8-seed b_true+exact stack.
%
%   Signatures to match (measured, b_true+exact): trough bias +0.013 a_o,
%   rms nubar 2.3e-4 in the oscillation, velocity-locked sign pattern,
%   zero drift in the holds.  Both signs of every candidate are propagated;
%   TEMPORARY numbers, no verdict implied by a single match.
% STATUS: ACTIVE | closed-loop mean-bias derivation line (2026-08-31)

    if nargin < 1 || isempty(seed); seed = 7; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'loop_mean_bias');
    C = load(fullfile(od, sprintf('loop_capture_seed%d.mat', seed)));
    assert(C.ok, 'capture is not PASS-stamped');
    R = load(fullfile(od, sprintf('run_log_seed%d.mat', seed)));  r = R.run_log;
    ax = 3;  lam = 0.7;  ns = size(C.F, 1);  n = size(C.F, 3);
    kk = C.KK;  t = C.t(:);
    pcR = physical_constants().R;

    % ---- per-step ingredients (driver log rows kk) -------------------------
    a_nom = r.a_nom;
    ab  = r.a_true_out(kk, ax) / a_nom;                 % true a_bar (proxy for the operating point)
    ah  = r.a_bar_hat_out(kk, ax);
    bh  = r.b_hat_out(kk, ax);
    dwd = [0; diff(r.h_bar_d_out(:))];  dwd = dwd(kk);
    ap  = bh .* (1 - ab).^2;  app = 2 * bh.^2 .* (1 - ab).^3;      % a', |a''|
    wT  = r.a_true_out(kk, ax) .* r.F_th_out(kk, ax) / pcR;        % thermal step [R]
    Varw = movmean(wT.^2, 801) / (1 - lam^2);                      % stationary Var(dw) [R^2]
    S8 = load(fullfile(od, '..', 'btrue_log_ledger', 'check_btrue_log_ledger.mat'));
    Fa = S8.out.noisyExact;
    VarA = interp1(Fa.tt(:), movmean(var(Fa.ah(2:end,:), 0, 2), 801), t, 'nearest', 'extrap');

    CAND = {'unit 1e-6 (motion)', 1e-6 * double(abs(dwd) > 0); ...
            'F1 Jensen mobility', 0.5 * (app ./ ab) .* Varw .* dwd; ...
            'F2 closed-loop cov', -(1 - lam) * ap .* Varw ./ max(ab, 0.02); ...
            'F3 1/a_hat convexity', dwd .* VarA ./ max(ah, 0.02).^2};

    % ---- measured targets --------------------------------------------------
    nu_meas = mean(Fa.n1(2:end,:), 2) - S8.out.detExact.n1(2:end);
    nu_meas = interp1(Fa.tt(:), movmean(nu_meas, 81), t, 'nearest', 'extrap');
    m_osc = t >= 1.5 & t < 3.5;  m_hold = t > 3.7;
    fprintf('\nMEASURED: trough bias +0.013 a_o | rms nubar(osc) %.2e | hold drift ~0\n', rms(nu_meas(m_osc)));

    H1 = zeros(1, ns);  H1(1) = 1;
    fprintf('%-22s %5s | %11s %11s %8s | %11s\n', 'candidate', 'sign', 'e4(end) a_o', 'rms nubar', 'corr', 'x to 0.013');
    out = struct();
    for c = 1:size(CAND, 1)
        for sgn = [1 -1]
            g = sgn * CAND{c, 2};
            e = zeros(ns, 1);  E4 = zeros(n, 1);  NB = zeros(n, 1);
            for i = 1:n
                ep = C.F(:,:,i) * e + [0; 0; g(i); zeros(ns - 3, 1)];
                NB(i) = H1 * ep;
                e = ep - C.K1(:,i) * (H1 * ep);
                e = e - C.K2(:,i) * (C.H2S(:,i).' * e);
                E4(i) = e(4);
            end
            % e4 is truth-minus-estimate: BIAS of the estimate = -e4
            bias_end = -mean(E4(m_hold));
            cc = corr(movmean(NB(m_osc), 81), nu_meas(m_osc));
            fprintf('%-22s %+5d | %+11.5f %11.2e %+8.2f | %+9.1f\n', CAND{c,1}, sgn, ...
                    bias_end, rms(NB(m_osc)), cc, 0.013 / max(abs(bias_end), 1e-12) * sign(bias_end));
            if sgn == 1
                out.(matlab.lang.makeValidName(CAND{c,1})) = struct('E4', E4, 'NB', NB, 'bias', bias_end, 'cc', cc);
            end
        end
    end
    out.t = t;  out.nu_meas = nu_meas;
    save(fullfile(od, 'propagate_mean_forcing.mat'), 'out');
    fprintf('PROPAGATE DONE\n');
end
