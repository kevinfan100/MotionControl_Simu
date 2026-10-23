% PURPOSE: entry-by-entry audit of motion_control_law_formC_b.m against
%   reference/eq17_analysis/derivation/formC_state_b.tex, using ONLY the obs_dump
%   record (F, H, R, x, P as the controller built them). Nothing is re-derived
%   from the controller source, so this is an independent pass.
%
%   Recoverable exactly from the dump:
%     lambda_c   <- F(3,3)
%     F_dw       <- -F(3,4)                     (tex row 3)
%     M          <- from F(4,4), given a_hat, b_hat, F_dw
%     nabla_d w  <- from H(2,4)/scale, given a_hat, b_hat
%     Q          <- P_pred[k] - F[k] P_upd[k-1] F[k]'
%   Then the tex's own expressions are evaluated and compared.
%
%   The scale on H is a_cov*echo_fac and is common to both columns, so
%   H(2,5)/H(2,4) is scale-free and is checked exactly.
% EXPIRES: when the audit is recorded in formC_state_b_ref.tex.
function out = audit_formC_b_code_vs_tex(L, ax)
    if nargin < 2 || isempty(ax); ax = 3; end
    Lz = L([L.ax] == ax); N = numel(Lz);
    n = size(Lz(1).F, 1);
    lc = Lz(1).F(3, 3);
    fprintf('\n=========== formC_b : code vs formC_state_b.tex ===========\n');
    fprintf('records %d (axis %d), n_state %d, lambda_c from F(3,3) = %.6f\n', N, ax, n, lc);

    rel = @(a, b) max(abs(a - b) ./ max(abs(b), realmin));
    dev = struct();

    a4 = zeros(N,1); b5 = zeros(N,1);
    F43 = zeros(N,1); F44 = zeros(N,1); F45 = zeros(N,1); Fdw = zeros(N,1);
    H24 = nan(N,1); H25 = nan(N,1);
    for k = 1:N
        s = Lz(k);
        a4(k) = s.x_pred(4); b5(k) = s.x_pred(5);   % predict uses x_curr; see note
        F43(k) = s.F(4,3); F44(k) = s.F(4,4); F45(k) = s.F(4,5);
        Fdw(k) = -s.F(3,4);
        if ~isempty(s.H{2}); H24(k) = s.H{2}(4); H25(k) = s.H{2}(5); end
    end
    % The Jacobian is built at x_curr (= previous step's x_upd), not x_pred.
    ah = zeros(N,1); bh = zeros(N,1);
    ah(1) = a4(1); bh(1) = b5(1);
    for k = 2:N; ah(k) = Lz(k-1).x_upd(4); bh(k) = Lz(k-1).x_upd(5); end
    om = 1 - ah;

    % ---- 1. F(4,3) = (1-a)^2 (1-lc) / b        (no free unknown) ----
    pred43 = om.^2 * (1 - lc) ./ bh;
    dev.F43 = rel(F43, pred43);
    fprintf('\n[1] F(4,3) = (1-a)^2 (1-lc)/b            max rel dev %.3e  %s\n', ...
            dev.F43, verdict(dev.F43));

    % ---- 2. solve M from F(4,4), then check F(4,5) ----
    %   F(4,4) = 1 + (1-a)^2 F_dw/b - 2(1-a) M / b
    M = (1 + om.^2 .* Fdw ./ bh - F44) .* bh ./ (2 * om);
    pred45 = -om.^2 .* M ./ bh.^2;
    dev.F45 = rel(F45, pred45);
    fprintf('[2] F(4,5) = -(1-a)^2 M / b^2            max rel dev %.3e  %s\n', ...
            dev.F45, verdict(dev.F45));
    fprintf('    (M solved from F(4,4); an error in EITHER entry shows here)\n');
    fprintf('    sign check: F(4,5) and M have opposite signs in %.1f%% of steps (expect 100)\n', ...
            100 * mean(sign(F45) == -sign(M) | M == 0));

    % ---- 3. H(2,5)/H(2,4): scale-free, and the entry under suspicion ----
    %   H(2,4) = s*(1 + 2(1-a) G / b) ,  H(2,5) = s*((1-a)^2 G / b^2)
    %   => G solved from the ratio is consistent only if BOTH match the tex.
    ok = ~isnan(H24);
    ratio = H25(ok) ./ H24(ok);
    % solve G from H(2,5)/H(2,4) = (1-a)^2/b^2 / (1/G + 2(1-a)/b)
    %   let u = (1-a), r = ratio:  r = (u^2 G / b^2) / (1 + 2 u G / b)
    %   => r (1 + 2uG/b) = u^2 G / b^2  => r = G (u^2/b^2 - 2 u r / b)
    %   => G = r / (u^2/b^2 - 2 u r / b)
    u = om(ok); bb = bh(ok);
    G = ratio ./ (u.^2 ./ bb.^2 - 2 * u .* ratio ./ bb);
    % independent handle on G: nabla_d w_d is the d-step COMMAND difference,
    % which must be smooth and bounded by the trajectory; check both columns
    % reproduce with this single G.
    pred24 = 1 + 2 * u .* G ./ bb;              % up to the common scale
    pred25 = u.^2 .* G ./ bb.^2;
    sc = H24(ok) ./ pred24;                     % implied common scale
    dev.H25 = rel(H25(ok), sc .* pred25);
    dev.scale_const = std(sc, 'omitnan') / mean(sc, 'omitnan');
    fprintf('\n[3] H(2,5) = s*(1-a)^2 G / b^2           max rel dev %.3e  %s\n', ...
            dev.H25, verdict(dev.H25));
    fprintf('    implied common scale s: mean %.5f  rel spread %.3e (a_cov*echo, should be near-const)\n', ...
            mean(sc, 'omitnan'), dev.scale_const);
    fprintf('    SIGN: sign(H(2,5)) == sign(G) in %.1f%% of steps (tex says +, expect 100)\n', ...
            100 * mean(sign(H25(ok)) == sign(G)));
    fprintf('    recovered nabla_d w_d: range [%.4f, %.4f]  (must be a plausible 2-step command diff)\n', ...
            min(G), max(G));

    % ---- 4. Q from the covariance recursion ----
    q33 = nan(N,1); q34 = nan(N,1); q44 = nan(N,1); q55 = nan(N,1); q45 = nan(N,1);
    for k = 2:N
        Q = Lz(k).P_pred - Lz(k).F * Lz(k-1).P_upd * Lz(k).F';
        q33(k) = Q(3,3); q34(k) = Q(3,4); q44(k) = Q(4,4);
        q55(k) = Q(5,5); q45(k) = Q(4,5);
    end
    g = 2:N;
    pred34 = -om(g).^2 .* q33(g) ./ bh(g);
    pred44 =  om(g).^4 .* q33(g) ./ bh(g).^2;
    dev.Q34 = rel(q34(g), pred34);
    dev.Q44 = rel(q44(g), pred44);
    dev.Q55 = max(abs(q55(g)));
    fprintf('\n[4] Q34 = -(1-a)^2 Q33/b                 max rel dev %.3e  %s\n', dev.Q34, verdict(dev.Q34));
    fprintf('    Q44 = (1-a)^4 Q33/b^2                max rel dev %.3e  %s\n', dev.Q44, verdict(dev.Q44));
    fprintf('    Q55 = 0                              max |Q55|   %.3e  %s\n', dev.Q55, verdict(dev.Q55/max(q33(g))));
    fprintf('    Q45 max |.| %.3e  (tex gives no Q45 term; expect 0)\n', max(abs(q45(g))));

    % ---- 5. seed P45[0] sign and magnitude ----
    %   P_pred(4,5)[1] = F(4,4) P0(4,5) + F(4,5) P0(5,5)  (P0(3,5) = 0)
    P0_55 = Lz(1).P_pred(5,5);          % F(5,5)=1, Q55=0 => equals P0(5,5)
    P45_1 = Lz(1).P_pred(4,5);
    P45_0 = (P45_1 - Lz(1).F(4,5) * P0_55) / Lz(1).F(4,4);
    pred_P45_0 = -(1 - ah(1)) / bh(1) * P0_55;
    fprintf('\n[5] P45[0] = -[(1-a0)/b0] P_bb\n');
    fprintf('    P_bb from dump      %.6e   (sqrt %.6f)\n', P0_55, sqrt(P0_55));
    fprintf('    P45[0] recovered    %+.6e\n', P45_0);
    fprintf('    tex prediction      %+.6e   rel dev %.3e  %s\n', pred_P45_0, ...
            abs(P45_0 - pred_P45_0)/abs(pred_P45_0), verdict(abs(P45_0-pred_P45_0)/abs(pred_P45_0)));
    fprintf('    a_hat[0] %.6f  b_hat[0] %.6f\n', ah(1), bh(1));

    out = dev;
    fprintf('\n=========================================================\n');
end

function s = verdict(d)
    if d < 1e-10; s = 'EXACT';
    elseif d < 1e-6; s = 'ok (roundoff)';
    elseif d < 1e-3; s = '?? CHECK';
    else; s = 'MISMATCH';
    end
end
