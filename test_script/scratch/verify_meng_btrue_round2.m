function out = verify_meng_btrue_round2()
%VERIFY_MENG_BTRUE_ROUND2  Two follow-ups on the Meng b_true cross-check (2026-08-31).
%
%   (A) row-3 mean forcing with a CONTROL VARIATE: G3's variance is dominated
%       by the zero-mean thermal step (rms ~3.7e-3 vs the ~1e-5 mean we are
%       after). Regress G3 per seed on the known step of the true position
%       (proxy for w_T) and subtract the fit; the residual mean is unbiased
%       and its SEM shrinks by the correlation factor. Then re-compare the
%       segment means with the closed form.
%   (B) the closure test on Meng: capture the loop (A matrices) for seed 7,
%       replay the 6-seed ensemble-mean forcing (rows 3+4) through it, and
%       compare the predicted mean e4 trajectory with the actual cross-seed
%       mean E4A -- the Meng analogue of canonical's -0.0115 vs -0.0121.
% STATUS: ACTIVE | closed-loop mean-bias line, cross-scenario leg

    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'loop_mean_bias');
    S = load(fullfile(od, 'meng_btrue_profile.mat'));
    t = S.t(:);  ns0 = numel(S.seeds);  N = numel(t);
    ax = 3;  lam = 0.7;  pcR = physical_constants().R;

    % ---- (A) control-variate cleaned row-3 profile -------------------------
    G3c = zeros(N, ns0);  G4c = zeros(N, ns0);
    for q = 1:ns0
        w = -S.DWJ(:, q);                       % dw_true level (w_d - w)... DWJ = w - w_d, so w level jitter = DWJ
        cv = [0; diff(S.DWJ(:, q))];            % step of the true position (thermal-dominated)
        cv2 = [0; cv(1:end-1)];                 % one-step lag (update-level mixing)
        X = [cv, cv2];
        b3 = X \ S.G3(:, q);   G3c(:, q) = S.G3(:, q) - X * b3;
        b4 = X \ S.G4(:, q);   G4c(:, q) = S.G4(:, q) - X * b4;
    end
    SEG = {'far half',  t > 0.5 & t < 5.5; 'near half', t > 5.5 & t < 10.5; ...
           'near-wall 8-10 s', t > 8 & t < 10; 'hold', t > 11.5};
    DWcv = S.DWJ - mean(S.DWJ, 2);  STc = S.STP - mean(S.STP, 2);
    comp = mean(S.APH, 2) .* mean(DWcv .* STc, 2);
    fprintf('\n(A) control-variate cleaned mean forcing (x1e-5), %d seeds:\n', ns0);
    fprintf('%-18s %16s %16s | %10s %7s\n', 'segment', 'G3 clean', 'G4 clean', 'closedform', 'ratio3');
    for g = 1:4
        m = SEG{g,2};
        m3 = mean(G3c(m,:), 1);  m4 = mean(G4c(m,:), 1);  cf = mean(comp(m));
        fprintf('%-18s %+7.2f +- %-6.2f %+7.2f +- %-6.2f | %+10.2f %7.2f\n', SEG{g,1}, ...
                1e5*mean(m3), 1e5*std(m3)/sqrt(ns0), 1e5*mean(m4), 1e5*std(m4)/sqrt(ns0), 1e5*cf, mean(m3)/max(abs(cf),1e-12)*sign(cf));
    end
    fprintf('  SEM shrink factor row3 (raw/clean): %.1f\n', mean(std(mean(S.G3(SEG{2,2},:),1))) / max(std(mean(G3c(SEG{2,2},:),1)), 1e-12));

    % ---- (B) closure: capture seed 7 Meng loop and replay the mean forcing --
    ov = struct('trajectory_type','osc','h_init',15.0,'h_bottom',2.5,'amplitude',0,'frequency',1,'n_cycles',1, ...
                't_hold',0.5,'t_descend_override',10.0,'T_sim',12.5,'h_min',1.1*pcR);
    O = run_formC_b(struct('arm', 'best', 'ap_src', 'post', 'seeds', 7, 'verbose', false, ...
                           'b_true', true, 'b_true_at', 'true', 'config_override', ov, ...
                           'ctrl_const_override', struct('law_exact_step', true, 'obs_dump', true)));
    r = O.runs{1};  L = obs_dump('get');  Lz = L([L.ax] == ax);
    n = numel(Lz);  nsl = numel(Lz(1).x_pred);
    gb3 = mean(S.G3, 2);  gb4 = mean(S.G4, 2);
    e = zeros(nsl, 1);  E4 = zeros(n, 1);  H1 = zeros(1, nsl);  H1(1) = 1;
    for i = 2:n
        s = Lz(i);  P = s.P_pred;
        k1 = (P * H1.') / (H1 * P * H1.' + s.R(1));
        P1 = (eye(nsl) - k1 * H1) * P;
        H2 = s.H{2};  I2 = eye(nsl);
        if ~isempty(H2) && ~s.gate
            k2 = (P1 * H2.') / (H2 * P1 * H2.' + s.R(2));
            I2 = eye(nsl) - k2 * H2;
        end
        A = I2 * (eye(nsl) - k1 * H1) * s.F;
        g = zeros(nsl, 1);  g(3) = gb3(min(i, N));  g(4) = gb4(min(i, N));
        e = A * e + g;
        E4(i) = e(4);
    end
    mh = t > 11.5;  mnw = t > 8 & t < 10;
    act = mean(S.E4A, 2);
    fprintf('\n(B) Meng closure: predicted vs actual mean e4 (a_o)\n');
    fprintf('  near-wall 8-10 s:  pred %+.4f   actual %+.4f +- %.4f\n', mean(E4(mnw)), mean(mean(S.E4A(mnw,:),1)), std(mean(S.E4A(mnw,:),1))/sqrt(ns0));
    fprintf('  hold 11.5-12.5 s:  pred %+.4f   actual %+.4f +- %.4f\n', mean(E4(mh)),  mean(mean(S.E4A(mh,:),1)),  std(mean(S.E4A(mh,:),1))/sqrt(ns0));
    fprintf('  trajectory corr(pred, actual) over t > 0.5: %+.3f\n', corr(E4(t > 0.5), act(t > 0.5)));
    out = struct('t', t, 'G3c', G3c, 'G4c', G4c, 'comp', comp, 'E4', E4, 'act', act);
    save(fullfile(od, 'meng_btrue_round2.mat'), 'out');
    fprintf('MENG ROUND2 DONE\n');
end
