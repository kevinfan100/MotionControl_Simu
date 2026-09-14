% VERIFY_FORMC_B_AMODEL_FE  Acceptance for ctrl_const.amodel_fe (the "green" variant, default off).
%   Derivation: reference/eq17_analysis/derivation/0911_estb_green_variant.tex. The flag changes ONLY the covariance
%   model of formC_b: in F_e rows 3-4 the gain error carried by the step error, F_dw e_4, is replaced by F_dw psi with
%   psi = -a' e_3 + ((1-a)/b) e_b, and the second-order mean term of the a_w predict follows (Cov(e_4,u) -> Cov(psi,u),
%   u built with the green step-error vector c). H, Q, R, P[0], the mean predict and the control law are untouched.
%   Checks (z axis, canonical deep scenario, arm 'best'):
%     A1  explicit amodel_fe = false == default, bit-identical (seeds 1:2)                       [guard]
%     A2  amodel_fe = true runs, is finite and differs from the default (the flag is wired)
%     A3  amodel_fe = true: every obs_dump F_e satisfies S4 Green -- F34 = 0 exactly, F35 = -F_dw (1-a)/b,
%         F43 = (1-lc) a' - F_dw a'^2, F44 = 1 + A_a M with M recovered from F45 = (1-a)^2 M + F_dw (1-a)^3,
%         absolute residual <= 1e-12, and the green change itself is not vacuous (max |F_dw a'| > 1e-4)
%     A4  instrument first: with the flag OFF the vector form u = c_orig' e + w_T reproduces the logged pred_mean2
%         term (this fixes the dump/log alignment); then with the flag ON the logged term equals S7 Green
%         -a'' Cov(e3,u) - (1-a)^2 Cov(e_b,u) + a''/2 Var(u) with c_green
%     A5  health, flag ON, canon and Meng: finite, w above contact, a_hat above 0, b_hat inside (0.5, 1.5)
%     A6  amodel_fe with jac_exact_step, and amodel_fe with pred_mean2_e4 = false, must error
%   Checks B (the "orange" variant, ctrl_const.amodel_h, derivation 0912_estb_orange_variant.tex): the same substitution
%   in the y2 row, H2(4) -> 0 with its share moved onto e_3 and e_b (y2_pred, Q, R, P[0] and the mean predict unchanged):
%     B1  amodel_h = false == amodel_fe alone, bit-identical                                    [guard]
%     B2  amodel_h = true runs, is finite and differs from green (the flag is wired)
%     B3a every logged y2 row has H24 = 0 exactly and a non-vacuous moved share
%     B3b at the first y2 step (both arms bit-identical up to it) the two rows differ only by the redistribution:
%         a' and (1-a)/b recovered from them give back the record's own b_hat
%     B4  health, orange on, canon and Meng
%     B5  amodel_h without amodel_fe must error
%   Checks C (the self-consistent 4-state, ctrl_const.gain_algebraic, derivation 0914_estb_4state_core.tex): the gain
%   slot stops being a state -- it is overwritten every step with the law value at the estimator's own height and its
%   row/col of P is locked, so the filter is x = [dw1 dw2 dw3 b] with a derived gain output:
%     C1  gain_algebraic = false == orange, bit-identical                                     [guard]
%     C2  gain_algebraic = true runs, is finite and differs from orange (the flag is wired)
%     C3  a_hat equals the law at (w_d - dw3_hat, b_hat) at every unclamped step
%     C4  P(4,:) = 0 exactly in every record
%     C5  gain_algebraic without amodel_h must error
%     C6  health, 4-state on, canon and Meng
%   Usage: verify_formC_b_amodel_fe   (prints PASS/FAIL per check, errors on the first FAIL)
function verify_formC_b_amodel_fe()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(here);
    base = struct('arm','best','scenario','deep','verbose',false,'seeds',1:2,'log_P_full',false);
    % ---- A1 guard ----
    E_def = local_E(base, struct());
    E_off = local_E(base, struct('amodel_fe', false));
    d1 = max(abs(E_def - E_off), [], 'all');
    fprintf('A1 amodel_fe=false == default: max |dE| = %.3e  -> %s\n', d1, local_pf(d1 == 0));
    assert(d1 == 0, 'A1 FAIL: the flag is not off by default');
    % ---- A3 / A4 ----
    lc = 0.7;  al = 1 - lc;
    [L0, pm0] = local_dump(base, struct());
    [off, err0, scale0] = local_align(L0, pm0, lc, al, 'orig');
    fprintf('A4a instrument: vector form c_orig reproduces the logged pred_mean2 (flag off): max err %.2e of scale %.2e, log offset %d  -> %s\n', ...
            err0, scale0, off, local_pf(err0 <= 1e-9 * scale0));
    assert(err0 <= 1e-9 * scale0, 'A4a FAIL: the test instrument does not reproduce production (fix the test, not the code)');
    % ---- A2 wired ----
    E_on = local_E(base, struct('amodel_fe', true));
    d2 = max(abs(E_def - E_on), [], 'all');
    ok2 = all(isfinite(E_on(:))) && d2 > 1e-6;
    fprintf('A2 amodel_fe=true finite (%d) and differs: max |dE| = %.3e  -> %s\n', all(isfinite(E_on(:))), d2, local_pf(ok2));
    assert(ok2, 'A2 FAIL: the flag is not wired');
    [L1, pm1] = local_dump(base, struct('amodel_fe', true));
    [e3, f34, gsz] = local_check_F(L1, lc);
    ok3 = e3 <= 1e-12 && f34 == 0 && gsz > 1e-4;
    fprintf('A3 green F_e rows 3-4: max |residual| %.2e, max |F34| %.1e, size of the green change max|F_dw a''| %.2e over %d records  -> %s\n', ...
            e3, f34, gsz, numel(L1) - 1, local_pf(ok3));
    assert(ok3, 'A3 FAIL: F_e rows 3-4 do not follow the green substitution');
    [err1, scale1] = local_mean_err(L1, pm1, lc, al, 'green', off);
    fprintf('A4b green second-order mean term: max err %.2e of scale %.2e  -> %s\n', err1, scale1, local_pf(err1 <= 1e-9 * scale1));
    assert(err1 <= 1e-9 * scale1, 'A4b FAIL: the second-order mean does not follow the green step error');
    % ---- A5 health ----
    local_smoke(setfield(base, 'seeds', 7), struct('amodel_fe', true), 'canon', struct());
    OV = struct('trajectory_type','osc','h_init',15,'h_bottom',2.5,'amplitude',0,'frequency',1, ...
                'n_cycles',1,'t_hold',0.5,'t_descend_override',10,'T_sim',12.5,'h_min',2.475);
    local_smoke(setfield(base, 'seeds', 7), struct('amodel_fe', true), 'meng', OV);
    % ---- A6 incompatible combinations ----
    b1 = setfield(base, 'seeds', 1);
    e_jac = false; try; local_E(b1, struct('amodel_fe', true, 'jac_exact_step', true)); catch; e_jac = true; end
    e_e4  = false; try; local_E(b1, struct('amodel_fe', true, 'pred_mean2_e4', false)); catch; e_e4 = true; end
    fprintf('A6 amodel_fe + jac_exact_step errors (%d), amodel_fe + pred_mean2_e4=false errors (%d)  -> %s\n', e_jac, e_e4, local_pf(e_jac && e_e4));
    assert(e_jac && e_e4, 'A6 FAIL: incompatible combinations are accepted');
    % ---- B: amodel_h, the "orange" variant (0912_estb_orange_variant.tex) ----
    E_gh0 = local_E(base, struct('amodel_fe', true, 'amodel_h', false));
    db1 = max(abs(E_on - E_gh0), [], 'all');
    fprintf('B1 amodel_h=false == amodel_fe alone: max |dE| = %.3e  -> %s\n', db1, local_pf(db1 == 0));
    assert(db1 == 0, 'B1 FAIL: amodel_h is not off by default');
    E_gh1 = local_E(base, struct('amodel_fe', true, 'amodel_h', true));
    db2 = max(abs(E_on - E_gh1), [], 'all');
     okb2 = all(isfinite(E_gh1(:))) && db2 > 1e-6;
    fprintf('B2 amodel_h=true finite (%d) and differs from green: max |dE| = %.3e  -> %s\n', all(isfinite(E_gh1(:))), db2, local_pf(okb2));
    assert(okb2, 'B2 FAIL: amodel_h is not wired');
    [Lg, ~] = local_dump(base, struct('amodel_fe', true));
    [Lo, ~] = local_dump(base, struct('amodel_fe', true, 'amodel_h', true));
    [h24, h23sz] = local_check_H(Lo);
    ok3 = h24 == 0 && h23sz > 1e-6;
    fprintf('B3a orange H row 2: max |H24| %.1e, size of the moved share max |H23| %.2e  -> %s\n', h24, h23sz, local_pf(ok3));
    assert(ok3, 'B3a FAIL: the y2 row does not follow the orange substitution');
    [dx, eb, szh, k1] = local_first_y2(Lg, Lo);
    ok4 = dx == 0 && eb <= 1e-12 && szh > 1e-6;
    fprintf('B3b first y2 step (record %d): arms bit-identical up to it %.1e | b_hat recovered from the two rows, err %.2e | moved share |H24| %.2e  -> %s\n', ...
            k1, dx, eb, szh, local_pf(ok4));
    assert(ok4, 'B3b FAIL: the orange row is not the original row with e_4 redistributed');
    local_smoke(setfield(base, 'seeds', 7), struct('amodel_fe', true, 'amodel_h', true), 'canon-o', struct());
    local_smoke(setfield(base, 'seeds', 7), struct('amodel_fe', true, 'amodel_h', true), 'meng-o', OV);
    e_h = false; try; local_E(b1, struct('amodel_h', true)); catch; e_h = true; end
    fprintf('B5 amodel_h without amodel_fe errors (%d)  -> %s\n', e_h, local_pf(e_h));
    assert(e_h, 'B5 FAIL: amodel_h is accepted without amodel_fe');
    % ---- C: gain_algebraic, the self-consistent 4-state (0914_estb_4state_core.tex) ----
    orange = struct('amodel_fe', true, 'amodel_h', true);
    E_g0 = local_E(base, setfield(orange, 'gain_algebraic', false));
    dc1 = max(abs(E_gh1 - E_g0), [], 'all');
    fprintf('C1 gain_algebraic=false == orange: max |dE| = %.3e  -> %s\n', dc1, local_pf(dc1 == 0));
    assert(dc1 == 0, 'C1 FAIL: gain_algebraic is not off by default');
    E_g1 = local_E(base, setfield(orange, 'gain_algebraic', true));
    dc2 = max(abs(E_gh1 - E_g1), [], 'all');
    okc2 = all(isfinite(E_g1(:))) && dc2 > 1e-6;
    fprintf('C2 gain_algebraic=true finite (%d) and differs from orange: max |dE| = %.3e  -> %s\n', all(isfinite(E_g1(:))), dc2, local_pf(okc2));
    assert(okc2, 'C2 FAIL: gain_algebraic is not wired');
    [L4, ~, r4] = local_dump(base, setfield(orange, 'gain_algebraic', true));
    [dlaw, nrow] = local_check_law(r4);
    fprintf('C3 a_hat equals the law at the estimator height every step: max |da| = %.2e over %d unclamped rows  -> %s\n', ...
            dlaw, nrow, local_pf(dlaw <= 1e-12 && nrow > 1000));
    assert(dlaw <= 1e-12 && nrow > 1000, 'C3 FAIL: the gain slot is not the algebraic law value');
    p44 = max(arrayfun(@(s) max(abs(s.P_upd(4,:))), L4));
    fprintf('C4 the gain slot carries no covariance: max |P(4,:)| = %.1e  -> %s\n', p44, local_pf(p44 == 0));
    assert(p44 == 0, 'C4 FAIL: slot 4 is not locked');
    e_ga = false; try; local_E(b1, struct('amodel_fe', true, 'gain_algebraic', true)); catch; e_ga = true; end
    fprintf('C5 gain_algebraic without amodel_h errors (%d)  -> %s\n', e_ga, local_pf(e_ga));
    assert(e_ga, 'C5 FAIL: gain_algebraic is accepted without amodel_h');
    local_smoke(setfield(base, 'seeds', 7), setfield(orange, 'gain_algebraic', true), 'canon-4', struct());
    local_smoke(setfield(base, 'seeds', 7), setfield(orange, 'gain_algebraic', true), 'meng-4', OV);
    fprintf('verify_formC_b_amodel_fe: ALL PASS\n');
end

function [d, n] = local_check_law(r)
    % the 4-state contract: a_hat[k] = 1 - 1/( b_hat (w_d - dw3_hat - w_s) ), read from the run's own logs.
    % Rows where the clamp binds are excluded (the law value is outside [a_bar_floor, a_bar_ceil] there).
    ws = r.ctrl_const.ws0_perp - 1;  fl = 0.05;  ce = 0.999;
    wd = r.h_bar_d_out(:);  d3 = r.delta_x_hat_3_out(:,3) / r.R;  bh = r.b_hat_out(:,3);  ah = r.a_bar_hat_out(:,3);
    law = 1 - 1 ./ (bh .* (wd - d3 - ws));
    ok = isfinite(law) & law > fl + 1e-6 & law < ce - 1e-6 & (1:numel(law))' > 5;
    d = max(abs(ah(ok) - law(ok)));  n = sum(ok);
    if isempty(d); d = Inf; end
end

function [h24, h23sz] = local_check_H(L)
    % orange: the y2 row has no a_w column left, and the share it carried has moved onto e_3
    h24 = 0;  h23sz = 0;
    for k = 1:numel(L)
        H2 = L(k).H{2};  if isempty(H2); continue; end
        h24 = max(h24, abs(H2(4)));  h23sz = max(h23sz, abs(H2(3)));
    end
end

function [dx, eb, sz, k1] = local_first_y2(Lg, Lo)
    % Up to the FIRST step that carries a y2 row the two arms are bit-identical, so at that step their rows differ
    % only by the redistribution: H^o(3) = H(3) - H(4) a', H^o(5) = H(5) + H(4) (1-a)/b. Recover a' and (1-a)/b from
    % the two rows; the law then fixes b = (a' / ((1-a)/b)^2)^(1/3), which must be the record's own b_hat (F55 = 1, so
    % x_pred(5) is the value the step used). That is the one constraint the pair of rows has to satisfy -- it does not
    % need to know which a_hat the slope reads (ap_src), which is why the first version of this check failed at 1e-9.
    k1 = NaN;  dx = Inf;  eb = Inf;  sz = 0;
    for k = 1:min(numel(Lg), numel(Lo))
        if isempty(Lg(k).H{2}) || isempty(Lo(k).H{2}); continue; end
        k1 = k;  H = Lg(k).H{2};  Ho = Lo(k).H{2};
        dx = max(abs(Lg(k).x_pred - Lo(k).x_pred));
        for j = 1:(k-1); dx = max(dx, max(abs(Lg(j).x_upd - Lo(j).x_upd))); end
        ap = (H(3) - Ho(3)) / H(4);  cb = (Ho(5) - H(5)) / H(4);  sz = abs(H(4));
        eb = abs((ap / cb^2)^(1/3) - Lg(k).x_pred(5));
        return;
    end
end

function E = local_E(o, cc)
    o.ctrl_const_override = cc;
    clear run_formC_b motion_control_law_formC_b;
    evalc('R = run_formC_b(o);');
    nS = numel(R.runs);  N = numel(R.runs{1}.tout);  E = zeros(N, nS);
    for q = 1:nS; r = R.runs{q}; E(:,q) = r.a_bar_hat_out(:,3) - r.a_true_out(:,3) / (r.a_hat_out(1,3)/r.a_bar_hat_out(1,3)); end
end

function [L, pm, r] = local_dump(o, cc)
    cc.obs_dump = true;  o.seeds = 1;  o.ctrl_const_override = cc;
    clear run_formC_b motion_control_law_formC_b;
    evalc('R = run_formC_b(o);');
    L = obs_dump('get');  L = L([L.ax] == 3);  r = R.runs{1};  pm = r.pred_mean2_out(:, 3);
end

function [m, c] = local_green_or_orig(Lp, Lk, lc, al, which)
    % second-order mean term at record k from the previous posterior (0911 tex S5-S7); returns the term and c
    x = Lp.x_upd;  a = x(4);  b = x(5);  P = Lp.P_upd;  F = Lk.F;
    ap = b*(1-a)^2;  Aa = -2*b*(1-a);  app = -2*ap^2/(1-a);  cb = (1-a)/b;
    Q = Lk.P_pred - F*P*F';  s2T = Q(3,3);
    c = zeros(size(F,1), 1);  c(1) = -(1-lc);  c(8) = al;  c(9) = al;
    if strcmp(which, 'orig')
        Fdw = -F(3,4);  c(3) = 1 - lc;  c(4) = Fdw;
    else
        Fdw = (F(3,3) - lc)/ap;  c(3) = (1 - lc) - Fdw*ap;  c(5) = Fdw*cb;
    end
    cv = P*c;  vu = c'*P*c + s2T;
    if strcmp(which, 'orig')
        m = Aa*cv(4) + (ap/b)*cv(5) + 0.5*app*vu;
    else
        m = -app*cv(3) - (1-a)^2*cv(5) + 0.5*app*vu;
    end
end

function [off, err, scale] = local_align(L, pm, lc, al, which)
    best = Inf;  off = 0;
    for o = -3:3
        [e, s] = local_mean_err(L, pm, lc, al, which, o);
        if e < best; best = e; off = o; scale = s; end
    end
    err = best;
end

function [err, scale] = local_mean_err(L, pm, lc, al, which, off)
    N = numel(L);  d = nan(N, 1);  v = nan(N, 1);
    for k = 2:N
        row = double(L(k).k) + off;
        if row < 1 || row > numel(pm); continue; end
        m = local_green_or_orig(L(k-1), L(k), lc, al, which);
        d(k) = pm(row) - m;  v(k) = m;
    end
    err = max(abs(d), [], 'omitnan');  scale = max(abs(v), [], 'omitnan');
    if isempty(err) || ~isfinite(err); err = Inf; scale = 1; end
end

function [worst, f34, gsz] = local_check_F(L, lc)
    % absolute residuals: the entries are O(1) or smaller and F_dw is recovered from F33 - lc, whose cancellation near
    % a hold makes a RELATIVE measure meaningless (2026-09-11: abs residual 2e-16 read as rel 1e-9 at |F_dw| = 2.5e-6)
    worst = 0;  f34 = 0;  gsz = 0;
    for k = 2:numel(L)
        x = L(k-1).x_upd;  a = x(4);  b = x(5);  F = L(k).F;
        ap = b*(1-a)^2;  Aa = -2*b*(1-a);  cb = (1-a)/b;
        Fdw = (F(3,3) - lc)/ap;
        M = (F(4,5) - Fdw*(1-a)^3)/(1-a)^2;
        r = [abs(F(3,5) + Fdw*cb), abs(F(4,3) - ((1-lc)*ap - Fdw*ap^2)), abs(F(4,4) - (1 + Aa*M))];
        worst = max(worst, max(r));  f34 = max(f34, abs(F(3,4)));  gsz = max(gsz, abs(Fdw*ap));
    end
end

function local_smoke(o, cc, tag, OV)
    o.ctrl_const_override = cc;  o.config_override = OV;
    clear run_formC_b motion_control_law_formC_b;
    evalc('R = run_formC_b(o);');
    r = R.runs{1};  E = r.a_bar_hat_out(:,3) - r.a_true_out(:,3) / (r.a_hat_out(1,3)/r.a_bar_hat_out(1,3));
    bh = r.b_hat_out(:,3);
    ok = all(isfinite(E)) && min(r.h_bar_true_out(:,1)) > 1.0 && min(r.a_bar_hat_out(:,3)) > 0 && all(bh > 0.5 & bh < 1.5);
    fprintf('A5 %-6s finite %d | min w %.4f | min a_hat %.4f | b_hat %.4f .. %.4f  -> %s\n', tag, all(isfinite(E)), ...
            min(r.h_bar_true_out(:,1)), min(r.a_bar_hat_out(:,3)), min(bh), max(bh), local_pf(ok));
    assert(ok, 'A5 FAIL: %s', tag);
end

function s = local_pf(ok)
    if ok; s = 'PASS'; else; s = 'FAIL'; end
end
