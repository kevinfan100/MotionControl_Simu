% VERIFY_FORMC_B_LAW_ERR_Q44  Acceptance for ctrl_const.law_err_q44 (default off).
%   Derivation: reference/eq17_analysis/derivation/0915_estb_law_error_q44.tex. The flag adds ONE term to the process
%   noise of the gain slot -- what the filter admits about its own law being wrong:
%       dQ44[k] = ( (1-a)^2 * delta_b * M_law[k] )^2 ,   dQ45 = 0,   M_law = the step the law actually integrates
%   with delta_b = sup |b_true - b_hat| over the envelope, defaulting to the SAME envelope as the b prior (Pf_b_std),
%   so no new number enters. Nothing else changes: F_e, H, R, P[0], the mean predict and the control law are untouched.
%   Checks (z axis, canonical deep scenario, arm 'best'):
%     D1  explicit law_err_q44 = false == default, bit-identical (seeds 1:2)                     [guard]
%     D2  law_err_q44 = true runs, is finite and differs from the default (the flag is wired)
%     D3  at the first step (both runs still identical) the recovered Q(4,4) differs by exactly the injection
%         computed from that record's own state; and Q(4,5) does not change
%     D4  mechanism: the injection is path-driven -- the ratio dQ44 / Q44_thermal in hold must be below 5% of its
%         descent value (in hold the commanded step is zero but dw3_hat still jitters, so it is small, not zero)
%     D5  health, flag ON, canon and Meng: finite, w above contact, a_hat above 0, b_hat inside (0.5, 1.5)
%     D6  reported (not asserted here, measured properly in the closed-loop probe): the y2 share of the gain update
%         on the descent, against the 2026-09-14 baseline 0.02% (canon) / 15% (Meng)
%   Usage: verify_formC_b_law_err_q44   (prints PASS/FAIL per check, errors on the first FAIL)
function verify_formC_b_law_err_q44()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(here);
    base = struct('arm','best','scenario','deep','verbose',false,'seeds',1:2,'log_P_full',false);
    E_def = local_E(base, struct());
    E_off = local_E(base, struct('law_err_q44', false));
    d1 = max(abs(E_def - E_off), [], 'all');
    fprintf('D1 law_err_q44=false == default: max |dE| = %.3e  -> %s\n', d1, local_pf(d1 == 0));
    assert(d1 == 0, 'D1 FAIL: the flag is not off by default');
    E_on = local_E(base, struct('law_err_q44', true));
    d2 = max(abs(E_def - E_on), [], 'all');
    ok2 = all(isfinite(E_on(:))) && d2 > 1e-9;
    fprintf('D2 law_err_q44=true finite (%d) and differs: max |dE| = %.3e  -> %s\n', all(isfinite(E_on(:))), d2, local_pf(ok2));
    assert(ok2, 'D2 FAIL: the flag is not wired');
    [L0, r0] = local_dump(base, struct());
    [L1, ~]  = local_dump(base, struct('law_err_q44', true));
    [dq, dq45, k1, inj] = local_first_step(L0, L1, r0);
    ok3 = dq <= 1e-6 * inj && dq45 == 0 && inj > 0;   % 1e-6: Q is RECOVERED as P_pred - F P_upd F', a cancellation of ~1e-9 numbers
    fprintf('D3 first step (record %d): |dQ44 - injection| = %.2e (injection %.2e), max |dQ45| = %.1e  -> %s\n', ...
            k1, dq, inj, dq45, local_pf(ok3));
    assert(ok3, 'D3 FAIL: the injection does not follow the derivation');
    [rh, rd] = local_segment_ratio(L1, r0);
    ok4 = rh < 0.05 * rd && rd > 1e-3;
    fprintf('D4 injection / thermal Q44: hold %.2e, descent %.2e  -> %s\n', rh, rd, local_pf(ok4));
    assert(ok4, 'D4 FAIL: the injection is not path-driven');
    local_smoke(setfield(base, 'seeds', 7), struct('law_err_q44', true), 'canon', struct());
    OV = struct('trajectory_type','osc','h_init',15,'h_bottom',2.5,'amplitude',0,'frequency',1, ...
                'n_cycles',1,'t_hold',0.5,'t_descend_override',10,'T_sim',12.5,'h_min',2.475);
    local_smoke(setfield(base, 'seeds', 7), struct('law_err_q44', true), 'meng', OV);
    local_share(L0, L1, r0);
    fprintf('verify_formC_b_law_err_q44: ALL PASS\n');
end

function E = local_E(o, cc)
    o.ctrl_const_override = cc;
    clear run_formC_b motion_control_law_formC_b;
    evalc('R = run_formC_b(o);');
    nS = numel(R.runs);  N = numel(R.runs{1}.tout);  E = zeros(N, nS);
    for q = 1:nS; r = R.runs{q}; E(:,q) = r.a_bar_hat_out(:,3) - r.a_true_out(:,3) / (r.a_hat_out(1,3)/r.a_bar_hat_out(1,3)); end
end

function [L, r] = local_dump(o, cc)
    cc.obs_dump = true;  o.seeds = 1;  o.ctrl_const_override = cc;
    clear run_formC_b motion_control_law_formC_b;
    evalc('R = run_formC_b(o);');
    L = obs_dump('get');  L = L([L.ax] == 3);  r = R.runs{1};
end

function [dq, dq45, k1, inj] = local_first_step(L0, L1, r)
    % Q = P_pred - F P_upd F' at the first record whose states still agree AND whose step is nonzero (the run starts in
    % a hold). The step is NOT rebuilt from the logs -- it is read back from the dump itself through F_e(4,5) =
    % (1-a)^2 * M_tot, which removes every convention question (MA(2) share, alignment); the only piece added by hand
    % is the nw_mcorr residual (1-lc) * res1, and res1 = innov1 - (x_upd(1) - x_pred(1)) is exact from the same record.
    k1 = 2;
    for k = 2:numel(L0)
        if max(abs(L0(k).x_pred - L1(k).x_pred)) > 0; break; end
        k1 = k;
    end
    lc = 0.7;  db = r.ctrl_const.Pf_b_std;
    a = L0(k1-1).x_upd(4);  dap_db = (1 - a)^2;
    M_tot = L0(k1).F(4,5) / dap_db;
    iv = r.innov_y1_out(:,3);  row = double(L0(k1-1).k) + 1;
    res1 = iv(row) - (L0(k1-1).x_upd(1) - L0(k1-1).x_pred(1));
    M_law = M_tot + (1 - lc) * res1;
    inj = (dap_db * db * M_law)^2;
    Q0 = L0(k1).P_pred - L0(k1).F * L0(k1-1).P_upd * L0(k1).F';
    Q1 = L1(k1).P_pred - L1(k1).F * L1(k1-1).P_upd * L1(k1).F';
    dq   = abs((Q1(4,4) - Q0(4,4)) - inj);
    dq45 = max(abs(Q1(4,5) - Q0(4,5)), abs(Q1(5,4) - Q0(5,4)));
end

function [rh, rd] = local_segment_ratio(L, r)
    % injection / thermal, read from the ON run: Q44 with the term vs the same Q44 rebuilt without it is not available
    % here, so the reported ratio uses the closed form of the injection against the recovered Q44.
    lc = 0.7;  wd = r.h_bar_d_out(:);  d3 = r.delta_x_hat_3_out(:,3) / r.R;  db = r.ctrl_const.Pf_b_std;
    n = numel(L);  Ts = 0.000625;  t = (double([L.k]) - double(L(1).k)) * Ts;
    inj = zeros(n,1);  q44 = zeros(n,1);
    for k = 2:n
        x = L(k-1).x_upd;  a = x(4);
        row = double(L(k).k) + 1;
        if row < 2 || row > numel(wd); continue; end
        dap = (1 - a)^2;
        inj(k) = (dap * db * (L(k).F(4,5) / dap))^2;
        Q = L(k).P_pred - L(k).F * L(k-1).P_upd * L(k).F';
        q44(k) = Q(4,4);
    end
    mh = t(:) > 3.5;  md = t(:) > 0.5 & t(:) <= 1.5;
    rh = median(inj(mh) ./ max(q44(mh) - inj(mh), eps));
    rd = median(inj(md) ./ max(q44(md) - inj(md), eps));
end

function local_share(L0, L1, ~)
    % D6, reported only: how much of the gain update comes from the data, before and after
    for j = 1:2
        L = L0;  nm = 'off';  if j == 2; L = L1; nm = 'on '; end
        n = numel(L);  Ts = 0.000625;  t = (double([L.k]) - double(L(1).k)) * Ts;
        law = zeros(n,1);  dat = zeros(n,1);
        for k = 2:n
            law(k) = L(k).x_pred(4) - L(k-1).x_upd(4);
            dat(k) = L(k).x_upd(4)  - L(k).x_pred(4);
        end
        m = t(:) > 0.5 & t(:) <= 1.5;
        fprintf('D6 law_err_q44 %s | descent: sum law %+.4f, sum data %+.4f, data share of |increment| %.3f\n', ...
                nm, sum(law(m)), sum(dat(m)), sum(abs(dat(m)))/(sum(abs(dat(m))) + sum(abs(law(m)))));
    end
end

function local_smoke(o, cc, tag, OV)
    o.ctrl_const_override = cc;  o.config_override = OV;
    clear run_formC_b motion_control_law_formC_b;
    evalc('R = run_formC_b(o);');
    r = R.runs{1};  E = r.a_bar_hat_out(:,3) - r.a_true_out(:,3) / (r.a_hat_out(1,3)/r.a_bar_hat_out(1,3));
    bh = r.b_hat_out(:,3);
    ok = all(isfinite(E)) && min(r.h_bar_true_out(:,1)) > 1.0 && min(r.a_bar_hat_out(:,3)) > 0 && all(bh > 0.5 & bh < 1.5);
    fprintf('D5 %-6s finite %d | min w %.4f | min a_hat %.4f | b_hat %.4f .. %.4f  -> %s\n', tag, all(isfinite(E)), ...
            min(r.h_bar_true_out(:,1)), min(r.a_bar_hat_out(:,3)), min(bh), max(bh), local_pf(ok));
    assert(ok, 'D5 FAIL: %s', tag);
end

function s = local_pf(ok)
    if ok; s = 'PASS'; else; s = 'FAIL'; end
end
