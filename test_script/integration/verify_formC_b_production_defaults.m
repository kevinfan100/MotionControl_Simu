% VERIFY_FORMC_B_PRODUCTION_DEFAULTS  Acceptance for the 2026-09-04 production defaults of formC_b.
%   The four blocks law_exact_step + pred_mean2 + nw_mcorr + pred_mean2_e4 are ON by default since 2026-09-04
%   (derivation reference/eq17_analysis/derivation/0903_aptrue_4state_from_true.tex S1-S11). Self-contained
%   (no stored results needed). Three checks, z axis, canonical deep scenario:
%     V1  default ctrl_const == the four flags set explicitly true          (bit-identical, arm 'best', seeds 1:3)
%     V2  the four flags explicitly false runs and is finite (the pre-09-04 Euler recipe), and differs from V1
%         (the flags are wired)
%     V3  every driver arm and both oracle arms run to completion with the defaults, 1 seed each:
%         b1 / bmid / best / b98 / bfree1, ap_known@est + app_known (lock_b), b_true@true; health = finite,
%         w_bar above contact, a_hat above its floor
%   Also asserts the dependency defaults: law_exact_step = false alone must NOT error (pred_mean2 and pred_mean2_e4
%   follow it), and pred_mean2 = true with law_exact_step = false must error.
%   Usage: verify_formC_b_production_defaults   (prints PASS/FAIL per check, errors on the first FAIL)
function verify_formC_b_production_defaults()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(here);
    seeds = 1:3;
    base = struct('arm','best','scenario','deep','verbose',false,'seeds',seeds,'log_P_full',false);
    ON  = struct('law_exact_step',true,'pred_mean2',true,'nw_mcorr',true,'pred_mean2_e4',true);
    OFF = struct('law_exact_step',false,'pred_mean2',false,'nw_mcorr',false,'pred_mean2_e4',false);
    E_def = local_E(base, struct());
    E_on  = local_E(base, ON);
    d = max(abs(E_def - E_on), [], 'all');
    fprintf('V1 default == explicit ON: max |dE| = %.3e  -> %s\n', d, local_pf(d == 0));
    assert(d == 0, 'V1 FAIL: the production defaults are not the four flags');
    E_off = local_E(base, OFF);
    d2 = max(abs(E_def - E_off), [], 'all');
    fprintf('V2 explicit OFF finite (%d) and differs from default: max |dE| = %.3e  -> %s\n', all(isfinite(E_off(:))), d2, local_pf(all(isfinite(E_off(:))) && d2 > 1e-6));
    assert(all(isfinite(E_off(:))) && d2 > 1e-6, 'V2 FAIL');
    % dependency defaults
    ok_dep = true;
    try; local_E(setfield(base, 'seeds', 1), struct('law_exact_step', false)); catch; ok_dep = false; end
    ok_err = false;
    try; local_E(setfield(base, 'seeds', 1), struct('law_exact_step', false, 'pred_mean2', true)); catch; ok_err = true; end
    fprintf('V2b law_exact_step=false alone runs (%d), pred_mean2 without exact step errors (%d)  -> %s\n', ok_dep, ok_err, local_pf(ok_dep && ok_err));
    assert(ok_dep && ok_err, 'V2b FAIL: dependency defaults');
    % arm smoke
    arms = {'b1','bmid','best','b98','bfree1'};
    for a = 1:numel(arms)
        o = base; o.arm = arms{a}; o.seeds = 7;
        local_smoke(o, struct(), arms{a});
    end
    o = base; o.seeds = 7; o.ap_known = true; o.ap_known_at = 'est'; o.app_known = true;
    local_smoke(o, struct('lock_b', true), 'ap_known@est');
    o = base; o.seeds = 7; o.b_true = true; o.b_true_at = 'true';
    local_smoke(o, struct(), 'b_true@true');
    fprintf('verify_formC_b_production_defaults: ALL PASS\n');
end

function E = local_E(o, cc)
    o.ctrl_const_override = cc;
    clear run_formC_b motion_control_law_formC_b;
    evalc('R = run_formC_b(o);');
    nS = numel(R.runs);  N = numel(R.runs{1}.tout);  E = zeros(N, nS);
    for q = 1:nS; r = R.runs{q}; E(:,q) = r.a_bar_hat_out(:,3) - r.a_true_out(:,3) / (r.a_hat_out(1,3)/r.a_bar_hat_out(1,3)); end
end

function local_smoke(o, cc, tag)
    o.ctrl_const_override = cc;
    clear run_formC_b motion_control_law_formC_b;
    evalc('R = run_formC_b(o);');
    r = R.runs{1};  E = r.a_bar_hat_out(:,3) - r.a_true_out(:,3) / (r.a_hat_out(1,3)/r.a_bar_hat_out(1,3));
    ok = all(isfinite(E)) && min(r.h_bar_true_out(:,1)) > 1.0 && min(r.a_bar_hat_out(:,3)) > 0;
    fprintf('V3 %-12s finite %d | min w %.4f | min a_hat %.4f | hold est-true %+.5f  -> %s\n', tag, all(isfinite(E)), ...
        min(r.h_bar_true_out(:,1)), min(r.a_bar_hat_out(:,3)), mean(E(r.tout > R.cfg.t_hold + R.cfg.t_descend_override + R.cfg.n_cycles/R.cfg.frequency)), local_pf(ok));
    assert(ok, 'V3 FAIL: %s', tag);
end

function s = local_pf(ok)
    if ok; s = 'PASS'; else; s = 'FAIL'; end
end
