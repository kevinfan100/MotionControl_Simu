% PURPOSE: verify -- against the run log, not against the derivation's own claim -- the
% structural argument "Q_theta = 0  =>  P monotone non-increasing  =>  one learning window
% then frozen  =>  no within-run recovery".  Reads the cached widened-b-prior arm
% (plot_formB_b_widened_prior.m, sqrt(P_bb)[0] = 1.00, plant b = 0.13).
% EXPIRES: when the no-recovery question is settled one way or the other.
%
% STATUS: ACTIVE -- verifies the Q_theta = 0 => P monotone => frozen argument against the
%   run log rather than the derivation's own claim (B5 in the 08-06 probe).
%   EXPIRES: when the no-recovery question (B8) is settled either way.
%   See memory project-formb-bonly-prior-reproducibility-2026-08-06.
%
% Two-step audit (.claude/rules/derivation-workflow.md rule 4):
%   step 1 -- premise vs code: F_e rows 5-7 are identity, Q(5:7,5:7) = 0 (checked by eye)
%   step 2 -- quantity vs log: rebuild "travel" and "P reduction" per phase from the log

here = fileparts(mfilename('fullpath'));
proj = fullfile(here, '..', '..');
cache = fullfile(proj, 'test_results', 'formB_bwide_shown.mat');
assert(exist(cache, 'file') == 2, 'run plot_formB_b_widened_prior.m first');
S = load(cache);  out = S.out_shown;

AX_Z   = 3;
B_TRUE = 0.13;
t1 = out.cfg.t_hold;
t2 = t1 + out.cfg.t_descend_override;
t3 = t2 + out.cfg.n_cycles / out.cfg.frequency;

fprintf('\n=== structural check: Q_theta = 0 => monotone P => freeze ===\n');
for q = 1:numel(out.seeds)
    r  = out.runs{q};
    t  = r.tout(:);
    b  = r.b_hat_out(:, AX_Z);
    sP = r.P_b_out(:, AX_Z);        % driver stores SQRT of the variance
    P  = sP.^2;

    % --- claim 1: P is monotone non-increasing (Q = 0 + identity F rows) ---
    dP     = diff(P);
    n_up   = sum(dP > 0);
    max_up = max([dP; -Inf]);
    fprintf('\nseed %d\n', out.seeds(q));
    fprintf('  P monotone: %d of %d steps increase (max increase %.3e)  -> %s\n', ...
            n_up, numel(dP), max(max_up, 0), ...
            local_verdict(n_up == 0 || max_up < 1e-12 * P(1)));

    % --- claim 2: essentially all learning happens in ONE window ---
    idx = @(a, bnd) find(t >= a & t < bnd);
    ph  = {'hold1', [0 t1]; 'descent', [t1 t2]; 'osc', [t2 t3]; 'hold2', [t3 t(end)]};
    % NOTE do NOT use (P_start - P_end)/total_drop as the "learning share": the
    % variance drop is dominated by the first large fall and hides everything after
    % it.  Report the width sqrt(P) and the confidence 1/P instead -- 1/P is the
    % quantity that adds up when independent evidence arrives.
    fprintf('  %-8s | %7s | %8s | %9s | %8s | %9s\n', ...
            'phase-end', 'sqrt P', 'conf 1/P', 'b_hat', '|b-true|', 'in sigma');
    fprintf('  %-8s | %7.4f | %8.1f | %9.4f | %8.4f | %8.2f\n', ...
            'start', sP(1), 1 / P(1), b(1), abs(b(1) - B_TRUE), abs(b(1) - B_TRUE) / sP(1));
    for k = 1:size(ph, 1)
        w = idx(ph{k, 2}(1), ph{k, 2}(2));
        if isempty(w); continue; end
        e = w(end);
        fprintf('  %-8s | %7.4f | %8.1f | %9.4f | %8.4f | %8.2f\n', ph{k, 1}, ...
                sP(e), 1 / P(e), b(e), abs(b(e) - B_TRUE), abs(b(e) - B_TRUE) / sP(e));
    end

    % --- claim 3: after the window the estimate cannot recover ---
    w_after = t >= t2;
    b_frozen = b(find(w_after, 1));
    fprintf('  b_hat at end of descent %.4f -> at end of run %.4f (moved %.4f)\n', ...
            b_frozen, b(end), abs(b(end) - b_frozen));
    fprintf('  remaining budget after descent: sqrt(P) %.4f (was %.4f at t=0)\n', ...
            sP(find(w_after, 1)), sP(1));
    fprintf('  distance still to truth at freeze: %.4f  = %.1f remaining sigma\n', ...
            abs(b_frozen - B_TRUE), abs(b_frozen - B_TRUE) / sP(find(w_after, 1)));
end

fprintf(['\nReading: if the P-drop share is concentrated in one phase and |db| after that\n' ...
         'phase is small compared with the distance still to truth, the run has no\n' ...
         'within-run recovery path -- that is a property of Q = 0, not of the noise draw.\n']);


function s = local_verdict(ok)
    if ok; s = 'CONFIRMED'; else; s = 'FALSIFIED'; end
end
