function out = verify_state_observability(L, cfg)
%VERIFY_STATE_OBSERVABILITY  Gates 4 and 5 of the observability workflow.
%   STATUS: ACTIVE | SSOT for the numerical half of
%           `.claude/rules/observability-workflow.md` (gates 4 and 5).
%           Gates 1-3 are pencil work and belong in the derivation .tex.
%
%   out = verify_state_observability(L, cfg)
%
%   L   = the record array from obs_dump('get') -- see model/diag/obs_dump.m.
%         Produce it by running any instrumented driver with
%         ctrl_const_override.obs_dump = true.
%   cfg = struct:
%       .axis        (3)     which axis's records to use
%       .free        req'd   state slots that are FREE in this arm. Locked /
%                            inert slots must NOT be listed: a pinned slot is
%                            unobservable by construction and would be
%                            indistinguishable from a real finding.
%       .neg_ctrl    req'd   the instrument check. Either
%                              - a slot index that is pinned/inert in this arm, or
%                              - 'synthetic': append one extra state that is
%                                decoupled by construction (F gets a unit
%                                diagonal entry, every H row gets a zero). Use
%                                this when the arm has no spare pinned slot --
%                                it is always available and always valid.
%                            There is no default and no way to skip it.
%       .prior_std   req'd   struct/array: sqrt(P[0]) per slot in .free, the
%                            width gate 5 compares the CRLB against. NaN for a
%                            slot skips its verdict but still reports the CRLB.
%       .labels      {}      display names for .free slots (optional)
%       .window      500     window length in steps
%       .Ts          []      [] = infer from .t_end; used only for the report
%       .t_end       4.8     run duration [s], for the time axis
%       .segments    []      optional k x 3 cell {name, t_start, t_end} to
%                            report per phase (hold / descend / oscillation)
%       .channels    {}      {} = auto ({'y1','y2'} for a 2-row H)
%
%   WHY THE NEGATIVE CONTROL IS MANDATORY. A rank test has three silent failure
%   modes -- a mis-set tolerance, a transition product that has under/overflowed,
%   and simply listing the wrong slots -- and every one of them returns a
%   plausible integer instead of an error. Feeding in a direction that MUST come
%   back unobservable is the cheapest way to make those failures visible. This
%   is rule 7 (verify the instrument before it counts as evidence) and rule 13
%   (write down what would give the wiring away) applied to this measurement.
%
%   WHY EIG AND NOT PINV. The CRLB of an unobservable state is +Inf. pinv() maps
%   the null space to zero, so an unobservable slot comes back with the SMALLEST
%   variance in the filter -- i.e. as the best-determined state. The first pass
%   of this analysis (2026-08-17, formC delta a) reported the pinned negative
%   control at 4.4e-19 for exactly this reason. Do not reintroduce pinv here.
%
%   WHAT THE TWO GATES MEAN
%       Gate 4 (rank)  rank(O) over each window, where O stacks H_k*Phi(k,k0),
%                      taken on the FREE block; plus sigma_min/sigma_max, which
%                      says how close to singular the free block actually is.
%                      PASS = every window full rank AND the negative control
%                      comes back with CRLB = Inf.
%       Gate 5 (CRLB)  G = sum_k Phi' H' R^-1 H Phi is the Fisher information
%                      about x[k0]; sqrt([G^-1]_jj) is the best achievable
%                      std on slot j with all other free slots unknown.
%                      PASS  = CRLB < prior_std in at least one window
%                      TIGHT = within 2x of the prior everywhere
%                      FAIL  = CRLB > prior_std in EVERY window. Formally
%                              observable, practically unidentifiable -- the
%                              filter's estimate is its prior. Correct response
%                              is to demote the slot or absorb it algebraically,
%                              NOT to open Q and let it wander.
%
%   Channel decomposition (which measurement carries the information) and the
%   per-segment report (when in the scenario it arrives) are both produced
%   unconditionally, because "observable" without "from where" and "when" has
%   repeatedly turned out to be the wrong summary in this project.
%
%   !! THE CRLB IS ONLY MEANINGFUL FOR Q = 0 SLOTS. G accumulates measurement
%   information about x[k0] and carries NO process noise, so it is the bound for
%   a state that is genuinely constant over the window -- the shape parameters
%   (b, p, w_s), the disturbance da. For a slot driven by Q > 0 (the dw chain,
%   the MA memory) x[k0] does not determine x[k], so the printed number is NOT
%   the filter's achievable error and must not be read as one; it is reported
%   only because it costs nothing and because a sudden change in it flags a
%   linearization problem. Pass NaN as their prior so they get no verdict.
%
%   !! AND IT IS A BOUND ON THE LINEARIZATION, NOT A PREDICTION OF THE FILTER.
%   G is built from (F_e, H, R) alone. It cannot see clamps, gates, lock flags
%   or nonlinearity, so gate 5 PASSing does NOT mean the filter is actually
%   using that information. 2026-08-17: b's CRLB/prior was 0.82 (usable) while
%   the estimate was in fact pinned by a mis-copied clamp. To close that gap,
%   compare sqrt(P[end]) from the run against the run-long CRLB -- if the filter
%   claims far more uncertainty than the bound, something is holding it.
%
%   See also: obs_dump, plot_state_observability

    % ------------------------------------------------------------------
    % [0] Config, with the two mandatory fields enforced up front
    % ------------------------------------------------------------------
    assert(nargin >= 2 && isstruct(cfg), 'verify_state_observability:cfg', ...
           'cfg is required.');
    assert(isfield(cfg, 'free') && ~isempty(cfg.free), ...
           'verify_state_observability:noFree', ...
           'cfg.free is required: list the slots that are FREE in this arm.');
    assert(isfield(cfg, 'neg_ctrl') && ~isempty(cfg.neg_ctrl), ...
           'verify_state_observability:noNegCtrl', ...
           ['cfg.neg_ctrl is required: either a slot that is KNOWN ', ...
            'unobservable (pinned/inert) or the string ''synthetic''. A rank ', ...
            'test without a negative control is not evidence -- see the header.']);
    assert(isfield(cfg, 'prior_std'), 'verify_state_observability:noPrior', ...
           ['cfg.prior_std is required: gate 5 compares the CRLB against ', ...
            'sqrt(P[0]). Pass NaN for a slot to skip its verdict.']);
    free = cfg.free(:).';
    neg = cfg.neg_ctrl;
    synth_neg = ischar(neg) || isstring(neg);
    if synth_neg
        assert(strcmpi(neg, 'synthetic'), 'verify_state_observability:badNeg', ...
               'the only string cfg.neg_ctrl accepted is ''synthetic''.');
    else
        assert(isscalar(neg) && ~ismember(neg, free), ...
               'verify_state_observability:negInFree', ...
               'cfg.neg_ctrl (%g) must be a scalar slot not listed in cfg.free.', neg);
    end
    idx = [];    % filled after the synthetic slot (if any) is appended
    n_free = numel(free);
    i_neg = n_free + 1;   % the negative control is always the last column of idx

    axis_sel = local_default(cfg, 'axis', 3);
    W_req    = local_default(cfg, 'window', 500);
    t_end    = local_default(cfg, 't_end', 4.8);
    segments = local_default(cfg, 'segments', []);
    labels   = local_default(cfg, 'labels', {});
    prior    = cfg.prior_std(:).';
    assert(numel(prior) == n_free, 'verify_state_observability:priorLen', ...
           'cfg.prior_std must have one entry per cfg.free (%d).', n_free);
    if isempty(labels)
        labels = arrayfun(@(i) sprintf('slot%d', i), free, 'uni', 0);
    end
    labels_all = {};   % completed once neg is resolved (synthetic case)

    % ------------------------------------------------------------------
    % [1] Select the axis and sanity-check the dump BEFORE using it
    % ------------------------------------------------------------------
    assert(~isempty(L), 'verify_state_observability:emptyDump', ...
           ['the dump is empty. Did you set ctrl_const_override.obs_dump = ', ...
            'true, and is obs_dump.m on the path?']);
    L = L([L.ax] == axis_sel);
    N = numel(L);
    assert(N > W_req, 'verify_state_observability:tooShort', ...
           'axis %d has %d records, need more than the window (%d).', ...
           axis_sel, N, W_req);
    n = size(L(1).F, 1);
    if synth_neg
        % One extra state, decoupled by construction: it evolves as x[k+1]=x[k]
        % and appears in no measurement, so it MUST come back unobservable.
        for k = 1:N
            L(k).F = [L(k).F, zeros(n, 1); zeros(1, n), 1];
            for j = 1:numel(L(k).H)
                if ~isempty(L(k).H{j}); L(k).H{j} = [L(k).H{j}, 0]; end
            end
            L(k).P_pred = blkdiag(L(k).P_pred, 1);
            L(k).P_upd  = blkdiag(L(k).P_upd, 1);
        end
        n = n + 1;
        neg = n;
    end
    idx = [free, neg];
    assert(max(idx) <= n, 'verify_state_observability:slotRange', ...
           'slot %d exceeds the state dimension (%d).', max(idx), n);
    if synth_neg
        labels_all = [labels(:).', {'synthetic (NEG CTRL)'}];
    else
        labels_all = [labels(:).', {sprintf('slot%d (NEG CTRL)', neg)}];
    end
    n_chan = numel(L(1).H);
    channels = local_default(cfg, 'channels', {});
    if isempty(channels)
        channels = arrayfun(@(i) sprintf('y%d', i), 1:n_chan, 'uni', 0);
    end
    Ts = t_end / N;
    n_gated = sum(arrayfun(@(s) any(cellfun(@isempty, s.H)), L));

    % How many runs armed the buffer. The house drivers clear the controller
    % once per seed, so a multi-seed run leaves only the last seed's records --
    % correct, but silent. Say it out loud.
    n_reset = 0;
    if exist('obs_dump', 'file') == 2
        n_reset = obs_dump('resets');
    end

    % Instrument check A: the negative control must have an all-zero column in
    % F (off-diagonal) and in every H. If it does not, it is not a valid
    % negative control and everything downstream is uninterpretable.
    neg_col_F = 0; neg_col_H = 0;
    for k = 1:N
        c = L(k).F(:, neg); c(neg) = 0;
        neg_col_F = max(neg_col_F, max(abs(c)));
        for j = 1:n_chan
            if ~isempty(L(k).H{j})
                neg_col_H = max(neg_col_H, abs(L(k).H{j}(neg)));
            end
        end
    end

    fprintf('\n================ verify_state_observability ================\n');
    fprintf('axis %d | %d steps | n_state %d | Ts %.4g s | %d steps with a gated channel\n', ...
            axis_sel, N, n, Ts, n_gated);
    if n_reset > 1
        fprintf(['buffer was armed %d times -> these records are the LAST run ', ...
                 'ONLY (drivers clear the controller per seed). Run one seed.\n'], n_reset);
    end
    fprintf('free slots  : %s\n', strjoin(labels, ', '));
    if synth_neg
        fprintf('neg control : SYNTHETIC appended slot %d   (decoupled by construction)\n', neg);
    else
        fprintf('neg control : slot %d   (max |F col| off-diag %.2e, max |H col| %.2e)\n', ...
                neg, neg_col_F, neg_col_H);
    end
    if neg_col_F > 0 || neg_col_H > 0
        fprintf('  !! slot %d is COUPLED -- it is not a valid negative control.\n', neg);
    end

    % ------------------------------------------------------------------
    % [2] Gates 4 and 5, per window, per channel subset
    % ------------------------------------------------------------------
    % channel subsets: all, then each channel alone
    subs = cell(1, n_chan + 1);
    subs{1} = 1:n_chan;
    sub_names = cell(1, n_chan + 1);
    sub_names{1} = strjoin(channels, '+');
    for j = 1:n_chan
        subs{j + 1} = j;
        sub_names{j + 1} = [channels{j} ' only'];
    end
    n_sub = numel(subs);

    starts = 1:W_req:(N - W_req + 1);
    nw = numel(starts);
    rk   = zeros(nw, n_sub);
    srat = zeros(nw, n_sub);
    crlb = nan(nw, numel(idx), n_sub);
    near = false(nw, numel(idx), n_sub);
    for w = 1:nw
        k0 = starts(w);
        for c = 1:n_sub
            [rk(w, c), srat(w, c), crlb(w, :, c), near(w, :, c)] = ...
                local_window(L, k0, W_req, n, idx, subs{c});
        end
    end
    t_win = (starts + W_req / 2 - 1) * Ts;

    % ------------------------------------------------------------------
    % [3] Verdicts
    % ------------------------------------------------------------------
    % Gate 4: rank must equal the number of FREE slots (neg control missing)
    rank_full = rk(:, 1);
    g4_pass = all(rank_full == n_free);
    neg_crlb = crlb(:, i_neg, 1);
    neg_ok = all(isinf(neg_crlb));

    fprintf('\n--- Gate 4 (rank), window %d steps (%.3f s), %d windows\n', ...
            W_req, W_req * Ts, nw);
    fprintf('  rank(O) over the FREE block = %d ... %d   (full = %d)\n', ...
            min(rank_full), max(rank_full), n_free);
    fprintf('  sigma_min/sigma_max of O : med %.3e (rank tol 1e-10)\n', median(srat(:, 1)));
    fprintf('  negative control CRLB = Inf in %d / %d windows  -> %s\n', ...
            sum(isinf(neg_crlb)), nw, local_tf(neg_ok, 'INSTRUMENT OK', ...
            'INSTRUMENT BROKEN -- results below are NOT evidence'));
    fprintf('  verdict: %s\n', local_tf(g4_pass && neg_ok, 'PASS', 'FAIL'));

    fprintf('\n--- Gate 5 (CRLB vs prior), channel set "%s"\n', sub_names{1});
    verdict = cell(1, n_free);
    for j = 1:n_free
        cj = crlb(:, j, 1);
        pj = prior(j);
        if isnan(pj)
            verdict{j} = 'n/a';
        elseif any(cj < pj)
            if all(cj < pj)
                verdict{j} = 'PASS';
            else
                verdict{j} = 'PASS (motion-only)';
            end
        elseif any(cj < 2 * pj)
            verdict{j} = 'TIGHT';
        else
            verdict{j} = 'FAIL (unidentifiable)';
        end
        n_near = sum(near(:, j, 1));
        tag = '';
        if n_near > 0
            tag = sprintf('  [%d/%d windows beyond numerical resolution]', n_near, nw);
        end
        fprintf(['  %-22s CRLB med %.4g  min %.4g  max %.4g | prior %.4g ', ...
                 '| best/prior %.3g -> %s%s\n'], labels{j}, ...
                median(cj), min(cj), max(cj), pj, min(cj) / pj, verdict{j}, tag);
    end

    fprintf('\n--- Channel decomposition (median CRLB over windows)\n');
    fprintf('  %-22s', 'slot');
    fprintf('%16s', sub_names{:}); fprintf('\n');
    for j = 1:n_free
        fprintf('  %-22s', labels{j});
        for c = 1:n_sub
            fprintf('%16.4g', median(crlb(:, j, c)));
        end
        fprintf('\n');
    end

    if ~isempty(segments)
        fprintf('\n--- Per-segment CRLB (channel set "%s")\n', sub_names{1});
        for si = 1:size(segments, 1)
            nm = segments{si, 1}; ta = segments{si, 2}; tb = segments{si, 3};
            sel = t_win >= ta & t_win <= tb;
            if ~any(sel)
                fprintf('  %-14s [%.2f %.2f] s : no window centre falls inside\n', ...
                        nm, ta, tb);
                continue;
            end
            fprintf('  %-14s [%.2f %.2f] s (%d win):', nm, ta, tb, sum(sel));
            for j = 1:n_free
                fprintf('  %s %.4g', labels{j}, median(crlb(sel, j, 1)));
            end
            fprintf('\n');
        end
    end
    fprintf('===========================================================\n');

    out = struct('N', N, 'n', n, 'Ts', Ts, 'axis', axis_sel, ...
                 'idx', idx, 'free', free, 'neg_ctrl', neg, ...
                 'labels', {labels_all}, 'prior', prior, ...
                 'window', W_req, 'starts', starts, 't_win', t_win, ...
                 'rank', rk, 'srat', srat, 'crlb', crlb, 'near', near, ...
                 'sub_names', {sub_names}, 'channels', {channels}, ...
                 'gate4_pass', g4_pass, 'neg_ok', neg_ok, ...
                 'verdict', {verdict}, 'n_gated', n_gated, ...
                 'neg_col_F', neg_col_F, 'neg_col_H', neg_col_H);
end

% ======================================================================
function [rk, srat, crlb, near] = local_window(L, k0, W, n, idx, chan)
%LOCAL_WINDOW  Stack O and accumulate G over one window.
    Phi = eye(n);
    O = zeros(0, n);
    G = zeros(n);
    for k = k0:(k0 + W - 1)
        for j = chan
            Hj = L(k).H{j};
            if isempty(Hj); continue; end
            O = [O; Hj * Phi];                       %#ok<AGROW>
            M = (Hj / sqrt(L(k).R(j))) * Phi;
            G = G + M' * M;
        end
        Phi = L(k).F * Phi;
    end
    % Gate 4's rank is taken on the FREE block only. Including the negative
    % control would make rank(O) = numel(free) trivially (its column is zero)
    % and would put a hard zero into sigma_min, hiding how close the FREE
    % directions are to singular -- which is the thing gate 4 exists to see.
    % The negative control is checked by the exact CRLB = Inf test instead.
    free_cols = idx(1:end-1);
    Or = O(:, free_cols);
    Gr = 0.5 * (G(idx, idx) + G(idx, idx)');
    s = svd(Or);
    if isempty(s)
        rk = 0; srat = 0; crlb = inf(1, numel(idx));
        near = true(1, numel(idx)); return;
    end
    rk = sum(s > max(s) * 1e-10);
    srat = s(end) / s(1);
    [crlb, near] = local_crlb(Gr);
end

function [c, near] = local_crlb(G)
%LOCAL_CRLB  sqrt(diag(inv(G))) with unobservable directions returned as Inf.
%   Deliberately NOT pinv: see the header. A slot with any weight on a
%   null-space eigenvector has unbounded variance, and that is the answer.
%
%   TOLERANCE. The cut is at eps-level (max eigenvalue x 1e-15), NOT at the
%   1e-12 a first version used, because G = O'R^-1 O squares O's singular
%   values: a direction that gate 4's rank test keeps at sigma/sigma_max = 1e-6
%   lands at eigenvalue ratio 1e-12 and would be reported as EXACTLY
%   unobservable while the rank test calls it full. The two gates must not
%   disagree on the same data. With the eps-level cut, a merely starved
%   direction returns a huge FINITE number (honest: "the bound is enormous")
%   and only a structurally decoupled one -- the negative control, whose row
%   and column are identically zero -- returns Inf.
%
%   `near` flags slots whose value rests on an eigenvalue within 1e3 of the
%   tolerance, i.e. where the number is at the edge of double precision and
%   should be read as "beyond resolution", not as a measurement.
    [V, D] = eig(G);
    dv = real(diag(D));
    if isempty(dv) || max(dv) <= 0
        c = inf(1, size(G, 1)); near = true(1, size(G, 1)); return;
    end
    tol = max(dv) * 1e-15;
    ok = dv > tol;
    m = size(G, 1);
    c = zeros(1, m);
    near = false(1, m);
    for j = 1:m
        wj = real(V(j, :)).^2;
        if any(wj(~ok) > 1e-12)
            c(j) = Inf;
            near(j) = true;
        else
            c(j) = sqrt(sum(wj(ok) ./ dv(ok).'));
            % does the value rest on an eigenvalue barely above the cut?
            contrib = wj(ok) ./ dv(ok).';
            [~, imax] = max(contrib);
            dv_ok = dv(ok);
            near(j) = dv_ok(imax) < tol * 1e3;
        end
    end
end

function v = local_default(s, f, d)
    if isfield(s, f) && ~isempty(s.(f)); v = s.(f); else; v = d; end
end

function s = local_tf(tf, a, b)
    if tf; s = a; else; s = b; end
end
