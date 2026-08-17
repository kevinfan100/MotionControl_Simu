function varargout = obs_dump(action, payload)
%OBS_DUMP  Opt-in per-step capture of the EKF linearization, for observability work.
%   STATUS: ACTIVE | SSOT for the (F_e, H, R, P) capture consumed by
%           test_script/integration/verify_state_observability.m
%
%   WHY THIS EXISTS. Observability of a filter state is a property of the pair
%   (F_e[k], H[k]) together with R[k] -- none of which any controller returns.
%   Before this file the only way to get them was to fork the controller into
%   scratch and add print statements, which meant (a) a fork per controller,
%   (b) forks that silently drift from the production code they are supposed to
%   describe. The capture now lives in production behind a flag that is OFF by
%   default, so the answer always comes from the code that actually runs.
%
%   CONTRACT WHEN OFF (the only thing that matters for production):
%   'append' returns immediately, and the call sites are guarded by a single
%   persistent logical anyway, so a run with obs_dump = false must be
%   BIT-IDENTICAL to the same run before this file existed. That is a testable
%   claim, not a promise -- see verify_state_observability.m's provenance note.
%
%   USAGE (controller side, 3 lines; see motion_control_law_formC_dist.m)
%       obs_dump('reset', enabled);              % once, in the init branch
%       if obs_dump_on                           % once per axis per step
%           obs_dump('append', struct( ... ));
%       end
%
%   USAGE (analysis side)
%       ctrl_const_override.obs_dump = true;
%       ... run the driver ...
%       L = obs_dump('get');                     % struct array, one per append
%
%   ACTIONS
%       obs_dump('reset', enabled)   arm (or disarm) the buffer and clear it.
%                                    MUST be called from the controller's init
%                                    branch: a stale buffer from a previous run
%                                    is the classic way to audit the wrong data.
%       obs_dump('append', s)        append one record. No-op when disarmed.
%       L = obs_dump('get')          the records so far (1 x N struct array).
%       tf = obs_dump('enabled')     is the buffer armed?
%       n  = obs_dump('count')       number of records.
%       n  = obs_dump('resets')      how many times it was armed-and-cleared.
%                                    > 1 means the buffer holds ONLY the LAST
%                                    run -- see MULTI-SEED below.
%
%   MULTI-SEED RUNS HOLD ONLY THE LAST SEED. The house drivers call
%   `clear motion_control_law_*` inside local_run_once, i.e. ONCE PER SEED, so
%   the controller's init branch -- and therefore this buffer's reset -- fires
%   per seed. A 6-seed run leaves the records of seed 6 only. That is the safe
%   behaviour (mixing two seeds' linearizations into one Gramian would be
%   meaningless) but it is silent, so 'resets' counts the arming events and
%   verify_state_observability.m reports it. Run one seed at a time.
%
%   RECORD CAP. Long runs at 1.6 kHz x 3 axes produce ~1.3 kB per record, so
%   the buffer is capped and WARNS ONCE on hitting the cap rather than
%   truncating quietly. Raise it with obs_dump('cap', n) after the reset.
%
%   RECORD FIELDS (what 'append' expects in s; the verify script requires all)
%       .ax       axis index 1..3
%       .k        controller step counter
%       .F        n x n error-dynamics Jacobian F_e[k]  (post-augmentation)
%       .H        1 x m cell of 1 x n measurement rows, in update order.
%                 An EMPTY cell entry means that channel did not update this
%                 step (gated off / disabled) -- keep the slot so the channel
%                 bookkeeping stays aligned across steps.
%       .R        1 x m variances matching .H (NaN allowed where .H is empty)
%       .x_pred   n x 1 predicted state
%       .x_upd    n x 1 updated state (post all measurement updates and clamps)
%       .P_pred   n x n predicted covariance
%       .P_upd    n x n updated covariance
%       .gate     logical, was the y2 gate closed this step
%
%   See also: verify_state_observability, plot_state_observability

    persistent armed buf n_rec n_reset cap warned

    if nargin < 1
        error('obs_dump:noAction', 'obs_dump requires an action.');
    end

    switch lower(action)
        case 'reset'
            if nargin < 2 || isempty(payload)
                armed = false;
            else
                armed = logical(payload);
            end
            buf = local_empty_record();
            n_rec = 0;
            if isempty(n_reset); n_reset = 0; end
            if armed; n_reset = n_reset + 1; end
            if isempty(cap); cap = 200000; end
            warned = false;

        case 'append'
            if isempty(armed) || ~armed
                return;
            end
            if n_rec >= cap
                if ~warned
                    warning('obs_dump:cap', ...
                            ['record cap %d reached -- no further records ', ...
                             'are being stored. Raise it with ', ...
                             'obs_dump(''cap'', n) or run a shorter scenario.'], cap);
                    warned = true;
                end
                return;
            end
            local_check_record(payload);
            if n_rec >= numel(buf)
                % geometric growth; struct arrays cannot be preallocated
                % cheaply without knowing n_state, and the buffer is a
                % diagnostic so the copy cost is irrelevant
                grow = max(1024, numel(buf));
                buf(numel(buf) + grow) = payload;
            end
            n_rec = n_rec + 1;
            buf(n_rec) = payload;

        case 'get'
            if isempty(n_rec) || n_rec == 0
                varargout{1} = local_empty_record();
            else
                varargout{1} = buf(1:n_rec);
            end

        case 'enabled'
            varargout{1} = ~isempty(armed) && armed;

        case 'count'
            if isempty(n_rec); varargout{1} = 0; else; varargout{1} = n_rec; end

        case 'resets'
            if isempty(n_reset); varargout{1} = 0; else; varargout{1} = n_reset; end

        case 'cap'
            assert(nargin >= 2 && isscalar(payload) && payload > 0, ...
                   'obs_dump:badCap', 'cap requires a positive scalar.');
            cap = double(payload);

        otherwise
            error('obs_dump:badAction', 'Unknown action "%s".', action);
    end
end

function s = local_empty_record()
    s = struct('ax', {}, 'k', {}, 'F', {}, 'H', {}, 'R', {}, ...
               'x_pred', {}, 'x_upd', {}, 'P_pred', {}, 'P_upd', {}, ...
               'gate', {});
end

function local_check_record(s)
%LOCAL_CHECK_RECORD  Fail loudly at the call site, not three hours later in the
%   analysis. A record that is missing a field or whose H rows do not match F
%   cannot produce a meaningful observability matrix, and the failure mode of
%   NOT checking is a plausible-looking rank.
    req = {'ax', 'k', 'F', 'H', 'R', 'x_pred', 'x_upd', 'P_pred', 'P_upd', 'gate'};
    for i = 1:numel(req)
        assert(isfield(s, req{i}), 'obs_dump:missingField', ...
               'record is missing field "%s".', req{i});
    end
    n = size(s.F, 1);
    assert(size(s.F, 2) == n, 'obs_dump:badF', 'F must be square.');
    assert(iscell(s.H), 'obs_dump:badH', 'H must be a cell of measurement rows.');
    assert(numel(s.R) == numel(s.H), 'obs_dump:badR', ...
           'R must have one entry per H channel (got %d vs %d).', ...
           numel(s.R), numel(s.H));
    for i = 1:numel(s.H)
        if isempty(s.H{i}); continue; end
        assert(size(s.H{i}, 1) == 1 && size(s.H{i}, 2) == n, ...
               'obs_dump:badHrow', 'H{%d} must be 1 x %d.', i, n);
    end
    assert(isequal(size(s.P_pred), [n n]) && isequal(size(s.P_upd), [n n]), ...
           'obs_dump:badP', 'P_pred and P_upd must be %d x %d.', n, n);
    assert(numel(s.x_pred) == n && numel(s.x_upd) == n, 'obs_dump:badX', ...
           'x_pred and x_upd must have %d entries.', n);
end
