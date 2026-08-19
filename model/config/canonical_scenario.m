function cfg = canonical_scenario(a_cov, h_bar_min_prior, name)
%CANONICAL_SCENARIO  The one trajectory every gain-law driver runs on.
%
%   cfg = canonical_scenario(a_cov, h_bar_min_prior)          % 'deep'
%   cfg = canonical_scenario(a_cov, h_bar_min_prior, 'shallow')
%
%   Shape is always hold -> descend -> oscillate -> hold. Only the DEPTH of the
%   band changes between the two names, and the band is TRANSLATED, never
%   stretched: both span 2.22 in w_bar, so depth is the single variable.
%
%       name        trough      band            drag at trough   status
%       'deep'      w_bar 1.10  [1.10, 3.32]    11.45x           DEFAULT 2026-08-19
%       'shallow'   w_bar 2.00  [2.00, 4.22]     2.12x           pre-2026-08-19
%
%   WHY DEEP IS THE DEFAULT. The shape information the gain-law line exists to
%   estimate lives near the wall: d(a_bar)/db vanishes in the far field, so a
%   band that stops at w_bar = 2 is asking the filter to identify a curve from
%   its flattest part. Measured on the b-as-state arm, the deep band widens the
%   excursion of b_hat from 0.037 to 0.213 -- 6x more travel for the same run --
%   and widens the truth's own variation, b_true, from 0.0094 to 0.0608.
%
%   WHY NOT DEEPER. Two limits meet at w_bar = 1.10 and neither is a house
%   value: the two-sphere series behind c_perp is only trusted above it, and it
%   is also where h_min sits, so the prior envelope may not be read below it.
%   Commanding the trough there already carries the particle to w_bar = 1.085 on
%   the tracking error, which is inside the series' extrapolated region but
%   still 190 nm clear of contact, where c_perp diverges.
%
%   WHAT MOVES WITH THE DEPTH, and must not be read across the two names:
%     - every error PERCENTAGE. a_bar at the trough falls 0.47 -> 0.087, so the
%       denominator shrinks 5.4x and the same absolute error reads 5.4x larger.
%       Compare the two bands only through seed-independent signatures (drift
%       rates, honesty ratios, observability ratios), never through a percent.
%     - every envelope-derived prior (b's width, the a_bar floor, P44[0]).
%       These are computed from env_lo/env_hi by each driver, so they follow the
%       config automatically -- but their VALUES change, and any number quoted
%       from a run predating 2026-08-19 was measured on 'shallow'.
%
%   h_bar_safe is 1.0 on the deep band, not the 1.5 house value: 1.5 sits above
%   the whole near-wall half of the oscillation and would gate y2 off exactly
%   where the experiment lives. The gate with a physical basis is G2.
%
%   See .claude/rules/canonical-trajectory.md.

    if nargin < 3 || isempty(name); name = 'deep'; end
    name = lower(name);
    assert(any(strcmp(name, {'deep','shallow'})), ...
           'canonical_scenario:name', 'name must be deep|shallow.');

    pc  = physical_constants();
    cfg = user_config();

    cfg.trajectory_type = 'osc';
    cfg.h_init    = 50;                  % [um] w_bar_0 = 22.2
    cfg.amplitude = 2.5;                 % [um] half-amplitude; band span 2.22
    cfg.frequency = 1;                   % [Hz]
    cfg.n_cycles  = 2;
    cfg.t_hold    = 0.5;                 % [s] initial hold
    cfg.t_descend_override = 1.0;        % [s] descent duration
    cfg.T_sim     = 4.8;                 % [s] leaves a 1.3 s final hold
    cfg.h_min     = h_bar_min_prior * pc.R;   % [um] prior-domain clamp

    switch name
        case 'deep'
            cfg.h_bottom   = 1.10 * pc.R;    % [um] trough w_bar = 1.10
            cfg.h_bar_safe = 1.00;           % see header: 1.5 would gate the band off
        case 'shallow'
            cfg.h_bottom   = 4.5;            % [um] trough w_bar = 2.00
            cfg.h_bar_safe = 1.50;           % house value, kept for reproducibility
    end

    cfg.ctrl_enable       = true;
    cfg.thermal_enable    = true;
    cfg.meas_noise_enable = true;
    cfg.lambda_c = 0.7;                  % closed-loop pole
    cfg.a_pd     = 0.05;                 % LP EWMA weight
    cfg.a_cov    = a_cov;                % variance EWMA weight (base 0.05)
    cfg.meas_noise_std = [0.00062; 0.00057; 0.00331];   % [um] per axis
end
