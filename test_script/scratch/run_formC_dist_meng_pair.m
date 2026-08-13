% STATUS: ACTIVE (scratch) | PURPOSE: does the ADDITIVE disturbance state
%   (formC_state_dist.tex derivation (b)) beat the parameter-free baseline
%   (derivation (a)) on a MONOTONE descent, where the S3(b) closed form is
%   one-signed, as opposed to the canonical two-oscillation-cycle scenario
%   where it integrates to exactly zero over each closed cycle?
%   Scenario from Fei Long's dissertation 4.3.2 (Meng-group experiment):
%   z ramps 12.5 um down over 10 s, ending 2.5 um from the wall (h_bar 1.111),
%   no oscillation on the wall-normal axis.
%   EXPIRES: baseline / disturbance adjudication.
%   Production files untouched; the scenario is built with config_override.
function out = run_formC_dist_meng_pair(scenario, seeds)
%RUN_FORMC_DIST_MENG_PAIR  Both formC_dist arms on one scenario, plus the
%   run-health diagnostics the canonical driver does not print (y2 gate duty,
%   a_bar clamp saturation, plant h_bar clamp saturation).
%
%   out = run_formC_dist_meng_pair('meng')       % the monotone-ramp scenario
%   out = run_formC_dist_meng_pair('canonical')  % the house scenario, same session

    if nargin < 1 || isempty(scenario); scenario = 'meng'; end
    if nargin < 2 || isempty(seeds);    seeds = [7 11 23 42 101 777 27 31]; end

    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));
    addpath(fullfile(root, 'test_script', 'integration'));

    AX_Z = 3;
    ov = local_scenario_override(scenario);

    fprintf('\n################ SCENARIO: %s ################\n', upper(scenario));
    oB = run_formC_dist(struct('arm', 'base', 'seeds', seeds, 'config_override', ov));
    oD = run_formC_dist(struct('arm', 'dist', 'seeds', seeds, 'config_override', ov));

    % ---- run-health diagnostics (both arms) ----------------------------
    fprintf('\n--- run health, %s scenario (z axis) ---\n', scenario);
    fprintf('%6s | %-5s %8s %8s %9s %9s %8s %8s\n', 'seed', 'arm', ...
            'y2 gate', 'aBarClmp', 'min aBar', 'max aBar', 'hbClamp', 'minHbar');
    H = struct();
    for c = 1:2
        o = {oB, oD}; o = o{c}; nm = {'base', 'dist'}; nm = nm{c};
        for q = 1:numel(seeds)
            r  = o.runs{q};
            fl = local_field(r.ctrl_const, 'a_bar_floor', 0.05);
            cl = local_field(r.ctrl_const, 'a_bar_ceil',  1 - 1e-4);
            ab = r.a_bar_hat_out(:, AX_Z);
            hb_raw = r.p_true_out(:, AX_Z) / r.R;            % unclamped plant h_bar
            hb_flr = r.meta.params_value.wall.h_bar_min;
            H.(nm)(q, :) = [mean(r.gate_out(:, AX_Z)), ...
                            mean(ab <= fl + 1e-12 | ab >= cl - 1e-12), ...
                            min(ab), max(ab), ...
                            mean(hb_raw <= hb_flr + 1e-12), min(hb_raw)];
            fprintf('%6d | %-5s %8.4f %8.4f %9.4f %9.4f %8.4f %8.4f\n', ...
                    seeds(q), nm, H.(nm)(q, 1), H.(nm)(q, 2), H.(nm)(q, 3), ...
                    H.(nm)(q, 4), H.(nm)(q, 5), H.(nm)(q, 6));
        end
    end

    out = struct('oB', oB, 'oD', oD, 'seeds', seeds, 'scenario', scenario, ...
                 'cfg_override', ov, 'health', H, ...
                 'health_cols', {{'gate_frac', 'a_bar_clamp_frac', 'min_a_bar', ...
                                  'max_a_bar', 'hbar_clamp_frac', 'min_hbar_raw'}});
end


function ov = local_scenario_override(scenario)
%LOCAL_SCENARIO_OVERRIDE  config_override for the scenario under test.
%   'canonical' = {} (the driver's own scenario, untouched).
%   'meng'      = Fei Long 4.3.2: z ramps from h = 15 um (h_bar 6.667) down to
%                 h = 2.5 um (h_bar 1.111) over 10 s, NO oscillation on z.
%                 Phase structure is kept (hold -> descent -> phase3 -> hold)
%                 so every metric window keeps the canonical definition;
%                 amplitude = 0 collapses phase 3 to a flat 1 s at the trough.
    pc = physical_constants();
    switch lower(scenario)
        case 'canonical'
            ov = struct();
        case 'meng'
            ov = struct();
            ov.trajectory_type = 'osc';
            ov.h_init    = 15.0;    % [um] h_bar 6.6667 (probe centre, R = 2.25)
            ov.h_bottom  = 2.5;     % [um] h_bar 1.1111 (the paper's closest approach)
            ov.amplitude = 0;       % [um] NO wall-normal oscillation
            ov.frequency = 1;       % [Hz] (only sets the phase-3 length now)
            ov.n_cycles  = 1;       % flat 1 s at the trough (amplitude = 0)
            ov.t_hold    = 0.5;     % [s] initial hold, as canonical
            ov.t_descend_override = 10.0;   % [s] the 12.5 um ramp
            ov.T_sim     = 12.5;    % [s] leaves a 1.0 s final hold
            ov.h_min     = 1.1 * pc.R;      % [um] truth-curve validity floor
        case 'meng_gateopen'
            % Supplementary arm: identical to 'meng' but the near-wall y2 gate
            % is opened down to the truth-curve floor. On 'meng' the gain
            % readout is gated off for 29.6 % of the run (h_bar < 1.5), which
            % is exactly where the S3(b) disturbance is largest (sup at
            % w_bar 1.623), so a null result there could be blamed on the gate
            % rather than on the construction. This removes that confound.
            ov = local_scenario_override('meng');
            ov.h_bar_safe = 1.1;
        otherwise
            error('run_formC_dist_meng_pair:scenario', 'unknown scenario %s', scenario);
    end
end


function v = local_field(s, name, dflt)
    if isstruct(s) && isfield(s, name) && ~isempty(s.(name)); v = s.(name); else; v = dflt; end
end
