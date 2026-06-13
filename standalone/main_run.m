function out = main_run(scenario, seed)
%MAIN_RUN  Single entry point for the 6-state standalone package.
%
%   main_run                    % h50, seed 42, makes figures
%   main_run(scenario)          % 'h50' | 'h10' | 'ramp2p7'
%   main_run(scenario, seed)
%   out = main_run(...)         % also returns the run struct
%
%   Adds the package folders to the path, runs one closed-loop simulation,
%   prints a one-line tracking summary, and saves the two verification
%   figures to <standalone>/test_results. This is the "press run and see
%   it work" file; for the quantitative 3-scenario PASS gate use
%   verify_standalone, and for the per-part fidelity gates run
%   test/gates_part1..7.
%
%   See also: verify_standalone, run_simulation, config, controller_6state

    if nargin < 1 || isempty(scenario); scenario = 'h50'; end
    if nargin < 2 || isempty(seed); seed = 42; end

    here = fileparts(mfilename('fullpath'));
    addpath(here, fullfile(here, 'physics'), fullfile(here, 'sim'), fullfile(here, 'test'));

    fprintf('[main_run] scenario=%s seed=%d ...\n', scenario, seed);
    out = run_simulation(scenario, struct('seed', seed));

    % physics ground-truth gain from the noise-free probe
    p = out.meta.params;
    a_nom = p.common.Ts / p.common.gamma_N;
    N = numel(out.tout);
    h_true = (out.p_true_out * p.wall.w_hat - p.wall.pz) / p.common.R;
    a_true = zeros(N, 3);
    for k = 1:N
        [cpa, cpe] = wall_corrections(max(h_true(k), 1.001));
        a_true(k, :) = [a_nom/cpa, a_nom/cpa, a_nom/cpe];
    end

    n_warm = round(p.traj.t_hold / p.common.Ts); idx = (n_warm+1):N;
    trk = std(out.p_d_out(idx, :) - out.p_m_out(idx, :), 0, 1) * 1e3;
    fprintf('[main_run] tracking std [x y z] = [%.1f %.1f %.1f] nm\n', trk);

    fig_dir = fullfile(here, 'test_results');
    make_figures(out, a_true, scenario, fig_dir);
    fprintf('[main_run] figures saved to %s\n', fig_dir);
end
