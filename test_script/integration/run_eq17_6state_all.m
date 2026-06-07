function results = run_eq17_6state_all(opts)
%RUN_EQ17_6STATE_ALL  One-shot regression over all 6-state scenarios.
%
%   results = run_eq17_6state_all()
%   results = run_eq17_6state_all(opts)
%
%   Runs verify_eq17_6state for {h50, h10, ramp} in sequence and prints a
%   combined report. Run this after ANY change to the 6-state controller to
%   check the three trajectories at once:
%       - h50  : positioning, PASS/FAIL on tracking + a_hat bias/rel-std
%       - h10  : positioning near wall, figures + descriptive only
%                (small a_z -> a_hat rel-std legitimately above the 5% gate)
%       - ramp : ramp_descent 50->5, figures + descriptive only
%
%   Each scenario writes test_results/eq17_6state_<scenario>/
%   {summary.md, summary.mat, fig1_gain_estimation.png, fig2_tracking_error.png}.
%
%   opts (optional) is forwarded to verify_eq17_6state (e.g. seeds, T_sim,
%   save_fig, verbose, thresholds) and applies to every scenario.
%
%   See also: verify_eq17_6state, make_eq17_6state_figures

    if nargin < 1; opts = struct(); end

    scenarios = {'h50', 'h10', 'ramp'};
    results = struct('scenario', {}, 'pass', {}, 'evaluate', {}, 'aggregate', {}, 'out_dir', {});

    for i = 1:numel(scenarios)
        r = verify_eq17_6state(scenarios{i}, opts);
        results(i) = struct('scenario', r.scenario, 'pass', r.pass, ...
                            'evaluate', r.evaluate, 'aggregate', r.aggregate, ...
                            'out_dir', r.out_dir);
    end

    % --- combined report ---
    fprintf('\n================ run_eq17_6state_all : COMBINED ================\n');
    all_eval_pass = true;
    for i = 1:numel(results)
        r = results(i);
        if r.evaluate
            tag = tern(r.pass, 'PASS', 'FAIL');
            all_eval_pass = all_eval_pass && r.pass;
            fprintf('  %-5s : %s   trk [%.1f %.1f %.1f] nm   bias [%+.1f %+.1f %+.1f] %%   relstd [%.1f %.1f %.1f] %%\n', ...
                    r.scenario, tag, r.aggregate.trk_meas_mean, ...
                    r.aggregate.a_hat_bias_pct, r.aggregate.a_hat_relstd_pct);
        else
            fprintf('  %-5s : FIGURES   trk [%.1f %.1f %.1f] nm   rel-err [%+.1f %+.1f %+.1f] %% (descriptive)\n', ...
                    r.scenario, r.aggregate.trk_meas_mean, r.aggregate.relerr_mean_pct);
        end
    end
    fprintf('---------------------------------------------------------------\n');
    fprintf('  OVERALL (evaluated): %s   (h10/ramp: figures only)\n', tern(all_eval_pass, 'PASS', 'FAIL'));
    fprintf('  Figures: test_results/eq17_6state_{h50,h10,ramp}/fig{1,2}_*.png\n');
end


function s = tern(cond, a, b)
    if cond; s = a; else; s = b; end
end
