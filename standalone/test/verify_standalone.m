function results = verify_standalone(scenario, opts)
%VERIFY_STANDALONE  Headline 3-scenario verification of the 6-state package.
%
%   results = verify_standalone()            % runs all three scenarios
%   results = verify_standalone(scenario)    % 'h50' | 'h10' | 'osc1hz' | 'all'
%   results = verify_standalone(scenario, opts)
%
%   Self-contained (standalone only, no mother repo). For each scenario it
%   runs the closed loop, computes the physics ground-truth gain a_true
%   from the noise-free position probe, and reports tracking std + a_hat
%   bias / rel-std vs a_true. h50 carries the quantitative PASS gate; h10
%   and osc1hz are descriptive (near-wall a_hat legitimately exceeds the
%   h50-calibrated thresholds -- the documented L2 limitation).
%
%   PASS gate (h50, aggregate over seeds):
%       tracking std  < trk_thresh_nm   (default 40 nm)
%       a_hat bias    < bias_thresh_pct (default 5 %)
%       a_hat rel-std < relstd_thresh_pct (default 5 %)
%
%   opts (all optional): seeds (default 1:5 for h50, 1:3 for h10/osc),
%       trk_thresh_nm, bias_thresh_pct, relstd_thresh_pct, make_figs (true),
%       fig_dir (default <standalone>/test_results).
%
%   See also: main_run, run_simulation, make_figures

    if nargin < 1 || isempty(scenario); scenario = 'all'; end
    if nargin < 2 || isempty(opts); opts = struct(); end
    dflt = struct('trk_thresh_nm', 40, 'bias_thresh_pct', 5, ...
                  'relstd_thresh_pct', 5, 'make_figs', true, 'fig_dir', '');
    fn = fieldnames(dflt);
    for i = 1:numel(fn)
        if ~isfield(opts, fn{i}) || isempty(opts.(fn{i})); opts.(fn{i}) = dflt.(fn{i}); end
    end

    here = fileparts(mfilename('fullpath'));         % standalone/test
    sa_root = fileparts(here);
    addpath(sa_root, fullfile(sa_root, 'physics'), fullfile(sa_root, 'sim'), here);
    if isempty(opts.fig_dir); opts.fig_dir = fullfile(sa_root, 'test_results'); end

    if strcmpi(scenario, 'all')
        scen_list = {'h50', 'h10', 'osc1hz'};
    else
        scen_list = {lower(scenario)};
    end

    results = struct('scenario', {}, 'trk_nm', {}, 'bias_pct', {}, ...
                     'relstd_pct', {}, 'pass', {});
    for s = 1:numel(scen_list)
        results(s) = run_one(scen_list{s}, opts);
    end
end


function r = run_one(scn, opts)
    is_h50 = strcmp(scn, 'h50');
    if isfield(opts, 'seeds') && ~isempty(opts.seeds)
        seeds = opts.seeds;
    elseif is_h50
        seeds = 1:5;
    else
        seeds = 1:3;
    end

    % geometry (RNG-independent; the seeded runs reseed inside run_simulation)
    params = config(scn);
    Ts = params.common.Ts; R = params.common.R; a_nom = Ts / params.common.gamma_N;
    w_hat = params.wall.w_hat; pz = params.wall.pz;
    n_warm = round(params.traj.t_hold / Ts);

    ns = numel(seeds);
    trk = zeros(ns, 3); bias = zeros(ns, 3); rstd = zeros(ns, 3);
    sample = [];
    for i = 1:ns
        out = run_simulation(scn, struct('seed', seeds(i)));
        N = numel(out.tout); idx = (n_warm+1):N;
        trk(i, :) = std(out.p_d_out(idx, :) - out.p_m_out(idx, :), 0, 1) * 1e3;
        a_true = local_a_true(out.p_true_out, w_hat, pz, R, a_nom);
        a_hat = out.ekf_out(:, 1:3);
        bias(i, :) = (mean(a_hat(idx, :), 1) - mean(a_true(idx, :), 1)) ./ mean(a_true(idx, :), 1) * 100;
        rstd(i, :) = std(a_hat(idx, :), 0, 1) ./ mean(a_hat(idx, :), 1) * 100;
        if i == 1
            sample = struct('out', out, 'a_true', a_true, 'idx', idx);
        end
    end

    r.scenario   = scn;
    r.trk_nm     = mean(trk, 1);
    r.bias_pct   = mean(bias, 1);
    r.relstd_pct = mean(rstd, 1);

    if is_h50
        pass_trk  = all(r.trk_nm   < opts.trk_thresh_nm);
        pass_bias = all(abs(r.bias_pct) < opts.bias_thresh_pct);
        pass_rstd = all(r.relstd_pct < opts.relstd_thresh_pct);
        r.pass = pass_trk && pass_bias && pass_rstd;
    else
        r.pass = NaN;   % descriptive only
    end

    % ---- report ----
    fprintf('\n===== verify_standalone : %s  (%d seeds) =====\n', scn, ns);
    fprintf('tracking std [x y z] nm : [%.2f %.2f %.2f]', r.trk_nm);
    if is_h50, fprintf('   (< %g -> %s)', opts.trk_thresh_nm, passstr(all(r.trk_nm < opts.trk_thresh_nm))); end
    fprintf('\n');
    fprintf('a_hat bias   [x y z] %%  : [%.2f %.2f %.2f]', r.bias_pct);
    if is_h50, fprintf('   (< %g -> %s)', opts.bias_thresh_pct, passstr(all(abs(r.bias_pct) < opts.bias_thresh_pct))); end
    fprintf('\n');
    fprintf('a_hat rel-std[x y z] %%  : [%.2f %.2f %.2f]', r.relstd_pct);
    if is_h50, fprintf('   (< %g -> %s)', opts.relstd_thresh_pct, passstr(all(r.relstd_pct < opts.relstd_thresh_pct))); end
    fprintf('\n');
    if is_h50
        fprintf('OVERALL: %s\n', passstr(r.pass));
    else
        fprintf('(descriptive -- near-wall a_hat may exceed h50 thresholds; L2 limitation)\n');
    end

    % ---- figures (seed 1) ----
    if opts.make_figs && ~isempty(sample)
        if ~exist(opts.fig_dir, 'dir'); mkdir(opts.fig_dir); end
        make_figures(sample.out, sample.a_true, scn, opts.fig_dir);
        fprintf('figures -> %s\n', opts.fig_dir);
    end
end


function a_true = local_a_true(p_true, w_hat, pz, R, a_nom)
%LOCAL_A_TRUE  Per-step physics ground-truth gain from the noise-free probe.
%   Columns [x y z]; x,y use c_para, z uses c_perp.
    N = size(p_true, 1);
    h_true = (p_true * w_hat - pz) / R;
    a_para = zeros(N, 1); a_perp = zeros(N, 1);
    for k = 1:N
        [cpa, cpe] = wall_corrections(max(h_true(k), 1.001));
        a_para(k) = a_nom / cpa;
        a_perp(k) = a_nom / cpe;
    end
    a_true = [a_para, a_para, a_perp];
end


function s = passstr(b)
    if isnan(b); s = 'N/A'; elseif b; s = 'PASS'; else; s = 'FAIL'; end
end
