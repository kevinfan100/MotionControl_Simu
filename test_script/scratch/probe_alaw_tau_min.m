% STATUS: SCRATCH -- 2026-10-19 (user): a law only (no u), no speed cap, slope gate off (thr 0): how small can tau be?
%   Plane a_d 0.07 (h* 1.078 R), parked at 22.2 R, seeds 1:5. T = min(max(3, 2.5 tau), 20) s. Per tau and estimator:
%   contacts, time to h < h* + 0.1, min h, last 0.5 s h - h*. Log to the console.
function probe_alaw_tau_min(TAUS, arms, seeds, wall, ad)
    if nargin < 4 || isempty(wall); wall = 'plane'; end   % 10-19: 'cell' added
    if nargin < 5 || isempty(ad); ad = 0.07; end
    if nargin < 1 || isempty(TAUS); TAUS = [0.1 0.3 1 2 4 8]; end
    if nargin < 2 || isempty(arms); arms = {'ap', 'ws'}; end
    if nargin < 3 || isempty(seeds); seeds = 1:5; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);
    addpath(genpath(fullfile(root, 'model')));  addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'temp_1009_ws');  hs = fzero(@(x) local_a(x, wall) - ad, [1.001 30]);
    fprintf('%s wall, a_d %.2f, h* %.3f R\n', wall, ad, hs);
    AP = struct('ap_slot', true, 'fe_cmd_eval', true, 'delta_init', 1 / 22.2^2, 'Pf_delta_std', 0.002, 'plaw_delta_lim', [-1 10], ...
                'plaw_seed_cross', false, 'q77_adapt_P0', 0.4, 'q77_hscale', 1.1);
    WS = struct('fe_cmd_eval', true);  GX = struct('beta', 2 / 51, 'thr', 0, 'ap_min', 1 / 22.2^2);
    fprintf('arm | tau [s] | T [s] | contacts | t(h < h*+0.1) mean | min h | last 0.5 s h - h* mean (sd)\n');
    for ia = 1:numel(arms)
        E = AP;  if strcmp(arms{ia}, 'ws'); E = WS; end
        for tau = TAUS
            T = min(max(3, 2.5 * tau), 20);  nc = 0;  t1 = [];  mh = [];  dh = [];
            for s = seeds
                tg = sprintf('_tm%s%d', arms{ia}, s);  clear run_formC_b motion_control_law_formC_b;
                try
                    evalc('run_mc_s9_gainreg({wall}, s, ad, tau, T, 0, E, tg, [], 0, GX);');
                    f = fullfile(od, sprintf('gainreg_%s_tau%03d%s%s.mat', wall, round(1000 * tau), local_ttag(T), tg));  S = load(f);  delete(f);
                    i1 = find(S.h_true < hs + 0.1, 1);  if ~isempty(i1); t1(end+1) = S.t(i1); end %#ok<AGROW>
                    mh(end+1) = min(S.h_true);  dh(end+1) = mean(S.h_true(S.t > S.t(end) - 0.5)) - hs; %#ok<AGROW>
                catch
                    nc = nc + 1;
                end
            end
            fprintf('%-3s | %5.1f | %4.1f | %d/%d | %6.2f s (%d reached) | %.3f | %+.3f (%.3f)\n', arms{ia}, tau, T, nc, numel(seeds), ...
                    mean(t1), numel(t1), min([mh Inf]), mean(dh), std(dh));
        end
    end
end

function s = local_ttag(T)
    s = '';  if T ~= 1.5; s = sprintf('_T%g', T); end
end

function a = local_a(h, wall)
    if strcmp(wall, 'cell'); a = 1 ./ cell_cperp_delta(h); return; end
    [~, c] = calc_correction_functions(h);  a = 1 / c;
end
