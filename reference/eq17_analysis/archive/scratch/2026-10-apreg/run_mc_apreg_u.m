% STATUS: SCRATCH -- 2026-10-18 (user-approved test): a' state (ap_slot, same settings as run_mc_apreg 'state') with the regulation law
%   written in u = 1/(1 - a) instead of a, no speed cap:  Delta h_d = (1 - lambda_a)(u_d - u_hat)/u_hat',  u_hat' = a_hat'/(1 - a_hat)^2,
%   same law-slope gate (beta 2/51, thr 0.0066 R/step, floor 1/22.2^2). tau_a in TAUS. Walls plane a_d 0.07, cell a_d 0.60 and 0.55.
%   Output test_results/temp_1009_ws/apreg_stateu<tau x100>_<wall><tag>.mat (same fields as run_mc_apreg).
function run_mc_apreg_u(TAUS, seeds, arm, cases, oracle)
    if nargin < 1 || isempty(TAUS); TAUS = [0.5 1.0]; end
    if nargin < 2 || isempty(seeds); seeds = 1:5; end
    if nargin < 3 || isempty(arm); arm = 'state'; end      % 10-18: 'state' (a' state) | 'ws' (Section 8/9 estimator, w_s state, fe_cmd_eval)
    if nargin < 5; oracle = ''; end                          % 10-18: '' | 'slope' | 'a' (law reads the truth, diagnostic)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);
    addpath(genpath(fullfile(root, 'model')));  addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'temp_1009_ws');  pc = physical_constants();
    T = 6;  B = 2 / 51;  THR = 0.0066;  AP_MIN = 1 / 22.2^2;  n = round(T / pc.Ts);
    CASES = {'plane', 0.07, ''; 'cell', 0.60, '_ad060'; 'cell', 0.55, '_ad055'};
    if nargin >= 4 && ~isempty(cases); CASES = CASES(cases, :); end
    AP = struct('ap_slot', true, 'fe_cmd_eval', true, 'delta_init', 1 / 22.2^2, 'Pf_delta_std', 0.002, 'plaw_delta_lim', [-1 10], ...
                'plaw_seed_cross', false, 'q77_adapt_P0', 0.4, 'q77_hscale', 1.1);
    if strcmp(arm, 'ws'); AP = struct('fe_cmd_eval', true); end
    GX = struct('beta', B, 'thr', THR, 'ap_min', AP_MIN, 'coord', 'u');  if ~isempty(oracle); GX.oracle = oracle; end
    atag = 'stateu';  if strcmp(arm, 'ws'); atag = 'wsu'; end;  if ~isempty(oracle); atag = [atag oracle]; end
    for tau = TAUS
        for ic = 1:size(CASES, 1)
            w = CASES{ic, 1};  ad = CASES{ic, 2};
            res = struct('arm', atag, 'wall', w, 'a_d', ad, 'tau', tau, 'seeds', seeds, 't', (0:n-1)' * pc.Ts, 'contact', false(1, numel(seeds)));
            [res.wd, res.h_true, res.a_true, res.a_hat, res.ap_hat, res.dxm] = deal(NaN(n, numel(seeds)));
            for iq = 1:numel(seeds)
                s = seeds(iq);  tg = sprintf('_apregu_s%d', s);
                try
                    evalc('run_mc_s9_gainreg({w}, s, ad, tau, T, 0, AP, tg, [], 0, GX);');
                    f = fullfile(od, sprintf('gainreg_%s_tau%03d_T%g%s.mat', w, round(1000 * tau), T, tg));  S = load(f);  delete(f);
                    m = min(n, numel(S.t));  r = struct('wd', S.wd, 'h_true', S.h_true, 'a_true', S.a_true, 'a_hat', S.a_hat, 'ap_hat', -S.w_hat, 'dxm', S.dxm);
                    fn = fieldnames(r);  for q = 1:numel(fn); res.(fn{q})(1:m, iq) = r.(fn{q})(1:m); end
                catch ME
                    res.contact(iq) = true;  fprintf('  [%s]\n', ME.message);
                end
                st = 'ok';  if res.contact(iq); st = 'CONTACT'; end
                fprintf('%s tau %.1f %-5s a_d %.2f seed %d: %s\n', atag, tau, w, ad, s, st);
            end
            if numel(seeds) == 5; save(fullfile(od, sprintf('apreg_%s%03d_%s%s.mat', atag, round(100 * tau), w, CASES{ic, 3})), '-struct', 'res'); end
        end
    end
end
