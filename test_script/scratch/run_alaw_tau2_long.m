% STATUS: SCRATCH -- 2026-10-19 (user): final value check. a law, tau 2 s, no speed cap, slope gate off, plane a_d 0.07 (h* 1.078 R),
%   parked at 22.2 R, 20 s, seeds 1:5, w_s state and a' state. Saves test_results/temp_1009_ws/alaw_tau2_long_<arm>.mat (seed columns).
function run_alaw_tau2_long(seeds, T)
    if nargin < 1 || isempty(seeds); seeds = 1:5; end
    if nargin < 2 || isempty(T); T = 20; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);
    addpath(genpath(fullfile(root, 'model')));  addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'temp_1009_ws');
    AP = struct('ap_slot', true, 'fe_cmd_eval', true, 'delta_init', 1 / 22.2^2, 'Pf_delta_std', 0.002, 'plaw_delta_lim', [-1 10], ...
                'plaw_seed_cross', false, 'q77_adapt_P0', 0.4, 'q77_hscale', 1.1);
    WS = struct('fe_cmd_eval', true);  GX = struct('beta', 2 / 51, 'thr', 0, 'ap_min', 1 / 22.2^2);
    arms = {'ws', 'ap'};  EE = {WS, AP};
    for ia = 1:2
        E = EE{ia};  res = struct('arm', arms{ia}, 'seeds', seeds, 'a_d', 0.07, 'tau', 2, 'contact', false(1, numel(seeds)));
        for iq = 1:numel(seeds)
            s = seeds(iq);  tg = sprintf('_l2%s%d', arms{ia}, s);  clear run_formC_b motion_control_law_formC_b;
            try
                evalc('run_mc_s9_gainreg({''plane''}, s, 0.07, 2.0, T, 0, E, tg, [], 0, GX);');
                f = fullfile(od, sprintf('gainreg_plane_tau2000_T%g%s.mat', T, tg));  S = load(f);  delete(f);
                if iq == 1; n = numel(S.t);  res.t = S.t;  [res.wd, res.h_true, res.a_true, res.a_hat, res.ap_hat, res.ap_true, res.dxm] = deal(NaN(n, numel(seeds))); end
                res.wd(:, iq) = S.wd;  res.h_true(:, iq) = S.h_true;  res.a_true(:, iq) = S.a_true;  res.a_hat(:, iq) = S.a_hat;
                res.ap_hat(:, iq) = S.ap_hat;  res.ap_true(:, iq) = S.ap_true;  res.dxm(:, iq) = S.dxm;
            catch ME
                res.contact(iq) = true;  fprintf('  %s seed %d: %s\n', arms{ia}, s, ME.message);
            end
            fprintf('%s seed %d done\n', arms{ia}, s);
        end
        sfx = '';  if T ~= 20; sfx = sprintf('_T%g', T); end
        save(fullfile(od, sprintf('alaw_tau2_long_%s%s.mat', arms{ia}, sfx)), '-struct', 'res');
    end
end
