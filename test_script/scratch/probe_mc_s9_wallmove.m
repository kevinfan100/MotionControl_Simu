% STATUS: SCRATCH -- 2026-10-15 (user): the estimator always starts at w_s-hat[0] = 0 while the TRUE wall is moved by DELTA R on the
%   plant side (run_formC_b opts.ws_inject; DELTA > 0 = wall closer to the probe than assumed). Section 9 setting: plane, a_d 0.07,
%   tau_a 0.1 s, 6 s, S8 estimator with fe_cmd_eval, seeds 1:5. The plant gain reaches a_d at the true gap h* = 1.078 R.
%   Prints per DELTA: contact (run fails), last 0.5 s mean of (h_true - h*) [R] and of a_true - a_d (seed mean, worst seed), w_s-hat end
%   vs DELTA, min true gap. Files gainreg_plane_tau100_T6_wm<..>.mat with tag _mv<DELTA>.
function probe_mc_s9_wallmove(deltas, seeds, plant, ad, hs)
    if nargin < 1 || isempty(deltas); deltas = [-3 -2 -1 0 1 2 3]; end
    if nargin < 2 || isempty(seeds); seeds = 1:5; end
    if nargin < 3 || isempty(plant); plant = 'plane'; end   % 10-15: 'cell' too (target height hs must match)
    if nargin < 4 || isempty(ad); ad = 0.07; end
    if nargin < 5 || isempty(hs); hs = 1.0781; end
    here = fileparts(mfilename('fullpath'));  od = fullfile(fileparts(fileparts(here)), 'test_results', 'temp_1009_ws');  addpath(here);
    E = struct('fe_cmd_eval', true);
    fprintf('[%s, a_d %.2f, h* %.3f R]\n', plant, ad, hs);
    fprintf('DELTA | h_true - h* [R] mean (worst seed) | a - a_d mean (worst) | w_s-hat end mean (sd) | min true gap h\n');
    for DE = deltas
        tg = strrep(strrep(sprintf('_mv%+g', DE), '+', 'p'), '-', 'm');  if ~strcmp(plant, 'plane') || ad ~= 0.07; tg = sprintf('%s_ad%03d', tg, round(1000 * ad)); end
        try
            evalc('run_mc_s9_gainreg({plant}, seeds, ad, 0.1, 6, 0, E, tg, [], DE);');
        catch ME
            fprintf('%+5g | CONTACT (%s)\n', DE, ME.message);  continue
        end
        S = load(fullfile(od, sprintf('gainreg_%s_tau100_T6%s.mat', plant, tg)));  L = S.t > 5.5;
        dh = mean(S.h_true(L, :), 1) - hs;  da = mean(S.a_true(L, :), 1) - ad;  [~, iw] = max(abs(dh));
        fprintf('%+5g | %+.4f (%+.4f) | %+.4f (%+.4f) | %+.3f (%.3f) | %.4f\n', DE, mean(dh), dh(iw), mean(da), da(iw), mean(S.w_hat(end, :)), std(S.w_hat(end, :)), min(S.h_true(:)));
    end
end
