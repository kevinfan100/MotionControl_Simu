% STATUS: SCRATCH -- 2026-10-15 (user, test 2 of the wrong-wall plan): where does the final position error come from, and does the
%   estimator know it? Realistic protocol (w_s-hat[0] = 0, true wall moved by DELTA, plant side), Section 9 regulation, S8 + fe_cmd_eval,
%   tau_a 0.1 s, 6 s, seeds 1:5. Per case and seed, last 0.5 s:
%     dh   = h_true - h*                      final position error [R]
%     e_a  = a_true - a_hat                   level error (the regulation holds a_hat at a_d, so dh ~ e_a / a'(h*))
%     e_w  = w_s-hat - w_true                 wall error (w_true = DELTA on the plane, DELTA - 1.05 on the cell, estimator frame)
%     hon_a = |e_a| / sqrt P_aa,  hon_w = |e_w| / sqrt P_ws   (~1 honest, >> 1 overconfident)
%     C    = sum (a'_true(h_true) - a'_hat) Delta h_d over the run (level error the slope error alone would build; e_a - C = repaid by the readings)
function probe_mc_s9_errsplit(run_new, K, extra, sfx, tau, ws0)
    if nargin < 1; run_new = true; end
    if nargin < 3 || isempty(extra); extra = struct(); end   % 10-15: extra ctrl_const overrides (test 3: r2_scale)
    if nargin < 4; sfx = ''; end
    if nargin < 5 || isempty(tau); tau = 0.1; end   % 10-15: tau_a (test 5)
    if nargin < 6 || isempty(ws0); ws0 = 0; end     % 10-15: estimator start w_s-hat[0] (discriminator: start at the true wall)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(genpath(fullfile(root, 'model')));  addpath(here);
    od = fullfile(root, 'test_results', 'temp_1009_ws');  E = struct('fe_cmd_eval', true);  fx = fieldnames(extra);  for i = 1:numel(fx); E.(fx{i}) = extra.(fx{i}); end
    if nargin < 2 || isempty(K); K = {'plane', -1, 0.07, 1.0781; 'plane', 0, 0.07, 1.0781; 'plane', 2, 0.07, 1.0781; 'cell', -1, 0.52, 1.1741; 'cell', 0, 0.52, 1.1741; 'cell', 1, 0.52, 1.1741}; end
    fprintf('wall  DELTA | dh [R] mean (sd) | e_a mean | dh from e_a/a''(h*) | e_w mean (sd) | hon_a | hon_w | C (slope-only level error) | repaid %%\n');
    for i = 1:size(K, 1)
        pl = K{i, 1};  DE = K{i, 2};  ad = K{i, 3};  hs = K{i, 4};
        tg = [strrep(strrep(sprintf('_dc%+g', DE), '+', 'p'), '-', 'm') sfx];
        if run_new; evalc('run_mc_s9_gainreg({pl}, 1:5, ad, tau, 6, ws0, E, tg, [], DE);'); end
        S = load(fullfile(od, sprintf('gainreg_%s_tau%03d%s_T6%s.mat', pl, round(1000 * tau), local_wt(ws0), tg)));  t = S.t;  L = t > 5.5;  ns = size(S.a_hat, 2);
        wtrue = DE - 1.05 * strcmp(pl, 'cell');  aps = local_ap(hs, pl);
        dh = mean(S.h_true(L, :), 1) - hs;  ea = mean(S.a_true(L, :) - S.a_hat(L, :), 1);  ew = S.w_hat(end, :) - wtrue;
        ha = abs(ea) ./ S.sPa(end, :);  hw = abs(ew) ./ S.sPw(end, :);
        C = zeros(1, ns);
        for q = 1:ns
            wd = S.wd(:, q);  wd(1) = wd(2);  dhd = [diff(wd); 0];
            apt = local_ap(max(S.h_true(:, q), 1.001), pl);  aph = 1 ./ max(wd - S.w_hat(:, q), 0.5).^2;
            C(q) = sum((apt - aph) .* dhd);
        end
        fprintf('%-5s %+3g%s | %+.4f (%.4f) | %+.4f | %+.4f | %+.3f (%.3f) | %5.1f | %5.1f | %+.4f | %3.0f\n', pl, DE, sfx, mean(dh), std(dh), mean(ea), mean(ea) / aps, ...
                mean(ew), std(ew), mean(ha), mean(hw), mean(C), 100 * (1 - mean(ea) / mean(C)));
    end
end

function ap = local_ap(h, plant)
    d = 1e-4;  ap = (local_a(h + d, plant) - local_a(h - d, plant)) / (2 * d);
end

function a = local_a(h, plant)
    if strcmp(plant, 'cell'); a = 1 ./ cell_cperp_delta(h); return; end
    a = zeros(size(h));  for i = 1:numel(h); [~, c] = calc_correction_functions(h(i)); a(i) = 1 / c; end
end

function s = local_wt(ws0)
    if ws0 == 0; s = ''; elseif ws0 < 0; s = sprintf('_wm%g', -ws0); else; s = sprintf('_wp%g', ws0); end
end
