% STATUS: SCRATCH -- 2026-11-01 (user): why two zero-mean quantities F_w and K(11,:) nu add up steadily in the feedback of delta-w-hat_3. S(L) = sum over the scan of -F_w[k - L] (K(11,:) nu)[k], L = -4 .. 6.
%   F_w = f_d[k] + (1 - lc)(f_d[k-1] + f_d[k-2]), control uses estimates after the feedback, so F_w[k] contains nu[k]. Pre-registered: same-step if |S(0)| >= 2 max |S(L != 0)| with the sign of the total; otherwise built over steps.
%   Sanity: S(0) = sum L.fb9(:,2) (error ~ 0), signs: negative at P alive 40, positive at P alive 10 and C. 5 seeds, window t_scan .. +3 s. Arms: plane, V-hat frozen: P alive 40 | P alive 10 | C 40; sphere, true normal, V-hat moving.
%   Console: S(L) mean over seeds and per seed for L = 0, normalized corr at L = 0 (chance sd 1 / sqrt(N)). Figure fig_1101_fw_lag.png.
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  rb = 15;  Ts = 1 / 1612;  tS = 12;  Tw = 3;  FS = 14;
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
base = struct('theta_x', 0, 'theta_y', 10, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 10], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'gap_gate', 2, ...
              'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 0.5, 'a_d', ad, 'ap_q', true, 'ap_floor_law', true, 'scan_dir', [1 0], ...
              'kappa_state', false, 'scan_ramp', 0.5, 'scan_len', 1e6, 'du_d', 40 * Ts / Rr, 'sP_fix_theta_deg', 2, 'descent_T', 1, 'descent_v', 15 - hs * Rr, ...
              'post_wait', [0 1], 'T', tS + Tw, 'tag', 'fl', 'freeze_angle', true);
arm = {'P alive, 40', 'P alive, 10', 'C, 40', 'sphere, true normal, 40'};  NA = numel(arm);  NS = 5;  [IA, ID] = ndgrid(1:NA, 1:NS);  rs = cell(size(IA));
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
parfor q = 1:numel(IA)
    o = base;  o.seeds = ID(q);
    if IA(q) == 2; o.du_d = 10 * Ts / Rr; end
    if IA(q) == 3; o.sP_theta_deg = 1e-3;  o.q_theta_off = true; end
    if IA(q) == 4
        o = rmfield(o, 'freeze_angle');  o.theta_y = 0;  o.th0_deg = [0 0];  o.wall = 'hemi';  o.hemi_r = rb;  o.hemi_void = true;  o.hemi_start_deg = 10;  o.scan_plane_n = [0 1 0];
        o.kappa_state = true;  o.kappa_from_scan = true;  o.cheat_w = true;
    end
    R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t, 'kend', r.kend, 'Fw', r.vF(:, 3), 'dg', sum(r.da33, 2), 'fb2', r.fb9(:, 2));
end
LL = -4:6;  NL = numel(LL);  cs = lines(NA);
f = figure('Color', 'w', 'Position', [20 20 900 620]);  a = axes(f);  hold(a, 'on');  box(a, 'on');
for ia = 1:NA
    S = NaN(NS, NL);  C = NaN(NS, 1);  ck = NaN(NS, 1);
    for id = 1:NS
        r = rs{ia, id};  kk = (1:r.kend)';  ks = kk(r.t(kk) >= tS & r.t(kk) <= tS + Tw);  ks = ks(1:end - 5);
        for j = 1:NL; S(id, j) = sum(-r.Fw(ks - LL(j)) .* r.dg(ks)); end
        C(id) = sum(r.Fw(ks) .* r.dg(ks)) / sqrt(sum(r.Fw(ks).^2) * sum(r.dg(ks).^2));  ck(id) = S(id, LL == 0) - sum(r.fb2(ks));
    end
    fprintf('%s | sanity S(0) - sum fb9: max %.1e | N %d, chance sd of corr %.3f\n   S(L=0) per seed %s | corr(F_w, K11 nu) %s\n', arm{ia}, max(abs(ck)), numel(ks), 1 / sqrt(numel(ks)), mat2str(round(S(:, LL == 0)', 3)), mat2str(round(C', 3)));
    fprintf('   L:     %s\n   S mean %s\n   S sem  %s\n   sum over L %+.3f\n', sprintf('%7d', LL), sprintf('%+7.3f', mean(S, 1)), sprintf('%7.3f', std(S, 0, 1) / sqrt(NS)), mean(sum(S, 2)));
    errorbar(a, LL + (ia - 2.5) * 0.08, mean(S, 1), std(S, 0, 1) / sqrt(NS), '-o', 'Color', cs(ia, :), 'MarkerFaceColor', cs(ia, :), 'LineWidth', 1.4, 'DisplayName', arm{ia});
end
yline(a, 0, 'k:', 'HandleVisibility', 'off');  xline(a, 0, 'k-.', 'HandleVisibility', 'off');  xlabel(a, 'lag L  (steps, F_w[k - L])');  ylabel(a, 'sum of -F_w[k - L] K_{11} \nu[k]  [R]');
legend(a, 'Location', 'northoutside', 'NumColumns', 2);  set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);
fn = fullfile(fo, 'fig_1101_fw_lag.png');  exportgraphics(f, fn, 'Resolution', 130);  close(f);  system(['open ' fn]);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
