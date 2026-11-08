% STATUS: SCRATCH -- 2026-11-02 (user): size of the off-diagonal estimate per step (a_uw, a_vw before the angle step, off_k1) with and without the circle, flat wall true (8,-8), seeds 1-3, 0-10 s, against the true element left in the frame of V-hat. Sanity: no motion 8-10 s about 4e-5. Figure in Preview. Output fig_1102_offdiag_size.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');  Rr = 2.25;  Ts = 1 / 1612;  ad = 0.07;  NS = 3;  hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  AB = char(257);  cc = [0 0 0; 0.85 0 0];
base = struct('theta_x', 8, 'theta_y', -8, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'doc_timing', true, 'theta_out', true, 'gap_gate', 2, 'gate_one', true, 'hold_until', 50, 'tau_a', 0.06, 'a_d', ad, 'descent_v', 15 - hs * Rr, 'descent_T', 1, 'post_wait', [0 1], 'ap_q', true, 'ap_floor_law', true, 'sP_fix_theta_deg', 2, 'q_geo', [0.9 0.9], 'tag', 'od', 'du_d', 40 * Ts / Rr, 'kappa_state', true, 'kappa_from_scan', true, 'kappa_pfix', 0.3, 'scan_ramp', 0.5, 'scan_len', 1e6, 'T', 10);
[IA, ID] = ndgrid(1:2, 1:NS);  N = numel(IA);  rs = cell(N, 1);  if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end;  pctRunOnAll warning('off', 'all');
parfor q = 1:N
    o = base;  o.seeds = ID(q);  if IA(q) == 2; o.dither = [3 8 2];  o.dither_win = [0 50]; end;  R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t(:), 'off', r.off_k1, 'off_tr', r.off_tr, 'dth', r.dth);
end
W = [0.5 2; 2 4; 4 6; 6 10];  nm = {'no motion', 'circle 3 \mum'};  f = figure('Color', 'w', 'Position', [40 40 1500 620], 'Visible', 'off');  a = axes(f);  hold(a, 'on');  hl = gobjects(2, 2);
for c = 1:2
    id = find(IA == c);  n = min(cellfun(@(r) find(isfinite(r.off(:, 1)), 1, 'last'), rs(id)));  t = rs{id(1)}.t(1:n);  O = zeros(n, 1);  T = zeros(n, 1);  for s = id'; O = O + sum(rs{s}.off(1:n, :).^2, 2);  T = T + sum(rs{s}.off_tr(1:n, :).^2, 2); end;  O = O / (2 * NS);  T = T / (2 * NS);
    fprintf('%s:', nm{c});  for j = 1:size(W, 1); m = t >= W(j, 1) & t < W(j, 2); fprintf(' | %.1f-%.0f s estimate rms %.1e, true left %.1e', W(j, 1), W(j, 2), sqrt(mean(O(m))), sqrt(mean(T(m)))); end;  fprintf('\n');
    k = (1:4:n)';  hl(c, 1) = plot(a, t(k), sqrt(movmean(O(k), 25)), 'Color', cc(c, :), 'LineWidth', 1.3);  hl(c, 2) = plot(a, t(k), sqrt(movmean(T(k), 25)), '--', 'Color', cc(c, :), 'LineWidth', 1.3);
end
set(a, 'YScale', 'log', 'FontSize', 14, 'FontWeight', 'bold', 'LineWidth', 1.2);  xlim(a, [0 10]);  ylim(a, [1e-6 0.05]);  box(a, 'on');  xlabel(a, 'Time (sec)');  ylabel(a, [AB '_{uw}, ' AB '_{vw}  rms']);  legend(a, hl(:), {'estimate per step, no motion', 'estimate per step, circle', 'true left, no motion', 'true left, circle'}, 'Location', 'northoutside', 'Orientation', 'horizontal');
fn = fullfile(fo, 'fig_1102_offdiag_size.png');  exportgraphics(f, fn, 'Resolution', 120);  close(f);  system(['open -a Preview "' fn '"']);
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
