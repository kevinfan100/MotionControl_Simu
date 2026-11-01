% STATUS: SCRATCH -- 2026-11-01 (user): who moves a'-hat_33 when V-hat moves (the real symptom). x(15) moves only through K(15,:) nu, so the 7-channel sum must equal the change of a'-hat (closure).
%   Current flow, 40 um/s, 5 seeds. Arms: plane V-hat true+frozen (arm 3) | plane V-hat estimated, starts true, kappa off (arm 1) | sphere as is (theta-hat 0, kappa on). Window t_scan .. +3 s, up to the last finite step.
%   Pre-registered: the channel with >= 50 % of the fastest 0.2 s fall of a'-hat in arm 1 and sphere, without such a fall in arm 3, is the source. Sanity: closure ~ 0.
%   Console per arm: sum per channel / slope (mean, seeds), fraction a' < 0, fastest 0.2 s fall and its split. Figure fig_1101_ap_channels.png (seed 5).
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  rb = 15;  Ts = 1 / 1612;  tS = 12;  Tw = 3;  FS = 14;  cGr = [0.5 0.5 0.5];
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
base = struct('theta_x', 0, 'theta_y', 10, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 10], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'gap_gate', 2, ...
              'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 0.5, 'a_d', ad, 'ap_q', true, 'ap_floor_law', true, 'scan_dir', [1 0], ...
              'kappa_state', false, 'scan_ramp', 0.5, 'scan_len', 1e6, 'du_d', 40 * Ts / Rr, 'sP_fix_theta_deg', 2, 'descent_T', 1, 'descent_v', 15 - hs * Rr, ...
              'post_wait', [0 1], 'T', tS + Tw, 'tag', 'ch');
arm = {'plane, V-hat frozen', 'plane, V-hat moving', 'sphere, as is'};  NA = numel(arm);  NS = 5;  [IA, ID] = ndgrid(1:NA, 1:NS);  rs = cell(size(IA));
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
pctRunOnAll warning('off', 'all');
parfor q = 1:numel(IA)
    o = base;  o.seeds = ID(q);
    if IA(q) == 1; o.freeze_angle = true; end
    if IA(q) == 3
        o.theta_y = 0;  o.th0_deg = [0 0];  o.wall = 'hemi';  o.hemi_r = rb;  o.hemi_void = true;  o.hemi_start_deg = 10;  o.scan_plane_n = [0 1 0];  o.kappa_state = true;  o.kappa_from_scan = true;
    end
    R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t, 'kend', r.kend, 'dap', r.dap, 'apN', r.apN, 'h', r.h);
end
slope = @(hb) arrayfun(@(h) (1 / local_cp(max(h, 1.0003) + 1e-4) - 1 / local_cp(max(h, 1.0003) - 1e-4)) / 2e-4, hb);  ap0 = slope(hs);
nm = {'y1 u', 'y1 v', 'y1 w', 'ym (uu+vv)/2', 'ym ww', 'ym uw', 'ym vw'};  W = round(0.2 / Ts);
for ia = 1:NA
    S = NaN(NS, 7);  M = NaN(NS, 4);  J = NaN(NS, 7);
    for id = 1:NS
        r = rs{ia, id};  kk = (1:r.kend)';  ks = kk(r.t(kk) >= tS & r.t(kk) <= tS + Tw);  ks = ks(all(isfinite(r.dap(ks, :)), 2) & isfinite(r.apN(ks)));
        if numel(ks) < 2 * W; continue; end
        ig = find(diff(ks) > 1, 1);  if ~isempty(ig); ks = ks(1:ig); end
        D = r.dap(ks(2:end), :);  dA = r.apN(ks(end)) - r.apN(ks(1));  S(id, :) = sum(D, 1) / ap0;
        tot = movsum(sum(D, 2), [0 W - 1], 'Endpoints', 'discard');  [mn, i0] = min(tot);  J(id, :) = sum(D(i0:i0 + W - 1, :), 1) / mn;
        M(id, :) = [mean(r.apN(ks) < 0), mn / ap0, r.t(ks(1) + i0) - tS, dA - sum(D(:))];
    end
    fprintf('%s | a''<0 %s | fastest 0.2 s fall / slope %s at t %s | closure max %.1e\n', arm{ia}, mat2str(round(M(:, 1)', 2)), mat2str(round(M(:, 2)', 2)), mat2str(round(M(:, 3)', 2)), max(abs(M(:, 4))));
    for c = 1:7; fprintf('   %-14s sum/slope mean %+6.2f %s | share of the fall %s\n', nm{c}, mean(S(:, c), 'omitnan'), mat2str(round(S(:, c)', 2)), mat2str(round(J(:, c)', 2))); end
end
f = figure('Color', 'w', 'Position', [20 20 620 * NA 900]);  tl = tiledlayout(2, NA, 'TileSpacing', 'compact', 'Padding', 'compact');  cs = lines(7);
for ia = 1:NA
    r = rs{ia, 5};  kk = (1:r.kend)';  k = kk(r.t(kk) >= tS - 1);  kd = k(1:5:end);  ks0 = k(r.t(k) >= tS);
    a1 = nexttile(tl, ia);  hold on;  box on;  plot(r.t(kd) - tS, r.apN(kd) ./ slope(r.h(kd)), 'Color', [0 0.25 0.85], 'LineWidth', 1.3, 'DisplayName', arm{ia});  yline(1, 'r-', 'LineWidth', 1.4, 'HandleVisibility', 'off');
    ylim([-3 3]);  ylabel('estimate / true slope');  legend('Location', 'northoutside', 'Orientation', 'horizontal');
    a2 = nexttile(tl, NA + ia);  hold on;  box on;  for c = 1:7; plot(r.t(ks0) - tS, cumsum(r.dap(ks0, c)) / ap0, 'Color', cs(c, :), 'LineWidth', 1.5, 'DisplayName', nm{c}); end
    yline(0, 'k:', 'HandleVisibility', 'off');  ylim([-4 4]);  ylabel('cumulative update / true slope');  xlabel('time since t_{scan}  (sec)');  legend('Location', 'northoutside', 'NumColumns', 4);
    for a = [a1 a2]; xline(a, 0, '-.', 'Color', cGr, 'LineWidth', 1.4, 'HandleVisibility', 'off');  xlim(a, [-1 Tw]);  set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off'); end
end
fn = fullfile(fo, 'fig_1101_ap_channels.png');  exportgraphics(f, fn, 'Resolution', 130);  close(f);  system(['open ' fn]);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
