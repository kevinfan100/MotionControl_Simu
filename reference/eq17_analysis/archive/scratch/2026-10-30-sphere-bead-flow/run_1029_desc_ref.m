% STATUS: SCRATCH -- 2026-10-29 (user): reference arm for the descent lag -- constant-speed descent 15 um -> h* over Td = 5 / 10 s (no
%   regulation), 40 um/s turn afterwards (not evaluated here), + kappa. Arms: estimator as is | reference arm: the gain prediction uses the
%   true slopes at the true gap (opts.cheat_slope_pred). Seeds 1-3. Console: at the end of the descent (scan start) gap, a_33, a-hat_33,
%   a'-hat_33 / true slope; lag of a-hat_33 over the last 2 s of the descent. Figure: a_33 and a-hat_33 over the descent, Td = 10 s, seed 1.
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  rb = 15;  Ts = 1 / 1612;  FS = 14;  hatc = char(770);  AB = char(257);  cT = [0.85 0 0];  cE = [0 0.25 0.85];  cO = [0.85 0.4 0];
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  Lturn = 2 * pi * (rb + hs * Rr);
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, ...
              'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 2, 'a_d', ad, ...
              'ap_q', true, 'wall', 'hemi', 'hemi_r', rb, 'hemi_void', true, 'ap_floor_law', true, 'scan_dir', [1 0], 'hemi_start_deg', 10, ...
              'scan_plane_n', [0 1 0], 'ap_pfix_law', true, 'kappa_state', true, 'scan_len', Lturn, 'du_d', 40 * Ts / Rr, 'sP_fix_theta_deg', 2, 'tag', 'dr');
TD = [5 10];  arm = {'estimator as is', 'reference: true slope in the prediction'};
[IT, IA, ID] = ndgrid(1:2, 1:2, 1:3);  rs = cell(size(IT));
if isempty(gcp('nocreate')); parpool('Processes', 12); end
parfor q = 1:numel(IT)
    o = base;  Td = TD(IT(q));  tS = 10 + Td;  o.descent_T = Td;  o.descent_v = (15 - hs * Rr) / Td;  o.q_theta_from = tS;  o.T = tS + 0.05;  o.seeds = ID(q);
    if IA(q) == 2; o.cheat_slope_pred = true; end
    R = run_t3_xyz_fixed(o);  rs{q} = R.runs{1};
end
for it = 1:2
    for ia = 1:2
        M = zeros(3, 5);  tS = 10 + TD(it);
        for id = 1:3
            r = rs{it, ia, id};  kS = find(r.t >= tS - 1e-9, 1);  kl = find(r.t >= tS - 2 & r.t < tS);
            h = max(r.h(kS), 1.0003);  [~, c1] = calc_correction_functions(h + 1e-4);  [~, c0] = calc_correction_functions(h - 1e-4);
            M(id, :) = [r.h(kS) * Rr, r.aN_tr(kS), r.aN(kS), r.apN(kS) / ((1 / c1 - 1 / c0) / 2e-4), mean(r.aN(kl) - r.aN_tr(kl))];
        end
        fprintf('descent %2d s, %-40s: at the end gap %s um | a33 %s | a-hat_33 %s | a''-hat/true %s | mean a-hat - a over the last 2 s %s\n', TD(it), arm{ia}, ...
                mat2str(round(M(:, 1)', 2)), mat2str(round(M(:, 2)', 3)), mat2str(round(M(:, 3)', 3)), mat2str(round(M(:, 4)', 2)), mat2str(round(M(:, 5)', 3)));
    end
end
f = figure('Color', 'w', 'Position', [40 40 1800 650]);  tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for it = 1:2
    ax = nexttile(tl);  hold on;  box on;  tS = 10 + TD(it);
    r0 = rs{it, 1, 1};  r1 = rs{it, 2, 1};  k = find(r0.t >= 10 & r0.t <= tS);
    h1 = plot(r0.t(k), r0.aN_tr(k), 'Color', cT, 'LineWidth', 2.4);  h2 = plot(r0.t(k), r0.aN(k), 'Color', cE, 'LineWidth', 1.4);
    h3 = plot(r1.t(k), r1.aN(k), 'Color', cO, 'LineWidth', 1.4);  h4 = yline(ad, 'k--', 'LineWidth', 1.4);
    ylim([0 0.9]);  xlim([10 tS]);  xlabel('Time (sec)');  ylabel(sprintf('%s_{33}', AB));  title(sprintf('constant-speed descent %d s, seed 1', TD(it)));
    legend([h1 h2 h3 h4], {sprintf('%s_{33} (true)', AB), sprintf('%s%s_{33}, estimator as is', AB, hatc), sprintf('%s%s_{33}, true slope in the prediction', AB, hatc), sprintf('%s_d', AB)}, 'Location', 'southwest');
    set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(ax.Toolbar, 'Visible', 'off');
end
fn = fullfile(fo, 'fig_1029_desc_ref.png');  exportgraphics(f, fn, 'Resolution', 140);  close(f);  system(['open ' fn]);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
