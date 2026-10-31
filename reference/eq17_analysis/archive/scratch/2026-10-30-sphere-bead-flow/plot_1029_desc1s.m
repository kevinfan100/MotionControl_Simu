% STATUS: SCRATCH -- 2026-10-29 (user): as the 1019 report Fig. 2.1 (constant-velocity descent in 1 s, a' as a state) but with the current
%   flow: void sphere radius 15 um, start 10 deg off the top 15 um up, theta-hat[0] = 0, prior 10 deg, + kappa; hold 0-10 s, constant-speed
%   descent 10-11 s along the estimated normal from 15 um to h* (a_d = 0.05), then held (no regulation, no scan) to 12 s.
%   Arms: slope P_fix of the 1019 report 0.4 (1.1/h_d)^2 | law L. Seeds 1-3 (console); seed 1 drawn in the report layout:
%   gap (command and actual), a_33 vs a-hat_33, a'_33 vs a'-hat_33 (log), from 9.8 s. Output test_results/temp_3d_wsonly/fig_1029_desc1s.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  rb = 15;  Ts = 1 / 1612;  FS = 14;  hatc = char(770);  AB = char(257);  cT = [0.85 0 0];  cE = [0 0.25 0.85];  cGr = [0.5 0.5 0.5];
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, ...
              'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 2, 'a_d', ad, ...
              'ap_q', true, 'wall', 'hemi', 'hemi_r', rb, 'hemi_void', true, 'ap_floor_law', true, 'scan_dir', [1 0], 'hemi_start_deg', 10, ...
              'scan_plane_n', [0 1 0], 'kappa_state', true, 'scan_len', 0, 'reg_off', true, 'descent_T', 1, 'descent_v', 15 - hs * Rr, ...
              'q_theta_from', 1e9, 'sP_fix_theta_deg', 2, 'T', 12, 'tag', 'd1');
arm = {'slope P_{fix} of the 1019 report', 'slope P_{fix} law L'};
[IA, ID] = ndgrid(1:2, 1:3);  rs = cell(size(IA));
if isempty(gcp('nocreate')); parpool('Processes', 6); end
parfor q = 1:numel(IA)
    o = base;  o.seeds = ID(q);  o.ap_pfix_law = IA(q) == 2;  R = run_t3_xyz_fixed(o);  rs{q} = R.runs{1};
end
slope = @(hb) arrayfun(@(h) (1 / local_cp(max(h, 1.0003) + 1e-4) - 1 / local_cp(max(h, 1.0003) - 1e-4)) / 2e-4, hb);
for ia = 1:2
    for id = 1:3
        r = rs{ia, id};  k1 = find(r.t >= 11 - 1e-9, 1);  kh = find(r.t >= 11.1 & r.t <= 12 & (1:numel(r.t))' <= r.kend);
        fprintf('%-34s seed %d: contact %d | at 11 s gap %.2f um, a33 %.3f a-hat %.3f, a''-hat/true %.2f | hold 11.1-12 s: gap %.2f um, a-hat - a %+.3f, a''-hat/true %.2f\n', ...
                strrep(arm{ia}, '_{fix}', '_fix'), id, r.contact, r.h(k1) * Rr, r.aN_tr(k1), r.aN(k1), r.apN(k1) / slope(r.h(k1)), mean(r.h(kh)) * Rr, mean(r.aN(kh) - r.aN_tr(kh)), ...
                median(r.apN(kh(1:10:end)) ./ slope(r.h(kh(1:10:end)))));
    end
end
f = figure('Color', 'w', 'Position', [30 30 1700 1300]);  tl = tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for ia = 1:2
    r = rs{ia, 1};  k = find(r.t >= 9.8 & (1:numel(r.t))' <= r.kend);  t = r.t(k);  kd = k(1:5:end);
    hd = r.hd_int(k) * Rr;
    a1 = nexttile(tl, ia);  hold on;  box on;  h1 = plot(t, hd, 'Color', cT, 'LineWidth', 2.2);  h2 = plot(t, r.h(k) * Rr, '--', 'Color', cE, 'LineWidth', 1.6);
    ylim([0 16]);  ylabel('gap  [\mum]');  title(sprintf('%s, seed 1', arm{ia}));  legend([h1 h2], {'h_d (command)', 'h'}, 'Location', 'northeast');
    a2 = nexttile(tl, 2 + ia);  hold on;  box on;  h1 = plot(t, r.aN_tr(k), 'Color', cT, 'LineWidth', 2.2);  h2 = plot(t, r.aN(k), 'Color', cE, 'LineWidth', 1.2);  yline(ad, 'k--', 'LineWidth', 1.3);
    ylim([0 1]);  ylabel(sprintf('%s_{33}', AB));  legend([h1 h2], {sprintf('%s_{33}', AB), sprintf('%s%s_{33}', AB, hatc)}, 'Location', 'northeast');
    a3 = nexttile(tl, 4 + ia);  hold on;  box on;  h1 = plot(r.t(kd), slope(r.h(kd)), 'Color', cT, 'LineWidth', 2.2);  h2 = plot(t, max(r.apN(k), 1e-4), 'Color', cE, 'LineWidth', 1.2);
    set(a3, 'YScale', 'log');  ylim([1e-3 3]);  ylabel(sprintf('%s''_{33}', AB));  xlabel('Time (sec)');
    legend([h1 h2], {sprintf('%s''_{33}', AB), sprintf('%s%s''_{33}', AB, hatc)}, 'Location', 'southeast');
    for a = [a1 a2 a3]; xline(a, 10, '--', 'Color', cGr, 'LineWidth', 1.2, 'HandleVisibility', 'off');  xline(a, 11, '-.', 'Color', cGr, 'LineWidth', 1.2, 'HandleVisibility', 'off');
        set(a, 'XLim', [9.8 12], 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off'); end
end
fn = fullfile(fo, 'fig_1029_desc1s.png');  exportgraphics(f, fn, 'Resolution', 140);  close(f);  system(['open ' fn]);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
