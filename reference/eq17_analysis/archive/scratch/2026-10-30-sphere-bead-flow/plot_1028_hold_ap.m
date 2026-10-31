% STATUS: SCRATCH -- 2026-10-28 (user): a'_33 without any scan -- hold, regulation, hold. Void hemisphere (radius 15 um), start 10 deg off the
%   top on the +x meridian, 15 um above the sphere, theta-hat[0] = 0, angle prior 10 deg; hold 0-10 s, regulation to a_d = 0.05 (law-derived
%   a' floor) from 10 s to the end (25 s). No scan, angle Q = 0 throughout. Document version, seeds 1-3 (seed 1 drawn).
%   Arms (columns): A as is | B slope sqrt P_fix x 1/4 | C slope sqrt P_fix x 1/10 | D = B with tau_a 4 s (slower descent).
%   Rows: gap h (true), gains, a'_33 vs the true slope, angles. Output test_results/temp_3d_wsonly/fig_1028_hold_ap.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  rb = 15;  tH = 10;  T = 25;  ph0 = 10;
FS = 13;  hatc = char(770);  AB = char(257);  th = char(952);  cT = [0.85 0 0];  cE = [0 0.25 0.85];  cG = [0 0.55 0.2];  cGr = [0.5 0.5 0.5];
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, ...
              'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', tH, 'tau_a', 2, 'a_d', ad, ...
              'ap_q', true, 'wall', 'hemi', 'hemi_r', rb, 'hemi_void', true, 'ap_floor_law', true, 'scan_dir', [1 0], 'hemi_start_deg', ph0, 'T', T, 'tag', 'ha');
arm = {'A: as is', 'B: slope sqrt P_{fix} x 1/4', 'C: slope sqrt P_{fix} x 1/10', 'D: B + \tau_a 4 s'};  na = numel(arm);
[IA, ID] = ndgrid(1:na, 1:3);  rs = cell(size(IA));
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
parfor q = 1:numel(IA)
    o = base;  o.seeds = ID(q);
    if IA(q) == 2 || IA(q) == 4; o.ap_pfix_scale = 0.25; end
    if IA(q) == 3; o.ap_pfix_scale = 0.1; end
    if IA(q) == 4; o.tau_a = 4; end
    R = run_t3_xyz_fixed(o);  rs{q} = R.runs{1};
end
slope = @(hb) arrayfun(@(h) (1 / local_cp(max(h, 1.0003) + 1e-4) - 1 / local_cp(max(h, 1.0003) - 1e-4)) / 2e-4, hb);
for ia = 1:na
    for id = 1:3
        r = rs{ia, id};  k = (1:r.kend)';  w1 = k(r.t(k) >= 11 & r.t(k) < 17);  w2 = k(r.t(k) >= 17);  w1 = w1(1:5:end);  w2 = w2(1:5:end);
        e1 = r.apN(w1) - slope(r.h(w1));  e2 = r.apN(w2) - slope(r.h(w2));
        fprintf('%-30s seed %d: contact %d | a''33 err 11-17 s: mean %+.2f rms %.2f | 17-25 s: mean %+.2f rms %.2f | end %+.2f | a33 err 17-25 s rms %.3f | gap at 25 s %.2f um | angle err at 25 s %.1f deg\n', ...
                strrep(arm{ia}, '\tau', 'tau'), id, r.contact, mean(e1), rms(e1), mean(e2), rms(e2), e2(end), rms(r.aN(w2) - r.aN_tr(w2)), r.h(r.kend) * Rr, r.th_tr(r.kend, 2) - r.thy(r.kend));
    end
end
f = figure('Color', 'w', 'Position', [20 20 2400 1300]);  tl = tiledlayout(4, na, 'TileSpacing', 'compact', 'Padding', 'compact');
for ia = 1:na
    r = rs{ia, 1};  k = (1:r.kend)';  t = r.t(k);  kd = k(1:10:end);
    a0 = nexttile(tl, ia);  hold on;  box on;  plot(t, r.h(k) * Rr, 'Color', cE, 'LineWidth', 1.3);  yline(hs * Rr, 'k--', 'LineWidth', 1.2);
    set(a0, 'YScale', 'log');  ylim([2 20]);  ylabel('gap  [\mum]');  title(sprintf('%s, seed 1', arm{ia}));
    axg = nexttile(tl, na + ia);  hold on;  box on;
    h1 = plot(t, r.aN_tr(k), 'Color', cT, 'LineWidth', 2.2);  plot(t, r.aT_tr(k), 'Color', cT, 'LineWidth', 2.2);
    h2 = plot(t, r.aN(k), 'Color', cE, 'LineWidth', 1.2);  h3 = plot(t, r.aT(k), 'Color', cG, 'LineWidth', 1.2);  ylim([0 1.1]);  ylabel('gain');
    legend([h1 h2 h3], {sprintf('%s_{33}, %s_{11}', AB, AB), sprintf('%s%s_{33}', AB, hatc), sprintf('%s%s_{11}', AB, hatc)}, 'Location', 'northoutside', 'Orientation', 'horizontal');
    axp = nexttile(tl, 2 * na + ia);  hold on;  box on;
    h1 = plot(r.t(kd), slope(r.h(kd)), 'Color', cT, 'LineWidth', 2.2);  h2 = plot(t, r.apN(k), 'Color', cE, 'LineWidth', 1.2);  ylim([0 2]);
    ylabel(sprintf('%s''_{33}', AB));  legend([h1 h2], {sprintf('d%s_{33}/dh at the true gap', AB), sprintf('%s%s''_{33}', AB, hatc)}, 'Location', 'northoutside', 'Orientation', 'horizontal');
    axa = nexttile(tl, 3 * na + ia);  hold on;  box on;
    h1 = plot(t, r.th_tr(k, 2), 'Color', cT, 'LineWidth', 2.2);  plot(t, r.th_tr(k, 1), 'Color', cT, 'LineWidth', 2.2);
    h2 = plot(t, r.thy(k), 'Color', cE, 'LineWidth', 1.2);  h3 = plot(t, r.thx(k), 'Color', cG, 'LineWidth', 1.2);  ylim([-5 20]);
    ylabel(sprintf('%s  [\\circ]', th));  xlabel('Time (sec)');
    legend([h1 h2 h3], {sprintf('%s_y, %s_x', th, th), sprintf('%s%s_y', th, hatc), sprintf('%s%s_x', th, hatc)}, 'Location', 'northoutside', 'Orientation', 'horizontal');
    for a = [a0 axg axp axa]
        xline(a, tH, '--', 'Color', cGr, 'LineWidth', 1.3, 'HandleVisibility', 'off');
        set(a, 'XLim', [0 T], 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off');
    end
end
fn = fullfile(fo, 'fig_1028_hold_ap.png');  exportgraphics(f, fn, 'Resolution', 130);  close(f);  system(['open ' fn]);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
