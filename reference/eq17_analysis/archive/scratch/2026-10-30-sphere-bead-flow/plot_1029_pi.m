% STATUS: SCRATCH -- 2026-10-29 (user): P vs PI regulation (opts.reg_pi: integral of a_d - a-hat_33 from the scan start, tau_i = 4 tau_a,
%   tau_a 2 s) on the full turn of the void sphere (radius 15 um) with the curvature state, flow hold 10 s, constant-speed descent 10-15 s,
%   scan + regulation from 15 s. 5 um/s (angle P_fix 1 deg) and 40 um/s (2 deg), seeds 1-3; seed 1 drawn.
%   Figure: rows = 5 | 40 um/s, columns = P | PI; each panel a_33 (true red), a-hat_33 (blue), a_d (dashed) vs time since the scan start.
%   Output test_results/temp_3d_wsonly/fig_1029_pi.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  rb = 15;  tH = 10;  Td = 5;  tS = tH + Td;  Ts = 1 / 1612;  FS = 15;  hatc = char(770);  AB = char(257);
cT = [0.85 0 0];  cE = [0 0.25 0.85];
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  Lturn = 2 * pi * (rb + hs * Rr);  vd = (15 - hs * Rr) / Td;
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, ...
              'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', tH, 'tau_a', 2, 'a_d', ad, ...
              'ap_q', true, 'q_theta_from', tS, 'wall', 'hemi', 'hemi_r', rb, 'hemi_void', true, 'ap_floor_law', true, 'scan_dir', [1 0], ...
              'hemi_start_deg', 10, 'scan_plane_n', [0 1 0], 'ap_pfix_law', true, 'kappa_state', true, 'descent_v', vd, 'descent_T', Td, 'scan_len', Lturn, 'tag', 'pi');
SP = [5 40];  PF = [1 2];  arm = {'P', 'PI'};
[IS, IA, ID] = ndgrid(1:2, 1:2, 1:3);  rs = cell(size(IS));
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
parfor q = 1:numel(IS)
    o = base;  sp = SP(IS(q));  o.du_d = sp * Ts / Rr;  o.sP_fix_theta_deg = PF(IS(q));  o.T = tS + Lturn / sp + 0.2;  o.seeds = ID(q);
    if IA(q) == 2; o.reg_pi = true; end
    R = run_t3_xyz_fixed(o);  rs{q} = R.runs{1};
end
for is = 1:2
    for ia = 1:2
        M = zeros(3, 5);
        for id = 1:3
            r = rs{is, ia, id};  ks = find(r.t >= tS & r.t <= tS + Lturn / SP(is) & (1:numel(r.t))' <= r.kend);  k2 = ks(ks >= ks(1) + round(0.25 * numel(ks)));
            M(id, :) = [r.contact, mean(r.aN(k2)) - ad, mean(r.aN_tr(k2)) - ad, mean(r.h(ks)) * Rr, max(r.h(ks)) * Rr];
        end
        fprintf('%2d um/s %-3s: contact %d | after the first quarter: control offset a-hat - a_d %s | true a33 - a_d %s | gap mean %s max %s um\n', SP(is), arm{ia}, sum(M(:, 1)), ...
                mat2str(round(M(:, 2)', 3)), mat2str(round(M(:, 3)', 3)), mat2str(round(M(:, 4)', 2)), mat2str(round(M(:, 5)', 2)));
    end
end
f = figure('Color', 'w', 'Position', [30 30 1900 1000]);  tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for is = 1:2
    for ia = 1:2
        r = rs{is, ia, 1};  ks = find(r.t >= tS & r.t <= tS + Lturn / SP(is) & (1:numel(r.t))' <= r.kend);  t = r.t(ks) - tS;
        ax = nexttile(tl);  hold on;  box on;
        h1 = plot(t, r.aN_tr(ks), 'Color', cT, 'LineWidth', 2.2);  h2 = plot(t, r.aN(ks), 'Color', cE, 'LineWidth', 1.3);  h3 = yline(ad, 'k--', 'LineWidth', 1.6);
        ylim([0 0.3]);  xlim([0 t(end)]);  ylabel(sprintf('%s_{33}', AB));  title(sprintf('%d um/s, %s regulation, seed 1', SP(is), arm{ia}));
        if is == 2; xlabel('time since t_{scan}  (sec)'); end
        if is == 1 && ia == 1; legend([h1 h2 h3], {sprintf('%s_{33}', AB), sprintf('%s%s_{33}', AB, hatc), sprintf('%s_d = %g', AB, ad)}, 'Location', 'northeast'); end
        set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(ax.Toolbar, 'Visible', 'off');
    end
end
fn = fullfile(fo, 'fig_1029_pi.png');  exportgraphics(f, fn, 'Resolution', 140);  close(f);  system(['open ' fn]);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
