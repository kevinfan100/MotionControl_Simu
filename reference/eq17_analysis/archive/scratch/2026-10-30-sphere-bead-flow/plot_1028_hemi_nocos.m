% STATUS: SCRATCH -- 2026-10-28 (user): what dropping cos theta-hat_x in the theta_y step does on the hemisphere scan.
%   delta-theta-hat_y = -a-hat_13 / ((a-hat_11 - a-hat_33) cos theta-hat_x) (document) vs the same without cos (opts.no_cos).
%   Meridian scan (opts.scan_meridian), void hemisphere, law-derived a' floor, a_d = 0.05; 5 um/s with P_fix 3 deg, 40 um/s with P_fix 4 deg;
%   seeds 1-3 (console), seed 1 drawn: rows = speed, columns = with cos | without cos, angles (truth red).
%   Output test_results/temp_3d_wsonly/fig_1028_hemi_nocos.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  rb = 15;  tH = 10;  tS = 17;  FS = 15;  hatc = char(770);  th = char(952);  cT = [0.85 0 0];  cE = [0 0.25 0.85];  cG = [0 0.55 0.2];
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  Ts = 1 / 1612;  Lscan = pi / 2 * (rb + hs * Rr);
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 5, ...
              'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', tH, 'tau_a', 2, 'a_d', ad, ...
              't_scan', tS, 'ap_q', true, 'q_theta_from', tS, 'wall', 'hemi', 'hemi_r', rb, 'hemi_void', true, 'ap_floor_law', true, ...
              'scan_meridian', true, 'scan_dir', [1 1], 'scan_len', Lscan, 'tag', 'nc');
spd = [5 40];  pf = [3 4];  seeds = 1:3;  [IS, IC, ID] = ndgrid(1:2, 1:2, 1:3);  rs = cell(size(IS));
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
parfor q = 1:numel(IS)
    o = base;  o.du_d = spd(IS(q)) * Ts / Rr;  o.T = tS + Lscan / spd(IS(q)) + 0.5;  o.sP_fix_theta_deg = pf(IS(q));  o.no_cos = IC(q) == 2;  o.seeds = seeds(ID(q));
    R = run_t3_xyz_fixed(o);  rs{q} = R.runs{1};
end
arm = {'with cos', 'without cos'};
for is = 1:2
    for ic = 1:2
        st = zeros(3, 6);
        for id = 1:3
            r = rs{is, ic, id};  Tsc = Lscan / spd(is);  k = find(r.t >= tS & r.t <= tS + Tsc & (1:numel(r.t))' <= r.kend);  m = k(end);
            st(id, :) = [r.contact, mean(abs(r.thy(k) - r.th_tr(k, 2))), mean(abs(r.thx(k) - r.th_tr(k, 1))), r.th_tr(m, 2) - r.thy(m), max(r.h(k)) * Rr, atan2d(norm(r.p(m, 1:2)), r.p(m, 3))];
        end
        fprintf('%2d um/s %-11s: contact %d | mean |e theta_y| %s | mean |e theta_x| %s | theta_y lag at the end %s deg | max gap %s um | end polar %s deg\n', spd(is), arm{ic}, ...
                sum(st(:, 1)), mat2str(round(st(:, 2)', 1)), mat2str(round(st(:, 3)', 1)), mat2str(round(st(:, 4)', 1)), mat2str(round(st(:, 5)', 2)), mat2str(round(st(:, 6)', 1)));
    end
end
f = figure('Color', 'w', 'Position', [40 40 1900 1100]);  tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for is = 1:2
    for ic = 1:2
        r = rs{is, ic, 1};  Tsc = Lscan / spd(is);  k = find(r.t >= tS & r.t <= tS + Tsc & (1:numel(r.t))' <= r.kend);  t = r.t(k) - tS;
        ax = nexttile(tl);  hold on;  box on;
        h1 = plot(t, r.th_tr(k, 2), 'Color', cT, 'LineWidth', 2.4);  plot(t, r.th_tr(k, 1), 'Color', cT, 'LineWidth', 2.4);
        h2 = plot(t, r.thy(k), 'Color', cE, 'LineWidth', 1.5);  h3 = plot(t, r.thx(k), 'Color', cG, 'LineWidth', 1.5);
        ylim([-60 100]);  xlim([0 t(end)]);  ylabel(sprintf('%s  [\\circ]', th));  if is == 2; xlabel('time since t_{scan}  (sec)'); end
        title(sprintf('%d um/s, P_{fix} %d deg, %s, seed 1', spd(is), pf(is), arm{ic}));
        legend([h1 h2 h3], {sprintf('%s_y, %s_x', th, th), sprintf('%s%s_y', th, hatc), sprintf('%s%s_x', th, hatc)}, 'Location', 'northwest');
        set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(ax.Toolbar, 'Visible', 'off');
    end
end
fn = fullfile(fo, 'fig_1028_hemi_nocos.png');  exportgraphics(f, fn, 'Resolution', 150);  close(f);  system(['open ' fn]);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
