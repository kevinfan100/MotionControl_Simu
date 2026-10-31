% STATUS: SCRATCH -- 2026-10-28 (user, option 1): regulation without a'-hat -- the slope in Delta h_d = (Ts/tau_a)(a_d - a-hat_33)/slope is
%   the plane law (17/18)(1 - a-hat_33)^2 (1/b at the middle of [8/9, 1]; opts.reg_slope_law) instead of a'-hat_33. Settled flow otherwise
%   (void hemisphere / sphere radius 15 um, start 10 deg off the top, hold 10 s, regulation tau_a 2 s to a_d 0.05 from 10 s, scan from 17 s,
%   slope P_fix law L). Cases: raster over half of the hemisphere 5 um/s (angle P_fix 2) and 40 um/s (4); full turn of the sphere 40 um/s (4).
%   Arms: regulation with a'-hat | with the law slope. Seeds 1-3. Figure: gap vs time since t_scan, one panel per case, both arms, 3 seeds.
%   Output test_results/temp_3d_wsonly/fig_1028_reg_law_gap.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  rb = 15;  tH = 10;  tS = 17;  ph0 = 10;  Ts = 1 / 1612;  FS = 15;
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  Lturn = 2 * pi * (rb + hs * Rr);
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, ...
              'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', tH, 'tau_a', 2, 'a_d', ad, ...
              't_scan', tS, 'ap_q', true, 'q_theta_from', tS, 'wall', 'hemi', 'hemi_r', rb, 'hemi_void', true, 'ap_floor_law', true, ...
              'scan_dir', [1 0], 'hemi_start_deg', ph0, 'ap_pfix_law', true, 'tag', 'rl');
cases = {'raster 5 um/s', 'raster 40 um/s', 'full turn 40 um/s'};  arms = {'regulation with a''-hat', 'regulation with the law slope'};
[IC, IA, ID] = ndgrid(1:3, 1:2, 1:3);  rs = cell(size(IC));
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
parfor q = 1:numel(IC)
    o = base;  o.seeds = ID(q);  if IA(q) == 2; o.reg_slope_law = true; end
    switch IC(q)
        case 1; sp = 5;  o.sP_fix_theta_deg = 2;  o.raster = [12 2 7];  o.scan_len = 1e9;  Tsc = 40;
        case 2; sp = 40; o.sP_fix_theta_deg = 4;  o.raster = [12 2 7];  o.scan_len = 1e9;  Tsc = 6;
        case 3; sp = 40; o.sP_fix_theta_deg = 4;  o.scan_plane_n = [0 1 0];  o.scan_len = Lturn;  Tsc = Lturn / sp;
    end
    o.du_d = sp * Ts / Rr;  o.T = tS + Tsc + 0.2;  R = run_t3_xyz_fixed(o);  rs{q} = R.runs{1};
end
wn = @(tx, ty) [sind(ty) .* cosd(tx), -sind(tx), cosd(ty) .* cosd(tx)];
f = figure('Color', 'w', 'Position', [30 30 2100 1000]);  tl = tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
cl = {[0 0.25 0.85], [0.85 0.4 0], [0 0.55 0.2]};
for ic = 1:3
    for ia = 1:2
        ax = nexttile(tl, (ia - 1) * 3 + ic);  hold on;  box on;
        for id = 1:3
            r = rs{ic, ia, id};  te = r.t(r.kend);  if ~isnan(r.rs_tend); te = r.rs_tend; end
            ks = find(r.t >= tS & r.t <= te & (1:numel(r.t))' <= r.kend);  gap = r.h(ks) * Rr;
            bad = find(gap > 5 | isnan(r.aN(ks)), 1);  if isempty(bad); kon = numel(ks); else; kon = bad - 1; end
            kk = ks(1:max(kon, 1));  ew = acosd(min(1, sum(wn(r.thx(kk), r.thy(kk)) .* wn(r.th_tr(kk, 1), r.th_tr(kk, 2)), 2)));
            fprintf('%-18s %-30s seed %d: on the sphere %.2f | gap %.2f-%.2f um (mean %.2f) | w-hat err mean %.1f deg | rms a33 err %.3f\n', cases{ic}, arms{ia}, id, ...
                    kon / numel(ks), min(gap(1:max(kon, 1))), max(gap(1:max(kon, 1))), mean(gap(1:max(kon, 1))), mean(ew), rms(r.aN(kk) - r.aN_tr(kk)));
            plot(r.t(ks) - tS, min(gap, 6), 'Color', cl{id}, 'LineWidth', 1.1, 'DisplayName', sprintf('seed %d', id));
        end
        yline(hs * Rr, 'k--', 'LineWidth', 1.4, 'HandleVisibility', 'off');  ylim([2 6]);
        if ic == 1; ylabel(sprintf('%s\ngap  [\\mum]', arms{ia})); end
        if ia == 1; title(cases{ic}); end;  if ia == 2; xlabel('time since t_{scan}  (sec)'); end
        if ic == 1 && ia == 1; legend('Location', 'northwest'); end
        set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(ax.Toolbar, 'Visible', 'off');
    end
end
fn = fullfile(fo, 'fig_1028_reg_law_gap.png');  exportgraphics(f, fn, 'Resolution', 140);  close(f);  system(['open ' fn]);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
