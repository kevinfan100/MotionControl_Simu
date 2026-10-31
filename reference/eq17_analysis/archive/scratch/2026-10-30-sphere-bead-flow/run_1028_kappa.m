% STATUS: SCRATCH -- 2026-10-28 (user test): curvature state kappa (slot 18, opts.kappa_state, constant, Q = 0, prior 0.2 / R, start 0):
%   theta-hat[k+1] = theta-hat[k] + delta-theta-hat[k] + kappa-hat M (theta_y: + kappa M_u / cos theta_x, theta_x: - kappa M_v), the
%   off-diagonal error rows get the kappa column -gap M. Settled flow (start 10 deg off the top / 15 um above the plane, theta-hat[0] = 0,
%   prior 10 deg, hold 10 s, regulation tau_a 2 s to a_d 0.05, law a' floor, slope P_fix law L, scan from 17 s). Cases: void sphere
%   (radius 15 um) raster half hemisphere 5 / 40 um/s, full turn 40 um/s; plane tilted 10 deg (theta_y) raster 5 / 40 um/s (negative
%   control: true kappa = 0). Arms: document version | + kappa | + kappa with half the angle P_fix. Seeds 1-3.
%   Figure: per case, gap (top) and kappa-hat (bottom, truth dashed) vs time since t_scan. Output fig_1028_kappa.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  tS = 17;  Ts = 1 / 1612;  FS = 13;  hstar = 2.37;  rb = 15;  Lturn = 2 * pi * (rb + hstar);
cases = {'sphere raster 5 um/s', 'sphere raster 40 um/s', 'sphere full turn 40 um/s', 'plane 10 deg raster 5 um/s', 'plane 10 deg raster 40 um/s'};
SP = [5 40 40 5 40];  PF = [2 4 4 2 4];  TSC = [40 6 Lturn / 40 40 6];  arms = {'document', '+ kappa', '+ kappa, angle P_fix / 2'};
[IC, IA, ID] = ndgrid(1:5, 1:3, 1:3);  rs = cell(size(IC));
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
parfor q = 1:numel(IC)
    ic = IC(q);  o = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, ...
        'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 2, 'a_d', 0.05, ...
        't_scan', tS, 'ap_q', true, 'q_theta_from', tS, 'ap_floor_law', true, 'scan_dir', [1 0], 'ap_pfix_law', true, ...
        'du_d', SP(ic) * Ts / Rr, 'sP_fix_theta_deg', PF(ic), 'T', tS + TSC(ic) + 0.2, 'seeds', ID(q), 'tag', 'kp');
    if ic <= 3; o.wall = 'hemi';  o.hemi_r = rb;  o.hemi_void = true;  o.hemi_start_deg = 10; else; o.theta_y = 10; end
    if ic == 3; o.scan_plane_n = [0 1 0];  o.scan_len = Lturn; else; o.raster = [12 2 7];  o.scan_len = 1e9; end
    if IA(q) >= 2; o.kappa_state = true; end
    if IA(q) == 3; o.sP_fix_theta_deg = PF(ic) / 2; end
    R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t, 'kend', r.kend, 'h', r.h, 'aN', r.aN, 'aN_tr', r.aN_tr, 'thx', r.thx, 'thy', r.thy, 'th_tr', r.th_tr, 'kap', [r.kap(:); NaN(numel(r.t) - numel(r.kap), 1)], 'tend', r.rs_tend, 'contact', r.contact);
end
wn = @(tx, ty) [sind(ty) .* cosd(tx), -sind(tx), cosd(ty) .* cosd(tx)];
f = figure('Color', 'w', 'Position', [20 20 2600 1000]);  tl = tiledlayout(2, 5, 'TileSpacing', 'compact', 'Padding', 'compact');
cl = {[0.6 0.6 0.6], [0 0.25 0.85], [0.85 0.4 0]};  ls = {'-', '-', '-'};
for ic = 1:5
    ag = nexttile(tl, ic);  hold on;  box on;  ak = nexttile(tl, 5 + ic);  hold on;  box on;
    ktrue = 0;  if ic <= 3; ktrue = Rr / (rb + hstar); end
    for ia = 1:3
        for id = 1:3
            r = rs{ic, ia, id};  te = tS + TSC(ic);  if ~isnan(r.tend); te = r.tend; end
            ks = find(r.t >= tS & r.t <= te & (1:numel(r.t))' <= r.kend);  gap = r.h(ks) * Rr;  nfull = round((te - tS) / Ts);
            bad = find(gap > 5 | isnan(r.aN(ks)), 1);  if isempty(bad); kon = numel(ks); else; kon = bad - 1; end
            kk = ks(1:max(kon, 1));  if r.kend < find(r.t >= te, 1) - 1; kon = min(kon, numel(ks) - 1); end
            ew = acosd(min(1, sum(wn(r.thx(kk), r.thy(kk)) .* wn(r.th_tr(kk, 1), r.th_tr(kk, 2)), 2)));
            fprintf('%-28s %-26s seed %d: on the surface %.2f | gap mean %.2f max %.2f um | w-hat err mean %.1f deg | kappa-hat end %.3f (true %.3f) | contact %d\n', ...
                    cases{ic}, arms{ia}, id, kon / max(nfull, 1), mean(gap(1:max(kon, 1))), max(gap(1:max(kon, 1))), mean(ew), r.kap(kk(end)), ktrue, r.contact);
            hv = 'off';  if id == 1; hv = 'on'; end
            plot(ag, r.t(ks) - tS, min(gap, 6), 'Color', cl{ia}, 'LineWidth', 1, 'DisplayName', arms{ia}, 'HandleVisibility', hv);
            if ia >= 2; plot(ak, r.t(ks) - tS, r.kap(ks), 'Color', cl{ia}, 'LineWidth', 1, 'HandleVisibility', 'off'); end
        end
    end
    yline(ag, hstar, 'k--', 'LineWidth', 1.3, 'HandleVisibility', 'off');  ylim(ag, [2 6]);  title(ag, cases{ic});  if ic == 1; ylabel(ag, 'gap  [\mum]');  legend(ag, 'Location', 'northwest'); end
    yline(ak, ktrue, 'r--', 'LineWidth', 1.6);  ylim(ak, [-0.3 0.4]);  xlabel(ak, 'time since t_{scan}  (sec)');  if ic == 1; ylabel(ak, '\kappa-hat  [1/R]  (true red)'); end
    for a = [ag ak]; set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off'); end
end
fn = fullfile(fo, 'fig_1028_kappa.png');  exportgraphics(f, fn, 'Resolution', 130);  close(f);  system(['open ' fn]);
