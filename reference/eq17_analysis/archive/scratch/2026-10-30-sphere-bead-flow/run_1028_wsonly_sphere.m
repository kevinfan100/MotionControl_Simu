% STATUS: SCRATCH -- 2026-10-28 (user): the earlier ws-only three-axis estimator (run_t2_3d_reg, 1015_estimator_3d_wsonly.tex: angles as
%   Kalman states, undivided angle rows, gains start at 1) on the same sphere scans as the document version (run_t3_xyz_fixed form 'local').
%   Void sphere radius 15 um (bead_void), start 10 deg off the top 15 um up, theta-hat[0] = 0, angle prior 10 deg, hold 0-10 s, regulation
%   tau_a 2 s to a_d 0.05 (law a' floor in both), scan from 17 s with angle q = P_fix^2 I from t_scan. Raster over half of the hemisphere
%   (lines along u turning at x = +-12 um, steps 2 um along v, 7 lines) at 5 um/s (P_fix 2 deg) and 40 um/s (4 deg). Seeds 1-3.
%   Console: fraction of the raster on the sphere (gap < 5 um, no NaN), gap, w-hat error, rms a33 error. Figure: gap vs time since t_scan.
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  tS = 17;  Ts = 1 / 1612;  FS = 15;  hstar = 2.37;
SP = [5 40];  PF = [2 4];  TSC = [40 6];  arms = {'document version (local)', 'ws-only (angle as state)'};
[IS, IA, ID] = ndgrid(1:2, 1:2, 1:3);  rs = cell(size(IS));
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
parfor q = 1:numel(IS)
    sp = SP(IS(q));  pf = PF(IS(q));  T = tS + TSC(IS(q)) + 0.2;
    if IA(q) == 1
        o = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, ...
                   'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 2, 'a_d', 0.05, ...
                   't_scan', tS, 'ap_q', true, 'q_theta_from', tS, 'wall', 'hemi', 'hemi_r', 15, 'hemi_void', true, 'ap_floor_law', true, ...
                   'scan_dir', [1 0], 'hemi_start_deg', 10, 'ap_pfix_law', true, 'raster', [12 2 7], 'scan_len', 1e9, ...
                   'du_d', sp * Ts / Rr, 'sP_fix_theta_deg', pf, 'T', T, 'seeds', ID(q), 'tag', 'wd');
        R = run_t3_xyz_fixed(o);  r = R.runs{1};
        rs{q} = struct('t', r.t, 'kend', r.kend, 'h', r.h, 'aN', r.aN, 'aN_tr', r.aN_tr, 'thx', r.thx, 'thy', r.thy, 'thx_t', r.th_tr(:, 1), 'thy_t', r.th_tr(:, 2), 'tend', r.rs_tend);
    else
        o = struct('wall', 'bead', 'bead_r', 15, 'bead_void', true, 'start_deg', 10, 'h0', 15 / Rr, 'a_d', 0.05, 'tau_a', 2, 'hold_until', 10, ...
                   't_scan', tS, 'T_scan', T - tS, 'du_d', sp * Ts / Rr, 'raster_x', [12 2 7], 'sP_theta_deg', 10, 'th0_deg', [0 0], ...
                   'q_theta', 'pfix', 'sP_fix_theta_deg', pf, 'q_theta_from', tS, 'ap_floor_law', true, 'seeds', ID(q), 'save', false, 'tag', 'ws');
        R = run_t2_3d_reg(o);  r = R.runs{1};
        rs{q} = struct('t', r.t, 'kend', r.kend, 'h', r.h, 'aN', r.aN, 'aN_tr', r.aN_true, 'thx', r.thx, 'thy', r.thy, 'thx_t', r.thx_true, 'thy_t', r.thy_true, 'tend', r.rs_tend);
    end
end
wn = @(tx, ty) [sind(ty) .* cosd(tx), -sind(tx), cosd(ty) .* cosd(tx)];
f = figure('Color', 'w', 'Position', [30 30 1900 1000]);  tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');  cl = {[0 0.25 0.85], [0.85 0.4 0], [0 0.55 0.2]};
for is = 1:2
    for ia = 1:2
        ax = nexttile(tl, (ia - 1) * 2 + is);  hold on;  box on;
        for id = 1:3
            r = rs{is, ia, id};  te = tS + TSC(is);  if ~isnan(r.tend); te = r.tend; end
            ks = find(r.t >= tS & r.t <= te & (1:numel(r.t))' <= r.kend);  gap = r.h(ks) * Rr;  nfull = round((te - tS) / Ts);
            bad = find(gap > 5 | isnan(r.aN(ks)), 1);  if isempty(bad); kon = numel(ks); else; kon = bad - 1; end
            kk = ks(1:max(kon, 1));  ew = acosd(min(1, sum(wn(r.thx(kk), r.thy(kk)) .* wn(r.thx_t(kk), r.thy_t(kk)), 2)));
            fprintf('%2d um/s %-27s seed %d: raster done %d | on the sphere %.2f of %.1f s | gap %.2f-%.2f um (mean %.2f) | w-hat err mean %.1f max %.1f deg | rms a33 err %.3f\n', ...
                    SP(is), arms{ia}, id, ~isnan(r.tend), kon / max(nfull, numel(ks)), te - tS, min(gap(1:max(kon, 1))), max(gap(1:max(kon, 1))), mean(gap(1:max(kon, 1))), mean(ew), max(ew), rms(r.aN(kk) - r.aN_tr(kk)));
            plot(r.t(ks) - tS, min(gap, 6), 'Color', cl{id}, 'LineWidth', 1.1, 'DisplayName', sprintf('seed %d', id));
        end
        yline(hstar, 'k--', 'LineWidth', 1.4, 'HandleVisibility', 'off');  ylim([2 6]);  ylabel('gap  [\mum]');
        title(sprintf('%s, raster %d um/s', arms{ia}, SP(is)));  if ia == 2; xlabel('time since t_{scan}  (sec)'); end
        if is == 1 && ia == 1; legend('Location', 'northwest'); end
        set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(ax.Toolbar, 'Visible', 'off');
    end
end
fn = fullfile(fo, 'fig_1028_wsonly_vs_doc_raster.png');  exportgraphics(f, fn, 'Resolution', 140);  close(f);  system(['open ' fn]);
