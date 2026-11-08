% STATUS: SCRATCH -- 2026-11-01 (user): why seeds 3, 4 hit the wall on the 40 um/s whole bead for every theta_x P_fix >= 2 deg. PI default, theta_x P_fix 4 deg, rho 10, seeds 1-5. Same path in time, so finished seeds are overlaid in the 1.5 s before the failure.
%   Sanity: 3/5 finished, seeds 3, 4 end 17.8, 23.5 s. Console: contact point, angles, last-0.2 s a33 true / est, min eig, normal tracking error, commanded normal step; first time the failing seed departs from the median of the finished seeds in each signal (> 3 sd of the finished seeds over the same window). Figure fig_1101_bead_fail.png.
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.1;  Ts = 1 / 1612;  Xe = 13;  dy = 1;  nl = 27;  FS = 13;  cT = [0.85 0 0];  cE = [0 0.25 0.85];  cO = [0.7 0.7 0.7];  NS = 5;
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, ...
              'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 0.5, 'a_d', ad, ...
              'ap_q', true, 'wall', 'fillet', 'bead_r', 5, 'fillet_rho', 10, 'bead_start', [Xe; -Xe], 'ap_floor_law', true, 'raster_keep_y', true, ...
              'raster', [Xe dy nl], 'scan_ramp', 0.5, 'scan_len', 1e6, 'sP_fix_theta_deg', 2, 'sP_fix_thx_deg', 4, 'descent_T', 1, 'descent_v', 15 - hs * Rr, 'post_wait', [0 1], 'tag', 'bx', ...
              'du_d', 40 * Ts / Rr, 'T', 12 + 1.4 * nl * (2 * Xe + dy) / 40 + 1, 'kappa_state', true, 'kappa_from_scan', true, 'kappa_pfix', 0.3);
rs = cell(NS, 1);
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
pctRunOnAll warning('off', 'all');
parfor id = 1:NS
    o = base;  o.seeds = id;  R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{id} = struct('t', r.t, 'kend', r.kend, 'contact', r.contact, 'tsc', r.tsc, 'te', r.rs_tend, 'p', r.p, 'pd', r.pd, 'h', r.h, 'aN', r.aN, 'aN_tr', r.aN_tr, 'ap', r.apN, 'thx', r.thx, 'thy', r.thy, 'th_tr', r.th_tr, 'kap', r.kap(:), 'me', r.mineig, 'hd', r.hd_int, 'vF', r.vF, 'eI', r.eI(:));
end
save(fullfile(fo, 'temp_bead_fail.mat'), 'rs', '-v7.3');
fin = cellfun(@(r) isfinite(r.te) && ~r.contact, rs);  fprintf('finished seeds: %s\n', mat2str(find(fin)'));
for id = 1:NS
    r = rs{id};  b = find(~isfinite(r.aN(1:r.kend)) | ~isfinite(r.thx(1:r.kend)), 1);  ke = r.kend;  if ~isempty(b); ke = b - 1; end;  rs{id}.ke = ke;  cx = r.th_tr(:, 1) * pi / 180;  cy = r.th_tr(:, 2) * pi / 180;
    wt = [sin(cy) .* cos(cx), -sin(cx), cos(cy) .* cos(cx)];  rs{id}.dw = sum((r.pd - r.p) .* wt, 2);  rs{id}.ex = r.thx - r.th_tr(:, 1);  rs{id}.ey = r.thy - r.th_tr(:, 2);  rs{id}.dh = [0; diff(r.hd)] / Ts * Rr;
    fprintf('seed %d: contact %d, end %.2f s', id, r.contact, r.t(ke));  if ~fin(id); k2 = (ke - round(0.2 / Ts)):ke; fprintf(' | at (%.1f, %.1f, %.1f) um, true th (%.0f, %.0f) deg | last .2 s: true a33 %.3f, est %.3f, min eig %.3f, dw %+.0f nm, cmd normal rate %+.2f um/s, gap %.3f um', r.p(ke, :), r.th_tr(ke, :), mean(r.aN_tr(k2)), mean(r.aN(k2)), min(r.me(k2)), 1e3 * mean(rs{id}.dw(k2)), mean(rs{id}.dh(k2)), mean(r.h(k2)) * Rr); end;  fprintf('\n');
end
sg = {'aN_tr', 'aN', 'ex', 'ey', 'kap', 'ap'};  sn = {'true a33', 'est a33', 'theta_x err', 'theta_y err', 'kappa', 'a'' est'};  bad = find(~fin)';  W = round(1.5 / Ts);
f = figure('Color', 'w', 'Position', [20 20 800 * numel(bad) 1500]);  tl = tiledlayout(7, numel(bad), 'TileSpacing', 'compact', 'Padding', 'compact');
for j = 1:numel(bad)
    id = bad(j);  r = rs{id};  ke = r.ke;  k = (ke - W):ke;  k = k(:);  t = r.t(k) - r.t(ke);  ot = find(fin)';
    for s = 1:numel(sg)
        y = rs{id}.(sg{s})(k);  O = cell2mat(cellfun(@(c) c(:), arrayfun(@(q) rs{q}.(sg{s})(k), ot, 'UniformOutput', false), 'UniformOutput', false));  m = median(O, 2);  sd = std(O(:) - repelem(m, numel(ot)));  i0 = find(movmean(abs(y - m), 40) > 3 * sd, 1);
        tf = NaN;  if ~isempty(i0); tf = t(i0); end;  fprintf('seed %d %-12s departs %.2f s before the end\n', id, sn{s}, -tf);
    end
    rw = {'aN', 'gap', 'ex', 'ey', 'kap', 'ap', 'dw'};  yl = {'a_{33} / a_{nom}', 'gap  [\mum]', '\theta_x error [deg]', '\theta_y error [deg]', '\kappa estimate [1/R]', 'slope a''_{33}', 'normal tracking error [nm]'};  ax = gobjects(7, 1);
    for i = 1:7
        ax(i) = nexttile(tl, (i - 1) * numel(bad) + j);  hold on;  box on;
        for q = ot
            c = rs{q};  switch rw{i}; case 'aN'; v = c.aN_tr(k); case 'gap'; v = c.h(k) * Rr; case 'dw'; v = 1e3 * c.dw(k); otherwise; v = c.(rw{i})(k); end;  plot(t, movmean(v, 8), 'Color', cO, 'LineWidth', 0.7, 'HandleVisibility', 'off');
        end
        switch rw{i}
            case 'aN'; plot(t, r.aN_tr(k), 'Color', cT, 'LineWidth', 1.3, 'DisplayName', 'true');  plot(t, r.aN(k), 'Color', cE, 'LineWidth', 0.9, 'DisplayName', sprintf('estimate, seed %d', id));  yline(ad, 'k--', 'HandleVisibility', 'off');  ylim([0 0.25]);  legend('Location', 'northoutside', 'Orientation', 'horizontal');
            case 'gap'; plot(t, r.h(k) * Rr, 'Color', cT, 'LineWidth', 1.3);  yline(hs * Rr, 'k--');  yline(Rr, 'k-');  ylim([2.1 3]);
            case 'dw'; plot(t, movmean(1e3 * rs{id}.dw(k), 8), 'Color', cE, 'LineWidth', 1.0);  yline(0, 'k:');  ylim([-200 200]);
            otherwise; plot(t, movmean(rs{id}.(rw{i})(k), 8), 'Color', cE, 'LineWidth', 1.0);  yline(0, 'k:');  if i <= 4; ylim([-20 20]); end;  if i == 5; ylim([-1.2 1.2]); end;  if i == 6; ylim([-3 4]); end
        end
        ylabel(yl{i});  xlim([t(1) 0]);  set(ax(i), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.1);  set(ax(i).Toolbar, 'Visible', 'off');
    end
    xlabel(ax(7), 'time before the end  (sec)');
end
fn = fullfile(fo, 'fig_1101_bead_fail.png');  exportgraphics(f, fn, 'Resolution', 115);  close(f);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
