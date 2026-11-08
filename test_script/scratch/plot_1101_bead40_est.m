% STATUS: SCRATCH -- 2026-11-01 (user): estimates against truth on the whole-bead raster at 40 um/s after the rotation fix, same settings as arm E of run_1029_fillet_opts.m, seeds 1-3. Rows: a_33, a_11, a'_33, angle error,
%   gap, kappa-hat, wall normal. Columns: seeds. Two figures: whole scan (fig_1101_bead40_est_all.png) and last 2 s before the end (fig_1101_bead40_est_end.png). Sanity: end times 27.1 / 15.9 / 24.2 s as run_1101_bead_v2.
%   Console: mean true - est a_33, angle error, a'/true, |a33 - a_d| whole scan and last 2 s.
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.1;  Ts = 1 / 1612;  Xe = 13;  dy = 1;  nl = 27;  FS = 13;  cT = [0.85 0 0];  cE = [0 0.25 0.85];  cGr = [0.5 0.5 0.5];  NS = 3;
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, ...
              'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 0.5, 'a_d', ad, ...
              'ap_q', true, 'wall', 'fillet', 'bead_r', 5, 'fillet_rho', 10, 'bead_start', [Xe; -Xe], 'ap_floor_law', true, 'raster_keep_y', true, ...
              'raster', [Xe dy nl], 'scan_ramp', 0.5, 'scan_len', 1e6, 'sP_fix_theta_deg', 2, 'descent_T', 1, 'descent_v', 15 - hs * Rr, 'post_wait', [0 1], 'tag', 'be', ...
              'du_d', 40 * Ts / Rr, 'T', 12 + 1.4 * nl * (2 * Xe + dy) / 40 + 1, 'kappa_state', true, 'kappa_from_scan', true, 'kappa_pfix', 0.3);
rs = cell(NS, 1);
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
parfor id = 1:NS
    o = base;  o.seeds = id;  R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{id} = struct('t', r.t, 'kend', r.kend, 'contact', r.contact, 'tsc', r.tsc, 'p', r.p, 'h', r.h, 'aN', r.aN, 'aN_tr', r.aN_tr, 'aT', r.aT, 'aT_tr', r.aT_tr, 'ap', r.apN, 'kap', r.kap(:), 'thx', r.thx, 'thy', r.thy, 'th_tr', r.th_tr);
end
wr = @(d) mod(d + 180, 360) - 180;
for id = 1:NS
    r = rs{id};  ok = find(isfinite(r.aN(1:r.kend)) & isfinite(r.h(1:r.kend)) & isfinite(r.ap(1:r.kend)));  ke = ok(end);  rs{id}.ke = ke;
    rs{id}.tr = local_slope(r.h(ok));  rs{id}.ok = ok;
    for w = 1:2
        if w == 1; g = ok(r.t(ok) >= r.tsc + 0.5); lb = 'scan'; else; g = ok(r.t(ok) >= r.t(ke) - 2); lb = 'last 2 s'; end
        tg = local_slope(r.h(g));  ex = wr(r.thx(g) - r.th_tr(g, 1));  ey = wr(r.thy(g) - r.th_tr(g, 2));
        fprintf('seed %d %-8s end t %.1f contact %d | true a33 %.3f est %.3f (true - est %+.4f) | |a33-a_d| %.3f | a11 true - est %+.4f | angle err rms x %.1f y %.1f deg | a''/true median %.2f, a''<0 %.2f | gap %.2f-%.2f um\n', id, lb, r.t(ke), r.contact, mean(r.aN_tr(g)), mean(r.aN(g)), mean(r.aN_tr(g) - r.aN(g)), mean(abs(r.aN_tr(g) - ad)), mean(r.aT_tr(g) - r.aT(g)), sqrt(mean(ex.^2)), sqrt(mean(ey.^2)), median(r.ap(g) ./ tg), mean(r.ap(g) < 0), min(r.h(g)) * Rr, max(r.h(g)) * Rr);
    end
end
for w = 1:2
    f = figure('Color', 'w', 'Position', [20 20 680 * NS 1500]);  tl = tiledlayout(7, NS, 'TileSpacing', 'compact', 'Padding', 'compact');
    for id = 1:NS
        r = rs{id};  ok = r.ok;  ke = r.ke;  if w == 1; k = ok(r.t(ok) >= r.tsc - 0.5); ds = 10; t = r.t(k) - r.tsc; xl = 'time since t_{scan}  (sec)'; else; k = ok(r.t(ok) >= r.t(ke) - 2); ds = 2; t = r.t(k) - r.t(ke); xl = 'time before the end  (sec)'; end
        ix = 1:ds:numel(k);  kd = k(ix);  t = t(ix);  tr = r.tr(ismember(ok, kd));  ax = gobjects(7, 1);
        ax(1) = nexttile(tl, id);  hold on;  plot(t, r.aN_tr(kd), 'Color', cT, 'LineWidth', 1.2, 'DisplayName', 'true');  plot(t, r.aN(kd), 'Color', cE, 'LineWidth', 0.8, 'DisplayName', sprintf('estimate, seed %d', id));  yline(ad, 'k--', 'HandleVisibility', 'off');  ylim([0 0.25]);  ylabel('a_{33} / a_{nom}');  legend('Location', 'northoutside', 'Orientation', 'horizontal');
        ax(2) = nexttile(tl, NS + id);  hold on;  plot(t, r.aT_tr(kd), 'Color', cT, 'LineWidth', 1.2);  plot(t, r.aT(kd), 'Color', cE, 'LineWidth', 0.8);  ylim([0.2 0.7]);  ylabel('a_{11} / a_{nom}');
        ax(3) = nexttile(tl, 2 * NS + id);  hold on;  plot(t, tr, 'Color', cT, 'LineWidth', 1.2);  plot(t, r.ap(kd), 'Color', cE, 'LineWidth', 0.8);  ylim([-3 4]);  ylabel('slope a''_{33}');
        ax(4) = nexttile(tl, 3 * NS + id);  hold on;  plot(t, wr(r.thy(kd) - r.th_tr(kd, 2)), 'Color', cE, 'LineWidth', 0.8, 'DisplayName', '\theta_y error');  plot(t, wr(r.thx(kd) - r.th_tr(kd, 1)), 'Color', [0 0.55 0.2], 'LineWidth', 0.8, 'DisplayName', '\theta_x error');  yline(0, 'k:', 'HandleVisibility', 'off');  ylim([-15 15]);  ylabel('angle error [deg]');  if id == 1; legend('Location', 'northoutside', 'Orientation', 'horizontal'); end
        ax(5) = nexttile(tl, 4 * NS + id);  hold on;  plot(t, r.h(kd) * Rr, 'Color', cT, 'LineWidth', 1.2);  yline(hs * Rr, 'k--');  yline(Rr, 'k-');  ylim([2 4]);  ylabel('center gap [\mum]');
        ax(6) = nexttile(tl, 5 * NS + id);  hold on;  plot(t, r.kap(kd), 'Color', cE, 'LineWidth', 0.8);  yline(0, 'k:');  ylim([-1 1]);  ylabel('\kappa estimate [1/R]');
        ax(7) = nexttile(tl, 6 * NS + id);  hold on;  plot(t, r.th_tr(kd, 2), 'Color', cT, 'LineWidth', 1.2);  ylim([-90 90]);  ylabel('true \theta_y [deg]');  xlabel(xl);
        for i = 1:7; box(ax(i), 'on');  xlim(ax(i), [t(1) t(end)]);  if w == 1; xline(ax(i), 0, '-.', 'Color', cGr, 'LineWidth', 1.2, 'HandleVisibility', 'off'); end;  set(ax(i), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);  set(ax(i).Toolbar, 'Visible', 'off'); end
    end
    nmf = {'all', 'end'};  fn = fullfile(fo, ['fig_1101_bead40_est_' nmf{w} '.png']);  exportgraphics(f, fn, 'Resolution', 110);  close(f);  system(['open ' fn]);
end

function s = local_slope(hb)
    s = arrayfun(@(h) (1 / local_cp(max(h, 1.0003) + 1e-4) - 1 / local_cp(max(h, 1.0003) - 1e-4)) / 2e-4, hb);
end
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
