% STATUS: SCRATCH -- 2026-11-01 (user): common scoreboard of a'-hat_33 for every arm; judge by overall accuracy and a' < 0 (feedback memory 11-01); target = exaggerated errors, a'/true outside 0.1-1.9 (2 sqrt P_fix). 40 um/s unless named, 5 seeds.
%   Arms: C (angle known, no off-diagonal P) | plane P alive 10 | plane P alive 40 | plane V-hat moving 40 | sphere as is 40 | sphere with cheat_w 40. Segments: hold (t_scan - 0.9 .. 0), onset (+0.15 .. +0.5), scan (+0.5 .. +3).
%   Per segment: a'-hat / true (median per seed: median of seeds and range), rms(a'-hat / true - 1), honesty mean(e^2 / P(a')), true - est a33, |a33 - a_d|, fraction a' < 0.
%   Diverging seeds stop 0.5 s before the first non-finite value; counted. Sanity: C |a33 - a_d| 0.007, P alive 40 median 0.52 (scan, from run_1031), true slope printed. Figure fig_1101_scoreboard.png (all seeds thin, median thick).
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  rb = 15;  Ts = 1 / 1612;  tS = 12;  FS = 14;  cE = [0 0.25 0.85];  cGr = [0.5 0.5 0.5];
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  ap0 = (1 / local_cp(hs + 1e-4) - 1 / local_cp(hs - 1e-4)) / 2e-4;  fprintf('true slope at a_d: %.3f, h-bar %.4f\n', ap0, hs);
base = struct('theta_x', 0, 'theta_y', 10, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 10], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'gap_gate', 2, ...
              'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 0.5, 'a_d', ad, 'ap_q', true, 'ap_floor_law', true, 'scan_dir', [1 0], ...
              'kappa_state', false, 'scan_ramp', 0.5, 'scan_len', 1e6, 'du_d', 40 * Ts / Rr, 'sP_fix_theta_deg', 2, 'descent_T', 1, 'descent_v', 15 - hs * Rr, ...
              'post_wait', [0 1], 'T', 15, 'tag', 'sb');
arm = {'C, 40', 'P alive, 10', 'P alive, 40', 'V-hat moving, 40', 'sphere, 40', 'sphere true normal, 40'};  NA = numel(arm);  NS = 5;  [IA, ID] = ndgrid(1:NA, 1:NS);  rs = cell(size(IA));
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
pctRunOnAll warning('off', 'all');
parfor q = 1:numel(IA)
    o = base;  o.seeds = ID(q);
    switch IA(q)
        case 1; o.freeze_angle = true;  o.sP_theta_deg = 1e-3;  o.q_theta_off = true;
        case 2; o.freeze_angle = true;  o.du_d = 10 * Ts / Rr;
        case 3; o.freeze_angle = true;
        case {5, 6}; if IA(q) == 6; o.cheat_w = true; end;  o.theta_y = 0;  o.th0_deg = [0 0];  o.wall = 'hemi';  o.hemi_r = rb;  o.hemi_void = true;  o.hemi_start_deg = 10;  o.scan_plane_n = [0 1 0];  o.kappa_state = true;  o.kappa_from_scan = true;
    end
    R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t, 'kend', r.kend, 'ap', r.apN, 'h', r.h, 'aN', r.aN, 'aN_tr', r.aN_tr, 'Pap', r.Pap(:));
end
wn = {'hold', 'onset', 'scan'};  w0 = [-0.9 0.15 0.5];  w1 = [0 0.5 3];  tg = (-1:0.02:3)';  Rt = NaN(numel(tg), NS, NA);
for ia = 1:NA
    bad = 0;
    for id = 1:NS
        r = rs{ia, id};  b = find(~isfinite(r.ap(1:r.kend)) | ~isfinite(r.aN(1:r.kend)), 1);  kl = r.kend;  if ~isempty(b); kl = b - round(0.5 / Ts);  bad = bad + 1; end;  rs{ia, id}.kl = kl;
        kk = (1:kl)';  k = kk(r.t(kk) >= tS - 1);  [tu, iu] = unique(r.t(k));  rat = movmean(r.ap(k) ./ local_slope(r.h(k)), 32);  Rt(:, id, ia) = interp1(tu - tS, rat(iu), tg, 'linear', NaN);
    end
    fprintf('%s | diverging seeds %d\n', arm{ia}, bad);
    for iw = 1:3
        M = NaN(NS, 9);
        for id = 1:NS
            r = rs{ia, id};  kk = (1:r.kl)';  g = kk(r.t(kk) >= tS + w0(iw) & r.t(kk) < tS + w1(iw));  if numel(g) < 50; continue; end
            tr = local_slope(r.h(g));  rt = r.ap(g) ./ tr;  e = r.ap(g) - tr;
            M(id, :) = [median(rt), sqrt(mean((rt - 1).^2)), mean(e.^2 ./ r.Pap(g)), mean(r.aN_tr(g) - r.aN(g)), mean(abs(r.aN_tr(g) - ad)), mean(r.ap(g) < 0), mean(tr), mean(rt < 0.1 | rt > 1.9), max(abs(rt - 1))];
        end
        fprintf('   %-6s a''/true %+.2f %s | rms(ratio-1) %.2f | honesty %.2f | true-est a33 %+.4f | |a33-a_d| %.3f | a''<0 %.2f %s | outside 0.1-1.9 %.2f %s | max|ratio-1| %.1f | true slope %.3f\n', wn{iw}, median(M(:, 1), 'omitnan'), mat2str(round(M(:, 1)', 2)), median(M(:, 2), 'omitnan'), median(M(:, 3), 'omitnan'), median(M(:, 4), 'omitnan'), median(M(:, 5), 'omitnan'), median(M(:, 6), 'omitnan'), mat2str(round(M(:, 6)', 2)), median(M(:, 8), 'omitnan'), mat2str(round(M(:, 8)', 2)), max(M(:, 9)), median(M(:, 7), 'omitnan'));
    end
end
f = figure('Color', 'w', 'Position', [20 20 460 * NA 460]);  tl = tiledlayout(1, NA, 'TileSpacing', 'compact', 'Padding', 'compact');
for ia = 1:NA
    a = nexttile(tl, ia);  hold on;  box on;  plot(tg, Rt(:, :, ia), 'Color', [0.7 0.78 1], 'LineWidth', 0.7, 'HandleVisibility', 'off');  plot(tg, median(Rt(:, :, ia), 2, 'omitnan'), 'Color', cE, 'LineWidth', 2, 'DisplayName', arm{ia});
    yline(1, 'r-', 'LineWidth', 1.4, 'HandleVisibility', 'off');  xline(0, '-.', 'Color', cGr, 'LineWidth', 1.4, 'HandleVisibility', 'off');  ylim([-3 4]);  xlim([-1 3]);  ylabel('estimate / true slope');  xlabel('time since t_{scan}  (sec)');
    legend('Location', 'northoutside', 'Orientation', 'horizontal');  set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off');
end
fn = fullfile(fo, 'fig_1101_scoreboard.png');  exportgraphics(f, fn, 'Resolution', 130);  close(f);  system(['open ' fn]);

function s = local_slope(hb)
    s = arrayfun(@(h) (1 / local_cp(max(h, 1.0003) + 1e-4) - 1 / local_cp(max(h, 1.0003) - 1e-4)) / 2e-4, hb);
end
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
