% STATUS: SCRATCH -- 2026-11-01 (user): step 2 of the review of 1026_local_angle_estimator.tex: rebuild the tracking-error equation (line 120) from the log.
%   res[k] = dp[k+1] - lambda_c dp[k] + (A - A-hat) F, local coordinate, true gains; the document says res = -eps_p: zero mean, unrelated to F_u and F_w.
%   Plane theta_y 10, current flow, kappa off, frame true and frozen (true A is then diagonal), 5 seeds. Arms: C 40 um/s (no off-diagonal P) | C 10 | 3 at 40 um/s (P alive).
%   Also summed update of a-hat_33 by y1-w and mean true - estimate. Sanity: arm C fraction a' < 0 = 0 and |a33 - a_d| = 0.007; hold mean res ~ 0.
%   Console per arm, hold and scan: mean res (nm) +- sem over seeds, sd, regression of res on F_u and on F_w (nm per unit), u and w. Figure fig_1031_model_residual.png (seed 5).
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  Ts = 1 / 1612;  tS = 12;  Tw = 3;  FS = 14;  cGr = [0.5 0.5 0.5];  cE = [0 0.25 0.85];  cT = [0.85 0 0];  nmR = 1e3 * Rr;  thy = 10 * pi / 180;
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
base = struct('theta_x', 0, 'theta_y', 10, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 10], 'gain0', 'seed', 'sP_theta_deg', 1e-3, 'form', 'local', 'gap_gate', 2, ...
              'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 0.5, 'a_d', ad, 'ap_q', true, 'ap_floor_law', true, 'scan_dir', [1 0], ...
              'kappa_state', false, 'scan_ramp', 0.5, 'scan_len', 1e6, 'du_d', 40 * Ts / Rr, 'sP_fix_theta_deg', 2, 'descent_T', 1, 'descent_v', 15 - hs * Rr, ...
              'post_wait', [0 1], 'T', tS + Tw, 'tag', 'mr', 'freeze_angle', true, 'q_theta_off', true);
arm = {'C: 40 um/s, no P', 'C: 10 um/s, no P', '3: 40 um/s, P alive'};  NA = numel(arm);  NS = 5;  [IA, ID] = ndgrid(1:NA, 1:NS);  rs = cell(size(IA));
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
parfor q = 1:numel(IA)
    o = base;  o.seeds = ID(q);
    if IA(q) == 2; o.du_d = 10 * Ts / Rr; end
    if IA(q) == 3; o = rmfield(o, 'q_theta_off');  o.sP_theta_deg = 10; end
    R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t, 'kend', r.kend, 'lc', r.opts.lambda_c, 'apN', r.apN, 'aN', r.aN, 'aT', r.aT, 'aN_tr', r.aN_tr, 'aT_tr', r.aT_tr, 'off', r.off_post, 'vF', r.vF, 'dp', r.dp, 'da33', r.da33, 'dap', r.dap);
end
V = [cos(thy) 0 sin(thy); 0 1 0; -sin(thy) 0 cos(thy)];   % V-hat = V, true and frozen
wn = {'hold', 'scan'};  Q = cell(NA, NS);
for ia = 1:NA
    for id = 1:NS
        r = rs{ia, id};  vd = r.dp * V;  lc = r.lc;  N = r.kend - 1;  k = (1:N)';
        e11 = r.aT_tr(k) - r.aT(k);  e33 = r.aN_tr(k) - r.aN(k);  e13 = -r.off(k, 1);  e23 = -r.off(k, 2);  Fu = r.vF(k, 1);  Fv = r.vF(k, 2);  Fw = r.vF(k, 3);
        ru = vd(k + 1, 1) - lc * vd(k, 1) + e11 .* Fu + e13 .* Fw;
        rw = vd(k + 1, 3) - lc * vd(k, 3) + e33 .* Fw + e13 .* Fu + e23 .* Fv;
        Q{ia, id} = struct('k', k, 'ru', ru, 'rw', rw, 'Fu', Fu, 'Fw', Fw, 'e33', e33);
    end
end
for ia = 1:NA
    for iw = 1:2
        M = NaN(NS, 9);
        for id = 1:NS
            r = rs{ia, id};  s = Q{ia, id};  tk = r.t(s.k);  if iw == 1; g = tk >= tS - 0.9 & tk < tS; else; g = tk >= tS + 0.5 & tk <= tS + Tw; end
            bw = [ones(sum(g), 1), s.Fu(g) - mean(s.Fu(g)), s.Fw(g) - mean(s.Fw(g))] \ s.rw(g);  bu = [ones(sum(g), 1), s.Fu(g) - mean(s.Fu(g)), s.Fw(g) - mean(s.Fw(g))] \ s.ru(g);
            ks = s.k(g);  M(id, :) = [nmR * mean(s.rw(g)), nmR * std(s.rw(g)), nmR * bw(2), nmR * bw(3), nmR * mean(s.ru(g)), nmR * std(s.ru(g)), mean(s.e33(g)), sum(r.da33(ks, 3)), sum(r.dap(ks, 3)) / ((1 / local_cp(hs + 1e-4) - 1 / local_cp(hs - 1e-4)) / 2e-4)];
        end
        sem = @(x) std(x) / sqrt(numel(x));
        fprintf('%-20s %s | w: mean res %+.3f +- %.3f nm, sd %.2f, on F_u %+.1f +- %.1f, on F_w %+.1f +- %.1f | u: mean res %+.3f +- %.3f nm, sd %.2f | true - est a33 %+.4f | y1-w on a33 %s | y1-w on a''/slope %s\n', arm{ia}, wn{iw}, ...
                mean(M(:, 1)), sem(M(:, 1)), mean(M(:, 2)), mean(M(:, 3)), sem(M(:, 3)), mean(M(:, 4)), sem(M(:, 4)), mean(M(:, 5)), sem(M(:, 5)), mean(M(:, 6)), mean(M(:, 7)), mat2str(round(M(:, 8)', 3)), mat2str(round(M(:, 9)', 2)));
    end
end
f = figure('Color', 'w', 'Position', [20 20 650 * NA 1000]);  tl = tiledlayout(3, NA, 'TileSpacing', 'compact', 'Padding', 'compact');
for ia = 1:NA
    r = rs{ia, 5};  s = Q{ia, 5};  g = r.t(s.k) >= tS - 1;  k = s.k(g);  t = r.t(k) - tS;  ks0 = k(r.t(k) >= tS);
    a1 = nexttile(tl, ia);  hold on;  box on;  plot(t, nmR * movmean(s.rw(g), 160), 'Color', cE, 'LineWidth', 1.3, 'DisplayName', arm{ia});  yline(0, 'k-', 'HandleVisibility', 'off');  ylim([-3 3]);  ylabel('normal residual, 160-step mean [nm]');
    legend('Location', 'northoutside', 'Orientation', 'horizontal');
    a2 = nexttile(tl, NA + ia);  hold on;  box on;  plot(r.t(ks0) - tS, cumsum(r.da33(ks0, 3)), 'Color', cE, 'LineWidth', 1.4);  yline(0, 'k-');  ylim([-0.3 0.3]);  ylabel('cumulative y_1-w update of a_{33} / a_{nom}');
    a3 = nexttile(tl, 2 * NA + ia);  hold on;  box on;  plot(t, movmean(s.e33(g), 80), 'Color', cT, 'LineWidth', 1.3);  yline(0, 'k-');  ylim([-0.03 0.03]);  ylabel('(true - estimate) a_{33} / a_{nom}');  xlabel('time since t_{scan}  (sec)');
    for a = [a1 a2 a3]; xline(a, 0, '-.', 'Color', cGr, 'LineWidth', 1.4, 'HandleVisibility', 'off');  xlim(a, [-1 Tw]);  set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off'); end
end
fn = fullfile(fo, 'fig_1031_model_residual.png');  exportgraphics(f, fn, 'Resolution', 130);  close(f);  system(['open ' fn]);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
