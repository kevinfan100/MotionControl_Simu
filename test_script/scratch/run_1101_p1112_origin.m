% STATUS: SCRATCH -- 2026-11-01 (user): origin of P(a_33, a_uw) and causal test. Log L.E1112: dP(11,12) = update by 7 channels + prediction groups (a' F_w, position, off-diagonal, slope); opts.no_mu_term_from drops only a' M_u,v / gap from rows 10:11 of F_e from t_scan (diagnostic; a first version forced P(11,12) = 0 and lost positive definiteness, all NaN).
%   5 seeds. Arms: plane V-hat frozen P alive 40 | same, cut | sphere as is | sphere as is, cut. Pre-registered: causal if with the cut mean K(11,w) < 0, y1-w update of a' not all negative and a'/true >= 1. Sanity: budget closure ~ 0 (frozen arm); |P(11,12)| must shrink in the cut arms.
%   Console: budget summed over t_scan .. +0.5 and over +0.5 .. +3 (x 1e6), K(11,w), y1-w on a', a'/true, a' < 0, |a33 - a_d|, min eig. Figure fig_1101_p1112_cut.png (seed 5).
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  rb = 15;  Ts = 1 / 1612;  tS = 12;  Tw = 3;  FS = 14;  cE = [0 0.25 0.85];  cT = [0.85 0 0];  cGr = [0.5 0.5 0.5];
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  ap0 = (1 / local_cp(hs + 1e-4) - 1 / local_cp(hs - 1e-4)) / 2e-4;
base = struct('theta_x', 0, 'theta_y', 10, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 10], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'gap_gate', 2, ...
              'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 0.5, 'a_d', ad, 'ap_q', true, 'ap_floor_law', true, 'scan_dir', [1 0], ...
              'kappa_state', false, 'scan_ramp', 0.5, 'scan_len', 1e6, 'du_d', 40 * Ts / Rr, 'sP_fix_theta_deg', 2, 'descent_T', 1, 'descent_v', 15 - hs * Rr, ...
              'post_wait', [0 1], 'T', tS + Tw, 'tag', 'p2');
arm = {'plane frozen', 'plane frozen, no M_u/gap', 'sphere', 'sphere, no M_u/gap'};  NA = numel(arm);  NS = 5;  [IA, ID] = ndgrid(1:NA, 1:NS);  rs = cell(size(IA));
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
pctRunOnAll warning('off', 'all');
parfor q = 1:numel(IA)
    o = base;  o.seeds = ID(q);
    if IA(q) <= 2; o.freeze_angle = true; end
    if IA(q) >= 3
        o.theta_y = 0;  o.th0_deg = [0 0];  o.wall = 'hemi';  o.hemi_r = rb;  o.hemi_void = true;  o.hemi_start_deg = 10;  o.scan_plane_n = [0 1 0];  o.kappa_state = true;  o.kappa_from_scan = true;
    end
    if IA(q) == 2 || IA(q) == 4; o.no_mu_term_from = tS; end
    R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t, 'kend', r.kend, 'E', r.E1112, 'Kw', r.B119(:, 7), 'dap', r.dap(:, 3), 'ap', r.apN, 'h', r.h, 'aN', r.aN, 'aN_tr', r.aN_tr, 'me', r.mineig, 'contact', r.contact);
end
slope = @(hb) arrayfun(@(h) (1 / local_cp(max(h, 1.0003) + 1e-4) - 1 / local_cp(max(h, 1.0003) - 1e-4)) / 2e-4, hb);
ch = {'y1u', 'y1v', 'y1w', 'ym11', 'ym33', 'ymuw', 'ymvw'};  gp = {'a''F_w', 'pos', 'off', 'slope'};  w0 = [0 0.5];  w1 = [0.5 3];
for ia = 1:NA
    M = NaN(NS, 8);
    for id = 1:NS
        r = rs{ia, id};  kk = (1:r.kend)';  ks = kk(r.t(kk) >= tS + 0.5 & r.t(kk) <= tS + Tw);  k0 = kk(r.t(kk) >= tS & r.t(kk) <= tS + Tw);
        M(id, :) = [mean(r.Kw(ks)), sum(r.dap(k0)) / ap0, median(r.ap(ks) ./ slope(r.h(ks))), mean(r.ap(ks) < 0), mean(abs(r.aN_tr(ks) - ad)), min(r.me(ks)), double(r.contact), mean(abs(r.E(ks, 13)))];
    end
    fprintf('%s | contact %d\n   K(11,w) %s | y1-w on a''/slope %s\n   a''/true %s | a''<0 %s | |a33-a_d| %s | min eig %s | |P(11,12)| %.1e\n', arm{ia}, sum(M(:, 7)), mat2str(round(M(:, 1)', 3)), mat2str(round(M(:, 2)', 2)), mat2str(round(M(:, 3)', 2)), mat2str(round(M(:, 4)', 2)), mat2str(round(M(:, 5)', 3)), mat2str(round(M(:, 6)', 3)), mean(M(:, 8)));
    if ia == 1 || ia == 3
        for iw = 1:2
            U = NaN(NS, 7);  G = NaN(NS, 4);  dP = NaN(NS, 1);  cl = NaN(NS, 1);
            for id = 1:NS
                r = rs{ia, id};  kk = (1:r.kend - 1)';  g = kk(r.t(kk) >= tS + w0(iw) & r.t(kk) < tS + w1(iw));  U(id, :) = sum(r.E(g, 1:7), 1);  G(id, :) = sum(r.E(g, 9:12), 1);  dP(id) = r.E(g(end) + 1, 13) - r.E(g(1), 13);  cl(id) = dP(id) - sum(U(id, :)) - sum(G(id, :));
            end
            fprintf('   budget of P(11,12), t_scan + %.1f .. %.1f, x 1e6: net %+.2f | closure %.1e\n      update: ', w0(iw), w1(iw), 1e6 * mean(dP), max(abs(cl)));  for c = 1:7; fprintf('%s %+.2f  ', ch{c}, 1e6 * mean(U(:, c))); end
            fprintf('\n      predict: ');  for c = 1:4; fprintf('%s %+.2f  ', gp{c}, 1e6 * mean(G(:, c))); end;  fprintf('\n');
        end
    end
end
f = figure('Color', 'w', 'Position', [20 20 520 * NA 700]);  tl = tiledlayout(2, NA, 'TileSpacing', 'compact', 'Padding', 'compact');
for ia = 1:NA
    r = rs{ia, 5};  kk = (1:r.kend)';  k = kk(r.t(kk) >= tS - 1);  kd = k(1:5:end);  t = r.t(kd) - tS;
    a1 = nexttile(tl, ia);  hold on;  box on;  plot(t, r.aN_tr(kd), 'Color', cT, 'LineWidth', 1.1, 'DisplayName', 'a_{33} / a_{nom}');  plot(t, r.aN(kd), 'Color', cE, 'LineWidth', 1, 'DisplayName', arm{ia});  yline(ad, 'k--', 'HandleVisibility', 'off');  ylim([0 0.15]);  ylabel('a_{33} / a_{nom}');  legend('Location', 'northoutside', 'Orientation', 'horizontal');
    a2 = nexttile(tl, NA + ia);  hold on;  box on;  plot(t, r.ap(kd) ./ slope(r.h(kd)), 'Color', cE, 'LineWidth', 1.3);  yline(1, 'r-', 'LineWidth', 1.4);  ylim([-3 3]);  ylabel('estimate / true slope');  xlabel('time since t_{scan}  (sec)');
    for a = [a1 a2]; xline(a, 0, '-.', 'Color', cGr, 'LineWidth', 1.4, 'HandleVisibility', 'off');  xlim(a, [-1 Tw]);  set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off'); end
end
fn = fullfile(fo, 'fig_1101_p1112_cut.png');  exportgraphics(f, fn, 'Resolution', 130);  close(f);  system(['open ' fn]);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
