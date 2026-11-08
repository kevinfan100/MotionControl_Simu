% STATUS: SCRATCH -- 2026-11-02 (user): reference arm, cheating: angle Q and kappa-hat only near the bead, read from the true position (r < 9.5 um, opts.cheat_flat_r), off on the floor. rho 4, a_d .07, 40 um/s, 1 um lines, seeds 1-5. Arms: fixed 36 um | square +-13 um, against the same arms without the cheat (temp_square_why.mat, before the kappa coupling fix, no change without motion in the hold). Not a result that can be had in an experiment.
%   Sanity: floor error drops as in run_1102_plane_scan.m arm E. Finished = trajectory ended, no contact, estimator finite. Cached in temp_cheat_flat.mat. Windows 26, 27. Outputs fig_1102_cheat_result.png, fig_1102_cheat_error.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');  Rr = 2.25;  Ts = 1 / 1612;  ad = 0.07;  NS = 5;  hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  L = Rr * hs;  MU = [char(956) 'm'];  cf = fullfile(fo, 'temp_cheat_flat.mat');
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 0.06, 'a_d', ad, 'descent_v', 15 - hs * Rr, 'ap_q', true, 'wall', 'fillet', 'bead_r', 5, 'fillet_rho', 4, 'ap_floor_law', true, 'scan_ramp', 0.5, 'scan_len', 1e6, 'sP_fix_theta_deg', 2, 'q_geo', [0.9 0.9], 'descent_T', 1, 'post_wait', [0 1], 'tag', 'cf', 'du_d', 40 * Ts / Rr, 'kappa_state', true, 'kappa_from_scan', true, 'kappa_pfix', 0.3, 'cheat_flat_r', 9.5);
[IA, ID] = ndgrid(1:2, 1:NS);  N = numel(IA);
if isfile(cf)
    load(cf, 'rs');
else
    rs = cell(N, 1);
    if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
    pctRunOnAll warning('off', 'all');
    parfor q = 1:N
        o = base;  o.seeds = ID(q);
        if IA(q) == 1; o.raster_local = [36 1 27 0.05];  o.bead_start = [18; -13];  o.T = 12 + 1.2 * 27 * 36 / 40 + 1; else; o.raster = [13 1 27];  o.bead_start = [13; -13];  o.T = 12 + 1.4 * 27 * 27 / 40 + 1; end
        R = run_t3_xyz_fixed(o);  r = R.runs{1};
        rs{q} = struct('t', r.t, 'kend', r.kend, 'contact', r.contact, 'tsc', r.tsc, 'te', r.rs_tend, 'p', r.p, 'h', r.h, 'aN', r.aN, 'aN_tr', r.aN_tr, 'thx', r.thx, 'thy', r.thy, 'th_tr', r.th_tr);
    end
    save(cf, 'rs', '-v7.3');
end
S = load(fullfile(fo, 'temp_square_why.mat'), 'rs', 'A');  nm = {'fixed 36', 'square 13'};  am = [3 1];
for c = 1:2
    for w = 1:2
        if w == 1; lst = S.rs([S.A.a] == am(c)); tag = 'default'; else; lst = rs(IA == c); tag = 'cheat'; end
        Ang = [];  RR = [];  fs = 0;  mg = [];  et = [];
        for q = 1:NS
            r = lst{q};  ok = find(isfinite(r.aN(1:r.kend)) & isfinite(r.thx(1:r.kend)));  ke = ok(end);  fin = isfinite(r.te) && ~r.contact && r.t(ke) >= r.te;  fs = fs + fin;  te = r.t(ke);  if isfinite(r.te); te = min(te, r.te); end;  et = [et; te];  k = ok(r.t(ok) >= r.tsc + 0.5 & r.t(ok) <= te);  Ang = [Ang; acosd(min(sum(local_w(r.thx(k), r.thy(k)) .* local_w(r.th_tr(k, 1), r.th_tr(k, 2)), 2), 1))];  RR = [RR; hypot(r.p(k, 1), r.p(k, 2))];  mg = [mg; 1e3 * (min(r.h(k)) * Rr - Rr)];
        end
        m1 = RR > 10;  m2 = RR > 7.2 & RR < 9;  m3 = RR < 7.2;
        fprintf('%-9s %-7s finished %d/%d, ends %s | floor med %.2f 95%% %.2f | ring med %.2f 95%% %.2f | bead med %.2f 95%% %.2f | min gap - contact %s nm\n', nm{c}, tag, fs, NS, mat2str(round(et', 1)), median(Ang(m1)), prctile(Ang(m1), 95), median(Ang(m2)), prctile(Ang(m2), 95), median(Ang(m3)), prctile(Ang(m3), 95), mat2str(round(mg', 1)));
    end
end
fin = cellfun(@(r) isfinite(r.te) && ~r.contact, rs(:));  idb = find(IA(:) == 2 & fin, 1);  if isempty(idb); idb = find(IA(:) == 1 & fin, 1); end;  r = rs{idb};  fprintf('figure run: %s seed %d\n', nm{IA(idb)}, ID(idb));
ok = find(isfinite(r.aN(1:r.kend)));  k = ok(r.t(ok) >= r.tsc & r.t(ok) <= r.te);  P = r.p(k, :);  Ws = movmean(local_w(r.thx(k), r.thy(k)), 81, 1);  Ws = Ws ./ vecnorm(Ws, 2, 2);  W = P - L * Ws;  stepv = conv(double(movmean([0; diff(P(:, 2))], 21) > 0.012), ones(61, 1), 'same') > 0;  bad = stepv | abs(P(:, 1)) > 13.2 | abs(P(:, 2)) > 13.2;  P2 = P;  P2(bad, :) = NaN;  W(bad, :) = NaN;
if any(findall(0, 'Type', 'figure') == 26); close(26); end
f = figure(26);  set(f, 'Color', 'w', 'Position', [40 60 1900 820], 'Visible', 'on', 'Name', 'cheat: probe path and wall from estimated w');  tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');  a1 = nexttile(tl);  hold(a1, 'on');  h1 = plot3(a1, P2(:, 1), P2(:, 2), P2(:, 3), 'Color', [0 0 1], 'LineWidth', 0.9);  legend(a1, h1, 'Probe path', 'Location', 'northoutside');  a2 = nexttile(tl);  hold(a2, 'on');  h2 = plot3(a2, W(:, 1), W(:, 2), W(:, 3), 'Color', [0.9 0 0], 'LineWidth', 0.9);  legend(a2, h2, 'Estimated wall', 'Location', 'northoutside');
for a = [a1 a2]; axis(a, 'equal');  xlim(a, [-14 14]);  ylim(a, [-14 14]);  zlim(a, [-0.5 13.5]);  view(a, -37.5, 22);  grid(a, 'off');  box(a, 'off');  xlabel(a, ['X (' MU ')']);  ylabel(a, ['Y (' MU ')']);  zlabel(a, ['Z (' MU ')']);  set(a, 'FontSize', 15, 'LineWidth', 1.1);  rotate3d(a, 'on'); end
exportgraphics(f, fullfile(fo, 'fig_1102_cheat_result.png'), 'Resolution', 130);
if any(findall(0, 'Type', 'figure') == 27); close(27); end
f2 = figure(27);  set(f2, 'Color', 'w', 'Position', [120 100 1600 620], 'Visible', 'on', 'Name', 'cheat: angle error');  b = axes(f2);  hold(b, 'on');  lst = {S.rs([S.A.a] == am(IA(idb))), rs(IA == IA(idb))};  cc = [0.6 0.6 0.6; 0 0.25 0.85];  hh = gobjects(2, 1);
for w = 1:2
    nmax = max(cellfun(@(r) numel(r.t), lst{w}));  E2 = NaN(nmax, NS);  for q = 1:NS; r = lst{w}{q};  ok = find(isfinite(r.thx(1:r.kend)) & isfinite(r.aN(1:r.kend)));  ke = ok(end);  if isfinite(r.te); ke = min(ke, find(r.t >= r.te, 1)); end;  ok = ok(ok <= ke - 200);  E2(ok, q) = acosd(min(sum(local_w(r.thx(ok), r.thy(ok)) .* local_w(r.th_tr(ok, 1), r.th_tr(ok, 2)), 2), 1)).^2; end
    M = mean(E2, 2, 'omitnan');  cnt = sum(isfinite(E2), 2);  M(cnt < 1) = NaN;  er = sqrt(movmean(M, 81, 'omitnan'));  t = lst{w}{1}.t(:);  t = t(1:min(numel(t), numel(er)));  er = er(1:numel(t));  kk = (1:4:numel(t))';  hh(w) = plot(b, t(kk), er(kk), 'Color', cc(w, :), 'LineWidth', 1.1);
end
xline(b, 12, 'k:', 'HandleVisibility', 'off');  xlim(b, [8 40]);  ylim(b, [0 16]);  xlabel(b, 'Time (sec)');  ylabel(b, ['angle error of w' char(770) '  [deg]']);  box(b, 'on');  set(b, 'FontSize', 15, 'FontWeight', 'bold', 'LineWidth', 1.2);  legend(b, hh, {'Default', 'Q and \kappa off on the floor'}, 'Location', 'northoutside', 'Orientation', 'horizontal');
exportgraphics(f2, fullfile(fo, 'fig_1102_cheat_error.png'), 'Resolution', 130);
function w = local_w(thx, thy)
    tx = thx(:) * pi / 180;  ty = thy(:) * pi / 180;  w = [sin(ty) .* cos(tx), -sin(tx), cos(ty) .* cos(tx)];
end
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
