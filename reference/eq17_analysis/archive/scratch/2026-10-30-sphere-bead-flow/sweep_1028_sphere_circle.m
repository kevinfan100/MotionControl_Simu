% STATUS: SCRATCH -- 2026-10-28 (user): why the fast scan cannot go round the whole sphere -- sweep with the current model. Whole sphere
%   (radius 15 um, void), start 10 deg off the top, hold 10 s, regulation to a_d = 0.05 (tau_a), great-circle scan from 17 s (one turn),
%   slope P_fix law (L). Grid: speed {10 20 40} um/s x tau_a {0.5 1 2} s x angle P_fix {2 3 4} deg, seeds 1-3.
%   "On the sphere" = gap < 5 um. Per run: fraction of the turn completed while on the sphere, mean angle error over that part, NaN.
%   Output: console table + test_results/temp_3d_wsonly/fig_1028_sphere_sweep.png (fraction of the turn, mean of 3 seeds, per speed panel)
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  rb = 15;  tH = 10;  tS = 17;  ph0 = 10;  Ts = 1 / 1612;  gmax = 5;
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  Lscan = 2 * pi * (rb + hs * Rr);
SP = [10 20 40];  TA = [0.5 1 2];  if exist('TA_in', 'var'); TA = TA_in; end;  PF = [2 3 4];
[I1, I2, I3, I4] = ndgrid(1:3, 1:3, 1:3, 1:3);  nq = numel(I1);  res = zeros(nq, 4);
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, ...
              'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', tH, 'a_d', ad, ...
              't_scan', tS, 'ap_q', true, 'q_theta_from', tS, 'wall', 'hemi', 'hemi_r', rb, 'hemi_void', true, 'ap_floor_law', true, ...
              'scan_dir', [1 0], 'hemi_start_deg', ph0, 'scan_plane_n', [0 1 0], 'scan_len', Lscan, 'ap_pfix_law', true, 'tag', 'sw');
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
parfor q = 1:nq
    o = base;  sp = SP(I1(q));  Tsc = Lscan / sp;
    o.du_d = sp * Ts / Rr;  o.T = tS + Tsc + 0.2;  o.tau_a = TA(I2(q));  o.sP_fix_theta_deg = PF(I3(q));  o.seeds = I4(q);
    R = run_t3_xyz_fixed(o);  r = R.runs{1};
    ks = find(r.t >= tS & r.t <= tS + Tsc & (1:numel(r.t))' <= r.kend);
    bad = find(r.h(ks) * Rr > gmax | isnan(r.aN(ks)) | isnan(r.thy(ks)), 1);  if isempty(bad); kk = ks; else; kk = ks(1:bad - 1); end
    ty = unwrap(r.th_tr(1:r.kend, 2) * pi / 180) * 180 / pi;  e = abs(ty(kk) - r.thy(kk));
    frac = numel(kk) / numel(ks);  if isempty(kk); me = NaN; else; me = mean(e); end
    res(q, :) = [frac, me, any(isnan(r.aN(ks))), r.contact];   % contact = true contact with the sphere
end
F = reshape(res(:, 1), 3, 3, 3, 3);  E = reshape(res(:, 2), 3, 3, 3, 3);  Cn = reshape(res(:, 4), 3, 3, 3, 3);
fprintf('fraction of the turn completed on the sphere (gap < %d um), seeds 1/2/3 | mean angle error over that part [deg]\n', gmax);
for i1 = 1:3
    for i2 = 1:3
        for i3 = 1:3
            fprintf('%2d um/s  tau_a %.1f s  angle P_fix %d deg: %s | %s | contact %s\n', SP(i1), TA(i2), PF(i3), mat2str(round(squeeze(F(i1, i2, i3, :))', 2)), mat2str(round(squeeze(E(i1, i2, i3, :))', 1)), mat2str(squeeze(Cn(i1, i2, i3, :))'));
        end
    end
end
f = figure('Color', 'w', 'Position', [40 40 1800 560]);  tl = tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
for i1 = 1:3
    ax = nexttile(tl);  M = mean(F(i1, :, :, :), 4);  M = squeeze(M);   % rows tau_a, cols P_fix
    imagesc(ax, M);  colormap(ax, parula);  caxis(ax, [0 1]);  axis(ax, 'square');
    for a = 1:3; for b = 1:3; text(ax, b, a, sprintf('%.2f', M(a, b)), 'HorizontalAlignment', 'center', 'FontSize', 15, 'FontWeight', 'bold', 'Color', 'k'); end; end
    set(ax, 'XTick', 1:3, 'XTickLabel', compose('%d°', PF), 'YTick', 1:3, 'YTickLabel', compose('%.1f s', TA), 'FontSize', 14, 'FontWeight', 'bold');
    xlabel(ax, 'angle P_{fix}');  ylabel(ax, '\tau_a');  title(ax, sprintf('%d um/s: fraction of the turn on the sphere', SP(i1)));  set(ax.Toolbar, 'Visible', 'off');
end
cb = colorbar(ax);  cb.Layout.Tile = 'east';
fn = fullfile(fo, sprintf('fig_1028_sphere_sweep_tau%s.png', strrep(num2str(TA(1)), '.', 'p')));  exportgraphics(f, fn, 'Resolution', 150);  close(f);  system(['open ' fn]);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
