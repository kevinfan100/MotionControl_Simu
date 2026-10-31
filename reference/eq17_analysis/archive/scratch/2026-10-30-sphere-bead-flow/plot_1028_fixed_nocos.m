% STATUS: SCRATCH -- 2026-10-28 (user): fixed point on a plane wall, theta_x = 0 | -45 | -70 deg, theta_y = 20 deg, h = 5 um, no motion, 5 s.
%   Small initial error (inside the small-angle derivation): theta-hat[0] = (theta_x, 17 deg), prior 3 deg, gains at the seeds.
%   With cos | without cos (opts.no_cos) in the theta_y step. Seeds 1-3. Row 1: theta_y - theta-hat_y; row 2: w-hat error (angle between the
%   estimated and true normal). Output test_results/temp_3d_wsonly/fig_1028_fixed_nocos.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  TX = [0 -45 -70];  FS = 15;  hatc = char(770);  th = char(952);  cA = [0 0.25 0.85];  cB = [0.85 0.4 0];
[IA, IC, ID] = ndgrid(1:3, 1:2, 1:3);  rg = cell(size(IA));
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
parfor q = 1:numel(IA)
    o = struct('theta_x', TX(IA(q)), 'theta_y', 20, 'h0', 5 / Rr, 'T', 5, 'aprime', true, 'th0_deg', [TX(IA(q)) 17], 'sP_theta_deg', 3, 'gain0', 'seed', ...
               'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'no_cos', IC(q) == 2, 'seeds', ID(q), 'tag', 'fs');
    R = run_t3_xyz_fixed(o);  rg{q} = R.runs{1};
end
wn = @(tx, ty) [sind(ty) .* cosd(tx), -sind(tx), cosd(ty) .* cosd(tx)];
f = figure('Color', 'w', 'Position', [40 40 2100 950]);  tl = tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
for ia = 1:3
    a1 = nexttile(tl, ia);  hold on;  box on;  a2 = nexttile(tl, 3 + ia);  hold on;  box on;
    for ic = 1:2
        for id = 1:3
            r = rg{ia, ic, id};  k = 1:r.kend;  t = r.t(k);  c = cA;  if ic == 2; c = cB; end;  hv = 'off';  if id == 1; hv = 'on'; end
            plot(a1, t, 20 - r.thy(k), 'Color', c, 'LineWidth', 1.2, 'HandleVisibility', hv);
            plot(a2, t, acosd(min(1, wn(r.thx(k), r.thy(k)) * wn(TX(ia), 20)')), 'Color', c, 'LineWidth', 1.2, 'HandleVisibility', hv);
        end
    end
    title(a1, sprintf('%s_x = %d°  (cos %.2f), %s_y = 20°', th, TX(ia), cosd(TX(ia)), th));
    ylabel(a1, sprintf('%s_y - %s%s_y  [\\circ]', th, th, hatc));  ylim(a1, [-15 15]);  ylabel(a2, sprintf('w%s error  [\\circ]', hatc));  ylim(a2, [0 20]);  xlabel(a2, 'Time (sec)');
    legend(a1, {'with cos', 'without cos'}, 'Location', 'northeast');
    for a = [a1 a2]; yline(a, 0, 'k:', 'HandleVisibility', 'off');  set(a, 'XLim', [0 5], 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off'); end
end
fn = fullfile(fo, 'fig_1028_fixed_nocos.png');  exportgraphics(f, fn, 'Resolution', 150);  close(f);  system(['open ' fn]);
