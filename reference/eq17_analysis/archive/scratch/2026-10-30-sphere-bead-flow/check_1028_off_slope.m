% STATUS: SCRATCH -- 2026-10-28 (user): off-diagonal rows without the slope term (document, default) vs with it (opts.off_slope).
%   Document version (form 'local', doc_timing + theta_out, one-sided gate 2 sqrt P), wall theta_y = 20 deg, theta-hat[0] = 0, gains at
%   the seeds, prior 10 deg, seeds 1-10 paired, 10 s. Scenarios: fixed 15 um, fixed 5 um, circle 3 um 8 Hz at 15 um, w oscillation
%   3 um 8 Hz at 15 um. Criteria written before the run: fixed points identical (no normal motion); motion within the seed spread.
%   Output test_results/temp_3d_wsonly/fig_1028_off_slope.png (rms theta_y / theta_x, columns = scenarios, two arms).
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  T = 10;  seeds = 1:10;  n = numel(seeds);  hatc = char(770);  th = ['θ' hatc];
sc = {struct('nm', 'fixed 15 µm', 'h', 15), struct('nm', 'fixed 5 µm', 'h', 5), struct('nm', 'circle u-v, 15 µm', 'h', 15, 'dither', [3 8 1]), ...
      struct('nm', 'w oscillation, 15 µm', 'h', 15, 'wdither', [3 8 1])};
ns = numel(sc);  arm = {'without slope term (document)', 'with slope term'};  col = {[0 0.3 0.85], [0.85 0.4 0]};
nj = ns * 2 * n;  jobs = cell(1, nj);
if isempty(gcp('nocreate')); parpool('Processes', min(10, feature('numcores'))); end
parfor q = 1:nj
    is = 1 + floor((q - 1) / (2 * n));  rr = q - (is - 1) * 2 * n;  f = 1 + floor((rr - 1) / n);  i = rr - (f - 1) * n;  S = sc{is};
    o = struct('theta_x', 0, 'theta_y', 20, 'h0', S.h / Rr, 'T', T, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'form', 'local', ...
               'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'off_slope', f == 2, 'seeds', seeds(i), 'tag', 'os');
    if isfield(S, 'dither'); o.dither = S.dither; end
    if isfield(S, 'wdither'); o.wdither = S.wdither; end
    R = run_t3_xyz_fixed(o);  jobs{q} = R.runs{1};
end
fig = figure('Color', 'w', 'Position', [40 60 2200 750]);  tiledlayout(2, ns, 'TileSpacing', 'compact', 'Padding', 'compact');
ax = gobjects(2, ns);  for p = 1:2 * ns; ax(p) = nexttile(p);  hold(ax(p), 'on');  box(ax(p), 'on'); end
t = jobs{1}.t;  ks = arrayfun(@(s) find(t >= s - 1e-9, 1), [1 2 5 10]);
for is = 1:ns
    E = cell(1, 2);
    for f = 1:2
        rs = jobs((is - 1) * 2 * n + (f - 1) * n + (1:n));  EY = NaN(numel(t), n);  EX = EY;  bad = 0;
        for i = 1:n
            r = rs{i};  k = 1:r.kend;  EY(k, i) = r.thy(k) - 20;  EX(k, i) = r.thx(k);
            bad = bad + (r.kend < numel(r.t) || any(isnan(r.thy(k))) || any(abs(EY(k, i)) > 60));
        end
        E{f} = {EY(ks, :), EX(ks, :)};
        h(f) = plot(ax(1, is), t, sqrt(mean(EY.^2, 2, 'omitnan')), 'Color', col{f}, 'LineWidth', 2);
        plot(ax(2, is), t, sqrt(mean(EX.^2, 2, 'omitnan')), 'Color', col{f}, 'LineWidth', 2);
        fprintf('%-22s %-30s | bad %d | rms theta_y 1/2/5/10 s %s | theta_x %s\n', sc{is}.nm, arm{f}, bad, ...
                mat2str(round(sqrt(mean(E{f}{1}.^2, 2, 'omitnan'))', 1)), mat2str(round(sqrt(mean(E{f}{2}.^2, 2, 'omitnan'))', 1)));
    end
    dy = abs(E{2}{1}) - abs(E{1}{1});  dx = abs(E{2}{2}) - abs(E{1}{2});
    fprintf('%-22s paired |err| with - without: theta_y %s +- %s | theta_x %s +- %s | max |diff| over all steps %.2g deg\n', sc{is}.nm, ...
            mat2str(round(mean(dy, 2, 'omitnan')', 2)), mat2str(round(std(dy, 0, 2, 'omitnan')' / sqrt(n), 2)), ...
            mat2str(round(mean(dx, 2, 'omitnan')', 2)), mat2str(round(std(dx, 0, 2, 'omitnan')' / sqrt(n), 2)), ...
            max(cellfun(@(a, b) max(abs(a.thy(1:min(a.kend, b.kend)) - b.thy(1:min(a.kend, b.kend)))), ...
                jobs((is - 1) * 2 * n + (1:n)), jobs((is - 1) * 2 * n + n + (1:n)))));
    title(ax(1, is), sc{is}.nm, 'FontWeight', 'bold');
    legend(ax(1, is), h, arm, 'Location', 'northeast', 'FontSize', 11);
end
ylabel(ax(1, 1), sprintf('rms(θ_y - %s_y)  [\\circ]', th));  ylabel(ax(2, 1), sprintf('rms(θ_x - %s_x)  [\\circ]', th));
for is = 1:ns; xlabel(ax(2, is), 'Time (sec)'); end
set(ax(:), 'XLim', [0 T], 'YLim', [0 20], 'FontSize', 13, 'FontWeight', 'bold', 'LineWidth', 1.2);  arrayfun(@(a) set(a.Toolbar, 'Visible', 'off'), ax(:));
exportgraphics(fig, fullfile(fo, 'fig_1028_off_slope.png'), 'Resolution', 150);  close(fig);
system(['open ' fullfile(fo, 'fig_1028_off_slope.png')]);
