% STATUS: SCRATCH -- 2026-10-25 (user, tests 3 and 4): fixed point, 20 deg wall, 15 / 50 um, seeds 1-3, 20 s. Columns: computed angle
%   (Section 5, A-hat[0] = I, angle theta(A-hat)) | estimated angle (Section 6) with theta-hat[0] = (0, 15) deg (5 deg off, test 3, inside
%   the first-order assumption) | estimated angle with theta-hat[0] = 0 (20 deg off, test 4, outside it). Both forms start from equal
%   gains (A-hat[0] = I); only the estimated form can hold an angle prior (10 deg). Slope q = 0 at the fixed point in both.
%   Rows: theta_y error, theta_x error (line-angle error in +-90 deg for the computed form). Seed 1 dark, 2-3 light.
%   Console (criteria written before the run): NaN / contact / floor count; mean |error| over 10-20 s per arm.
%   Output test_results/temp_3d_wsonly/fig_1025_fixed_{15,50}um.png | EXPIRES: when Section 7 of the document is written
linerr = @(e) local_break(mod(e + 90, 180) - 90);
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  um = [15 50];  seeds = 1:3;  T = 20;  n = numel(seeds);  FS = 15;  hatc = char(770);
if exist('um_sel', 'var'); um = um_sel; end              % 10-25 (user): um_sel = [15 5] for the document
cB = [0 0.25 0.85];  cBl = [0.6 0.72 1];  cG = [0 0.5 0.2];  cGl = [0.6 0.85 0.65];
ttl = {'computed angle', sprintf('estimated, %s_y[0] = 15%s', ['θ' hatc], char(176)), sprintf('estimated, %s_y[0] = 0', ['θ' hatc])};
cols = 1:3;  sfx = '';                                         % 10-25 (user): doc_mode = true -> two columns computed |
if exist('doc_mode', 'var') && doc_mode; cols = [1 3];  sfx = '_doc'; end   %   estimated with theta-hat[0] = 0 (all inits 0)
for ih = 1:2
    runs = cell(3, n);
    if isempty(gcp('nocreate')); parpool('Processes', min(9, feature('numcores'))); end   % 10-25: parallel (3 arms x 3 seeds)
    jobs = cell(1, 3 * n);  hh = um(ih) / Rr;
    parfor q = 1:3 * n
        c = 1 + floor((q - 1) / n);  i = q - (c - 1) * n;
        o = struct('theta_y', 20, 'h0', hh, 'seeds', seeds(i), 'T', T, 'aprime', true, 'tag', 'cmp');
        if c == 2; o.form = 'estimated';  o.th0_deg = [0 15]; end
        if c == 3; o.form = 'estimated';  o.th0_deg = [0 0]; end
        R = run_t3_xyz_fixed(o);  jobs{q} = R.runs{1};
    end
    for q = 1:3 * n;  c = 1 + floor((q - 1) / n);  i = q - (c - 1) * n;  runs{c, i} = jobs{q};  end
    for c = 1:3
        m = NaN(n, 2);  bad = 0;
        for i = 1:n
            r = runs{c, i};  k = find(r.t(1:r.kend) >= 10);
            m(i, :) = [mean(abs(linerr(r.thy(k) - 20))), mean(abs(linerr(r.thx(k))))];
            bad = bad + r.contact + any(isnan(r.thy(1:r.kend)));
            nf(i) = r.n_floor; %#ok<SAGROW>
        end
        fprintf('%d um %-28s | contact/NaN %d | floor %s | mean |err| 10-20 s: theta_y %s deg, theta_x %s deg\n', um(ih), ttl{c}, bad, ...
                mat2str(nf), mat2str(round(m(:, 1)', 2)), mat2str(round(m(:, 2)', 2)));
    end
    fig = figure('Color', 'w', 'Position', [60 80 600 * numel(cols) 720]);
    tiledlayout(2, numel(cols), 'TileSpacing', 'compact', 'Padding', 'compact');
    for row = 1:2
        for c = cols
            nexttile;  hold on;  box on;  hs = gobjects(0);
            for i = n:-1:1
                r = runs{c, i};  k = 1:r.kend;  s1 = (seeds(i) == 1);
                if row == 1; y = linerr(r.thy(k) - 20); col = cBl; if s1; col = cB; end
                else;        y = linerr(r.thx(k));      col = cGl; if s1; col = cG; end; end
                h = plot(r.t(k), y, 'Color', col, 'LineWidth', 1 + 0.8 * s1);  if s1; hs = h; end
            end
            yline(0, 'r', 'LineWidth', 2.5);  uistack(hs, 'top');  xlim([0 T]);  ylim([-30 30]);
            set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);
            if row == 1
                if c == cols(1); ylabel('\theta_y error  [\circ]'); end
                legend(hs, sprintf('%s_y - %s_y,  seed 1   (%s)', ['θ' hatc], char(952), ttl{c}), 'Location', 'northoutside');
                if c == 1
                    text(0.97, 0.9, sprintf('%d %sm', um(ih), char(181)), 'Units', 'normalized', 'HorizontalAlignment', 'right', ...
                         'FontSize', FS, 'FontWeight', 'bold');
                end
            else
                if c == cols(1); ylabel('\theta_x error  [\circ]'); end
                legend(hs, sprintf('%s_x - %s_x', ['θ' hatc], char(952)), 'Location', 'northoutside');  xlabel('Time (sec)');
            end
        end
    end
    exportgraphics(fig, fullfile(fo, sprintf('fig_1025_fixed_%dum%s.png', um(ih), sfx)), 'Resolution', 150);
    close(fig);
end

function e = local_break(e)
    e = e(:);  j = find(abs(diff(e)) > 90);  e(j + 1) = NaN;
end
