% STATUS: SCRATCH -- 2026-10-28 (user): document timing (1026 Section 4: delta-theta-hat[k+1] = a_off-hat[k+1] / gap[k], off-diagonal at k+1
%   in the coordinate of V-hat[k]) with and without the angle's own gain L10. Arm A = document (run_t3_xyz_fixed form 'local',
%   doc_timing + theta_out: no L10); arm B1 = doc_timing, angle kept in the KF with F_e angle rows = I, L10 = angle rows of F_e K.
%   Wall theta_y = 20 deg, theta-hat[0] = 0, gains at the seeds, prior 10 deg, one-sided gate 2 sqrt P (simulation setting), 20 s,
%   seeds 1-10. Figures in the style of the document (fig_1026_fixed): one per height, columns A | B1, rows theta_y and theta_x error
%   (theta-hat - theta); 15 um: 3 seeds with the smallest angle error of arm A (same seeds in B1), 5 um: seeds 1-3; best dark.
%   Criteria written before the run: B1 counts the off-diagonal correction twice (L10 ~ L7 / gap) -> expect larger early errors /
%   overshoot in B1; report rms at 1/2/5/10/20 s, paired |error| difference B1 - A, seeds leaving the wall.
%   Outputs test_results/temp_3d_wsonly/fig_1028_L10_{15,5}um.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  T = 20;  seeds = 1:10;  n = numel(seeds);  um = [15 5];  FS = 15;  hatc = char(770);  th = ['θ' hatc];  mu = char(181);
cB = [0 0.25 0.85];  cBl = [0.6 0.72 1];  cG = [0 0.5 0.2];  cGl = [0.6 0.85 0.65];
ttl = {'A: no L_{10} (document)', 'B1: with L_{10}'};
nj = numel(um) * 2 * n;  jobs = cell(1, nj);
if isempty(gcp('nocreate')); parpool('Processes', min(10, feature('numcores'))); end
parfor q = 1:nj
    ih = 1 + floor((q - 1) / (2 * n));  rr = q - (ih - 1) * 2 * n;  f = 1 + floor((rr - 1) / n);  i = rr - (f - 1) * n;
    o = struct('theta_x', 0, 'theta_y', 20, 'h0', um(ih) / Rr, 'T', T, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', ...
               'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', f == 1, 'seeds', seeds(i), 'tag', 'l10');
    R = run_t3_xyz_fixed(o);  jobs{q} = R.runs{1};
end
get = @(ih, f) jobs((ih - 1) * 2 * n + (f - 1) * n + (1:n));
rA = get(1, 1);  sc = zeros(1, n);
for i = 1:n; r = rA{i};  k = find(r.t >= 2 & r.t <= T);  k = k(k <= r.kend);  sc(i) = mean(abs(20 - r.thy(k)) + abs(r.thx(k))); end
[~, oa] = sort(sc);  sel = {oa(1:3), 1:3};
fprintf('15 um arm-A angle scores %s -> seeds %s\n', mat2str(round(sc, 2)), mat2str(seeds(oa(1:3))));
for ih = 1:2
    fig = figure('Color', 'w', 'Position', [60 80 1200 720]);  tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for row = 1:2
        for f = 1:2
            nexttile;  hold on;  box on;  rs = get(ih, f);  ss = sel{ih};
            for j = 3:-1:1
                r = rs{ss(j)};  k = 1:r.kend;  s1 = j == 1;
                if row == 1; y = r.thy(k) - 20; col = cBl; if s1; col = cB; end; else; y = r.thx(k); col = cGl; if s1; col = cG; end; end
                h = plot(r.t(k), y, 'Color', col, 'LineWidth', 1 + 0.8 * s1);  if s1; hs = h; end
            end
            yline(0, 'r', 'LineWidth', 2.5);  uistack(hs, 'top');  xlim([0 T]);  ylim([-30 30]);
            set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);  set(gca().Toolbar, 'Visible', 'off');
            if row == 1
                if f == 1; ylabel('\theta_y error  [\circ]'); end
                legend(hs, sprintf('%s_y - θ_y,  seed %d   (%s)', th, seeds(ss(1)), ttl{f}), 'Location', 'northoutside');
                if f == 1; text(0.97, 0.9, sprintf('%d %sm', um(ih), mu), 'Units', 'normalized', 'HorizontalAlignment', 'right', 'FontSize', FS, 'FontWeight', 'bold'); end
            else
                if f == 1; ylabel('\theta_x error  [\circ]'); end
                legend(hs, sprintf('%s_x - θ_x', th), 'Location', 'northoutside');  xlabel('Time (sec)');
            end
        end
    end
    exportgraphics(fig, fullfile(fo, sprintf('fig_1028_L10_%dum.png', um(ih))), 'Resolution', 150);  close(fig);
    t = jobs{1}.t;  ks = arrayfun(@(s) find(t >= s - 1e-9, 1), [1 2 5 10 20]);  E = cell(1, 2);
    for f = 1:2
        rs = get(ih, f);  EY = cell2mat(cellfun(@(r) r.thy(ks) - 20, rs, 'UniformOutput', false));  EX = cell2mat(cellfun(@(r) r.thx(ks), rs, 'UniformOutput', false));
        bad = sum(cellfun(@(r) any(abs(r.thy(1:r.kend) - 20) > 60 | abs(r.thx(1:r.kend)) > 60) || r.kend < numel(r.t), rs));
        E{f} = {EY, EX};
        fprintf('%2d um %-26s | leave %d | rms theta_y 1/2/5/10/20 s %s | theta_x %s\n', um(ih), strrep(ttl{f}, '_{10}', '10'), bad, ...
                mat2str(round(sqrt(mean(EY.^2, 2, 'omitnan'))', 1)), mat2str(round(sqrt(mean(EX.^2, 2, 'omitnan'))', 1)));
    end
    dy = abs(E{2}{1}) - abs(E{1}{1});  dx = abs(E{2}{2}) - abs(E{1}{2});
    fprintf('%2d um paired |err| B1 - A: theta_y %s +- %s | theta_x %s +- %s\n', um(ih), mat2str(round(mean(dy, 2)', 2)), mat2str(round(std(dy, 0, 2)' / sqrt(n), 2)), ...
            mat2str(round(mean(dx, 2)', 2)), mat2str(round(std(dx, 0, 2)' / sqrt(n), 2)));
end
system(['open ' fullfile(fo, 'fig_1028_L10_15um.png') ' ' fullfile(fo, 'fig_1028_L10_5um.png')]);
