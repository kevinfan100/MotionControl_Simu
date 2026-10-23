% STATUS: SCRATCH -- 2026-10-16 (user): motion_ctrl style, a' difference vs a' state on the constant-velocity descent
%   22.2 -> 1.10 R in 1 s + 0.2 s hold, lambda_c 0.4, seed 1. One figure per wall; columns:
%     difference : first-order estimator (5state_noap production, eps 1e-5) + gain feed-forward a'*del_pd with a' = low-passed
%                  difference of its own a_hat (ctrl_const.ap_ff 'lpf', beta 2/51) -- run_apff_desc_l1.m;
%     state      : a' as a state (formC_b ap_slot, apP002F400H110) -- run_mc_apslot_compare(..., 'desc').
%   Rows: z_d (red) / z (blue dashed) [um]; a-bar_z true (red) / estimate (blue); a-bar' true (red) / estimate (blue), log scale,
%   negative values not drawn; delta z_m [um]. Output test_results/temp_1009_ws/fig_mc_apdiff_<wall>_s<seed>.png.
%   10-19 (user): mode 'ws' = columns a' state | w_s state (Section 8 + fe_cmd_eval, run_mc_apslot_compare arm s8fc 'desc'), plane only;
%   the w_s column's slope row is the slope the estimator uses, 1/(h-hat - w_s-hat)^2. Output fig_mc_apws_desc_plane_s<seed>.png.
function plot_mc_apdiff(seed, mode)
    if nargin < 1; seed = 1; end
    if nargin < 2 || isempty(mode); mode = 'diff'; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(genpath(fullfile(root, 'model')));
    od = fullfile(root, 'test_results', 'temp_1009_ws');  pc = physical_constants();  R_UM = pc.R;
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  FS = 22;  LFS = 17;  AXLW = 2.2;
    AB = char(257);  HAT = char(770);  az = [AB '_z'];  NM = {'difference', 'state'};  WALLS = {'plane', 'cell'};
    if strcmp(mode, 'ws'); NM = {[AB char(8242) ' state'], ['w' char(772) '_s state']};  WALLS = {'plane'}; end
    a_nom = pc.Ts / pc.gamma_N;
    for wall = WALLS
        w = wall{1};
        if strcmp(mode, 'ws')
            B = load(fullfile(od, sprintf('apcmp_s8fc_%s_desc.mat', w)));  B.wd(1, :) = B.wd(2, :);
            rl = struct('t', B.t, 'h_d', B.wd(:, seed), 'h', B.h_true(:, seed), 'a_true', B.a_true(:, seed), 'a_hat', B.a_hat(:, seed), ...
                        'ap_hat', B.ap_hat(:, seed) / a_nom, 'dxm', B.dxm(:, seed));   % a_prime_out [physical] / a_nom
        else
            rl = load(fullfile(od, sprintf('apff_lpf_%s_s%d.mat', w, seed)));
        end
        A = load(fullfile(od, sprintf('apcmp_apP002F400H110_%s_desc.mat', w)));  A.wd(1, :) = A.wd(2, :);   % first sample is a placeholder
        C = {struct('t', rl.t, 'h_d', rl.h_d, 'h', rl.h, 'a_true', rl.a_true, 'a_hat', rl.a_hat, 'ap', rl.ap_hat, 'dxm', rl.dxm), ...
             struct('t', A.t, 'h_d', A.wd(:, seed), 'h', A.h_true(:, seed), 'a_true', A.a_true(:, seed), 'a_hat', A.a_hat(:, seed), ...
                    'ap', -A.w_hat(:, seed), 'dxm', A.dxm(:, seed))};
        if strcmp(mode, 'ws'); C = C([2 1]); end   % a' state left, w_s state right
        f = figure('Position', [40 40 1600 2000], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
        tl = tiledlayout(4, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
        for ic = 1:2
            r = C{ic};  t = r.t;  ax = gobjects(4, 1);
            ax(1) = nexttile(tl, ic);  hold on; box on;
            hd = plot(ax(1), t, r.h_d * R_UM, '-', 'Color', cT, 'LineWidth', 5.0, 'DisplayName', 'z_d');
            hz = plot(ax(1), t, r.h * R_UM, '--', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', 'z');
            ylim(ax(1), [0 55]);
            lg = legend(ax(1), [hd hz], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
            lg.Title.String = sprintf('%s wall, %s', w, NM{ic});
            ax(2) = nexttile(tl, 2 + ic);  hold on; box on;
            ht = plot(ax(2), t, r.a_true, '-', 'Color', cT, 'LineWidth', 5.0, 'DisplayName', az);
            hh = plot(ax(2), t, r.a_hat, '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', [AB HAT '_z']);
            ylim(ax(2), [-0.2 1.6]);
            legend(ax(2), [ht hh], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
            ax(3) = nexttile(tl, 4 + ic);  hold on; box on;
            apt = local_ap(max(r.h, 1.001), w);  aph = r.ap;  aph(aph <= 0) = NaN;
            ht3 = plot(ax(3), t, apt, '-', 'Color', cT, 'LineWidth', 5.0, 'DisplayName', [AB '''_z']);
            hh3 = plot(ax(3), t, aph, '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', [AB HAT '''_z']);
            set(ax(3), 'YScale', 'log');  ylim(ax(3), [1e-4 3]);  yticks(ax(3), [1e-4 1e-2 1]);
            legend(ax(3), [ht3 hh3], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
            ax(4) = nexttile(tl, 6 + ic);  hold on; box on;
            plot(ax(4), t, r.dxm, '-', 'Color', cE, 'LineWidth', 1.0);
            ylim(ax(4), [-0.2 0.2]);  yticks(ax(4), [-0.1 0 0.1]);
            xlabel(ax(4), 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
            for q = 1:4; xlim(ax(q), [0 t(end)]);  set(ax(q), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(ax(q), 'off'); end
            if ic == 1
                ylabel(ax(1), ['z  [' char(181) 'm]'], 'FontSize', FS, 'FontWeight', 'bold');
                ylabel(ax(2), az, 'FontSize', FS + 2, 'FontWeight', 'bold');
                ylabel(ax(3), [AB '''_z'], 'FontSize', FS + 2, 'FontWeight', 'bold');
                ylabel(ax(4), [char(948) 'z_m  [' char(181) 'm]'], 'FontSize', FS, 'FontWeight', 'bold');
            end
            e = r.a_true - r.a_hat;  pk = @(tc) e(find(t >= tc - 1e-9, 1));
            fprintf('%-5s %-10s seed %d | descent mean|e_a| %.3f | e_a 0.9 / 0.95 / 1.0 s %+.3f / %+.3f / %+.3f | hold %+.3f | dz_m rms %.4f um\n', ...
                    w, NM{ic}, seed, mean(abs(e(t <= 1))), pk(0.9), pk(0.95), pk(1.0), mean(e(t >= 1.05)), rms(r.dxm));
        end
        out = fullfile(od, sprintf('fig_mc_apdiff_%s_s%d.png', w, seed));
        if strcmp(mode, 'ws'); out = fullfile(od, sprintf('fig_mc_apws_desc_%s_s%d.png', w, seed)); end
        exportgraphics(f, out, 'Resolution', 150);  close(f);  fprintf('figure: %s\n', out);
    end
end

function ap = local_ap(h, wall)
    d = 1e-4;  ap = (local_a(h + d, wall) - local_a(h - d, wall)) / (2 * d);
end

function a = local_a(h, wall)
    if strcmp(wall, 'cell'); a = 1 ./ cell_cperp_delta(h); return; end
    a = zeros(size(h));  for i = 1:numel(h); [~, c] = calc_correction_functions(h(i)); a(i) = 1 / c; end
end
