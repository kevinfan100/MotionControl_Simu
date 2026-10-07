% PURPOSE (2026-09-23, user-defined comparison set, aligned with the mainline conventions):
%   cells   ramp16   PLANE plant, cheat init: hold 0.5 s at 22.2 R (h_init 50 um) -> CONSTANT-VELOCITY descent to 1.10 R
%                    over T_d = 15 s -> hold 0.5 s (total 16 s); trajectory_type 'ramp_descent' with t_hold / t_descend_override
%           fixed_h22p2_cell, fixed_h1p10_cell   CELL plant (plane slid 1.05 R), far-field init a_hat[0] = a_nom, prior 0.5 a_nom,
%                    3 s hold; heights above the visible surface
%   arms    gain_order 2 (level + rate, current) vs gain_order 1 (rate slot locked): "with / without delta a"; everything else
%           at the controller defaults (Ljung schedule, y2_mirror on, MA(2), nw_mcorr). 10 seeds, paired.
%   Saves test_results/temp_5state_noap/ladder_<cell>_o<order>.mat; figure by plot_5state_noap_ladder.m.
%   STATUS: ACTIVE (2026-09-23)
function out = run_5state_noap_ladder(opts)
    if nargin < 1; opts = struct(); end
    if ~isfield(opts, 'seeds');  opts.seeds  = 1:10; end
    if ~isfield(opts, 'cells');  opts.cells  = {'fixed_h22p2_cell', 'fixed_h1p10_cell', 'ramp16'}; end
    if ~isfield(opts, 'orders'); opts.orders = [2 1]; end
    if ~isfield(opts, 'T_d');    opts.T_d    = 15; end
    if ~isfield(opts, 'ctrl_const_override'); opts.ctrl_const_override = struct(); end
    if ~isfield(opts, 'tag'); opts.tag = ''; end     % appended to the saved file name (e.g. '_Td5' for a T_d scan)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));  addpath(fullfile(root, 'test_script', 'integration'));  addpath(fullfile(root, 'test_script', 'scratch'));
    od = fullfile(root, 'test_results', 'temp_5state_noap');  if ~exist(od, 'dir'); mkdir(od); end
    pc = physical_constants();  R = pc.R;  a_nom = pc.Ts / pc.gamma_N;
    out = struct();
    for ic = 1:numel(opts.cells)
        cell_name = opts.cells{ic};
        for od_i = 1:numel(opts.orders)
            order = opts.orders(od_i);
            cfg = canonical_scenario(0.05, 1.10, 'deep');  cfg.eq17_variant = '5state_noap';
            cc = opts.ctrl_const_override;  cc.gain_order = order;  plant_cperp = [];
            switch cell_name
                case {'fixed_h22p2_cell', 'fixed_h1p10_cell'}
                    h = str2double(strrep(cell_name(8:end-5), 'p', '.'));
                    cfg.trajectory_type = 'positioning';  cfg.h_init = h * R;  cfg.h_bottom = h * R;
                    cfg.amplitude = 0;  cfg.t_hold = 0;  cfg.t_descend_override = 0;  cfg.T_sim = 3.0;
                    cc.seed_mode = 'anom';  seg = [0 0 0];  plant_cperp = @cell_cperp_delta;  plant = 'cell';
                case 'ramp16'
                    cfg.trajectory_type = 'ramp_descent';  cfg.h_init = 50;  cfg.h_bottom = 1.10 * R;  cfg.amplitude = 0;
                    cfg.t_hold = 0.5;  cfg.t_descend_override = opts.T_d;  cfg.T_sim = 0.5 + opts.T_d + 0.5;  cfg.h_min = 1.10 * R;
                    cc.seed_mode = 'true_h0';  seg = [0.5, 0.5 + opts.T_d, 0.5 + opts.T_d];  plant = 'plane';
                otherwise
                    error('unknown cell %s', cell_name);
            end
            nS = numel(opts.seeds);  res = struct();
            fprintf('\n=== %s | order %d | plant %s | seed_mode %s | %d seeds ===\n', cell_name, order, plant, cc.seed_mode, nS);
            for q = 1:nS
                o = struct('seed', opts.seeds(q), 'collect_diag', true, 'ctrl_const_override', cc);
                if ~isempty(plant_cperp); o.plant_cperp = plant_cperp; end
                tic; s = run_pure_simulation(cfg, o); tr = toc;
                if q == 1
                    N = numel(s.tout);  res.t = s.tout(:);  res.hd = s.p_d_out(:, 3) / R;  z = zeros(N, nS);
                    res.hb = z; res.a_true = z; res.a_hat = z; res.da_hat = z; res.da_true = z; res.P_a = z; res.P_da = z;
                    res.innov1 = z; res.S_y1 = z; res.innov2 = z; res.S_y2 = z; res.trk = z;
                end
                res.hb(:, q) = s.p_true_out(:, 3) / R;
                res.a_true(:, q) = s.a_true_out(:, 3) / a_nom;  res.a_hat(:, q) = s.diag.a_hat(:, 3) / a_nom;
                res.da_hat(:, q) = s.diag.delta_a_hat(:, 3) / a_nom;  res.da_true(:, q) = [diff(s.a_true_out(:, 3)); 0] / a_nom;
                res.P_a(:, q) = s.diag.P_a(:, 3) / a_nom^2;  res.P_da(:, q) = s.diag.P_da(:, 3) / a_nom^2;
                res.innov1(:, q) = s.diag.innovation_y1(:, 3);  res.S_y1(:, q) = s.diag.S_y1(:, 3);
                res.innov2(:, q) = s.diag.innovation_y2(:, 3);  res.S_y2(:, q) = s.diag.S_y2(:, 3);
                res.trk(:, q) = (s.p_d_out(:, 3) - s.p_true_out(:, 3)) * 1e3;
                fprintf('   seed %2d | %.0f s | a_hat(end) %.4f true %.4f\n', opts.seeds(q), tr, res.a_hat(end, q), res.a_true(end, q));
            end
            res.cell = cell_name;  res.order = order;  res.plant = plant;  res.seg = seg;  res.seeds = opts.seeds;  res.a_nom = a_nom;  res.ctrl_const = s.ctrl_const;
            local_report(res);
            save(fullfile(od, sprintf('ladder_%s%s_o%d.mat', cell_name, opts.tag, order)), '-struct', 'res');
            out.(sprintf('%s%s_o%d', cell_name, strrep(opts.tag, '_', ''), order)) = res;
        end
    end
end

function local_report(res)
    e = res.a_true - res.a_hat;  t = res.t;
    if all(res.seg == 0)
        wins = {'0.0-0.5', [0 0.5]; '0.5-1.5', [0.5 1.5]; 'last 1 s', [t(end) - 1, t(end)]};
    else
        wins = {'hold0', [0, res.seg(1)]; 'descent', [res.seg(1), res.seg(2)]; 'near (h<3R)', [res.seg(1), res.seg(2)]; 'hold-end', [res.seg(3), t(end)]};
    end
    fprintf('   window       | mean e_a   sd_seed   sqrtP44   honesty | y1 hon | y2 hon x IF | |e_da| mean  sd  sqrtP55\n');
    for w = 1:size(wins, 1)
        m = t >= wins{w, 2}(1) & t < wins{w, 2}(2);
        if strcmp(wins{w, 1}, 'near (h<3R)'); m = m & res.hd <= 3; end
        if ~any(m); continue; end
        ew = e(m, :);  sd = mean(std(ew, 0, 2));  sp = mean(sqrt(res.P_a(m, :)), 'all');
        h1 = var(res.innov1(m, :), 0, 'all') / mean(res.S_y1(m, :), 'all');
        g2 = res.S_y2(m, :) > 0;  i2 = res.innov2(m, :);  s2 = res.S_y2(m, :);
        h2 = var(i2(g2)) / mean(s2(g2));
        eda = res.da_hat(m, :) - res.da_true(m, :);
        fprintf('   %-12s | %+8.4f  %8.4f  %8.4f  %7.2f | %6.2f | %6.3f | %9.2e  %8.2e  %8.2e\n', wins{w, 1}, mean(ew, 'all'), sd, sp, sd / sp, h1, h2, ...
                mean(abs(eda), 'all'), mean(std(eda, 0, 2)), mean(sqrt(res.P_da(m, :)), 'all'));
    end
end
