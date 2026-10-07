% PURPOSE (2026-09-20): the five-cell battery of the no-a' 5-state layer (0920_5state_noaprime.tex, S10)
%   on the plane plant, house conventions of the delta-law week:
%     fixed   positioning at {22.2, 2.0, 1.10} R for 3 s; seed = plant truth by default (mainline convention, 2026-09-23),
%             opts.fixed_seed_mode = 'anom' for the far-field-init convergence demo (a_hat[0] = a_nom, prior 0.5 a_nom)
%     canon   canonical deep trajectory (hold 0.5 / descent 1 / 2 x 1 Hz to 1.10 R / hold), CHEAT init a_hat[0] = a(h_bar_0)
%     meng    15 um -> 2.5 um ramp over 10 s, hold to 12.5 s, CHEAT init
%   10 seeds, canonical noise, lambda_c 0.7. Saves per-seed z-axis series to test_results/temp_5state_noap/<cell>.mat.
%   Figures: plot_5state_noap_battery.m.  STATUS: ACTIVE (moved from scratch 2026-09-22; 0920_5state_noaprime.tex depends on it)
function out = run_5state_noap_battery(opts)
    if nargin < 1; opts = struct(); end
    if ~isfield(opts, 'seeds');  opts.seeds  = 1:10; end
    if ~isfield(opts, 'cells');  opts.cells  = {'fixed_h22p2', 'fixed_h2p00', 'fixed_h1p10', 'canon', 'meng'}; end
    if ~isfield(opts, 'ctrl_const_override'); opts.ctrl_const_override = struct(); end
    if ~isfield(opts, 'tag_suffix'); opts.tag_suffix = ''; end
    if ~isfield(opts, 'plant'); opts.plant = 'plane'; end
    if ~isfield(opts, 'fixed_seed_mode'); opts.fixed_seed_mode = 'true_h0'; end   % 2026-09-23: mainline convention (seed = plant truth); 'anom' = far-field convergence demo            % 'plane' | 'cell' (cell_cperp_delta: plane slid down 1.05 R; heights are above the VISIBLE surface)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));  addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'temp_5state_noap');  if ~exist(od, 'dir'); mkdir(od); end
    pc = physical_constants();  R = pc.R;  a_nom = pc.Ts / pc.gamma_N;
    plant_cperp = [];  if strcmp(opts.plant, 'cell'); addpath(fullfile(root, 'test_script', 'scratch')); plant_cperp = @cell_cperp_delta; end   % cell_cperp_delta.m lives in scratch
    out = struct();
    for ic = 1:numel(opts.cells)
        cell_name = opts.cells{ic};
        cfg = canonical_scenario(0.05, 1.10, 'deep');  cfg.eq17_variant = '5state_noap';
        cc = opts.ctrl_const_override;
        switch cell_name
            case {'fixed_h22p2', 'fixed_h2p00', 'fixed_h1p10'}
                h = str2double(strrep(cell_name(8:end), 'p', '.'));
                cfg.trajectory_type = 'positioning';  cfg.h_init = h * R;  cfg.h_bottom = h * R;
                cfg.amplitude = 0;  cfg.t_hold = 0;  cfg.t_descend_override = 0;  cfg.T_sim = 3.0;
                cc.seed_mode = opts.fixed_seed_mode;  seg = [0 0 0];
            case 'canon'
                cc.seed_mode = 'true_h0';  seg = [0.5, 1.5, 3.5];
            case 'meng'
                cfg.h_init = 15;  cfg.h_bottom = 2.5;  cfg.amplitude = 0;  cfg.frequency = 1;  cfg.n_cycles = 1;
                cfg.t_hold = 0.5;  cfg.t_descend_override = 10;  cfg.T_sim = 12.5;  cfg.h_min = 2.475;
                cc.seed_mode = 'true_h0';  seg = [0.5, 10.5, 10.5];
            otherwise
                error('unknown cell %s', cell_name);
        end
        if strcmp(cc.seed_mode, 'true_h0') && ~isempty(plant_cperp)   % cheat init = the PLANT's truth at h_0 (z), plane c_para for x/y
            hb0 = cfg.h_init / R;  [cpa0, ~] = calc_correction_functions(hb0);
            cc.seed_value = [a_nom / cpa0; a_nom / cpa0; a_nom / plant_cperp(hb0)];
        end
        nS = numel(opts.seeds);  res = struct();
        fprintf('\n=== %s | plant %s | seed_mode %s | %d seeds ===\n', cell_name, opts.plant, cc.seed_mode, nS);
        for q = 1:nS
            o = struct('seed', opts.seeds(q), 'collect_diag', true, 'ctrl_const_override', cc);
            if ~isempty(plant_cperp); o.plant_cperp = plant_cperp; end
            tic; s = run_pure_simulation(cfg, o); tr = toc;
            if q == 1
                N = numel(s.tout);  res.t = s.tout(:);  res.hd = s.p_d_out(:, 3) / R;
                z = zeros(N, nS);
                res.hb = z; res.a_true = z; res.a_hat = z; res.da_hat = z; res.da_true = z; res.P_a = z; res.P_da = z;
                res.innov1 = z; res.S_y1 = z; res.a_xm = z; res.y2_fed = z; res.gate = z; res.trk = z;
                res.a_hat_x = z; res.a_true_x = z;
            end
            res.hb(:, q) = s.p_true_out(:, 3) / R;
            res.a_true(:, q) = s.a_true_out(:, 3) / a_nom;    res.a_hat(:, q) = s.diag.a_hat(:, 3) / a_nom;
            res.a_true_x(:, q) = s.a_true_out(:, 1) / a_nom;  res.a_hat_x(:, q) = s.diag.a_hat(:, 1) / a_nom;
            res.da_hat(:, q) = s.diag.delta_a_hat(:, 3) / a_nom;
            res.da_true(:, q) = [diff(s.a_true_out(:, 3)); 0] / a_nom;
            res.P_a(:, q) = s.diag.P_a(:, 3) / a_nom^2;  res.P_da(:, q) = s.diag.P_da(:, 3) / a_nom^2;
            res.innov1(:, q) = s.diag.innovation_y1(:, 3);  res.S_y1(:, q) = s.diag.S_y1(:, 3);
            res.a_xm(:, q) = s.diag.a_xm(:, 3) / a_nom;  res.y2_fed(:, q) = s.diag.y2_fed(:, 3) / a_nom;
            res.gate(:, q) = double(s.diag.gate_active(:, 3));
            res.trk(:, q) = (s.p_d_out(:, 3) - s.p_true_out(:, 3)) * 1e3;   % nm
            fprintf('   seed %2d | %.0f s | a_hat(end) %.4f true %.4f | e_a end %+.4f | sqrtP44 %.4f | gate %.2f\n', ...
                    opts.seeds(q), tr, res.a_hat(end, q), res.a_true(end, q), res.a_hat(end, q) - res.a_true(end, q), ...
                    sqrt(res.P_a(end, q)), mean(res.gate(:, q)));
        end
        res.cell = cell_name;  res.plant = opts.plant;  res.seg = seg;  res.seeds = opts.seeds;  res.ctrl_const = s.ctrl_const;  res.a_nom = a_nom;
        local_report(res);
        save(fullfile(od, [cell_name opts.tag_suffix '.mat']), '-struct', 'res');
        out.(cell_name) = res;
    end
end

function local_report(res)
    e = res.a_hat - res.a_true;  t = res.t;
    if all(res.seg == 0)
        wins = {'0.0-0.5', [0 0.5]; '0.5-1.5', [0.5 1.5]; 'last 1 s', [t(end) - 1, t(end)]};
    else
        wins = {'hold0', [0, res.seg(1)]; 'descent', [res.seg(1), res.seg(2)]; 'osc', [res.seg(2), res.seg(3)]; 'hold-end', [res.seg(3), t(end)]};
    end
    fprintf('   window      | mean e_a   sd_seed   sqrtP44   honesty | innov1 var/S1 | |e_da| mean  sd_seed  sqrtP55\n');
    for w = 1:size(wins, 1)
        m = t >= wins{w, 2}(1) & t < wins{w, 2}(2);  if ~any(m); continue; end
        ew = e(m, :);  sd = mean(std(ew, 0, 2));  sp = mean(sqrt(res.P_a(m, :)), 'all');
        hon = var(res.innov1(m, :), 0, 'all') / mean(res.S_y1(m, :), 'all');
        eda = res.da_hat(m, :) - res.da_true(m, :);
        fprintf('   %-11s | %+8.4f  %8.4f  %8.4f  %7.2f | %13.3f | %10.2e  %8.2e  %8.2e\n', wins{w, 1}, mean(ew, 'all'), sd, sp, sd / sp, hon, ...
                mean(abs(eda), 'all'), mean(std(eda, 0, 2)), mean(sqrt(res.P_da(m, :)), 'all'));
    end
end
