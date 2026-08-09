function res = run_formB_blind_para(opts)
%RUN_FORMB_BLIND_PARA  The controller is initialised at the plane wall and
%   then meets a different c(h_bar).
%
%   STATUS: ACTIVE -- feeds formB_amp_functions p.7.
%
%   Earlier version of that page gave each truth function its OWN far-field
%   anchor (9/8 for c_perp, 9/16 for c_para). That is a fair writing-versus-
%   writing comparison but it is the wrong experiment: it hands the controller
%   the one number it would not have. A controller that does not know the
%   boundary starts from the plane anchor it always carries.
%
%   So: b_0 = 9/8 on BOTH truths, both writings, and two prior widths.
%
%     narrow  the plane-derived prior, sup|b_eff - 9/8| on c_perp
%             (0.0157 B, 0.0708 C) -- what a controller that believes it is
%             looking at a plane would honestly declare
%     wide    sup|b_eff - 9/8| evaluated on c_para instead
%             (0.7505 B, 0.5697 C) -- what the new boundary actually demands
%             of the plane anchor. Derived, not assumed.
%
%   Output -> test_results/temp_formB_blind_para.mat (gitignored)

    if nargin < 1; opts = struct(); end
    if ~isfield(opts, 'seeds')
        opts.seeds = [7 11 23 42 101 777 31 53 89 137 211 313];
    end
    if ~isfield(opts, 'save'); opts.save = true; end

    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));
    addpath(genpath(fullfile(root, 'test_script')));

    pc = physical_constants(); AX_Z = 3; ANC = 9/8;
    seeds = opts.seeds(:).'; n_s = numel(seeds);
    cfg = local_cfg(pc);
    plant_para = @(hb) calc_correction_functions(hb, true);   % c_para as the truth

    % priors: the plane-derived pair, and what c_para demands of the 9/8 anchor
    [pnB, pnC, flB, flC] = local_priors(cfg, pc, 'perp', ANC);
    [pwB, pwC, fwB, fwC] = local_priors(cfg, pc, 'para', ANC);

    fprintf('=== controller initialised at the plane wall, plant = c_para ===\n');
    fprintf('anchor b0 = 9/8 for BOTH writings on BOTH truths\n');
    fprintf('prior narrow (from c_perp): B %.4f  C %.4f\n', pnB, pnC);
    fprintf('prior wide   (from c_para): B %.4f  C %.4f\n', pwB, pwC);
    [~, bB_t, bC_t] = local_beff_at(cfg, pc, 'para');
    fprintf('what c_para demands at the trough:  B %.4f  C %.4f', bB_t, bC_t);
    fprintf('   (that is %.1f / %.1f narrow-prior widths from 9/8)\n\n', ...
            abs(ANC - bB_t) / pnB, abs(ANC - bC_t) / pnC);

    ARMS = { ...
      'B narrow', 'B', pnB, flB;  'C narrow', 'C', pnC, flC; ...
      'B wide',   'B', pwB, fwB;  'C wide',   'C', pwC, fwC };
    n_a = size(ARMS, 1);

    res = struct('seeds', seeds, 'cfg', cfg, 'anchor', ANC, ...
                 'arms', {ARMS(:,1).'}, 'prior', cell2mat(ARMS(:,3)));
    res.tab = nan(n_a, n_s, 5);
    res.trace = cell(1, n_a);

    fprintf('%-10s  prior    desc %%          osc %%           hold %%          b end\n', 'arm');
    for a = 1:n_a
        ov = struct('lock_b', false, 'lock_p', true, 'lock_ws', true, ...
                    'b_init', ANC, 'Pf_b_std', ARMS{a,3}, 'Pf_a_floor', ARMS{a,4});
        if strcmp(ARMS{a,2}, 'C'); ov.law_form_amp = true; ov.p_init = 1; end
        for q = 1:n_s
            s = run_formB_ws(cfg, struct('seed', seeds(q), ...
                    'ctrl_const_override', ov, 'plant_cperp', plant_para));
            m = local_metrics(s, cfg, AX_Z);
            res.tab(a, q, :) = [m.desc, m.osc, m.hold, ...
                                s.b_hat_out(end, AX_Z), s.P_b_out(end, AX_Z)];
            if q == 1
                res.trace{a} = struct('t', s.tout(:), 'b', s.b_hat_out(:, AX_Z), ...
                    'P', s.P_b_out(:, AX_Z), 'w', s.h_bar_d_out(:), ...
                    'e', 100 * (s.a_hat_out(:,AX_Z) - s.a_true_out(:,AX_Z)) ./ ...
                         s.a_true_out(:,AX_Z));
            end
        end
        T = squeeze(res.tab(a, :, :));
        fprintf('%-10s  %.4f  %6.2f+-%-6.2f  %6.2f+-%-6.2f  %+6.2f+-%-6.2f %.4f+-%.4f\n', ...
                ARMS{a,1}, ARMS{a,3}, mean(T(:,1)), std(T(:,1)), ...
                mean(T(:,2)), std(T(:,2)), mean(T(:,3)), std(T(:,3)), ...
                mean(T(:,4)), std(T(:,4)));
    end
    fprintf('\ntarget at the trough: B %.4f   C %.4f\n', bB_t, bC_t);

    if opts.save
        save(fullfile(root, 'test_results', 'temp_formB_blind_para.mat'), 'res', '-v7.3');
    end
end

% --------------------------------------------------------------------------
function [pB, pC, fB, fC] = local_priors(cfg, pc, which, anchor)
    h = linspace(cfg.h_bottom / pc.R - 0.1, cfg.h_init / pc.R + 1.0, 20001).';
    c = zeros(size(h));
    for i = 1:numel(h)
        if strcmp(which, 'perp'); [~, c(i)] = calc_correction_functions(h(i), true);
        else;                      c(i)     = calc_correction_functions(h(i), true); end
    end
    bB = (c - 1) .* (h - 1);   bC = h .* (c - 1) ./ c;
    pB = max(abs(bB - anchor)); pC = max(abs(bC - anchor));
    fB = max(abs((1 - anchor ./ ((h - 1) + anchor)) - 1 ./ c));
    fC = max(abs((1 - anchor ./ h) - 1 ./ c));
end

function [h, bB, bC] = local_beff_at(cfg, pc, which)
    h = cfg.h_bottom / pc.R;
    if strcmp(which, 'perp'); [~, c] = calc_correction_functions(h, true);
    else;                      c     = calc_correction_functions(h, true); end
    bB = (c - 1) * (h - 1);  bC = h * (c - 1) / c;
end

function m = local_metrics(s, cfg, ax)
    t = s.tout(:);
    e = 100 * (s.a_hat_out(:, ax) - s.a_true_out(:, ax)) ./ s.a_true_out(:, ax);
    t1 = cfg.t_hold; t2 = t1 + cfg.t_descend_override;
    t3 = t2 + cfg.n_cycles / cfg.frequency;
    m.desc = max(abs(e(t > t1 & t <= t2)));
    m.osc  = sqrt(mean(e(t > t2 + 0.2 & t <= t3).^2));
    m.hold = mean(e(t > t3 + 0.3));
end

function cfg = local_cfg(pc)
    cfg = user_config();
    cfg.trajectory_type = 'osc';
    cfg.h_init = 50; cfg.h_bottom = 4.5; cfg.amplitude = 2.5;
    cfg.frequency = 1; cfg.n_cycles = 2;
    cfg.t_hold = 0.5; cfg.t_descend_override = 1.0; cfg.T_sim = 4.8;
    cfg.h_min = 1.1 * pc.R;
    cfg.ctrl_enable = true; cfg.thermal_enable = true; cfg.meas_noise_enable = true;
    cfg.lambda_c = 0.7; cfg.a_pd = 0.05; cfg.a_cov = 0.05;
    cfg.meas_noise_std = [0.00062; 0.00057; 0.00331];
    cfg.h_bar_safe = 1.5;
end
