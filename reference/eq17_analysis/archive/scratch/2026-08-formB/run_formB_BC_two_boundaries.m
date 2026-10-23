function res = run_formB_BC_two_boundaries(opts)
%RUN_FORMB_BC_TWO_BOUNDARIES  Two truth functions x two writings x free/locked.
%
%   STATUS: ACTIVE -- feeds the pages appended to
%   reference/eq17_analysis/derivation/formB_amp_functions.tex.
%
%   Eight arms, one z axis, one trajectory, one set of seeds. The ONLY things
%   that change are (i) which truth function the plant obeys, (ii) which
%   writing the controller assumes, (iii) whether the writing's constant is
%   estimated or frozen at its anchor.
%
%   TRUTH FUNCTIONS. Both come from the same published wall solution:
%       perp  c_perp(w)   the wall-normal correction  (the production case)
%       para  c_para(w)   the wall-parallel correction, substituted into the
%                         wall-normal channel via the driver's plant_cperp
%                         handle. SYNTHETIC by construction: it is not "the
%                         x/y axes", it is the same filter on the same axis
%                         fed a different truth, which is what makes it a
%                         controlled comparison.
%
%   ANCHORS. Each boundary uses ITS OWN published far-field reflection
%   coefficient, and both writings use the same one, so the comparison is
%   between writings and not between anchors:
%       perp  9/8      para  9/16
%   Each writing then declares the prior its own read-off honestly costs,
%   sup|b_eff - anchor| over the envelope. Nothing is fitted.
%
%   PRE-REGISTERED PREDICTION (before the run). The estimator is worth
%   something only when the anchor is further from the demanded constant than
%   the filter's own final uncertainty. The distance to close, at the trough:
%       Form B   perp 0.0002  ->  para 0.1764   (x880)
%       Form C   perp 0.0663  ->  para 0.0054   (/12)
%   so the roles should REVERSE: B's estimator should go from harmful to
%   useful, C's from useful to nearly worthless. If that fails, the criterion
%   is wrong and must be withdrawn.
%
%   Output -> test_results/temp_formB_two_boundaries.mat (gitignored)

    if nargin < 1; opts = struct(); end
    if ~isfield(opts, 'seeds')
        opts.seeds = [7 11 23 42 101 777 31 53 89 137 211 313];
    end
    if ~isfield(opts, 'save'); opts.save = true; end

    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));
    addpath(genpath(fullfile(root, 'test_script')));

    pc = physical_constants(); AX_Z = 3;
    seeds = opts.seeds(:).'; n_s = numel(seeds);
    cfg = local_cfg(pc);

    BND = { 'perp', 9/8,  [];                                        ...
            'para', 9/16, @(hb) calc_correction_functions(hb, true) };
    WRT = {'B', 'C'};
    LCK = {false, true};

    % ---- priors and the pre-registered distance, per (boundary, writing) --
    fprintf('=== 兩個真值 x 兩個寫法 x free/locked（%d seeds）===\n\n', n_s);
    fprintf('%-6s %-6s  anchor  prior     shape floor   trough b_eff   |anchor-b_eff|\n', ...
            'truth', 'form');
    P = struct();
    for ib = 1:2
        for iw = 1:2
            [sp, fl, bt] = local_prior(cfg, pc, BND{ib,1}, WRT{iw}, BND{ib,2});
            P.(BND{ib,1}).(WRT{iw}) = [sp, fl];
            fprintf('%-6s %-6s  %.4f  %.4f    %.5f       %.4f         %.4f\n', ...
                    BND{ib,1}, WRT{iw}, BND{ib,2}, sp, fl, bt, abs(BND{ib,2} - bt));
        end
    end

    res = struct('seeds', seeds, 'cfg', cfg, 'bnd', {BND(:,1).'}, 'wrt', {WRT});
    res.anchor = [BND{1,2}, BND{2,2}];
    res.tab  = nan(2, 2, 2, n_s, 5);   % bnd x writing x lock x seed x [desc osc hold bend Pend]
    res.trace = cell(2, 2, 2);         % first seed only, for the figures

    fprintf('\n%-6s %-6s %-8s  desc %%          osc %%           hold %%          b end\n', ...
            'truth', 'form', 'const');
    t0 = tic;
    for ib = 1:2
        for iw = 1:2
            pr = P.(BND{ib,1}).(WRT{iw});
            for il = 1:2
                ov = local_ov(WRT{iw}, LCK{il}, BND{ib,2}, pr);
                for q = 1:n_s
                    topts = struct('seed', seeds(q), 'ctrl_const_override', ov, ...
                                   'plant_cperp', BND{ib,3});
                    s = run_formB_ws(cfg, topts);
                    m = local_metrics(s, cfg, AX_Z);
                    res.tab(ib, iw, il, q, :) = [m.desc, m.osc, m.hold, ...
                                                 s.b_hat_out(end, AX_Z), s.P_b_out(end, AX_Z)];
                    if q == 1
                        res.trace{ib, iw, il} = struct( ...
                            't', s.tout(:), 'e', 100 * (s.a_hat_out(:,AX_Z) - ...
                                 s.a_true_out(:,AX_Z)) ./ s.a_true_out(:,AX_Z), ...
                            'b', s.b_hat_out(:, AX_Z), 'P', s.P_b_out(:, AX_Z), ...
                            'w', s.h_bar_d_out(:));
                    end
                end
                T = squeeze(res.tab(ib, iw, il, :, :));
                if LCK{il}; lb = 'locked'; else; lb = 'free'; end
                fprintf('%-6s %-6s %-8s  %6.3f+-%-6.3f  %6.3f+-%-6.3f  %+6.3f+-%-6.3f %.4f\n', ...
                        BND{ib,1}, WRT{iw}, lb, mean(T(:,1)), std(T(:,1)), ...
                        mean(T(:,2)), std(T(:,2)), mean(T(:,3)), std(T(:,3)), mean(T(:,4)));
            end
        end
    end
    fprintf('\ntotal %.0f s for %d runs\n', toc(t0), 2*2*2*n_s);

    % ---- the pre-registered test ---------------------------------------
    fprintf('\n=== 估測器的貢獻 = free - locked（配對，負 = 有幫助）===\n');
    fprintf('%-6s %-6s  desc            osc             hold |mean|\n', 'truth', 'form');
    for ib = 1:2
        for iw = 1:2
            f_ = squeeze(res.tab(ib, iw, 1, :, :));
            l_ = squeeze(res.tab(ib, iw, 2, :, :));
            d1 = f_(:,1) - l_(:,1);  d2 = f_(:,2) - l_(:,2);
            d3 = abs(f_(:,3)) - abs(l_(:,3));
            fprintf('%-6s %-6s  %+6.3f t=%+5.2f  %+6.3f t=%+5.2f  %+6.3f t=%+5.2f\n', ...
                    BND{ib,1}, WRT{iw}, ...
                    mean(d1), mean(d1)/(std(d1)/sqrt(n_s)), ...
                    mean(d2), mean(d2)/(std(d2)/sqrt(n_s)), ...
                    mean(d3), mean(d3)/(std(d3)/sqrt(n_s)));
        end
    end
    fprintf('\n預測：perp 上 B 有害 / C 有用；para 上兩者對調。\n');

    if opts.save
        save(fullfile(root, 'test_results', 'temp_formB_two_boundaries.mat'), 'res', '-v7.3');
        fprintf('saved test_results/temp_formB_two_boundaries.mat\n');
    end
end

% --------------------------------------------------------------------------
function ov = local_ov(writing, locked, anchor, pr)
    ov = struct('lock_b', locked, 'lock_p', true, 'lock_ws', true, ...
                'b_init', anchor, 'Pf_b_std', pr(1), 'Pf_a_floor', pr(2));
    if strcmp(writing, 'C')
        ov.law_form_amp = true;  ov.p_init = 1;
    end
end

function [sP, fl, b_trough] = local_prior(cfg, pc, bnd, writing, anchor)
    h = linspace(cfg.h_bottom / pc.R - 0.1, cfg.h_init / pc.R + 1.0, 20001).';
    c = zeros(size(h));
    for i = 1:numel(h)
        if strcmp(bnd, 'perp'); [~, c(i)] = calc_correction_functions(h(i), true);
        else;                    c(i)     = calc_correction_functions(h(i), true); end
    end
    if strcmp(writing, 'B')
        b = (c - 1) .* (h - 1);  a = 1 - anchor ./ ((h - 1) + anchor);
    else
        b = h .* (c - 1) ./ c;   a = 1 - anchor ./ h;
    end
    sP = max(abs(b - anchor));  fl = max(abs(a - 1 ./ c));
    b_trough = interp1(h, b, cfg.h_bottom / pc.R);
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
