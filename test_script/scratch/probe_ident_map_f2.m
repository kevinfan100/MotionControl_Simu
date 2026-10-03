% PURPOSE (2026-09-22, user: "analyse concretely what is observable in the current cases, and what (delta, p) would do instead"):
%   an identifiability MAP for the two-constant slope law, per wall (plane / cell / sphere) x trajectory (canon / Meng) x
%   segment, for two parametrisations of the same one-family idea:
%     (delta, c):  a' = (1 - c)/(h + delta)^2            (F2, flag plaw_dc_slope)         level  a = 1 - (1-c)/g,      g = h + delta
%     (delta, p):  a' = 1/(h + delta)^p                  (F4, contact slope pinned at 1)   level  a = 1 - 1/((p-1) g^(p-1))
%   Two computations on the SAME logged run (f2 estimator, plane init, obs_dump on, one seed):
%     exact  gates 4-5 tool (verify_state_observability) with 1 s windows for (delta, c)  -> CRLB per segment
%     proxy  Fisher information of the constants through the a slot: the gain error a constant error produces is the level
%            sensitivity relative to the start, v(g) = s(g) - s(g0) with s = [d a/d theta1, d a/d theta2]; y1 sees it through
%            the force response, weight (F_dw/(1-lambda_c))^2/R1, y2 sees it directly, weight (H2(4))^2/R2 when not gated;
%            I = sum_k w_k v_k v_k'  per segment, posterior = (I + diag(1/sigma^2))^-1, CRLB/prior and the weak eigendirection.
%            The proxy drops the loop's own damping, so it is validated against the exact tool on (delta, c) first, then read
%            for (delta, p) where no filter exists.
%   Also printed: the one-height alias direction (level view) and how much it rotates across the band for each parametrisation.
%   Output: console + reference/eq17_analysis/derivation/figures/ident_map_f2.png (CRLB/prior of each constant accumulating along
%   the run, proxy, both parametrisations, 2 x 3 panels).  EXPIRES: with the F2 line
function out = probe_ident_map_f2(seed)
    if nargin < 1 || isempty(seed); seed = 7; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration')); addpath(here);
    od = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    cfg0 = canonical_scenario(0.05, 1.1, 'deep');  LAM = cfg0.lambda_c;  pc = physical_constants();  Ts = pc.Ts;
    SIG_C = 0.10;  SIG_D = 0.64;  SIG_P = 0.10;                                   % priors: c and delta as the tex; p = family spread 1.94-2.02 + in-wall walk 0.04-0.13
    WALL = {'plane', [], 0.104, -0.085, 1.95; 'cell', @cell_cperp_delta, 1.238, -0.163, 1.94; 'sphere', @sphere_cperp_fig11_ptail, -0.041, 0.014, 2.02};   % name, plant, delta_w, c_w, p_w (F2 / F4 band fits)
    TRAJ = {'canon', struct(), {'hold0', 0, 0.5; 'descent', 0.5, 1.5; 'osc', 1.5, 3.5; 'hold', 3.5, 4.8}; ...
            'meng', struct('trajectory_type', 'osc', 'h_init', 15, 'h_bottom', 2.5, 'amplitude', 0, 'frequency', 1, 'n_cycles', 1, ...
                            't_hold', 0.5, 't_descend_override', 10, 'T_sim', 12.5, 'h_min', 2.475), {'hold0', 0, 0.5; 'ramp', 0.5, 10.5; 'hold', 10.5, 12.5}};
    out = struct();
    fprintf('\n===== one-height alias direction (level view), angle of v(g) = [d a/d theta1, d a/d theta2] over the band =====\n');
    for g = [1.2 1.6 2.2 3.3 5 10]
        adc = atan2d(1.085 / g^2, 1 / g);                                          % (delta, c): [ (1-c)/g^2 , 1/g ]      -> atan((1-c)/g)
        adp = atan2d(1 / g^2, (1 / 1 + log(g)) / (1 * g));                        % (delta, p) at p = 2: [ 1/g^p , (1/(p-1) + ln g)/((p-1) g^(p-1)) ]
        fprintf('  g = %4.1f : (delta, c) %5.1f deg   (delta, p) %5.1f deg\n', g, adc, adp);
    end
    fprintf('  (a direction that does not rotate with g cannot separate the two constants; the rotation across the band is the separability)\n');
    fig = figure('Position', [60 60 1800 900], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');  FS = 15; LFS = 11;
    for it = 1:size(TRAJ, 1)
        SEG = TRAJ{it, 3};
        for iw = 1:size(WALL, 1)
            [name, plant, d_w, c_w, p_w] = WALL{iw, :};
            cc = struct('ma2_aug', false, 'nw_mcorr', false, 'pred_mean2', false, 'pred_mean2_e4', false, 'law_exact_step', false, ...
                        'lock_b', false, 'gain_algebraic', false, 'amodel_fe', false, 'amodel_h', false, ...
                        'plaw_dc_slope', true, 'plaw_p', 2, 'c_init', -0.085, 'Pf_c_std', SIG_C, 'delta_init', 0.104, 'Pf_delta_std', SIG_D, ...
                        'ws0_perp', 0, 'Pf_w0_std', 0, 'Pf_a_floor', 0.002, 'plaw_g_min', 0.5, 'obs_dump', true);
            o = struct('arm', 'best', 'ctrl_const_override', cc, 'config_override', TRAJ{it, 2}, 'scenario', 'deep', 'verbose', false, 'seeds', seed, 'log_P_full', false);
            if ~isempty(plant); o.plant_cperp = plant; end
            clear run_formC_b motion_control_law_formC_b;
            evalc('R = run_formC_b(o);');  L = obs_dump('get');  S1 = R.runs{1};  wd = S1.h_bar_d_out(:);  n = numel(wd);  t = (0:n-1)' * Ts;
            Lz = L([L.ax] == 3);
            % ---------- exact: gates 4-5 tool, 1 s windows, (delta, c)
            cfgv = struct('axis', 3, 'free', [1 2 3 4 5 7], 'neg_ctrl', 6, 'prior_std', [NaN NaN NaN NaN SIG_C SIG_D], ...
                          'labels', {{'dw1', 'dw2', 'dw3', 'a_w', 'c', 'delta'}}, 'window', 1600, 't_end', t(end), 'segments', {SEG});
            ev = evalc('res = verify_state_observability(L, cfgv);');
            out.(TRAJ{it, 1}).(name).exact = res;  out.(TRAJ{it, 1}).(name).exact_console = ev;
            % ---------- proxy: per-step weights and level sensitivities at the TRUE wall constants
            m = numel(Lz);  wk = zeros(m, 1);  gk = zeros(m, 1);  tk = zeros(m, 1);
            for i = 1:m
                k = Lz(i).k + 1;  if k < 1 || k > n; continue; end
                tk(i) = t(k);  gk(i) = max(wd(k) + d_w, 0.5);
                F_dw = -Lz(i).F(3, 4);  R1 = Lz(i).R(1);  R2 = Lz(i).R(2);  H2 = Lz(i).H{2};
                wk(i) = (F_dw / (1 - LAM))^2 / R1;                                % y1: force response of a gain error, integrated by the tracking loop
                if ~isempty(H2) && R2 > 0; wk(i) = wk(i) + H2(4)^2 / R2; end        % y2: the level read directly (echo-scaled row)
            end
            keep = tk > 0 | (1:m)' == 1;  wk = wk(keep); gk = gk(keep); tk = tk(keep);
            g0 = gk(1);
            sDC = @(g) [(1 - c_w) ./ g.^2, 1 ./ g];                                  % d a/d delta, d a/d c
            sDP = @(g) [1 ./ g.^p_w, (1 / (p_w - 1) + log(g)) ./ ((p_w - 1) * g.^(p_w - 1))];   % d a/d delta, d a/d p
            [cr_dc, wk_dc, ang_dc] = local_accum(sDC(gk) - sDC(g0), wk, [SIG_D SIG_C]);
            [cr_dp, wk_dp, ang_dp] = local_accum(sDP(gk) - sDP(g0), wk, [SIG_D SIG_P]);
            out.(TRAJ{it, 1}).(name).proxy = struct('t', tk, 'g', gk, 'w', wk, 'cr_dc', cr_dc, 'cr_dp', cr_dp);
            % ---------- console: per segment
            fprintf('\n===== %s | %s  (true constants delta %+.3f, c %+.3f, p %.2f) =====\n', upper(TRAJ{it, 1}), upper(name), d_w, c_w, p_w);
            fprintf('  %-8s | exact 1 s windows: CRLB/prior c, delta (best window) | proxy over the segment: (delta,c) delta, c | (delta,p) delta, p | weak dir (delta,c) / (delta,p)\n', 'segment');
            for s = 1:size(SEG, 1)
                ms = tk >= SEG{s, 2} & tk < SEG{s, 3};
                [cs, ~, as] = local_accum(sDC(gk(ms)) - sDC(g0), wk(ms), [SIG_D SIG_C]);
                [ps, ~, ap] = local_accum(sDP(gk(ms)) - sDP(g0), wk(ms), [SIG_D SIG_P]);
                ex = local_exact_seg(res, SEG{s, 1});
                fprintf('  %-8s | %6.3f %6.3f | %6.3f %6.3f | %6.3f %6.3f | %5.0f deg / %5.0f deg\n', SEG{s, 1}, ex(1), ex(2), cs(end, 1) / SIG_D, cs(end, 2) / SIG_C, ps(end, 1) / SIG_D, ps(end, 2) / SIG_P, as(end), ap(end));
            end
            fprintf('  whole run: (delta,c) delta %.3f c %.3f | (delta,p) delta %.3f p %.3f  (CRLB/prior, proxy, information accumulated from t = 0)\n', ...
                    cr_dc(end, 1) / SIG_D, cr_dc(end, 2) / SIG_C, cr_dp(end, 1) / SIG_D, cr_dp(end, 2) / SIG_P);
            % ---------- figure panel
            nexttile((it - 1) * 3 + iw); hold on; box on;
            h1 = plot(tk, cr_dc(:, 1) / SIG_D, '-', 'Color', [0 0.2 0.9], 'LineWidth', 1.8, 'DisplayName', '(\delta, c):  \delta');
            h2 = plot(tk, cr_dc(:, 2) / SIG_C, '--', 'Color', [0 0.2 0.9], 'LineWidth', 1.8, 'DisplayName', '(\delta, c):  c');
            h3 = plot(tk, cr_dp(:, 1) / SIG_D, '-', 'Color', [0.8 0 0], 'LineWidth', 1.8, 'DisplayName', '(\delta, p):  \delta');
            h4 = plot(tk, cr_dp(:, 2) / SIG_P, '--', 'Color', [0.8 0 0], 'LineWidth', 1.8, 'DisplayName', '(\delta, p):  p');
            h0 = plot(NaN, NaN, 'w.', 'DisplayName', sprintf('%s, %s', upper(name), TRAJ{it, 1}));
            for s = 2:size(SEG, 1); xline(SEG{s, 2}, '--', 'Color', [0.5 0.5 0.5], 'HandleVisibility', 'off'); end
            set(gca, 'YScale', 'log'); ylim([1e-2 1.2]); xlim([0 tk(end)]);
            if iw == 1; ylabel('CRLB / prior  (proxy, accumulated)', 'FontSize', FS, 'FontWeight', 'bold'); end
            if it == 2; xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold'); end
            legend([h0 h1 h2 h3 h4], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on', 'NumColumns', 5);
            set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.5, 'Box', 'on'); grid off;
        end
    end
    fn = fullfile(od, 'ident_map_f2.png');  exportgraphics(fig, fn, 'Resolution', 150); close(fig); fprintf('\nfigure: %s\n', fn);
end

function [cr, W, ang] = local_accum(V, w, sig)
% accumulated posterior CRLB of the two constants along the steps (rows of V), and the weak eigendirection angle (deg, w.r.t. theta1 axis)
    m = size(V, 1);  cr = zeros(m, 2);  ang = zeros(m, 1);  I = diag(1 ./ sig.^2);  W = zeros(m, 1);
    for k = 1:m
        I = I + w(k) * (V(k, :)' * V(k, :));
        C = inv(I);  cr(k, :) = sqrt(diag(C))';
        [E, D] = eig(I);  [~, j] = min(diag(D));  ang(k) = atan2d(E(2, j), E(1, j));  if ang(k) < 0; ang(k) = ang(k) + 180; end
        W(k) = w(k);
    end
end

function ex = local_exact_seg(res, segname)
% pull the tool's per-segment CRLB (best window) of c and delta; tolerate field-name differences by scanning
    ex = [NaN NaN];
    try
        SEG = evalin('caller', 'SEG');  j = find(strcmp(SEG(:, 1), segname), 1);
        sel = res.t_win >= SEG{j, 2} & res.t_win <= SEG{j, 3};
        if any(sel)
            c5 = median(res.crlb(sel, 5, 1));  c7 = median(res.crlb(sel, 6, 1));   % free order [1 2 3 4 5 7] -> columns 5 = c, 6 = delta; channel set 1 = y1+y2
            ex = [c5 / 0.10, c7 / 0.64];
        end
    catch
    end
end
