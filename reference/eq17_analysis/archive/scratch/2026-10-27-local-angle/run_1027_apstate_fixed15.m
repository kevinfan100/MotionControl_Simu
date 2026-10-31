% STATUS: SCRATCH -- 2026-10-27 (user): the earlier a-bar' state estimator (formC_b ap_slot, arm 'white_fin' of run_mc_apslot_r2.m =
%   10-16 apP002F400H110: whitened y2, Q77 = P_fix^2 I with sqrt P_fix = 0.4 (1.1/h_d)^2) held at a fixed point h = 15 um, plane wall,
%   z axis, seeds 1-10, 10 s (figure, original layout: a-hat vs true and e_a; the 3 seeds with the smallest error, best dark). Question: how fast does a-hat converge at 15 um with the a' state (compare the local three-axis version).
%   10-27 (user): a-hat[0] = 1 (a0 = [] keeps the law seed), sqrt P44[0] = (9/8)/h_bar_0 = 0.169 (as the three-axis runner).
%   Fixed point = 'ramp_descent' with h_init = h_bottom = 15 um. Criteria written before the run: h_d constant at 15 um (sanity);
%   report e_a at 0.5/1/2/5/10 s and sqrt P44. Output test_results/temp_3d_wsonly/fig_1027_apstate_fixed15.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
addpath(genpath(fullfile(root, 'model')));  addpath(fullfile(root, 'test_script', 'integration'));  addpath(here);
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');  pc = physical_constants();  Ts = pc.Ts;  seeds = 1:10;  T = 10;
OV = struct('trajectory_type', 'ramp_descent', 'h_init', 15, 'h_bottom', 15, 't_hold', 0, 't_descend_override', 1.0, ...
            'T_sim', T, 'lambda_c', 0.4);
cc = struct('ma2_aug', false, 'nw_mcorr', false, 'pred_mean2', false, 'pred_mean2_e4', false, 'law_exact_step', false, ...
            'lock_b', false, 'b_ceil', 1.5, 'b_floor', 0.2, 'gain_algebraic', false, 'amodel_fe', false, 'amodel_h', false, ...
            'ws0_perp', 0, 'Pf_w0_std', 0, 'Pf_a_floor', 0.002, ...
            'plaw_dc_slope', true, 'plaw_p', 2, 'c_init', 0, 'Pf_c_std', 1e-6, 'ap_slot', true, 'fe_cmd_eval', true, ...
            'delta_init', 1 / 22.2^2, 'Pf_delta_std', 0.002, 'plaw_g_min', 0.5, 'plaw_delta_lim', [-1, 10], ...
            'plaw_seed_cross', false, 'q77_adapt_P0', 0.4, 'q77_hscale', 1.1, 'y2_whiten', true);
if ~exist('a0', 'var'); a0 = 1; end                                % 10-27 (user): gain starts at 1 (far field), prior (9/8)/h_bar_0
if ~isempty(a0); cc.a_seed_override = a0;  cc.Pf_a_seed_std = (9/8) / (15 / pc.R); end
o = struct('arm', 'best', 'ctrl_const_override', cc, 'config_override', OV, 'scenario', 'deep', 'verbose', false, ...
           'seeds', seeds, 'log_P_full', false);
clear run_formC_b motion_control_law_formC_b;
evalc('Rr = run_formC_b(o);');
ns = numel(Rr.runs);  n = numel(Rr.runs{1}.h_bar_d_out);  t = (0:n-1)' * Ts;
E = zeros(n, ns);  S = E;
for q = 1:ns
    Sq = Rr.runs{q};  hb = max(Sq.h_bar_true_out(:), 1.001);  at = zeros(n, 1);
    for i = 1:n; [~, cp] = calc_correction_functions(hb(i)); at(i) = 1 / cp; end
    E(:, q) = at - Sq.a_bar_hat_out(:, 3);  S(:, q) = Sq.P_a_out(:, 3) / Sq.a_nom;
end
k2 = t >= 2;  scr = mean(abs(E(k2, :)), 1);  [~, og] = sort(scr);  selg = og(1:3);   % 10-27 (user): 3 seeds with the smallest error (2-10 s)
fprintf('seed scores %s -> seeds %s\n', mat2str(round(scr, 4)), mat2str(seeds(selg)));
abar = char(257);  hatc = char(770);                              % 10-27 (user): original layout -- a-hat vs true, e_a; best seed dark, next two light
fig = figure('Color', 'w', 'Position', [60 60 1500 800]);  tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
a1 = nexttile;  hold on;  box on;  a2 = nexttile;  hold on;  box on;
for j = 3:-1:1
    q = selg(j);  Sq = Rr.runs{q};  ah = Sq.a_bar_hat_out(:, 3);  at = ah + E(:, q);  s1 = j == 1;  cb = [0.6 0.72 1];  if s1; cb = [0 0.25 0.85]; end
    h2 = plot(a1, t, at, 'Color', 'r', 'LineWidth', 2);  h1 = plot(a1, t, ah, 'Color', cb, 'LineWidth', 0.8 + 0.8 * s1);
    plot(a2, t, E(:, q), 'Color', cb, 'LineWidth', 0.8 + 0.8 * s1);
    if s1; hs = [h1 h2]; end
end
uistack(hs, 'top');  yline(a2, 0, 'r', 'LineWidth', 2);
ylabel(a1, sprintf('%s_z', abar));  ylim(a1, [0.7 1.05]);  ylabel(a2, sprintf('e_{%s}', abar));  ylim(a2, [-0.2 0.1]);
legend(a1, hs, {sprintf('%s%s_z  (a'' state, z axis, plane, 15 %sm fixed; seed %d, seeds %d %d light)', abar, hatc, char(181), seeds(selg(1)), seeds(selg(2)), seeds(selg(3))), ...
       sprintf('%s_z true', abar)}, 'Location', 'northoutside', 'Orientation', 'horizontal');
xlabel(a2, 'Time (sec)');  set([a1 a2], 'XLim', [0 T], 'FontSize', 15, 'FontWeight', 'bold', 'LineWidth', 1.2);
set(a1.Toolbar, 'Visible', 'off');  set(a2.Toolbar, 'Visible', 'off');
rm = sqrt(mean(E.^2, 2));  sp = mean(S, 2);
ks = arrayfun(@(s) find(t >= s - 1e-9, 1), [0.5 1 2 5 10]);
fprintf('rms over 10 seeds at 0.5/1/2/5/10 s %s | mean sqrt P %s\n', mat2str(round(rm(ks)', 4)), mat2str(round(sp(ks)', 4)));
exportgraphics(fig, fullfile(fo, sprintf('fig_1027_apstate_fixed15_a0%g.png', a0)), 'Resolution', 150);  close(fig);
system(['open ' fullfile(fo, sprintf('fig_1027_apstate_fixed15_a0%g.png', a0))]);
