% STATUS: SCRATCH -- 2026-10-25 (user): regulation from t = 0 (no hold), start 15 um, 20 deg wall, a_d = 0.1, tau_a = 2 s (1021 law
%   Delta h = (Ts/tau_a)(a_d - a_perp-hat)/a_perp'-hat along w-hat), seed 1, 10 s, slope q on. Columns computed | estimated
%   (theta-hat[0] = 0). Rows in the 1021 style (fig_1021_flow_plane_est_s1): a_perp (red true, blue estimate, black dashed a_d, gray
%   dashed = the exponential a_d + (a0 - a_d) exp(-t/tau_a) the law aims at), a_perp' (red true slope along the normal, blue estimate;
%   the law is exponential only while they agree), a_par (red, green), angles (red true, blue theta_y-hat, green theta_x-hat; band
%   +-sqrt P for the estimated form). Output test_results/temp_3d_wsonly/fig_1025_reg0.png | EXPIRES: when the response is explained
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);
addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  T = 10;  ad = 0.1;  tau = 2;  FS = 14;  hatc = char(770);  abar = char(257);
cB = [0 0.25 0.85];  cBl = [0.6 0.72 1];  cG = [0 0.5 0.2];  cR = [0.85 0 0];
o = struct('theta_y', 20, 'h0', 15 / Rr, 'seeds', 1, 'T', T, 'aprime', true, 'tag', 'reg0');
oc = o;  oc.flow = true;  oc.hold_until = 0;  oc.t_scan = Inf;  oc.a_d = ad;  oc.tau_a = tau;
R = run_t3_xyz_fixed(oc);  rc = R.runs{1};  rc.aNe = rc.aNw;  rc.aTe = rc.aTw;  rc.apNe = rc.apNw;
oe = o;  oe.form = 'estimated';  oe.th0_deg = [0 0];  oe.reg = [ad tau];  oe.ap_q = true;
R = run_t3_xyz_fixed(oe);  re = R.runs{1};  re.aNe = re.aN;  re.aTe = re.aT;  re.apNe = re.apN;
runs = {rc, re};  ttl = {'computed angle', 'estimated angle'};
dh = 1e-3;  apN_true = @(h) (1 ./ local_cpe(h + dh) - 1 ./ local_cpe(h - dh)) / (2 * dh);
for c = 1:2
    r = runs{c};  k = 1:r.kend;  r.apN_tr = apN_true(max(r.h(k), 1.01));  runs{c} = r;
    t1 = find(r.aN_tr(k) <= ad + 0.1 * (r.aN_tr(1) - ad), 1);
    fprintf('%-16s | contact %d | min h %.2f um | true a_perp reaches 10%% of the step at t = %.2f s (exponential: %.2f s) | a_perp''-hat / true at t = 0.5, 1, 2 s: %s\n', ...
            ttl{c}, r.contact, min(r.h(k)) * Rr, r.t(t1), tau * log(10), mat2str(round(r.apNe([0.5 1 2] / (r.t(2) - r.t(1)) + 1)' ./ r.apN_tr([0.5 1 2] / (r.t(2) - r.t(1)) + 1)', 2)));
end
fig = figure('Color', 'w', 'Position', [60 60 1300 1350]);
tiledlayout(4, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for row = 1:4
    for c = 1:2
        nexttile;  hold on;  box on;  r = runs{c};  k = 1:r.kend;  tk = r.t(k);
        switch row
            case 1
                aid = ad + (r.aN_tr(1) - ad) * exp(-tk / tau);
                h1 = plot(tk, r.aN_tr(k), 'Color', cR, 'LineWidth', 3);  h2 = plot(tk, r.aNe(k), 'Color', cB, 'LineWidth', 1.4);
                h3 = yline(ad, 'k--', 'LineWidth', 1.5);  h4 = plot(tk, aid, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.5);
                legend([h1 h2 h3 h4], {[abar '_' char(8869)], [abar hatc '_' char(8869)], sprintf('%s_d = %.1f', abar, ad), 'exponential, \tau_a = 2 s'}, ...
                       'Location', 'northoutside', 'Orientation', 'horizontal');
                ylim([0 1.05]);  if c == 1; ylabel([abar '_' char(8869)]); end
                text(0.97, 0.88, ttl{c}, 'Units', 'normalized', 'HorizontalAlignment', 'right', 'FontSize', 13, 'FontWeight', 'bold');
            case 2
                h1 = plot(tk, r.apN_tr, 'Color', cR, 'LineWidth', 3);  h2 = plot(tk, r.apNe(k), 'Color', cB, 'LineWidth', 1.4);
                set(gca, 'YScale', 'log');  ylim([0.01 2]);
                legend([h1 h2], {[abar '''_' char(8869)], [abar hatc '''_' char(8869)]}, 'Location', 'northoutside', 'Orientation', 'horizontal');
                if c == 1; ylabel([abar '''_' char(8869)]); end
            case 3
                h1 = plot(tk, r.aT_tr(k), 'Color', cR, 'LineWidth', 3);  h2 = plot(tk, r.aTe(k), 'Color', cG, 'LineWidth', 1.4);
                legend([h1 h2], {[abar '_' char(8741)], [abar hatc '_' char(8741)]}, 'Location', 'northoutside', 'Orientation', 'horizontal');
                ylim([0 1.05]);  if c == 1; ylabel([abar '_' char(8741)]); end
            case 4
                if c == 2
                    fill([tk; flipud(tk)], [r.thy(k) + r.sPth(k, 2); flipud(r.thy(k) - r.sPth(k, 2))], cBl, 'EdgeColor', 'none', 'FaceAlpha', 0.5);
                end
                h1 = plot(tk, 20 + 0 * tk, 'Color', cR, 'LineWidth', 3);  plot(tk, 0 * tk, 'Color', cR, 'LineWidth', 3);
                h2 = plot(tk, r.thy(k), 'Color', cB, 'LineWidth', 1.4);  h3 = plot(tk, r.thx(k), 'Color', cG, 'LineWidth', 1.4);
                legend([h1 h2 h3], {[char(952) '_y,  ' char(952) '_x'], ['θ' hatc '_y'], ['θ' hatc '_x']}, 'Location', 'northoutside', 'Orientation', 'horizontal');
                ylim([-30 50]);  if c == 1; ylabel('\theta  [\circ]'); end;  xlabel('Time (sec)');
        end
        xlim([0 T]);  set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);
    end
end
exportgraphics(fig, fullfile(fo, 'fig_1025_reg0.png'), 'Resolution', 150);
close(fig);

function c = local_cpe(h)
    c = arrayfun(@(z) local_cpe1(z), h);
end

function c = local_cpe1(z)
    [~, c] = calc_correction_functions(z);
end
