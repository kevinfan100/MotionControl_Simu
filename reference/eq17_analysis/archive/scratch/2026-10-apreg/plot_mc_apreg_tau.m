% STATUS: SCRATCH -- 2026-10-18 (user): does the u-law regulation reach the set tau? With the law right, |u - u_d| ~ exp(-t/tau)
%   (u = 1/(1 - a) is nearly linear in h, so the same holds for |h - h*|). Per panel, log scale: black e^(-t/tau); blue what the law
%   sees |u_hat - u_d| / |u_hat(0) - u_d|, u_hat = 1/(1 - a_hat); red the truth |u_true - u_d| / |u_true(0) - u_d|.
%   tau_eff = (t_0.1 - t_0.9) / ln 9 from the first crossings of 0.9 and 0.1 (console, every seed).
%   Rows = estimator (a' state | w_s state), columns = tau 0.5 | 1.0 s. One figure per case: plane a_d 0.07 seed 1, cell a_d 0.60 seed 2.
%   Output test_results/temp_1009_ws/fig_mc_apreg_tau_<wall>_s<seed>.png.
function plot_mc_apreg_tau()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_1009_ws');
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  FS = 20;  LFS = 15;  AXLW = 2.2;  AB = char(257);  HAT = char(770);
    ARMS = {'stateu', [AB ''' state']; 'wsu', 'w_s state'};  TAUS = [0.5 1.0];  CASES = {'plane', '', 1; 'cell', '_ad060', 2};
    for ic0 = 1:2
        w = CASES{ic0, 1};  seed = CASES{ic0, 3};
        f = figure('Position', [40 40 1800 1200], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
        tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
        for ia = 1:2
            for it = 1:2
                tau = TAUS(it);  fn = fullfile(od, sprintf('apreg_%s%03d_%s%s.mat', ARMS{ia, 1}, round(100 * tau), w, CASES{ic0, 2}));
                ax = nexttile(tl, (ia - 1) * 2 + it);  hold on; box on;
                if ~exist(fn, 'file'); title(ax, 'missing'); continue; end
                S = load(fn);  ud = 1 / (1 - S.a_d);  t = S.t;
                for q = 1:numel(S.seeds)
                    if S.contact(q); fprintf('%-6s %-5s tau %.1f seed %d: contact\n', ARMS{ia, 1}, w, tau, S.seeds(q)); continue; end
                    eh = abs(1 ./ (1 - S.a_hat(:, q)) - ud);  et = abs(1 ./ (1 - S.a_true(:, q)) - ud);  eh = eh / eh(1);  et = et / et(1);
                    fprintf('%-6s %-5s tau %.1f seed %d: tau_eff/tau  law view %.2f | truth %.2f\n', ARMS{ia, 1}, w, tau, S.seeds(q), ...
                            local_teff(t, eh) / tau, local_teff(t, et) / tau);
                    if S.seeds(q) ~= seed; continue; end
                    hr = plot(ax, t, et, '-', 'Color', cT, 'LineWidth', 4.0, 'DisplayName', '|u - u_d|, truth');
                    hb = plot(ax, t, eh, '-', 'Color', cE, 'LineWidth', 1.6, 'DisplayName', ['|u' HAT ' - u_d|, law view']);
                end
                hk = plot(ax, t, exp(-t / tau), '--', 'Color', 'k', 'LineWidth', 1.8, 'DisplayName', 'e^{-t/\tau}');
                set(ax, 'YScale', 'log');  ylim(ax, [1e-3 2]);  xlim(ax, [0 4]);  set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(ax, 'off');
                if exist('hr', 'var') && isgraphics(hr)
                    lg = legend(ax, [hr hb hk], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
                    lg.Title.String = sprintf('%s wall, %s_d = %.2f, seed %d: %s, \\tau = %.1f s', w, AB, S.a_d, seed, ARMS{ia, 2}, tau);
                end
                if ia == 2; xlabel(ax, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold'); end
                if it == 1; ylabel(ax, 'normalized |u - u_d|', 'FontSize', FS, 'FontWeight', 'bold'); end
                clear hr hb
            end
        end
        out = fullfile(od, sprintf('fig_mc_apreg_tau_%s_s%d.png', w, seed));
        exportgraphics(f, out, 'Resolution', 150);  close(f);  fprintf('figure: %s\n', out);
    end
end

function te = local_teff(t, e)
    i9 = find(e < 0.9, 1);  i1 = find(e < 0.1, 1);  te = NaN;
    if ~isempty(i9) && ~isempty(i1); te = (t(i1) - t(i9)) / log(9); end
end
