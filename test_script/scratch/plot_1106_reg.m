function S = plot_1106_reg(reg, seeds, tag, ropts)
% PURPOSE: 1-D plane, a-bar' state estimator, Meng law, PI gain regulation from t = 0 at 15 um (run_1106_meng_law_1d opts.reg).
%   Console: contact, smallest gap, |a - a_d| (true gain) by window, e_a, a'-hat / a'. Figure (seed of the first entry): style of
%   plot_mc_apdiff.m -- z_d / z / z* [um], a-bar_z (a_d dashed), a-bar'_z (log, negatives not drawn), delta z_m [um].
% EXPIRES: with run_1106_meng_law_1d.m.
    if nargin < 1 || isempty(reg); reg = struct('a_d', 0.05, 'tau_p', 0.5, 'tau_i', 0.5); end
    if nargin < 2 || isempty(seeds); seeds = 1:5; end
    if nargin < 3; tag = ''; end
    if nargin < 4; ropts = struct(); end                      % extra run_1106_meng_law_1d options (e.g. ym_true); ropts.T = run length [s]
    addpath(genpath(fullfile(fileparts(mfilename('fullpath')), '..', '..', 'model')));
    pc = physical_constants();  R_UM = pc.R;  T = 4;  if isfield(ropts, 'T'); T = ropts.T;  ropts = rmfield(ropts, 'T'); end;  h0 = 15 / R_UM;
    hs = fzero(@(h) local_a(h) - reg.a_d, [1.001 30]);
    o = struct('seeds', seeds, 'laws', {{'meng'}}, 'h0', h0, 'h1', h0, 'T_move', 1, 'T_hold', T - 1, 'reg', reg);
    fn = fieldnames(ropts);  for i = 1:numel(fn); o.(fn{i}) = ropts.(fn{i}); end
    R = run_1106_meng_law_1d(o);
    W = [0 1; 1 2; 2 T];  n = numel(seeds);  S = struct('contact', 0, 'gap', nan(n, 1), 'dev', nan(n, size(W, 1)), 'bias', nan(n, size(W, 1)), 'ea', nan(n, size(W, 1)), 'apr', nan(n, size(W, 1)));
    fprintf('\n=== Meng law, PI regulation from t = 0 at %.2f R: a_d %.3f (h* %.4f R, gap %.0f nm), tau_P %.2f s, tau_I %.2f s, seeds %s ===\n', ...
            h0, reg.a_d, hs, (hs - 1) * R_UM * 1e3, reg.tau_p, reg.tau_i, mat2str(seeds));
    for is = 1:n
        r = R.meng(is);  m = r.a > 0;  S.contact = S.contact + r.contact;  S.gap(is) = (min(r.x(m)) - 1) * R_UM * 1e3;
        for iw = 1:size(W, 1)
            w = m & r.t >= W(iw, 1) & r.t < W(iw, 2);  if ~any(w); continue; end
            S.dev(is, iw) = mean(abs(r.a(w) - reg.a_d));  S.bias(is, iw) = mean(r.a(w) - reg.a_d);  S.ea(is, iw) = mean(r.a(w) - r.ah(w));
            S.apr(is, iw) = median(r.aph(w) ./ r.ap(w));
        end
        fprintf('seed %d: contact %d  smallest gap %6.1f nm\n', seeds(is), r.contact, S.gap(is));
    end
    fprintf('contact %d / %d\n%-8s %22s %22s %22s %14s\n', S.contact, n, 'window', 'mean|a - a_d|', 'mean (a - a_d)', 'mean e_a', 'a''-hat/a''');
    for iw = 1:size(W, 1)
        fprintf('%g-%g s   %10.4f (sd %.4f)   %+10.4f (sd %.4f)   %+10.4f (sd %.4f)   %8.2f\n', W(iw, :), mean(S.dev(:, iw), 'omitnan'), std(S.dev(:, iw), 'omitnan'), ...
                mean(S.bias(:, iw), 'omitnan'), std(S.bias(:, iw), 'omitnan'), mean(S.ea(:, iw), 'omitnan'), std(S.ea(:, iw), 'omitnan'), median(S.apr(:, iw), 'omitnan'));
    end
    % ---- figure, first seed ----
    r = R.meng(1);  m = r.a > 0;  t = r.t(m);
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  cM = [0.55 0.75 1.0];  cG = [0.3 0.3 0.3];  FS = 22;  LFS = 17;  AXLW = 2.2;
    AB = char(257);  HAT = char(770);  az = [AB '_z'];
    f = figure('Position', [40 40 1100 2000], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');  ax = gobjects(4, 1);
    ax(1) = nexttile(tl);  hold on; box on;
    hd = plot(ax(1), t, r.xd(m) * R_UM, '-', 'Color', cT, 'LineWidth', 5.0, 'DisplayName', 'z_d');
    hz = plot(ax(1), t, r.x(m) * R_UM, '--', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', 'z');
    hh = yline(ax(1), hs * R_UM, ':', 'Color', cG, 'LineWidth', 2.0, 'DisplayName', 'z^*');
    yline(ax(1), R_UM, '-', 'Color', cG, 'LineWidth', 1.0, 'HandleVisibility', 'off');
    ylim(ax(1), [0 16]);
    lg = legend(ax(1), [hd hz hh], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
    lg.Title.String = sprintf('plane wall, Meng law, %s_d = %.2f, seed %d', AB, reg.a_d, seeds(1));
    ax(2) = nexttile(tl);  hold on; box on;
    hh2 = plot(ax(2), t, r.ah(m), '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', [AB HAT '_z']);
    hdd = yline(ax(2), reg.a_d, '--', 'Color', cG, 'LineWidth', 2.0, 'DisplayName', [AB '_d']);
    ylim(ax(2), [-0.2 1.6]);
    legend(ax(2), [hh2 hdd], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
    ax(3) = nexttile(tl);  hold on; box on;
    aph = r.aph(m);  aph(aph <= 0) = NaN;
    ht3 = plot(ax(3), t, r.ap(m), '-', 'Color', cT, 'LineWidth', 5.0, 'DisplayName', [AB '''_z']);
    hh3 = plot(ax(3), t, aph, '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', [AB HAT '''_z']);
    set(ax(3), 'YScale', 'log');  ylim(ax(3), [1e-4 3]);  yticks(ax(3), [1e-4 1e-2 1]);
    legend(ax(3), [ht3 hh3], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
    ax(4) = nexttile(tl);  hold on; box on;
    plot(ax(4), t, r.dxm(m) * R_UM, '-', 'Color', cM, 'LineWidth', 1.0);
    ylim(ax(4), [-0.2 0.2]);  yticks(ax(4), [-0.1 0 0.1]);
    xlabel(ax(4), 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    ylabel(ax(1), ['z  [' char(181) 'm]'], 'FontSize', FS, 'FontWeight', 'bold');
    ylabel(ax(2), az, 'FontSize', FS + 2, 'FontWeight', 'bold');
    ylabel(ax(3), [AB '''_z'], 'FontSize', FS + 2, 'FontWeight', 'bold');
    ylabel(ax(4), [char(948) 'z_m  [' char(181) 'm]'], 'FontSize', FS, 'FontWeight', 'bold');
    for q = 1:4; xlim(ax(q), [0 T]);  set(ax(q), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(ax(q), 'off'); end
    od = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'test_results', 'temp_1106_meng');  if ~exist(od, 'dir'); mkdir(od); end
    out = fullfile(od, sprintf('fig_1106_reg%s_s%d.png', tag, seeds(1)));  exportgraphics(f, out, 'Resolution', 150);  close(f);  fprintf('figure: %s\n', out);
end

function a = local_a(h)
    [~, cp] = calc_correction_functions(max(h, 1.0005));  a = 1 / cp;
end
