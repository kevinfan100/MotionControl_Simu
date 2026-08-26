% STATUS: ACTIVE (scratch figure) | PURPOSE: the data points of ONE EWMA burst
%   and nothing else -- the input samples (dh_mr[k])^2 and the output
%   sigma2[k] of the recursion
%       sigma2[k+1] = (1 - a_cov) sigma2[k] + a_cov (dh_mr[k])^2
%   on the Meng ramp, production arm, seed 7, t 0.955-0.980 s (page 1 of
%   readout_chain_record, the t ~ 1 s peak). x axis = step k, k = 0 at the
%   first burst sample. The seven burst steps of the lower row carry the
%   injected amount a_cov (dh_mr[k])^2 -- the jump each step makes.
function out = plot_ewma_burst_zoom(mat, seed_idx, tzoom)

    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    if nargin < 1 || isempty(mat);      mat = fullfile(od, 'pair_both_arm.mat'); end
    if nargin < 2 || isempty(seed_idx); seed_idx = 1; end
    if nargin < 3 || isempty(tzoom);    tzoom = [0.955 0.980]; end
    S = load(mat); O = S.oBoth.O{1}; r = O.runs{seed_idx};
    a_pd = S.oBoth.ARMS{1}.a_pd;  a_cov = S.oBoth.ARMS{1}.a_cov;  ax = 3;

    t = r.tout(:); dhm = r.dh_m_out(:, ax); N = numel(dhm);
    dhmd = zeros(N+1, 1);
    for k = 1:N; dhmd(k+1) = a_pd*dhm(k) + (1-a_pd)*dhmd(k); end
    dhmr = dhm - dhmd(2:end);
    stat = t > 1 & t < 4;
    s2 = zeros(N+1, 1); s2(1) = mean(dhmr(stat).^2);
    for k = 1:N; s2(k+1) = a_cov*dhmr(k)^2 + (1-a_cov)*s2(k); end
    u = dhmr.^2;                                    % input of the recursion

    m = find(t >= tzoom(1) & t <= tzoom(2));
    k0 = m(find(abs(dhmr(m)) > 2*std(dhmr(stat)) & dhmr(m) < 0, 1));   % first burst sample
    kk = m - k0;                                     % step index, 0 at burst start
    burst = (kk >= 0 & kk <= 6);
    fprintf('seed %d  a_cov %.3g  k=0 at t=%.4f s ; %d steps shown\n', O.seeds(seed_idx), a_cov, t(k0), numel(m));
    fprintf('%4s %10s %12s %12s\n', 'k', 'u=dhmr^2', 'a_cov*u', 'sigma2[k+1]');
    for i = find(burst).'
        fprintf('%4d %10.3e %12.3e %12.3e\n', kk(i), u(m(i)), a_cov*u(m(i)), s2(m(i)+1));
    end

    C_U = [0 0.2 0.9]; C_S = [0.85 0.33 0.10];
    FS = 18; LFS = 14; AXLW = 2.0;
    f = figure('Position', [10 10 1500 1000], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(f, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    A = gobjects(2, 1);

    a = nexttile(tl); A(1) = a; hold(a, 'on');
    h = plot(a, kk, u(m), 'o', 'Color', C_U, 'MarkerFaceColor', C_U, 'MarkerSize', 7);
    legend(a, h, {'(\delta h_{mr}[k])^2'}, 'Location', 'northoutside', ...
           'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    ylabel(a, '(\delta h_{mr})^2  (\mum^2)', 'FontSize', FS, 'FontWeight', 'bold');

    a = nexttile(tl); A(2) = a; hold(a, 'on');
    h = plot(a, kk, s2(m+1), 'o', 'Color', C_S, 'MarkerFaceColor', C_S, 'MarkerSize', 7);
    % label = the injected amount a_cov*(dh_mr[k])^2, in the axis' 1e-3 units
    for i = find(burst).'
        lab = sprintf('+%.2f', 1e3 * a_cov * u(m(i)));
        switch kk(i)
            case 5;     pos = [kk(i)-0.15, s2(m(i)+1)+0.22e-3]; ha = 'right';  va = 'bottom';
            case 6;     pos = [kk(i)+0.15, s2(m(i)+1)+0.22e-3]; ha = 'left';   va = 'bottom';
            otherwise;  pos = [kk(i)-0.25, s2(m(i)+1)];         ha = 'right';  va = 'middle';
        end
        text(a, pos(1), pos(2), lab, 'FontSize', 13, 'FontWeight', 'bold', ...
             'HorizontalAlignment', ha, 'VerticalAlignment', va, 'Color', C_S);
    end
    legend(a, h, {sprintf(['\\^\\sigma^2[k+1] = (1 - a_{cov}) \\^\\sigma^2[k] + a_{cov} (\\delta h_{mr}[k])^2', ...
           '        a_{cov} = %.3g        labels: a_{cov} (\\delta h_{mr}[k])^2  (\\times10^{-3})'], a_cov)}, ...
           'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    ylabel(a, '\^\sigma^2  (\mum^2)', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel(a, sprintf('step k   (k = 0 at t = %.4f s, 1600 steps/s)', t(k0)), 'FontSize', FS, 'FontWeight', 'bold');

    for q = 1:2
        set(A(q), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');
        grid(A(q), 'off'); xlim(A(q), [kk(1)-0.5, kk(end)+0.5]);
        yl = ylim(A(q)); ylim(A(q), [0, yl(2)*1.35]);
        if q < 2; set(A(q), 'XTickLabel', []); end
    end
    fn = fullfile(od, 'ewma_burst_zoom.png');
    exportgraphics(f, fn, 'Resolution', 150); close(f);
    fprintf('figure -> %s\n', fn);
    out = struct('k', kk, 't', t(m), 'u', u(m), 's2', s2(m+1));
end
