function res = analyze_formB_Jb_coupling(seed)
%ANALYZE_FORMB_JB_COUPLING  Does the sign change of J_b actually cancel the
%   filter's b-to-gain coupling on the production trajectory?
%
% STATUS: ACTIVE -- 08-06 probe: J_b's sign change does NOT cancel the b-to-gain coupling on
%   the production trajectory (the cancellation seen is M changing sign).
%   See memory project-formb-amplitude-writing-reparam-2026-08-06.
%
% THE CLAIM UNDER TEST.  Form B's slope Jacobian J_b = d(a_bar')/db changes
%   sign at gap = b.  The predict step feeds it into the (b, a_bar) block of P:
%
%       P(b,a) += P(b,b) * J_b * M ,      M = Dw_d[k-1] + (1-lc)*dw3_hat[k]
%
%   and the y2 update moves b by  K_b * innov,  K_b ~ P(b,a)*H2(a)/S2.
%   So if J_b flips sign mid-run, the two halves add opposite-signed increments
%   to P(b,a), it is driven back toward zero, and b stops responding to the
%   data.  Form C has no zero in J_b (J_b = a_bar'/b > 0 always), so this
%   cannot happen to it -- that part is algebra and needs no test.  What DOES
%   need testing is whether the production trajectory actually crosses B's zero
%   and whether P(b,a) visibly turns there.
%
% SCOPE: OFFLINE.  Runs the canonical t1 arm once to capture (cfg, ctrl_const),
%   replays the same seed through the driver's test-ladder entry with
%   log_P_full so P is available, and reads P(5,4) straight out of the log.
%   Nothing under model/ is touched.  t1 (w_s locked) is used on purpose: the
%   locked rows of P are exact zeros, so P(b,p) and P(b,ws) cross-terms cannot
%   confound the reading.
%
% OUTPUT  figures/formB_Jb_coupling.png  -- two stacked panels sharing time:
%   top    J_b along the run, with its zero crossings marked
%   bottom P(b, a_bar) straight from the logged covariance
%   If the bottom curve turns where the top crosses zero, the claim holds.
%
%   See also: analyze_formB_correction_anatomy, plot_formB_form_compare

    if nargin < 1 || isempty(seed); seed = 1; end
    AX = 3;

    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    fig_dir = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');

    arm = run_formB_ws(struct('seeds', seed, 'tier', 't1'));
    cfg = arm.cfg;  cc = arm.runs{1}.ctrl_const;
    ref = run_formB_ws(cfg, struct('seed', seed, 'ctrl_const_override', cc, ...
                                   'log_P_full', true));
    e_rep = max(abs(ref.b_hat_out(:,AX) - arm.runs{1}.b_hat_out(:,AX)));
    fprintf('\n[Jb coupling]  replay check: max |b_hat replay - arm| = %.3e\n', e_rep);

    N  = numel(ref.tout);  t = ref.tout(:);
    wd = ref.h_bar_d_out(:); wd(1) = wd(2);
    b_cl = local_f(cc, 'b_clamp', [0.05 5]);
    p_cl = local_f(cc, 'p_clamp', [0.05 5]);
    gfl  = local_f(cc, 'ws_margin', 1e-3);
    ws0  = local_f(cc, 'ws0_perp', 1);

    % prior row = what x_curr held when step k ran
    b_pri  = [NaN; ref.b_hat_out(1:end-1,AX)];
    p_pri  = [NaN; ref.p_hat_out(1:end-1,AX)];
    ws_pri = [NaN; ref.ws_hat_out(1:end-1,AX)];

    % ---- J_b, exactly as the controller computes it ----------------------
    Jb = zeros(N,1); gap = zeros(N,1);
    for k = 3:N
        lb = min(max(b_pri(k), b_cl(1)), b_cl(2));
        lp = min(max(p_pri(k), p_cl(1)), p_cl(2));
        g  = max(wd(k) - (ws_pri(k) + ws0 - 1), gfl);
        ap = lp * (1 + g/lb)^(-lp-1) / lb;
        Jb(k)  = (-1/lb + (lp+1)*g/(lb*(g+lb))) * ap;   % controller line 1142
        gap(k) = g;
    end

    % ---- the covariance element the claim is about ----------------------
    Pba = squeeze(ref.P_full_out(:, 5, 4, AX));         % P(b, a_bar)
    Pbb = squeeze(ref.P_full_out(:, 5, 5, AX));
    open2 = ~ref.gate_out(:,AX);

    % zero crossings of J_b within the run
    m  = (3:N).';
    zc = m(Jb(m(1:end-1)).*Jb(m(2:end)) < 0);
    fprintf('  J_b zero crossings at t = %s  (gap = %s)\n', ...
        mat2str(round(t(zc).',3)), mat2str(round(gap(zc).',3)));
    fprintf('  gap range visited: [%.3f, %.3f];  b_hat range [%.4f, %.4f]\n', ...
        min(gap(m)), max(gap(m)), min(b_pri(3:end)), max(b_pri(3:end)));
    [~, ipk] = max(abs(Pba));
    fprintf('  |P(b,a)| peaks at t = %.3f (value %+.3e), ends at %+.3e\n', ...
        t(ipk), Pba(ipk), Pba(end));
    fprintf('  P(b,b): %.4e -> %.4e  (shrink %.2f%%)\n', ...
        Pbb(2), Pbb(end), 100*(1 - Pbb(end)/Pbb(2)));

    % ---- figure ---------------------------------------------------------
    COL_J = [0 0.2 0.9]; COL_P = [0.8 0 0]; COL_R = [0.45 0.45 0.45];
    FS = 20; LFS = 15; AXLW = 2.0; LW = 2.2;
    f = figure('Position', [80 80 1100 820], 'Color', 'w', ...
               'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    nexttile; hold on;
    h1 = plot(t(m), Jb(m), '-', 'Color', COL_J, 'LineWidth', LW, ...
              'DisplayName', '$J_b$');
    yline(0, '-', 'Color', COL_R, 'LineWidth', 1.0, 'HandleVisibility', 'off');
    for i = 1:numel(zc)
        xline(t(zc(i)), '--', 'Color', COL_R, 'LineWidth', 1.5, 'HandleVisibility', 'off');
    end
    ylabel('$J_b$', 'Interpreter', 'latex', 'FontSize', FS);
    legend(h1, 'Interpreter', 'latex', 'Location', 'northoutside', ...
           'Orientation', 'horizontal', 'FontSize', LFS, 'Box', 'on');
    set(gca, 'FontSize', FS, 'LineWidth', AXLW, 'Box', 'on', ...
             'TickLabelInterpreter', 'latex', 'XTickLabel', []);
    xlim([t(3) t(end)]); grid off;

    nexttile; hold on;
    h2 = plot(t, Pba, '-', 'Color', COL_P, 'LineWidth', LW, ...
              'DisplayName', '$P(b,\bar{a})$');
    yline(0, '-', 'Color', COL_R, 'LineWidth', 1.0, 'HandleVisibility', 'off');
    for i = 1:numel(zc)
        xline(t(zc(i)), '--', 'Color', COL_R, 'LineWidth', 1.5, 'HandleVisibility', 'off');
    end
    xlabel('$t$ \ [s]', 'Interpreter', 'latex', 'FontSize', FS);
    ylabel('$P(b,\bar{a})$', 'Interpreter', 'latex', 'FontSize', FS);
    legend(h2, 'Interpreter', 'latex', 'Location', 'northoutside', ...
           'Orientation', 'horizontal', 'FontSize', LFS, 'Box', 'on');
    set(gca, 'FontSize', FS, 'LineWidth', AXLW, 'Box', 'on', ...
             'TickLabelInterpreter', 'latex');
    xlim([t(3) t(end)]); grid off;

    out = fullfile(fig_dir, 'formB_Jb_coupling.png');
    exportgraphics(f, out, 'Resolution', 200, 'Padding', 'figure');
    close(f);
    fprintf('  wrote %s\n', out);

    res = struct('t', t, 'Jb', Jb, 'Pba', Pba, 'Pbb', Pbb, 'gap', gap, ...
                 'zc', zc, 'open2', open2);
end

% --------------------------------------------------------------------------
function v = local_f(s, f, dflt)
    v = dflt; if isfield(s, f) && ~isempty(s.(f)); v = s.(f); end
end
