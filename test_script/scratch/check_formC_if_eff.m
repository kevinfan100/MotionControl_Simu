function out = check_formC_if_eff(S, opts)
%CHECK_FORMC_IF_EFF  Where does the factor 3.3 between R(2,2) and the measured
%   var(y2) come from, and does IF_eff really fall to 2.79 near the wall?
%
%   S   = load('test_results/formC_cdpmr_var_check/raw_seeds.mat');  S = S.S400;
%   out = check_formC_if_eff(S);
%
% STATUS: ACTIVE | follow-up to check_formC_var_am_vs_atrue (same 400-seed stack)
%
% THE CLOSED FORM the controller evaluates every step
% (motion_control_law_formC_b.m:1531 compute_R2_formB, constants from
%  build_eq17_6state_constants.m:184 compute_if_abc):
%
%   s     = 1 - a_cov                                  EWMA memory factor
%   sxT   = kappa_T * a_bar                            thermal share of the residual
%   IF    = 1 + 2*(sxT^2*A + 2*sxT*sn*B + sn^2*C) / (C_dpmr*sxT + C_n*sn)^2
%   K_var = 2*a_cov/(2-a_cov)
%   R2_int= amlpf * K_var * IF * (a_bar + xi_bar)^2      = Var(a_bar_wm)
%   R2    = a_cov*(2-a_cov)*R2_int + dscale*a_cov^2*d*Q44
%
% with A = sum_{tau>=1} R_fT(tau)^2 s^tau, B = sum R_fT R_fN s^tau,
% C = sum R_fN^2 s^tau -- the s-weighted autocorrelation sums of the high-pass
% residual, so IF is exactly 1 + 2*sum_tau rho^2(tau) s^tau: the variance
% inflation of an EWMA fed CORRELATED squares. IF = 1 would be white squares.
%
% THE ARITHMETIC THIS SCRIPT TESTS. y2 = a_cov*u[k] is a SINGLE sample, so its
% variance carries no correlation inflation:
%       Var(y2) = a_cov^2 * Var(u) = 2 * a_cov^2 * (a_bar + xi_bar)^2
% while the code builds R2 from Var(a_bar_wm) (which contains IF) times the
% white-driven AR(1) increment factor a_cov*(2-a_cov). Those two steps assume
% opposite things about the drive. If that is what is happening, then
%       var(y2)_measured / R2  =  1 / (amlpf * IF)
% exactly, and the measured ratio inverted must reproduce the closed form
% point by point. That is the test.

    if nargin < 2; opts = struct(); end
    if ~isfield(opts, 'ax');       opts.ax = 3;   end
    if ~isfield(opts, 'n_bin');    opts.n_bin = 18; end
    if ~isfield(opts, 'save_fig'); opts.save_fig = true; end

    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    out_dir = fullfile(root, 'test_results', 'formC_cdpmr_var_check');

    ax = opts.ax;  K = S.K;  ns = numel(S.seeds);  ac = K.a_cov;
    tt = S.t(:);
    kappa_T = 4 * (K.kBT / K.R) * K.a_o;
    sn      = K.sigma2_n_s(ax) / K.R^2;
    xi      = K.xi_per_axis(ax);
    ABC     = K.IF_abc(:);

    apb = squeeze(S.a_prime_out(:, ax, 1:ns)) / K.a_nom;   % d a_bar / d w_bar
    Q33 = squeeze(S.Q33_out(:, ax, 1:ns));
    am  = squeeze(S.a_xm_out(:, ax, 1:ns)) / K.a_nom;
    at  = squeeze(S.a_true_out(:, ax, 1:ns)) / K.a_nom;
    ah  = squeeze(S.a_bar_hat_out(:, ax, 1:ns));
    R2  = squeeze(S.R2_out(:, ax, 1:ns));

    y2 = am(2:end,:) - (1 - ac) * am(1:end-1,:);
    y2(1,:) = NaN;
    kk = 2:numel(tt);
    tt = tt(kk); at = at(kk,:); ah = ah(kk,:); R2 = R2(kk,:);
    apb = apb(kk,:); Q33 = Q33(kk,:);
    % R2's second term, rebuilt from the documented Q44 = a_bar'^2 * Q33
    dly = mean(ac^2 * K.d * (apb.^2) .* Q33, 2);
    f_dly = dly ./ mean(R2, 2);

    IFf = @(a) 1 + 2 * ((kappa_T*a).^2 * ABC(1) + 2*(kappa_T*a)*sn*ABC(2) + sn^2*ABC(3)) ...
                   ./ (K.C_dpmr*(kappa_T*a) + K.C_n*sn).^2;

    m_at = mean(at, 2);
    IF_hat  = mean(IFf(ah), 2);          % what the controller actually used
    IF_true = IFf(m_at);                 % same form evaluated at the truth
    ratio   = var(y2, 0, 2) ./ mean(R2, 2);
    IF_meas = 1 ./ ratio;                % = amlpf * IF if the algebra above holds

    % the white-drive prediction, built from constants only -- no fitting
    var_y2_white = 2 * ac^2 * (m_at + xi).^2;
    r_white = var(y2, 0, 2) ./ var_y2_white;

    fprintf('\nconstants: a_cov %.3f  xi_bar %.5g  IF_abc [%.4g %.4g %.4g]  C_dpmr %.4f  C_n %.4f\n', ...
            ac, xi, ABC(1), ABC(2), ABC(3), K.C_dpmr, K.C_n);
    fprintf('kappa_T %.4g   sigma2_n_bar %.4g   sn/(kappa_T) = %.4g [a_o units]\n', ...
            kappa_T, sn, sn/kappa_T);

    SEG = {'hold start', tt > 0.05 & tt < 0.50; ...
           'descend',    tt > 0.55 & tt < 1.45; ...
           'oscillate',  tt > 1.60 & tt < 3.40; ...
           'hold end',   tt > 3.70};
    fprintf('\n%-11s %8s %10s %10s %10s %10s %10s\n', 'segment', 'a/a_o', ...
            'IF(a_hat)', 'IF(a_true)', '1/ratio', 'var/white', 'delay/R2');
    for q = 1:size(SEG,1)
        m = SEG{q,2};
        fprintf('%-11s %8.4f %10.4f %10.4f %10.4f %10.4f %10.4f\n', SEG{q,1}, ...
                mean(m_at(m)), mean(IF_hat(m)), mean(IF_true(m)), ...
                mean(IF_meas(m)), mean(r_white(m)), mean(f_dly(m)));
    end

    % IF over the whole reachable range of a, from the closed form alone
    ag = logspace(log10(0.05), log10(1.0), 40).';
    fprintf('\n%8s %10s\n', 'a/a_o', 'IF closed');
    for q = 1:6:numel(ag)
        fprintf('%8.4f %10.4f\n', ag(q), IFf(ag(q)));
    end
    fprintf('IF limits:  a -> 0 : %.4f    a -> inf : %.4f\n', IFf(1e-9), IFf(1e9));

    out = struct('f_dly', f_dly, 't', tt, 'm_at', m_at, 'IF_hat', IF_hat, 'IF_true', IF_true, ...
                 'IF_meas', IF_meas, 'r_white', r_white, 'ag', ag, 'IFg', IFf(ag));

    if opts.save_fig
        local_fig(out, fullfile(out_dir, 'if_eff_audit.png'));
        fprintf('\nfigure -> %s\n', out_dir);
    end
end

% =======================================================================
function local_fig(o, fpath)
    FS = 17; AXLW = 2.0;
    RED = [0.85 0.1 0.1];  BLU = [0 0.2 0.9];  GRN = [0.1 0.6 0.2];
    GRY = [0.5 0.5 0.5];
    f = figure('Position',[40 40 1350 560],'Color','w','Visible','off');
    tl = tiledlayout(f, 1, 2, 'TileSpacing','compact','Padding','compact');

    a1 = nexttile(tl,1); hold(a1,'on'); box(a1,'on');
    plot(a1, o.ag, o.IFg, '-', 'Color', RED, 'LineWidth', 2.4, ...
         'DisplayName','IF closed form');
    plot(a1, o.m_at, o.IF_meas, '.', 'Color', BLU, 'MarkerSize', 7, ...
         'DisplayName','1 / (var(y_2)/R_{22})  measured');
    set(a1,'XScale','log');
    xlabel(a1,'a / a_o'); ylabel(a1,'IF  [-]');
    legend(a1,'Location','northoutside','Orientation','horizontal','FontSize',12);
    set(a1,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW);

    a2 = nexttile(tl,2); hold(a2,'on'); box(a2,'on');
    yline(a2, 1, '-', 'Color', GRY, 'LineWidth', 1.5, 'HandleVisibility','off');
    plot(a2, o.t, o.r_white, '-', 'Color', GRN, 'LineWidth', 2.0, ...
         'DisplayName','var(y_2) measured / 2 a_{cov}^2 (a+\xi)^2');
    ylim(a2,[0 2]);
    xlabel(a2,'t [s]'); ylabel(a2,'[-]');
    legend(a2,'Location','northoutside','Orientation','horizontal','FontSize',12);
    set(a2,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW);

    exportgraphics(f, fpath, 'Resolution', 150); close(f);
end
