function out = check_formC_y2_whiteness(S, opts)
%CHECK_FORMC_Y2_WHITENESS  Is the y2 measurement noise white, and if not, what
%   is the CORRECT inflation -- the one R(2,2) should be carrying?
%
%   S   = load('test_results/formC_cdpmr_var_check/raw_seeds.mat');  S = S.S400;
%   out = check_formC_y2_whiteness(S);
%
% STATUS: ACTIVE | fourth of the a_m readout audit (same 400-seed stack)
%
% WHY. R(2,2) currently equals the per-sample variance of y2 times IF = 3.3
% (measured: check_formC_var_am_vs_atrue / check_formC_if_eff). IF is derived as
% the variance inflation of the EWMA, and the Kalman filter it is handed to
% assumes WHITE measurement noise. Two different quantities are being conflated,
% so before deciding whether IF belongs there at all, three things have to be
% measured on the data rather than argued:
%
%   T1  WHITENESS. rho of the y2 innovation at lags 1..L inside a stationary
%       hold. If it is white, there is no correlation to compensate and IF
%       should simply not be there.
%   T3  THE CORRECT PENALTY. tau_int = 1 + 2*sum_{tau>=1} rho_v(tau) -- the
%       integrated autocorrelation time, i.e. how many samples one sample is
%       really worth. This is the number a Kalman filter fed correlated noise
%       must inflate R by, and it is NOT the same sum as IF.
%   T2  THE CLOSED FORM ITSELF. IF = 1 + 2*sum rho_u(tau)*s^tau with s = 1-a_cov
%       is measurable the same way. Comparing it with the analytic 3.366 tests
%       whether compute_if_abc is right, independently of where it is used.
%
% Wick's theorem is checked as a by-product: u is a squared Gaussian residual,
% so rho_u(tau) must equal rho_dwr(tau)^2. That identity is the basis of the
% whole IF derivation, so it is verified, not assumed.
%
% STATIONARITY. Autocorrelation is only meaningful on a stationary stretch, so
% everything is computed INSIDE the two holds (the command is fixed there) and
% the segment mean is removed per seed. During descent/oscillation a_true sweeps
% 11x and any autocorrelation would be measuring the trajectory, not the noise.
%
% INSTRUMENT CHECKS, written before the run:
%   C1  rho(0) = 1 by construction -- printed as a arithmetic check only.
%   C2  rho_u(tau) must equal rho_dwr(tau)^2 within the seed-group bar (Wick).
%   C3  the innovation and u must give the same rho: K2 is tiny (|K1|/|K2| = 43)
%       so the innovation is almost pure measurement noise. A difference would
%       mean the state correction is not negligible after all.

    if nargin < 2; opts = struct(); end
    if ~isfield(opts, 'ax');      opts.ax = 3;   end
    if ~isfield(opts, 'maxlag');  opts.maxlag = 80; end
    if ~isfield(opts, 'save_fig');opts.save_fig = true; end

    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    out_dir = fullfile(root, 'test_results', 'formC_cdpmr_var_check');

    ax = opts.ax;  K = S.K;  ns = numel(S.seeds);  ac = K.a_cov;  L = opts.maxlag;
    tt = S.t(:);
    kappa_T = 4 * (K.kBT / K.R) * K.a_o;
    sn = K.sigma2_n_s(ax) / K.R^2;

    dwr = squeeze(S.dx_r_out(:, ax, 1:ns)) / K.R;
    iy2 = squeeze(S.innov_y2_out(:, ax, 1:ns));
    at  = squeeze(S.a_true_out(:, ax, 1:ns)) / K.a_nom;
    u   = (dwr.^2 - K.C_n * sn) / (K.C_dpmr * kappa_T);

    HOLD = {'hold start', tt > 0.06 & tt < 0.50; ...
            'hold end',   tt > 3.70 & tt < 4.79};

    s_ew = 1 - ac;
    G = 20;  grp = mod((1:ns) - 1, G) + 1;
    R = struct();

    for h = 1:size(HOLD, 1)
        m = HOLD{h, 2};
        rho_u = local_acf(u(m,:),   L);
        rho_d = local_acf(dwr(m,:), L);
        rho_i = local_acf(iy2(m,:), L);

        lags = (1:L).';
        tau_int = 1 + 2 * sum(rho_u);                    % T3
        IF_meas = 1 + 2 * sum(rho_u .* s_ew.^lags);      % T2
        IF_from_innov = 1 + 2 * sum(rho_i .* s_ew.^lags);
        % closed form at the mean gain of this hold
        ab = mean(mean(at(m,:)));
        ABC = K.IF_abc(:);  sxT = kappa_T * ab;
        IF_cf = 1 + 2*(sxT^2*ABC(1) + 2*sxT*sn*ABC(2) + sn^2*ABC(3)) ...
                    / (K.C_dpmr*sxT + K.C_n*sn)^2;

        % seed-group bars on the two summed quantities
        gt = nan(G,1);  gi = nan(G,1);
        for gg = 1:G
            sel = grp == gg;
            ru = local_acf(u(m,sel), L);
            gt(gg) = 1 + 2*sum(ru);
            gi(gg) = 1 + 2*sum(ru .* s_ew.^lags);
        end

        fprintf('\n=== %s   (a/a_o = %.4f, %d samples x %d seeds) ===\n', ...
                HOLD{h,1}, ab, sum(m), ns);
        fprintf('  rho_u   at lag 1,2,5,10,20,40 : %+.4f %+.4f %+.4f %+.4f %+.4f %+.4f\n', ...
                rho_u([1 2 5 10 20 40]));
        fprintf('  rho_inv at lag 1,2,5,10,20,40 : %+.4f %+.4f %+.4f %+.4f %+.4f %+.4f\n', ...
                rho_i([1 2 5 10 20 40]));
        fprintf('  [C2 Wick]  max|rho_u - rho_dwr^2| over lags 1..%d = %.4f\n', ...
                L, max(abs(rho_u - rho_d.^2)));
        fprintf('  [C3]       max|rho_u - rho_innov|              = %.4f\n', ...
                max(abs(rho_u - rho_i)));
        fprintf('  T1 whiteness : rho_u(1) = %+.4f   (white would be 0 +- %.4f)\n', ...
                rho_u(1), 1/sqrt(sum(m)*ns));
        fprintf('  T2 IF        : measured %.4f  +-%.4f   closed form %.4f\n', ...
                IF_meas, std(gi)/sqrt(G), IF_cf);
        fprintf('  T3 tau_int   : measured %.4f  +-%.4f   (R2 currently uses %.4f)\n', ...
                tau_int, std(gt)/sqrt(G), IF_cf);
        fprintf('  from innovation instead of u: IF %.4f\n', IF_from_innov);

        R(h).name = HOLD{h,1};  R(h).rho_u = rho_u;  R(h).rho_d = rho_d;
        R(h).rho_i = rho_i;     R(h).tau_int = tau_int;  R(h).IF_meas = IF_meas;
        R(h).IF_cf = IF_cf;     R(h).se_tau = std(gt)/sqrt(G);
        R(h).se_IF = std(gi)/sqrt(G);  R(h).a = ab;  R(h).n = sum(m);
    end

    out = struct('R', R, 'maxlag', L, 'a_cov', ac, 'ns', ns);

    if opts.save_fig
        local_fig(R, L, s_ew, ns, fullfile(out_dir, 'y2_whiteness.png'));
        fprintf('\nfigure -> %s\n', out_dir);
    end
end

% =======================================================================
function rho = local_acf(x, L)
%LOCAL_ACF  Per-seed autocorrelation, averaged over seeds. The mean is removed
%   PER SEED (each seed has its own realized level), and each lag uses the
%   overlapping product normalized by the seed's own variance.
    x = x - mean(x, 1);
    v = mean(x.^2, 1);
    rho = zeros(L, 1);
    for q = 1:L
        c = mean(x(1:end-q, :) .* x(1+q:end, :), 1);
        rho(q) = mean(c ./ v);
    end
end

% =======================================================================
function local_fig(R, L, s_ew, ns, fpath)
    FS = 17; AXLW = 2.0;
    RED = [0.85 0.1 0.1];  BLU = [0 0.2 0.9];  GRY = [0.5 0.5 0.5];
    GRN = [0.1 0.6 0.2];
    lags = (1:L).';
    f = figure('Position',[40 40 1350 620],'Color','w','Visible','off');
    tl = tiledlayout(f, 1, 2, 'TileSpacing','compact','Padding','compact');

    for h = 1:numel(R)
        a = nexttile(tl, h); hold(a,'on'); box(a,'on');
        band = 2/sqrt(R(h).n * ns);
        yline(a, 0, '-', 'Color', GRY, 'LineWidth', 1.5, 'HandleVisibility','off');
        fill(a, [lags; flipud(lags)], [band*ones(L,1); -band*ones(L,1)], GRY, ...
             'FaceAlpha', 0.25, 'EdgeColor','none', 'DisplayName','\pm2 SE white band');
        plot(a, lags, R(h).rho_u, '-', 'Color', RED, 'LineWidth', 2.4, ...
             'DisplayName','\rho_u  readout noise');
        plot(a, lags, R(h).rho_d.^2, '--', 'Color', BLU, 'LineWidth', 2.0, ...
             'DisplayName','\rho_{dwr}^2  (Wick)');
        plot(a, lags, s_ew.^lags, ':', 'Color', GRN, 'LineWidth', 2.2, ...
             'DisplayName','s^\tau  EWMA weight');
        xlabel(a,'lag \tau [steps]'); ylabel(a,'\rho');
        legend(a,'Location','northoutside','Orientation','horizontal','FontSize',11);
        set(a,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW);
        text(a, 0.45, 0.80, sprintf('%s\n\\tau_{int} = %.2f\nIF meas %.2f / closed %.2f', ...
             R(h).name, R(h).tau_int, R(h).IF_meas, R(h).IF_cf), ...
             'Units','normalized','FontSize',13,'FontWeight','bold');
    end
    exportgraphics(f, fpath, 'Resolution', 150); close(f);
end
