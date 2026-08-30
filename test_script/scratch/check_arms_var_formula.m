% STATUS: ACTIVE (scratch instrument + figure) | PURPOSE: does the VARIANCE of
%   the gain readout match its formula, for both (a_pd, a_cov) arms of the
%   Meng ramp (page 4 of readout_chain_record)? Five checks, run in order,
%   each one a precondition of the next:
%     1  Var(dh_mr)            vs  C_dpmr 4kBT a + C_n sigma_n^2          (paper 12)
%     2  Var(dh_mr^2)          vs  2 Var(dh_mr)^2                          (Gaussian)
%     3  IF_emp = 1 + 2 sum rho(tau)(1-a_cov)^tau   vs  IF closed form (IF_abc)
%     4  Var(a_m)              vs  K_var IF (a + xi)^2,  K_var = 2a_cov/(2-a_cov)  <- main
%     5  Var(y2)               vs  2 a_cov^2 (a + xi)^2   and vs R2 (controller)
%   Everything in physical units (um, um/pN). a_m = a_xm_out (bit-exact with
%   the controller, rebuild_am_chain.m); y2 = a_m[k] - (1-a_cov) a_m[k-1].
%   Slow trends are removed with a_true before taking a variance.
%
%   Figure (one page, two arms side by side, y shared per row):
%     row 1  a_m (light blue) + a_true (red), seed 7            context
%     row 2  moving sd(a_m) (blue)  vs  sqrt(K_var IF) (a_true + xi) (red)
%     row 3  moving sd(y2)  (blue)  vs  sqrt(2) a_cov (a_true + xi) (red), sqrt(R2) grey dashed
%   Rows 2-3 are the mean over the 6 seeds; moving window 0.5 s.
%   Numbers go to the console: per-step ratios, mean +- SE over seeds,
%   two windows (far 1-4 s, near 8.5-10 s), and the arm1/arm2 ratio.
function out = check_arms_var_formula(mat, ax, opts)

    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    if nargin < 1 || isempty(mat); mat = fullfile(od, 'pair_both_arm.mat'); end
    if nargin < 2 || isempty(ax);  ax = 3; end
    if nargin < 3; opts = struct(); end
    if ~isfield(opts, 'tag');  opts.tag  = ''; end                % figure file suffix
    if ~isfield(opts, 'arms'); opts.arms = []; end                % which arms of the file to use
    % Windows must be STATIONARY: far-field 1-4 s and the trough hold
    % 10.5-12.5 s. The descent 8.5-10 s is not (a drops 0.35 -> 0.09 inside
    % it): the trend inflates the acf sum of dh_mr^2 and step 3 reads 1.4-3x
    % there while every other step is fine -- an instrument artefact, not a
    % formula failure (seen 2026-08-26).
    if ~isfield(opts, 'win');  opts.win  = [1 4; 10.5 12.5]; end   % analysis windows [s]
    if ~isfield(opts, 'Tmov'); opts.Tmov = 0.5; end               % moving-sd window [s]
    if ~isfield(opts, 'tlim'); opts.tlim = [0.5 10.5]; end
    % input: a pair_*.mat with oBoth.O{c} (a_pd/a_cov arms), a pair_if_*.mat
    % with out.O{c} (IF arms), or a struct with field O directly.
    if ischar(mat) || isstring(mat)
        S = load(mat);
        if isfield(S, 'oBoth'); OO = S.oBoth.O; elseif isfield(S, 'out'); OO = S.out.O; else; error('unknown mat layout'); end
    else
        OO = mat.O;
    end
    if ~isempty(opts.arms); OO = OO(opts.arms); end
    nA = numel(OO); nS = numel(OO{1}.runs);

    % ---------------- per arm, per seed: chain and the five ratios ----------
    A = cell(1, nA);
    for c = 1:nA
        O = OO{c}; cc = O.runs{1}.ctrl_const;
        P = O.runs{1}.meta.params_value; fourkT = 4*P.ctrl.k_B*P.ctrl.T;   % [pN um]
        s2n = P.ctrl.sigma2_noise(ax);                                     % [um^2]
        xi  = cc.C_n*s2n/(cc.C_dpmr*fourkT);                               % [um/pN]
        Kvar = 2*cc.a_cov/(2-cc.a_cov);
        IFfun = @(a) 1 + 2*((fourkT*a).^2*cc.IF_abc(1) + 2*(fourkT*a)*s2n*cc.IF_abc(2) + s2n^2*cc.IF_abc(3)) ...
                        ./ (cc.C_dpmr*fourkT*a + cc.C_n*s2n).^2;
        arm = struct('label', sprintf('a_{pd} %.4g , a_{cov} %.4g', cc.a_pd, cc.a_cov), ...
                     'a_pd', cc.a_pd, 'a_cov', cc.a_cov, 'C_dpmr', cc.C_dpmr, 'C_n', cc.C_n, ...
                     'xi', xi, 'Kvar', Kvar, 'Kvar_cc', cc.K_var, 'fourkT', fourkT, 's2n', s2n, 'IFfun', IFfun);
        R = cell(1, nS);
        for q = 1:nS
            r = O.runs{q}; t = r.tout(:); Ts = median(diff(t));
            dhm = r.dh_m_out(:, ax);  N = numel(dhm);
            md = zeros(N+1, 1); for k = 1:N; md(k+1) = (1-cc.a_pd)*md(k) + cc.a_pd*dhm(k); end
            dhmr = dhm - md(2:end);
            am  = r.a_xm_out(:, ax);                                       % [um/pN]
            at  = r.a_true_out(:, ax);
            ad  = r.a_hat_out(1, ax)/r.a_bar_hat_out(1, ax);
            R2  = r.R2_out(:, ax)*ad^2;                                    % [ (um/pN)^2 ]
            y2  = [NaN; am(2:end) - (1-cc.a_cov)*am(1:end-1)];
            nW = round(opts.Tmov/Ts);
            R{q} = struct('t', t, 'dhm', dhm, 'dhmr', dhmr, 'am', am, 'at', at, 'y2', y2, 'R2', R2, ...
                          'sd_am', movstd(am - at, nW), 'sd_y2', movstd(y2 - cc.a_cov*at, nW, 'omitnan'), ...
                          'f_am', sqrt(Kvar*IFfun(at)).*(at + xi), 'f_y2', sqrt(2)*cc.a_cov*(at + xi), 'sqR2', sqrt(R2));
        end
        arm.R = R; A{c} = arm;
    end

    % step 0: how different are the two loops? a_pd/a_cov change y2 -> a_hat
    % -> control force, so the arms share the noise realisation but NOT the
    % trajectory (max|diff| ~ 4.5e-3 um, 2026-08-26). Printed for the record;
    % a value of exactly 0 would mean the readout is not wired into the law.
    if nA >= 2
        d0 = zeros(1, nS);
        for q = 1:nS; d0(q) = max(abs(A{1}.R{q}.dhm - A{2}.R{q}.dhm)); end
        fprintf('step 0  max|dh_m arm1 - arm2| over seeds = %.2e um  (same noise, different loop; 0 would be a wiring fault)\n', max(d0));
    end

    % ---------------- ratios per window ------------------------------------
    names = {'1  Var(dh_mr)/formula(12)', '2  Var(dh_mr^2)/(2 Var^2)', '3  IF_emp/IF_abc', ...
             '4  Var(a_m)/[K_var IF (a+xi)^2]', '5a Var(y2)/[2 a_cov^2 (a+xi)^2]', '5b Var(y2)/R2'};
    out = struct('arms', {A}, 'ratio', zeros(6, nA, size(opts.win,1), nS));
    for w = 1:size(opts.win, 1)
        fprintf('\n=== window %.1f-%.1f s  (mean +- SE over %d seeds) ===\n', opts.win(w,:), nS);
        fprintf('%-36s', 'step'); for c = 1:nA; fprintf(' %-22s', A{c}.label); end; fprintf('\n');
        for c = 1:nA
            arm = A{c};
            for q = 1:nS
                r = arm.R{q}; m = r.t >= opts.win(w,1) & r.t <= opts.win(w,2);
                a_bar = mean(r.at(m));
                v1 = var(r.dhmr(m)); f1 = arm.C_dpmr*arm.fourkT*a_bar + arm.C_n*arm.s2n;
                v2 = var(r.dhmr(m).^2); f2 = 2*v1^2;
                L = ceil(-log(1e-3)/(-log(1-arm.a_cov)));               % (1-a_cov)^L = 1e-3
                rho = local_acf(r.dhmr(m).^2, L);
                IFe = 1 + 2*sum(rho .* (1-arm.a_cov).^(1:L)); f3 = mean(arm.IFfun(r.at(m)));
                v4 = var(r.am(m) - r.at(m)); f4 = mean(arm.Kvar*arm.IFfun(r.at(m)).*(r.at(m)+arm.xi).^2);
                v5 = var(r.y2(m) - arm.a_cov*r.at(m), 'omitnan'); f5 = mean(2*arm.a_cov^2*(r.at(m)+arm.xi).^2);
                f5b = mean(r.R2(m));
                out.ratio(:, c, w, q) = [v1/f1; v2/f2; IFe/f3; v4/f4; v5/f5; v5/f5b];
            end
        end
        for s = 1:6
            fprintf('%-36s', names{s});
            for c = 1:nA
                x = squeeze(out.ratio(s, c, w, :));
                fprintf(' %6.3f +- %-13.3f', mean(x), std(x)/sqrt(nS));
            end
            fprintf('\n');
        end
        if nA >= 2
            x1 = zeros(1,nS); x2 = x1; f1 = x1; f2 = x1;
            for q = 1:nS
                for c = 1:2
                    r = A{c}.R{q}; m = r.t >= opts.win(w,1) & r.t <= opts.win(w,2);
                    v = var(r.am(m) - r.at(m)); f = mean(A{c}.Kvar*A{c}.IFfun(r.at(m)).*(r.at(m)+A{c}.xi).^2);
                    if c == 1; x1(q) = v; f1(q) = f; else; x2(q) = v; f2(q) = f; end
                end
            end
            fprintf('%-36s measured %.3f +- %.3f   formula %.3f\n', 'arm1/arm2  Var(a_m)', ...
                    mean(x1./x2), std(x1./x2)/sqrt(nS), mean(f1./f2));
        end
    end
    fprintf('\nconstants: '); for c = 1:nA
        fprintf('[%s] C_dpmr %.4f C_n %.4f xi %.3e um/pN K_var %.5f (cc %.5f) IF(far) %.2f | ', ...
                A{c}.label, A{c}.C_dpmr, A{c}.C_n, A{c}.xi, A{c}.Kvar, A{c}.Kvar_cc, A{c}.IFfun(mean(A{c}.R{1}.at(A{c}.R{1}.t>1&A{c}.R{1}.t<4))));
    end; fprintf('\n');

    % ---------------- figure ------------------------------------------------
    C_T = [0.8 0 0]; C_E = [0 0.2 0.9]; C_M = [0.55 0.78 1.0]; C_G = [0.45 0.45 0.45];
    FS = 18; LFS = 13; AXLW = 2.0;
    f = figure('Position', [10 10 800*nA 1500], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(f, 3, nA, 'TileSpacing', 'compact', 'Padding', 'compact');
    G = gobjects(3, nA);
    for c = 1:nA
        arm = A{c}; r1 = arm.R{1}; t = r1.t; m = t >= opts.tlim(1) & t <= opts.tlim(2);
        % seed mean of a per-run column: stack runs as ROWS (nS x N), then average
        M = @(fld) mean(cell2mat(cellfun(@(r) r.(fld)(:).', arm.R(:), 'UniformOutput', false)), 1).';
        sd_am = M('sd_am'); sd_y2 = M('sd_y2'); f_am = M('f_am'); f_y2 = M('f_y2'); sqR2 = M('sqR2');

        a = nexttile(tl, c); G(1,c) = a; hold(a, 'on');
        h1 = plot(a, t(m), r1.am(m), '-', 'Color', C_M, 'LineWidth', 0.8);
        h2 = plot(a, t(m), r1.at(m), '-', 'Color', C_T, 'LineWidth', 2.4);
        legend(a, [h1 h2], {['a_m      ' arm.label], 'a  true'}, 'Location', 'northoutside', ...
               'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        if c == 1; ylabel(a, 'a  (\mum/pN)', 'FontSize', FS, 'FontWeight', 'bold'); end

        a = nexttile(tl, nA + c); G(2,c) = a; hold(a, 'on');
        h1 = plot(a, t(m), sd_am(m), '-', 'Color', C_E, 'LineWidth', 1.8);
        h2 = plot(a, t(m), f_am(m),  '-', 'Color', C_T, 'LineWidth', 2.4);
        legend(a, [h1 h2], {sprintf('sd(a_m)   %.2g s window', opts.Tmov), ...
               '[K_{var} IF]^{1/2} (a + \xi)'}, 'Location', 'northoutside', ...
               'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        if c == 1; ylabel(a, 'sd(a_m)  (\mum/pN)', 'FontSize', FS, 'FontWeight', 'bold'); end

        a = nexttile(tl, 2*nA + c); G(3,c) = a; hold(a, 'on');
        h1 = plot(a, t(m), sd_y2(m), '-', 'Color', C_E, 'LineWidth', 1.8);
        h2 = plot(a, t(m), f_y2(m),  '-', 'Color', C_T, 'LineWidth', 2.4);
        h3 = plot(a, t(m), sqR2(m), '--', 'Color', C_G, 'LineWidth', 1.6);
        legend(a, [h1 h2 h3], {sprintf('sd(y_2)   %.2g s window', opts.Tmov), ...
               '2^{1/2} a_{cov} (a + \xi)', 'R_2^{1/2}'}, 'Location', 'northoutside', ...
               'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        if c == 1; ylabel(a, 'sd(y_2)  (\mum/pN)', 'FontSize', FS, 'FontWeight', 'bold'); end
        xlabel(a, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    end
    for row = 1:3
        yl = [inf -inf];
        for c = 1:nA; l = ylim(G(row,c)); yl = [min(yl(1), l(1)), max(yl(2), l(2))]; end
        for c = 1:nA
            set(G(row,c), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');
            grid(G(row,c), 'off'); xlim(G(row,c), opts.tlim); ylim(G(row,c), [0 yl(2)]);
            if row < 3; set(G(row,c), 'XTickLabel', []); end
            if c > 1;   set(G(row,c), 'YTickLabel', []); end
        end
    end
    fn = fullfile(od, sprintf('arms_var_formula%s.png', opts.tag));
    exportgraphics(f, fn, 'Resolution', 150); close(f);
    fprintf('figure -> %s\n', fn);
end

function rho = local_acf(x, L)
    x = x(:) - mean(x); d = sum(x.^2); rho = zeros(1, L);
    for l = 1:L; rho(l) = sum(x(1+l:end).*x(1:end-l))/d; end
end
