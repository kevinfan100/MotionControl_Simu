% STATUS: ACTIVE (scratch figure) | PURPOSE: Var(a_m) against its formula in
%   the house layout of var_identity_binned.png (check_formC_var_identity),
%   for the two readout arms of readout_chain_record page 4, N = 100 each.
%
%   ONE page, one column per arm (shared y per row), Var(a_m) only:
%     row 1  cross-seed var at every sample vs time, formula overlaid
%     row 2  binned on a/a_o, seed-group error bars, formula line
%   measured = cross-seed variance at each sample k of (a_m[k] - a_true[k]),
%   formula  = K_var IF (a + xi)^2, both in (a/a_o)^2. The console also
%   prints the Var(y2) check (2 a_cov^2 (a + xi)^2) per bin. Error bars: G disjoint seed groups, std/sqrt(G)
%   (the project's arbiter; the across-seed distribution of a_m is not Gaussian).
%   y limits are shared across the two arms so the pages compare by eye.
%
%   ARMS = {O_struct, label; ...}   (O from run_formC_b, 100 seeds)
function out = plot_var_am_binned_arms(ARMS, ax, opts)

    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    if nargin < 2 || isempty(ax); ax = 3; end
    if nargin < 3; opts = struct(); end
    if ~isfield(opts, 'n_bin');  opts.n_bin  = 18; end
    if ~isfield(opts, 'G');      opts.G      = 5;  end          % seed groups for bars
    if ~isfield(opts, 'tlim');   opts.tlim   = [0.5 10.5]; end  % the ramp
    nA = size(ARMS, 1); D = cell(1, nA);

    for c = 1:nA
        O = ARMS{c,1}; nS = numel(O.runs); cc = O.runs{1}.ctrl_const;
        P = O.runs{1}.meta.params_value; fourkT = 4*P.ctrl.k_B*P.ctrl.T; s2n = P.ctrl.sigma2_noise(ax);
        xi = cc.C_n*s2n/(cc.C_dpmr*fourkT); Kvar = 2*cc.a_cov/(2-cc.a_cov);
        IFfun = @(a) 1 + 2*((fourkT*a).^2*cc.IF_abc(1) + 2*(fourkT*a)*s2n*cc.IF_abc(2) + s2n^2*cc.IF_abc(3)) ...
                        ./ (cc.C_dpmr*fourkT*a + cc.C_n*s2n).^2;
        t = O.runs{1}.tout(:); N = numel(t);
        AM = zeros(N, nS); AT = AM; ad = zeros(1, nS);
        for q = 1:nS
            r = O.runs{q}; ad(q) = r.a_hat_out(1,ax)/r.a_bar_hat_out(1,ax);
            AM(:,q) = r.a_xm_out(:,ax)/ad(q); AT(:,q) = r.a_true_out(:,ax)/ad(q);   % a/a_o units
        end
        a0 = mean(ad);
        Y2 = [nan(1,nS); AM(2:end,:) - (1-cc.a_cov)*AM(1:end-1,:)];
        E1 = AM - AT; E2 = Y2 - cc.a_cov*AT;
        at = mean(AT, 2);
        f1 = Kvar*IFfun(at*a0).*(at + xi/a0).^2;            % (a/a_o)^2
        f2 = 2*cc.a_cov^2*(at + xi/a0).^2;
        v1 = var(E1, 0, 2); v2 = var(E2, 0, 2);
        use = t >= opts.tlim(1) & t <= opts.tlim(2) & isfinite(v2);
        % binned on a/a_o with seed-group bars
        edges = linspace(min(at(use)), max(at(use)), opts.n_bin+1); ib = discretize(at, edges);
        grp = mod(0:nS-1, opts.G) + 1;
        B = struct('a', nan(opts.n_bin,1), 'v1', nan(opts.n_bin,1), 's1', nan(opts.n_bin,1), 'f1', nan(opts.n_bin,1), ...
                   'v2', nan(opts.n_bin,1), 's2', nan(opts.n_bin,1), 'f2', nan(opts.n_bin,1));
        for b = 1:opts.n_bin
            m = use & ib == b; if nnz(m) < 50; continue; end
            B.a(b) = mean(at(m)); B.f1(b) = mean(f1(m)); B.f2(b) = mean(f2(m));
            g1 = zeros(1,opts.G); g2 = g1;
            for g = 1:opts.G
                sel = grp == g;
                g1(g) = mean(var(E1(m,sel),0,2)); g2(g) = mean(var(E2(m,sel),0,2));
            end
            B.v1(b) = mean(v1(m)); B.s1(b) = std(g1)/sqrt(opts.G);
            B.v2(b) = mean(v2(m)); B.s2(b) = std(g2)/sqrt(opts.G);
        end
        D{c} = struct('t', t, 'at', at, 'v1', v1, 'f1', f1, 'v2', v2, 'f2', f2, 'B', B, 'use', use, ...
                      'label', ARMS{c,2}, 'nS', nS, 'a_pd', cc.a_pd, 'a_cov', cc.a_cov, 'Kvar', Kvar, 'IF', IFfun);
        fprintf('\n[%s]  N = %d, %d bins on a/a_o, bars = %d seed groups\n', ARMS{c,2}, nS, opts.n_bin, opts.G);
        fprintf('%6s %10s %10s %8s | %10s %10s %8s\n', 'a/a_o', 'Var(a_m)', 'formula', 'ratio', 'Var(y2)', 'formula', 'ratio');
        for b = 1:opts.n_bin
            if isnan(B.a(b)); continue; end
            fprintf('%6.3f %10.3e %10.3e %5.3f+-%.3f | %10.3e %10.3e %5.3f+-%.3f\n', B.a(b), B.v1(b), B.f1(b), ...
                    B.v1(b)/B.f1(b), B.s1(b)/B.f1(b), B.v2(b), B.f2(b), B.v2(b)/B.f2(b), B.s2(b)/B.f2(b));
        end
        r1 = B.v1./B.f1; r2 = B.v2./B.f2;
        fprintf('  all bins: Var(a_m)/formula mean %.3f (min %.3f max %.3f) | Var(y2)/formula mean %.3f (min %.3f max %.3f)\n', ...
                mean(r1,'omitnan'), min(r1), max(r1), mean(r2,'omitnan'), min(r2), max(r2));
    end
    if nA == 2
        r = D{1}.B.v1 ./ D{2}.B.v1; f = D{1}.B.f1 ./ D{2}.B.f1;
        fprintf('\narm1/arm2 Var(a_m) per bin: measured %.3f +- %.3f (sd over bins)   formula %.3f\n', ...
                mean(r,'omitnan'), std(r,'omitnan'), mean(f,'omitnan'));
    end

    % ---------------- ONE page: arms side by side, Var(a_m) only -------------
    %   row 1  cross-seed var(a_m) vs time, one column per arm, shared y
    %   row 2  binned on a/a_o with seed-group bars, shared y
    C_M = [0 0.2 0.9]; C_F = [0.10 0.60 0.20]; FS = 18; LFS = 13; AXLW = 2.0;
    y1 = 0; for c = 1:nA; y1 = max(y1, max(D{c}.v1(D{c}.use))); end; YL = [0 1.05*y1];
    f = figure('Position',[10 10 800*nA 1300],'Color','w','NumberTitle','off','Visible','off');
    tl = tiledlayout(f,2,nA,'TileSpacing','compact','Padding','compact'); A = gobjects(2,nA);
    for c = 1:nA
        d = D{c}; u = d.use;
        a = nexttile(tl, c); A(1,c) = a; hold(a,'on');
        h1 = plot(a, d.t(u), d.v1(u), '-', 'Color', C_M, 'LineWidth', 0.6);
        h2 = plot(a, d.t(u), d.f1(u), '-', 'Color', C_F, 'LineWidth', 2.8);
        legend(a,[h2 h1],{'formula', sprintf('measured  (cross-seed var, N = %d)    %s', d.nS, d.label)}, ...
               'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
        if c == 1; ylabel(a, 'var(a_m)   [(a/a_o)^2]', 'FontSize', FS, 'FontWeight', 'bold'); end
        xlabel(a, 'time  [s]', 'FontSize', FS, 'FontWeight', 'bold'); xlim(a, opts.tlim); ylim(a, YL);
        a = nexttile(tl, nA + c); A(2,c) = a; hold(a,'on');
        ok = ~isnan(d.B.a);
        h2 = plot(a, d.B.a(ok), d.B.f1(ok), '-', 'Color', C_F, 'LineWidth', 2.8);
        h1 = errorbar(a, d.B.a(ok), d.B.v1(ok), d.B.s1(ok), 'o', 'Color', C_M, 'MarkerFaceColor', C_M, ...
                      'MarkerSize', 7, 'LineWidth', 1.6, 'CapSize', 8);
        legend(a,[h2 h1],{'formula', sprintf('measured  (cross-seed var, N = %d)', d.nS)}, ...
               'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
        if c == 1; ylabel(a, 'var(a_m)   [(a/a_o)^2]', 'FontSize', FS, 'FontWeight', 'bold'); end
        xlabel(a, 'a / a_o   [-]', 'FontSize', FS, 'FontWeight', 'bold'); ylim(a, YL); xlim(a, [0 0.9]);
    end
    for k = 1:numel(A)
        set(A(k),'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid(A(k),'off');
    end
    for row = 1:2; for c = 2:nA; set(A(row,c), 'YTickLabel', []); end; end
    fns = {fullfile(od, 'var_am_binned_arms.png')};
    exportgraphics(f, fns{1}, 'Resolution', 150); close(f);
    fprintf('figure -> %s\n', fns{1});
    out = struct('D', {D}, 'files', {fns});
end
