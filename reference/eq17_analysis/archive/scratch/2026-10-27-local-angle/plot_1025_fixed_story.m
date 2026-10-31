% STATUS: SCRATCH -- 2026-10-25 (user: show directly why the angle is slow at a fixed point). 1025 estimated angle (run_t3_xyz_fixed,
%   form 'estimated', theta-hat[0] = 0, default prior), 20 deg wall, fixed point (no motion), 20 s, runs temp_t3_est{50,15,5}_s{1,2,3}.
%   Columns = 50 / 15 / 5 um. Read top to bottom:
%   Row 1 (the cloud): tracking error delta p (x, z) [nm], seed 1; red = shape of the true A-bar (semi-axes ~ sqrt of the gains, scaled to the
%     cloud), red dashed = wall normal. The angle is written only in the shape of the cloud.
%   Row 2 (the data alone, no estimator): running mean from t = 0 of delta x * delta z in units of a-bar (scale s = mean(dx^2 + dz^2) /
%     (a_xx + a_zz) of the run); red = the value it settles to, a_xz = -(a_par - a_perp) sin theta cos theta; grey = +- 2 sd of the running
%     mean (sd of one product / sqrt(steps / IF)). Seed 1 bold, 2-3 light.
%   Row 3 (the estimator): theta-hat_y - theta_y with +- sqrt P (seed 1 band); seed 1 bold, 2-3 light.
%   Console: per column, when the data alone resolves the tilt (2 sd < |a_xz|) and when the estimator's sqrt P reaches 5 deg.
%   -> test_results/temp_3d_wsonly/fig_1025_fixed_story.png | run: plot_1025_fixed_story()
function plot_1025_fixed_story()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_3d_wsonly');  Rnm = 2250;
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  cL = [0.65 0.75 0.95];  cB = [0.85 0.88 0.97];  cS = [0.85 0.85 0.85];  FS = 15;  LFS = 11;
    TH = char(952);  HT = char(770);  DG = char(176);  AB = char(257);  PE = char(8869);  PA = char(8741);  MU = char(181);
    GAP = [50 15 5];  SD = 1:3;
    f = figure('Position', [40 40 1800 1350], 'Color', 'w', 'Visible', 'off');
    tl = tiledlayout(3, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ic = 1:3
        R = cell(1, 3);  sd = [];
        for s = SD
            fn = fullfile(od, sprintf('temp_t3_est%d_s%d.mat', GAP(ic), s));
            if exist(fn, 'file'); S = load(fn);  R{s} = S.R.runs{1};  sd(end + 1) = s; end   %#ok<AGROW>   % seeds that exist
        end
        so = [setdiff(sd, 1), 1];
        r = R{1};  k = (1:r.kend)';  t = r.t(k);  ty = 20;  apa = mean(r.aT_true(k));  ape = mean(r.aN_true(k));
        % ---- row 1: cloud ----
        ax = nexttile(tl, ic);  hold on;  box on;
        d = r.dp(k, [1 3]) * Rnm;  d = d - mean(d);
        plot(d(1:4:end, 1), d(1:4:end, 2), '.', 'Color', cL, 'MarkerSize', 3, 'DisplayName', 'tracking error  \delta p  in the x-z plane');
        V2 = [cosd(ty) sind(ty); -sind(ty) cosd(ty)];  [Ev, Dv] = eig(V2 * diag([apa ape]) * V2');  ph = linspace(0, 2 * pi, 300);
        el = Ev * sqrt(Dv) * [cos(ph); sin(ph)];  el = el * 2 * sqrt(mean(var(d))) / sqrt(mean(diag(Dv)));
        plot(el(1, :), el(2, :), '-', 'Color', cT, 'LineWidth', 2.5, 'DisplayName', ['shape of the true  ' char(256)]);
        Lq = 1.15 * max(abs(el(:)));  plot([-1 1] * Lq * sind(ty), [-1 1] * Lq * cosd(ty), '--', 'Color', cT, 'LineWidth', 1.5, 'DisplayName', 'wall normal');
        axis equal;  lim = 1.3 * Lq;  xlim([-lim lim]);  ylim([-lim lim]);
        text(0.03, 0.95, sprintf('%d %sm\n%s_%s - %s_%s = %.3f', GAP(ic), MU, AB, PA, AB, PE, apa - ape), 'Units', 'normalized', ...
             'VerticalAlignment', 'top', 'FontSize', FS - 1, 'FontWeight', 'bold', 'BackgroundColor', 'w');
        xlabel('\delta x  [nm]', 'FontSize', FS - 2, 'FontWeight', 'bold');  if ic == 1; ylabel('\delta z  [nm]', 'FontSize', FS - 2, 'FontWeight', 'bold'); end
        if ic == 2; legend('Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold'); end
        set(ax, 'FontSize', FS - 3, 'FontWeight', 'bold', 'LineWidth', 1.5);
        % ---- row 2: running mean of dx dz (data only) ----
        ax = nexttile(tl, 3 + ic);  hold on;  box on;  axz = -(apa - ape) * sind(ty) * cosd(ty);  tres = NaN(1, 3);
        for s = so
            rr = R{s};  kk = (1:rr.kend)';  tt = rr.t(kk);  dd = rr.dp(kk, [1 3]);  dd = dd - mean(dd);
            sc = mean(dd(:, 1).^2 + dd(:, 2).^2) / (apa * cosd(ty)^2 + ape * sind(ty)^2 + apa * sind(ty)^2 + ape * cosd(ty)^2);
            pr = dd(:, 1) .* dd(:, 2) / sc;  n = (1:numel(kk))';  ra = cumsum(pr) ./ n;
            c = xcorr(pr - mean(pr), 20, 'coeff');  IF = 1 + 2 * sum(c(22:41));  band = 2 * std(pr) ./ sqrt(n / max(IF, 1));
            if s == 1
                hf = fill([tt; flipud(tt)], [axz + band; flipud(axz - band)], cS, 'EdgeColor', 'none', 'DisplayName', 'noise left in the mean  (\pm2\sigma)');  uistack(hf, 'bottom');
                plot(tt, ra, '-', 'Color', cE, 'LineWidth', 1.6, 'DisplayName', 'mean of  \delta x \delta z  from t = 0  (data only),  seed 1');
            else
                hl = plot(tt, ra, '-', 'Color', cL, 'LineWidth', 1, 'DisplayName', 'seeds 2-3');  if s ~= so(1); hl.HandleVisibility = 'off'; end
            end
            i2 = find(band < abs(axz), 1);  if ~isempty(i2); tres(s) = tt(i2); end
        end
        plot([0 20], [axz axz], '-', 'Color', cT, 'LineWidth', 2.5, 'DisplayName', [AB '_{xz}  it settles to  (true)']);
        plot([0 20], [0 0], ':', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.2, 'HandleVisibility', 'off');
        xlim([0 20]);  ylim([-0.2 0.1]);
        if ic == 1; ylabel(['[' AB ']'], 'FontSize', FS - 2, 'FontWeight', 'bold'); end
        if ic == 2; legend('Location', 'northoutside', 'NumColumns', 2, 'FontSize', LFS, 'FontWeight', 'bold'); end
        set(ax, 'FontSize', FS - 3, 'FontWeight', 'bold', 'LineWidth', 1.5);
        % ---- row 3: estimator ----
        ax = nexttile(tl, 6 + ic);  hold on;  box on;  t5 = NaN(1, 3);
        for s = so
            rr = R{s};  kk = (1:rr.kend)';  tt = rr.t(kk);  e = rr.thy(kk) - ty;  sp = rr.sPth(kk, 2);
            if s == 1
                hf = fill([tt; flipud(tt)], [sp; -flipud(sp)], cB, 'EdgeColor', 'none', 'DisplayName', '\pm\surdP  (the estimator''s own uncertainty)');  uistack(hf, 'bottom');
                plot(tt, e, '-', 'Color', cE, 'LineWidth', 1.6, 'DisplayName', [TH HT '_y - ' TH '_y  (estimator),  seed 1']);
            else
                hl = plot(tt, e, '-', 'Color', cL, 'LineWidth', 1, 'DisplayName', 'seeds 2-3');  if s ~= so(1); hl.HandleVisibility = 'off'; end
            end
            i5 = find(sp < 5, 1);  if ~isempty(i5); t5(s) = tt(i5); end
        end
        plot([0 20], [0 0], '-', 'Color', cT, 'LineWidth', 2.5, 'HandleVisibility', 'off');
        xlim([0 20]);  ylim([-30 15]);  xlabel('Time (sec)', 'FontSize', FS - 2, 'FontWeight', 'bold');
        if ic == 1; ylabel([TH '_y error  [' DG ']'], 'FontSize', FS - 2, 'FontWeight', 'bold'); end
        if ic == 2; legend('Location', 'northoutside', 'NumColumns', 2, 'FontSize', LFS, 'FontWeight', 'bold'); end
        set(ax, 'FontSize', FS - 3, 'FontWeight', 'bold', 'LineWidth', 1.5);
        fprintf('%2d um: a_par - a_perp %.3f, a_xz %.4f | data alone resolves the tilt (2 sd < |a_xz|) at %s s | estimator sqrt P < 5 deg at %s s\n', ...
                GAP(ic), apa - ape, axz, mat2str(round(tres, 1)), mat2str(round(t5, 1)));
    end
    out = fullfile(od, 'fig_1025_fixed_story.png');  exportgraphics(f, out, 'Resolution', 150);  close(f);  fprintf('figure: %s\n', out);
end
