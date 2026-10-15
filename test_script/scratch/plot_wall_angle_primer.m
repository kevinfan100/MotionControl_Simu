% STATUS: SCRATCH -- 2026-10-14, primer figures for estimating the wall angle from the tracking-error covariance
%   (Estimator_and_Control__.pdf, reference/controller_paper_source/). User 10-14: fig_wall_angle_model.png is the STARTING
%   POINT of this explanation (ellipse of the random component, long axis = wall, plus the assumptions); the three-numbers
%   figure is the next step. Synthetic Gaussian points only (no closed loop), a_par = 1, a_perp = 0.3, theta_y = 20 deg.
%   Sign convention of the notes (p.6): V = R_y(theta_y) R_x(theta_x); theta_y > 0 tilts the wall down toward +x.
%   which: 'model' | 'three' | 'frames' | 'sign' | 'all'. Output: test_results/figs_unused/fig_wall_angle_<which>.png
function plot_wall_angle_primer(which)
    if nargin < 1; which = 'all'; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    fd = fullfile(root, 'test_results', 'figs_unused');  if ~exist(fd, 'dir'); mkdir(fd); end
    P = {'model', 'three', 'frames', 'sign', 'backwards'};  if ~strcmp(which, 'all'); P = {which}; end
    for i = 1:numel(P); feval(['local_' P{i}], fd); end
end

function c = C()
    c = struct('E', [0 0.25 0.85], 'T', [0.85 0 0], 'G', [0.3 0.65 0.2], 'M', [0.72 0.82 0.98], 'K', [0.55 0.55 0.55], 'FS', 16);
end

function local_model(fd)                             % the starting point: (a) wall frame, (b) measured frame, (c) far field
    c = C();  rng(2);  n = 1500;  ph = linspace(0, 2 * pi, 200);
    S = {1, 0.3, 0, '(a) 牆的座標：橢圓沿著牆', '沿牆 u', '法向 w'; 1, 0.3, 20, '(b) 量測座標：同一個橢圓斜 \theta_y', 'x', 'z'; ...
         1, 1, 20, '(c) 遠場 a_{||} = a_\perp：圓', 'x', 'z'};
    f = figure('Position', [40 40 1900 680], 'Color', 'w', 'Visible', 'off');  tl = tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    for i = 1:3
        ap = S{i,1};  an = S{i,2};  th = S{i,3} * pi / 180;  R = [cos(th) sin(th); -sin(th) cos(th)];   % columns: wall direction, normal
        ax = nexttile(tl);  hold(ax, 'on');  axis(ax, 'equal');  xlim(ax, [-3 3]);  ylim(ax, [-3 3]);  box(ax, 'on');
        q = R * [sqrt(ap) * randn(1, n); sqrt(an) * randn(1, n)];  plot(ax, q(1,:), q(2,:), '.', 'Color', c.M, 'MarkerSize', 5);
        xline(ax, 0, '-', 'Color', c.K);  yline(ax, 0, '-', 'Color', c.K);
        plot(ax, 3 * [-R(1,1) R(1,1)], 3 * [-R(2,1) R(2,1)], '-', 'Color', c.T, 'LineWidth', 3);
        e = R * (2 * [sqrt(ap) * cos(ph); sqrt(an) * sin(ph)]);  plot(ax, e(1,:), e(2,:), '-', 'Color', c.E, 'LineWidth', 2.5);
        a1 = R * [2 * sqrt(ap); 0];  a2 = R * [0; 2 * sqrt(an)];
        quiver(ax, 0, 0, a1(1), a1(2), 0, 'Color', c.E, 'LineWidth', 2.5, 'MaxHeadSize', 0.25);
        quiver(ax, 0, 0, a2(1), a2(2), 0, 'Color', c.E, 'LineWidth', 2.5, 'MaxHeadSize', 0.4);
        text(ax, a1(1) - 0.2, a1(2) - 0.35, '\propto\surda_{||}', 'Color', c.E, 'FontSize', c.FS, 'FontWeight', 'bold');
        text(ax, a2(1) + 0.1, a2(2) + 0.25, '\propto\surda_\perp', 'Color', c.E, 'FontSize', c.FS, 'FontWeight', 'bold');
        xlabel(ax, S{i,5});  ylabel(ax, S{i,6});  title(ax, S{i,4}, 'FontSize', c.FS);
        text(ax, -2.85, -2.6, '紅線 = 牆', 'Color', c.T, 'FontSize', c.FS - 2, 'FontWeight', 'bold');
        set(ax, 'FontSize', c.FS, 'FontWeight', 'bold', 'LineWidth', 1.6);
    end
    local_save(f, fd, 'fig_wall_angle_model');
end

function local_three(fd)                             % the ellipse = three numbers, and how they move with theta_y
    c = C();  ap = 1;  an = 0.3;  t = (0:0.5:90)' * pi / 180;
    Cxx = cos(t).^2 * ap + sin(t).^2 * an;  Czz = sin(t).^2 * ap + cos(t).^2 * an;  Cxz = -sin(t) .* cos(t) * (ap - an);
    f = figure('Position', [40 40 1800 680], 'Color', 'w', 'Visible', 'off');  tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    rng(2);  th = 20 * pi / 180;  R = [cos(th) sin(th); -sin(th) cos(th)];  q = R * [sqrt(ap) * randn(1, 1500); sqrt(an) * randn(1, 1500)];
    ph = linspace(0, 2 * pi, 200);  i20 = find(abs(t * 180 / pi - 20) < 1e-9);
    ax = nexttile(tl);  hold(ax, 'on');  axis(ax, 'equal');  xlim(ax, [-3 3]);  ylim(ax, [-3 3]);  box(ax, 'on');
    plot(ax, q(1,:), q(2,:), '.', 'Color', c.M, 'MarkerSize', 5);  xline(ax, 0, '-', 'Color', c.K);  yline(ax, 0, '-', 'Color', c.K);
    plot(ax, 3 * [-R(1,1) R(1,1)], 3 * [-R(2,1) R(2,1)], '-', 'Color', c.T, 'LineWidth', 3);
    e = R * (2 * [sqrt(ap) * cos(ph); sqrt(an) * sin(ph)]);  plot(ax, e(1,:), e(2,:), '-', 'Color', c.E, 'LineWidth', 2.5);
    text(ax, -2.85, 2.6, sprintf('平均(x^2)  = %.2f', Cxx(i20)), 'FontSize', c.FS, 'FontWeight', 'bold', 'Color', c.E);
    text(ax, -2.85, 2.1, sprintf('平均(z^2)  = %.2f', Czz(i20)), 'FontSize', c.FS, 'FontWeight', 'bold', 'Color', c.G);
    text(ax, -2.85, 1.6, sprintf('平均(x\\cdotz) = %.2f', Cxz(i20)), 'FontSize', c.FS, 'FontWeight', 'bold', 'Color', c.T);
    xlabel(ax, 'x');  ylabel(ax, 'z');  title(ax, '\theta_y = 20°：橢圓 = 三個數', 'FontSize', c.FS);  set(ax, 'FontSize', c.FS, 'FontWeight', 'bold', 'LineWidth', 1.6);
    ax2 = nexttile(tl);  hold(ax2, 'on');  box(ax2, 'on');  d = t * 180 / pi;
    h1 = plot(ax2, d, Cxx, '-', 'Color', c.E, 'LineWidth', 3, 'DisplayName', '平均(x^2)');
    h2 = plot(ax2, d, Czz, '-', 'Color', c.G, 'LineWidth', 3, 'DisplayName', '平均(z^2)');
    h3 = plot(ax2, d, Cxz, '-', 'Color', c.T, 'LineWidth', 3, 'DisplayName', '平均(x\cdotz)');
    h4 = plot(ax2, d, zeros(size(d)), '--', 'Color', c.T, 'LineWidth', 2, 'DisplayName', '平均(x\cdotz)，遠場 a_{||} = a_\perp');
    xline(ax2, 20, ':', 'Color', [0.4 0.4 0.4], 'LineWidth', 2);  yline(ax2, 0, '-', 'Color', [0.6 0.6 0.6]);
    xlabel(ax2, '牆的角度 \theta_y (deg)');  ylim(ax2, [-0.45 1.1]);  xlim(ax2, [0 90]);  xticks(ax2, 0:15:90);
    legend(ax2, [h1 h2 h3 h4], 'Location', 'northoutside', 'NumColumns', 2, 'FontSize', 13);  set(ax2, 'FontSize', c.FS, 'FontWeight', 'bold', 'LineWidth', 1.6);
    local_save(f, fd, 'fig_wall_angle_three_numbers');
end

function local_frames(fd)                            % 3-D: measurement axes x y z and wall axes u v w for the two angles
    c = C();  FS = 15;  cV = [0 0.6 0.2];
    Vf = @(tx, ty) [cos(ty), sin(ty) * sin(tx), sin(ty) * cos(tx); 0, cos(tx), -sin(tx); -sin(ty), cos(ty) * sin(tx), cos(ty) * cos(tx)];
    S = {30, 0, '\theta_x = 30°, \theta_y = 0  (繞 x 軸)'; 0, 30, '\theta_x = 0, \theta_y = 30°  (繞 y 軸)'; 30, 30, '\theta_x = 30°, \theta_y = 30°'};
    f = figure('Position', [40 40 1900 720], 'Color', 'w', 'Visible', 'off');  tl = tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    for i = 1:3
        V = Vf(S{i,1} * pi / 180, S{i,2} * pi / 180);  u = V(:,1);  v = V(:,2);  w = V(:,3);
        ax = nexttile(tl);  hold(ax, 'on');  s = 0.9;  Q = [s*u + s*v, s*u - s*v, -s*u - s*v, -s*u + s*v];
        patch(ax, Q(1,:), Q(2,:), Q(3,:), [0.85 0.85 0.85], 'FaceAlpha', 0.5, 'EdgeColor', [0.5 0.5 0.5]);
        E = eye(3);  nm = {'x', 'y', 'z'};
        for j = 1:3
            quiver3(ax, 0, 0, 0, E(1,j), E(2,j), E(3,j), 0, 'k--', 'LineWidth', 1.5, 'MaxHeadSize', 0.3);
            text(ax, 1.12 * E(1,j), 1.12 * E(2,j), 1.12 * E(3,j), nm{j}, 'FontSize', FS, 'FontWeight', 'bold');
        end
        A = {u, v, w};  CC = {c.E, cV, c.T};  L = {'u (沿牆)', 'v (沿牆)', 'w (法向)'};
        for j = 1:3
            quiver3(ax, 0, 0, 0, A{j}(1), A{j}(2), A{j}(3), 0, 'Color', CC{j}, 'LineWidth', 3, 'MaxHeadSize', 0.3);
            text(ax, 1.18 * A{j}(1), 1.18 * A{j}(2), 1.18 * A{j}(3), L{j}, 'Color', CC{j}, 'FontSize', FS, 'FontWeight', 'bold');
        end
        view(ax, 55, 20);  axis(ax, 'equal');  xlim(ax, [-1.1 1.3]);  ylim(ax, [-1.1 1.3]);  zlim(ax, [-0.7 1.3]);  axis(ax, 'off');
        title(ax, S{i,3}, 'FontSize', FS + 1);
    end
    local_save(f, fd, 'fig_wall_angle_frames');
end

function local_sign(fd)                              % side views with the positive direction of each angle (right-hand rule)
    c = C();  th = 25 * pi / 180;  cA = [0.3 0.3 0.3];  FS = 17;  cV = [0 0.6 0.2];
    S = {'y', 'z', 'x', char(8857), 'v', +1, '\theta_x > 0：繞 x 軸（x 指出紙面）', cV; 'x', 'z', 'y', char(8855), 'u', -1, '\theta_y > 0：繞 y 軸（y 指入紙面）', c.E};
    f = figure('Position', [40 40 1700 780], 'Color', 'w', 'Visible', 'off');  tl = tiledlayout(1, 2, 'TileSpacing', 'loose', 'Padding', 'compact');
    for i = 1:2
        sg = S{i,6};  a = sg * th;  cH = S{i,8};
        ax = nexttile(tl);  hold(ax, 'on');  axis(ax, 'equal');  axis(ax, 'off');  xlim(ax, [-1.35 1.45]);  ylim(ax, [-0.9 1.4]);
        h = [cos(a) sin(a)];  n = [-sin(a) cos(a)];
        plot(ax, 1.3 * [-h(1) h(1)], 1.3 * [-h(2) h(2)], '-', 'Color', [0.6 0.6 0.6], 'LineWidth', 7);
        quiver(ax, 0, 0, 1.1, 0, 0, 'k--', 'LineWidth', 1.6, 'MaxHeadSize', 0.12);  text(ax, 1.17, 0, S{i,1}, 'FontSize', FS + 2, 'FontWeight', 'bold');
        quiver(ax, 0, 0, 0, 1.1, 0, 'k--', 'LineWidth', 1.6, 'MaxHeadSize', 0.12);  text(ax, -0.03, 1.2, S{i,2}, 'FontSize', FS + 2, 'FontWeight', 'bold');
        text(ax, -0.25, -0.13, [S{i,4} ' ' S{i,3}], 'FontSize', FS + 4, 'FontWeight', 'bold');
        quiver(ax, 0, 0, h(1), h(2), 0, 'Color', cH, 'LineWidth', 3.5, 'MaxHeadSize', 0.18);
        text(ax, 1.08 * h(1), 1.08 * h(2) + sg * 0.09, [S{i,5} ' (沿牆)'], 'Color', cH, 'FontSize', FS, 'FontWeight', 'bold');
        quiver(ax, 0, 0, n(1), n(2), 0, 'Color', c.T, 'LineWidth', 3.5, 'MaxHeadSize', 0.18);
        text(ax, n(1) - 0.05, n(2) + 0.1, 'w (法向)', 'Color', c.T, 'FontSize', FS, 'FontWeight', 'bold');
        for r = [0.72 0.55]
            if r == 0.72; b0 = 0; else; b0 = pi / 2; end
            t = linspace(b0, b0 + a, 40);  plot(ax, r * cos(t), r * sin(t), '-', 'Color', cA, 'LineWidth', 2.2);
            pe = r * [cos(t(end)) sin(t(end))];  tg = sign(a) * [-sin(t(end)) cos(t(end))];  nr = [cos(t(end)) sin(t(end))];  L = 0.07;
            tri = [pe; pe - L * tg + 0.5 * L * nr; pe - L * tg - 0.5 * L * nr];  patch(ax, tri(:,1), tri(:,2), cA, 'EdgeColor', cA);
            tm = b0 + a / 2;  text(ax, (r + 0.1) * cos(tm), (r + 0.1) * sin(tm), ['\theta_' S{i,3}], 'FontSize', FS - 2, 'FontWeight', 'bold', 'Color', cA);
        end
        title(ax, S{i,7}, 'FontSize', FS + 1);
    end
    local_save(f, fd, 'fig_wall_angle_sign');
end

function local_backwards(fd)                         % read the three numbers backwards: a right triangle with angle 2 theta_y
    c = C();  ap = 1;  an = 0.3;  dl = ap - an;  TH = [0 20 45 70];  ph = linspace(0, 2 * pi, 300);
    f = figure('Position', [40 40 1000 860], 'Color', 'w', 'Visible', 'off');  ax = axes(f);  hold(ax, 'on');  axis(ax, 'equal');  box(ax, 'on');
    plot(ax, dl * cos(ph), dl * sin(ph), '-', 'Color', c.K, 'LineWidth', 1.5);
    xline(ax, 0, '-', 'Color', c.K);  yline(ax, 0, '-', 'Color', c.K);
    for t = TH
        th = t * pi / 180;  Cxx = cos(th)^2 * ap + sin(th)^2 * an;  Czz = sin(th)^2 * ap + cos(th)^2 * an;  Cxz = -sin(th) * cos(th) * dl;
        X = Cxx - Czz;  Y = -2 * Cxz;
        if t == 20
            plot(ax, [0 X], [0 0], '-', 'Color', c.E, 'LineWidth', 4);  plot(ax, [X X], [0 Y], '-', 'Color', c.T, 'LineWidth', 4);
            plot(ax, [0 X], [0 Y], '-', 'Color', [0.2 0.2 0.2], 'LineWidth', 3);
            a = linspace(0, 2 * th, 40);  plot(ax, 0.18 * cos(a), 0.18 * sin(a), '-', 'Color', [0.2 0.2 0.2], 'LineWidth', 2);
            text(ax, 0.21, 0.05, '2\theta_y', 'FontSize', c.FS, 'FontWeight', 'bold');
            text(ax, X / 2 - 0.25, -0.1, sprintf('平均(x^2) - 平均(z^2) = %.2f', X), 'Color', c.E, 'FontSize', c.FS - 1, 'FontWeight', 'bold');
            text(ax, X + 0.03, Y / 2, sprintf('-2·平均(x\\cdotz) = %.2f', Y), 'Color', c.T, 'FontSize', c.FS - 1, 'FontWeight', 'bold');
            text(ax, X / 2 - 0.42, Y / 2 + 0.08, sprintf('斜邊 = a_{||} - a_\\perp = %.2f', hypot(X, Y)), 'FontSize', c.FS - 1, 'FontWeight', 'bold');
        end
        plot(ax, X, Y, 'o', 'Color', c.E, 'MarkerFaceColor', c.E, 'MarkerSize', 10);
        text(ax, X + 0.03 * sign(X + 1e-9), Y + 0.05, sprintf('\\theta_y = %d°', t), 'FontSize', c.FS - 2, 'FontWeight', 'bold', 'Color', c.E);
    end
    plot(ax, 0, 0, 's', 'Color', c.T, 'MarkerFaceColor', c.T, 'MarkerSize', 11);  text(ax, -0.8, -0.18, '紅方塊 = 遠場：全部擠在原點', 'Color', c.T, 'FontSize', c.FS - 2, 'FontWeight', 'bold');
    xlim(ax, [-0.85 0.95]);  ylim(ax, [-0.25 0.85]);  xlabel(ax, '平均(x^2) - 平均(z^2)');  ylabel(ax, '-2·平均(x\cdotz)');
    set(ax, 'FontSize', c.FS, 'FontWeight', 'bold', 'LineWidth', 1.6);
    local_save(f, fd, 'fig_wall_angle_backwards');
end

function local_box(fd)                               % where the three numbers sit on the ellipse: bounding box and touch point
    c = C();  ap = 1;  an = 0.3;  TH = [0 20 45];  ph = linspace(0, 2 * pi, 300);  FS = 16;
    f = figure('Position', [40 40 1900 700], 'Color', 'w', 'Visible', 'off');  tl = tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    for i = 1:3
        th = TH(i) * pi / 180;  R = [cos(th) sin(th); -sin(th) cos(th)];
        Cxx = cos(th)^2 * ap + sin(th)^2 * an;  Czz = sin(th)^2 * ap + cos(th)^2 * an;  Cxz = -sin(th) * cos(th) * (ap - an);
        hw = 2 * sqrt(Cxx);  hh = 2 * sqrt(Czz);  zt = 2 * Cxz / sqrt(Cxx);              % 2-sigma ellipse: half width, half height, height of the right touch point
        ax = nexttile(tl);  hold(ax, 'on');  axis(ax, 'equal');  xlim(ax, [-3.9 2.9]);  ylim(ax, [-2.1 2.1]);  box(ax, 'on');
        xline(ax, 0, '-', 'Color', c.K);  yline(ax, 0, '-', 'Color', c.K);
        rectangle(ax, 'Position', [-hw -hh 2 * hw 2 * hh], 'EdgeColor', [0.4 0.4 0.4], 'LineStyle', '--', 'LineWidth', 1.5);
        e = R * (2 * [sqrt(ap) * cos(ph); sqrt(an) * sin(ph)]);  plot(ax, e(1,:), e(2,:), '-', 'Color', c.E, 'LineWidth', 3);
        plot(ax, [0 hw], [-hh - 0.12, -hh - 0.12], '-', 'Color', c.E, 'LineWidth', 4);
        text(ax, hw / 2, -hh - 0.32, '2\surdmean(x^2)', 'Color', c.E, 'FontSize', FS - 1, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
        plot(ax, [-hw - 0.12, -hw - 0.12], [0 hh], '-', 'Color', c.G, 'LineWidth', 4);
        text(ax, -hw - 0.2, hh / 2, '2\surdmean(z^2)', 'Color', c.G, 'FontSize', FS - 1, 'FontWeight', 'bold', 'HorizontalAlignment', 'right');
        plot(ax, hw, zt, 'o', 'Color', c.T, 'MarkerFaceColor', c.T, 'MarkerSize', 11);
        plot(ax, [hw + 0.1 hw + 0.1], [0 zt], '-', 'Color', c.T, 'LineWidth', 4);
        text(ax, hw + 0.18, zt / 2 - 0.05, sprintf('%.2f', zt), 'Color', c.T, 'FontSize', FS, 'FontWeight', 'bold');
        title(ax, sprintf('\\theta_y = %d°：mean(x\\cdotz) = %+.2f', TH(i), Cxz), 'FontSize', FS + 1);
        xlabel(ax, 'x');  if i == 1; ylabel(ax, 'z'); end
        set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.6);
        fprintf('theta %2d: half width %.2f, half height %.2f, right touch point at z = %+.2f (= 2 mean(xz)/sqrt(mean(x^2)))\n', TH(i), hw, hh, zt);
    end
    local_save(f, fd, 'fig_wall_angle_box');
end

function local_flat(fd)                              % same wall angle, different flatness: the red point moves, the long axis does not
    c = C();  ap = 1;  AN = [0.02 0.3 0.7 1.0];  th = 20 * pi / 180;  R = [cos(th) sin(th); -sin(th) cos(th)];  ph = linspace(0, 2 * pi, 300);  FS = 16;
    f = figure('Position', [40 40 2000 640], 'Color', 'w', 'Visible', 'off');  tl = tiledlayout(1, 4, 'TileSpacing', 'compact', 'Padding', 'compact');
    for i = 1:4
        an = AN(i);  Cxx = cos(th)^2 * ap + sin(th)^2 * an;  Czz = sin(th)^2 * ap + cos(th)^2 * an;  Cxz = -sin(th) * cos(th) * (ap - an);
        hw = 2 * sqrt(Cxx);  hh = 2 * sqrt(Czz);  zt = 2 * Cxz / sqrt(Cxx);  ang = -atan(Cxz / Cxx) * 180 / pi;
        ax = nexttile(tl);  hold(ax, 'on');  axis(ax, 'equal');  xlim(ax, [-2.4 2.4]);  ylim(ax, [-2.9 2.9]);  box(ax, 'on');
        xline(ax, 0, '-', 'Color', c.K);  yline(ax, 0, '-', 'Color', c.K);
        rectangle(ax, 'Position', [-hw -hh 2 * hw 2 * hh], 'EdgeColor', [0.4 0.4 0.4], 'LineStyle', '--', 'LineWidth', 1.5);
        plot(ax, 2.3 * [-R(1,1) R(1,1)], 2.3 * [-R(2,1) R(2,1)], '-', 'Color', c.T, 'LineWidth', 4);                 % long axis = wall, always 20 deg
        e = R * (2 * [sqrt(ap) * cos(ph); sqrt(an) * sin(ph)]);  plot(ax, e(1,:), e(2,:), '-', 'Color', c.E, 'LineWidth', 3);
        plot(ax, [0 hw], [0 zt], '--', 'Color', [0.2 0.2 0.2], 'LineWidth', 2.5);  plot(ax, hw, zt, 'o', 'Color', c.T, 'MarkerFaceColor', c.T, 'MarkerSize', 11);
        title(ax, sprintf('a_\\perp = %.2f', an), 'FontSize', FS + 1);
        text(ax, -2.25, 2.65, '長軸（牆）= 20°', 'Color', c.T, 'FontSize', FS - 1, 'FontWeight', 'bold');
        text(ax, -2.25, 2.25, sprintf('中心到紅點 = %.1f°', ang), 'Color', [0.2 0.2 0.2], 'FontSize', FS - 1, 'FontWeight', 'bold');
        text(ax, -2.25, -2.55, sprintf('mean(x\\cdotz) = %.2f', Cxz), 'Color', c.T, 'FontSize', FS - 1, 'FontWeight', 'bold');
        xlabel(ax, 'x');  if i == 1; ylabel(ax, 'z'); end
        set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.6);
        fprintf('a_perp %.2f: mean(xz) %+.3f, center-to-red %.1f deg, long axis 20 deg\n', an, Cxz, ang);
    end
    local_save(f, fd, 'fig_wall_angle_flat');
end

function local_data(fd)                              % the three averages measured on the dots: x spread, z spread, average z per x-slice
    c = C();  ap = 1;  an = 0.3;  th = 20 * pi / 180;  R = [cos(th) sin(th); -sin(th) cos(th)];  FS = 16;  rng(3);  n = 4000;
    q = R * [sqrt(ap) * randn(1, n); sqrt(an) * randn(1, n)];  x = q(1,:);  z = q(2,:);
    Cxx = mean(x.^2);  Czz = mean(z.^2);  Cxz = mean(x .* z);
    f = figure('Position', [40 40 2000 700], 'Color', 'w', 'Visible', 'off');  tl = tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    T = {sprintf('mean(x^2) = %.2f：左右散多開', Cxx), sprintf('mean(z^2) = %.2f：上下散多開', Czz), sprintf('mean(x\\cdotz) = %.2f：每條直切片的平均高度', Cxz)};
    for i = 1:3
        ax = nexttile(tl);  hold(ax, 'on');  axis(ax, 'equal');  xlim(ax, [-3.2 3.2]);  ylim(ax, [-2.6 2.6]);  box(ax, 'on');
        plot(ax, x, z, '.', 'Color', c.M, 'MarkerSize', 4);  xline(ax, 0, '-', 'Color', c.K);  yline(ax, 0, '-', 'Color', c.K);
        if i == 1
            xline(ax, 2 * sqrt(Cxx) * [-1 1], '--', 'Color', c.E, 'LineWidth', 3);
            plot(ax, [0 2 * sqrt(Cxx)], [-2.3 -2.3], '-', 'Color', c.E, 'LineWidth', 4);
            text(ax, sqrt(Cxx), -2.05, '2\surdmean(x^2)', 'Color', c.E, 'FontSize', FS, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
        elseif i == 2
            yline(ax, 2 * sqrt(Czz) * [-1 1], '--', 'Color', c.G, 'LineWidth', 3);
            plot(ax, [-2.95 -2.95], [0 2 * sqrt(Czz)], '-', 'Color', c.G, 'LineWidth', 4);
            text(ax, -2.85, sqrt(Czz) + 0.15, '2\surdmean(z^2)', 'Color', c.G, 'FontSize', FS, 'FontWeight', 'bold');
        else
            ed = -2.4:0.4:2.4;  xm = (ed(1:end-1) + ed(2:end)) / 2;  zm = nan(size(xm));
            for b = 1:numel(xm); in = x >= ed(b) & x < ed(b+1); if nnz(in) > 20; zm(b) = mean(z(in)); end; end
            for b = 1:numel(ed); xline(ax, ed(b), ':', 'Color', [0.75 0.75 0.75], 'LineWidth', 1); end
            plot(ax, [-2.4 2.4], Cxz / Cxx * [-2.4 2.4], '--', 'Color', [0.2 0.2 0.2], 'LineWidth', 2.5);
            plot(ax, xm, zm, 'o', 'Color', c.T, 'MarkerFaceColor', c.T, 'MarkerSize', 10);
            text(ax, -3.05, -2.3, sprintf('斜率 = mean(x\\cdotz) / mean(x^2) = %.2f', Cxz / Cxx), 'Color', [0.2 0.2 0.2], 'FontSize', FS - 1, 'FontWeight', 'bold');
        end
        title(ax, T{i}, 'FontSize', FS);  xlabel(ax, 'x');  if i == 1; ylabel(ax, 'z'); end
        set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.6);
    end
    fprintf('dots: mean(x^2) %.3f, mean(z^2) %.3f, mean(xz) %.3f, slice slope %.3f\n', Cxx, Czz, Cxz, Cxz / Cxx);
    local_save(f, fd, 'fig_wall_angle_data');
end

function local_combined(fd)                          % dots + the three averages + the ellipse they define, in one picture
    c = C();  ap = 1;  an = 0.3;  th = 20 * pi / 180;  R = [cos(th) sin(th); -sin(th) cos(th)];  FS = 17;  rng(3);  n = 4000;  cC = [0.6 0.29 0.13];
    q = R * [sqrt(ap) * randn(1, n); sqrt(an) * randn(1, n)];  x = q(1,:);  z = q(2,:);
    Cxx = mean(x.^2);  Czz = mean(z.^2);  Cxz = mean(x .* z);  S = [Cxx Cxz; Cxz Czz];  [E, L] = eig(S);  [~, im] = max(diag(L));  ev = E(:, im);
    if ev(1) < 0; ev = -ev; end;  ang = -atan2(ev(2), ev(1)) * 180 / pi;  ph = linspace(0, 2 * pi, 400);  el = 2 * sqrtm(S) * [cos(ph); sin(ph)];
    hw = 2 * sqrt(Cxx);  hh = 2 * sqrt(Czz);  zt = 2 * Cxz / sqrt(Cxx);
    f = figure('Position', [40 40 1750 900], 'Color', 'w', 'Visible', 'off');  ax = axes(f);  hold(ax, 'on');  axis(ax, 'equal');  box(ax, 'on');
    xlim(ax, [-3.3 5.7]);  ylim(ax, [-2.4 2.4]);
    plot(ax, x, z, '.', 'Color', c.M, 'MarkerSize', 4);  xline(ax, 0, '-', 'Color', c.K);  yline(ax, 0, '-', 'Color', c.K);
    plot(ax, [-hw -hw], [-hh hh], '--', 'Color', c.E, 'LineWidth', 2.5);  plot(ax, [hw hw], [-hh hh], '--', 'Color', c.E, 'LineWidth', 2.5);
    plot(ax, [-hw hw], [-hh -hh], '--', 'Color', c.G, 'LineWidth', 2.5);  plot(ax, [-hw hw], [hh hh], '--', 'Color', c.G, 'LineWidth', 2.5);
    plot(ax, el(1,:), el(2,:), '-', 'Color', c.E, 'LineWidth', 3.5);
    plot(ax, 2.4 * [-ev(1) ev(1)], 2.4 * [-ev(2) ev(2)], '-', 'Color', cC, 'LineWidth', 4);
    ed = -2.4:0.4:2.4;  xm = (ed(1:end-1) + ed(2:end)) / 2;  zm = nan(size(xm));
    for b = 1:numel(xm); in = x >= ed(b) & x < ed(b+1); if nnz(in) > 20; zm(b) = mean(z(in)); end; end
    plot(ax, [-hw hw], Cxz / Cxx * [-hw hw], '--', 'Color', [0.2 0.2 0.2], 'LineWidth', 2.5);
    plot(ax, xm, zm, 'o', 'Color', c.T, 'MarkerFaceColor', c.T, 'MarkerSize', 9);
    plot(ax, hw, zt, 'p', 'Color', c.T, 'MarkerFaceColor', c.T, 'MarkerSize', 22);
    tx = 2.25;  ty = [2.0 1.55 1.1 0.65 0.2];
    text(ax, tx, ty(1), sprintf('藍虛線：x = ±2\\surdmean(x^2) = ±%.2f', hw), 'Color', c.E, 'FontSize', FS - 2, 'FontWeight', 'bold');
    text(ax, tx, ty(2), sprintf('綠虛線：z = ±2\\surdmean(z^2) = ±%.2f', hh), 'Color', c.G, 'FontSize', FS - 2, 'FontWeight', 'bold');
    text(ax, tx, ty(3), sprintf('紅點：切片平均，斜率 mean(x\\cdotz)/mean(x^2) = %.2f', Cxz / Cxx), 'Color', c.T, 'FontSize', FS - 2, 'FontWeight', 'bold');
    text(ax, tx, ty(4), sprintf('星號：斜線碰到方框右邊 = 橢圓最右點'), 'Color', c.T, 'FontSize', FS - 2, 'FontWeight', 'bold');
    text(ax, tx, ty(5), sprintf('棕線：橢圓長軸 = 牆，%.1f°', ang), 'Color', cC, 'FontSize', FS - 2, 'FontWeight', 'bold');
    text(ax, tx, -0.5, '藍橢圓：由三個平均畫出', 'Color', c.E, 'FontSize', FS - 2, 'FontWeight', 'bold');
    text(ax, tx, -1.0, sprintf('mean(x^2) = %.2f   mean(z^2) = %.2f', Cxx, Czz), 'Color', [0.2 0.2 0.2], 'FontSize', FS - 2, 'FontWeight', 'bold');
    text(ax, tx, -1.4, sprintf('mean(x\\cdotz) = %.2f', Cxz), 'Color', [0.2 0.2 0.2], 'FontSize', FS - 2, 'FontWeight', 'bold');
    xlabel(ax, 'x');  ylabel(ax, 'z');  set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.6);
    fprintf('combined: box +-%.2f x +-%.2f, star at z = %.2f, slice slope %.3f (%.1f deg), long axis %.1f deg\n', hw, hh, zt, Cxz / Cxx, -atan(Cxz / Cxx) * 180 / pi, ang);
    local_save(f, fd, 'fig_wall_angle_combined');
end

function local_areas(fd)                             % 1-sigma ellipse: the three averages are two squares and one rectangle
    c = C();  ap = 1;  an = 0.3;  th = 20 * pi / 180;  R = [cos(th) sin(th); -sin(th) cos(th)];  FS = 17;  rng(3);  n = 4000;  cC = [0.6 0.29 0.13];
    q = R * [sqrt(ap) * randn(1, n); sqrt(an) * randn(1, n)];  x = q(1,:);  z = q(2,:);
    Cxx = mean(x.^2);  Czz = mean(z.^2);  Cxz = mean(x .* z);  S = [Cxx Cxz; Cxz Czz];  [E, L] = eig(S);  [~, im] = max(diag(L));  ev = E(:, im);
    a = sqrt(Cxx);  b = sqrt(Czz);  zt = Cxz / a;  ph = linspace(0, 2 * pi, 400);  el = sqrtm(S) * [cos(ph); sin(ph)];
    f = figure('Position', [40 40 1750 900], 'Color', 'w', 'Visible', 'off');  ax = axes(f);  hold(ax, 'on');  axis(ax, 'equal');  box(ax, 'on');
    xlim(ax, [-1.5 3.3]);  ylim(ax, [-1.2 1.2]);
    plot(ax, x, z, '.', 'Color', [0.86 0.9 0.97], 'MarkerSize', 4);  xline(ax, 0, '-', 'Color', c.K);  yline(ax, 0, '-', 'Color', c.K);
    patch(ax, [-a 0 0 -a], [0 0 a a], c.E, 'FaceAlpha', 0.18, 'EdgeColor', c.E, 'LineWidth', 2.5);
    patch(ax, [0 b b 0], [0 0 b b], c.G, 'FaceAlpha', 0.22, 'EdgeColor', c.G, 'LineWidth', 2.5);
    patch(ax, [0 a a 0], [0 0 zt zt], c.T, 'FaceAlpha', 0.22, 'EdgeColor', c.T, 'LineWidth', 2.5);
    plot(ax, a * [-1 1 1 -1 -1], b * [-1 -1 1 1 -1], '--', 'Color', [0.35 0.35 0.35], 'LineWidth', 1.8);
    plot(ax, el(1,:), el(2,:), '-', 'Color', c.E, 'LineWidth', 3.5);
    plot(ax, 1.25 * [-ev(1) ev(1)], 1.25 * [-ev(2) ev(2)], '-', 'Color', cC, 'LineWidth', 4);
    plot(ax, a, zt, 'p', 'Color', c.T, 'MarkerFaceColor', c.T, 'MarkerSize', 22);
    text(ax, -a / 2, a / 2, {'邊長 \surdmean(x^2)', sprintf('面積 = mean(x^2) = %.2f', Cxx)}, 'Color', c.E, 'FontSize', FS - 3, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    text(ax, b + 0.05, b - 0.12, {'邊長 \surdmean(z^2)', sprintf('面積 = mean(z^2) = %.2f', Czz)}, 'Color', c.G, 'FontSize', FS - 3, 'FontWeight', 'bold');
    text(ax, a + 0.12, zt / 2, {'寬 \surdmean(x^2) × 高 = 星號往下多少', sprintf('面積 = |mean(x\\cdotz)| = %.2f', -Cxz)}, 'Color', c.T, 'FontSize', FS - 3, 'FontWeight', 'bold');
    text(ax, 1.35 * ev(1) - 0.05, 1.35 * ev(2) - 0.08, '長軸 = 牆', 'Color', cC, 'FontSize', FS - 3, 'FontWeight', 'bold');
    xlabel(ax, 'x');  ylabel(ax, 'z');  set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.6);
    fprintf('1-sigma: half width %.3f, half height %.3f, star depth %.3f; areas %.3f %.3f %.3f\n', a, b, zt, a^2, b^2, a * zt);
    local_save(f, fd, 'fig_wall_angle_areas');
end

function local_bridge(fd)                            % areas -> numbers -> bars -> legs of the triangle (theta = 20 deg, a_par 1, a_perp 0.3)
    c = C();  FS = 16;  Cxx = 0.918;  Czz = 0.382;  Cxz = -0.225;  A = sqrt(Cxx);  B = sqrt(Czz);  Dp = -Cxz / A;  cK = [0.25 0.25 0.25];
    f = figure('Position', [40 40 2100 640], 'Color', 'w', 'Visible', 'off');  tl = tiledlayout(1, 4, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax = nexttile(tl);  hold(ax, 'on');  axis(ax, 'equal');  axis(ax, 'off');  xlim(ax, [-0.1 2.6]);  ylim(ax, [-0.6 1.3]);
    rectangle(ax, 'Position', [0 0 A A], 'FaceColor', [0.84 0.91 0.93], 'EdgeColor', c.E, 'LineWidth', 2.5);
    rectangle(ax, 'Position', [1.1 0 B B], 'FaceColor', [0.88 0.9 0.91], 'EdgeColor', c.K, 'LineWidth', 2.5, 'LineStyle', '--');
    rectangle(ax, 'Position', [1.85 0 A Dp], 'FaceColor', [0.85 0.87 0.88], 'EdgeColor', cK, 'LineWidth', 2.5);
    text(ax, A / 2, A / 2, '0.92', 'Color', c.E, 'FontSize', FS + 2, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    text(ax, 1.1 + B / 2, B / 2, '0.38', 'Color', cK, 'FontSize', FS, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    text(ax, 1.85 + A / 2, Dp + 0.12, '0.225', 'Color', cK, 'FontSize', FS, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    text(ax, A / 2, -0.15, 'x 正方形', 'FontSize', FS - 2, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'Color', c.E);
    text(ax, 1.1 + B / 2, -0.15, 'z 正方形', 'FontSize', FS - 2, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'Color', cK);
    text(ax, 1.85 + A / 2, -0.15, '長方形', 'FontSize', FS - 2, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'Color', cK);
    text(ax, 1.25, 1.2, '① 第 5 頁的三塊面積', 'FontSize', FS + 1, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    ax = nexttile(tl);  hold(ax, 'on');  axis(ax, 'off');  xlim(ax, [-0.05 1.15]);  ylim(ax, [-0.6 1.3]);
    Y = [0.85 0.45 0.05];  Vv = [Cxx Czz -Cxz];  Cl = {c.E, c.K, cK};  Nm = {'x 正方形', 'z 正方形', '長方形'};
    for i = 1:3
        rectangle(ax, 'Position', [0 Y(i) Vv(i) 0.18], 'FaceColor', Cl{i}, 'EdgeColor', 'none');
        text(ax, Vv(i) + 0.03, Y(i) + 0.09, sprintf('%.3g', Vv(i)), 'Color', Cl{i}, 'FontSize', FS, 'FontWeight', 'bold');
        text(ax, 0, Y(i) - 0.07, Nm{i}, 'Color', Cl{i}, 'FontSize', FS - 3, 'FontWeight', 'bold');
    end
    text(ax, 0.55, 1.2, '② 只留數值：畫成長條', 'FontSize', FS + 1, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    ax = nexttile(tl);  hold(ax, 'on');  axis(ax, 'off');  xlim(ax, [-0.05 1.15]);  ylim(ax, [-0.6 1.3]);
    rectangle(ax, 'Position', [0 0.75 Cxx 0.18], 'FaceColor', 'none', 'EdgeColor', c.E, 'LineWidth', 2);
    rectangle(ax, 'Position', [Cxx - Czz 0.75 Czz 0.18], 'FaceColor', [0.88 0.9 0.91], 'EdgeColor', c.K, 'LineWidth', 2, 'LineStyle', '--');
    rectangle(ax, 'Position', [0 0.75 Cxx - Czz 0.18], 'FaceColor', c.E, 'EdgeColor', 'none');
    text(ax, (Cxx - Czz) / 2, 0.66, sprintf('0.92 − 0.38 = %.2f', Cxx - Czz), 'Color', c.E, 'FontSize', FS - 1, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    text(ax, 0, 1.02, '橫邊：x 長條切掉 z 長條', 'Color', c.E, 'FontSize', FS - 2, 'FontWeight', 'bold');
    rectangle(ax, 'Position', [0 0.15 -Cxz 0.18], 'FaceColor', cK, 'EdgeColor', 'none');
    rectangle(ax, 'Position', [-Cxz 0.15 -Cxz 0.18], 'FaceColor', [0.55 0.55 0.55], 'EdgeColor', 'none');
    text(ax, -Cxz, 0.06, sprintf('0.225 + 0.225 = %.2f', -2 * Cxz), 'Color', cK, 'FontSize', FS - 1, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    text(ax, 0, 0.42, '直邊：長方形長條接兩次', 'Color', cK, 'FontSize', FS - 2, 'FontWeight', 'bold');
    text(ax, 0.55, 1.2, '③ 長條相減、相接', 'FontSize', FS + 1, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    ax = nexttile(tl);  hold(ax, 'on');  axis(ax, 'equal');  axis(ax, 'off');  xlim(ax, [-0.1 0.95]);  ylim(ax, [-0.3 0.75]);
    Xl = Cxx - Czz;  Yl = -2 * Cxz;
    plot(ax, [0 Xl], [0 0], '-', 'Color', c.E, 'LineWidth', 9);  plot(ax, [Xl Xl], [0 Yl], '-', 'Color', cK, 'LineWidth', 9);
    plot(ax, [0 Xl], [0 Yl], '-', 'Color', [0.6 0.29 0.13], 'LineWidth', 4);
    ph = linspace(0, atan2(Yl, Xl), 40);  plot(ax, 0.12 * cos(ph), 0.12 * sin(ph), '-', 'Color', 'k', 'LineWidth', 2);
    text(ax, 0.15, 0.06, '40°', 'FontSize', FS, 'FontWeight', 'bold');
    text(ax, Xl / 2, -0.07, '0.54', 'Color', c.E, 'FontSize', FS, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    text(ax, Xl + 0.03, Yl / 2, '0.45', 'Color', cK, 'FontSize', FS, 'FontWeight', 'bold');
    text(ax, Xl / 2 - 0.05, Yl / 2 + 0.1, {'量出來 0.70', '= 1 − 0.3 = ā_{||} − ā_\perp'}, 'Color', [0.6 0.29 0.13], 'FontSize', FS - 2, 'FontWeight', 'bold', 'HorizontalAlignment', 'right');
    text(ax, 0.42, -0.22, 'θ = 40° / 2 = 20°', 'FontSize', FS + 1, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    text(ax, 0.42, 0.72, '④ 兩條長條擺成直角', 'FontSize', FS + 1, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    local_save(f, fd, 'fig_wall_angle_bridge');
end

function local_save(f, fd, name)
    out = fullfile(fd, [name '.png']);  exportgraphics(f, out, 'Resolution', 130);  close(f);  fprintf('figure: %s\n', out);
end
