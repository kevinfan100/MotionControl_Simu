% STATUS: SCRATCH -- 2026-10-24 (user: 10-21 flow, single seed): computed angle with A-bar' (run_t3_xyz_fixed, opts.flow + aprime) vs
%   estimated angle (saved run_t2_3d_reg flow_s1). 20 deg wall, start 15 um, hold 4 s + dither, regulation (a_d 0.1, tau_a 2 s), scan
%   from 11 s. Columns = computed | estimated; rows = theta_y error, theta_x error, gap h [um] (red = h* where a_N = a_d),
%   a_N-hat (red = true). 1022 style, Time (sec). Output test_results/temp_3d_wsonly/fig_t3_flow_s1.png | EXPIRES: when the flow is read
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
if ~exist('src', 'var'); src = 'flow_s1'; end           % 'flow_s1' (hold 4 s + dither, 10-24 first) | 'flow_wait5' (plain 5 s wait)
if strcmp(src, 'flow_s1')
    C = load(fullfile(fo, 'temp_t3_flow_s1.mat'));  F = load(fullfile(fo, 'temp_t2_flow_s1.mat'));  rc = C.rc;  rb = F.R.runs{1};
    if isfield(C, 'rq'); rc = C.rq; end                    % 10-24: A-bar' with P_fix once moving
    lbl = sprintf('start 15 %sm, hold 4 s + dither, scan from 11 s', char(181));  Tend = 14;
else
    C = load(fullfile(fo, 'temp_t3_flow_wait5.mat'));  rc = C.rc;  rb = C.rb;
    lbl = sprintf('start 15 %sm, wait %d s, scan from %d s', char(181), C.o.hold_until, C.o.t_scan);  Tend = C.o.T;
end
Rr = 2.25;  hstar = rb.h_star * Rr;
FS = 15;  hatc = char(770);  abar = char(257);  cB = [0 0.25 0.85];  cG = [0 0.5 0.2];
arm = {'computed angle', 'estimated angle'};
fig = figure('Color', 'w', 'Position', [80 80 1300 1250]);
tl = tiledlayout(4, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for row = 1:4
    for c = 1:2
        nexttile;  hold on;  box on;
        if c == 1; r = rc; k = 1:rc.kend; aN = rc.aNw(k); aNt = rc.aN_tr(k);
        else;      r = rb; k = 1:rb.kend; aN = rb.aN(k);  aNt = rb.aN_true(k); end
        switch row
            case 1; y = r.thy(k) - 20; tru = 0 * k; col = cB; yl = [-30 30];
            case 2; y = r.thx(k);      tru = 0 * k; col = cG; yl = [-30 30];
            case 3; y = r.h(k) * Rr;   tru = hstar + 0 * k; col = cB; yl = [0 16];
            case 4; y = aN;            tru = aNt; col = cB; yl = [0 1];
        end
        plot(r.t(k), tru, 'r', 'LineWidth', 2.5);
        h = plot(r.t(k), y, 'Color', col, 'LineWidth', 1.6);
        xlim([0 Tend]);  ylim(yl);  set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);
        switch row
            case 1
                if c == 1; ylabel('\theta_y error  [\circ]'); end
                legend(h, sprintf('%s_y - %s_y,  seed 1   (%s)', ['θ' hatc], char(952), arm{c}), 'Location', 'northoutside');
                text(0.97, 0.88, lbl, 'Units', 'normalized', ...
                     'HorizontalAlignment', 'right', 'FontSize', 13, 'FontWeight', 'bold');
            case 2
                if c == 1; ylabel('\theta_x error  [\circ]'); end
                legend(h, sprintf('%s_x - %s_x', ['θ' hatc], char(952)), 'Location', 'northoutside');
            case 3
                if c == 1; ylabel(sprintf('h  [%sm]', char(181))); end
                legend(h, sprintf('gap h   (red = target %.2f %sm)', hstar, char(181)), 'Location', 'northoutside');
            case 4
                if c == 1; ylabel(sprintf('%s_N', abar)); end
                legend(h, sprintf('%s_N%s   (red = true)', abar, hatc), 'Location', 'northoutside');  xlabel('Time (sec)');
        end
    end
end
exportgraphics(fig, fullfile(fo, ['fig_t3_' src '.png']), 'Resolution', 150);
close(fig);
