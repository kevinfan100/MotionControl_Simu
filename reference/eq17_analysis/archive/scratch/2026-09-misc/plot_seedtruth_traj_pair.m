% STATUS: ACTIVE (scratch) | PURPOSE: the two trajectories the seed-at-truth
%   oracle pair is run on, side by side, so the 5x2 panels can be read against
%   what the particle was actually asked to do. Left = meng (10 s cosine
%   descent, no oscillation), right = canon (canonical deep, 1 Hz x 2).
%   Row 1 = commanded w_bar; row 2 = the truth the gain rows are estimating,
%   a_bar_true = 1/c_perp(w_bar). Both trajectories END at the trough, and the
%   troughs agree to 1% in w_bar -- that is what makes the HOLD windows
%   comparable and the descent/oscillation windows not.
function plot_seedtruth_traj_pair()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    pc = physical_constants();
    CFG = {struct('h_init',15,'h_bottom',2.5,'amplitude',0,'frequency',1,'n_cycles',1, ...
                  't_hold',0.5,'t_descend_override',10,'T_sim',12.5), ...
           canonical_scenario(0.05, 1.1, 'deep')};
    NM = {'meng: 10 s descent, no oscillation', 'canon: 1 Hz x 2 oscillation'};
    FS = 15; AXLW = 1.6; COL = [0 0.2 0.9]; COL_T = [0.8 0 0];
    f = figure('Units','inches','Position',[0 0 12 6.5], 'Color','w', 'Visible','off');
    tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
    for r = 1:2
        for c = 1:2
            g = CFG{c}; t = (0:pc.Ts:g.T_sim).';
            t1 = g.t_hold; t2 = t1 + g.t_descend_override; t3 = t2 + g.n_cycles/g.frequency;
            h = zeros(size(t));
            for i = 1:numel(t)
                tn = t(i);
                if     tn <= t1; h(i) = g.h_init;
                elseif tn <= t2; h(i) = g.h_bottom + (g.h_init-g.h_bottom)*(1+cos(pi*(tn-t1)/g.t_descend_override))/2;
                elseif tn <= t3; h(i) = (g.h_bottom + g.amplitude) - g.amplitude*cos(2*pi*g.frequency*(tn-t2));
                else;            h(i) = g.h_bottom;
                end
            end
            w = h / pc.R;
            nexttile((r-1)*2 + c); hold on;
            if r == 1
                plot(t, w, '-', 'Color', COL, 'LineWidth', 2.0);
                yline(g.h_bottom/pc.R, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2);
                if c == 1; ylabel('$\bar{w}$  (commanded)', 'Interpreter','latex', 'FontSize', FS+3, 'FontWeight','bold'); end
                ylim([0 24]); title(NM{c}, 'FontSize', FS, 'FontWeight','bold');
            else
                ab = zeros(size(w));
                for i = 1:numel(w); [~, cpv] = calc_correction_functions(w(i)); ab(i) = 1/cpv; end
                plot(t, ab, '-', 'Color', COL_T, 'LineWidth', 2.0);
                if c == 1; ylabel('a_z / a_{nom}   (truth)', 'FontSize', FS, 'FontWeight','bold'); end
                ylim([0 1]); xlabel('time  [s]', 'FontSize', FS, 'FontWeight','bold');
            end
            xlim([0 ceil(g.T_sim)]);
            set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off;
        end
    end
    png = fullfile(od, 'seedtruth_traj_pair.png');
    exportgraphics(f, png, 'Resolution', 150); close(f);
    fprintf('wrote %s\n', png);
end
