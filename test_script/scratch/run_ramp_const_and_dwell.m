% STATUS: ACTIVE (scratch) | PURPOSE: two follow-ups to run_ramp_duration_sweep.
%   A  CONSTANT-VELOCITY descent (trajectory_type 'ramp_descent': linear
%      w_bar 22.2 -> 1.10 over T_sim, no initial hold, run ends on arrival),
%      durations 1 / 2.5 / 5 / 10 s.  Replaces the cosine ease-in/out of the
%      'osc' type used before.
%   B  DWELL at the trough: cosine 1 s descent, then the particle sits at
%      w_bar = 1.10 for 1 / 3 / 10 s (no oscillation).  Does the a_hat bias
%      grow with the time spent near the wall, and at what rate?
%   Production arm, 6 house seeds, figures via plot_seed_spread_cols.
function out = run_ramp_const_and_dwell(seeds, which)
    if nargin < 1 || isempty(seeds); seeds = [7 11 23 42 101 777]; end
    if nargin < 2 || isempty(which); which = 'AB'; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'apd_acov_meng'); out = struct();
    if contains(which, 'A')
        TD = [1 2.5 5 10]; O = cell(1,4); NAMES = O; W = O;
        for i = 1:4
            ov = struct('trajectory_type','ramp_descent','h_init',50,'amplitude',0,'T_sim',TD(i),'t_hold',0,'t_descend_override',TD(i));
            clear run_formC_b motion_control_law_formC_b;
            evalc("O{i} = run_formC_b(struct('arm','best','seeds',seeds,'config_override',ov));");
            NAMES{i} = sprintf('const-v descent %g s', TD(i));
            W{i} = [0, TD(i); 0.5*TD(i), TD(i); TD(i)-0.1, TD(i)];        % whole | near-wall half | last 0.1 s
            fprintf('ran %s\n', NAMES{i});
        end
        figA = fullfile(od, 'ramp_constv_sweep.png');
        DA = plot_seed_spread_cols(O, NAMES, figA, W, {'descent', 'near-wall half', 'arrival'});
        save(fullfile(od, 'ramp_constv_sweep.mat'), 'O', 'seeds', 'TD', '-v7.3'); out.A = DA; out.figA = figA;
    end
    if contains(which, 'B')
        TH = [1 3 10]; O = cell(1,3); NAMES = O; W = O;
        for i = 1:3
            ov = struct('amplitude',0,'frequency',1,'n_cycles',1,'t_hold',0.5,'t_descend_override',1.0,'T_sim',0.5+1.0+1.0+TH(i)-1.0);
            clear run_formC_b motion_control_law_formC_b;
            evalc("O{i} = run_formC_b(struct('arm','best','seeds',seeds,'config_override',ov));");
            NAMES{i} = sprintf('1 s descent, dwell %g s', TH(i));
            W{i} = [0.5 1.5; 1.5, 1.5+TH(i); 1.5+TH(i)-0.5, 1.5+TH(i)];   % descent | whole dwell | last 0.5 s of dwell
            t = O{i}.runs{1}.tout(:); E = zeros(numel(t), numel(seeds));
            for q = 1:numel(seeds); r = O{i}.runs{q}; ad = r.a_hat_out(1,3)/r.a_bar_hat_out(1,3); E(:,q) = r.a_bar_hat_out(:,3) - r.a_true_out(:,3)/ad; end
            m = t >= 1.5 & t <= 1.5+TH(i); p = polyfit(t(m), mean(E(m,:),2), 1);
            fprintf('ran %s : dwell-window bias %+.4f -> %+.4f, drift %+.4f /s (a_bar units; a_bar at trough = 0.087 => %+.1f %%/s)\n', ...
                    NAMES{i}, mean(E(find(m,1),:)), mean(E(find(m,1,'last'),:)), p(1), 100*p(1)/0.087);
        end
        figB = fullfile(od, 'trough_dwell_sweep.png');
        DB = plot_seed_spread_cols(O, NAMES, figB, W, {'descent', 'dwell', 'dwell end'});
        save(fullfile(od, 'trough_dwell_sweep.mat'), 'O', 'seeds', 'TH', '-v7.3'); out.B = DB; out.figB = figB;
    end
end
