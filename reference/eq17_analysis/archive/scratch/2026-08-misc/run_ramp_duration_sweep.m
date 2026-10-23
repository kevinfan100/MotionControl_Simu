% STATUS: ACTIVE (scratch) | PURPOSE: TEST 1 -- no oscillation (amplitude 0),
%   canonical deep band (w_bar 22.2 -> 1.10), and only the descent duration
%   changes: 1, 2.5, 5, 10 s. How do the a_hat error (seed mean) and the
%   seed spread depend on how long the same descent takes? Production arm.
function out = run_ramp_duration_sweep(seeds, T_DESC, no_hold)
    if nargin < 1 || isempty(seeds);  seeds  = [7 11 23 42 101 777]; end
    if nargin < 2 || isempty(T_DESC); T_DESC = [1 2.5 5 10]; end
    if nargin < 3 || isempty(no_hold); no_hold = false; end   % true: the run ends on arrival at the trough (no near-wall hold)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    nT = numel(T_DESC); O = cell(1,nT); NAMES = cell(1,nT); W = cell(1,nT);
    for i = 1:nT
        Td = T_DESC(i);
        ov = struct('amplitude', 0, 'frequency', 1, 'n_cycles', 1, 't_hold', 0.5, ...
                    't_descend_override', Td, 'T_sim', 0.5 + Td + 1.0 + 1.3);   % flat 1 s at the trough + 1.3 s hold
        if no_hold; ov.T_sim = 0.5 + Td + 0.05; end                            % stop 50 ms after arrival
        clear run_formC_b motion_control_law_formC_b;
        evalc("O{i} = run_formC_b(struct('arm','best','seeds',seeds,'config_override',ov));");
        NAMES{i} = sprintf('descent %g s, no osc%s', Td, char("" + (no_hold * 0)));
        if no_hold; NAMES{i} = sprintf('descent %g s, no osc, no hold', Td); end
        W{i} = [0.5, 0.5+Td; 0.5+0.5*Td, 0.5+Td; 0.5+Td+1.0, 0.5+Td+2.3];   % whole descent | second half (near wall) | final hold
        if no_hold; W{i} = W{i}(1:2, :); end
        fprintf('ran %s  (T = %.1f s)\n', NAMES{i}, O{i}.runs{1}.tout(end));
    end
    tag = ''; if no_hold; tag = '_nohold'; end
    fig = fullfile(od, ['ramp_duration_sweep' tag '.png']);
    D = plot_seed_spread_cols(O, NAMES, fig, W, {'descent', 'near-wall half', 'hold'});
    save(fullfile(od, ['ramp_duration_sweep' tag '.mat']), 'O', 'seeds', 'T_DESC', '-v7.3');
    out = struct('D', {D}, 'file', fig);
end
