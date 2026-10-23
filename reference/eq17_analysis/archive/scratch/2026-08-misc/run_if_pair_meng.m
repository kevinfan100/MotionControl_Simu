% STATUS: ACTIVE (scratch) | PURPOSE: paired formC_b runs on the Meng 10 s
%   ramp with the R2 colour penalty IF kept (production as in the working
%   tree) vs removed (IF == 1 exactly, via ctrl_const_override
%   IF_abc = IF_abc_white = 0 -> IF = 1 + 2*0/den). Controller untouched.
%   Per seed: the house comparison page (plot_arms_pair) and the l*e ledger
%   page (plot_y2_contribution), IF on | IF off side by side.
%
%   out = run_if_pair_meng([7 11])
%
% LIVENESS (read before the figures): R2 on/off must differ by ~IF in the
%   far field, and dh_m must differ between arms (the readout feeds the law).
function out = run_if_pair_meng(seeds)

    if nargin < 1 || isempty(seeds); seeds = [7 11]; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));
    addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    AX = 3;

    ov = local_meng_override();
    ARMS = {struct(), 'IF on'; struct('IF_abc', [0;0;0], 'IF_abc_white', [0;0;0]), 'IF off'};
    O = cell(1, 2);
    for c = 1:2
        clear run_formC_b motion_control_law_formC_b;
        fprintf('\n######## arm %s ########\n', ARMS{c,2});
        O{c} = run_formC_b(struct('arm', 'best', 'seeds', seeds, ...
                                  'config_override', ov, ...
                                  'ctrl_const_override', ARMS{c,1}));
    end

    fprintf('\n=============== LIVENESS ===============\n');
    for q = 1:numel(seeds)
        r1 = O{1}.runs{q}; r2 = O{2}.runs{q}; t = r1.tout(:); w = t >= 1 & t <= 4;
        fprintf('seed %3d : R2 on/off far-field median %.3f | max|dh_m on-off| %.2e um | a_hat end on %.4f off %.4f\n', ...
                seeds(q), median(r1.R2_out(w,AX)./r2.R2_out(w,AX)), max(abs(r1.dh_m_out(:,AX)-r2.dh_m_out(:,AX))), ...
                r1.a_bar_hat_out(end,AX), r2.a_bar_hat_out(end,AX));
    end

    P = {O{1}, ARMS{1,2}; O{2}, ARMS{2,2}};
    if numel(seeds) <= 4                       % per-seed pages only for a pilot
        for q = 1:numel(seeds)
            plot_arms_pair(P, q, AX, [], '_if');
            plot_y2_contribution(P, q, AX, sprintf('_if_s%03d', seeds(q)));
        end
    end
    out = struct('O', {O}, 'ARMS', {ARMS}, 'seeds', seeds, 'ov', ov);
    if numel(seeds) <= 4; fn = 'pair_if_meng.mat'; else; fn = sprintf('pair_if_meng_%d.mat', numel(seeds)); end
    save(fullfile(od, fn), 'out', '-v7.3');
    fprintf('saved -> %s\n', fullfile(od, fn));
end

function ov = local_meng_override()
%   Copied verbatim from pair_apd_acov_meng.m (Fei Long 4.3.2 ramp).
    pc = physical_constants();
    ov = struct();
    ov.trajectory_type = 'osc';
    ov.h_init    = 15.0;
    ov.h_bottom  = 2.5;
    ov.amplitude = 0;
    ov.frequency = 1;
    ov.n_cycles  = 1;
    ov.t_hold    = 0.5;
    ov.t_descend_override = 10.0;
    ov.T_sim     = 12.5;
    ov.h_min     = 1.1 * pc.R;
end
