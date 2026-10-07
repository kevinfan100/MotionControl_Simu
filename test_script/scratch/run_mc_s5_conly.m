% STATUS: SCRATCH -- 2026-10-07, motion_ctrl Section 5 (position-dependent motion gain, c-only slope law, q55 = 0), plane wall,
%   lambda_c 0.4, production refinements on (formC_b defaults; not shown in the document). Same two trajectories as Section 4:
%     traj 'desc' : the Section 2 trajectory (z 22.2 -> 1.10 R in 5 s, x/y 1 Hz 9 um sines)        -> tag _mcs5_desc
%     traj 'tri'  : 22.2 -> 1.10 R in 1 s, then triangle 1.10 <-> 10 R, 2 Hz, 2 s                 -> tag _mcs5_tri
%   c seeded at 0 on every wall (user 10-07; before: the plane's band fit 0.0533), sqrt(P_cc[0]) = 0.1 (opts.sig).
%     traj 'raster110' : raster scan at a fixed height 1.10 R (user 10-07): x 4 Hz triangle +-20 um, y +1 um at every
%                        x turnaround (tan_v_stair), 2 s                                                 -> tag _mcs5_raster110
%   Results in test_results/temp_plaw_walls (run_plaw_walls naming); load with the tag.
function run_mc_s5_conly(trajs, seeds, q55, plants)
    if nargin < 1 || isempty(trajs); trajs = {'desc', 'tri'}; end
    if nargin < 2 || isempty(seeds); seeds = 1:5; end
    if nargin < 3 || isempty(q55); q55 = 'none'; end
    if nargin < 4 || isempty(plants); plants = {'plane'}; end   % Section 6: {'plane', 'cell'} (estimator code unchanged, plant only)
    here = fileparts(mfilename('fullpath'));  addpath(here);
    for it = 1:numel(trajs)
        o = struct('law', 'f1p2', 'seeds', seeds, 'sig', 0.1, 'plants', {plants}, 'lambda_c', 0.4);
        if ~strcmp(q55, 'none'); o.q55_adapt = q55; o.q55_adapt_P0 = 0.15; end
        o.c_seed = struct('plane', 0, 'cell', 0, 'sphere', 0, 'sphere_pt', 0);   % c[0] = 0 (user 10-07)
        switch trajs{it}
            case 'desc'
                o.traj = 'ramp';  o.Td = 5;
                o.ov_extra = struct('trajectory_type', 'ramp_descent', 'h_init', 50, 'h_bottom', 1.1 * 2.25, 't_hold', 0, ...
                                    't_descend_override', 5, 'T_sim', 5, 'amp_u', -9, 'freq_u', 1, 'amp_v', -9, 'freq_v', 1, 'tan_shape', 1);
                o.tag_suffix = '_mcs5_desc';
            case {'rh050', 'rh100', 'rh200'}               % 10-07 (user): wall-type trajectory, 22.2 -> 1.10 R in T_d, then hold 0.2 T_d
                Td = str2double(trajs{it}(3:end)) / 100;
                o.traj = 'ramp';  o.Td = Td;
                o.ov_extra = struct('trajectory_type', 'ramp_descent', 'h_init', 50, 'h_bottom', 1.1 * 2.25, 't_hold', 0, ...
                                    't_descend_override', Td, 'T_sim', 1.2 * Td);
                o.tag_suffix = ['_mcs5_' trajs{it}];
            case {'fix200', 'fix110'}                      % 10-07: parked at 2.00 / 1.10 R for 5 s (no motion)
                hf = str2double(trajs{it}(4:end)) / 100 * 2.25;
                o.traj = 'ramp';  o.Td = 5;
                o.ov_extra = struct('trajectory_type', 'ramp_descent', 'h_init', hf, 'h_bottom', hf, 't_hold', 0, 't_descend_override', 5, 'T_sim', 5);
                o.tag_suffix = ['_mcs5_' trajs{it}];
            case {'raster110', 'raster200'}               % 10-07 (user): raster scan parallel to the wall at a fixed height
                hf = str2double(trajs{it}(7:end)) / 100 * 2.25;
                o.traj = 'ramp';  o.Td = 2;
                o.ov_extra = struct('trajectory_type', 'ramp_descent', 'h_init', hf, 'h_bottom', hf, 't_hold', 0, 't_descend_override', 2, 'T_sim', 2, ...
                                    'amp_u', -20, 'freq_u', 4, 'tan_shape', 1, 'amp_v', -1, 'freq_v', 4, 'tan_v_stair', 1);   % u_hat = -x, v_hat = -y
                o.tag_suffix = ['_mcs5_' trajs{it}];
            case 'tri'
                o.traj = 'ramp_tri';  o.f_osc = 2;  o.h_top_tri = 10;
                o.tag_suffix = '_mcs5_tri';
        end
        if ~strcmp(q55, 'none'); o.tag_suffix = [o.tag_suffix '_' q55]; end
        run_plaw_walls(o);
    end
end
