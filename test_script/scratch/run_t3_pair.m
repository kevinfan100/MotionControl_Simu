% STATUS: SCRATCH -- 2026-10-24 (user): simple comparison, wall theta_y = 20 deg, fixed points 50 um (22.2 R) and 15 um (6.67 R),
%   T = 20 s, seeds 1-3; arms 'xyz' (run_t3_xyz_fixed, angles from A-hat by the plane formulas) and '1015' (run_t2_3d_reg, fixed_point,
%   angles as states, prior 10 deg). Output test_results/temp_3d_wsonly/temp_t3_pair.mat | EXPIRES: when the comparison is read
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
addpath(here);
Rr = 2.25;  hs = [50 15] / Rr;  seeds = 1:3;  T = 20;  thy = 20;
[HH, SS] = ndgrid(hs, seeds);  n = numel(HH);
xyz = cell(n, 1);  ref = cell(n, 1);
if isempty(gcp('nocreate')); parpool('Processes'); end
parfor i = 1:n
    Rx = run_t3_xyz_fixed(struct('theta_y', thy, 'h0', HH(i), 'seeds', SS(i), 'T', T));  xyz{i} = Rx.runs{1};
    R2 = run_t2_3d_reg(struct('theta_y', thy, 'h0', HH(i), 'seeds', SS(i), 'fixed_point', true, 't_scan', 100, 'T_scan', T - 100, ...
                              'save', false, 'tag', 'ref'));  ref{i} = R2.runs{1};
end
save(fullfile(root, 'test_results', 'temp_3d_wsonly', 'temp_t3_pair.mat'), 'xyz', 'ref', 'HH', 'SS', 'T', 'thy');
fprintf('pair done: %d cells\n', n);
