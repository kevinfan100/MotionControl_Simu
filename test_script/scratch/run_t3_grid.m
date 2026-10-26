% STATUS: SCRATCH -- 2026-10-23 (user): first test of 1023_estimator_3d_xyz.tex. Grid wall angle theta_y {0, 10, 20, 30} deg x fixed
%   height h0 {1.5, 3, 6.67} R x seeds 1-3, T = 5 s, p_d held. Two arms, paired seeds: 'xyz' = run_t3_xyz_fixed (angles from A-hat),
%   '1015' = run_t2_3d_reg with fixed_point (angles as states, prior 10 deg). Output test_results/temp_3d_wsonly/temp_t3_grid.mat
%   EXPIRES: when the first xyz test is read
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
addpath(here);
ths = [0 10 20 30];  hs = [1.5 3 20/3];  seeds = 1:3;  T = 5;
[TH, HH, SS] = ndgrid(ths, hs, seeds);  n = numel(TH);
xyz = cell(n, 1);  ref = cell(n, 1);
if isempty(gcp('nocreate')); parpool('Processes'); end
parfor i = 1:n
    o = struct('theta_y', TH(i), 'h0', HH(i), 'seeds', SS(i), 'T', T);
    Rx = run_t3_xyz_fixed(o);  xyz{i} = Rx.runs{1};
    o2 = struct('theta_y', TH(i), 'h0', HH(i), 'seeds', SS(i), 'fixed_point', true, 't_scan', 100, 'T_scan', T - 100, 'save', false, 'tag', 'ref');
    R2 = run_t2_3d_reg(o2);  ref{i} = R2.runs{1};
end
save(fullfile(root, 'test_results', 'temp_3d_wsonly', 'temp_t3_grid.mat'), 'xyz', 'ref', 'TH', 'HH', 'SS', 'T');
fprintf('grid done: %d cells\n', n);
