% STATUS: SCRATCH -- 2026-10-24 (user: "is theta_x really worse than theta_y?"): computed angle only, 20 deg wall, 50 um and 15 um,
%   seeds 1-10, 20 s. Question: is the error of the y-z pair (A_yy - A_zz, -2 A_yz) larger than that of the x-z pair (3 seeds said ~2x)?
%   Output test_results/temp_3d_wsonly/temp_t3_planes10.mat | EXPIRES: when the question is answered
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);
hs = [50 15] / 2.25;  seeds = 1:10;  [HH, SS] = ndgrid(hs, seeds);  n = numel(HH);  xyz = cell(n, 1);
if isempty(gcp('nocreate')); parpool('Processes'); end
parfor i = 1:n
    Rx = run_t3_xyz_fixed(struct('theta_y', 20, 'h0', HH(i), 'seeds', SS(i), 'T', 20));  xyz{i} = Rx.runs{1};
end
save(fullfile(root, 'test_results', 'temp_3d_wsonly', 'temp_t3_planes10.mat'), 'xyz', 'HH', 'SS');
fprintf('planes10 done\n');
