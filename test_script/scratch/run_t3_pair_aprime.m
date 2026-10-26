% STATUS: SCRATCH -- 2026-10-24 (user): fixed-point check of the A-bar' extension: same cells as run_t3_pair (20 deg wall, 50 / 15 um,
%   seeds 1-3, 20 s) with opts.aprime = true; compared with the saved run_t3_pair xyz arm (same parfor random streams).
%   Output test_results/temp_3d_wsonly/temp_t3_pair_aprime.mat | EXPIRES: when the A-bar' estimator is accepted
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);
S = load(fullfile(root, 'test_results', 'temp_3d_wsonly', 'temp_t3_pair.mat'));
HH = S.HH;  SS = S.SS;  n = numel(HH);  xyzp = cell(n, 1);
if isempty(gcp('nocreate')); parpool('Processes'); end
parfor i = 1:n
    Rx = run_t3_xyz_fixed(struct('theta_y', 20, 'h0', HH(i), 'seeds', SS(i), 'T', 20, 'aprime', true));  xyzp{i} = Rx.runs{1};
end
save(fullfile(root, 'test_results', 'temp_3d_wsonly', 'temp_t3_pair_aprime.mat'), 'xyzp', 'HH', 'SS');
fprintf('pair aprime done\n');
