% STATUS: SCRATCH -- 2026-10-24 (user): scan comparison over seeds 1-10, same flow as temp_t3_flow_wait5 (start 15 um, wait 5 s,
%   regulation a_d 0.1 / tau_a 2 s, scan along u-hat from 12 s for 3 s), computed angle (A-bar', P_fix once moving, no B) vs estimated
%   angle (run_t2_3d_reg). Output test_results/temp_3d_wsonly/temp_t3_scan_seeds.mat | EXPIRES: when the scan summary is written
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);
W = load(fullfile(root, 'test_results', 'temp_3d_wsonly', 'temp_t3_flow_wait5.mat'));  o = W.o;  ob = W.ob;
seeds = 1:10;  n = numel(seeds);  rc = cell(n, 1);  rb = cell(n, 1);
if isempty(gcp('nocreate')); parpool('Processes'); end
parfor i = 1:n
    oc = o;  oc.seeds = seeds(i);  Rc = run_t3_xyz_fixed(oc);  rc{i} = Rc.runs{1};
    oe = ob;  oe.seeds = seeds(i);  Rb = run_t2_3d_reg(oe);  rb{i} = Rb.runs{1};
end
save(fullfile(root, 'test_results', 'temp_3d_wsonly', 'temp_t3_scan_seeds.mat'), 'rc', 'rb', 'o', 'ob', 'seeds');
fprintf('scan seeds done\n');
