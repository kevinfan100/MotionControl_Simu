% STATUS: SCRATCH -- 2026-10-24 (user): single seed, 20 deg wall, start at 15 um, hold 5 s or 10 s, then scan 3 s along the estimated
%   u-hat (du_d = 0.011 R per step, as the 10-21 flow), no regulation. Arms: computed angle (run_t3_xyz_fixed) and estimated angle
%   (run_t2_3d_reg, fixed_point = no regulation, scan from t_scan). Output test_results/temp_3d_wsonly/temp_t3_scan15.mat
%   EXPIRES: when the scan comparison is read
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);
h0 = 15 / 2.25;  du = 0.011;  Tsc = 3;  holds = [5 10];  seed = 1;  out = struct();
for j = 1:2
    th = holds(j);
    Rx = run_t3_xyz_fixed(struct('theta_y', 20, 'h0', h0, 'seeds', seed, 'T', th + Tsc, 't_scan', th, 'du_d', du));
    R2 = run_t2_3d_reg(struct('theta_y', 20, 'h0', h0, 'seeds', seed, 'fixed_point', true, 't_scan', th, 'T_scan', Tsc, 'du_d', du, ...
                              'save', false, 'tag', 'ref'));
    out.xyz{j} = Rx.runs{1};  out.ref{j} = R2.runs{1};
end
save(fullfile(root, 'test_results', 'temp_3d_wsonly', 'temp_t3_scan15.mat'), 'out', 'holds', 'h0', 'du', 'Tsc', 'seed');
