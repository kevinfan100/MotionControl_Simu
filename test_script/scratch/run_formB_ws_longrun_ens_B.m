% run_formB_ws_longrun_ens_B.m -- PURPOSE: E1 arm B of the seed-11 self-lock
%   investigation (2026-08-03): 6 fresh seeds, LONG identification run
%   (n_cycles 32, T 34.8 s), c4 conditions (lock p, TRUE wall injected +5%,
%   honest prior 0.111). Trap criterion (pre-registered): end |z| > 2 where
%   z = |dev_end - 5| / sqP_end in percent units.
%   EXPIRES: absorbed into the self-lock case file.
here = fileparts(mfilename('fullpath'));
proj = fullfile(here, '..', '..');
pdirs = {'model','model/dual_track','model/diag','model/thermal_force', ...
         'model/config','model/wall_effect','model/controller', ...
         'model/trajectory','test_script/integration'};
for i = 1:numel(pdirs); addpath(fullfile(proj, pdirs{i})); end
res_file = fullfile(proj, 'test_results', 'formB_longrun_ens_B.mat');
SEEDS = 5007:5012; INJ = 0.05; N_CYC = 32; AX = 3;
ov = struct('lock_b', false, 'lock_p', true, 'lock_ws', false, 'Pf_ws_std', 0.111);
if exist(res_file, 'file'); load(res_file, 'E');
else; E = struct('done', zeros(1, numel(SEEDS))); end
for i = 1:numel(SEEDS)
    if E.done(i); continue; end
    o = struct('seeds', SEEDS(i), 'ws_inject', INJ, 'verbose', false, ...
               'ctrl_const_override', ov, ...
               'config_override', struct('n_cycles', N_CYC, 'T_sim', 0.5+1.0+N_CYC+1.3));
    out = run_formB_ws(o); r = out.runs{1};
    t = r.tout(:); dev = 100*(r.ws_hat_out(:,AX)-1); sqP = 100*r.P_ws_out(:,AX);
    E.dev_end(i) = dev(end); E.sqP_end(i) = sqP(end);
    E.z_end(i) = abs(dev(end) - 100*INJ) / sqP(end);
    E.dev_min(i) = min(dev); E.dev_max(i) = max(dev);
    E.b_end(i) = r.b_hat_out(end, AX);
    E.tds = t(1:20:end); E.traj{i} = dev(1:20:end); E.band{i} = sqP(1:20:end);
    E.done(i) = 1; save(res_file, 'E');
    fprintf('E1B seed %d: dev_end %+.2f  sqP %.2f  z_end %.2f  b_end %.4f\n', ...
            SEEDS(i), dev(end), sqP(end), E.z_end(i), E.b_end(i));
end
fprintf('E1B done: trapped(z>2) %d/%d\n', nnz(E.z_end > 2), numel(SEEDS));
