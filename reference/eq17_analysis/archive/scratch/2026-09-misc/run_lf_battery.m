% PURPOSE (2026-09-24, user: "Meng's conditional forgetting on c as the main line -- start"): the c-only slope law (p = 2,
%   runner law 'f1p2'), plane init c = 0.053, prior 0.10, ramp T_d = 2 s, plane + cell, 10 seeds. Arms (slot-5 forgetting):
%     ma1 / ma3 / ma10   Meng-style conditional: lambda_k = max(exp(-alpha * max(0, innov1^2/S1 - 1)), 0.99), alpha in {1, 3, 10}
%     ib1 / ib3          information-driven:      lambda_k = max(exp(-beta * (1 - P55_upd/P55_pred)), 0.99),  beta  in {1, 3}
%     hold3 arms         final hold stretched to 3 s for the windup check: lambda = 1, uniform 0.999, ma3
%   Existing references: _sig010_ramp2 (lambda = 1), _sig010_lf999_ramp2 (uniform 0.999). Figures per arm (plot_plaw_walls) and
%   one table (report_lf_arms). Pre-registered (user 09-24): cell hold <= 0.052, wall <= 0.062; plane hold <= 0.010, descent spread
%   <= 1.2x of lambda = 1; 3 s hold sqrtP55 flat; firing concentrated below 3 R, ~0 in hold; c end vs c_eff(1.1). Sweeps, not chosen values.
function run_lf_battery()
    here = fileparts(mfilename('fullpath'));  addpath(here);  t0 = tic;
    base = struct('law', 'f1p2', 'traj', 'ramp', 'Td', 2, 'plants', {{'plane', 'cell'}}, 'seeds', 1:10, 'sig', 0.10);
    ARMS = { ...
        '_sig010_ma1',  struct('lambda_f_b_alpha', 1,  'lambda_f_b_chan', 'y1', 'lambda_f_b_floor', 0.99); ...
        '_sig010_ma3',  struct('lambda_f_b_alpha', 3,  'lambda_f_b_chan', 'y1', 'lambda_f_b_floor', 0.99); ...
        '_sig010_ma10', struct('lambda_f_b_alpha', 10, 'lambda_f_b_chan', 'y1', 'lambda_f_b_floor', 0.99); ...
        '_sig010_ib1',  struct('lambda_f_b_info', 1, 'lambda_f_b_floor', 0.99); ...
        '_sig010_ib3',  struct('lambda_f_b_info', 3, 'lambda_f_b_floor', 0.99); ...
        '_sig010',      struct('T_hold_end', 3.0); ...
        '_sig010_lf999', struct('lambda_f_b', 0.999, 'T_hold_end', 3.0); ...
        '_sig010_ma3',  struct('lambda_f_b_alpha', 3, 'lambda_f_b_chan', 'y1', 'lambda_f_b_floor', 0.99, 'T_hold_end', 3.0)};
    tags = {};
    for a = 1:size(ARMS, 1)
        o = base;  o.tag_suffix = ARMS{a, 1};  ex = ARMS{a, 2};
        fn = fieldnames(ex);  for q = 1:numel(fn); o.(fn{q}) = ex.(fn{q}); end
        fprintf('\n##### arm %s\n', ARMS{a, 1});
        run_plaw_walls(o);
        sfx = sprintf('%s_ramp2', ARMS{a, 1});  if isfield(ex, 'T_hold_end'); sfx = sprintf('%s_hold%g', sfx, ex.T_hold_end); end
        plot_plaw_walls('PI', sfx, {'plane', 'cell'}, 'f1p2');
        tags = [tags, {sprintf('f1p2_plane_PI%s', sfx), sprintf('f1p2_cell_PI%s', sfx)}];
        fprintf('##### done %s (%.0f s)\n', ARMS{a, 1}, toc(t0));
    end
    report_lf_arms([{'f1p2_plane_PI_sig010_ramp2', 'f1p2_cell_PI_sig010_ramp2', 'f1p2_plane_PI_sig010_lf999_ramp2', 'f1p2_cell_PI_sig010_lf999_ramp2'}, tags]);
    fprintf('##### all done (%.0f s)\n', toc(t0));
end
