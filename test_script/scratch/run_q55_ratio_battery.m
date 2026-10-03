% PURPOSE (2026-10-03): reproduce every arm of the c-slot Q/R-ratio line (0930_q55_ratio_design.tex, ledger C82-C85 / R70-R74) from
%   run_plaw_walls alone: c-only slope law 'f1p2' (p = 2), plane init, prior 0.10, 10 seeds, plane + cell. One entry per arm, grouped
%   as the document's pages; pass a group name (or 'all') to run it. Figures: plot_plaw_walls / plot_f1p2_cell_lf_compare / plot_rho_diag /
%   plot_osc_potential_check (see the document's figure names). The 09-30 .. 10-02 one-off drivers lived in the job tmp folder; this is
%   their tracked replacement.  EXPIRES: with the c-only Q design line.
function run_q55_ratio_battery(group)
    if nargin < 1; group = 'all'; end
    B = struct('law', 'f1p2', 'plants', {{'plane', 'cell'}}, 'seeds', 1:10, 'sig', 0.10);
    ramp2 = struct('traj', 'ramp', 'Td', 2);
    osc05 = struct('traj', 'osc', 'Td', 0.5, 'descent_linear', true, 'h_bottom', 1.10, 'h_top', 2.0);
    R = @(P0) struct('q55_adapt', 'ratio', 'q55_adapt_P0', P0);
    G = struct();
    % --- P_0 scan (ramp 2 s): base | ratio 0.1 (prior) | 0.2 | 0.3
    G.p0scan = { m(ramp2, '_sig010'); m(ramp2, '_sig010_rho', struct('q55_adapt', 'ratio')); m(ramp2, '_sig010_rho20', R(0.2)); m(ramp2, '_sig010_rho30', R(0.3)) };
    % --- speed (ratio 0.2 and base): T_d 0.3 / 0.5 / 5 s
    G.speed = { m(struct('traj', 'ramp', 'Td', 0.3), '_sig010_rho20', R(0.2)); m(struct('traj', 'ramp', 'Td', 0.5), '_sig010_rho20', R(0.2)); m(struct('traj', 'ramp', 'Td', 5), '_sig010_rho20', R(0.2)); ...
                m(struct('traj', 'ramp', 'Td', 0.3), '_sig010'); m(struct('traj', 'ramp', 'Td', 0.5), '_sig010'); m(struct('traj', 'ramp', 'Td', 5), '_sig010') };
    % --- opponents (ramp 2 s): constant rho (R70), literal R2 ratio (R71)
    G.opponents = { m(ramp2, '_sig010_rhoc025', struct('q55_adapt', 'rhoconst', 'q55_adapt_rho', 1/25^2)); m(ramp2, '_sig010_rhoc100', struct('q55_adapt', 'rhoconst', 'q55_adapt_rho', 1/100^2)); ...
                    m(ramp2, '_sig010_r2r0003', struct('q55_adapt', 'r2ratio', 'q55_adapt_rho', 0.0003)); m(ramp2, '_sig010_r2r003', struct('q55_adapt', 'r2ratio', 'q55_adapt_rho', 0.003)); ...
                    m(ramp2, '_sig010_r2r010', struct('q55_adapt', 'r2ratio', 'q55_adapt_rho', 0.01)); m(ramp2, '_sig010_r2r030', struct('q55_adapt', 'r2ratio', 'q55_adapt_rho', 0.03)); ...
                    m(ramp2, '_sig010_r2r300', struct('q55_adapt', 'r2ratio', 'q55_adapt_rho', 0.3)) };
    % --- oscillation, constant-velocity descent 2 s (C84): canon band and 1.05-2.0 R, base and ratio 0.2
    o2 = struct('traj', 'osc', 'Td', 2, 'descent_linear', true);  o2n = o2;  o2n.h_bottom = 1.05;  o2n.h_top = 2.0;
    G.osc = { m(o2, '_sig010'); m(o2, '_sig010_rho20', R(0.2)); m(o2n, '_sig010'); m(o2n, '_sig010_rho20', R(0.2)) };
    % --- 0.5 s descent + 1 Hz 1.10-2.0 R: sine | triangle (R74); a-slot levers and in-band tracking were on the cosine osc2 (see tmp list in memory)
    G.wave = { m(osc05, '_sig010_rho20', R(0.2)); m(setfield(osc05, 'wave', 'tri'), '_sig010_rho20', R(0.2)) };   %#ok<SFLD>
    % --- innovation gate (C85) and the three-way (R71 on the same trajectory)
    G.gate = { m(osc05, '_sig010_rg02', struct('q55_adapt', 'ratiogate', 'q55_adapt_P0', 0.2, 'q55_adapt_N', 100, 'q55_adapt_thr', 0.2)); ...
               m(osc05, '_sig010_rg03', struct('q55_adapt', 'ratiogate', 'q55_adapt_P0', 0.2, 'q55_adapt_N', 100, 'q55_adapt_thr', 0.3)); ...
               m(ramp2, '_sig010_rg02', struct('q55_adapt', 'ratiogate', 'q55_adapt_P0', 0.2, 'q55_adapt_N', 100, 'q55_adapt_thr', 0.2)); ...
               m(ramp2, '_sig010_rg03', struct('q55_adapt', 'ratiogate', 'q55_adapt_P0', 0.2, 'q55_adapt_N', 100, 'q55_adapt_thr', 0.3)); ...
               m(osc05, '_sig010_r2r030', struct('q55_adapt', 'r2ratio', 'q55_adapt_rho', 0.03)) };
    names = fieldnames(G);  if ~strcmp(group, 'all'); names = {group}; end
    t0 = tic;
    for g = 1:numel(names)
        arms = G.(names{g});
        for a = 1:numel(arms)
            o = B;  fn = fieldnames(arms{a});  for q = 1:numel(fn); o.(fn{q}) = arms{a}.(fn{q}); end
            fprintf('\n##### %s / %s\n', names{g}, o.tag_suffix);  run_plaw_walls(o);  fprintf('##### done (%.0f s)\n', toc(t0));
        end
    end
end

function s = m(traj, tag, extra)
    s = traj;  s.tag_suffix = tag;
    if nargin > 2; fn = fieldnames(extra); for q = 1:numel(fn); s.(fn{q}) = extra.(fn{q}); end; end
end
