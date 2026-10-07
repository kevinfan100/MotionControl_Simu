% PURPOSE (2026-10-05, user: one running document motion_ctrl.tex): regenerate every figure motion_ctrl.tex includes, in page order.
%   Runs are cached (test_results/temp_5state_noap, test_results/temp_plaw_walls); a missing cache file is re-run, so a clean
%   checkout reproduces the figures. Fig. 2.1 (fig_conly_I_rtri) is the Layer-1 session's plot_conly_I_rtri.m and is not redrawn here.
%   STATUS: ACTIVE | companion of reference/eq17_analysis/derivation/motion_ctrl.tex (appendix = this list)
function make_motion_ctrl_figs()
    here = fileparts(mfilename('fullpath'));  cd(here);
    % ---- Part 1: first order (seed 1; plot functions run and cache their own simulations)
    plot_5state_noap_pair({'\epsilon = 0, fixed at 2.00 R', struct('kind', 'fixed', 'h', 2, 'eps', 0); ...
                           '\epsilon = 0, fixed at 1.10 R', struct('kind', 'fixed', 'h', 1.1, 'eps', 0)}, 'eps0_fixed');               % Fig. 1.1
    plot_5state_noap_pair({'\epsilon = Q_{44}/R_2 = 0', struct('kind', 'rtri', 'T_d', 2, 'f', 0, 'eps', 0); ...
                           '\epsilon = Q_{44}/R_2 = 10^{-5}', struct('kind', 'rtri', 'T_d', 2, 'f', 0, 'eps', 1e-5)}, 'eps0_vs_eps1e5_ramp_td2'); % Fig. 1.2
    plot_5state_noap_rtri(2, 0, 1);  plot_5state_noap_rtri(0.5, 0.5, 1);  plot_5state_noap_rtri(0.5, 5, 1);                          % Fig. 1.3-1.5
    % ---- Part 2: c-only (runs through run_plaw_walls, seed 1, plane | cell)
    id = fullfile(fileparts(fileparts(here)), 'test_results', 'temp_plaw_walls');
    base = struct('law', 'f1p2', 'sig', 0.10, 'seeds', 1, 'Td', 0.5);  base.plants = {'plane', 'cell'};
    RN = struct('q55_adapt', 'rationew', 'q55_adapt_P0', 0.15, 'q55_adapt_Prev', 0.05);
    local_run(id, base, struct('f_osc', 1), struct('q55_adapt', 'r2ratio', 'q55_adapt_rho', 1e-3), '_s1_eps1e3', {'plane'});   % Fig. 2.2 (plane eps)
    local_run(id, base, struct('f_osc', 1), struct('q55_adapt', 'r2ratio', 'q55_adapt_rho', 3e-3), '_s1_eps3e3', {'cell'});    % Fig. 2.2 (cell eps)
    local_run(id, base, struct('f_osc', 1), RN, '_s1_rn15_05', base.plants);                                                     % Fig. 2.3
    local_run(id, base, struct('f_osc', 1, 'h_top_tri', 10), RN, '_s1_rn15_05', base.plants);                                    % Fig. 2.4
    local_run(id, base, struct('f_osc', 5), RN, '_s1_rn15_05', base.plants);                                                     % Fig. 2.5
    P = 'P_{fix}';  Y2 = [-0.1 0.1];
    S = struct('crow', true, 'ylim2', Y2, 'name', 'slide6_besteps_f1');
    S.files = {'f1p2_plane_PI_s1_eps1e3_rtri_f1_td0p5.mat', 'f1p2_cell_PI_s1_eps3e3_rtri_f1_td0p5.mat'};
    S.labs = {'Q_{55} = \epsilon R_2, \epsilon = 10^{-3} (best for plane)', 'Q_{55} = \epsilon R_2, \epsilon = 3\times10^{-3} (best for cell)'};
    plot_conly_rtri_final(S);
    S = struct('crow', true, 'ylim2', Y2, 'name', 'slide7_pfix_f1');  S.labs = {P, P};
    S.files = {'f1p2_plane_PI_s1_rn15_05_rtri_f1_td0p5.mat', 'f1p2_cell_PI_s1_rn15_05_rtri_f1_td0p5.mat'};  plot_conly_rtri_final(S);
    S = struct('crow', true, 'ylim2', Y2, 'name', 'slide7b_pfix_f1_top10');  S.labs = {P, P};
    S.files = {'f1p2_plane_PI_s1_rn15_05_rtri_f1_td0p5_top1000.mat', 'f1p2_cell_PI_s1_rn15_05_rtri_f1_td0p5_top1000.mat'};
    S.wall_note = {', triangle 1.10-10.00 R', ', triangle 1.10-10.00 R'};  plot_conly_rtri_final(S);
    S = struct('crow', true, 'ylim2', Y2, 'name', 'slide8_pfix_f5');  S.labs = {P, P};
    S.files = {'f1p2_plane_PI_s1_rn15_05_rtri_f5_td0p5.mat', 'f1p2_cell_PI_s1_rn15_05_rtri_f5_td0p5.mat'};  plot_conly_rtri_final(S);
end

function local_run(id, base, traj, q55, suffix, plants)    % run only the walls whose cache file is missing
    o = base;  fn = [fieldnames(traj); fieldnames(q55)];  sv = [struct2cell(traj); struct2cell(q55)];
    for i = 1:numel(fn); o.(fn{i}) = sv{i}; end
    o.tag_suffix = suffix;  tg = strrep(sprintf('_rtri_f%g_td%g', o.f_osc, o.Td), '.', 'p');
    if isfield(o, 'h_top_tri'); tg = [tg sprintf('_top%03d', round(100 * o.h_top_tri))]; end
    need = plants(cellfun(@(w) ~exist(fullfile(id, sprintf('f1p2_%s_PI%s%s.mat', w, suffix, tg)), 'file'), plants));
    if isempty(need); return; end
    o.plants = need;  run_plaw_walls(o);
end
