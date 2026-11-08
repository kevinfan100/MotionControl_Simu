% STATUS: SCRATCH -- 2026-11-02 (user): curvature coupling off before the scan (fix in run_t3_xyz_fixed.m), flat wall true (8,-8), plane scan, seeds 1-5, a_d .07, 40 um/s, 3 lines of 36 um. A old (kappa_couple_always) | B fixed | C fixed, no angle Q (q_theta_off) | D no curvature slot | E neither.
%   Pre-registered: the fix leaves the no-motion part unchanged; the step of the w-hat error at the scan start stays in A; C or D without the step names its source. Sanity: finished runs, estimator finite. MATLAB window 25. Cached in temp_plane_scan.mat. Output fig_1102_plane_scan.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');  Rr = 2.25;  Ts = 1 / 1612;  ad = 0.07;  NS = 5;  Tx = 8;  Ty = -8;  hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  cf = fullfile(fo, 'temp_plane_scan.mat');
base = struct('theta_x', Tx, 'theta_y', Ty, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'doc_timing', true, 'theta_out', true, 'gap_gate', 2, 'gate_one', true, 'hold_until', 10, 'tau_a', 0.06, 'a_d', ad, 'descent_v', 15 - hs * Rr, 'descent_T', 1, 'post_wait', [0 1], 'ap_q', true, 'ap_floor_law', true, 'sP_fix_theta_deg', 2, 'q_geo', [0.9 0.9], 'tag', 'ps', 'du_d', 40 * Ts / Rr, 'kappa_state', true, 'kappa_from_scan', true, 'kappa_pfix', 0.3, 'scan_ramp', 0.5, 'scan_len', 1e6, 'raster_local', [36 1 3 0.05], 'T', 12 + 1.2 * 3 * 36 / 40 + 1);
[IA, ID] = ndgrid(1:4, 1:NS);  N = numel(IA);
if isfile(cf)
    load(cf, 'rs');
else
    rs = cell(N, 1);
    if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
    pctRunOnAll warning('off', 'all');
    parfor q = 1:N
        o = base;  o.seeds = ID(q);
        switch IA(q)
            case 1; o.kappa_couple_always = true;
            case 3; o.q_theta_off = true;
            case 4; o = rmfield(o, {'kappa_state', 'kappa_from_scan', 'kappa_pfix'});
        end
        R = run_t3_xyz_fixed(o);  r = R.runs{1};
        rs{q} = struct('t', r.t(:), 'kend', r.kend, 'contact', r.contact, 'tsc', r.tsc, 'te', r.rs_tend, 'thx', r.thx(:), 'thy', r.thy(:), 'th_tr', r.th_tr, 'h', r.h(:), 'aN', r.aN(:), 'aN_tr', r.aN_tr(:));
    end
    save(cf, 'rs', '-v7.3');
end
cf2 = fullfile(fo, 'temp_plane_scan_e.mat');
if isfile(cf2)
    load(cf2, 're');
else
    re = cell(NS, 1);
    parfor q = 1:NS
        o = rmfield(base, {'kappa_state', 'kappa_from_scan', 'kappa_pfix'});  o.q_theta_off = true;  o.seeds = q;  R = run_t3_xyz_fixed(o);  r = R.runs{1};
        re{q} = struct('t', r.t(:), 'kend', r.kend, 'contact', r.contact, 'tsc', r.tsc, 'te', r.rs_tend, 'thx', r.thx(:), 'thy', r.thy(:), 'th_tr', r.th_tr, 'h', r.h(:), 'aN', r.aN(:), 'aN_tr', r.aN_tr(:));
    end
    save(cf2, 're', '-v7.3');
end
rs = [rs(:); re(:)];  IA = [IA(:); 5 * ones(NS, 1)];
nm = {'A old', 'B fixed', 'C fixed, no angle Q', 'D no \kappa slot', 'E no \kappa, no angle Q'};  cc = [0 0 0; 0.85 0 0; 0 0.6 0.2; 0 0.25 0.85; 0.6 0.2 0.8];  f = figure(25);  clf(f);  set(f, 'Color', 'w', 'Position', [60 80 1500 640], 'Visible', 'on', 'Name', 'plane scan, 5 seeds');  a = axes(f);  hold(a, 'on');  hl = gobjects(5, 1);
for j = 1:5
    id = find(IA == j);  n = min(cellfun(@(r) find(isfinite(r.thx), 1, 'last'), rs(id)));  t = rs{id(1)}.t(1:n);  E = zeros(n, NS);  for s = 1:NS; r = rs{id(s)};  E(:, s) = acosd(min(sum(local_w(r.thx(1:n), r.thy(1:n)) .* local_w(Tx * ones(n, 1), Ty * ones(n, 1)), 2), 1)); end
    er = sqrt(mean(movmean(E.^2, 81, 1), 2));  k = (1:3:n)';  hl(j) = plot(a, t(k), er(k), 'Color', cc(j, :), 'LineWidth', 1.3);
    fin = cellfun(@(r) isfinite(r.te) && ~r.contact && find(isfinite(r.thx), 1, 'last') >= find(r.t >= r.te, 1), rs(id));  mg = cellfun(@(r) 1e3 * (min(r.h(r.t >= 12 & isfinite(r.h))) * Rr - Rr), rs(id));  w = @(a0, b0) rms(er(t >= a0 & t < b0));
    fprintf('%-22s finished %d/%d, data to %.1f s, scan end %.1f s | err 11.8-12 s %.2f, 12.1-12.5 s %.2f, 12.5-14 s %.2f, 14-16 s %.2f deg | min gap - contact %s nm\n', nm{j}, sum(fin), NS, t(end), rs{id(1)}.te, w(11.8, 12), w(12.1, 12.5), w(12.5, 14), w(14, 16), mat2str(round(mg', 1)));
end
xline(a, 12, 'k:', 'HandleVisibility', 'off');  xlim(a, [10 16]);  ylim(a, [0 8]);  xlabel(a, 'Time (sec)');  ylabel(a, ['angle error of w' char(770) '  [deg]']);  box(a, 'on');  set(a, 'FontSize', 15, 'FontWeight', 'bold', 'LineWidth', 1.2);  legend(a, hl, nm, 'Location', 'northoutside', 'Orientation', 'horizontal');
exportgraphics(f, fullfile(fo, 'fig_1102_plane_scan.png'), 'Resolution', 130);
function w = local_w(thx, thy)
    tx = thx(:) * pi / 180;  ty = thy(:) * pi / 180;  w = [sin(ty) .* cos(tx), -sin(tx), cos(ty) .* cos(tx)];
end
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
