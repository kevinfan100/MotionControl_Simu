% PURPOSE (2026-09-23, user: "R2 halved as a whole -- what changes?"): honesty readout for the y2-weight sweep. For each saved
%   run_plaw_walls result (.mat) print NIS2 = mean(innov2^2 / S2) over the descent and over the hold, the a_w honesty ratio
%   sd_seed(e_a) / mean(sqrt(P44)) in the same windows, and the c honesty. A whole-R2 scale of 1/2 makes S2 smaller, so the
%   filter's own NIS2 must rise; whether it rises to ~1 (was conservative) or beyond (now over-confident) is the question.
%   Usage: report_nis2_arms({'f1p2_plane_PI_sig010_ramp2', 'f1p2_plane_PI_sig010_r2h_ramp2', ...})
function report_nis2_arms(tags)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    rd = fullfile(root, 'test_results', 'temp_plaw_walls');
    fprintf('\n%-38s | %-17s | %-17s | %-12s | %s\n', 'run', 'NIS2 desc / hold', 'a_w hon desc/hold', 'c hon end', 'e_a hold');
    for k = 1:numel(tags)
        f = fullfile(rd, [tags{k} '.mat']);  if ~exist(f, 'file'); fprintf('%-38s | missing\n', tags{k}); continue; end
        r = load(f);  t = r.t;  seg = r.seg;
        desc = t >= seg(1) & t < seg(2);  hold = t >= seg(3);
        ok = isfinite(r.innov2) & isfinite(r.S2) & r.S2 > 0;
        nis = r.innov2.^2 ./ r.S2;  nis(~ok) = NaN;
        nis_d = mean(nis(desc, :), 'all', 'omitnan');  nis_h = mean(nis(hold, :), 'all', 'omitnan');
        ea = r.a_true - r.a_hat;  sPa = r.sP_a;
        hon_d = mean(std(ea(desc, :), 0, 2)) / max(mean(sPa(desc, :), 'all'), eps);
        hon_h = mean(std(ea(hold, :), 0, 2)) / max(mean(sPa(hold, :), 'all'), eps);
        hon_c = std(r.c_hat(end, :)) / max(mean(r.sP_c(end, :)), eps);
        fprintf('%-38s | %6.2f  / %6.2f  | %6.2f  / %6.2f  | %8.2f     | %+.4f +- %.4f\n', tags{k}, nis_d, nis_h, hon_d, hon_h, hon_c, ...
                mean(mean(ea(hold, :), 2)), mean(std(ea(hold, :), 0, 2)));
    end
end
