% PURPOSE (2026-09-29, layer 1 of the lambda_f / Q design): one table over saved run_plaw_walls results (c-only, plane init,
%   ramp 2 s) for the ceiling arms -- per plant and arm: e_a at the wall and in the hold, near-wall spread (sd_seed of e_a at the
%   wall), c_hat at the descent end with its sd_seed and sqrt(P) (honesty = sd / sqrtP), sqrt(P44) at the wall, NIS1 / NIS2 over
%   the descent. Usage: report_layer1_arms({'', '_q55cell', '_q55plane', '_lq44c70', '_lq44c10'})
function report_layer1_arms(ARM)
    if nargin < 1; ARM = {'', '_q55cell', '_q55plane', '_lq44c70', '_lq44c10'}; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  rd = fullfile(root, 'test_results', 'temp_plaw_walls');
    fprintf('\n%-6s %-10s | %8s %7s %8s | %7s %7s %7s %6s | %8s | %5s %5s\n', 'plant', 'arm', 'e_a_wall', 'sd', 'e_a_hold', 'c_end', 'sd', 'sqrtP', 'hon', 'sqrtP44w', 'NIS1', 'NIS2');
    for pl = {'plane', 'cell'}
        for ia = 1:numel(ARM)
            f = fullfile(rd, sprintf('f1p2_%s_PI_sig010%s_ramp2.mat', pl{1}, ARM{ia}));
            if ~exist(f, 'file'); fprintf('%-6s %-10s | missing\n', pl{1}, ARM{ia}); continue; end
            r = load(f);  t = r.t;  desc = t >= r.seg(1) & t < r.seg(2);  hold_ = t >= r.seg(3);  ie = find(desc, 1, 'last');
            ea = r.a_true - r.a_hat;  m = mean(ea, 2);  s = std(ea, 0, 2);
            cm = mean(r.c_hat, 2);  cs = std(r.c_hat, 0, 2);  cp = mean(r.sP_c, 2);  pa = mean(r.sP_a, 2);
            ok1 = isfinite(r.innov1) & isfinite(r.S1) & r.S1 > 0;  n1 = r.innov1.^2 ./ r.S1;  n1(~ok1) = NaN;
            ok2 = isfinite(r.innov2) & isfinite(r.S2) & r.S2 > 0;  n2 = r.innov2.^2 ./ r.S2;  n2(~ok2) = NaN;
            lab = ARM{ia};  if isempty(lab); lab = 'base'; end
            fprintf('%-6s %-10s | %+8.3f %7.3f %+8.3f | %7.3f %7.3f %7.4f %6.2f | %8.4f | %5.2f %5.2f\n', pl{1}, lab, m(ie), s(ie), mean(m(hold_)), ...
                    cm(ie), cs(ie), cp(ie), cs(ie) / cp(ie), pa(ie), mean(n1(desc, :), 'all', 'omitnan'), mean(n2(desc, :), 'all', 'omitnan'));
        end
    end
end
