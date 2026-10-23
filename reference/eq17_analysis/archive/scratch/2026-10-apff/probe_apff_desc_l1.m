% STATUS: SCRATCH -- 2026-10-16: scores for run_apff_desc_l1 (closed loop, Layer-1 + a' feed-forward off | lpf | true) next to the
%   a'-state run (formC_b ap_slot apP002F400H110, closed loop, same descent). e_a = a_true - a_hat. Per wall, seeds 1:5:
%   |e_a| mean over the descent, e_a at 0.9 / 0.95 / 1.0 s (mean (sd)), hold 1.05-1.2 s mean, min h; slope a'/true by band (descent).
function probe_apff_desc_l1()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(genpath(fullfile(root, 'model')));
    od = fullfile(root, 'test_results', 'temp_1009_ws');  WL = {'plane', 'cell'};  ARMS = {'off', 'lpf', 'true', 'state'};  seeds = 1:5;
    BANDS = [5 Inf; 2 5; 1 2];  BN = {'h>5', '2-5', '<2'};
    for iw = 1:2
        fprintf('\n[%s] arm  | |e_a| desc | e_a 0.90 s     | e_a 0.95 s     | e_a 1.00 s     | hold 1.05-1.2 | min h | slope/true median (%%neg) h>5, 2-5, <2\n', WL{iw});
        A = load(fullfile(od, sprintf('apcmp_apP002F400H110_%s_desc.mat', WL{iw})));
        for ia = 1:4
            E = [];  P = [];  T = [];  H = [];  AP = [];
            for iq = 1:numel(seeds)
                if ia < 4
                    r = load(fullfile(od, sprintf('apff_%s_%s_s%d.mat', ARMS{ia}, WL{iw}, seeds(iq))));
                    t = r.t;  e = r.a_true - r.a_hat;  h = max(r.h, 1.001);  ap = r.ap_hat;
                else
                    t = A.t;  e = A.a_true(:, iq) - A.a_hat(:, iq);  h = max(A.h_true(:, iq), 1.001);  ap = -A.w_hat(:, iq);
                end
                E = [E, e]; T = t; H = [H, h]; AP = [AP, ap]; %#ok<AGROW>
            end
            pk = @(tc) E(find(T >= tc - 1e-9, 1), :);
            Ld = T <= 1;  Lh = T >= 1.05;
            sl = '';
            if ia > 1
                for ib = 1:3
                    rat = [];  ng = 0;  nt = 0;
                    for iq = 1:numel(seeds)
                        apt = local_ap(H(:, iq), WL{iw});  L = Ld & H(:, iq) > BANDS(ib, 1) & H(:, iq) <= BANDS(ib, 2) & isfinite(AP(:, iq));
                        ng = ng + nnz(AP(L, iq) <= 0);  nt = nt + nnz(L);  Lp = L & AP(:, iq) > 0;  rat = [rat; AP(Lp, iq) ./ apt(Lp)]; %#ok<AGROW>
                    end
                    sl = [sl, sprintf('%6.2f (%2.0f)  ', median(rat), 100 * ng / nt)]; %#ok<AGROW>
                end
            end
            fprintf('  %-5s | %.3f      | %+.3f (%.3f) | %+.3f (%.3f) | %+.3f (%.3f) | %+.3f (%.3f) | %.3f | %s\n', ARMS{ia}, mean(mean(abs(E(Ld, :)))), ...
                    mean(pk(0.9)), std(pk(0.9)), mean(pk(0.95)), std(pk(0.95)), mean(pk(1.0)), std(pk(1.0)), mean(mean(E(Lh, :))), std(mean(E(Lh, :))), min(H(:)), sl);
        end
    end
end

function ap = local_ap(h, wall)
    d = 1e-4;  ap = (local_a(h + d, wall) - local_a(h - d, wall)) / (2 * d);
end

function a = local_a(h, wall)
    if strcmp(wall, 'cell'); a = 1 ./ cell_cperp_delta(h); return; end
    a = zeros(size(h));  for i = 1:numel(h); [~, c] = calc_correction_functions(h(i)); a(i) = 1 / c; end
end
