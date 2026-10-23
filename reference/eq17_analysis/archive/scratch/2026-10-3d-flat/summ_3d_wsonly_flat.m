% STATUS: SCRATCH -- 2026-10-15, summary table for run_3d_wsonly_flat (ramp_tri: descent 0-1 s, triangle 1.10 <-> 2.00 1-3 s).
%   Motion first: descent near wall (0.8-1.0 s), then troughs (1, 2, 3 s) / tops (1.5, 2.5 s), triangle mean |e| (1-3 s).
%   e = true - estimate. Paired difference joint - oneway (same seed, same draws). | run: summ_3d_wsonly_flat
function T = summ_3d_wsonly_flat(R)
    if nargin < 1; R = '_n1'; end                  % 10-16: tag of the .mat ('_n1' numerator 1, '_n916' numerator 9/16)
    if ischar(R)
        here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
        S = load(fullfile(root, 'test_results', 'temp_3d_wsonly', ['temp_3d_wsonly_runs' R '.mat']));  R = S.R;
    end
    runs = [R.runs{:}];
    tp = [1 2 3 1.5 2.5];
    T = [];
    for wall = {'plane', 'shift'}
        for lat = [0 1]
            fprintf('\n=== wall %s, lateral %d (mean over seeds +- sd) ===\n', wall{1}, lat);
            fprintf('%-7s %-4s | desc .8-1 | trough 1/2/3 s          | top 1.5/2.5 s     | tri mean|e| | end w_s-hat\n', 'arm', 'gain');
            M = struct();
            for arm = {'joint', 'oneway'}
                sel = runs(strcmp({runs.wall}, wall{1}) & [runs.lat] == lat & strcmp({runs.arm}, arm{1}));
                for g = {'aN', 'aT'}
                    v = zeros(numel(sel), 8);
                    for i = 1:numel(sel)
                        r = sel(i);  e = r.([g{1} '_true']) - r.(g{1});
                        wd = r.t >= 0.8 & r.t <= 1.0;  wt = r.t >= 1.0;
                        pt = arrayfun(@(x) e(find(r.t >= x - 1e-9, 1)), tp);
                        if tp(3) > r.t(end) - 1e-9; pt(3) = e(end); end
                        v(i, :) = [mean(e(wd)), pt, mean(abs(e(wt))), r.ws_hat(end)];
                    end
                    M.(arm{1}).(g{1}) = v;
                    m = mean(v, 1);  s = std(v, 0, 1);
                    fprintf('%-7s %-4s | %+.3f      | %+.3f %+.3f %+.3f      | %+.3f %+.3f     | %.3f       | %+.3f +- %.3f\n', ...
                            arm{1}, g{1}, m(1), m(2), m(3), m(4), m(5), m(6), m(7), m(8), s(8));
                end
            end
            for g = {'aN', 'aT'}
                dv = M.joint.(g{1}) - M.oneway.(g{1});  m = mean(dv, 1);  s = std(dv, 0, 1);
                fprintf('j - o   %-4s | %+.4f(%.4f) | trough %+.4f %+.4f %+.4f | top %+.4f %+.4f | tri |e| %+.4f(%.4f) | ws %+.4f(%.4f)\n', ...
                        g{1}, m(1), s(1), m(2), m(3), m(4), m(5), m(6), m(7), s(7), m(8), s(8));
            end
            T.([wall{1} '_lat' num2str(lat)]) = M;
        end
    end
end
