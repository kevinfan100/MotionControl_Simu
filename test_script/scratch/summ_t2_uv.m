% STATUS: SCRATCH -- 2026-10-20 (user: no gate, does holding u, v stop the divergence? regulation only, 3 s). Per-seed table of
%   run_t2_3d_reg arms (tags, seeds merged): fail (contact / NaN / end angle off by > 5 deg), min h, max |e_theta| in the run, end
%   e_theta_y, e_theta_x, end u, v of the probe relative to the start point in the TRUE local frame [um], max |u|, |v| over the run [um], end a_N-hat / a_N and a'_N-hat / a'_N (ratios to the truth).
%   | run: summ_t2_uv({'ng_base_a','ng_base_b'}, {'ng_uv_a','ng_uv_b'})
function summ_t2_uv(varargin)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_3d_wsonly');
    for ia = 1:nargin
        TG = varargin{ia};  nf = 0;  nr = 0;
        fprintf('=== %s\n seed | fail | min h | max|e_th| | end e_thy e_thx | end u  v [um] | max|u| max|v| | end aN-hat/aN  aN''-hat/aN''\n', strjoin(TG, '+'));
        for it = 1:numel(TG)
            S = load(fullfile(od, ['temp_t2_' TG{it} '.mat']));  ty = S.R.opts.theta_y;
            V = [cosd(ty) 0 sind(ty); 0 1 0; -sind(ty) 0 cosd(ty)];
            for i = 1:numel(S.R.runs)
                r = S.R.runs{i};  k = 1:r.kend;  nr = nr + 1;
                e = [r.thy(k) - ty, r.thx(k)];
                bad = r.contact || any(isnan(r.aN(k))) || any(abs(e(end, :)) > 5);  nf = nf + bad;
                uv = V' * (r.p(r.kend, :) - r.p(1, :))';  UV = (V(:, 1:2)' * (r.p(k, :) - r.p(1, :))')';
                fprintf('  %3d |  %d   | %5.3f | %8.1f | %+6.2f %+6.2f | %+6.2f %+6.2f | %5.1f %5.1f | %.2f  %.2f\n', r.seed, bad, min(r.h(k)), ...
                        max(abs(e(:))), e(end, 1), e(end, 2), uv(1), uv(2), max(abs(UV(:, 1))), max(abs(UV(:, 2))), ...
                        r.aN(r.kend) / r.aN_true(r.kend), r.apN(r.kend) / r.apN_true(r.kend));
            end
        end
        fprintf(' fail %d / %d\n', nf, nr);
    end
end
