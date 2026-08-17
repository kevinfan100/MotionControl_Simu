% PURPOSE: gate-3 support -- find WHERE a shape slot's information vanishes, by
%   reading the per-step Jacobian columns the production filter actually builds.
%   Measurement information about slot theta = H2(theta)^2 / R2 ; the state path
%   contribution is F(4, theta). A zero crossing of either column is a
%   candidate degeneracy condition (gate 3), not just a low-information segment.
% EXPIRES: when the per-state degeneracy conditions are in the .tex.
function rep = probe_shape_info_rate(L, slots, names, tag, t_end)
    if nargin < 5 || isempty(t_end); t_end = 4.8; end
    Lz = L([L.ax] == 3); N = numel(Lz); Ts = t_end / N; t = (0:N-1)' * Ts;
    ns = numel(slots);
    h2 = nan(N, ns); f4 = nan(N, ns); R2 = nan(N, 1);
    for k = 1:N
        if ~isempty(Lz(k).H{2})
            h2(k, :) = Lz(k).H{2}(slots); R2(k) = Lz(k).R(2);
        end
        f4(k, :) = Lz(k).F(4, slots);
    end
    fprintf('\n===================== %s =====================\n', tag);
    for j = 1:ns
        I = h2(:, j).^2 ./ R2;
        cj = h2(~isnan(h2(:, j)), j);
        zc = sum(diff(sign(cj)) ~= 0);
        zf = sum(diff(sign(f4(:, j))) ~= 0);
        fprintf('%-9s H2 col : max|.| %.3e   sign changes %d\n', names{j}, max(abs(cj)), zc);
        fprintf('%-9s F4 col : max|.| %.3e   sign changes %d\n', '', max(abs(f4(:, j))), zf);
        fprintf('%-9s info I : max %.3e  med %.3e  med in hold t<0.5 %.3e  ratio %.1e\n', '', ...
                max(I), median(I, 'omitnan'), median(I(t < 0.5), 'omitnan'), ...
                median(I(t < 0.5), 'omitnan') / median(I, 'omitnan'));
        if zc > 0
            tz = t(find(diff(sign(cj)) ~= 0) + 1);
            fprintf('%-9s H2 zero crossings at t = %s ... (%d total)\n', '', ...
                    mat2str(round(tz(1:min(6, numel(tz)))', 3)), numel(tz));
        end
    end
    rep = struct('t', t, 'h2', h2, 'f4', f4, 'R2', R2, 'names', {names});
end
