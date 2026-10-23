% PURPOSE (2026-09-18): after the coordinate change w_s = contact height (1) -> wall surface (0) with every [1/1] constant
%   shifted by -1, the closed loop must be bit-identical (the law only sees g + c0 and g + d1). Compares the rerun result
%   sets against the copies made before the change (test_results/<set>_ws1): max |a_hat_new - a_hat_old| over all seeds
%   and steps, and max |c0_new + 1 - c0_old|, |d1_new + 1 - d1_old| for version A. [2/2] is listed for information only
%   (its family prior is basis-dependent, so P/N arms are not expected to match; T arms are).
%   EXPIRES: with the [1/1] two-wall line
function check_ws_surface_identity()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    rd = fullfile(root, 'test_results');
    sets = {'temp_11_two_walls', 'temp_11_fixed_height', 'temp_11_speed_scan', 'temp_22_two_walls'};
    fprintf('%-24s %-22s | %10s | %10s %10s\n', 'set', 'tag', 'max|da_hat|', 'max|dc0+1|', 'max|dd1+1|');
    for s = 1:numel(sets)
        files = dir(fullfile(rd, [sets{s} '_ws1'], '*.mat'));
        for k = 1:numel(files)
            fn_new = fullfile(rd, sets{s}, files(k).name);
            if ~exist(fn_new, 'file'); fprintf('%-24s %-22s | missing new\n', sets{s}, files(k).name); continue; end
            o = load(fullfile(rd, [sets{s} '_ws1'], files(k).name), 'a_hat', 'c0_hat', 'd1_hat');
            n = load(fn_new, 'a_hat', 'c0_hat', 'd1_hat');
            if ~isequal(size(o.a_hat), size(n.a_hat)); fprintf('%-24s %-22s | size differs\n', sets{s}, files(k).name); continue; end
            da = max(abs(n.a_hat(:) - o.a_hat(:)));
            if s < 4
                dc = max(abs(n.c0_hat(:) + 1 - o.c0_hat(:)));  dd = max(abs(n.d1_hat(:) + 1 - o.d1_hat(:)));
            else
                dc = NaN; dd = NaN;                                  % [2/2]: basis change is not a shift of (c0, d1)
            end
            fprintf('%-24s %-22s | %10.2e | %10.2e %10.2e\n', sets{s}, strrep(files(k).name, '.mat', ''), da, dc, dd);
        end
    end
end
