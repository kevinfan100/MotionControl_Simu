function test_calc_correction_functions()
%TEST_CALC_CORRECTION_FUNCTIONS Unit tests for calc_correction_functions
%
%   Verifies:
%       (1) Backward compatibility (1-input, 2-output call still works)
%       (2) Default behavior does not produce derivs unless requested
%       (3) When 3 outputs are requested, derivs struct is returned with
%           all required fields
%       (4) Numerical values of derivs match pre-computed reference values
%           (relative tolerance 1e-3)
%       (5) Analytical dc_para_dh / dc_perp_dh match a central finite
%           difference (relative error < 1e-5)
%       (6) Error is thrown for h_bar < 1
%
%   Usage:
%       Run from MATLAB after addpath('../../model/wall_effect') or after
%       calc_simulation_params has set up paths.

    % Add wall_effect path so calc_correction_functions is on the search path
    this_dir = fileparts(mfilename('fullpath'));
    repo_root = fullfile(this_dir, '..', '..');
    addpath(fullfile(repo_root, 'model', 'wall_effect'));

    n_pass = 0;
    rel_tol_ref  = 1e-3;
    rel_tol_fd   = 1e-5;

    % ------------------------------------------------------------
    % Test 1: Backward compatibility (1 input -> 2 outputs)
    % ------------------------------------------------------------
    h_test = 5;
    [c_para, c_perp] = calc_correction_functions(h_test);
    assert(isscalar(c_para) && isnumeric(c_para), ...
        'Test 1 failed: c_para must be a numeric scalar');
    assert(isscalar(c_perp) && isnumeric(c_perp), ...
        'Test 1 failed: c_perp must be a numeric scalar');
    assert(c_para > 1 && c_perp > 1, ...
        'Test 1 failed: corrections should be > 1 for finite h_bar');
    fprintf('[PASS] Test 1: Backward compatibility (1 in, 2 out)\n');
    n_pass = n_pass + 1;

    % ------------------------------------------------------------
    % Test 2: Default behavior (no derivs unless requested)
    % ------------------------------------------------------------
    [a, b] = calc_correction_functions(5);  %#ok<ASGLU> Ensure no error
    fprintf('[PASS] Test 2: Default 2-out call no error\n');
    n_pass = n_pass + 1;

    % ------------------------------------------------------------
    % Test 3: 3-output call returns struct with all 6 fields
    % ------------------------------------------------------------
    [~, ~, derivs] = calc_correction_functions(5);
    assert(isstruct(derivs), 'Test 3 failed: derivs must be a struct');
    required_fields = {'dc_para_dh', 'dc_perp_dh', ...
                       'K_h_para',   'K_h_perp', ...
                       'K_h_prime_para', 'K_h_prime_perp'};
    for k = 1:numel(required_fields)
        f = required_fields{k};
        assert(isfield(derivs, f), ...
            'Test 3 failed: derivs is missing field "%s"', f);
        assert(isscalar(derivs.(f)) && isnumeric(derivs.(f)), ...
            'Test 3 failed: derivs.%s must be numeric scalar', f);
    end
    fprintf('[PASS] Test 3: 3-out struct contains all 6 required fields\n');
    n_pass = n_pass + 1;

    % ------------------------------------------------------------
    % Test 4: Reference value validation
    % ------------------------------------------------------------
    % Reference table (h_bar, c_para, c_perp,
    %                  dc_para_dh, dc_perp_dh,
    %                  K_h_para, K_h_perp,
    %                  K_h_prime_para, K_h_prime_perp)
    % NaN means "do not check this entry" (some entries not provided in spec)
    % Reference values computed via MATLAB analytical evaluation at 10-digit
    % precision; rel_tol_ref = 1e-3 below, so 4-5 significant digits suffice.
    ref = struct();
    ref(1).h_bar = 1.10;
    ref(1).c_para = 2.36026;            ref(1).c_perp = 11.45229;
    ref(1).dc_para_dh = -4.57771;       ref(1).dc_perp_dh = -101.3598;
    ref(1).K_h_para = -1.93949;         ref(1).K_h_perp = -8.85061;
    ref(1).K_h_prime_para = 10.51316;
    ref(1).K_h_prime_perp = 96.95386;

    ref(2).h_bar = 1.50;
    ref(2).c_para = 1.61529;            ref(2).c_perp = 3.20524;
    ref(2).dc_para_dh = -0.77219;       ref(2).dc_perp_dh = -4.27028;
    ref(2).K_h_para = -0.47805;         ref(2).K_h_perp = -1.33228;
    ref(2).K_h_prime_para = 1.12370;
    ref(2).K_h_prime_perp = 3.45400;

    ref(3).h_bar = 22.0;
    ref(3).c_para = 1.02623;            ref(3).c_perp = 1.05384;
    ref(3).dc_para_dh = -1.22241e-3;    ref(3).dc_perp_dh = -2.57479e-3;
    ref(3).K_h_para = -1.19117e-3;      ref(3).K_h_perp = -2.44324e-3;
    ref(3).K_h_prime_para = 1.09577e-4;
    ref(3).K_h_prime_perp = 2.27531e-4;

    for k = 1:numel(ref)
        h = ref(k).h_bar;
        [cp, cw, dv] = calc_correction_functions(h, true);

        check_close(cp,                ref(k).c_para,         rel_tol_ref, sprintf('Test 4 (h=%.2f) c_para',         h));
        check_close(cw,                ref(k).c_perp,         rel_tol_ref, sprintf('Test 4 (h=%.2f) c_perp',         h));
        check_close(dv.dc_para_dh,     ref(k).dc_para_dh,     rel_tol_ref, sprintf('Test 4 (h=%.2f) dc_para_dh',     h));
        check_close(dv.dc_perp_dh,     ref(k).dc_perp_dh,     rel_tol_ref, sprintf('Test 4 (h=%.2f) dc_perp_dh',     h));
        check_close(dv.K_h_para,       ref(k).K_h_para,       rel_tol_ref, sprintf('Test 4 (h=%.2f) K_h_para',       h));
        check_close(dv.K_h_perp,       ref(k).K_h_perp,       rel_tol_ref, sprintf('Test 4 (h=%.2f) K_h_perp',       h));
        check_close(dv.K_h_prime_para, ref(k).K_h_prime_para, rel_tol_ref, sprintf('Test 4 (h=%.2f) K_h_prime_para', h));
        check_close(dv.K_h_prime_perp, ref(k).K_h_prime_perp, rel_tol_ref, sprintf('Test 4 (h=%.2f) K_h_prime_perp', h));
    end
    fprintf('[PASS] Test 4: Reference values match (rel tol %.0e) at h_bar=[1.10, 1.50, 22.0]\n', rel_tol_ref);
    n_pass = n_pass + 1;

    % ------------------------------------------------------------
    % Test 5: Analytical vs central finite difference
    % ------------------------------------------------------------
    eps_h = 1e-7;
    h_grid = [1.1, 1.5, 5, 22];
    for h = h_grid
        [~, ~, dv] = calc_correction_functions(h, true);

        [cp_p, cw_p] = calc_correction_functions(h + eps_h);
        [cp_m, cw_m] = calc_correction_functions(h - eps_h);

        fd_para = (cp_p - cp_m) / (2 * eps_h);
        fd_perp = (cw_p - cw_m) / (2 * eps_h);

        rel_err_para = abs(dv.dc_para_dh - fd_para) / max(abs(fd_para), eps);
        rel_err_perp = abs(dv.dc_perp_dh - fd_perp) / max(abs(fd_perp), eps);

        assert(rel_err_para < rel_tol_fd, ...
            'Test 5 failed at h=%.2f: dc_para_dh rel err %.3e > %.0e (analytical=%.6e, FD=%.6e)', ...
            h, rel_err_para, rel_tol_fd, dv.dc_para_dh, fd_para);
        assert(rel_err_perp < rel_tol_fd, ...
            'Test 5 failed at h=%.2f: dc_perp_dh rel err %.3e > %.0e (analytical=%.6e, FD=%.6e)', ...
            h, rel_err_perp, rel_tol_fd, dv.dc_perp_dh, fd_perp);
    end
    fprintf('[PASS] Test 5: Analytical derivs match central FD (rel err < %.0e)\n', rel_tol_fd);
    n_pass = n_pass + 1;

    % ------------------------------------------------------------
    % Test 6: Error for h_bar < 1
    % ------------------------------------------------------------
    threw = false;
    try
        [~, ~] = calc_correction_functions(0.95);
    catch ME
        if strcmp(ME.identifier, 'calc_correction_functions:invalidInput')
            threw = true;
        else
            rethrow(ME);
        end
    end
    assert(threw, 'Test 6 failed: expected error for h_bar = 0.95');
    fprintf('[PASS] Test 6: Error correctly thrown for h_bar < 1\n');
    n_pass = n_pass + 1;

    fprintf('\n=== ALL %d tests PASS ===\n', n_pass);
end

function check_close(actual, expected, rel_tol, label)
%CHECK_CLOSE Assert |actual - expected| / |expected| < rel_tol
%   Skips check if `expected` is NaN (used to mark "not provided in spec").
    if isnan(expected)
        return;
    end
    denom = max(abs(expected), eps);
    rel_err = abs(actual - expected) / denom;
    assert(rel_err < rel_tol, ...
        '%s failed: actual=%.6g, expected=%.6g, rel_err=%.3e > %.0e', ...
        label, actual, expected, rel_err, rel_tol);
end
