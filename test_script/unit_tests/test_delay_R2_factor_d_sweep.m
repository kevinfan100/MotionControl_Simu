function test_delay_R2_factor_d_sweep()
%TEST_DELAY_R2_FACTOR_D_SWEEP Verify delay_R2_factor for d=0..4
%
%   Priority 1 from deep audit §6: closes coverage gap that allowed
%   buffer off-by-one bug (effective d=1 vs spec d=2) to slip past
%   build_eq17_constants tests (which only checked d=2 and d=3).
%
%   Closed form (build_eq17_constants line 191-198):
%       factor(d) = sum_{j=1..d} (d - j + 1)^2 = 1^2 + 2^2 + ... + d^2
%
%   Reference values:
%       d=0: 0
%       d=1: 1                  (1^2)
%       d=2: 5                  (1^2 + 2^2 = 1 + 4)
%       d=3: 14                 (1^2 + 2^2 + 3^2 = 1 + 4 + 9)
%       d=4: 30                 (1^2 + 2^2 + 3^2 + 4^2 = 1 + 4 + 9 + 16)

    this_dir = fileparts(mfilename('fullpath'));
    repo_root = fullfile(this_dir, '..', '..');
    addpath(fullfile(repo_root, 'model', 'controller'));

    fprintf('=== test_delay_R2_factor_d_sweep ===\n');

    base_opts.lambda_c   = 0.7;
    base_opts.option     = 'A_MA2_full';
    base_opts.sigma2_n_s = [1; 1; 1];
    base_opts.kBT        = 4.114e-9;

    expected = struct( ...
        'd0', 0,  ...
        'd1', 1,  ...
        'd2', 5,  ...
        'd3', 14, ...
        'd4', 30);

    fields = {'d0', 'd1', 'd2', 'd3', 'd4'};
    d_values = [0, 1, 2, 3, 4];

    n_pass = 0;
    for k = 1:numel(fields)
        d_val = d_values(k);
        f = fields{k};
        opts = base_opts;
        opts.d = d_val;
        cc = build_eq17_constants(opts);
        got = cc.delay_R2_factor;
        exp = expected.(f);
        assert(got == exp, ...
            'FAIL: d=%d got delay_R2_factor=%g, expected %g', ...
            d_val, got, exp);

        % Closed-form double-check (independent of build_eq17_constants impl)
        if d_val == 0
            cf = 0;
        else
            cf = sum((d_val - (1:d_val) + 1).^2);
        end
        assert(got == cf, ...
            'FAIL closed-form check: d=%d got %g vs cf %g', d_val, got, cf);

        fprintf('[PASS] d=%d  delay_R2_factor = %g (expected %g)\n', ...
            d_val, got, exp);
        n_pass = n_pass + 1;
    end

    fprintf('=== ALL %d tests PASS ===\n', n_pass);
end
