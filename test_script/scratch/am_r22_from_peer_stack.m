% STATUS: ACTIVE (scratch) | PURPOSE: re-run the a_m / R(2,2) checks on the
%   400-seed DEEP-band ensemble produced by the sibling session
%   (test_results/formC_cdpmr_var_check/raw_seeds.mat) instead of simulating
%   again | EXPIRES: when the R22 audit closes
%
% The sibling's run is the same controller and arm (run_formC_b, arm 'best',
% ap_src 'post', a_cov 0.05, y2_on) on the CANONICAL DEEP band (trough
% w_bar = 1.10, h_bar_safe = 1.0). My own 200-seed set is on the SHALLOW band
% (trough 2.00) because run_formC_b still carried its inline scenario then, so
% ratios may be compared across the two but levels and percentages may not.
%
% PRE-REGISTERED (written before running, per rules/derivation-workflow #13):
%   P1  the sibling measures Var(dx_r) / [C_dpmr*kappa_T*a + C_n*sigma2_n]
%       rising to 1.052 at the trough and 1.107 in the innermost a-bin. Since
%       Var(a_m) goes as (sigma2_dwr)^2, MY ratio there must be the SQUARE of
%       theirs (~1.11 at the trough). Anything else is a different mechanism.
%   P2  R2_out is built at a_bar_HAT, and their final-hold a_hat runs 19%
%       high, so R2_used / Var(y2) should leave 3.3 near the wall and climb
%       toward 3.3 * 1.19^2 ~ 4.7, while staying at 3.3 in the far field.
function out = am_r22_from_peer_stack(opts)
    if nargin < 1; opts = struct(); end
    if ~isfield(opts, 'n_seed'); opts.n_seed = 400; end
    if ~isfield(opts, 'ax');     opts.ax     = 3;   end

    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    addpath(fullfile(root, 'test_script', 'integration'));
    addpath(fullfile(root, 'model', 'config'));
    src  = fullfile(root, 'test_results', 'formC_cdpmr_var_check', 'raw_seeds.mat');
    dst_dir = fullfile(root, 'test_results', 'am_r22_deep');
    if ~exist(dst_dir, 'dir'); mkdir(dst_dir); end
    stack_file = fullfile(dst_dir, sprintf('stack_deep%d.mat', opts.n_seed));

    if ~exist(stack_file, 'file')
        L = load(src);  S = L.S400;  clear L;
        K = S.K;  ax = opts.ax;  ns = opts.n_seed;
        a_nom   = K.a_nom;
        kappa_T = 4 * (K.kBT / K.R) * K.a_o;
        s2n_nd  = K.sigma2_n_s(:) / K.R^2;
        xi_bar  = (K.C_n / K.C_dpmr) * s2n_nd / kappa_T;
        % their xi_per_axis is the PHYSICAL xi; xi_bar = xi_phys / a_nom
        assert(max(abs(xi_bar - K.xi_per_axis(:) / a_nom)) < 1e-9, ...
               'am_r22_from_peer_stack:xi', 'xi_bar disagrees with their xi_per_axis/a_nom');

        pick = @(C) squeeze(C(:, ax, 1:ns));
        A_wm = pick(S.a_xm_out) / a_nom;
        A_tr = pick(S.a_true_out) / a_nom;
        R2_u = pick(S.R2_out);
        A_ht = pick(S.a_bar_hat_out);
        gate = logical(pick(S.gate_out));
        t    = S.t(:);

        % Row 1 is the controller's init-only call in their save too.
        assert(all(A_wm(1, :) == 0), 'am_r22_from_peer_stack:initRow', 'row 1 is not the init row');
        A_wm = A_wm(2:end, :); A_tr = A_tr(2:end, :); R2_u = R2_u(2:end, :);
        A_ht = A_ht(2:end, :); gate = gate(2:end, :); t = t(2:end);

        cc = struct('a_cov', K.a_cov, 'a_pd', K.a_pd, 'C_dpmr', K.C_dpmr, ...
                    'C_n', K.C_n, 'K_var', K.K_var, 'IF_abc', K.IF_abc(:), ...
                    'd', K.d, 'h_bar_safe', K.h_bar_safe, 'amlpf_var_factor', 1);
        cfg = canonical_scenario(K.a_cov, K.h_bottom / K.R, 'deep');
        seeds = S.seeds(1:ns);
        save(stack_file, 'A_wm', 'A_tr', 'R2_u', 'A_ht', 'gate', 't', 'cc', ...
             'kappa_T', 's2n_nd', 'xi_bar', 'a_nom', 'cfg', 'seeds', 'ax', '-v7.3');
        fprintf('converted -> %s\n', stack_file);
    end

    out = verify_formC_am_r22(struct('stack', stack_file, 'out_dir', dst_dir));
    out.stack_file = stack_file;
end
