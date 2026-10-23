% STATUS: ACTIVE (scratch) | PURPOSE: k^2 discriminator for the exact-step noise
%   rectification residual (problem point 2b), canon, a'_true arm, 30 seeds.
%   Noise AMPLITUDE k: T_scale = k^2 (plant + filter kBT together), meas *= k.
%   PRE-REGISTERED (2026-09-01, before running):
%     mechanism "nonlinear map rectifies noise" => hold bias proportional to k^2:
%       exact k=0.5  ->  ~ -0.00055      (= -0.00221/4)
%       exact k=1    ->    -0.00221      (already measured, arms30 canon exact)
%       exact k=2    ->  ~ -0.0088       (= -0.00221*4)
%     criterion: log-log slope of |hold bias| vs k = 2 +- 0.4.
%     euler k=2 control: deterministic part must stay +0.0058; its noise part
%     stays MUCH smaller than the exact arm's shift (report as measured).
%   FAIL => the mechanism label is wrong; residual 2b goes back to open.
function out = run_aptrue_exact_noise_scale()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    pc = physical_constants(); c0 = canonical_scenario(0.05, 1.1, 'deep');
    w0bar = c0.h_init/pc.R; [~,cp] = calc_correction_functions(w0bar);
    ws0 = 1 + w0bar - 1/((8/9)*(1 - 1/cp));
    MEAS0 = [0.00062; 0.00057; 0.00331];
    ARMS = {  % {tag, k, exact}
      {'exact_k05', 0.5, true}, {'exact_k20', 2.0, true}, {'euler_k20', 2.0, false}};
    out = struct('ws0', ws0);
    for i = 1:numel(ARMS)
        tag = ARMS{i}{1}; k = ARMS{i}{2}; ex = ARMS{i}{3};
        OV = struct('T_scale', k^2, 'meas_noise_std', MEAS0*k);
        o = struct('arm','best','ap_known',true,'ap_known_at','cmd', ...
                   'ctrl_const_override',struct('lock_b',true,'ws0_perp',ws0,'law_exact_step',ex), ...
                   'config_override',OV,'scenario','deep','verbose',false,'seeds',1:30);
        clear run_formC_b motion_control_law_formC_b;
        evalc('R = run_formC_b(o);');
        t = R.runs{1}.tout(:); nS = 30; E = zeros(numel(t), nS);
        for q = 1:nS
            r = R.runs{q}; ad = r.a_hat_out(1,3)/r.a_bar_hat_out(1,3);
            E(:,q) = r.a_bar_hat_out(:,3) - r.a_true_out(:,3)/ad;
        end
        m = t > 3.5;   % canon hold
        pm = mean(E(m,:),1);
        fprintf('[%s] hold bias %+.5f (SEM %.5f) | E(end) %+.5f | min a_bar_hat %.4f | NaN %d\n', ...
            tag, mean(pm), std(pm)/sqrt(nS), mean(E(end,:)), ...
            min(cellfun(@(r) min(r.a_bar_hat_out(:,3)), R.runs)), sum(~isfinite(E(:))));
        out.(tag) = struct('t', t, 'E', E, 'k', k, 'exact', ex);
        clear R;
    end
    save(fullfile(od,'aptrue_exact_noise_scale.mat'), '-struct', 'out', '-v7.3');
    fprintf('saved aptrue_exact_noise_scale.mat\n');
end
