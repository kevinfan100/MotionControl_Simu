function out = sweep_plant_thermal(seeds, mode)
%SWEEP_PLANT_THERMAL  Is the remaining bias proportional to the plant's
%   thermal power?  (b_true oracle + exact law step, canonical, 8 seeds.)
%   mode 'both' (default): plant AND estimator temperature scaled together
%   (opts.T_scale: kappa_T, Q33, xi_bar follow) -- the world is colder and the
%   filter knows it.  mode 'plant': plant only -- CONFOUNDED, kept as a record:
%   y2 reads the gain through kappa_T, so halving the plant's power makes y2
%   report half the gain (08-30: bias -34 % at x1/2, -40 % at x1/4).
%   Measurement noise stays on.
%
%   Registered prediction: a zero-mean disturbance can only bias a_hat through
%   an even-order term, so the noise-added excess over the oscillation
%   (excess(T) - excess(0)) is linear in the thermal POWER: at x1/2 and x1/4
%   it should be 1/2 and 1/4 of the x1 value (within the 8-seed SEM).  A
%   ratio well above that (e.g. proportional to the amplitude, sqrt) would
%   point at a clamp/gate/threshold rather than a smooth second-order term.
%   Known zero-noise point (det exact, 08-30): oscillation excess +0.0030,
%   end-hold -1.2 %.
% STATUS: ACTIVE | 2026-08-30

    if nargin < 1 || isempty(seeds); seeds = 1:8; end
    if nargin < 2 || isempty(mode); mode = 'both'; end   % 'both' = plant + estimator (T_scale); 'plant' = plant only (CONFOUNDED: y2 reads gain through kappa_T)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'btrue_log_ledger');
    ax = 3;  R = physical_constants().R;  lam = 0.7;
    SC = [1 0.5 0.25];  ns = numel(seeds);
    out = struct('scale', SC, 'bias', zeros(1,3), 'bias_sem', zeros(1,3), 'exc', zeros(1,3), 'exc_sem', zeros(1,3), ...
                 'y1', zeros(1,3), 'law', zeros(1,3), 'y2', zeros(1,3), 'res', zeros(1,3), 'nubar', zeros(1,3), 'sd_e3', zeros(1,3));
    if strcmp(mode, 'both'); kn = 'T_scale'; else; kn = 'plant_T_scale'; end
    fprintf('\n[mode %s]\n', mode);
    fprintf('\n%-8s %9s %7s | %9s %7s | %8s %8s %8s %8s | %9s %9s\n', 'T scale', 'bias %', 'SEM', 'osc exc', 'SEM', 'law', 'y1', 'y2', 'resid', 'mean nu', 'sd(x3)');
    for i = 1:3
        O = run_formC_b(struct('arm', 'best', 'ap_src', 'post', 'seeds', seeds, 'verbose', false, ...
                               'b_true', true, 'b_true_at', 'cmd', kn, SC(i), ...
                               'ctrl_const_override', struct('law_exact_step', true)));
        a_nom = O.runs{1}.a_nom;  t = O.runs{1}.tout(:);
        G = @(f) cell2mat(reshape(cellfun(@(r) r.(f)(:, ax), O.runs, 'UniformOutput', false), 1, []));
        ah = G('a_bar_hat_out');  at = G('a_true_out')/a_nom;  bh = G('b_hat_out');  dx3 = G('delta_x_hat_3_out')/R;
        K1 = G('K_a_y1_out');  n1 = G('innov_y1_out');  K2 = G('K_a_y2_out');  n2 = G('innov_y2_out');
        kk = 2:numel(t);  tt = t(kk);  dwd = [0; diff(O.runs{1}.h_bar_d_out(:))];
        dw = dwd(kk) + (1 - lam) * dx3(kk-1, :);
        law = bh(kk-1,:) .* (1 - ah(kk-1,:)).^2 .* dw;  y1 = K1(kk,:).*n1(kk,:);  y2 = K2(kk,:).*n2(kk,:);
        dah = ah(kk,:) - ah(kk-1,:);  dat = at(kk,:) - at(kk-1,:);  res = dah - (law + y1 + y2);
        m = tt >= 1.5 & tt < 3.5;  ho = tt > 3.7;
        eb = mean(ah(kk(ho),:),1)./mean(at(kk(ho),:),1) - 1;
        ex = sum(dah(m,:),1) - sum(dat(m,:),1);
        out.bias(i) = mean(eb);  out.bias_sem(i) = std(eb)/sqrt(ns);  out.exc(i) = mean(ex);  out.exc_sem(i) = std(ex)/sqrt(ns);
        out.law(i) = mean(sum(law(m,:),1));  out.y1(i) = mean(sum(y1(m,:),1));  out.y2(i) = mean(sum(y2(m,:),1));  out.res(i) = mean(sum(res(m,:),1));
        out.nubar(i) = mean(n1(kk(m),:),'all');  out.sd_e3(i) = mean(std(dx3(kk(m),:),0,2));
        fprintf('%-8.2f %+9.2f %7.2f | %+9.4f %7.4f | %+8.4f %+8.4f %+8.4f %+8.4f | %+9.2e %9.2e\n', SC(i), 100*out.bias(i), 100*out.bias_sem(i), ...
                out.exc(i), out.exc_sem(i), out.law(i), out.y1(i), out.y2(i), out.res(i), out.nubar(i), out.sd_e3(i));
    end
    EXC0 = 0.0030;  BIAS0 = -0.0121;   % zero-noise point (det exact)
    fprintf('\nnoise-added oscillation excess (minus zero-noise +0.0030):  x1 %+.4f   x1/2 %+.4f (ratio %.2f, power law predicts 0.50)   x1/4 %+.4f (ratio %.2f, predicts 0.25)\n', ...
            out.exc(1)-EXC0, out.exc(2)-EXC0, (out.exc(2)-EXC0)/(out.exc(1)-EXC0), out.exc(3)-EXC0, (out.exc(3)-EXC0)/(out.exc(1)-EXC0));
    fprintf('noise-added end-hold bias (minus zero-noise -1.21 %%):       x1 %+.2f   x1/2 %+.2f (ratio %.2f)   x1/4 %+.2f (ratio %.2f)  [pp]\n', ...
            100*(out.bias(1)-BIAS0), 100*(out.bias(2)-BIAS0), (out.bias(2)-BIAS0)/(out.bias(1)-BIAS0), 100*(out.bias(3)-BIAS0), (out.bias(3)-BIAS0)/(out.bias(1)-BIAS0));
    save(fullfile(od, ['sweep_plant_thermal_' mode '.mat']), 'out');
    fprintf('SWEEP DONE\n');
end
