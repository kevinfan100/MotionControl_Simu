% PURPOSE (2026-09-26, user: "T_d -> inf should become fixed points; does a lower bottom help?"): noise-free,
%   data-dominated limit of the Q = 0 Kalman filter = running least squares over the whole history (RLS equivalence).
%   No simulation. Truth a(h_d(t)) on the ramp template (hold 0.5 s, 22.2 -> bottom constant velocity over T_d, hold 0.5 s),
%   uniform weights (R22 is evaluated at a_hat, ~flat), y1 ignored. Three model families, each fitted to all samples 0..k
%   and evaluated at k:
%     first order   a[k+1] = a[k]                     -> a_hat = running mean
%     second order  a[k+1] = a[k] + delta a[k]        -> running straight-line fit in time
%     slope law     a[k+1] = a[k] + (1-c)/h_d^2 dh_d  -> running fit alpha + (1-c) g(t), g = 1/h0 - 1/h_d(t)
%   plus second order with a constant forgetting factor (finite memory) to show the T_d -> inf limit when memory is finite.
%   e_a = a_true - a_hat. Windows: near = descent samples with h_d < 3 R (mean), hold-end = last sample.
%   Instrument check (run before reading anything else): Layer-1 lambda_f = 1 sims, plane, 5 seeds:
%     first order  near -0.42 / -0.43 (T_d 5 / 15), hold-end -0.67 / -0.74
%     second order near -0.27 / -0.26 / -0.28 (T_d 2 / 5 / 15), hold-end -0.29 / -0.32 / -0.45
%   STATUS: SCRATCH
function probe_ls_limit_td_bottom()
    Ts = 1/1600;  h0 = 22.2;  TH = 0.5;
    plane = @(h) local_poly(1 ./ max(h, 1.0));
    cell  = @(h) local_poly(1 ./ (max(h, 1.0) + 1.05));
    fprintf('plane a(1.10) = %.4f (expect 0.0873), a(1.01) = %.4f\n', plane(1.10), plane(1.01));
    TD = [2 5 15 50];  BOT = [1.10 1.05 1.01];
    fprintf('\n%-8s %-6s %-5s | %-17s | %-17s | %-17s | %-17s | %-17s\n', 'plant', 'bottom', 'T_d', ...
        '1st: near / end', '2nd: near / end', '2nd lf.995', 'slope: near / end', 'c_hat end');
    for plant_name = {'plane', 'cell'}
        A = plane;  if strcmp(plant_name{1}, 'cell'); A = cell; end
        for hb = BOT
            for td = TD
                t = (0:Ts:(TH + td + TH)).';
                hd = h0 - (h0 - hb) * min(max((t - TH) / td, 0), 1);
                a = A(hd);  g = 1 / h0 - 1 ./ hd;
                e1 = a - cumsum(a) ./ (1:numel(a)).';
                e2 = a - local_run_fit(t, a, 1);
                e2f = a - local_run_fit(t, a, 0.995);
                [fs, bet] = local_run_fit_g(g, a);  es = a - fs;
                m_near = t > TH & t <= TH + td & hd < 3;
                fprintf('%-8s %-6.2f %-5g | %+.3f / %+.3f   | %+.3f / %+.3f   | %+.3f / %+.3f   | %+.3f / %+.3f   | %+.3f\n', ...
                    plant_name{1}, hb, td, mean(e1(m_near)), e1(end), mean(e2(m_near)), e2(end), ...
                    mean(e2f(m_near)), e2f(end), mean(es(m_near)), es(end), 1 - bet(end));
            end
        end
    end
end
function a = local_poly(u)
    a = 1 - (9/8) * u + (1/2) * u.^3 - (57/100) * u.^4 + (1/5) * u.^5 + (7/200) * u.^11 - (1/25) * u.^12;
end
function f = local_run_fit(t, a, lam)
    % running (exponentially weighted when lam < 1) straight-line fit a ~ th0 + th1 t, evaluated at the newest t
    n = numel(t);  f = zeros(n, 1);  S = zeros(5, 1);  w0 = 1e-9;
    for k = 1:n
        S = lam * S + [1; t(k); t(k)^2; a(k); a(k) * t(k)];
        M = [S(1) + w0, S(2); S(2), S(3) + w0];  th = M \ [S(4); S(5)];
        f(k) = th(1) + th(2) * t(k);
    end
end
function [f, bet] = local_run_fit_g(g, a)
    % running fit a ~ alpha + beta g, beta = 1 - c; tiny prior beta -> 1 - 0.053 (plane band fit) resolves the hold
    n = numel(g);  f = zeros(n, 1);  bet = zeros(n, 1);  S = zeros(5, 1);  w0 = 1e-6;  b0 = 1 - 0.053;
    for k = 1:n
        S = S + [1; g(k); g(k)^2; a(k); a(k) * g(k)];
        M = [S(1), S(2); S(2), S(3) + w0];  th = M \ [S(4); S(5) + w0 * b0];
        f(k) = th(1) + th(2) * g(k);  bet(k) = th(2);
    end
end
