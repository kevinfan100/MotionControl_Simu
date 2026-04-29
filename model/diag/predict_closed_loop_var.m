function pred = predict_closed_loop_var(lambda_c, a_true, g, sigma2_n, kBT)
%PREDICT_CLOSED_LOOP_VAR  Theoretical predictions for eq17_7state closed-loop variance
%
%   pred = predict_closed_loop_var(lambda_c, a_true, g, sigma2_n, kBT)
%
%   Computes Lyapunov-based predictions for the closed-loop variance of dx
%   under paper 2023 Eq.17 control law with possibly mismatched a_hat.
%   Used for systematic bias diagnosis (compares simulation observations
%   against theoretical expectations at various g = a_true/a_hat).
%
% Inputs:
%   lambda_c  - closed-loop pole, scalar in (0,1)
%   a_true    - true motion gain [um/pN]
%   g         - gain mismatch ratio = a_true/a_hat [scalar; g=1 means a_hat=a_true]
%   sigma2_n  - per-axis sensor noise variance [um^2]
%   kBT       - thermal energy [pN*um]
%
% Outputs (struct pred):
%   .var_dx_thermal_lyap  - Lyapunov sigma^2_dx (thermal contribution only)
%   .var_dx_sensor_lyap   - Lyapunov sigma^2_dx (sensor contribution only)
%   .var_dx_total_lyap    - sum of above
%   .var_dxr_lyap         - var_dx_total + sigma2_n (sensor adds to measurement)
%   .a_xm_pred            - predicted a_xm via design.md formula:
%                              (var_dxr - C_n*sigma2_n) / (C_dpmr * 4*kBT)
%   .C_dpmr_design        - 2 + 1/(1-lambda^2)
%   .C_n_design           - 2/(1+lambda)
%   .C_dpmr_lyap          - var_dx_thermal_lyap / sigma2_dxT (where sigma2_dxT=4kBT*a_true)
%                           - should equal C_dpmr_design at g=1
%   .C_n_lyap             - var_dxr_sensor_only / sigma2_n at g=1 - should equal C_n_design
%   .poles                - 3x1 closed-loop pole magnitudes
%   .stable               - logical, all poles |z|<1
%   .effective_feedback   - g*(1-lambda_c), the actual feedback coefficient
%
% Closed-loop dynamics (Eq.18 form, regulation, x_d=const, x_D_true=0):
%   dx[k+1] = dx[k] - g*(1-lambda_c)*dx[k-2] - g*(1-lambda_c)*n_x[k] - a_true*f_T[k]
%
%   State X = [dx[k]; dx[k-1]; dx[k-2]]
%   A = [1, 0, -g*(1-lambda_c); 1, 0, 0; 0, 1, 0]
%
%   For thermal (white f_T variance sigma^2_fT, but we use sigma^2_dxT = a_true^2*sigma^2_fT directly):
%     B_T = [-a_true; 0; 0]; driver var sigma^2_fT
%     OR equivalently: B_T = [-1; 0; 0]; driver var sigma^2_dxT = 4kBT*a_true
%
%   For sensor (white n_x variance sigma^2_n):
%     B_n = [-g*(1-lambda_c); 0; 0]; driver var sigma^2_n

% Closed-loop A matrix (3x3 companion form, state = [dx[k]; dx[k-1]; dx[k-2]])
A = [1, 0, -g*(1-lambda_c); ...
     1, 0, 0; ...
     0, 1, 0];

% Stability check via spectral radius
poles = abs(eig(A));
pred.poles = poles;
pred.stable = all(poles < 1);
pred.effective_feedback = g*(1-lambda_c);

% Design.md theoretical coefficients (always defined for reference)
pred.C_dpmr_design = 2 + 1/(1-lambda_c^2);
pred.C_n_design    = 2/(1+lambda_c);

if ~pred.stable
    % Unstable closed loop -> infinite stationary variance
    pred.var_dx_thermal_lyap = inf;
    pred.var_dx_sensor_lyap  = inf;
    pred.var_dx_total_lyap   = inf;
    pred.var_dxr_lyap        = inf;
    pred.a_xm_pred           = inf;
    pred.C_dpmr_lyap         = inf;
    pred.C_n_lyap            = inf;
    return;
end

% Thermal driver: B_T*B_T' * sigma^2_dxT (with B_T = [-1;0;0] and sigma^2_dxT = 4kBT*a_true)
sigma2_dxT = 4*kBT*a_true;
B_T = [-1; 0; 0];
Sigma_thermal = dlyap(A, B_T*sigma2_dxT*B_T');
pred.var_dx_thermal_lyap = Sigma_thermal(1,1);

% Sensor driver: B_n*B_n' * sigma^2_n with B_n = [-g*(1-lambda_c);0;0]
B_n = [-g*(1-lambda_c); 0; 0];
Sigma_sensor = dlyap(A, B_n*sigma2_n*B_n');
pred.var_dx_sensor_lyap = Sigma_sensor(1,1);

% Total state variance and measured variance (sensor noise added at output)
pred.var_dx_total_lyap = pred.var_dx_thermal_lyap + pred.var_dx_sensor_lyap;
pred.var_dxr_lyap      = pred.var_dx_total_lyap + sigma2_n;

% a_xm prediction using design.md formula on Lyapunov sigma^2_dxr
pred.a_xm_pred = (pred.var_dxr_lyap - pred.C_n_design*sigma2_n) / (pred.C_dpmr_design * 4*kBT);

% Lyapunov-implied C_dpmr, C_n (sanity check, should match design.md at g=1)
pred.C_dpmr_lyap = pred.var_dx_thermal_lyap / sigma2_dxT;

% Recompute C_n at g=1 with unit sensor variance to isolate the geometric coefficient
A_g1   = [1, 0, -(1-lambda_c); 1, 0, 0; 0, 1, 0];
B_n_g1 = [-(1-lambda_c); 0; 0];
S_n_g1 = dlyap(A_g1, B_n_g1*1*B_n_g1');
pred.C_n_lyap = S_n_g1(1,1) + 1;  % var_dx_sensor + 1*sigma^2_n (per sigma^2_n=1)
end
