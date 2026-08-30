% STATUS: ACTIVE (scratch instrument) | PURPOSE: rebuild the a_m readout of a
%   finished run OFFLINE with a different (a_pd, a_cov), so a readout arm can
%   be checked at N = 100 without re-running the loop. The readout is a pure
%   function of dh_m (rebuild_am_chain: bit-exact, 2e-16), and the loop does
%   not read a_pd/a_cov except through a_hat, so the tracking-error statistics
%   the formula is built on are the ones of the run.
%   Returns an O-like struct: runs{q}.a_xm_out replaced, ctrl_const carrying
%   the arm's constants (C_dpmr, C_n, IF_abc from a run that used that arm).
function O2 = rebuild_arm_offline(O, cc_arm, ax)
    if nargin < 3 || isempty(ax); ax = 3; end
    O2 = O; a_pd = cc_arm.a_pd; a_cov = cc_arm.a_cov;
    for q = 1:numel(O.runs)
        r = O.runs{q}; P = r.meta.params_value; fourkT = 4*P.ctrl.k_B*P.ctrl.T; s2n = P.ctrl.sigma2_noise(ax);
        dh = r.dh_m_out(:, ax); N = numel(dh);                          % [um]
        md = zeros(N,1); s2 = zeros(N,1); md(1) = dh(1);
        s2(1) = cc_arm.C_dpmr*fourkT*r.a_true_out(1,ax) + cc_arm.C_n*s2n;  % start on the formula; dies in 1/a_cov steps
        for k = 2:N
            md(k) = (1-a_pd)*md(k-1) + a_pd*dh(k);
            s2(k) = (1-a_cov)*s2(k-1) + a_cov*(dh(k)-md(k))^2;
        end
        am = (s2 - cc_arm.C_n*s2n) / (cc_arm.C_dpmr*fourkT);              % [um/pN]
        r.a_xm_out(:, ax) = am;
        r.ctrl_const.a_pd = a_pd; r.ctrl_const.a_cov = a_cov;
        r.ctrl_const.C_dpmr = cc_arm.C_dpmr; r.ctrl_const.C_n = cc_arm.C_n;
        r.ctrl_const.IF_abc = cc_arm.IF_abc; r.ctrl_const.K_var = cc_arm.K_var;
        O2.runs{q} = r;
    end
end
