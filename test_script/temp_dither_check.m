% TEMP sanity check on dither injection (chat 2026-07-21). Delete after use.
here = fileparts(mfilename('fullpath'));
S = load(fullfile(here, 'temp_dither_pilot.mat'));
arms = S.arms;  cfg = S.cfg;  lc = cfg.lambda_c;
t = arms(1).t;  Ts = t(2) - t(1);
fh = t < 0.5;  Fh = t >= 5.5;

fprintf('Ts = %.6e s,  N_firsthold = %d,  N_finalhold = %d\n', Ts, sum(fh), sum(Fh));
for ia = 1:numel(arms)
    a = arms(ia);
    % dither reaching commanded trajectory?
    dz_fh   = a.dz(fh);          % injected d1 [um]
    hdz_fh  = a.hdz(fh);         % desired z height (incl dither) [um]
    % dither component of dh_d (desired increment) in first hold
    dh_d = [a.hdz(2:end)-a.hdz(1:end-1); a.hdz(end)-a.hdz(end-1)];
    dh_d_fh = dh_d(fh);
    dh3_fh  = a.dh3(fh);
    phi_fh  = a.phi(fh);
    fprintf('\n[%s] f=%d A=%gnm\n', a.label, a.f, a.A*1e3);
    fprintf('  std(dz)_fh          = %.4e um  (expect A/sqrt2 = %.4e for hold)\n', std(dz_fh), a.A/sqrt(2));
    fprintf('  peak|dz|_fh         = %.4e um\n', max(abs(dz_fh)));
    fprintf('  std(hdz)_fh         = %.4e um  (desired z height ripple)\n', std(hdz_fh));
    fprintf('  std(dh_d)_fh        = %.4e um  (desired increment; dither ~ A*wd*Ts)\n', std(dh_d_fh));
    fprintf('    predicted A*wd*Ts = %.4e um\n', a.A*2*pi*a.f*Ts);
    fprintf('  std((1-lc)*dh3)_fh  = %.4e um  (thermal regressor term)\n', (1-lc)*std(dh3_fh));
    fprintf('  std(phi)_fh         = %.4e um\n', std(phi_fh));
    fprintf('  mean(Sy2)_fh        = %.4e\n', mean(a.Sy2(fh & a.Sy2>0)));
    fprintf('  frac steps gated_fh = %.3f\n', mean(a.gate3(fh)));
end
