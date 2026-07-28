% verify_blocal_exp.m
% ------------------------------------------------------------------------
% DISCRETE form throughout.  Two questions:
%  (1) Is the local decay rate b constant along hbar for the REAL c_perp?
%      b(k) = [c(k+1)-c(k)] / [ -Dhbar_d(k) * (c(k)-c_o) ]    (user's eq.1)
%  (2) Does the COUPLED difference equation reconstruct the curve?
%      c(k+1) = c(k) - Dhbar_d(k)*b(k)*(c(k)-c_o)             (drag recursion)
%      b(k+1) = b(k) / (1 + Dhbar_d(k)*b(k)/p)                (power-law-consistent b)
%      p(k+1) = p(k)   (the TRUE constant, ~1  Brenner lubrication)
% c_o = 1 (far field, hbar->inf).  hbar in [1.1,10], 100000 pts.
% ------------------------------------------------------------------------
here = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(here, '..', '..', 'model')));

N    = 100000;
hbar = linspace(1.1, 10, N).';
c_o  = 1.0;                        % c_perp far-field
dh   = hbar(2) - hbar(1);          % Dhbar_d (uniform grid spacing here)

% --- real wall curve ----------------------------------------------------
cperp = zeros(N,1);
try
    [~, cperp] = calc_correction_functions(hbar, true);
    if ~isequal(size(cperp), size(hbar)); error('not vectorized'); end
catch
    for i = 1:N; [~, cp] = calc_correction_functions(hbar(i), true); cperp(i) = cp; end
end

% --- DISCRETE first derivative dc_perp/dhbar ----------------------------
dcdh_disc          = zeros(N,1);
dcdh_disc(1:end-1) = ( cperp(2:end) - cperp(1:end-1) ) / dh;
dcdh_disc(end)     = dcdh_disc(end-1);

% --- (1) DISCRETE local b of the real curve (user's eq.1) ---------------
b_disc            = zeros(N,1);
b_disc(1:end-1)   = ( cperp(2:end) - cperp(1:end-1) ) ./ ( -dh * (cperp(1:end-1) - c_o) );
b_disc(end)       = b_disc(end-1);

% p = the constant power:  b ~ p/(hbar-1)  ->  p = (hbar-1)*b
w      = hbar >= 2;
p_fit  = median( (hbar(w)-1) .* b_disc(w) );          % ~1.00
b_pow  = p_fit ./ (hbar - 1);                          % power-law b (overlay)

% --- (2) reconstruct BOTH curves from the coupled difference equation ---
c_rec = zeros(N,1);  b_rec = zeros(N,1);
c_rec(1) = cperp(1);  b_rec(1) = b_disc(1);
for k = 1:N-1
    c_rec(k+1) = c_rec(k) - dh * b_rec(k) * (c_rec(k) - c_o);
    b_rec(k+1) = b_rec(k) / (1 + dh * b_rec(k) / p_fit);
end
err_c = max(abs(c_rec - cperp)) / max(cperp);
err_b = max(abs(b_rec(w) - b_disc(w))) / max(b_disc(w));

% --- console ------------------------------------------------------------
fprintf('\n Dhbar_d (grid) = %.3e     p_fit = %.4f     c_o = %.3f  [c_perp(10)=%.4f->1]\n', ...
        dh, p_fit, c_o, cperp(end));
fprintf(' b_disc over [2,10]: min %.4f  median %.4f  max %.4f  (CoV %.3f, 0=const)\n', ...
        min(b_disc(w)), median(b_disc(w)), max(b_disc(w)), std(b_disc(w))/mean(b_disc(w)));
fprintf(' recursion rebuild error:  c %.2e   b %.2e   (both ~0 => difference eq correct)\n', err_c, err_b);
fprintf('   hbar  |  c_perp  | dc/dhbar | b_disc | (hbar-1)*b\n');
for hh = [1.1 1.5 2 3 5 10]
    [~, ii] = min(abs(hbar - hh));
    fprintf('  %6.2f | %8.3f | %8.3f | %6.4f | %7.4f\n', ...
            hbar(ii), cperp(ii), dcdh_disc(ii), b_disc(ii), (hbar(ii)-1)*b_disc(ii));
end

% --- figure -------------------------------------------------------------
%   top   : ORIGINAL real wall curve c_perp(hbar)  (what we model)
%   bottom: b(k) BACK-SOLVED from the real curve, exactly the user's formula
%           b(k) = [c(k+1)-c(k)] / ( -Dhbar_d(k)*(c(k)-c_o) )
FS = 18;  hb = char([104 772]);                        % "h-bar"
fig = figure('Color','w','Position',[80 80 940 780]);

ax1 = subplot(2,1,1);
plot(ax1, hbar, cperp, 'Color',[0.85 0 0], 'LineWidth',2.4);
ylabel(ax1, ['c_\perp(' hb ')'], 'FontSize',FS,'FontWeight','bold');
legend(ax1, {['c_\perp(' hb ')   (real wall curve)']}, ...
       'Location','northeast','FontSize',FS-4,'Box','off');
set(ax1,'FontSize',FS-2,'FontWeight','bold','LineWidth',1.3,'Box','on'); grid(ax1,'off'); xlim(ax1,[1.1 10]);

ax2 = subplot(2,1,2); hold(ax2,'on');
plot(ax2, hbar, b_disc, 'Color',[0 0.45 0.74], 'LineWidth',2.4);     % real b (back-solved from c)
plot(ax2, hbar, b_rec,  '--', 'Color',[0 0 0], 'LineWidth',1.8);     % b difference equation
ylabel(ax2, ['b(' hb ')'], 'FontSize',FS,'FontWeight','bold');
xlabel(ax2, [hb ' = h/R'], 'FontSize',FS,'FontWeight','bold');
legend(ax2, {['real  b(k)=[c_\perp(k+1)-c_\perp(k)]/(-\Delta' hb '_d[c_\perp(k)-c_o])'], ...
             ['model  b(k+1)=b(k)/(1+\Delta' hb '_d b/p)  =  p/(' hb '-1),  p=1']}, ...
       'Location','northeast','FontSize',FS-7,'Box','off');
set(ax2,'FontSize',FS-2,'FontWeight','bold','LineWidth',1.3,'Box','on'); grid(ax2,'off'); xlim(ax2,[1.1 10]);

fig_dir  = fullfile(here, '..', '..', 'reference','eq17_analysis','derivation','figures');
if ~exist(fig_dir,'dir'); mkdir(fig_dir); end
fig_path = fullfile(fig_dir, 'blocal_exp_check.png');
exportgraphics(fig, fig_path, 'Resolution', 150);
close(fig);
fprintf(' FIGURE saved: %s\n', fig_path);

% --- figure 2: first derivative dc_perp/dhbar (discrete) ----------------
fig2 = figure('Color','w','Position',[80 80 940 460]);
ax = axes(fig2);
plot(ax, hbar, dcdh_disc, 'Color',[0.10 0.50 0.10], 'LineWidth',2.4);
ylabel(ax, ['dc_\perp / d' hb], 'FontSize',FS,'FontWeight','bold');
xlabel(ax, [hb ' = h/R'], 'FontSize',FS,'FontWeight','bold');
legend(ax, {['dc_\perp/d' hb ' = [c_\perp(k+1)-c_\perp(k)] / \Delta' hb '_d   (discrete)']}, ...
       'Location','southeast','FontSize',FS-5,'Box','off');
set(ax,'FontSize',FS-2,'FontWeight','bold','LineWidth',1.3,'Box','on'); grid(ax,'off'); xlim(ax,[1.1 10]);
fig2_path = fullfile(fig_dir, 'cprime_check.png');
exportgraphics(fig2, fig2_path, 'Resolution', 150);
close(fig2);
fprintf(' FIGURE saved: %s\n', fig2_path);
