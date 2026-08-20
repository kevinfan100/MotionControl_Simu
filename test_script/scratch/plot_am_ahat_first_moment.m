% STATUS: ACTIVE (scratch) | PURPOSE: first-moment check the variance work is
%   blind to -- E[a_m] and E[a_hat] against a_true, on the sibling session's
%   400-seed deep-band stack | EXPIRES: when the near-wall bias item closes
%
% Every R22 / a_m check in verify_formC_am_r22.m is a SECOND-moment check and
% cannot see a scale bias. This is the first moment, and it says the readout
% is unbiased in the far field and over-reports near the wall -- which is the
% arrow the sibling's rung-1 departure was missing.
%
% Percentages near the wall have a small denominator (a/a_o = 0.087 against
% 0.95 in the far field), so read the two ends as different questions.
% How much of a_hat's near-wall bias is the readout already carrying?
% Two first moments on the same axis: the readout a_m, and the posterior a_hat
% the filter produces from it. Deep band, 400 seeds, z. Read-only.
S = load('/Users/kevin/Code/MotionControl_Simu-motion-test/test_results/am_r22_deep/stack_deep400.mat');
A = S.A_wm; H = S.A_ht; T = S.A_tr; t = S.t;
am = mean(T, 2);
ed = linspace(min(am), max(am), 15);
c = []; bm = []; bh = [];
for b = 1:14
    in = am >= ed(b) & am < ed(b+1);
    if ~any(in); continue; end
    c(end+1)  = mean(am(in));                                  %#ok<AGROW>
    bm(end+1) = 100*(mean(mean(A(in,:),2))/mean(am(in)) - 1);  %#ok<AGROW>
    bh(end+1) = 100*(mean(mean(H(in,:),2))/mean(am(in)) - 1);  %#ok<AGROW>
end
fprintf('\n  a/a_o    readout a_m    posterior a_hat    a_hat - a_m\n');
for i = 1:numel(c)
    fprintf('  %6.4f    %+7.2f %%      %+7.2f %%        %+7.2f pp\n', c(i), bm(i), bh(i), bh(i)-bm(i));
end
w = {[0.05 0.45],'initial hold (far)'; [4.0 4.75],'final hold (trough)'};
fprintf('\n  stationary windows:\n');
for i = 1:2
    in = t >= w{i,1}(1) & t <= w{i,1}(2);
    r_m = 100*(mean(mean(A(in,:),2))/mean(am(in)) - 1);
    r_h = 100*(mean(mean(H(in,:),2))/mean(am(in)) - 1);
    fprintf('  %-22s a/a_o %.4f  a_m %+6.2f %%  a_hat %+6.2f %%  (a_hat carries %.0f %% of what a_m already has)\n', ...
            w{i,2}, mean(am(in)), r_m, r_h, 100*r_m/r_h);
end
% figure: the two first moments, no smoothing, no window
f = figure('Position',[80 80 1000 620],'Color','w','Visible','off');
hold on;
yline(0,'-','Color',[0.4 0.4 0.4],'LineWidth',1.0,'HandleVisibility','off');
h1 = plot(c, bm, 'o-', 'Color',[0.45 0.72 0.95], 'MarkerFaceColor',[0.45 0.72 0.95], ...
          'LineWidth',2.0,'MarkerSize',6,'DisplayName','a_m readout');
h2 = plot(c, bh, 's-', 'Color',[0 0.2 0.9], 'MarkerFaceColor',[0 0.2 0.9], ...
          'LineWidth',2.0,'MarkerSize',6,'DisplayName','\^a posterior');
set(gca,'XScale','log'); xlim([min(c)*0.9 max(c)*1.05]);
set(gca,'XTick',[0.1 0.15 0.2 0.3 0.4 0.6 0.9]);
gca_h = gca; gca_h.XAxis.Exponent = 0; xtickformat('%.2f');
xlabel('a_z / a_o','FontSize',18,'FontWeight','bold');
ylabel('bias vs a_{true}  (%)','FontSize',18,'FontWeight','bold');
legend([h1 h2],'Location','northoutside','Orientation','horizontal','FontSize',14,'FontWeight','bold','Box','on');
set(gca,'FontSize',18,'FontWeight','bold','LineWidth',1.2,'Box','on'); grid off;
exportgraphics(f, '/Users/kevin/Code/MotionControl_Simu-motion-test/test_results/am_r22_deep/fig8_first_moment_bias.png','Resolution',150);
close(f);
fprintf('\nfigure -> test_results/am_r22_deep/fig8_first_moment_bias.png\n');
