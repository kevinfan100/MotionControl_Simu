% PURPOSE (2026-09-09, user: "what is the key decision point / which direction"): the law's FUNCTION FORM is the only layer with
%   headroom (measured: best frozen constant b gives near-wall -0.010, estimating it gives -0.017, feeding the true curve +0.003).
%   This compares ONE-PARAMETER families against the exact Brenner curve in the estimator's own coordinate u = 1/(1 - a_bar):
%     A  u = 1 + b (w-1)                                    free b   -- the current family (chord; contact pinned at w = 1)
%     B  u = 1 + (8/9)(w-1) + (1/9)(w-1)/(1 + (w-1)/k)      free k   -- two ANCHORS built in (slope 1 at contact, 8/9 far field),
%     C  u = 1 + (8/9)(w-1) + (1/9)(w-1) exp(-(w-1)/k)      free k      the single unknown is the transition length
%     D  c_perp = 1 + K/(w-1),  a_bar = 1/c_perp            free K   -- the powerlaw sibling at p = 1
%   Metric: sup |a_bar_family - a_bar_Brenner| over the whole travelled band and over the revisited band; the parameter is chosen
%   by minimax (fminbnd on the sup), i.e. the best that family can possibly do. No simulation, no estimator.
%   Output printed + one figure one_param_families.png | EXPIRES: with the b line | 產線改動不會自動跟上
function out = compare_one_param_families()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));  od = fullfile(root, 'test_results', 'apd_acov_meng');
    BANDS = {'full  [1.10, 22.2]', 1.10, 22.222; 'revisited  [1.10, 3.32]', 1.10, 3.32; 'near wall  [1.10, 2.0]', 1.10, 2.0};
    fam = {'A current: u = 1 + b(w-1)',              @(p, w) 1 + p*(w-1),                                    [0.5 1.5]; ...
           'B two-anchor rational, free k',          @(p, w) 1 + (8/9)*(w-1) + (1/9)*(w-1)./(1 + (w-1)/p),   [0.02 50]; ...
           'C two-anchor exponential, free k',       @(p, w) 1 + (8/9)*(w-1) + (1/9)*(w-1).*exp(-(w-1)/p),   [0.02 50]; ...
           'D powerlaw p=1: c = 1 + K/(w-1)',        @(p, w) 1./(1 - 1./(1 + p./(w-1))),                     [0.05 5]};
    out = struct();
    for ib = 1:size(BANDS,1)
        w = linspace(BANDS{ib,2}, BANDS{ib,3}, 6000).';
        at = zeros(size(w)); for i = 1:numel(w); [~, c] = calc_correction_functions(w(i), true); at(i) = 1/c; end
        fprintf('\n=== band %s (true a_bar %.4f .. %.4f) ===\n', BANDS{ib,1}, at(1), at(end));
        for k = 1:size(fam,1)
            f = fam{k,2}; br = fam{k,3};
            sup = @(p) max(abs(local_a(f, p, w) - at));
            p = fminbnd(sup, br(1), br(2), optimset('TolX', 1e-10));
            fprintf('  %-38s best param %8.4f | sup |da_bar| %.5f | rms %.5f\n', fam{k,1}, p, sup(p), sqrt(mean((local_a(f,p,w) - at).^2)));
            out.(sprintf('band%d_fam%d', ib, k)) = struct('p', p, 'sup', sup(p));
        end
    end
    % figure on the revisited band
    w = linspace(1.10, 3.32, 2000).'; at = zeros(size(w)); for i = 1:numel(w); [~, c] = calc_correction_functions(w(i), true); at(i) = 1/c; end
    COL = [0.35 0.35 0.35; 0 0.55 0.2; 0.9 0.5 0; 0.55 0 0.7];
    f2 = figure('Units','inches','Position',[0 0 12 5], 'Color','w', 'Visible','off'); tiledlayout(1,2,'TileSpacing','compact','Padding','compact');
    for pnl = 1:2
        nexttile; hold on; H = []; L = {};
        if pnl == 1; H(end+1) = plot(w, at, '-', 'Color', [0.8 0 0], 'LineWidth', 2.4); L{end+1} = 'Brenner (true)'; end
        for k = 1:size(fam,1)
            sup = @(p) max(abs(local_a(fam{k,2}, p, w) - at)); p = fminbnd(sup, fam{k,3}(1), fam{k,3}(2), optimset('TolX',1e-10));
            y = local_a(fam{k,2}, p, w);  if pnl == 2; y = y - at; end
            H(end+1) = plot(w, y, '-', 'Color', COL(k,:), 'LineWidth', 1.6); L{end+1} = fam{k,1};
        end
        if pnl == 2; yline(0, '-', 'Color', [0.6 0.6 0.6], 'HandleVisibility','off'); ylabel('\^a_z/a_{nom} - a_z/a_{nom}   (family - true)', 'FontSize', 13, 'FontWeight','bold');
        else; ylabel('a_z / a_{nom}', 'FontSize', 13, 'FontWeight','bold'); end
        xlabel('w/R', 'FontSize', 13, 'FontWeight','bold');
        legend(H, L, 'Location','northoutside','Orientation','vertical','FontSize', 9, 'FontWeight','bold','Box','on');
        set(gca,'FontSize',13,'FontWeight','bold','LineWidth',1.5,'Box','on'); grid off; xlim([1.1 3.32]);
    end
    png = fullfile(od, 'one_param_families.png'); exportgraphics(f2, png, 'Resolution', 150); close(f2); fprintf('\nsaved %s\n', png);
end
function y = local_a(f, p, w); u = f(p, w); u = max(u, 1 + 1e-9); y = 1 - 1./u; end
