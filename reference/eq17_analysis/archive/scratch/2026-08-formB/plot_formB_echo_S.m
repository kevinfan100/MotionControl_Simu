function plot_formB_echo_S(mat_path)
%PLOT_FORMB_ECHO_S  Read check_formB_echo_S_measure output and judge it against
%   the Lyapunov model EVALUATED AT THE SAME eps.
%   FORK OF nothing -- companion of check_formB_echo_S_measure | PURPOSE: the
%   honest comparison. The controller's S_T/S_n are the eps->0 derivative, but a
%   forced-mismatch measurement at finite eps carries the model's own O(eps^2)
%   curvature. Comparing measurement-at-eps to model-at-0 charges the model for
%   a term it never claimed. | EXPIRES: with its measurement sibling.

    if nargin < 1 || isempty(mat_path)
        mat_path = fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), ...
                            'test_results', 'echo_S', 'echo_S_out.mat');
    end
    S = load(mat_path);  out = S.out;

    [script_dir, ~, ~] = fileparts(mfilename('fullpath'));
    project_root = fileparts(fileparts(script_dir));
    out_dir = fullfile(project_root, 'test_results', 'echo_S');
    if ~exist(out_dir, 'dir'); mkdir(out_dir); end

    LC = 0.7; APD = 0.05;

    fprintf('\n===== S vs model AT THE SAME eps =====\n');
    fprintf('%-4s %-6s %-4s | %-9s %-9s %-8s %s\n', ...
            'arm', 'eps', 'ax', 'measured', 'model', 'dev', 'sigma');
    for k = 1:numel(out.arms)
        r = out.arms{k};
        for ie = 1:numel(r.eps)
            ep = r.eps(ie);
            ST = model_S(ep, 1, LC, APD);
            Sn = model_S(ep, 2, LC, APD);
            for ax = 1:3
                switch r.tag
                    case 'T'; mdl = ST;
                    case 'N'; mdl = Sn;
                    otherwise
                        mdl = (ST*r.a_bar(ax) + Sn*r.xi_bar(ax)) / (r.a_bar(ax) + r.xi_bar(ax));
                end
                dev = r.S(ie,ax) - mdl;
                sg  = dev / max(r.S_sem(ie,ax), eps);
                fprintf('%-4s %-6.2f %-4d | %+9.4f %+9.4f %+8.4f %+5.1f %s\n', ...
                        r.tag, ep, ax, r.S(ie,ax), mdl, dev, sg, ...
                        pf(abs(sg) <= 2));
            end
        end
    end

    fprintf('\n===== coefficient cross-check (readout slope / variance slope) =====\n');
    for k = 1:numel(out.arms)
        r = out.arms{k};
        if ~any(strcmp(r.tag, {'M22','M2'})); continue; end
        ratio_meas = r.S_read(1,3) / r.S(1,3);
        ratio_pred = (r.a_bar(3) + r.xi_bar(3)) / r.a_bar(3);
        fprintf(['%-4s ax3: measured %.4f   predicted (a+xi)/a %.4f   ' ...
                 'excess %+.2f%%   [code uses 1.0000]\n'], ...
                r.tag, ratio_meas, ratio_pred, 100*(ratio_meas/ratio_pred - 1));
    end

    % ---------------- figure ----------------
    COL_MODEL = [0.8 0 0]; COL_X = [0.45 0.55 0.95]; COL_Z = [0 0.2 0.9];
    FS = 18; LFS = 13; AXLW = 2.0;
    f = figure('Position', [80 80 1150 780], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    % panel 1 : arm T, the eps curve -- model curvature vs measured curvature
    rT = out.arms{strcmp(cellfun(@(c) c.tag, out.arms, 'UniformOutput', false), 'T')};
    epg = linspace(0.02, 0.22, 40);
    mg  = arrayfun(@(e) model_S(e, 1, LC, APD), epg);
    nexttile; hold on;
    hm = plot(epg, mg, '-', 'Color', COL_MODEL, 'LineWidth', 2.5, ...
              'DisplayName', 'Lyapunov model S_T(\epsilon)');
    h1 = errorbar(rT.eps, rT.S(:,1), rT.S_sem(:,1), 'o', 'Color', COL_X, ...
                  'MarkerFaceColor', COL_X, 'LineWidth', 2.0, 'MarkerSize', 9, ...
                  'DisplayName', 'measured x');
    h3 = errorbar(rT.eps, rT.S(:,3), rT.S_sem(:,3), 's', 'Color', COL_Z, ...
                  'MarkerFaceColor', COL_Z, 'LineWidth', 2.0, 'MarkerSize', 9, ...
                  'DisplayName', 'measured z');
    xlabel('forced mismatch \epsilon', 'FontSize', FS, 'FontWeight', 'bold');
    ylabel('S_T  (thermal-only arm)', 'FontSize', FS, 'FontWeight', 'bold');
    legend([hm h1 h3], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
           'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
    xlim([0 0.24]);

    % panel 2 : all four arms on z at eps = 0.10, measured vs same-eps model
    nexttile; hold on;
    tags = {}; meas = []; msem = []; pred = [];
    for k = 1:numel(out.arms)
        r = out.arms{k};
        ie = find(abs(r.eps - 0.10) < 1e-9, 1);  if isempty(ie); ie = 1; end
        ST = model_S(r.eps(ie), 1, LC, APD);  Sn = model_S(r.eps(ie), 2, LC, APD);
        switch r.tag
            case 'T'; p = ST;
            case 'N'; p = Sn;
            otherwise; p = (ST*r.a_bar(3) + Sn*r.xi_bar(3)) / (r.a_bar(3) + r.xi_bar(3));
        end
        tags{end+1} = r.tag; %#ok<AGROW>
        meas(end+1) = r.S(ie,3); msem(end+1) = r.S_sem(ie,3); pred(end+1) = p; %#ok<AGROW>
    end
    xk = 1:numel(tags);
    hp = plot(xk, pred, 'd', 'Color', COL_MODEL, 'MarkerFaceColor', COL_MODEL, ...
              'MarkerSize', 13, 'LineStyle', 'none', 'DisplayName', 'model at same \epsilon');
    hq = errorbar(xk, meas, msem, 'o', 'Color', COL_Z, 'MarkerFaceColor', COL_Z, ...
                  'MarkerSize', 10, 'LineWidth', 2.0, 'LineStyle', 'none', ...
                  'DisplayName', 'measured (8 seeds)');
    yline(0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off');
    set(gca, 'XTick', xk, 'XTickLabel', tags, 'XLim', [0.5, numel(tags)+0.5]);
    xlabel('arm   (T thermal only | N noise only | M22 mixed far | M2 mixed trough)', ...
           'FontSize', 14, 'FontWeight', 'bold');
    ylabel('S   (z axis, \epsilon = 0.10)', 'FontSize', FS, 'FontWeight', 'bold');
    legend([hp hq], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
           'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;

    fp = fullfile(out_dir, 'fig_echo_S_vs_model.png');
    exportgraphics(f, fp, 'Resolution', 150);
    close(f);
    fprintf('\nfigure -> %s\n', fp);
end


function S = model_S(ep, iN, lambda_c, a_pd)
%MODEL_S  The controller's Lyapunov slope, evaluated with the SAME finite step.
    S = (log(vg(1/(1+ep), iN, lambda_c, a_pd)) - log(vg(1/(1-ep), iN, lambda_c, a_pd))) / (2*ep);
end


function v = vg(gE, iN, lambda_c, a_pd)
    alE = 1 - lambda_c;
    AE = zeros(6); BqE = zeros(6,1); BnE = zeros(6,1);
    AE(1,1)=1; AE(1,3)=-gE*alE; AE(1,4)=-gE*alE; AE(1,5)=-gE*alE;
    BnE(1)=-gE*alE; BqE(1)=1;
    AE(2,1)=1; AE(3,2)=1;
    AE(4,3)=-alE; AE(4,4)=-alE; AE(4,5)=-alE; BnE(4)=-alE;
    AE(5,4)=1; AE(6,3)=a_pd; AE(6,6)=1-a_pd; BnE(6)=a_pd;
    if iN == 1; QE = BqE*BqE.'; extraE = 0; else; QE = BnE*BnE.'; extraE = (1-a_pd)^2; end
    XE = reshape((eye(36) - kron(AE,AE)) \ QE(:), 6, 6);
    cE = zeros(1,6); cE(3) = 1-a_pd; cE(6) = -(1-a_pd);
    v = cE*XE*cE.' + extraE;
end


function s = pf(c)
    if c; s = 'PASS'; else; s = 'CHECK'; end
end
