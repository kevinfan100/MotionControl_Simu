function verify_qr_positioning_aggregate()
%VERIFY_QR_POSITIONING_AGGREGATE Combine batch outputs and produce summary
%
%   Loads qr_pos_b1.mat ... qr_pos_b4.mat (or however many exist)
%   Aggregates per (scenario, variant) across 5 seeds.
%   Generates:
%     - reference/for_test/qr_positioning_h25_report.md
%     - reference/for_test/qr_positioning_h50_report.md
%     - reference/for_test/qr_positioning_summary.md
%     - reference/for_test/fig_qr_positioning_summary.png
%     - test_results/verify/qr_positioning_combined.mat

    [script_dir,~,~] = fileparts(mfilename('fullpath'));
    project_root = fileparts(script_dir);
    addpath(fullfile(project_root, 'model'));
    addpath(fullfile(project_root, 'model', 'config'));
    addpath(fullfile(project_root, 'model', 'wall_effect'));
    addpath(script_dir);
    cd(project_root);

    out_dir = fullfile(project_root, 'test_results', 'verify');
    rep_dir = fullfile(project_root, 'reference', 'for_test');

    % Find batch files
    batch_files = dir(fullfile(out_dir, 'qr_pos_b*.mat'));
    if isempty(batch_files)
        error('No batch files found in %s', out_dir);
    end
    fprintf('Found %d batch files:\n', numel(batch_files));
    for k = 1:numel(batch_files)
        fprintf('  %s (%s)\n', batch_files(k).name, batch_files(k).date);
    end

    % Load all and concatenate results (skip empty cells from crashed batches)
    all_results = {};
    for k = 1:numel(batch_files)
        d = load(fullfile(out_dir, batch_files(k).name));
        for kk = 1:numel(d.results)
            if isempty(d.results{kk}), continue; end
            all_results{end+1, 1} = d.results{kk};
        end
    end
    fprintf('Total non-empty runs loaded: %d\n', numel(all_results));

    % Get last batch metadata (scenarios, variants, seeds)
    last = load(fullfile(out_dir, batch_files(end).name));
    scenarios = last.scenarios;
    variants  = last.variants;
    seeds     = last.seeds;

    n_sc = numel(scenarios); n_var = numel(variants); n_seed = numel(seeds);

    % Build (scenario, variant) -> [seed1..seed5 results]
    grid = cell(n_sc, n_var);
    for k = 1:numel(all_results)
        r = all_results{k};
        is = find(strcmp({scenarios.name}, r.scenario));
        iv = find(strcmp({variants.name},  r.variant));
        if isempty(is) || isempty(iv), continue; end
        grid{is, iv}(end+1) = r;
    end

    % Aggregate per cell
    metric_names = {'tracking_mean_x','tracking_mean_y','tracking_mean_z', ...
                    'tracking_std_x','tracking_std_y','tracking_std_z', ...
                    'tracking_rmse_3d', ...
                    'theory_std_dpmr_x_nm','theory_std_dpmr_z_nm', ...
                    'ahat_bias_x_pct','ahat_bias_z_pct', ...
                    'ahat_std_x_pct','ahat_std_z_pct', ...
                    'ahat_max_x_pct','ahat_max_z_pct', ...
                    'spike_x_pct','spike_z_pct','spike_z_t', ...
                    'nan_count'};

    agg = struct();
    for is = 1:n_sc
        for iv = 1:n_var
            cell_runs = grid{is, iv};
            if isempty(cell_runs), continue; end
            n_runs = numel(cell_runs);
            for m = 1:numel(metric_names)
                vals = arrayfun(@(rr) rr.(metric_names{m}), cell_runs);
                agg.(scenarios(is).name).(variants(iv).name).([metric_names{m} '_mean']) = mean(vals);
                agg.(scenarios(is).name).(variants(iv).name).([metric_names{m} '_std'])  = std(vals);
                agg.(scenarios(is).name).(variants(iv).name).([metric_names{m} '_n'])    = n_runs;
                agg.(scenarios(is).name).(variants(iv).name).([metric_names{m} '_vals']) = vals;
            end
            % Theory ratio (z) using mean theory
            th_mean = mean(arrayfun(@(rr) rr.theory_std_dpmr_z_nm, cell_runs));
            emp_z = arrayfun(@(rr) rr.tracking_std_z, cell_runs);
            agg.(scenarios(is).name).(variants(iv).name).ratio_z_mean = mean(emp_z) / th_mean;
            agg.(scenarios(is).name).(variants(iv).name).ratio_z_std  = std(emp_z) / th_mean;
        end
    end

    % Save combined
    combined_file = fullfile(out_dir, 'qr_positioning_combined.mat');
    save(combined_file, 'agg', 'all_results', 'grid', 'scenarios', 'variants', 'seeds');
    fprintf('\nSaved combined: %s\n', combined_file);

    % Generate per-scenario reports
    for is = 1:n_sc
        sc = scenarios(is);
        rep_file = fullfile(rep_dir, sprintf('qr_positioning_%s_report.md', sc.name));
        write_scenario_report(rep_file, sc, variants, seeds, agg, grid(is, :));
        fprintf('Saved: %s\n', rep_file);
    end

    % Combined summary
    sum_file = fullfile(rep_dir, 'qr_positioning_summary.md');
    write_summary_report(sum_file, scenarios, variants, agg);
    fprintf('Saved: %s\n', sum_file);

    % Plot
    fig_file = fullfile(rep_dir, 'fig_qr_positioning_summary.png');
    plot_summary(scenarios, variants, agg, fig_file);
    fprintf('Saved: %s\n', fig_file);

    % Console summary
    fprintf('\n===== Console summary =====\n');
    print_console_table(scenarios, variants, agg);
end

%% =========================================================================
function write_scenario_report(rep_file, sc, variants, seeds, agg, grid_row)
    fid = fopen(rep_file, 'w');
    fprintf(fid, '# Q/R Verification — %s (h_init = %.2f um)\n\n', sc.name, sc.h_init);
    fprintf(fid, '**Generated**: %s\n\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
    fprintf(fid, '## Setup\n\n');
    fprintf(fid, '- Trajectory: positioning (static hold at h_init = %.2f um)\n', sc.h_init);
    fprintf(fid, '- T_sim = 30 s, t_warmup = 10 s (steady-state window 20 s)\n');
    fprintf(fid, '- Noise ON, thermal ON, lc=0.7, controller_type=7 (7-state EKF)\n');
    fprintf(fid, '- Seeds (5): %s\n\n', mat2str(seeds));

    n_var = numel(variants);

    fprintf(fid, '## Tracking error per axis (nm) — mean and std across 5 seeds\n\n');
    fprintf(fid, '| variant | mean_x | mean_y | mean_z | std_x | std_y | std_z | 3D RMSE |\n');
    fprintf(fid, '|---|---|---|---|---|---|---|---|\n');
    for iv = 1:n_var
        if ~isfield(agg.(sc.name), variants(iv).name), continue; end
        a = agg.(sc.name).(variants(iv).name);
        fprintf(fid, '| %s | %+.2f ± %.2f | %+.2f ± %.2f | %+.2f ± %.2f | %.2f ± %.2f | %.2f ± %.2f | %.2f ± %.2f | %.2f ± %.2f |\n', ...
            variants(iv).name, ...
            a.tracking_mean_x_mean, a.tracking_mean_x_std, ...
            a.tracking_mean_y_mean, a.tracking_mean_y_std, ...
            a.tracking_mean_z_mean, a.tracking_mean_z_std, ...
            a.tracking_std_x_mean,  a.tracking_std_x_std, ...
            a.tracking_std_y_mean,  a.tracking_std_y_std, ...
            a.tracking_std_z_mean,  a.tracking_std_z_std, ...
            a.tracking_rmse_3d_mean, a.tracking_rmse_3d_std);
    end

    fprintf(fid, '\n## Tracking std vs theory (del_pmr ratio)\n\n');
    fprintf(fid, '| variant | empirical std_z | theory std_z | ratio_z |\n');
    fprintf(fid, '|---|---|---|---|\n');
    for iv = 1:n_var
        if ~isfield(agg.(sc.name), variants(iv).name), continue; end
        a = agg.(sc.name).(variants(iv).name);
        fprintf(fid, '| %s | %.2f ± %.2f nm | %.2f nm | %.3f ± %.3f |\n', ...
            variants(iv).name, a.tracking_std_z_mean, a.tracking_std_z_std, ...
            a.theory_std_dpmr_z_nm_mean, a.ratio_z_mean, a.ratio_z_std);
    end

    fprintf(fid, '\n## a_hat error in %% (relative to a_true)\n\n');
    fprintf(fid, '| variant | bias_x %% | std_x %% | bias_z %% | std_z %% | max_x %% | max_z %% |\n');
    fprintf(fid, '|---|---|---|---|---|---|---|\n');
    for iv = 1:n_var
        if ~isfield(agg.(sc.name), variants(iv).name), continue; end
        a = agg.(sc.name).(variants(iv).name);
        fprintf(fid, '| %s | %+.2f ± %.2f | %.2f ± %.2f | %+.2f ± %.2f | %.2f ± %.2f | %.1f ± %.1f | %.1f ± %.1f |\n', ...
            variants(iv).name, ...
            a.ahat_bias_x_pct_mean, a.ahat_bias_x_pct_std, ...
            a.ahat_std_x_pct_mean,  a.ahat_std_x_pct_std, ...
            a.ahat_bias_z_pct_mean, a.ahat_bias_z_pct_std, ...
            a.ahat_std_z_pct_mean,  a.ahat_std_z_pct_std, ...
            a.ahat_max_x_pct_mean,  a.ahat_max_x_pct_std, ...
            a.ahat_max_z_pct_mean,  a.ahat_max_z_pct_std);
    end

    fprintf(fid, '\n## Spike (max |error| in first 1000 samples)\n\n');
    fprintf(fid, '| variant | spike_x %% | spike_z %% | spike_z time [s] |\n');
    fprintf(fid, '|---|---|---|---|\n');
    for iv = 1:n_var
        if ~isfield(agg.(sc.name), variants(iv).name), continue; end
        a = agg.(sc.name).(variants(iv).name);
        fprintf(fid, '| %s | %.0f ± %.0f | %.0f ± %.0f | %.3f ± %.3f |\n', ...
            variants(iv).name, ...
            a.spike_x_pct_mean, a.spike_x_pct_std, ...
            a.spike_z_pct_mean, a.spike_z_pct_std, ...
            a.spike_z_t_mean,   a.spike_z_t_std);
    end

    fprintf(fid, '\n## Per-seed details (a_hat_z bias and 3D RMSE)\n\n');
    fprintf(fid, '| variant | seed→ |');
    n_seed = numel(seeds);
    for ie = 1:n_seed, fprintf(fid, ' %d |', seeds(ie)); end
    fprintf(fid, '\n|---|---|');
    for ie = 1:n_seed, fprintf(fid, '---|'); end
    fprintf(fid, '\n');
    for iv = 1:n_var
        if isempty(grid_row{iv}), continue; end
        % bias_z per seed
        fprintf(fid, '| %s | bias_z %% |', variants(iv).name);
        for ie = 1:n_seed
            % find the run with matching seed
            seed_target = seeds(ie);
            r = grid_row{iv}([grid_row{iv}.seed] == seed_target);
            if isempty(r)
                fprintf(fid, ' - |');
            else
                fprintf(fid, ' %+.1f |', r.ahat_bias_z_pct);
            end
        end
        fprintf(fid, '\n');
        % RMSE_3d per seed
        fprintf(fid, '| | RMSE_3D nm |');
        for ie = 1:n_seed
            seed_target = seeds(ie);
            r = grid_row{iv}([grid_row{iv}.seed] == seed_target);
            if isempty(r)
                fprintf(fid, ' - |');
            else
                fprintf(fid, ' %.1f |', r.tracking_rmse_3d);
            end
        end
        fprintf(fid, '\n');
    end

    fclose(fid);
end

%% =========================================================================
function write_summary_report(rep_file, scenarios, variants, agg)
    fid = fopen(rep_file, 'w');
    fprintf(fid, '# Q/R Positioning Verification — Summary\n\n');
    fprintf(fid, '**Generated**: %s\n\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
    fprintf(fid, '5 Q/R variants × 2 scenarios × 5 seeds = 50 runs.\n\n');

    fprintf(fid, '## Headline numbers (mean ± std across 5 seeds)\n\n');
    for is = 1:numel(scenarios)
        sc = scenarios(is);
        fprintf(fid, '### %s (h_init = %.2f um)\n\n', sc.name, sc.h_init);
        fprintf(fid, '| variant | 3D RMSE [nm] | std_z [nm] | a_hat_z bias %% | a_hat_z std %% | spike_z %% |\n');
        fprintf(fid, '|---|---|---|---|---|---|\n');
        for iv = 1:numel(variants)
            if ~isfield(agg.(sc.name), variants(iv).name), continue; end
            a = agg.(sc.name).(variants(iv).name);
            fprintf(fid, '| %s | %.1f ± %.1f | %.1f ± %.1f | %+.1f ± %.1f | %.1f ± %.1f | %.0f ± %.0f |\n', ...
                variants(iv).name, ...
                a.tracking_rmse_3d_mean, a.tracking_rmse_3d_std, ...
                a.tracking_std_z_mean, a.tracking_std_z_std, ...
                a.ahat_bias_z_pct_mean, a.ahat_bias_z_pct_std, ...
                a.ahat_std_z_pct_mean, a.ahat_std_z_pct_std, ...
                a.spike_z_pct_mean, a.spike_z_pct_std);
        end
        fprintf(fid, '\n');
    end

    fprintf(fid, '## Detailed per-scenario reports\n\n');
    for is = 1:numel(scenarios)
        fprintf(fid, '- [%s](qr_positioning_%s_report.md)\n', scenarios(is).name, scenarios(is).name);
    end

    fclose(fid);
end

%% =========================================================================
function plot_summary(scenarios, variants, agg, fig_file)
    fig = figure('Position', [50, 50, 1600, 1100], 'Visible', 'off');

    n_sc = numel(scenarios); n_var = numel(variants);
    sc_colors = {[0.2 0.4 0.8], [0.8 0.4 0.2]};   % blue near-wall, orange free-space

    metrics = {'tracking_rmse_3d', 'tracking_std_z', 'ahat_bias_z_pct', ...
               'ahat_std_z_pct', 'spike_z_pct', 'ratio_z'};
    metric_titles = {'3D RMSE [nm]', 'tracking std_z [nm]', 'a_{hat,z} bias [%]', ...
                     'a_{hat,z} std [%]', 'spike_z [%]', 'std_z / theory_z'};

    for m = 1:numel(metrics)
        subplot(3, 2, m);
        hold on;
        for is = 1:n_sc
            sc = scenarios(is);
            means = nan(1, n_var);
            stds  = nan(1, n_var);
            for iv = 1:n_var
                if ~isfield(agg.(sc.name), variants(iv).name), continue; end
                a = agg.(sc.name).(variants(iv).name);
                if strcmp(metrics{m}, 'ratio_z')
                    means(iv) = a.ratio_z_mean;
                    stds(iv)  = a.ratio_z_std;
                else
                    means(iv) = a.([metrics{m} '_mean']);
                    stds(iv)  = a.([metrics{m} '_std']);
                end
            end
            x_pos = (1:n_var) + (is-1.5)*0.18;
            errorbar(x_pos, means, stds, 'o-', 'Color', sc_colors{is}, ...
                     'LineWidth', 1.5, 'MarkerSize', 8, 'MarkerFaceColor', sc_colors{is}, ...
                     'DisplayName', sc.name);
        end
        set(gca, 'XTick', 1:n_var, 'XTickLabel', {variants.name}, 'XTickLabelRotation', 30);
        ylabel(metric_titles{m});
        if m == 1
            legend('Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', 10);
        end
        set(gca, 'FontSize', 10);
        if strcmp(metrics{m}, 'ratio_z')
            yline(1, 'k--', 'LineWidth', 1.0);
        end
        if strcmp(metrics{m}(1:min(end,5)), 'ahat_') && contains(metrics{m}, 'bias')
            yline(0, 'k--', 'LineWidth', 1.0);
        end
        xlim([0.5, n_var+0.5]);
    end

    saveas(fig, fig_file);
    close(fig);
end

%% =========================================================================
function print_console_table(scenarios, variants, agg)
    for is = 1:numel(scenarios)
        sc = scenarios(is);
        fprintf('\n--- %s (h_init = %.2f um) ---\n', sc.name, sc.h_init);
        fprintf('%-15s | %14s | %14s | %14s | %14s\n', 'variant', '3D RMSE nm', 'std_z nm', 'a_hat_z bias %', 'a_hat_z std %');
        for iv = 1:numel(variants)
            if ~isfield(agg.(sc.name), variants(iv).name), continue; end
            a = agg.(sc.name).(variants(iv).name);
            fprintf('%-15s | %5.1f ± %5.1f | %5.1f ± %5.1f | %+5.1f ± %5.1f | %5.1f ± %5.1f\n', ...
                variants(iv).name, ...
                a.tracking_rmse_3d_mean, a.tracking_rmse_3d_std, ...
                a.tracking_std_z_mean,   a.tracking_std_z_std, ...
                a.ahat_bias_z_pct_mean,  a.ahat_bias_z_pct_std, ...
                a.ahat_std_z_pct_mean,   a.ahat_std_z_pct_std);
        end
    end
end
