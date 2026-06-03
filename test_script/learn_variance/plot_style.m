function style = plot_style()
%PLOT_STYLE  Visual conventions for variance estimate learning sandbox.
%
%   style = plot_style()
%
%   Returns a struct of color palette, font sizes, line widths, layout
%   defaults, and save settings shared across all sandbox plots.
%
%   Companion helpers (in this folder):
%       apply_default_style(style)     - set groot defaults from style
%       new_stacked_figure(n, style)   - create tall stacked time-series fig
%       save_figure(fig, name, style)  - export PNG with consistent DPI/path
%
%   Usage:
%       style = plot_style();
%       apply_default_style(style);
%       fig = new_stacked_figure(6, style);
%       ...
%       save_figure(fig, 'baseline_dc', style);

    style = struct();

    % ===== Color palette =====================================================
    % Single-config "baseline" plots
    style.color.baseline    = [0.00 0.00 0.00];   % black, primary signal
    style.color.signal_in   = [0.30 0.30 0.30];   % dark gray, input del_pm
    style.color.theory      = [0.55 0.55 0.55];   % gray dashed, theoretical reference
    style.color.true_value  = [0.85 0.10 0.10];   % red, ground truth horizontal line
    style.color.zero_line   = [0.70 0.70 0.70];   % light gray, y=0 reference

    % Sweep palette (3 levels: small / medium / large window)
    %   small a  = large window = slow / smooth   -> blue
    %   medium a = baseline                        -> orange
    %   large a  = small window = fast / noisy    -> purple
    style.color.sweep_low   = [0.10 0.30 0.85];
    style.color.sweep_mid   = [0.95 0.55 0.10];
    style.color.sweep_high  = [0.55 0.20 0.65];
    style.sweep_cmap        = [style.color.sweep_low;
                               style.color.sweep_mid;
                               style.color.sweep_high];

    % Annotation accents (e.g. shading transient region, marking f_c)
    style.color.shade_transient = [0.95 0.95 0.80];   % pale yellow
    style.color.marker_event    = [0.25 0.65 0.25];   % green

    % ===== Fonts =============================================================
    style.font.family       = 'Helvetica';
    style.font.title_size   = 11;
    style.font.label_size   = 10;
    style.font.tick_size    = 9;
    style.font.legend_size  = 9;
    style.font.annotation_size = 8;

    % ===== Line widths =======================================================
    style.lw.signal     = 0.8;     % time-series line
    style.lw.baseline   = 1.4;     % bolded baseline curve
    style.lw.theory     = 1.0;     % theory reference (dashed)
    style.lw.true_value = 1.2;     % ground truth horizontal
    style.lw.gridline   = 0.4;
    style.lw.zero_line  = 0.6;

    % ===== Layout ============================================================
    %   width 9", height varies by panel count
    style.layout.figure_size_default  = [9, 5];        % single-panel
    style.layout.figure_size_stacked  = [9, 11];       % multi-row stacked time series
    style.layout.figure_size_sweep    = [11, 6];       % side-by-side sweep
    style.layout.tile_padding         = 'compact';
    style.layout.tile_spacing         = 'compact';

    % ===== Save settings =====================================================
    %   Resolved relative to project root (current working dir at script run)
    style.save.dpi          = 150;
    style.save.dir          = fullfile(pwd, 'test_results', 'learn_variance');
    style.save.prefix       = 'lv_';
    style.save.format       = 'png';

    % ===== Time-axis convention ==============================================
    %   Use seconds for >= 0.1 s windows, milliseconds for shorter.
    style.time.threshold_to_ms = 0.1;     % below this -> show in ms

end
