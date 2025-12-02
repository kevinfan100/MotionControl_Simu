function [f_T, t] = simulate_brownian_motion()
      % 參數
      k_B = 1.380649e-23;  % Boltzmann 常數 (J/K)
      T = 300;             % 溫度 (K)
      gamma_N = 0.0425;    % 拖曳係數 (pN·s/μm)
      f_s = 1606;          % 取樣頻率 (Hz)
      S_t = 1/f_s;         % 取樣週期 (s)
      N = 1600;            % 取樣點數
      gamma_SI = gamma_N * 1e-6;  % N·s/m

      % 計算熱力標準差 (SI 單位)
      sigma_fT_SI = sqrt(4 * k_B * T * S_t / gamma_SI);  % 單位: N

      % 轉成 pN
      sigma_fT_pN = sigma_fT_SI * 1e12;  % 單位: pN

      % 產生隨機熱力序列 (3D 高斯白雜訊)
      f_T = sigma_fT_pN * randn(3, N);  % 單位: pN  
      
      %% f_T 就是要加入block 的parameter
     
  %% 驗證熱力是否為高斯分布
      %% 繪製直方圖
       figure('Name', '熱力高斯分佈驗證', 'Position', [200 200 800 600]);
    
      % 使用 x 方向的資料
      data = f_T(1, :);
    
      % 繪製直方圖
      histogram(data, 50, 'Normalization', 'pdf', 'FaceColor', [0.3 0.6 0.9], 'EdgeColor', 'w');
      hold on;
    
      % 繪製理論高斯曲線
      x = linspace(-4*sigma_fT_pN, 4*sigma_fT_pN, 200);
      y = normpdf(x, 0, sigma_fT_pN);
      plot(x, y, 'r-', 'LineWidth', 2.5);
    
      % 標註
      xlabel('f_{Tx} (pN)', 'FontSize', 12);
      ylabel('Probability Density', 'FontSize', 12);
      title(sprintf('熱力高斯分佈驗證 (N = %d, σ = %.4f pN)', N, sigma_fT_pN), 'FontSize', 14);
      legend('實測分佈', sprintf('理論 N(0, %.4f²)', sigma_fT_pN), 'FontSize', 11);
      grid on;
    
      % 顯示統計資訊
      text_str = sprintf('實測統計:\n平均值 = %.4f pN\n標準差 = %.4f pN', mean(data), std(data));
      text(0.02, 0.98, text_str, 'Units', 'normalized', 'VerticalAlignment', 'top', ...
           'FontSize', 10, 'BackgroundColor', 'w', 'EdgeColor', 'k');
    end