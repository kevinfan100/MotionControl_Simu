# 分析功能與開發指南

## GUI 分析 Tabs
| Tab | 內容 | 數據來源 |
|-----|------|---------|
| 1 | 軌跡總覽 (3D) | p_m, p_d |
| 2-4 | X/Y/Z 軸分析 | p_m, p_d, F |
| 5 | 位置 vs 時間 | p_m |
| 6 | 控制力 vs 時間 | f_d |
| 7+ | 開迴路熱力分析 (條件觸發) | p_m |

## 建議測試參數
```matlab
frequency = 1;              % 軌跡頻率 (Hz)
amplitude = 2.5;            % h 方向振幅 (um)
openloop_cutoff_freq = 5;   % 開迴路分析截止頻率
```
