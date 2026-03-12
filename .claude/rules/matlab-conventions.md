---
globs: ["**/*.m"]
---

# MATLAB 檔案慣例

## 命名規範
- 臨時測試腳本用 `temp_*.m` 命名，用完必須刪除
- 模擬腳本用 `run_*.m`
- 分析腳本用 `analyze_*.m`
- 參數計算用 `calc_*.m`

## 編碼規則
- 程式碼遵循 MATLAB 風格指南
- 所有 Simulink Bus 元素必須為 double 型別
