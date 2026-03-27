# Research & Verification Workflow

## 討論優先原則

任何公式推導、驗證、或分析工作開始前，必須按以下順序和使用者討論：

1. **目標定義** — 要驗證什麼、用什麼方法
2. **Notation 統一** — 變數符號、矩陣命名、code 變數名對應，記錄在文件中
3. **模擬設定** — 參數、sweep 範圍、simulation duration、seed 策略
4. **驗證方法** — Lyapunov / 積分 / Monte Carlo / Simulink，以及交叉驗證策略
5. **圖表規格** — 圖的類型、軸標籤、風格（參照 memory: feedback_figure_style）
6. **輸出規格** — 存檔格式（.mat + .md）、存放位置、commit 策略

使用 interview 模式：一次問一個問題，讓使用者主導方向。不要假設使用者的意圖。

## 文件記錄要求

- 每個分析任務必須產出 .md（人可讀報告）和 .mat（MATLAB 數據）
- 報告集中存放，不要散落多個小 .md
- 可 commit 的文件（report、圖）放在 reference/ 或 agent_docs/
- 大型可重生的數據（.mat）放在 test_results/（gitignored）
- temp_*.m 腳本用完必須刪除

## 跨電腦同步

重要發現必須同步更新到 git-tracked 位置（確保另一台電腦可見）：
- 結論和公式 → agent_docs/verification-notes.md
- 規則和偏好 → .claude/rules/
- 完整報告 → reference/for_test/

記憶（~/.claude/projects/.../memory/）只存在本機，作為 session 連續性用。
關鍵內容必須同時存在 agent_docs/ 中。

## 驗證標準

- 解析公式 vs Lyapunov：應達到 machine precision（< 1e-10）
- Lyapunov vs Simulink：< 5% 為通過
- 所有結果必須有數值表格，不能只靠圖判斷
