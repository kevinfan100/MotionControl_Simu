# Agent Coordination Rules

## MATLAB 並行策略

MCP MATLAB 是單一 session，多個 agent 共用時會排隊。真正並行需要：

| 工具 | 用途 | 並行能力 |
|---|---|---|
| MCP `mcp__matlab__evaluate_matlab_code` | Simulink 模擬（需要 workspace） | 單 session，不可並行 |
| Bash `matlab -batch "..."` | 純數學（Lyapunov、積分、symbolic） | 獨立 instance，可並行 |
| Agent (Edit/Read/Write) | Code 修改、文件整理 | 可並行 |

## 並行安排原則

1. Simulink 模擬用 MCP agent（背景）
2. 純數學分析用 Bash MATLAB（背景）
3. Code 修改用獨立 agent（背景）
4. 主線保持空閒供使用者討論

## Agent 記錄要求

每個 agent 必須：
- 存結果到指定位置（.mat + .md）
- 完成後報告產出的檔案清單
- 不修改其他 agent 正在使用的檔案

## 清理規則

- temp_*.m 腳本：分析完成後刪除
- 只保留最終版本的腳本和文件
- .mat 數據檔留在 test_results/（gitignored）
