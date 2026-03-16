# EKF Estimation Controller — Development Progress

## 2026-03-16 — Q/R Matrix Analysis & EKF Stability Investigation

### Completed Parts
- ✅ Q/R 矩陣完整分析（Q11-Q99, R_pp/R_λλ/R_θθ）— 確認 Q11-Q33 物理推導，Q44-Q99 為 design heuristic
- ✅ Lambda convention 修正：lambda = 1/c_para（mobility factor），修正 run_simulation.m
- ✅ EKF 數值穩定性改善：Joseph form、Pf symmetrization、scaled init、force saturation
- ✅ Delay(2) on p_m 信號流驗證：確認為 ADC pipeline emulation（正確）
- ✅ Control law 驗證：known-lambda test 追蹤誤差 0 nm（控制律正確）
- ✅ EKF 11 次測試完整記錄 + root cause analysis
- ✅ 論文 Ch4/Ch5 閱讀 — 發現缺失的 forgetting factor
- ⏸️ EKF 收斂性修復（核心問題未解決，已有修復計畫）

### File Changes
**New Files:**
- `agent_docs/ekf-qr-analysis.md` (~500 lines)
  Purpose: 完整 Q/R 推導分析、測試記錄、root cause analysis、修復計畫
- `reference/Thesis_Ch4_Ch5/` (7 files)
  Purpose: 論文第 4、5 章 PDF 與 text，EKF 估測器原始設計參考

**Modified Files:**
- `model/controller/motion_control_law.m` (+85 lines)
  Main changes: Joseph form、Pf scaled init、lambda/force clamp、warmup、NaN guard
- `model/system_model.slx`
  Main changes: Delay(2) 先移除後復原（最終狀態 = 正確復原）
- `test_script/run_simulation.m` (+3 lines)
  Main changes: lambda_true = [1/c_para; 1/c_perp]（修正比較基準）

### Testing Status
⏸️ EKF 收斂性測試進行中
- Known-lambda controller: 追蹤 0 nm ✅
- Open-loop simulation: 正常 ✅
- EKF closed-loop: 11 次測試均無法收斂 ❌（已記錄詳細 log）

### Next Steps
- [ ] Phase 1: 加入 forgetting factor（論文 Ch4/Ch5 有，code 缺失）
- [ ] Phase 2: 清理 code，建立乾淨 baseline（移除 debug 暫時修改）
- [ ] Phase 3: EKF 收斂驗證（從 h_init=20 開始，逐步降低）
- [ ] Phase 4: 完善 Q88/Q99 和 R 參數

### Issues & Notes
⚠️ **Critical:**
- EKF 23-state 系統在 closed-loop 下無法收斂（controller-EKF positive feedback）
- Forgetting factor 是論文有但 code 缺失的關鍵元素
- Q44-Q99 和 R 都是 design heuristic，非物理推導

💡 **Key findings from thesis Ch4/Ch5:**
- Ch4 用 1D estimator（每軸分開），Ch5 擴展到 3D（我們的 PDF 架構原型）
- 論文的 Pf update 有 forgetting factor α（防止 Kalman gain 消失）
- 論文實驗從遠離壁面開始（lambda ≈ 1），給估測器 warmup 時間
- 控制律公式完全正確（known-lambda 驗證）

### Git Commits (this session)
`80d5114` - WIP(control): EKF Q/R analysis and stability investigation
`b345d35` - docs(control): Add comprehensive EKF test log and root cause analysis
`d43b371` - docs(control): Correct Delay(2) analysis — ADC pipeline is intentional
`54ffcab` - fix(simulation): Restore Delay(2) on p_m path to Controller
`36672e7` - fix(control): Add EKF numerical safeguards and remove Delay(2) bug
`f727da4` - docs(control): Add Pf divergence diagnosis and fix plan to Q/R analysis
`65e547c` - fix(control): Correct lambda_true to mobility factor (1/c) and add Q/R analysis

---
