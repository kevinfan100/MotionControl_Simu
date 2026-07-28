# 估測函數主線現況（gain-law family）

**每 session 自動載入的主線快照**。章程與約束見 CLAUDE.md「研究主線」節；本檔補檔案級對應。
更新時機：形式、入口、缺陷狀態任一改變時。

## 兩個對等形式

| | powerlaw | expgain |
|---|---|---|
| gain law | c(h̄) = 1 + K/(h̄−1)^p | a_h(h̄) = a_o·[1 − h̄^(−b)]，φ = ln h̄ |
| state | [δh̄₁ δh̄₂ δh̄₃, a_h, p] | 7b: […, a_h, b]／7a: […, a_o, b]（gain 代數求值） |
| controller | `motion_control_law_5state_powerlaw.m` | `_5state_expgain.m`(7b)／`_5state_expgain_alg.m`(7a) |
| driver | `test_script/integration/run_5state_powerlaw.m` | `test_script/integration/run_5state_expgain.m`（opts.variant='diff'/'alg'） |
| 推導 | `derivation/5state_powerlaw_hd.tex` | `derivation/5state_expgain_hd.tex` ⚠ 正文停在 7b，code 以 7a 檔為準 |
| 長短處 | ⊥ 最準（sup 0.034）；∥ 結構性失效（Goldman 對數非冪次） | 三軸可用（∥ h̄≥2 0.034）；⊥ 差 2.8× |

- prior：兩漸近錨（Brenner 近壁 + method-of-reflections 遠場）都逼出 p=1／b=1；
  √P[0] = 內插間隙 sup|θ_eff−1|（p: 0.035、b: 0.10），全域、與軌跡無關、不讀 c
- 種子：反射係數 9/8（⊥）、9/16（∥）；種子誤差係數 = A²（推導非擬合）→ 應 per-axis
- 兩形式 run-time 完全 c-free（全家唯一不呼叫 `calc_correction_functions` 的 controller）

## 驗收工具（形狀或 prior 改動後必跑）

- `verify_shape_exponent_bound.m` — sup|θ_eff−1| ≲ √P[0] ⟺ Q_θ=0 誠實（PASS/TIGHT/FAIL）
- a_cov 不變性 — `verify_5state_expgain_acov.m`／`verify_powerlaw_regress_A12.m`
- P[0] 預算 — E[(x̂∞−x̂₀)²] ≤ P[0]−P[∞]

## 現況（2026-07-28）

- 缺陷 1（增益模型微分式積分截斷）已由 7a 代數式修復：descent peak 11.03% → 5.05%
- **缺陷 2 = 未解主項**：運動時變異數讀數 +4~5%，隨頻率與 a_pd 時間常數增長、與形狀無關
  → 下一步重推 `Cdpmr_Cn_derivation.tex`（現行假設穩態、僅噪聲驅動的閉迴路）；
  證據腳本在 `test_script/scratch/`
- 兩形式 prior margin 皆 TIGHT（1.02–1.06×）→ 對照與定案門檻見
  `reference/eq17_analysis/shape_ledger.md`
