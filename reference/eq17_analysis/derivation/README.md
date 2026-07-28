# derivation/ — 推導文件索引

每支 tex 檔頭有一行 `% STATUS:`；本表是總覽。歷史（已否證／被取代）在 `archive/`，
舊→新路徑對照見 `../archive/MOVED.md`。

## 現役兩形式（co-equal，對照與定案門檻見 `../shape_ledger.md`）

| 檔 | 內容 |
|---|---|
| `5state_powerlaw_hd.tex` | powerlaw 形式：c = 1 + K/(h̄−1)^p；Stage F 含 R₂ 白化、p prior、fdet、漸近種子 |
| `5state_expgain_hd.tex` | expgain 形式：a_h = a_o[1−h̄^(−b)]，φ=ln h̄；Stage 2b = 形狀驗收判準 SSOT。⚠ 正文停在 7b，code 以 7a 為準 |

## SSOT 基礎鏈（雙機規約 rule 6）

| 檔 | 內容 |
|---|---|
| `4state_del_hd.tex` | 基礎鏈 SSOT（4-state，gain-rate 前饋） |
| `4state_taylor_gain.tex` | taylor gain SSOT |
| `6state_derivation_xd.tex` | RevisedConrol_Vpersonal 1:1 轉寫（結構主基準；flow spec = `../kf_canonical_spec.md`） |

## 推導元件（被上面引用的閉式與通道推導）

`Cdpmr_Cn_derivation.tex`（⚠ 缺陷 2 重推對象）、`R22_derivation.tex`、
`Fe_H_derivation.tex`、`pm_to_axm_derivation.tex`（⚠ 檔頭有白化勘誤）、
`aram_variance.tex`、`a_gain_chain.tex`、`a_physics_chain.tex`、`rw_q44_derivation.tex`、
`4state_del_hd_ar1.tex`（AR(1) companion）、`5state_derivation.tex`（5-state 結構底座）。

## archive/ — 已否證（死因＋結論所在）

| 檔 | 死因 | 結論記錄 |
|---|---|---|
| `5state_aprime_var_esti / var_centered / var_chain / mirror` | var-ratio 家族全滅（κ≈0 自我複讀、洩漏 13×、200× SNR 牆、閉環發散） | memory project_aprime_var_esti_2026-07-15 |
| `5state_aprime_coupdate` | co-update 否證（漂 13×／trk 158nm） | project_coupdate_falsify_asstate_jitter_2026-07-21 |
| `5state_aprime_axm_num` | 分子換 a_xm：結構對但撞資訊牆（readout 70×） | project_axm_numerator_coupdate_2026-07-21 |
| `5state_vs_6state_aprime` | 2nd-order IRW/Singer 全滅（弱可觀＋unknown-c） | project_aprime_2ndorder_compare_2026-07-22 |

## archive/ — 被取代（後繼）

| 檔 | 後繼 |
|---|---|
| `5state_est_aprime / 5state_aprime_selfmod / 5state_aprime_observability / aprime_blackbox_regression` | → `5state_aprime_unified.tex`（其檔頭明列） |
| `5state_aprime_unified / 5state_aprime_kf_meas` | → 函數估測線（powerlaw/expgain） |
| `6state_curvature_hd / 6state_curvature_taylor` | → 2-參數 gain law（6→5 state，關掉導數 regress） |
| `4state_para_c` | → 函數估測線（θ-as-state 概念的前身） |
| `4state_del_hd_thetafit` | UNCERTAIN——C1 theta-fit 線停擺（S1–S9 完成待審後未再推進） |

## archive/ — 歷史凍結

phase1–7 md 推導鏈、`Q66_OL_R22_derivation.md`、`drafts/`（colored_eps、gamma_split）
＝2026-04/05 六-state 時代，不需先讀。
