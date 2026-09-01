# 正規化邊界規約（2026-08-30，formC/formB 家族）

> 來源：Jones 問「Q33 = 4k_BT a_o ā/R 的分母該不該是 R²」→ 三方對帳（tex↔code↔plant）＋
> V1–V4 實測全 PASS（ledger L42–L45）。本檔規定「必須怎麼做」；量到什麼寫在 memory 與 ledger。

## 三個尺，不是兩個

```
物理            Δp  = a · f                    [um] = [um/pN]·[pN]
① 長度 /R       Δw̄ = (a/R) · f
② 增益 /a_nom   Δw̄ = (a_nom/R) · ā · f        ← 剩一個有因次係數 a_nom/R = a_o [1/pN]
③ 力  ×a_o      Δw̄ =  ā · f̄ ,  f̄ = a_o f = f/f_R ,  f_R = γ_N R/Δt = 1/a_o ≈ 153 pN
```

| 符號 | 定義 | 單位 | house 值 | code |
|---|---|---|---|---|
| a_nom | Ts/γ_N，物理遠場增益（driver 用名） | um/pN | 1.471e-2 | `run_formC_b.m` `a_nom_drv` |
| a_o | Ts/(γ_N R) = a_nom/R，R-正規化遠場增益 | 1/pN | 6.54e-3 | `formC_b.m:357` |
| f_R | 1/a_o，遠場一步走一個 R 的力 | pN | 153.0 | 隱含 |
| a_disp | a_o·R = a_nom，顯示尺 | um/pN | 1.471e-2 | `:360` |
| κ_T | 4k_BT a_o/R = 4k_BT a_nom/R²，遠場一步布朗 Var/R² | [-] | 4.98e-5 | `:359` |
| σ²_{n_w} | meas_noise_std²/R² | [-] | — | `:355` |
| ξ | C_n σ²_{n_w}/(C_dpmr κ_T)，量測雜訊的等效增益 | ā 單位 | 1.53e-2 (z) | `:423` `xi_bar` |

**a_o 已含 1/R，所以 Q33 的分母是 R 不是 R²。** 若改用 a_nom 寫，同一量是 4k_BT a_nom ā/R²。

## 五個跨界點位（`model/controller/motion_control_law_formC_b.m`）

| 點位 | 方向 | 行 | 做什麼 |
|---|---|---|---|
| U0 | init 一次 | `:355, :357, :359, :360` | σ²_n/R²、a_o、κ_T、a_disp |
| U1 | 每步入口 | `:780, :996, :1000, :1013–1016, :1050` | 所有長度 ÷R |
| U1′ | 可選入口 | `:981` | a_ctrl_override [um/pN] ÷ a_disp |
| U3 | 每步出口 → plant | `:1075` | f_d = f̄_d ÷ a_o [pN]（**唯一回 plant 的解正規化**） |
| U4 | 每步出口 → log | `:922–937, :1517–1591` | ×a_disp、×R、×R²；legacy ×a_o 給 1/pN |

迴圈算術（Jacobian、Q、R、predict、update、反演）裡沒有任何物理常數；k_BT、γ_N 只在 U0 出現一次；
R 每步只在 U1 門口把 um 換成 R 單位。plant（`calc_gamma_inv`、`calc_thermal_force`）與 driver 不認識 a_o。

## 必須怎麼做

1. **動 a_o 的定義 = 同一個 edit 裡同動 κ_T 的 R 次方與 U3 的除法。** 只改 `:357` 一行會靜默全錯
   （κ_T ×2.25、y₂ 讀數 ÷2.25、出力 ÷2.25、顯示 ×2.25），run 照跑完、數字看起來合理。
2. **新增任何跨界量，先歸到 U0–U4 其中一格再寫。** 不准在迴圈中段出現 R、a_o、k_BT、γ_N。
3. **legacy `×a_o` 顯示（`a_hat_nd`、`a_hm_nd`、`a_prime_hat`）不得與 `a_disp` 顯示混在同一張圖或同一張表。**
   新 log 一律用 a_disp（um/pN）或直接存 ā。
4. **改任何邊界後必跑 V1**：`test_script/scratch/l4_norm_v1_unit_invariance.m`（um→nm 換單位制，
   影子 `physical_constants`），判準 = 無因次 log 相對差 < 1e-9、長度／增益 ×1e3、力 ×1。
   漏掉哪一族就是哪個點位在漏。
5. **κ_T 的 4 是 plant 單邊積分慣例**（`calc_thermal_force.m:74`），不是教科書 2k_BT；要對 plant，
   不要對教科書（V2：`l4_norm_v2_kappaT_plant.m`）。
6. **bar／hat 慣例**（tex 與圖共用）：bar = 已除以尺（w̄、ā、f̄）；無 bar = 物理量或本來就無因次的量
   （b、λc、C_dpmr、ξ、n_w）；hat 只戴在無因次量上（ā̂、b̂、δw̄̂）。run-time Q33/Q44/R2/y₂ 全在 ā̂ 取值。
   圖軸細則見 `figure-style.md`「圖中 notation」節。
7. driver 側 a_nom 只做比值（ā_true = a/a_nom、dā/dw̄ = a′/a_nom），R 在比值中消掉；**不要**把 a_nom 塞進 controller，
   也不要把 controller 的 a_o 拿去除物理增益。

## 為什麼這樣設計（一句話）

正規化把「物理」壓到 5 行邊界，讓迴圈裡只剩純數與估測量——這同時給了 c-free 的結構保證、
零 tuning 的 prior（錨 9/8、√P[0] 與粒子無關）、可 grep 的單位審計、和推導與 code 的逐字對應（ā·f̄ ≡ a·f）。
