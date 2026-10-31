# 目前主線快照：三軸估測器 + 牆角度 + 曲面掃描（2026-10-15 起）

每 session 自動載入，只放「現在在哪、從哪接」。細節在 memory `project_sphere_turn_flow_2026-10-29`
與各日的 project 條目；一維 gain-law 線（powerlaw / expgain / Form B / F2 / c-only）見 `mainline-gainlaw.md`（需要時才讀）。

## 估測器與 runner

- runner：`test_script/scratch/run_t3_xyz_fixed.m`，local arm（`opts.form = 'local'`）；所有新選項預設關。
- 狀態 18 槽：δp 鏈（1–9）、â_11、â_33、â_uw、â_vw、â′_11、â′_33、θ_x、θ_y、κ（曲率，選用）。
- 角度由非對角增益讀出（文件 `1025_computed_estimated_angle.tex`、`1026_local_angle_estimator.tex`、`1029_local_gain_small_angle.tex`）。
- 亂數：`rng(seed, opts.rng_gen)`，預設 threefry（單獨跑 = parfor）。

## 現行流程（40 µm/s 繞球一圈，10 seeds 不撞牆）

停 10 s（15 µm）→ 沿 ŵ 等速下降 5 s（或 1 s）→ 停點開 regulation（τ_a 0.5 s）等 1 s → 掃描
（速度 0.5 s 漸增、κ̂ 到掃描才估、角度 P_fix 2°、斜率用報告的 P_fix）。
mean|ā_33 − ā_d| 0.019（舊流程 0.080）。不等 1 s 可改 PI（e 首次過零才積分，τ_i 0.5–1 s），0.023。

## 整顆玻璃珠（10 µm/s 完成，3/3 seeds）

牆 `fillet`（珠半徑 5 µm + 補角 ρ 10 µm + 玻璃）、ā_d 0.1、柵狀 27 線 ±13 µm、`raster_keep_y`、
κ̂ 可變（`kappa_pfix` 0.3/R）。重建半徑 5.00–5.01 µm。腳本 `run_1029_fillet_opts.m`、`plot_1030_bead_meng.m`。

## 開放（從這裡接）

1. 40 µm/s 時 â′_33 被 y₁ 法向分量拉壞（為負 38%）；控制靠斜率下限 max(â′, (8/9)(1−â)²) 撐住。
   已否證：曲率＋閉迴路、法向力放大（P0）、fx(7:9) 少的已知項 m。診斷開關 `k_ap_cut='y1'` 有效但無推導。
2. 40 µm/s 掃不完整顆珠子（凹凸轉換太快）。
3. κ̂ 的 √P_fix = 0.3/R 要寫成假設並推導；ā_33 讀數高速偏高 +0.012～0.020 來源未明。
