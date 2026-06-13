# Gain Compare Round 2 Findings — gate-free 100-seed 生產批

Date: 2026-06-12
Branch: `test/motion-test` @ 7483804
Data: `test_results/gain_compare/{f1Hz,f5Hz,f10Hz}/`（各 100 seeds × 4.0 s、h̄_safe = 1 = G3 永不觸發、suppress_xD 兩臂、CRN 配對）
Design: `gain_compare_design.md` §12；Plan: `gain_compare_plan_round2.md`
對照組: Round-1 gated 20-seed（`test_results/gain_compare/`，h̄_safe = 1.5、T = 7 s、{1,2,5} Hz）

---

## F1. 穩定性邊界：gate-free â 控制在 1 與 5 Hz 之間斷崖

| 頻率 | a=a_true（true-gain control）存活 | a=â（â）noisy 存活 | a=â det run |
|---|---|---|---|
| 1 Hz | 100/100 | **100/100** | 發散（見 F5） |
| 5 Hz | 100/100 | **0/100**（\|e\| 2.8–4.3 μm） | 發散 |
| 10 Hz | 100/100 | **0/100**（\|e\| 3.4–4.2 μm） | 發散 |

- 發散型態全部是「跟丟」（粒子留在低處跟不上向上軌跡，min h̄ ≈ 1.18），**無真實撞牆**（h̄ 全程 > 1）。
- 對照：Round-1 gated 在 5 Hz 20-seed 0 發散 → **失穩完全由拿掉 G3 造成**。
- D4 的三結局判定為**混合型**：低頻 gate 是累贅（F2）、高頻 gate 是穩定性剛需。

## F2. 1 Hz 近壁 â 精度：G3 蒙蔽佔近壁高估的一半

z 軸 â ensemble-mean rel-err（± 跨 seed SEM）：

| 窗 | Round-1 gated | Round-2 gate-free |
|---|---|---|
| desc | — | +18.9 ± 0.3% |
| 近壁（near, 谷） | **+41.9%** | **+19.6 ± 1.8%** |
| 遠壁（far, 峰） | −12.1% | −13.9 ± 0.6% |
| osc 全窗 | +0.8% | −5.9 ± 0.7% |

- gate-free 後 â 全程吃得到 y₂，**跟著 a_true 下到谷底**（fig_gain_compare 可視），近壁高估砍半；殘餘 +20% 屬估測器本身近壁極限（IIR lag + chi-squared floor 家族）。
- 代價：恢復段 lag/undershoot 使 osc 與 far 出現 −6 ~ −14% 偏低（gated 時 osc 接近 0 是「谷底外插高估 + 恢復不及」的互抵假象）。

## F3. 控制層代價（1 Hz, gate-free, z 軸 paired B/A）

- det（系統性）：B 臂 δz_det 下降谷 −66.9 nm、osc A_e 6.4 nm（det_v2，ensemble 參考）；A 臂 ≈ 0（萃取殘餘 ~10 nm 峰值水準）。
- ram（隨機）：paired ratio osc **1.00 ± 0.00**、近壁 near **1.10 ± 0.01**、desc 1.04 — 隨機代價只在近壁，且比 gated 的 1.13（20-seed）略低。
- 熱力理論驗證（V1 閉式 along a_pd）：x full-span meas/theory A 0.993 / B 0.992；z osc norm-var A 0.997 / B 1.040。

## F4. A1 — Q55 閉式近壁動態首驗（a=a_true，三頻率全過）

Level: Var(a_ram) vs C_δx·(a·K_h/R)²·4kBT·a_z；Increment（= Q55）: Var(δa_ram) vs [2/(1+λc)]·(a·K_h/R)²·4kBT·a_z（跨缺口差分已修正，gap-safe）。

| 頻率 | 窗 | level meas/th | incr meas/th |
|---|---|---|---|
| 1 Hz | near | 0.999 ± 0.013 | 1.022 ± 0.007 |
| 5 Hz | near | 1.019 ± 0.011 | 1.024 ± 0.006 |
| 10 Hz | near | 1.002 ± 0.011 | 1.022 ± 0.007 |
| （osc/far 各頻率） | | 0.99–1.02 | 1.01–1.02 |

- **閉式準到 ~2%**，h̄ 低至 1.2、quasi-static 撐到 10 Hz。對照 LF-drift audit：舊 random-walk 增益模型近壁高估 LF 變異 237×。Q55 閉式取代 heuristic 的數據基礎完備。
- 殘餘 +2% increment 偏高為一致性小偏差（三頻率同值），候選原因：理論用 a_pd 而實際 ram 沿真實位置、deflation 高階項。量級不影響用途。

## F5. a=â det run 發散機制（已驗證，1 Hz probe diag）

無噪聲 run 中 Guard 2 並非全程 latch：z 軸僅 87% 時間關閉，13% 時間 y₂ 開啟吃進無統計意義的 a_xm（IIR prefill 初值 + 確定性殘留推高 σ̂²_δxr）→ â/a_true 在 0.30–1.72 間擺動 → 誤差在下降末端（首次抵達 h̄ 1.2）尖峰 ~1 μm。Round-1 時 G3 恰在近壁段強制關 y₂ 兜底。**B det run 已無任何分析依賴**（thermal/ahat/圖全部改 ensemble 或 a_pd 參考），保留它作為免費診斷。

## F6. 設計意涵（後續方向）

1. **G3 不能單純刪除**——刪除條件是補上頻寬感知的保護：soft gate（R₂(h̄) 平滑上升取代硬開關）或近壁估測重設計。
2. **Q55 閉式可直接進 EKF**（F4 完成驗證），與 LF-drift 根因處方一致。
3. 5/10 Hz 失穩根源指向 IIR 頻寬（a_xm 跟不上 gain 調變）——a_cov 提速或替代增益通道是高頻方向。

## F7. 待辦 / 選項（未做）

- cycle folding 版 motion_var（單週期 var 剖面，再降噪 3–4×）
- Welch PSD A/B 頻段歸屬（design §6.5 接口）
- merge `test/motion-test` → `feat/eq17-6state`（使用者 review gate）
- 工程教訓：3×MATLAB 平行 100-seed 在 24 GB RAM 頂滿（collect_diag 全程駐留、尾端一次存檔），實跑 ~40 min；未來大批次 2-way 平行或分段存檔。
