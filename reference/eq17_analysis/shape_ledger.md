# Shape Ledger — gain-law 形式統一對照

> **定案門檻（freeze gate）**：任一形式要從 co-equal 升為唯一定案，需同時滿足——
> (i) ~~缺陷 2 解決~~ ~~y₂ 慢波動誠實處理~~ **2026-08-01 二改：c2/c3 seed 一致性病根
> 獲誠實處理 = y₁ 通道 ε_w MA(2) 跨步共變異數（#3）**；y₂ 慢波動經 y2_off 消融
> 證明非燃料（見下「未解主項」節）；
> (ii) **margin ≥ 3×**（= bound ÷ √P[0]；目標 sup|θ_eff−1| ~0.01 級，讓指數從
> nuisance 變可辨識參數）；(iii) **三層指標填滿、無 TODO**。
> 達標前 powerlaw 與 expgain 維持**對等並行**（使用者裁決 2026-07-28）。
>
> 新形狀加一欄，跑同一批腳本即可比。指標定義不得為單一形式客製。

更新紀律：形狀、prior、或讀數鏈任何改動後，重跑對應腳本並更新本表（含日期）。

## A 形狀層（離線，`test_script/integration/verify_shape_exponent_bound.m`）

指數定義：Ψ := 1 − a_h/a_o，θ_eff = −dlnΨ/dφ 從精確修正曲線讀回。

| 指標 | taylor 階梯 | powerlaw (p) | expgain (b)（7a=7b 同形狀） |
|---|---|---|---|
| sup\|θ_eff−1\| ⊥ 全域 | n/a（無形狀參數） | **0.034** @h̄=1.217 | 0.094 @h̄=1.424 |
| sup\|θ_eff−1\| ⊥ h̄≥2 | n/a | 0.024 | 0.082 |
| sup\|θ_eff−1\| ∥ 全域 | n/a | 0.994（近壁發散） | 0.778 |
| sup\|θ_eff−1\| ∥ h̄≥2 | n/a | 0.283 | **0.034** |
| √P[0]（prior，兩漸近錨） | n/a | 0.035 | 0.10 |
| margin（⊥） | n/a | 1.03× **TIGHT** | 1.06× **TIGHT** |
| ∥ 軸可用性 | n/a | **FAIL**（Goldman 對數非冪次，無近壁錨） | h̄≥2 可用（margin 2.9×） |

（數字：`5state_expgain_hd.tex` Stage 2b，2026-07-28 版）

## B 濾波層（閉迴路 sim，6-seed，canonical hold→descend→1Hz osc）

| 指標 | taylor 階梯 | powerlaw | expgain 7b | expgain 7a |
|---|---|---|---|---|
| descent peak \|a_hat 誤差\| (z) | —（基準見 level_knives） | TODO | 11.03% | **5.05%** |
| osc 偏差 a_hat/aT (z, 6-seed) | — | 1.0178 | 1.0238 | 1.0220 |
| tracking std (z) | — | 23.77 nm | ~24.2 nm | 24.24 nm |
| a_hat/aT 穩態 (z / x / y) | — | 0.943 / 0.756 / 0.756 | TODO | TODO |
| 參數誠實度（實誤差 ÷ 宣稱 √P） | — | p̂: 0.4 | b̂: 2.2×（過信） | b̂: **0.97×** |
| 跨 seed 散佈（b̂/p̂） | — | TODO | 0.0484 | 0.0272 |

（來源：memory project-powerlaw-r2-whitening-2026-07-27、project-expgain-impl-two-defects-2026-07-28）

## C 前提稽核（yes/no；「數字好看但前提作弊」防線）

| 前提 | powerlaw | expgain 7b | expgain 7a |
|---|---|---|---|
| y₂ 白化（AR(1) 修正） | ✓ production | knob（acov 測試以白化臂 PASS） | knob（同左） |
| F_dh 確定性鏡像（fdet） | ✓ | ✓（drift 測試 PASS） | ✓ |
| per-axis A² 種子 prior | ✗（單一 scalar，x/y 鬆 4×） | ✗（Pf_ao_frac 寫死 0.01；公差鏈應 2.8%） | ✗（同左） |
| a_cov 不變性 | PASS（verify_powerlaw_regress_A12） | PASS 0.02%（verify_5state_expgain_acov） | PASS 0.00% |
| P[0] 預算（V3） | TODO | PASS | PASS |
| run-time c-free | ✓ | ✓ | ✓ |

## 未解主項（讀數鏈共用；2026-08-01 改寫）

**缺陷 2 已撤案（2026-08-01）**：N=48 母體測試 osc 超額 +1.33%（CI 含零）、
hold−osc 配對 −0.44% n.s.——「運動時 +4~5%」是 6-seed 高抽樣假象（慣用組 = 1-in-18，
seed 11 = 母體最大值）。#5 重裁維持（調變機制配對處決 −0.11%±0.11%）、EWMA-lag
定量處決（功率短 25×）。**Cdpmr_Cn 含運動重推無標的，勿再立案。**
詳 memory project-formB-tier1-defect2-retraction-2026-08-01。

**單 run 濾波完備（08-01/02）**：MA(2) 增廣（白 Q 低估 DC 2.17×，log 閉合 0.99988）＋
y₂ 自迴音修正（S=0.32，Lyapunov/配對實測互證）皆 production；innov_y₁ 白化
[0.30,0.24]→[0.003,0.015]、C1 b 預算 2.141→1.610 PASS、N=20 1.963→1.053 PASS、
ŵ_s honesty 1.42→0.93；**效率審計：單 run 0.080 已在 CRLB（界限 ~0.09）**。
撤案帳：「y₂ 凍結偏差 6.1%」= seeds-1:20 幽靈（全新 40 seed corr 0.022）；
「+1.2% 靜態偏差」同死；β_w 推導下沉 archive。
**根治 = 校正鏈（文件 c5）**：後驗遞移 8 runs ±8.9%→±2.0% 貼 √P、作業 run desc
1.69% ≈ 鎖定地板。下一槓桿 = 三軸共享 ŵ_s（÷√3；前置 = ∥ 軸 Form B 錨 9/16 推導，
x/y 現偏 0.6）。osc 窗 15% 線性化殘差維持觀察項。
**讀數鏈數字紀律：6-seed SEM ≈ 2.6%，本表 B 層 1% 級數字不得引用到兩位小數。**

## 形式沿革（一行版）

taylor 階梯（逐階加 state，無限 regress）→ curvature 6-state（a'' 當 state，穩住但仍 regress）
→ **2-參數 gain law**（07-24 powerlaw；07-27 expgain；07-28 7a 代數式修缺陷 1）。
歷史推導在 `derivation/archive/`。
