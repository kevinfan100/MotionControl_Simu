<!-- Promoted from test_script/temp_agent_asstate_verify_report.md on 2026-07-20.
Figures live in gitignored test_results/temp_var_centered_figs/ (AG_T1/T2/T3);
AG_best_combo.png superseded by REC_trace.png (independent re-run of the
recommended config, numbers matched; data test_script/temp_rec_results.mat).
Formal code landing of the REC config goes through the wip/aprime-as-state
merge session, not the temp controller. -->

# a'-as-state (5-state EKF, z axis) 驗證戰役報告

> TEMP (chat 2026-07-19, AG)。純 MATLAB pure-track,scenario = 1 Hz canonical
> (osc, h̄ 22.2→2.0, hold 0.5s → descent 1.0s → osc 4 cyc → **final near-wall hold**, T=6s),
> seed 1-3。共跑 35 sims。相位線 t=0.5 (hold 尾) / 1.5 (descent 尾) / 5.5 (osc 尾)。
> a_nom = 1.4706e-2 μm/pN, R = 2.25 μm, truth a'(h̄=2) = 1.589e-3 μm/pN/μm。
> 資料:`temp_agent_asstate_verify_results.mat` (struct `AG`);
> 圖:`test_results/temp_var_centered_figs/AG_*.png`。
> 未動任何 production 檔;僅 temp controller/driver 加了 `aprime_state_Q55_floor`
> 旋鈕 (預設 0 = no-op,bit-identical 已由 T3a kappa=1 對照確認)。
>
> **Scope 更新 (2026-07-19, 用戶):E1SH(冷啟近壁 pure hold)已剔除** —— 章程下系統只能從
> 遠場 a=a_nom / a'=0 出發、經 descent 抵達近壁,冷啟近壁 hold 非操作條件。所有原 E1SH
> 檢查改用 **1 Hz full-path 的 final near-wall hold 段**(t∈[5.5,6.0],Q55 floor 回復檢查
> 用 cfg.T_sim +2s 延長至 [5.5,8.0])。已確認軌跡在 osc 結束後於底部 h̄=2.0 hold 平坦
> (h_d std = 0)。旋鈕值全部章程合規(a_nom/R/Ts/牆位/軌跡命令);truth 僅用於事後評分。

---

## 總判決 (raw conclusions)

| 項目 | 結論 |
|---|---|
| **負 a' 元凶** | **excitation-onset misattribution(P45 未建立時的暫態),不是 sign bug**。三重確認(見下)。 |
| **最佳 learn-gate** | t0 ≈ 1.25 s(gate 必須在 descent 尾段「弱激勵中」打開,讓 P45 先建;打在 osc onset t0=1.5 反而災難)。 |
| **P0 prior** | 預設 (0.1×a_nom/R)² 太緊,√P0 只有 truth 的 0.41×(低估 2.4×)→ 正是 onset 負值的成因。放大 prior 反而**消除** onset(與假設相反)。final near-wall hold 的 a'/P55/honesty 對 prior **完全無關**(已學過 osc,init 遺忘)。建議 frac ≈ 0.5(√P0 ≈ 2× truth)。 |
| **kappa (Q55 scale)** | 非乾淨的 honesty 旋鈕:↑kappa 讓 honesty 上升(0.085→0.166,仍到不了 [0.5,2])卻**降低** near-wall level ratio(0.72→0.45)、升 tracking。小 kappa(0.5-1.0)最利估計。 |
| **Q55 floor(PLACEHOLDER)** | 概念有效(final-hold 延長段 √P55 回復 ×1.59、osc honesty 0.11→0.34),但任何量級都在 osc 段過度注入 → near-ratio 崩壞且 seed 相依;**且 floor + selfmod 讓 warm final-hold 的 a' 漂到負值(−6.3e-4,錯號)**。**扁平 floor 不可上線**,需改成 excitation-gated。 |
| **推薦組合** | **gate t0=1.25 + prior frac 0.5 + kappa 1.0,floor 關**。cross-seed:near 0.68±0.08、trk 26.3±0.5 nm(= known-arm 26.2,零 tracking 代價)、honesty 0.13。 |

核心限制(所有組合共通):a'-state 只估到「level」,**估不到 intra-cycle shape**(corr_osc ≈ 0 全程),honesty 天生 < 1 因 err_rms 含它結構上追不到的 shape 擺動。這是章程真瓶頸,非本次旋鈕能解。

---

## T1 — 負 a' 診斷

### T1(a) baseline(E1SO state+selfmod, t0=0, seed 1)
- min(a') overall = descent = **−2.893e-3**;osc min = **+3.021e-4**(osc **從不**變負)
- a'<0 窗 = **[0.306, 1.139] s**(全在 descent),frac_descent = 0.62
- â_z descent max = **18.8** nm/pN(known-arm ref 14.0 → 汙染確認,對上 baseline ~18.7)
- osc corr −0.069 | near 0.56 | honesty 0.111 | trk 26.5 nm

### T1(b) learn-gate sweep(seed 1)
| t0 [s] | min(a') desc | a'<0 窗 | frac_desc | â_z desc max | corr | near | honesty | trk | late-desc slope |
|---|---|---|---|---|---|---|---|---|---|
| 0.00 | −2.893e-3 | [0.31,1.14] | 0.62 | 18.8 | −0.069 | 0.56 | 0.111 | 26.5 | +1.38e-3 |
| 0.75 | −4.685e-4 | [0.75,1.13] | 0.31 | 18.5 | −0.070 | 0.56 | 0.111 | 26.5 | +1.27e-3 |
| 1.00 | −5.224e-4 | [1.04,1.11] | 0.07 | 16.3 | −0.074 | 0.55 | 0.110 | 26.5 | +6.49e-4 |
| **1.25** | **0**(無) | 無 | 0.00 | **14.2** | −0.058 | **0.65** | 0.110 | 26.5 | +3.55e-3 |

gate 延後 → 負值單調縮小 + â_z 汙染消失(18.8→14.2 ≈ known 14.0),**且 osc metrics 不變(near 甚至 0.56→0.65)**。

### T1(c) sign-flow sanity
- code 確認:`fe45 = dh_d_step = dot(pd−pd_km1, w_hat)`,是 **SIGNED** step(descent 中 h_d 遞減 → 負);a' = x_e(5,3)。F(4,5) 符號正確。
- 穩定晚 descent(t∈[1.0,1.5])a' net drift **每臂皆正**(late-desc slope 全 >0,+3.55e-3 to +6.49e-4)→ P45 建好後方向正確,**無 sign bug**。

### T1(d) cross-seed(seeds 2,3)
| t0 | seed | min(a') desc | a'<0 窗 | frac_desc | â_z desc max | corr | near | honesty | trk |
|---|---|---|---|---|---|---|---|---|---|
| 0.00 | 2 | −1.802e-3 | [0.00,1.20] | 0.70 | 19.9 | 0.262 | 0.60 | 0.126 | 26.6 |
| 0.00 | 3 | −1.091e-4 | [0.02,1.19] | 0.09 | 14.2 | 0.140 | 0.69 | 0.133 | 25.7 |
| 1.25 | 2 | −3.608e-4 | [1.25,1.28] | 0.02 | 14.2 | 0.261 | 0.72 | 0.134 | 26.7 |
| 1.25 | 3 | −1.851e-5 | [1.25,1.26] | 0.00 | 14.1 | 0.142 | 0.76 | 0.132 | 25.7 |

負值**位置** seed-robust(永遠困在 descent 窗),**量級** seed 相依(noise-driven 暫態);gate 對所有 seed 都移除負值 + â_z 汙染。t0=1.25 殘餘只有 gate 打開瞬間(1.25s)1-2 sample,可忽略。

### T1 判決證據鏈 → **excitation-onset misattribution,非 sign bug**
1. 負值**只**出現在 descent onset 窗;osc 全程 min > 0。
2. 延 gate 單調消負值 + 消 â_z 汙染,osc 品質不變/改善。
3. 晚 descent net drift 正向(方向正確,一旦 P45 建立)。
4. seed-robust(位置固定、量級隨 noise)。
5. **反向確認**(gate1.5,見 T3 尾):把 gate 打在 osc onset,同一暫態在 osc onset **更大**重現(min −1.9e-2,near 崩到 0.24)→ 證明是通用「激勵突起 + P45 未建」機制,與 descent 無關。

---

## T2 — P0 prior sweep(與 T1 交互)

truth a'(h̄=2) = 1.589e-3;a_nom/R = 6.536e-3。√P0 vs truth:0.1×→0.41×、0.5×→2.06×、1.0×→4.11×(**預設 0.1× 低估 truth 2.4×**)。

### T2 osc(seed 1,gate × frac)
| gate | frac | √P0 | ×truth | min(a') desc | â_z desc max | corr | near | honesty | trk |
|---|---|---|---|---|---|---|---|---|---|
| 0 | 0.1 | 6.54e-4 | 0.41 | −2.893e-3 | 18.8 | −0.069 | 0.56 | 0.111 | 26.5 |
| 0 | 0.5 | 3.27e-3 | 2.06 | **+9.4e-5**(無負) | 16.0 | −0.074 | 0.55 | 0.110 | 26.5 |
| 0 | 1.0 | 6.54e-3 | 4.11 | **+8.6e-5**(無負) | 13.9 | −0.074 | 0.55 | 0.110 | 26.5 |
| 1.25 | 0.5 | 3.27e-3 | 2.06 | −2.4e-4 | 14.2 | −0.060 | 0.61 | 0.111 | 26.5 |
| 1.25 | 1.0 | 6.54e-3 | 4.11 | 0(無負) | 14.2 | −0.062 | 0.63 | 0.110 | 26.5 |

**關鍵(與假設相反)**:放大 prior **不會**惡化 onset —— 反而**消除**它(gate-off 下 frac 0.5/1.0 皆無負,â_z 汙染 18.8→16.0→13.9)。onset 負值是「prior 太緊 → overconfident」的產物,不是 gate 缺席。osc 穩態(corr/near/honesty/trk)prior-無關(init 被遺忘)。

### T2 final near-wall hold(章程替代 E1SH:1 Hz full-path 尾段 [5.5,6.0],gate 1.25,seed 1)
> E1SH(冷啟近壁 hold)已剔除;改看系統經 descent+osc **抵達**近壁後的 hold —— 此時 a'-state
> 已在 osc 學過,帶著學到的值進 hold(非冷啟凍結於 0)。truth a'(h̄=2)=1.589e-3。
| prior frac | final-hold a' | ×truth | √P55 (final-hold) | final-hold honesty |
|---|---|---|---|---|
| 0.1 (default) | 8.51e-4 | 0.54 | 4.92e-5 | 0.07 |
| 0.5 | 8.37e-4 | 0.53 | 4.93e-5 | 0.07 |
| 1.0 | 8.46e-4 | 0.53 | 4.92e-5 | 0.07 |

final near-wall hold 的 a'、P55、honesty 三者 **對 prior 完全無關**(三 arm 幾乎逐位吻合)→ 近壁
hold 已把 init 遺忘,穩定在 **0.53× truth**(低估 ~2×)、honesty **0.07**(overconfident)。這是章程下
真正相關的 hold honesty(取代 E1SH 冷啟的 √P0/truth 假象);且 **prior 修不了它**(穩態遺忘 init)—— 只有
Q55 floor 之類能開 P55,但扁平 floor 有副作用(見 T3b)。

### T2 判決
預設 prior 低估 truth 2.4× → overconfident → **正是** onset 負值成因;放大 prior(frac ≥ ~0.24 即 √P0 ≥ truth)
消除 onset。osc 與 final-hold 穩態皆 prior-無關(init 遺忘)。**建議 frac 0.5**(√P0 ≈ 2× truth:onset
乾淨、warm-hold honesty 不因它變差、osc 不變)。

---

## T3 — Q55 scale + floor

### T3(a) kappa sweep(E1SO, gate 1.25, seed 1;kappa=1 = floor edit 的 bit-identical 對照,完全重現 T1b t0=1.25)
| kappa | corr | near | honesty | trk | √P55 osc mean | err_rms |
|---|---|---|---|---|---|---|
| 0.5 | −0.043 | **0.72** | 0.085 | 26.4 | 3.83e-5 | 4.52e-4 |
| 1.0 | −0.058 | 0.65 | 0.110 | 26.5 | 5.65e-5 | 5.15e-4 |
| 2.0 | −0.075 | 0.55 | 0.133 | 26.7 | 8.49e-5 | 6.37e-4 |
| 4.0 | −0.068 | 0.45 | **0.166** | 27.0 | 1.35e-4 | 8.15e-4 |

tradeoff:↑kappa → Q55↑ → P55 不易塌 → honesty↑(但頂多 0.166,達不到 [0.5,2]),代價是 a' 變噪 → near-ratio↓ + err↑ + trk↑。corr≈0 全程(不追 shape)。**kappa 不是乾淨的 honesty 旋鈕;小 kappa 最利 level 估計**。

### T3(b) Q55 floor(PLACEHOLDER,P0=frac0.5=1.068e-5,floor = P0·Ts/T_forget,Ts=1/1600)
> floor = P0·Ts/T_forget:T_forget=2s → **3.337e-9**;T_forget=10s → **6.675e-10**(章程合規:P0/Ts/T_forget
> 全用 a_nom/R/Ts,無 truth)。E1SH 已剔除;hold 回復改用 1 Hz full-path 的 **延長 final near-wall hold**
> (cfg.T_sim=8s → hold 段 [5.5,8.0],軌跡在底部 h̄=2.0 平坦,h_d std=0 已確認)。

**osc 段品質**(E1SO gate1.25 frac0.5 kappa1,T_forget=2s floor):
| | corr | near | honesty | trk |
|---|---|---|---|---|
| no-floor | −0.060 | 0.61 | 0.111 | 26.5 |
| floor | +0.078 | **0.38** | **0.337** | **29.6** |

**延長 final near-wall hold [5.5,8.0]**(warm hold,a' 已學過;T_forget=10s floor,seed 1):
| | √P55 5.6s → 7.9s | 成長 | hold a' | hold honesty | hold trk |
|---|---|---|---|---|---|
| no-floor | 5.22e-5 → 4.03e-5 | ×0.77(**續塌,不回復**) | +8.91e-4(0.56× truth) | 0.06 | 21.9 |
| floor | 4.92e-4 → 7.81e-4 | **×1.59(回復)** | **−6.29e-4(錯號!)** | 0.34 | 21.3 |

floor **達成 P55 回復設計目的**(warm hold √P55 由續塌 ×0.77 轉為回復 ×1.59;osc honesty 0.11→0.34),
但兩個副作用:(1) T_forget=2s 在 osc 段**過度注入** → near 0.61→0.38、trk 26.5→29.6;(2) floor + selfmod
的 dxhat3 熱耦合讓 warm final-hold 的 **a' 漂到負值 −6.3e-4(錯號)**——inflated P55 給 KF 在 hold 上追噪聲的
license。**扁平 floor 不可上線**。

### T3(c) best combo + floor cross-seed(gate1.25 frac0.5 kappa1 + **較緩** floor T_forget=10s = 6.675e-10,seeds 1-3)
| seed | min(a') desc | corr | near | honesty | trk | P55 回復 |
|---|---|---|---|---|---|---|
| 1 | −3.24e-4 | −0.038 | 0.22 | 0.259 | 29.1 | ×2.07 |
| 2 | −1.56e-3 | 0.370 | 0.76 | 0.270 | 27.3 | ×1.94 |
| 3 | −4.57e-4 | 0.218 | 0.59 | 0.413 | 26.1 | ×1.90 |
| **mean±std** | −7.8e-4±6.8e-4 | +0.18±0.21 | **0.52±0.27** | 0.31±0.09 | 27.5±1.5 | ~×2 |

即使緩到 T_forget=10s,floor 仍讓 near-ratio **不穩且惡化**(0.22–0.76,spread ±0.27)。honesty 與 hold-forgetting 有改善,但代價是 osc level 估計崩壞。

### T3 判決
kappa 與 flat-floor 都用同一機制(注 Q55)換 honesty,都在 osc 段傷 level 估計。floor 概念驗證成立但**扁平量級不可上線**;正解是把 floor 改成 **excitation-gated**(只在 \|dh_d\| 小的低激勵段開),讓 hold 期 P55 重開而不污染 osc 學習(留給 derivation)。

---

## 推薦組合(production,無 floor)cross-seed(seeds 1-3)
**gate t0=1.25 + prior frac 0.5(√P0 ≈ 2× truth)+ kappa 1.0,floor 關**
| seed | min(a') desc | corr | near | honesty | trk | â_z desc max |
|---|---|---|---|---|---|---|
| 1 | −2.43e-4 | −0.060 | 0.61 | 0.111 | 26.5 | 14.2 |
| 2 | −1.53e-3 | 0.269 | 0.67 | 0.133 | 26.7 | 16.0 |
| 3 | −4.57e-4 | 0.140 | 0.76 | 0.132 | 25.7 | 14.1 |
| **mean±std** | −7.4e-4±6.9e-4 | +0.12±0.17 | **0.68±0.08** | 0.13±0.01 | **26.3±0.5** | ~14.8 |

trk 26.3 ± 0.5 nm = known-arm ref **26.2**(零 tracking 代價);near-ratio 穩(0.61-0.76);â_z descent 汙染基本清除。**唯一殘留**:gate 在 1.25 (< descent 尾 1.5) 打開,有 1-2 sample 的 gate-open 小殘餘負值(worst seed2 −1.5e-3),不影響 osc。

---

## Caveats
1. **Q55 floor = PLACEHOLDER**(`aprime_state_Q55_floor` = P0·Ts/T_forget,T_forget 純猜)。量級未推導;扁平形式證實(a) 傷 osc near-ratio、(b) 讓 warm final-hold 的 a' 漂到負值,不可上線。應改 excitation-gated(只在 |dh_d| 小時開)。已加旋鈕(temp controller/driver),預設 0 = bit-identical no-op。
2. **corr_osc ≈ 0 全程**:a'-state 只估 level,不追 intra-cycle shape。這是為何 known-arm 仍勝(A2 判決),與 Mac 07-14 結論一致。honesty < 1 部分是結構性(err_rms 含追不到的 shape)。近壁 warm hold 的 a' 穩定低估 ~2×(0.53× truth)—— 這是章程下 a'-as-state 的誠實效能底線。
3. **gate 時機是雙面刃**:必須在 descent 尾「弱激勵中」打開(t0≈1.25)讓 P45 先建;打在 osc onset(t0=1.5)反使 onset misattribution 在 osc 更大重現(near 0.24,min −1.9e-2)。不要把 gate 設在 ≥ 1.5。
4. **prior 是比 gate 更根本的 onset 修法**:frac ≥ 0.5 在 **無 gate** 下即消 onset 負值;gate 只是輔助(且需正確時機)。若想去掉 gate 複雜度,單靠 frac 0.5-1.0 prior 亦可(gate-off frac1.0 seed1:無負值、â_z 13.9)——但目前只有 seed1 gate-off 資料,cross-seed 未跑。
5. **E1SH(冷啟近壁 hold)已剔除**(用戶裁定非操作條件);hold 行為改用 1 Hz full-path 的 final near-wall hold 段(warm,a' 已學過),延長段用 cfg.T_sim=8s。此為 charter-consistent 探測方式。
6. 全部 seed 1-3,pure-track;Simulink SSOT 未驗。35 sims。所有旋鈕值章程合規(a_nom/R/Ts/牆位/軌跡),truth 僅事後評分。
