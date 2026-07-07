# C1/C2 Unknown-c(h̄) Gain Estimation — 推導審計與研究計畫

> 產生:2026-07-06,multi-agent audit(5 lens × 對抗式驗證,45 findings 全數 CONFIRMED、合併 27 項)。
> Scope:牆面位置已知(h̄_d/Δh_d/h̄_meas 可用)、drag law c(h̄) 解析式未知;目標 â_x 接近 known-c 基線(z 0.4%/24 nm,budget ~1%)。
> 架構:C1 = 4state_del_hd + 外掛 theta-fit slope supplier;C2 = 整合 para-c(b970a65)。
> 狀態:§5 #1 已裁決(2026-07-06):**T1 雙分支** — RW 與 AR(1)+theta-fit level 兩個底座都推導,L2/L3 ladder 各加一階,以實測裁決。其餘未決見 §5。

# C1/C2 Unknown-c Gain Estimation 審計總結報告

45/45 gap 全數 CONFIRMED(0 整項被推翻)。多 lens 重複發現合併後共 27 項。所有數字皆經獨立 python3 重算(真 c(h̄) 多項式取自 `model/wall_effect/calc_correction_functions.m` L75-116)並比對檔案原文;本報告引用之行號已於合成階段抽查覆核(4state_del_hd.tex L225-275、4state.m L300-349/L525-558、4state_para_c.tex L140-222、para.m L236-274/L344-363、run_pure_simulation.m L99-104/L197-201)。

---

## 1. 切入角度(最終版)

**單一完整描述**:supplier 曲線 â(h̄;θ) 進入 controller 有兩條注入路徑 — **level**(供 AR(1) reversion anchor 與 a_ctrl)與 **slope a′**(供 FF、Q44、R22 delay-leak)。每條路徑的誤差由四個物理通道產生,再經兩層傳遞落到 a_hat:(一)carrier 層 — 該項統計由哪個 Q/R entry、bridge、或 offline bias 表承載;(二)閉迴路層 — a_hat→a_ctrl→σ²_δx→a_xm 的 self-consistency fixed point。

四通道(經審計精煉):

1. **structure(基底形式誤差)**:必須同時以 level(在 h̄_d 逐點)與 slope 計分。審計前 scope 只計 slope 是本次最大盲點 — 擁有 0.4% baseline 的 AR(1) 架構把 level 以傳遞係數 T=(1−K)(1−λc)/((1−λc)+K·λc)=0.99–1.00 幾乎原封送進 a_hat(G1)。z pole 基:slope 佳(1.4–1.9%)但 level 在 trough −4.2%;x,y 1/h̄ 基:level、slope 皆壞(u² 結構項,G2)。
2. **parameter(e_θ)**:1600 Hz 下 P_θ 快速塌縮,殘餘噪聲小;真正的 parameter 風險是**凍結**(no-forgetting RLS / Q_θ=0)把 band-dependent θ*(band) 變成 bias(C1-10、C2-02),以及 Var(e_θ) 不可溯源(R_RLS 手設)。
3. **regressor(h̄ 輸入誤差)**:三個子項 — 決定性 d-step 延遲(已知 Δh_d 可免費橋接)、Brownian gap(不可約;meas-h̄ vs desired-h̄ 近乎 wash)、sensor noise K_h·σ_n/R(白)。外加 a_xm 的 EWMA 窗心 19 步 lag(奇部對稱軌跡相消、偶部即 h-blur)。
4. **measurement(a_xm 鏈)**:β(h̄) scale bias(~63% 可由 EWMA h-blur 二階項推導,G3)、chi-squared floor 44–47%(R22 承載)、IIR lag(同窗心)。

**Frame 外洩漏(審計新增,frame 需擴充四點才完備)**:

- **(a) bias/variance 分流規則**:Q/R 只能承載 white/variance;決定性項(e_K·Δh_d ramp、stale a_ctrl、frozen-θ level bias、β)硬塞 Q 是無效 carrier(white-equivalent ≤6.4e-5 Q44,C1-02),且 frozen 絕對形式有 121f86a 級遠場過脹風險(38–65×,C1-13)。決定性項的 carrier 只有三種:bridge(可補償)、offline bias 表(不可補償)、gate。
- **(b) control-coupling 是乘法層**:closed-loop fixed point(收縮,增益 0.41)把所有進 y2 的靜態 bias 放大 1.38–1.69×(C1-08)。budget 必須以放大後值計:靜態 a_xm bias >0.6% 即爆 1% budget。
- **(c) 計分可觀測性是 frame 的一部分**:band [1.5,3.33] + true-h̄ 計分 + gate 對齊,使 trough(27% cycle)既不訓練、不計分、卻每步被 a_ctrl 使用(C2-02、CF-1、C2-04)。「誤差不可見」≠「誤差不存在」。
- **(d) scope/合規類**:真 c(h̄) 殘留讀取(INV-1)與 infra 缺口(DRV)不產生數值誤差,但使「no calibrated wall model」宣稱對兩架構目前皆不成立。

45 項發現全部可歸入 4 通道 + (a)–(d);無漏網。

---

## 2. Gap 總表

合併規則:同一發現多 lens 命中者併為一列(括號列被併 ID),取最強證據、保留全部引註;materiality 取成員最高。

| ID | 標題 | arch | channel | 量級(z, vs 1% budget) | verdict | 一句話修法 |
|---|---|---|---|---|---|---|
| G1 (=C1-01+CR-1) | C1「slope-only 3 處」scope 不成立:0.4% baseline 屬 AR(1) 變體,reversion 需曲線 LEVEL(第 4 個 c-site),level 誤差 ~1:1 進 a_hat;RW-exact 底線 9.4% | C1 | structure | 傳遞 T=0.99–1.00;z trough level misfit −4.2〜−4.65%(4× budget);xy 至 −18.5%;RW 底線 9.4%(10× budget) | CONFIRMED·高 | 重定 scope 至 AR(1)(4state_del_hd_ar1.tex),theta-fit 同時供 level+slope;supplier 以 level@h̄_d 計分 |
| INV-1 (=CR-2+C1-04+G4+C2-07) | 真 c(h̄) 殘留讀取全清單:「3 處」低估 — sigma2_dh(c_⊥ 值讀取,跨軸)、Q33_randgain、init、4 個 optional flag 亦讀真 c;兩架構現有全部結果皆 known-c-assisted | both | other(scope) | 逐項 <0.4% of carrier(數值可忽略);方法論 100%(headline 宣稱目前不成立);錯接 sigma2_dh 自軸失效模式 = 近壁 K44 +76%/a_hat std +8% | CONFIRMED·高 | de-cheat 順序:a_det → 每步 K_h/c_⊥ → init;C2 換 model-implied K_h=−θφ²/c;C1 doc 逐站列替換源;use_deblur 禁用待分析 |
| G3 | a_xm 之 β(h̄) bias 為 h̄/軌跡相依,~63% 可由 EWMA h-blur 推導(full kernel trough +6.9% vs 實測 +11%);無任何推導文件;base 的 use_deblur 解藥未移植 para | both | measurement | trough +5〜7%(3–7× budget),h̄~2 後 <0.5%;常數 θ 吸收後近壁殘餘 1–3pp;constant-β 假設下 level +11.8%/a′ 8.1% | CONFIRMED·高 | 寫 β 推導(二階 Taylor + full kernel);de-blur port 到 C2/theta-fit(exogenous h̄_d 版);殘餘 β 進 y2 反演修正,不進 R22(是 bias 非 variance) |
| C1-08 | a_xm 靜態 bias 被 a_ctrl self-consistency loop 放大 1.38–1.69×(λ_eff=1−g(1−λc), m(g)=C_dpmr(λ_eff)/C_dpmr(λc));fixed-point 節在任何文件皆缺 | both | control-coupling | dm/dg|₁=−0.41(收縮,不發散);β=+0.11 → a_hat +16〜20%(若近壁 y2 未 gate);>0.6% 靜態 bias 即爆 budget;λc=0.9 時放大 3.2× | CONFIRMED·高 | 寫 fixed-point 推導節(λ_eff、m(g)、收縮證明、放大係數);量測 β(h̄>1.5)(目前無資料);budget 以放大後值填 |
| CF-1 (=C2-03+C1-15) | 近壁 gate tex/code 矛盾:4state_para_c.tex L214 稱「no near-wall h̄ gate」,code L361 有 G3(h̄<1.5)且 2.4% headline 即 gated 產出;gate 期間 θ 仍經 y1 路徑活更新(未分析);此矛盾污染 C1 的 gate 決策 | C2(牽動 C1) | parameter | 27.5% cycle y2 樣本被丟;gated band 含 ~53% θ Fisher 資訊;θ̂_z spread 8.6% → trough a 誤差 7–8%;tex 所述估測器從未被執行過 | CONFIRMED·高 | 裁定:code 為真相,tex L214 更正;2.4% re-caption 為 gated+band-limited;gate 去留以 β 暴露 trade-off 決策(見 T3-7) |
| C2-02 | Q_θ=0 使 θ̂ 凍在 info-weighted band 折衷;結構 misfit 化為 h̄-dependent level bias,最壞在 trough 且被計分 band 遮蔽(band 同時套在 a′-RMS 與 level 指標) | C2 | structure | z trough level −6.3%、xy +38%(g=1.07/0.72,穩定);27% cycle 不計分不訓練但 a_ctrl 每步用;in-band 僅 −0.07〜+0.38% | CONFIRMED·高 | 全訪問 h̄ 範圍計分 + trough-bias 列;xy 先換基底(T2)再談 Q_θ;若引入 Q_θ>0 需棄常數前提之全套重推 + 121f86a gate |
| G8 | z pole 基:a′ misfit ~2%(文件寫 ~1%)但未記錄的 LEVEL misfit 在 trough −4.20%(gated fit −4.65%);文件只報 a′ 數字 | both | structure | −4.2% @h̄=1.2(2.7–4.7× budget,無任何 1-param 權重可低於 −2.7%);局部 h̄<1.7;僅 level 計分可見 | CONFIRMED·高 | 每個 basis 研究強制報 level@h̄_d 逐點表;評 2-param pole+regular(collinearity 0.936 需正則化)或交由 β-corrected a_xm anchor 拉回(K 不可餓死) |
| C1-03 | G3 gate 通過 trough:y2 關閉 383 步/pass,slot-4 開迴路積分 FF 誤差;峰值誤差恰 = r·(a(1.5)−a(1.2));且 RLS 沿用 gate 則 h̄<1.5 之 a′ 為純基底外插(誤差不受 in-band residual 約束) | C1 | structure | z 每 1% slope 誤差 → trough ~1% of local a(吃光 budget 於最被計分點);xy 外插 slope 誤差 50–67%,積分後 ~15%;y1 在 trough 對 slot-4 幾乎盲(F_dx→0) | CONFIRMED·高 | 推導 gate-off 傳播 e_a(t)=∫e_K dh_d + P(4,4) 決定性膨脹;寫 [1.2,1.5) 外插誤差模型;gate 決議引 T3-7 |
| C1-11 (=C2-09) | a_xm 帶 EWMA 窗心 lag L_eff=(1−a_cov)/a_cov=19 步(11.9 ms):C1 decoupled fit 配對 h̄[k−d] 錯 19 步(mis-pair 7.5–8.4% @h̄=1.5);C2 R22 亦未建模同項;奇部對稱軌跡相消(淨 +0.2–0.4%)但單向段(descent/ramp)一階 3–8% 不消 | both | regressor | 瞬時 7.5–12.3% @gate 邊界;對稱 osc 淨 0.2–0.4%;descent 加權 RLS(P₀=9 無 forgetting)可鎖入數 % θ bias;Jensen 偶項 ~0.35% | CONFIRMED·高 | regressor 改配 h̄[k−d−19] 或 de-blur port(帶 self-reference guard);odd/even 分解入文件;ramp_descent 場景必測;smoothing-off ablation 強制 |
| C1-10 (=STUDY-1) | 唯一的 C1 slope supplier 規格 = 探索腳本:no-forgetting scalar KF,R=(0.42a)²·τ=20 手設(0.42 恰合推導鏈但未綁定;τ=20 vs 真 autocorr 39 → Var(e_θ) 低估 ~2.2×);P→0 凍結 vs band-dependent θ* | C1 | parameter | 凍結後 band 改變成本:z +0.74% @1.5(~75% budget)、xy ~8%;現行 supplier noise floor 映射 a 誤差 2.4–4.6%(2–5× budget);θ CI 未校準 | CONFIRMED·高 | 寫 θ-RLS 正式規格:R_RLS=K_var·IF_eff·(â+ξ)²(綁 build_eq17_6state_constants)、forgetting λ_f↔Var(e_θ)_ss 閉式、per-axis、τ/smoothing ablation |
| G2 | x,y 基底 c=1+θ/h̄ 結構缺陷:1/(1+θu) 展開注入 θ*²=0.72 的 u² 項而真 D_para 之 u² 係數恆 0;misfit 集中近壁 | both | structure | a′ RMS op 23%/near-wall 57–60%/max −66% @1.2;level max 18.5%;Q44 近壁 mis-scale ~9× 低估;z 不受影響 | CONFIRMED·中 | 改 mobility-side:M1 a=a_nom(1−b₁u)(同 DOF,2.6× 改善)或 M2 加 −b₃u³(Faxen 結構,5.5×);b₃ 加 shrinkage prior(corr(u,u³)=0.973) |
| C2-04 (=G5+CF-2) | C2 一個 a_gain 同時當 y2 預測(延遲 h̄ 正確)與 a_ctrl(錯:晚 d 步 + 帶噪);三個 regressor 誤差(決定性 lag、Brownian gap、n_z jitter)無建模、無橋接、且 2.4% headline 用 TRUE h̄ 計分完全看不見 | C2 | regressor | 決定性 0.9–1.4%(實際 harness trough 1.11 更差)+ Brownian 0.8–2.6% + 白 0.6–1.2%;近壁 RSS 1.2–2.9%(超 budget);tracking 影響 <1 nm;n_z 三軸共用 | CONFIRMED·中 | 拆兩次求值:y2 留 h̄[k−d];a_ctrl 用 h̄_meas+(h̄_d[k]−h̄_d[k−d]) 橋接(一行,免費);殘餘白項進 slot-3 Q33 add-on;加 delivered-gain 指標 |
| G6 | a_nom=Ts/γ_N「已知」僅條件成立:far-field a_xm 可識別 a_nom 但 chi-sq floor 下 1% 需 47–54 s dwell;誤差 ε 不被吸收 — AR(1) 1:1 傳遞、C2 化為 tilt(−0.92ε@trough〜+0.88ε@far) | both | parameter | = ε 全域(不集中近壁);sim 內 0;部署 γ_N(η(T)、R)不確定性可能 ≥1% | CONFIRMED·中 | 文件化 a_nom<1% 需求;選配 far-field hold 段 RLS 校正(gscalar 機制重用,h̄>5) |
| C1-02 | e_K·Δh_d 是相關 ramp 非白噪:white-equivalent ≤6.4e-5 Q44(inflation 無效);真效應 = ramp-lag bias e_ss=r·a′·Δh_d/K44 | C1 | parameter | z r=1% → 0.23–0.33% of a(1 Hz 正弦拒斥後 ~0.23%);r≥3% 爆 budget;xy(r=14–22%)3–7% 結構性超標 | CONFIRMED·中 | 推導證 white 部可忽略;carrier = offline ramp-lag bias 表 e_ss(h̄) + 選配 Schmidt consider P_(x,θ)(不加 state);書面否決 Q44 inflation |
| C1-07 | Q44/R22 以 a′_hat² 定標:z 二階無害、xy 30–50% mis-sizing;且 theta-fit 訓練在 a_xm=(1+β)a 上 → a′_hat 帶恆存系統差 β−β′/K_h(RLS 資料量不可消) | C1 | parameter | z Q44 誤差 2%(K44 ~1%);xy P(4,4) 一致性差 30–50%;β-slope 系統差 3–8% of a′ 於 h̄ 1.5–2.5(gated fit 主訓練區) | CONFIRMED·中 | tex 換 a′→a′_hat + Var(e_K)=(da′/dθ)²Var(e_θ)+misfit²+(β-slope)² 映射;需 β(h̄) 模型或文件化 bound;xy 記 K44 後果 |
| C1-12 | Warm-up A-floor(a′_hat=0):有界但昂貴;字面 A-floor(Q44=0)+G3 gate 組合近壁 170–490%;FF-enable 時機 load-bearing(descent 恰落在 trough,θ-fit 零餘裕) | C1 | parameter | 有利配置 9–30%(y2 活 + Q44 保真階);字面配置 170–490%;far field 忽略;無發散機制(誤差方向 g<1) | CONFIRMED·中 | 推導 BIBO 論證 + FF-enable 條件 (da′/dθ)²P_RLS<(r_target·a′_hat)² 含 hysteresis + Q44 warm-up floor + G3 交互 |
| C1-13 | 任何 Δh_d²-scaled 新項若以近壁凍結絕對 Var(e_K) 寫,遠場過脹 38–65× Q44(121f86a 家族,open-loop 不可見);relative 形式自動消解(8.6e-4) | C1 | parameter | frozen 形式 @h̄=22:38–65× Q44 → est. a_hat std ~2.5×/K 增 6–8×;relative 形式 ~1e-3×;status doc §6 的候選項正是 frozen 預設 | CONFIRMED·中 | 強制規則:e_K 統計一律 relative-to-a′_hat(h̄) 形式 + h̄→∞ 極限檢查 + osc 遠場下降段閉迴路測試 |
| C2-05 | EKF 線性化 O(e_θ²) 被丟且 a() 對 θ 凸:P_θ0=9 使首更新遠出線性區((φ/c)²P_θ = 11%@8.9 / 341%@1.5 / 513%@1.2 若近壁首更新);測試場景恰好遠場開 y2 自癒 | C2 | parameter | 已驗場景 transient-only(~0.1 s 後 <0.2%);場景脆弱:近壁首更新 O(100%) θ kick + clamp 區 Jacobian 不一致;穩態 ~0.1% | CONFIRMED·中 | P_θ0 縮至 ~0.25 或首 N 更新 iterated-EKF 或估 log θ;tex 加有效性條件 (φ/c)²P_θ≪1 |
| CR-3 (=C1-05) | 兩 controller init 讀真 c(a_x_init、K_h_init、σ²_δh_init、DARE P 種);C2 θ_init 本身乾淨(Goldman/Brenner);C1 需 θ-prior init 規格且 naive 一致性映射 P(4,4)_init 落在 documented 邊緣不穩帶 | both | parameter | 精度面 far-field <0.4%(transient,prefill+hold 吸收);穩定面:naive P44 映射 ~2000× DARE seed,在 a-slot blow-up 帶內 | CONFIRMED·中 | 推導 init 節:θ 先驗、a_x_init=a_nom/c(h̄_init;θ₀)、P(4,4)_init=(da/dθ)²Var(θ₀) + 與 G1 freeze 交互;P 種需壓縮設計非直代 |
| CR-4 (=C1-14) | C1 推導文件不存在;完整 tex-delta 清單已列(11 個行段修改 + 8 個新節);且 base RW 之 known-c 上限 9.4% 使 companion 必走 AR(1) 分支 | C1 | other | n/a(交付物);含一項 budget-critical scoping 修正(RW→AR1) | CONFIRMED·中 | 寫 4state_del_hd_thetafit.tex 依 T1 骨架;base 檔凍結不動 |
| DRV-1 | para 無法經 run_pure_simulation:無 dispatch key、clear 清單缺、6 個 unguarded diag 讀取會 crash、5-arg 簽名不合 | C2 | other(infra) | 0 數值誤差;100% 阻斷 same-seed C1-vs-C2 A/B(standalone harness RNG ordering 不保證等價) | CONFIRMED·中 | 加 dispatch 分支 + clear 項 + isfield guard/diag adapter + 簽名適配(T5,需核准) |
| C1-06 | y2 已知輸入修正 c[k] 變估計:r₂ 增 −Σ e_K·Δh_d 相關項 + 與 q₄ 共享 e_K 的 Q-R cross(既有 Δa_ram cross 也從未言明) | C1 | measurement | ≤1.5e-6 of R22 近壁;far field 0;推導一次即除名 | CONFIRMED·低 | tex 寫 ĉ[k]=a(h̄_d[k];θ̂)−a(h̄_d[k−d];θ̂)(telescoped level 形式)+ e_K 項 bound + Q-R cross 假設明文化 |
| C2-08 | F/H 雙重使用(θ 同入 F_e(3,4) 與 H(2,4))= 標準 EKF,P(3,4)/P(1,4) 正確承載,無 selfdet 式 anchor 相消(negative 結論);殘餘:q₃-r₁ cross=(1−λc)σ_n²、shared-n_z Cov(+號) | both | measurement | cross = 0.3·R11(family-wide,已含於 0.4% 基線定價);shared-n_z ρ 0.5–1.4%;皆遠低 budget | CONFIRMED·低 | 兩 tex 各加一行陳述兩個 correlation 及其 bound |
| G7 (=C2-01+CF-4) | R22 delay-leak:tex p3(r₂=n_a only)自相矛盾於 p5/code(+d·Var(δa_inc));借自 base 的統計形式對 C2 輸出方程無推導基礎(para-correct 應為 window-jitter 統計,~1.5–1.8× 現值) | C2 | measurement | ≤0.16–0.5% of R22(且多在 gate 區內);兩種形式對 a_hat 差 <0.1%;純 bookkeeping 債 | CONFIRMED·低 | 裁決:刪 delay-leak(合 tex p3)或換 (a·K_h/R)²(C_dx·σ²_δh+σ_n²) 並推導;A/B 一 seed 驗無行為差 |
| C2-06 (=CF-3+CF-5) | Q33_randgain 為 para 模型明文消去之項(code 從 base 抄入,靠真 K_h)+ F_e(3,4) Jacobian 在 h̄[k−d] 求值而 tex 指定 h̄[k] | C2 | other | Q33 比 ≤1e-5〜7e-5(隱形);F_e epoch ≤0.7–1.3%,僅入 P 傳播(二階);皆遠場自滅 | CONFIRMED·低 | 刪 para 的 Q33_randgain(對齊 tex,除一個 K_h 讀取);F_e 之 dadth 用 h̄_d[k] 或 tex 加 Impl 註記 |
| C1-09 | 獨立性 audit 結論 benign:decoupled RLS 只讀 (h̄, a_xm, gate),演算法圖無環,無 selfdet 式相消;但 e_K 與 e_ax 經共享 n_a 流相關 → 變異數分析視為獨立會使 P 樂觀(二階) | C1 | control-coupling | 物理迴圈增益 0.41(收縮);P-optimism 未定量但 ≪budget;精度定價屬 β 通道 | CONFIRMED·低 | 推導節:forbidden-inputs 清單(a_hat、a_ctrl、EKF innovations、fitted-normalized deblur)+ acyclic 論證 + Cov bound + freeze-θ oracle ablation 規格 |
| DRV-2 | 圖優先測試計畫所需 diag 欄位缺:innovation_y1 兩 controller 皆未回傳(離線不可重建)、driver 未記 θ̂/P_θ/K_θ/var_da_ram/da_x_ff | both | other(infra) | 0 誤差;硬阻斷 whiteness/θ-consistency/carrier 圖 | CONFIRMED·低 | 兩 controller 加 innov_y1/S 欄;driver 加槽位(isfield-guarded)+ para dispatch(T5) |

**被推翻項:無(45/45 CONFIRMED)。** 惟以下子論點在覆核中被修正(不影響各 gap 結論):

- ~~C2-03 子項(3):x,y G2 隨機 dropout~~ — 原算術差 ~1260×;真實 thermal 超額 ~1100–1800×,G2 in-band 幾乎不觸發。
- ~~C2-02 子項:「P_θ<1e-5 於 ~1 s」~~ — 差 ~100×(實際 ~1e-3 @1 s;1e-5 需 100–150 s);凍結機制不變。
- ~~C1-12 引註:「9.4% = RW-no-FF」~~ — 錯置;9.4% 是 RW 含 FF 之數字(4state_del_hd.tex L274)。
- ~~C2-05 凸性 bias 為負 / C2-08 shared-n_z Cov 為負~~ — 兩者號誌皆為正(Jensen;雙重變號),量級不變。
- ~~C1-04:「Q33_randgain 達數 % of Q33」~~ — 實際 ~1e-5;結論(可忽略但須列舉)不變。

---

## 3. 理論任務 T1–T5

每個 gap 之歸屬列於各任務「收編」行;27/27 全覆蓋。

### T1 — C1 companion 推導文件(`4state_del_hd_thetafit.tex`,以 AR(1) 為底座)

**收編**:G1、INV-1(C1 側)、C1-03、C1-10、C1-02、C1-07、C1-12、C1-13、CR-3、CR-4、C1-06、C1-09。

**目標**:補上 C1 唯一缺的推導文件;scope 從「RW + slope-only 3 處」修正為「AR(1) + level+slope + 8+ 站」;每個 supplier 誤差配正確 carrier 並通過遠場 gate。

**輸入**:4state_del_hd.tex(L100-222 RW 主體、L225-259 AR(1) 節)、4state_del_hd_ar1.tex、status doc §2/§6、INV-1 清單、T2 凍結 basis、T3 的 Var(e_θ) 映射與 gate 決議、plot_para_aprime.m L48-61 現況。

**推導步驟(公式層級)**:
1. **架構裁決**:底座取 AR(1) — predict x₄⁺ = λc·x₄ + (1−λc)·â_det[k] + Δâ_x[k],其中 â_det[k]=a(h̄_d[k];θ̂)、a′_hat=∂a/∂h̄|θ̂ 由同一 fitted 曲線讀出(a′ 已知 ⊃ a_det 已知,scope 原否決理由失效);若堅持 RW-exact,文件明載 9.4% 為接受底線。[G1/CR-4]
2. **替換站清單(8 站)**:a_det(4state.m L675)、Q44 slope(L444)、R22 delay-leak(L448/L469)、σ²_δh=4kBT·a_fit_z(h̄_meas)(L327;xy 亦吃 z-shape,錯接自軸 → K44 +76%)、Q33_randgain(~1e-5,enumerate-and-dismiss)、init(步驟 8)、optional flags(deblur/ar1/cap/aprime_ff 逐一 fit-based 或禁用;use_deblur 禁用待步驟 9 分析)。[INV-1]
3. **誤差動力學**:e_ax[k+1] = λc·e_ax[k] + (1−λc)·e_alevel[k] + e_K[k]·Δh_d[k] + Δa_ram[k];e_alevel = a_true(h̄_d)−a(h̄_d;θ̂);e_K = (da′/dθ)·e_θ + slope-misfit + (β−β′/K_h)·a′。[G1, C1-07]
4. **Carrier 判定**:證 e_K·Δh_d 之 white-equivalent ≤6.4e-5·Q44(z)→ 書面否決 Q44 inflation;誠實 carrier = offline ramp-lag bias 表 e_ss(h̄)=r·a′_hat·Δh_d/K44(1 Hz 正弦拒斥係數 z ×0.80)+ 選配 Schmidt consider P_(x,θ);Q44 += (1−λc)²·Var(e_alevel);**強制規則**:任何 Δh_d²·Var(e_a′) 項一律 relative-to-a′_hat(h̄) 形式 + h̄→∞ 極限檢查 + 遠場下降段閉迴路測試(frozen 形式 38–65× 過脹)。[C1-02, C1-13]
5. **y2 修正**:ĉ[k] = a(h̄_d[k];θ̂) − a(h̄_d[k−d];θ̂)(telescoped level,模型內精確);r₂ += −Σᵢ e_K[k−i]Δh_d[k−i](bound ≤1.5e-6·R22);Q-R cross 假設(e_K 與既有 Δa_ram 兩者)明文化。[C1-06]
6. **Gate-off 傳播**:G3 期間(383 步/pass)e_a(t)=∫e_K dh_d 開迴路積分,P(4,4) 以決定性 bound 膨脹(Q44 不涵蓋);a′_hat 在 h̄∈[1.2,1.5) 外插誤差模型(z pole 基 1–3%、xy 50–67%);gate 去留引 T3-7 決議。[C1-03]
7. **θ-RLS 供應者規格**:regressor 對 (h̄[k−d−L_eff], de-blurred a_xm[k]),L_eff=19;R_RLS = K_var·IF_eff·(â+ξ)²(取代 (0.42a)²·τ=20,現值 Var(e_θ) 低估 ~2.2×);forgetting λ_f 或 Q_θ floor ↔ Var(e_θ)_ss 閉式;per-axis(z pole / xy T2 選型);Var(e_θ)→Var(e_K)、Var(e_alevel) 之 Jacobian 映射。[C1-10, C1-11]
8. **Init 節**:θ 先驗(Goldman 9/16、Brenner 9/8)、a_x_init=a_nom/c(h̄_init;θ₀)、P(4,4)_init=(da/dθ)²·Var(θ₀) — 落在 documented 邊緣不穩帶(~2000× DARE seed),需壓縮設計 + G1 freeze 交互;warm-up A-floor BIBO 論證 + FF-enable 條件 (da′/dθ)²·P_RLS < (r_target·a′_hat)²(含 hysteresis)+ Q44 warm-up floor(否則字面 A-floor+G3 組合近壁 170–490%)。[CR-3, C1-12]
9. **獨立性節**:forbidden-inputs 清單(a_hat_x、a_ctrl、EKF innovations、以 fitted 曲線 normalize 的 deblur);acyclic 圖論證 vs selfdet(3/3 發散前例);Cov(e_K,e_ax) 共享 n_a bound;freeze-θ oracle ablation 測試規格(同 seed、含 clear)。[C1-09]

**驗收**:compile 通過;每公式↔code 行對映表;L0 level+slope 數字入文;121f86a 檢查表(每個新項 h̄→∞ 極限)全過;user sign-off。

### T2 — 基底選型升級(x,y mobility-side;z level 補強)

**收編**:G2、G8(C2-02 之基底面向 cross-ref)。

**目標**:消除 xy u² 結構缺陷;level 計分納入選型;凍結 per-axis basis。

**輸入**:calc_correction_functions.m 真多項式;osc_1hz dwell 權重;G2/G8 fit 數據。

**推導步驟**:
1. xy 棄 c=1+θ/h̄(注入 θ*²=0.72 之 u² 項,真 D_para u² 係數=0):M1 a=a_nom(1−b₁u)(同 DOF,a′ 23%→9.6–13.5%)或 M2 a=a_nom(1−b₁u−b₃u³)(Faxen 結構,op 4%/near 8.6–11%,level RMS 0.4–0.5%);b₃ 帶 shrinkage prior(corr(u,u³)=0.973 於 42% a_xm 噪聲下);線性 in 參數 → 純 RLS、H 條件數常數。
2. z 保 pole 基(a′ 1.4–1.9%)但 level 決策:1-param floor −4.2%(gated −4.65%、任何權重 ≥−2.7%)vs 2-param c=1+θ₁/(h̄−1)+θ₂/h̄(collinearity 0.936,正則化)vs 接受並交由 β-corrected a_xm anchor 拉回(連動近壁 R22/K sizing)。
3. 每候選 basis 產 level(逐點 @h̄_d)+ slope 雙表 × 四權重(uniform/dwell/gated≥1.5/info);θ*(band) 敏感度表(band shift 成本 z +0.74%、xy ~8%)。

**驗收**:L0 表完成;xy 選型 a′RMS ≤10% 且 level ≤2%;z level@trough 決策記錄(=G0 gate);θ*(band) 漂移 <0.5%(z)。

### T3 — 四通道 budget 形式化 + carriers + 跨架構文件債

**收編**:G3、C1-08、CF-1、C2-02、G6、C2-05、C2-08、G7、C2-06、INV-1(C2 側)。

**目標**:把 §1 frame 寫成正式文件:每通道 bias/variance 分流、carrier 指定、閉迴路放大、計分規格;一次清完 C2 tex/code 債。

**輸入**:全部 verified 數值;pm_to_axm_derivation.tex;4state_para_c.tex;build_eq17_6state_constants.m。

**推導步驟**:
1. **β(h̄) 推導**:E[a_xm] 對 EWMA 權重二階 Taylor → bias = 0.5·(|K_h|/R)·ḧ·E[j²]·Ts²,E[j²]=(1−a_cov)(2−a_cov)/a_cov²=741;full kernel(Σg²=3.161=C_dpmr)→ trough +6.9%(佔實測 +11% 的 ~63%);奇部(相位鎖 lag ±13–20%)零均值列於窗心節(T4);殘餘 ~4pp 列未決(EIV/g-shift);carrier = y2 反演之 h̄-dependent de-bias,**不進 R22**。[G3]
2. **Fixed-point 節**:λ_eff = 1−g(1−λc)(精確),m(g)=C_dpmr(λ_eff)/C_dpmr(λc),dm/dg|₁=−0.408,放大 1/(1−|dm/dg|)=1.69(production 常數 1.38–1.52);收縮證明;budget 規則:y2 靜態 bias ×1.4–1.7 後入表;β(h̄>1.5) 實測需求成文。[C1-08]
3. **a_nom 節**:far-field 可識別性(a_xm 鏈無 a_nom;c_⊥(22)=1.053);chi-sq floor 44–47%、τ_int=39 步 → 1% 需 47–54 s dwell;AR(1) 1:1 pass-through 與 C2 tilt 公式(dθ=+2.56ε;殘餘 −0.92ε〜+0.88ε);規格:a_nom 已知 <1% 或遠場校正段(gscalar 機制)。[G6]
4. **C2 凍結決策節**:θ*(h̄) 之 h̄ 依賴性 + Q_θ=0 → info-weighted 凍結(P_θ 塌縮時標修正:~1e-3 @1 s);選項 A「θ 常數 + 宣告外插誤差包絡(z −6.3%、xy +38% @trough)」vs 選項 B「Q_θ>0(棄常數前提:q₄ 出現、e_θ cyclostationary、閉迴路 variance 重推、任何 Δh_d/ḣ-scaled Q_θ 過 121f86a gate)」;計分規格:全訪問 h̄ 範圍 + trough-bias 列。[C2-02]
5. **線性化有效性**:條款 (φ/c)²·P_θ ≪ 1;P_θ0 9→~0.25 或首更新 iterated-EKF 或 log-θ 參數化;近壁首更新 313–513% bias 風險紀錄(tex 若除 gate 即失守)。[C2-05]
6. **殘餘相關**:E[q₃r₁]=(1−λc)σ_n²=0.3·R11(family-wide,已定價於 0.4% 基線);shared-n_z Cov(e_y1,e_y2)=+(da/dh̄)σ_nz²/R(ρ 0.5–1.4%);各一行入兩 tex。[C2-08]
7. **Gate 政策決議**:裁定 2.4% 為 gated 產出(code 為真相;tex L214 更正);兩支路成本表 — keep G3:trough 純外插(誤差不受 in-band residual 約束)+ headline re-caption;drop G3:β 暴露 ×1.69 → 最高 +16–20%;先決條件 = 步驟 1 之 β(h̄) 量測。C1 RLS gate 繼承決議。[CF-1]
8. **C2 文件債落地**:r₂ 統計裁決(刪 delay-leak 或換 (a·K_h/R)²(C_dx·σ²_δh+σ_n²),差 <0.1% R22,一 seed A/B 驗);刪 Q33_randgain;F_e(3,4) epoch 註記或橋;真 c 讀取換 model-implied K_h=−θφ²/c、a_⊥=a(h̄;θ_z)(零成本,拔掉 headline 的星號);nocheat status 之 out-of-scope caveat 移植入 4state_para_c.tex 與 code header。[G7, C2-06, INV-1 C2 側]

**驗收**:budget 表(4 通道 × bias/variance × 放大係數 × carrier)per arch 填滿,每項有公式+數值+遠場行為;C2 tex 修訂 diff 清單;gate 決議文件化並 re-caption 2.4%。

### T4 — Regressor/時序橋接項

**收編**:C2-04、C1-11(+C2-09);C2-06 之 F_e epoch 橋為選項。

**目標**:三個可免費修的決定性時序誤差(d-step、EWMA 窗心、epoch)橋掉;殘餘隨機部分配 carrier。

**輸入**:h̄_d[k] 全序列(scope 允許);T3 之 β/de-blur;a_cov。

**推導步驟**:
1. **C2 a_ctrl 橋**:a_ctrl = a(h̄_meas + (h̄_d[k]−h̄_d[k−d]); θ̂)(殺 0.9–1.4% 決定性 lag);F_e 之 dadth 同步;y2 預測維持 h̄[k−d]。評估直接用 h̄_d[k] 的反噬:K_h·σ_track/R ≈ 2.6–3.8% @trough(tracking 誤差 11–14 nm ≫ σ_n)→ 決議採 h̄_meas+Δh̄_d 而非 h̄_d 直代。[C2-04]
2. **殘餘白項 carrier**:O(a′²·(2·Var_inc+σ_nz²/R²)) 指派到 slot-3 之 F_dx·e_ax 耦合(Q33 add-on;f_d²-scaled → 過 121f86a 遠場檢查),非 R22。[C2-04]
3. **EWMA 窗心配對**:RLS regressor 改 h̄[k−d−19] 或 de-blur port(以 a(h̄_d[k−d];θ̂) normalize;self-reference guard:scale 在 κ 比中相消,仍須 selfdet 式獨立性檢查);odd/even 分解:odd 對稱相消(淨 +0.2–0.4%)但單向段 3–8% 一階不消 → ramp_descent 必測;Jensen 偶項 bound 0.5·|K_h²−K_h′|·Var_w ≈ 0.35%。[C1-11/C2-09]
4. **計分修正**:meas-h̄(或 delivered |a_ctrl−a_true|)指標與 clean 指標並列。[C2-04]

**驗收**:橋後 L2 通道圖中 stale-lag 曲線 <0.2%;de-blur A/B + smoothing-off ablation 通過;ramp_descent 單向 bias 縮 >3×;所有新 Q 項遠場圖乾淨。

### T5 — 工具鏈與診斷(非理論;先於 L2;code 變更需 user 核准,不動 control path)

**收編**:DRV-1、DRV-2。

**目標**:C1/C2 可經同一 driver same-seed A/B;產出 whiteness/consistency 圖。

**步驟**:(1) run_pure_simulation 加 '4state_para' dispatch + clear 清單項 + 5-arg 簽名適配 + 6 個 unguarded diag 讀取加 isfield/adapter;(2) 兩 controller 加 diag.innovation_y1/S_y1/S_y2;driver 加 θ̂/P_θ/K_kf_theta_y2/var_da_ram/da_x_ff 槽(isfield-guarded);(3) C1 供應者 hook(ctrl_const.aprime_supplier:a′_fit + Var(e_a′) per axis per step,最小表面);(4) 一次性 RNG-ordering 等價檢查(standalone harness vs driver,known-c 配置)。

**驗收**:para 經 driver 3 seeds 無 crash;diag 欄位齊;RNG 檢查通過或記錄差異。

---

## 4. 驗證矩陣 L0–L3(gap→圖 對映)

目標線:z |a_hat−a|/a ≈ 1%(known-c 基線 0.4% / 24 nm)。輸出 `test_results/audit_c1c2/L{0..3}/` + 每層 notes.md。L0/L1 無需新 code 可先行;L2/L3 blocked on T1–T5。

### L0 — 基底 oracle(無模擬,純解析)
| 圖 | 抓的 gap |
|---|---|
| fig_L0_a_level_vs_hbar_(axis)(a_model/a_true−1 逐點;含 gated≥1.5 外插曲線) | **G1、G8、G2(level)、C1-03(外插)** — level 計分首度成為一級指標 |
| fig_L0_aprime_ratio_vs_hbar_(axis) | G2(slope 23–66%)、C1-07(r 值) |
| fig_L0_weighting_shift(θ* × 4 權重 + band 敏感度) | C1-10(band-dependent θ*)、G8(gated −4.65%) |
| fig_L0_beta_pred(T3-1 理論 β(h̄) 曲線 vs +11% 錨點) | G3(推導驗證前置) |

**Gate G0**:per-axis basis 凍結(xy M1/M2、z 1-vs-2-param level 決策);level+slope floor 表 vs 1% budget 記錄。z 1-param level floor −4.2% 單獨超標 → 必須做出 (a) 2-param、(b) budget 放寬、(c) 接受並交由 anchor 拉回 三選一。

### L1 — 開迴路 θ 估測(於 known-c 閉迴路 log 上離線跑;ablation 免費)
| 圖 | 抓的 gap |
|---|---|
| fig_L1_theta_conv(θ̂(t) 每 seed + θ* 線 + β-shifted θ* 線;gate-off 陰影) | C1-10(凍結/P 崩)、G3(β 吸收 = 兩線間落點)、C2-05(早期線性區) |
| fig_L1_afit_overlay(a(h̄;θ̂) vs a_true vs a_xm 散點) | G3(β 可見)、G8(level 形狀) |
| fig_L1_ablation_smoothing(**強制**;raw vs smoothed corr) | C1-10;EWMA lesson(翻號即作廢) |
| fig_L1_ablation_beta(de-blur on/off → θ̂ 軌跡) | G3(on 落 θ*、off 落 shifted 線 = T3-1 驗證) |
| fig_L1_pairing_ablation(配 h̄[k−d] vs h̄[k−d−19] vs de-blurred;descent-only vs full-cycle 分開) | **C1-11/C2-09**(窗心 lag;單向段 bias) |
| fig_L1_forgetting_sweep(λ_f/τ ∈ {0.25,0.5,1,2 s}) | C1-10(bias-variance 曲線,預期無最優點) |
| fig_L1_killtest_D(視窗局部回歸 slope vs a′_true) | ladder rung 5 前置;預期遠場失效 → 證成全域 basis |

**Gate G1**:parameter 通道 ≤ 所選 basis 之 structure floor;level@trough 貢獻 <0.5%;smoothing-off corr 同號且 2× 內;θ̂ spread 與 R_RLS 後驗一致。

### L2 — 閉迴路單 seed 深潛(seed 1,osc_1hz;先 C1 後 C2)
每 run 前 `clear <controller> trajectory_generator calc_thermal_force`。
| 圖 | 抓的 gap |
|---|---|
| fig_L2_(arch)_gain_overlay(a_hat/a_true/a_xm + h̄ 底 tile) | G1(trough level 轉移)、C1-03(gate-off 漂移)、C1-12(warm-up) |
| fig_L2_(arch)_eax_channels(e_ax/a 疊加各通道理論曲線:L0 structure 帶、stale-lag 曲線、FF ramp e_K·ΣΔh_d、窗心 lag) | **預算指派圖**:C2-04、C1-02、C1-11、G1 — 殘差跟哪條曲線走即歸哪通道 |
| fig_L2_(arch)_theta_P(θ̂ + P_θ/P_a;3 guards 分色陰影) | C2-02(P 塌縮 ~1e-3@1 s 修正值)、C1-10、C2-05((φ/c)²P_θ 軌跡 vs 1) |
| fig_L2_(arch)_innov(e_y1/e_y2 正規化 + lag 1–20 autocorr;需 T5) | G7(r₂ 統計)、C2-08(whiteness/cross)、C1-06;記錄預期 MA(2) 色 |
| fig_L2_C1_q44_farfield(**強制**;每個 Δh_d²-scaled 項 vs Q44 時序,下降段) | **C1-13**(121f86a):任一項遠場 >10× 近壁設計值 → STOP |
| fig_L2_C1_independence(freeze-θ oracle A/B + cross-corr(a_xm 殘差, e_ax) lag 0–50) | C1-09 |
| fig_L2_C2_actrl_delivered(|a_ctrl−a_true|/a @true h̄) | C2-04(0.9–2.9% delivered 誤差;破解 true-h̄ 計分盲區) |
| fig_L2_(arch)_tracking(per-axis,24 nm 線) | 總體(目標 z ≤~30 nm) |

**Gate G2**:0 發散;通道疊加解釋 e_ax 至 ±30%;遠場圖乾淨;innovations 如模型;任何 >0.3% 未解釋殘差必須指派(新 carrier 或 control-coupling)才進 L3。

### L3 — Ensemble + ladder + sweeps
Ladder rungs:1) known-c AR(1) 基線(重現 0.4%/24 nm)2) C1(theta-fit)3) C2(integrated)4) A-floor(5state free-RW;註:其 Q 仍讀 K_h,floor 偏樂觀)5) D kill-test(離線視窗回歸當 supplier 計分)。
Grid:seeds 1:6(最終 spread 主張 1:20)× {h_bottom 2.7/4.5/6.0 × freq 0.5/1/2 Hz}@λc=0.7;λc {0.5,0.7,0.8} 僅 nominal;**加 ramp_descent 場景**(C1-11 單向段)與 **G3-on/off 一 seed A/B**(CF-1 決議驗證)。
| 圖 | 抓的 gap |
|---|---|
| fig_L3_ladder_bias / _spread(bar per rung per axis;0.4% 與 1% 線;**加 trough-h̄ 列與 level-bias 列**) | G1、C2-02/CF-1(遮蔽破解)、G3(β 修正前後) |
| fig_L3_ladder_tracking(24 nm 線) | 總體 |
| fig_L3_spaghetti_(sweep)(a_hat_z(t) 全 seed) | first-run 異常(persistent/RNG lesson) |
| fig_L3_stability_matrix(條件 × 架構發散熱圖) | C1-12(FF-enable 時機)、C1-13(遠場)、CF-1(gate A/B)、C1-11(ramp_descent) |

**Gate G3(最終)**:z |bias|+spread 於 h_bottom sweep 全程 ≤2.5× known-c、nominal ≤1%;0/N 發散;C1-vs-C2 裁決表以實測填入四通道 budget(bias/variance × 放大係數)。

**執行約束**:MATLAB 僅 user 機/MCP 主執行緒(agent 只做文字/圖規格);每 run 前 clear(不信 first-run-only 結果);temp_*.m 用完即刪,晉升腳本命名 verify_audit_L{0..3}_*.m;圖風格 True 紅/Est 藍/Meas 淡藍、FS18 bold、無 grid/title、legend northoutside、exportgraphics 150 dpi、Visible off;任何 corr/RMS 分數必附 smoothing-off ablation;T5 code 變更先取得明確核准(CLAUDE.md 規則 1)。

---

## 5. 未決事項

1. ~~**C1 scope 裁決(G1)**~~ **已裁決(2026-07-06):兩者都推** — T1 以 AR(1)+theta-fit level 為主文、RW-exact 為對照分支(同一 tex 內雙分支),L2/L3 ladder 各加一階(RW-supplier 與 AR1-supplier),以實測數據做最終裁決。
2. ~~**z level 決策(G0)**~~ **已裁決(2026-07-06,使用者授權 Claude 拍板)**:z = **Z2 + 雙先驗**(θ₁+θ₂ ~ N(9/8, soft) Brenner 遠場通用物理約束,恰好釘死 collinear 病態方向;in-band 訓練紀律)。理由:G1 level 1:1 傳遞 ⇒ basis level floor = â_x bias floor;Z1 最好 −2.67% 永久超標,Z2 是唯一 level<1% 選項;sum-prior 後有效噪聲 ≈ 1-param 等級(VIF 8→~1)。Fallback:Z1 rel-LS 留 ladder,kill criterion = L1 噪聲下 Z2 trough(bias+std)>2.7%。
3. ~~**Gate 政策(CF-1/T3-7)**~~ **已裁決(同上):分階段**。Phase 1 維持 G3(EKF y2 + supplier 訓練皆 gated;z 已知代價 −1.8% trough 外插);Phase 2 條件 = T3-1 β 推導 + L1 `fig_L1_ablation_beta` 實測後,殘餘 β×1.7 < 外插代價(≈殘餘 β<1% @trough)才開近壁。理由:β 未量測前 keep 在所有情境損失較小(drop 最壞 +16–20%);L3 保留 G3-on/off A/B 終審。2.4% headline 即刻 re-caption。
4. **β 殘餘 ~4pp 未推導**(G3):EIV pairing(窗平均 a_xm 配瞬時 trough h̄)與 g=a/â pole-shift 兩候選未測。
5. **a_nom 部署規格(G6)**:sim 內為 0,部署 γ_N 不確定性可能 ≥1%;<1% 已知 vs 47–54 s 遠場校正段,擇一成文。
6. **C2 Q_θ=0 vs >0(C2-02)**:引入 forgetting 即棄 θ-常數前提,閉迴路 variance 全套重推 + 121f86a gating 成本未估。
7. **de-blur port 自參照(G3/C1-11)**:以 a(h̄;θ̂) normalize 是弱自參照(κ 比中 scale 相消),仍須 selfdet 式獨立性檢查後才可啟用。
8. ~~**x,y 是否納入 1% 目標**~~ **已裁決(同上)**:xy = **X0 退役;M2 + 先驗**(b₁ ~ N(9/16, soft) Faxén 通用物理;b₃ shrinkage→0;in-band 訓練)。xy 驗收線:level in-band ≤1.5%、trough ≤2.5%、**近壁 â_x 總預算 ≤5%**(z 維持 1%;近壁 slope ~11–16% 為結構 floor,入 budget 不硬闖)。Fallback:M1 留 ladder。另採納 L0 metric note:band 統計一律 dwell 加權。
9. **RNG-ordering 等價**(DRV-1):standalone harness vs driver 同 seed 軌跡等價性一次性檢查(known-c 配置)。
10. **2.4% headline re-caption 措辭**(CF-1/INV-1):gated、band-limited、Q/R 仍 known-c-assisted(K_h/σ²_δh)三個限定詞如何入文。
---

## 6. 跨 session 增補(2026-07-07,平行 nocheat 線 g-mismatch 發現)

平行 session(另一機,`4state_del_hd_ar1.tex` nocheat 線)以 5 層驗證(介入實驗/Step-1 g 推導 corr=1.0000/6-state Lyapunov 0.1-0.5%/v1 均值代理/v2 隨機代理斷崖精確重現)定量鎖定 **g-mismatch anchor loop**,對本審計的修正:

- **C1-09 增補**:「演算法圖 acyclic ⇒ benign」**不充分**。a_xm 反解隱含 g:=a_true/â=1,迴路經物理閉合(â→f_d→δx→σ̂²_δxr→a_xm→anchor→predict),增益 ∝ anchor 每步增益 α。a_m_det 案例實測 α 斷崖 = 0.008(χ² 漲落噪聲觸發逃逸,seed-dependent),bias +12% 中 ~8pt 為迴路平衡點偏移(開迴路僅 1.6-3.7%)——「資訊量地板」假說推翻。
- **C1-08 定位**:審計的 fixed-point(λ_eff=1−g(1−λc), dm/dg=−0.41)= 該迴路的均值部分,正確但不完整;噪聲觸發逃逸是新增機制。
- **T1 S6/S8 對應更新**:S6 必須推導 θ-RLS 的等效迴路增益 + 斷崖裕度判準(forgetting 設計約束);S8 的獨立性論證從「acyclic」升級為「物理迴路增益分析」;基底剛性+遠場先驗作為 scale-absorption 阻斷器(G6 tilt 機制)需定量。
- **新資源**:p_md(dx_bar_m)含 (1−g)·D[k] 一階矩 = 帶符號 g 觀測通道(平行線發現,候選 de-confound,非 MVP)。
- **T5 協調警告**:平行線在 `run_pure_simulation.m` 有未 commit 的 TEMP dispatch(selfdet/aprime_ff/nocheat/adet_only 等 + true_gain_scale);本線 T5 動同一檔案前必須先收斂。
- **L3 新增 rung(S6 產出,2026-07-07)**:λ_f cliff-transfer 迷你掃描 {0.99, 0.999}——S6 Ruling 4 預測 0.99 逃逸/0.999 安全(α_eq=(1−λ_f)|P_tilt| vs 斷崖 0.008/M),可證偽,L2/L3 裁決。|P_tilt|≈0.9(弱抑制 ~10%,trough 翻號 −0.92 為穩定化負回饋);真正安全機制=RLS 記憶。
