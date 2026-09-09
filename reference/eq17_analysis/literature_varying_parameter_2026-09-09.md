# 文獻調查：隨位置／環境變的參數怎麼追（2026-09-09）

> 來源：research-assistant agent（perplexity 查證），應使用者要求「查別人遇到這種情境怎麼處理」。
> 問題：formC_b 的 b 是牆的屬性（曲率、邊界條件），掛在空間上；估測器把它當 run 的常數（b[k+1] = b[k]，Q₅₅ = 0），
> 學慢（資訊 ∝ (1−ā)²Δw̄，只在近壁運動）、學到後 P₅₅ 塌、換牆追不上。均勻遺忘與 Q₅₅ > 0 都試過（付散布、不追）。
> 標「推論」者為 agent 的判斷非引文；末段列未查證項。

一句話結論：文獻裡沒有人線上估「牆的形狀／軟硬參數」；最貼近的三個工具是 (i) directional forgetting（只在有激勵的方向忘）、
(ii) KF + CUSUM 變化偵測 + 協方差重置（Gustafsson 的路面摩擦估測，問題結構同構）、(iii) 把 b 當地圖（per-wall 常數向量、家族 prior 給交叉協方差、永不忘）。

## Q1 時變／切換參數的遞迴估測

- Uniform exponential forgetting：Ljung & Söderström 1983（MIT Press）；Ljung & Gunnarsson 1990（Automatica 26:7–21）給出 MSE = 追蹤誤差 + 雜訊誤差的閉式 trade-off。失效模式 = 無激勵時 covariance windup。hold 段對 b 零資訊，「加 forgetting 變 noisier」正是 windup 的教科書症狀，不代表 forgetting 這一族不適用（推論）。
- Variable forgetting factor：Fortescue, Kershenbaum & Ydstie 1981（Automatica 17:831–835），每步選 λ 使資訊量純量守恆；只緩解 windup，仍各向同性。
- Directional／restricted forgetting（真正「只沿激勵方向忘」的一族）：Kulhavý & Kárný 1984（IFAC World Congress）；Kulhavý 1987（Automatica 23:345–346）；Cao & Schwartz 2000（Automatica 36:1725–1731）；Parkum, Poulsen & Holst 1992（IJC 55:109–128，selective forgetting）；收斂性 Bittanti, Bolzern & Campi 1990（Automatica 26:929–932）。核心式（Cao–Schwartz，R = P⁻¹，φ = 回歸向量）：
  R₂ = R φ φᵀ R / (φᵀ R φ)，R₁ = R − R₂（R₁φ = 0）；R⁺ = R₁ + λ R₂ + φ φᵀ。
  協方差形式（推論）：P⁻ = P + ((1−λ)/λ)·φφᵀ/(φᵀP⁻¹φ)，只沿 φ 膨脹。對純量 b：H_b ≠ 0 時 P_bb ← P_bb/λ，H_b = 0（hold）時不動。
- 現代版：Goel, Bruce & Bernstein 2020（IEEE Control Systems Magazine 40(4):80–102，variable-direction forgetting）；Lai & Bernstein 2022（IEEE L-CSS 7:985–990，零激勵時 P 收斂到設計矩陣而非爆掉）。
- 變化偵測 + 重置：Gustafsson 1997（Automatica 33:1087–1099）用小 Q 的 KF 估 slip-slope，並列 CUSUM 看 innovation，偵測到路面突變就放大 P；Gustafsson 2000《Adaptive Filtering and Change Detection》（Wiley）。摩擦問題與本問題同構：參數是環境性質、資訊只在有驅動力時到、換環境時突變。
- IMM：Blom & Bar-Shalom 1988（IEEE TAC 33:780–783）；適合 b 屬離散集合 {8/9, 1.16, …}。

## Q2 隨位置／工作點變的參數

- LPV 辨識：Bamieh & Giarré 2002（IJRNC 12:841–853）θ(w) = Σ θ_j w^j 化成對常數 θ_j 的線性回歸 + RLS；PE 條件 = 排程變數要走過足夠範圍（對應 Fisher ∝ (1−ā)²Δw̄）；多項式基底在未訪區外插失控。專書 Tóth 2010（Springer LNCIS 403）。
- Terrain 參數：Iagnemma, Kang, Shibly & Dubowsky 2004（IEEE T-RO 20:921–927），線上 LS；換地形靠近期資料重解，不是 map。
- GP 場地圖（map、no forgetting）：Vallivaara et al. 2010（IEEE MFI，磁場 SLAM）；Solin, Kok, Wahlström, Schön & Särkkä 2018（IEEE T-RO 34:1112–1127）；Kok & Solin 2018（FUSION）。基底 Solin & Särkkä 2020（Stat. Comput. 30:419–446）：f(w) = Σ θ_j φ_j(w)，θ_j ~ N(0, S(λ_j))，θ 當常數 state 放進 KF；未訪區 = 先驗均值 + kernel 先驗變異；再進入直接用已學的 θ；場隨時間變才加時間核（Särkkä, Solin & Hartikainen 2013）。Online sparse GP：Csató & Opper 2002；Huber 2014。

## Q3 近壁流體力學

- 離線校正（法則已知、估高度）：Schäffer, Nørrelykke & Howard 2007（Langmuir 23:3654–3665）；Leach et al. 2009（PRE 79:026301）；Faucheux & Libchaber 1994（PRE 49:5158）；Bevan & Prieve 2000（JCP 113:1228）。全是離線曲線擬合。
- 線上估「值」不估「形狀」：Huang, Cheng & Menq 2010（IEEE/ASME T-Mech）KF 聯合估力與 trapping bandwidth（含 γ），bandwidth 為 random walk，Faxén 只用來說明 γ 為何變，無突變機制、無牆參數。Meng, Long & Menq 2025（IEEE TIE 72(1):928–937）：7-state EKF 估增益值、Q(6,6) = Q(7,7) = 0，Fig 11 三面是量測，未估牆參數。
- 有效牆位（支持「細胞 = 平面下移」）：Lecoq et al. 2004（JFM 513:247–264，波紋牆 ≡ 平滑有效牆）；Kunert, Harting & Vinogradova 2010（PRL 105:016001，隨機粗糙 ≡ no-slip 平面落在峰谷之間）。
- 軟牆：Leroy & Charlaix 2011（JFM 674:389–407）；Leroy et al. 2012（PRL 108:264501）；Zhang et al. 2022（PR Applied 17:064045）皆離線；Daddi-Moussa-Ider & Gekle 2018（EPJ E）：彈性膜旁 mobility 頻率相關，穩態極限回硬牆。推論：細胞的 b 隨速度與停留時間變，是「b 常數」在細胞牆上的失效條件。
- 空間相依 mobility 的 spurious drift：Volpe et al. 2010（PRL 104:170602）；Lançon et al. 2001（EPL 54:28）。∂D/∂z 對應 pred_mean2 的曲率均值項（推論）。
- Kim 群：查不到線上估牆參數的論文；微機器人文獻只把近壁 drag 當 disturbance 補償。

## Q4 建議（三個最小實作）

1. Directional forgetting 只對 b：predict 時 Q_bb[k] = ((1−λ)/λ)·P_bb[k]·g[k]，g = I_b[k]/Ī（I_b = H_b²/R₂ 每步 Fisher 資訊）或 g = 𝟙[H_b ≠ 0]。hold 時 Q_bb = 0 不 windup。λ 由「一段牆的行程步數 N」定（λ^N = 1/e）。失效：牆突變仍要 N 步；同一牆內多付約 √(1/λ−1) 散布。
2. KF + CUSUM + reset（Gustafsson 1997）：b 維持 Q = 0；只在近壁運動段（H_b ≠ 0）累積雙邊 CUSUM，統計量 = 正規化 innovation ν/√S（y₂ 為主）；觸發時 P_bb ← 家族 prior（0.15²）、P_ws ← P_ws[0]，b̂ 不動讓資料拉。失效：誤警報 = 白放大散布；資訊少時偵測延遲，仍受 (1−ā)²Δw̄ 限制。
3. Per-wall map（GP／LPV 精神）：state 換成向量 b_r（每面牆一格），P₀ = 家族 prior 對角 + 共享均值 8/9 的交叉相關；H 只對當前 region 的欄位非零；不忘、再進入直接用、未訪牆用 prior。失效：分區錯 = 混牆；region 數隨場景長。
建議組合 2 + 3：CUSUM 判「換牆了」，觸發開新 region 而非重置舊值。IMM 只在 b 確定屬離散集合時才划算。

## 參考文獻

Bamieh & Giarré 2002 IJRNC 12:841–853 · Bevan & Prieve 2000 JCP 113:1228 · Bittanti, Bolzern & Campi 1990 Automatica 26:929 · Blom & Bar-Shalom 1988 IEEE TAC 33:780 · Cao & Schwartz 2000 Automatica 36:1725 · Csató & Opper 2002 Neural Comput. 14:641 · Daddi-Moussa-Ider & Gekle 2018 EPJ E 41:19 · Faucheux & Libchaber 1994 PRE 49:5158 · Fortescue, Kershenbaum & Ydstie 1981 Automatica 17:831 · Goel, Bruce & Bernstein 2020 IEEE CSM 40(4):80 · Gustafsson 1997 Automatica 33:1087 · Gustafsson 2000 Adaptive Filtering and Change Detection (Wiley) · Huang, Cheng & Menq 2010 IEEE/ASME T-Mech (PMC3875182) · Huber 2014 PRL 45:85 · Iagnemma et al. 2004 IEEE T-RO 20:921 · Kok & Solin 2018 FUSION · Kulhavý & Kárný 1984 IFAC · Kulhavý 1987 Automatica 23:345 · Kunert, Harting & Vinogradova 2010 PRL 105:016001 · Lai & Bernstein 2022 IEEE L-CSS 7:985 · Lançon et al. 2001 EPL 54:28 · Leach et al. 2009 PRE 79:026301 · Lecoq et al. 2004 JFM 513:247 · Leroy & Charlaix 2011 JFM 674:389 · Leroy et al. 2012 PRL 108:264501 · Ljung & Söderström 1983 MIT Press · Ljung & Gunnarsson 1990 Automatica 26:7 · Meng, Long & Menq 2025 IEEE TIE 72(1):928 · Parkum, Poulsen & Holst 1992 IJC 55:109 · Särkkä, Solin & Hartikainen 2013 IEEE SPM 30(4):51 · Schäffer, Nørrelykke & Howard 2007 Langmuir 23:3654 · Solin et al. 2018 IEEE T-RO 34:1112 · Solin & Särkkä 2020 Stat. Comput. 30:419 · Tóth 2010 Springer LNCIS 403 · Vallivaara et al. 2010 IEEE MFI · Volpe et al. 2010 PRL 104:170602 · Zhang et al. 2022 PR Applied 17:064045.

未查證或信心較低：Schäffer 2007 頁碼與「表面位置為擬合參數」一句；Daddi-Moussa-Ider 2018 卷期；Huang–Cheng–Menq 2010 卷期（僅由 PMC 頁面確認）。Kim 群相關論文未找到。
