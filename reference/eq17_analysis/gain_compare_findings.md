# gain_compare 研究結果 — 現役資料集

Date: 2026-06-14（整理）
Branch: `test/motion-test`（未 merge 回 feat/eq17-6state）
實驗：6-state 控制器在「控制律餵真值 gain（a=a_true）vs 餵 EKF 估測 gain（a=â）」兩臂下，沿同一激進軌跡比較追蹤表現，拆成 det（系統性）/ ram（隨機）兩分量。
腳本：`compare_gain_6state.m`（跑批）+ `analyze_gain_6state.m`（分析/出圖）。

---

## 0. 現有資料集（磁碟上唯一保留的）

`test_results/gain_compare/{f0.5Hz, f1Hz, f2Hz}/`，每夾 = runs.mat + summary.md + 6 圖 + analysis.mat。

| 頻率 | seeds | 軌跡時間（hold / descend / osc / tail）| a=â 存活 |
|---|---|---|---|
| 0.5 Hz | 100 | 0.25 / 1.5 / 2.0(1 cyc) / 0.25 | 100/100 |
| 1 Hz | 200 | 0.5 / 1.0 / 2.0(2 cyc) / 0.5 | 100/100 |
| 2 Hz | 100 | 0.25 / 1.5 / 2.0(4 cyc) / 0.25 | 100/100 |

共通：gate-free（h̄_safe=1，G3 永不觸發）、h_init 50 / h_bottom 2.7 μm（h̄ 1.2↔3.42）、suppress_xD 兩臂、CRN 配對、熱力+量測噪聲全開。
（1 Hz 時間結構與 0.5/2 略不同——對 osc 窗分析無影響，osc 皆 2.0 s。）

---

## 1. 控制律本身 ≈ 完美（a=a_true 臂）

a=a_true（控制律餵真值 gain）的 det 誤差 e_det = p_d − p_true：z 軸 descent peak **0.1–0.2 nm**、osc A_e < 1 nm（三頻率）。完美 gain + Eq.17 延遲補償下，追蹤本身近乎零系統性誤差 → **控制律不是瓶頸**。

## 2. 估測代價（a=â 臂，系統性 det）

a=â（餵 EKF 估測）的 det 誤差（ensemble-det 參考）：z 軸 descent peak **~43–65 nm** + osc 週期波形。這是 â 在下降末端與近壁谷底跟不上 gain 驟變所致——估測誤差是端到端追蹤誤差的主要系統性來源。

## 3. 熱力理論驗證（V1/thermal closed form）

σ²_th,i(t) = C_δx·4k_BT·a_pd,i(t) + C_n·σ²_n,i 沿期望軌跡。a=a_true 臂：
- x 軸 var meas/theory：**0.99–1.00**（全窗全頻率）
- z 軸 normalized var：**~0.99**（osc）
→ 熱力基線吻合、資料品質乾淨可引用。
a=â 臂 z normalized var 超出 1（估測引入的超額擾動）：**1.05 / 1.04 / 1.44 @ 0.5 / 1 / 2 Hz** → 估測隨機代價隨頻率上升（2 Hz 已明顯，趨向高頻失穩）。

## 4. ⭐ Q55 閉式驗證（A1，現役主線）

Q55 = gain 隨機漲落的「一步增量」變異數，閉式 **Var(δa_ram) = [2/(1+λc)]·(a·K_h/R)²·σ²_δh**。
驗證法：a=a_true 的 100/200-seed `a_true_out` 堆疊 → 扣 ensemble 均值得 per-seed a_ram → 沿時間一步差分 → 跨 seed 取變異數 → 對閉式（gap-safe，§A1）。

實測 increment meas/theory（z 軸）：

| 頻率 | osc | near（近壁）|
|---|---|---|
| 0.5 Hz | 1.011 ± 0.005 | ~1.02 |
| 1 Hz | 1.014 ± 0.003 | ~1.02 |
| 2 Hz | 1.018 ± 0.004 | ~1.02 |

→ **Q55 閉式準到 ~1-2%**，h̄ 低至 1.2、慢軌 quasi-static 最強。對照 LF-drift audit 舊 random-walk 模型近壁高估 LF 變異 237× → Q55 閉式取代 heuristic 的數據基礎完備。

## 5. 穩定性（慢軌 gate-free）

a=â 存活率：0.5 Hz、1 Hz、2 Hz 皆 **100/100**（2 Hz 連 det run 都不發散）。失穩邊界在 2 Hz 以上（見 §7 歷史註記）。a=a_true 三頻率永遠全活。

---

## 6. Q55 / σ²_δh / x_ram 推導與認知（2026-06-15 深入討論，已收斂）

### 6.1 從 x_ram 出發的推導鏈（不經 Q66_OL）
- **δh 不是外來量，是 x_ram 每步的驅動**：plant `x[k+1]=x[k]+a_x(f_dx+f_T)` 每步注入熱位移 `w[k]=a_x·f_T[k]`，`Var(w)=a_x²·(4kBT·γ/Δt)=4kBT·a_x=σ²_δh`（白）。δh ≡ w。
- **x_ram = 閉迴路累積 δh**：`δx[k+1]=λc·δx[k]−w[k]` → `x_ram=Σ λc^j·w[k−1−j]`。
- **Level**：`Var(x_ram)=σ²_δh/(1−λc²)`（no-delay）；含 d=2 延遲（ε 變 MA(2)）→ `C_δx=2+1/(1−λc²)=3.96`（λc=0.7）。
- **自相關**：ρ(1)=λc（no-delay）/ 0.8515（d=2）。
- **Increment = Q55**：`Var(δx_ram)=2(1−ρ)·Var(x_ram)=[2/(1+λc)]·σ²_δh`，**no-delay 與 d=2 都代數 exact**（level 撐大被 (1−ρ) 變強精確抵消；增量免疫延遲，只認 λc 與 σ²_δh）。故 `Q55=Var(δa_ram)=(a·K_h/R)²·[2/(1+λc)]·σ²_δh`。
- **[2/(1+λc)] 已打包兩步**：`Var(δa_ram)=Var(a_ram[k+1])+Var(a_ram[k])−2Cov`；兩個 level 相加(=2·3.96G)被協方差(−2·0.85·3.96G)砍成 1.18G=[2/(1+λc)]G。**不可再乘 2**（那是被駁回的 i.i.d. 2× 陷阱）。

### 6.2 σ²_δh 的 a 取在哪 — 三方對照（det 操作點）
σ²_δh = 4kBT·a，其 a 是**操作點(det 位置)的 mobility**（= Vpersonal p.2 `a_x=a_x_det+a_x_ram` 的 a_x_det）：

| 場合 | 用的 a | 沿哪條 h̄ | 備註 |
|---|---|---|---|
| 推導本意 | a(h̄_det) | det 操作點 | a_x_ram 修正二階可忽略 |
| 控制器即時 | a_⊥(h̄_**measured**) | 量測 p_m | 不用 â；避 bias loop（code L280/291/298）|
| 驗證理論 | a_pd=a(h̄_**desired**) | 參考軌跡 | no privilege；control 近完美 → ≈h̄_det |
| 量測側 | a_true_out | 真值位置 | 只進「量測」，不進理論 |

doc 用詞「**deterministic function** evaluated at h̄_zm」= 函數 a(·) 確定性、在量測 h̄ 取值。

### 6.3 per-axis 結構（KF 實作已確認正確）
所有軸 gain 都只依賴 h̄（垂直壁距）→ 所有 gain 漲落都由**同一個 z(垂直壁)位置漲落**驅動：
- **σ²_δh = 4kBT·a_⊥ 三軸共用**（wall-normal kick；code L298「shared 3 axes」）。
- **K_h per-axis**：x,y=K_h_para、z=K_h_perp（code L292）。**â per-axis**（code L382）。
- `Q55_i = [2/(1+λc)]·(â_i·K_h_i/R)²·σ²_δh`。x 軸混 â_x(平行)+σ²_δh(垂直)，正確。
- **雷**：不可把 x 的 σ²_δh 寫成 4kBT·a_para（平行壁踢不改 h̄、不改 a_x）。

### 6.4 視覺化 + fuzz
- **新圖**（test_results，gitignored）：`fig_q55_scatter`（var(δa_ram) vs a，x/z，log-log，量測點+jackknife SEM 貼理論線）+ `fig_q55_time`（時域逐點 紅量測/綠理論，雙峰=osc 谷底近壁）。已出 f1Hz(200)/f0.5Hz(100)，f2 待。生成 recipe 在 job tmp（q55_extract/q55_plot_one）：抽 a_true_out stack→扣 ensemble mean→diff→cross-seed var；理論沿 a_pd/Kh_pd；osc 窗 h̄-binned。**尚未整進 analyzer**（待 style review）。
- **fuzz ∝ level**：pointwise var 的取樣散佈 = level·sqrt(2/(Ns−1))（相對定值），近壁 level 高→毛邊寬、遠壁→看不見。0.5Hz(Ns=100,~14%) 比 1Hz(Ns=200,~10%) 更毛，是 seed 數差異非物理。
- a=a_true 臂驗（pole 嚴格 λc）；a=â 臂 self-consistent bias，僅診斷。

### 6.5 相關驗證腳本（已 staged，2026-06-15）
- `verify_axm_cdpmr_6state.m`（擴充）：原 C_dpmr（Var dx_r）+ **C_dx（Var δx = x_ram Level）** + **R22（Var(a_xm)，見 §6.6）** 三重驗證，輸出 `test_results/axm_verify/<freq>`，a-binned scatter + jackknife。
- `plot_var_ahat_6state.m`（新）：Var(a_hat) 估測 bias+spread band（估測代價視覺化，§6 原 TODO 的 â 版）。

### 6.6 R22 閉式驗證（A2，2026-06-17，已成立）
R22 = a_xm 量測噪聲變異數（EKF R(2,2)）。**原始閉式** `Var(a_xm) = Var(σ̂²_δxr)/(C_dpmr·4kBT)²`，核心 `Var(σ̂²_δxr) = K_var·IF_eff·(σ²_δxr)²`（EWMA 增益 × Isserlis 2σ⁴ × 有色膨脹），代入 σ²_δxr 恆等式 →
```
R2_intrinsic = K_var · IF_eff(a,σ²_n) · (a + ξ)²
  K_var = 2·a_cov/(2−a_cov) = 0.05128
  ξ     = (C_n/C_dpmr)·σ²_n/(4kBT) = [7.88e-6, 6.66e-6, 2.24e-4]（z sensor floor 主導）
  IF_eff 逐步 exact（IF_abc=[12.2, 0.0649, 0.102] 有色噪聲膨脹）
  常數全從 stored ctrl_const（C_dpmr=3.161 full a_pd form, build_eq17_6state_constants）
```
- **本質 = variance OF a variance（四階矩）**，比 C_dpmr 一階矩難驗。
- **delay 項**：6-state R2_eff = R2_intrinsic + 過去 d 步 var_da_ram（=Q55）之和（`var_da_ram_km1+var_da_ram_km2`，**非** 7-state 的 5·Q77）；它是 a_xm 量延遲 gain 的 effective-R 修正，**不在 raw Var(a_xm)** → 只驗 R2_intrinsic，delay 靠已驗 Q55 組合。
- **方法**：a=a_true 臂跨 seed `var(axm_loc,0,3)`（a_true[k−d] common-mode 自動消）= R2_intrinsic；理論沿 a_pd。

實測 meas/theory（a=a_true,ratio [x y z]）：

| 頻率 | desc | osc | near | far |
|---|---|---|---|---|
| 1 Hz (200) | 1.03/0.99/1.02 | 0.99/1.00/0.98 | 1.00/1.01/1.00 | 0.99/1.00/0.98 |
| 0.5 Hz (100) | 1.02/1.02/1.00 | 0.98/1.02/0.99 | 0.97/0.90/0.98 | 0.98/1.04/0.99 |
| 2 Hz (100) | 1.02/1.02/1.00 | 1.00/1.00/0.96 | 0.97/0.97/1.06 | 1.00/1.00/0.96 |

→ **三頻率閉式整軌含近壁都對得上**（f1 ±3%、f0.5/f2 ±5%，最鬆 f0.5 near-y 0.902 = 近壁+100-seed 取樣毛邊，預期）。新圖 `fig_axm_var_time`+`fig_axm_var_scatter`；三張 scatter x 軸已統一對齊（純小數，x/z 不同 a range = 壁面各向異性 c_⊥≈5.9≫c_∥≈2.0 正確體現）。

## 7. 歷史註記（資料已刪，僅存結論）

整理前曾跑 {1, 5, 10} Hz 標準軌（hold 0.5/descend 1.0）100-seed gate-free 批，結論（資料已刪、可由 seed+code 重現）：
- **穩定性斷崖**：1 Hz 全活、5/10 Hz 全滅（|e| 3-4 μm 跟丟、無撞牆）→ G3 gate 低頻是累贅、高頻是剛需。
- **近壁 â 高估**：gated +42% → gate-free +20% → G3 蒙蔽佔近壁高估一半，殘餘為估測器近壁極限。
- 對應 design/plan 文件（gain_compare_design.md / _plan_round2.md）描述的是此標準軌 scenario，屬歷史設計記錄；現役慢軌（0.5/2Hz）為其 ad-hoc 延伸，無獨立 design。
