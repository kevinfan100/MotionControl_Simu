# test/law-error-budget — 法則形狀誤差的 P44 預算（2026-08-27 起）

## 每次開 MATLAB 第一件事（path 遮蔽防護）
```matlab
restoredefaultpath; rehash toolboxcache;
addpath(genpath('/Users/kevin/Code/MotionControl_Simu-law-error-budget/model'));
addpath(genpath('/Users/kevin/Code/MotionControl_Simu-law-error-budget/test_script'));
assert(contains(which('motion_control_law_formC_b'), 'law-error-budget'), 'WRONG WORKTREE ON PATH');
```
四個 worktree 有同名函數，path 上先到先贏、不報錯。`which` 不印本目錄就全部作廢。

## 起點
`08f9997`（= test/motion-test 同 commit）。四個 default-off 診斷旗標、注入響應／sweep／δb 腳本都已在。
負控制 fixture：canonical deep seed 7，`a_bar_hat_z[end] = 0.107505`（旗標關閉時逐位）。

## 要做的事（路線 A：常數 Δb 的累積容器）
缺的一行：`formC_state_b.tex` 誤差動態第 4 列 `+ δb(ā)·(1−ā)²·Δw̄_d`，δb = b_true − b̂ 沿路徑變化部分。
容器 A：Δb = b_half（包絡 sup，= √P55[0]），沿路徑累積 `σ_law += Δb·(1−ā)²·|Δw̄_d|`，P44 += 其平方（確定性有界誤差寫法）。hold 自動為零。

| 階段 | 出口 |
|---|---|
| 0 tex（第 4 列加項、雜訊節加容器、`_ref.tex` 散文）＋獨立驗算 | 驗算過；R₂ 的 d·Q44 延遲項是否納入已決定並寫明 |
| 1 default-off 旗標接線 | fixture 逐位、`scenario='shallow'` 逐位、五支 driver 都跑 |
| 2 驗收（跑前登記） | 注入響應：穿越段 K₁ ≥ 0、r 衰減、無路徑記憶；canonical sweep：偏差 ≤ +8%、**總 RMS/√P44 ∈ [0.8,1.2]**、sd < 12%；a_cov ×2/÷2 不變；疊加態 LOO |
| 3 | default-on 決定、mainline 文件、merge 回 test/motion-test |

預期（登記）：偏差降幅接近 ×3 臂（→ ~+7%）；總帳 ~0.5（A 是上界、高估 2–4×）；散布升。
若總帳 < 0.5 且偏差不再降 ⇒ 進路線 B（帶換號形狀）。

## 已知數字（08-27，`check_delta_b_curve.m`）
Σe 降落段 +0.0085（谷底 9.7%）、振盪整週期為零、hold 為零；相關因子 12.7／59.6；
容器 σ：白 0.0016、常數 Δb 積分 0.0382、內插 0.0525；需要 0.0085–0.020。
